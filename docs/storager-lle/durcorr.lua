-- durcorr.lua - what does the failing cmd 0x95's DURATION track?
--
-- Observed: 2.7s, 12.2s and 18.3s for the same command - a 6.8x spread.  That rules out every
-- fixed-duration mechanism (a second deferred period, a seek/settle timeout from the UIB, a poll with
-- its own counter): all of those would repeat to within a few percent.  A variable duration means an
-- outer loop with a variable iteration count, or a wait on a genuinely asynchronous condition.
--
-- So at each failing read's GO, capture the candidates and correlate against the elapsed time:
--   [$79aa]  retry counter (incremented $9328, clamped to 15 at $1A6A) -> retry loop
--   phase    arm time mod one revolution (200ms measured)              -> positional wait
--   [$7abc]  sectors requested                                         -> per-sector cost
-- tracks [$79aa] -> retry loop, and the per-attempt cost becomes the question
-- tracks phase   -> the read is waiting for something positional
-- tracks neither -> the wait is on host or drive state, outside the firmware
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/durcorr.lua -flop siemens/set1/mx2-001.imd -hard1 <img>

local SC, STOP = ":slot1:storager:cpu", 60.0
local REV = 0.200          -- one revolution: INDEX measured at 200.0ms
local NSEC = 16            -- sectors per track (FM)

local cpu, sp, armed = nil, nil, false
local rows, cur = {}, nil
local taps = {}
local lastcmd, laststat = -1, -1

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function B(a) return sp:read_u8(a & 0xffff) end
local function W(a) return sp:read_u16(a & 0xffff) end

local function arm_taps()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  -- capture activity, attributed to whichever 0x95 is live: is a 0x80 a REAL completion?
  for _, base in ipairs({ 0x00c800, 0xffc800 }) do
    taps[#taps+1] = sp:install_write_tap(base, base + 0x1ff, "c800", function()
      if cur then cur.c800 = cur.c800 + 1 end
    end)
  end
  for _, base in ipairs({ 0x00e000, 0xffe000 }) do
    taps[#taps+1] = sp:install_write_tap(base, base + 0x1f, "e000", function(off)
      if cur and (off - base) >= 2 then cur.load = cur.load + 1 end
    end)
  end
  taps[#taps+1] = sp:install_write_tap(0x7dac, 0x7db1, "mark", function()
    if cur then cur.marks = cur.marks + 1 end
  end)
  print("durcorr armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm_taps() then return end
  local node = W(0x71bc)
  if node < 0x4000 or node >= 0x8000 then node = 0x71f0 end
  local cmd, stat = B(node), B(node + 2)

  if cmd ~= lastcmd then
    if cmd == 0x95 then
      local t = now()
      local ph = (t % REV) / REV
      cur = { t0 = t, retry = W(0x79aa), count = W(0x7abc), phase = ph,
              sector = math.floor(ph * NSEC) % NSEC, c800 = 0, marks = 0, load = 0 }
    else cur = nil end
    lastcmd = cmd
  end
  if cur and stat ~= laststat then
    if stat == 0x82 or stat == 0x80 then
      cur.t1 = now(); cur.ok = (stat == 0x80)
      cur.retry_end = W(0x79aa)      -- the counter AT THE END: a retry loop shows up here, not at GO
      cur.count_end = W(0x7abc)      -- and [$7abc] is only loaded by op58, well after GO
      rows[#rows+1] = cur; cur = nil
    end
  end
  laststat = stat

  if now() >= STOP then
    print("=== what does the duration track? ===")
    print("   #  result   elapsed   [79aa]  [7abc]GO->END   C800   marks   E000ld   verdict")
    for i, r in ipairs(rows) do
      local real = (r.c800 or 0) > 0 or (r.marks or 0) > 0
      print(string.format("  %2d  %-7s %8.3fs   %d->%d    %3d -> %3d    %5d  %5d   %5d   %s",
        i, r.ok and "0x80" or "FAIL", (r.t1 or now()) - r.t0, r.retry, r.retry_end or -1,
        r.count, r.count_end or -1, r.c800 or 0, r.marks or 0, r.load or 0,
        r.ok and (real and "real completion" or "*** 0x80 WITH NO CAPTURE - FALSE COMPLETION ***")
              or (real and "failed after capturing" or "failed with no capture")))
    end
    -- correlate over the FAILING ones only
    local f = {}
    for _, r in ipairs(rows) do if not r.ok then f[#f+1] = r end end
    print(string.format("=== %d failing instance(s) ===", #f))
    if #f >= 2 then
      local function spread(get)
        local lo, hi = math.huge, -math.huge
        for _, r in ipairs(f) do local v = get(r); lo = math.min(lo, v); hi = math.max(hi, v) end
        return lo, hi
      end
      local dlo, dhi = spread(function(r) return (r.t1 or now()) - r.t0 end)
      local rlo, rhi = spread(function(r) return r.retry end)
      local clo, chi = spread(function(r) return r.count end)
      local plo, phi = spread(function(r) return r.phase end)
      print(string.format("  duration %.3f-%.3f  |  [79aa] %d-%d  |  [7abc] %d-%d  |  phase %.3f-%.3f",
        dlo, dhi, rlo, rhi, clo, chi, plo, phi))
      if rlo == rhi then print("  [79aa] CONSTANT across failures -> duration does not track the retry counter") end
      if clo == chi then print("  [7abc] CONSTANT across failures -> duration does not track the sector count") end
    end
    io.flush(); manager.machine:exit()
  end
end)
