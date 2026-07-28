-- f000poll.lua - DATA-access instrument (no prefetch exposure).
--
-- Four opcode taps this session turned out to be prefetch artifacts, so stop inferring control flow
-- from instruction fetches.  $A118 and $A1CE both poll F000 in tight dbra loops (up to 65520/65536
-- iterations).  Such a loop CANNOT hide from a read counter on F000.  Bucket F000 reads by time and
-- report the bursts: a burst of ~65k is an exhausted poll; a few hundred is normal status polling.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/f000poll.lua -flop siemens/set1/mx2-001.imd -hard1 <img>

local SC, STOP = ":slot1:storager:cpu", 30.0
local cpu, sp, armed = nil, nil, false
local taps = {}
local buckets = {}     -- [0.1s bucket] = count
local nf000, ne802 = 0, 0
local e802_clklow = 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  for _, base in ipairs({ 0x00f000, 0xfff000 }) do
    taps[#taps+1] = sp:install_read_tap(base, base + 1, "f000r", function()
      nf000 = nf000 + 1
      local b = math.floor(now() * 10)
      buckets[b] = (buckets[b] or 0) + 1
    end)
  end
  for _, base in ipairs({ 0x00e802, 0xffe802 }) do
    taps[#taps+1] = sp:install_write_tap(base, base + 1, "e802w", function(off, data, mask)
      ne802 = ne802 + 1
      if (data & 1) == 0 then e802_clklow = e802_clklow + 1 end
    end)
  end
  print("f000poll armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  if now() >= STOP then
    print("=== F000 read bursts (0.1s buckets, >2000 reads only) ===")
    local ks = {}
    for k in pairs(buckets) do ks[#ks+1] = k end
    table.sort(ks)
    for _, k in ipairs(ks) do
      if buckets[k] > 2000 then
        print(string.format("  t=%5.1f-%5.1f : %7d F000 reads%s", k/10, (k+1)/10, buckets[k],
          (buckets[k] > 40000) and "   *** EXHAUSTED POLL LOOP ***" or ""))
      end
    end
    print(string.format("=== F000 reads total: %d", nf000))
    print(string.format("=== E802 writes: %d, of which clock-LOW (bit0=0): %d", ne802, e802_clklow))
    print("=== (clock-low > 0 would mean a serial transaction really clocked)")
    io.flush(); manager.machine:exit()
  end
end)
