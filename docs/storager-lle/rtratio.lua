-- rtratio.lua - is the ~18.3s cmd 0x95 failure an outer retry loop over the 1.305s $7B1C watchdog?
--
-- THE TEST IS A RATIO, NOT A TALLY: does (arms x 1.305s) account for the elapsed time?  A bare count
-- answers nothing at 2 or 3, where the watchdog fired but is not setting the pace.
--
-- Everything is measured WITHIN ONE RUN and correlated per command - the failing read's own start to
-- its own error stamp.  Never across runs: the arm time is RTC-seeded and moves seconds, and applying
-- one run's window to another is how the previous attempt at this went wrong.
--
-- Node status byte: node+2 (the firmware's $1A54 move.b #$80,($2,A0) done stamp / $82 on error).
-- Command byte: node+0.  Both read as BYTES so they come through the swapped-SRAM mapping correctly.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/rtratio.lua -flop siemens/set1/mx2-001.imd -hard1 <img>

local SC, STOP = ":slot1:storager:cpu", 40.0
local PERIOD = 1.305        -- 50 ticks x 26.1ms, the $29F8 arm the three sites use

local cpu, sp, armed = nil, nil, false
local taps, rows = {}, {}
local cur = nil
local lastcmd, laststat = -1, -1

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function B(a) return sp:read_u8(a & 0xffff) end
local function W(a) return sp:read_u16(a & 0xffff) end

local function arm_taps()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  -- every watchdog arm; attribute it to whichever command is live right now
  taps[#taps+1] = sp:install_write_tap(0x7986, 0x7987, "wd", function(off, data, mask)
    if not cur then return end
    if (data & 0xffff) ~= 0 then cur.arms = cur.arms + 1
    else                         cur.cancels = cur.cancels + 1 end
  end)
  print("rtratio armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm_taps() then return end
  local node = W(0x71bc)
  if node < 0x4000 or node >= 0x8000 then node = 0x71f0 end
  local cmd  = B(node)
  local stat = B(node + 2)

  if cmd ~= lastcmd then
    if cmd == 0x95 then cur = { t0 = now(), arms = 0, cancels = 0 }
    else cur = nil end
    lastcmd = cmd
  end
  -- an error stamp closes the current command
  if cur and stat ~= laststat and stat == 0x82 then
    cur.t1 = now(); cur.stat = stat
    rows[#rows+1] = cur
    cur = nil
  end
  laststat = stat

  if now() >= STOP then
    print("=== retry-loop ratio test ===")
    if #rows == 0 then
      print("  no cmd 0x95 reached an error stamp in this run - test not applicable to this seed")
    end
    for i, r in ipairs(rows) do
      local elapsed = r.t1 - r.t0
      local acct = r.arms * PERIOD
      local frac = elapsed > 0 and (acct / elapsed) or 0
      print(string.format("  failing read #%d: elapsed %.3fs, arms=%d, cancels=%d", i, elapsed, r.arms, r.cancels))
      print(string.format("     arms x %.3fs = %.3fs  =  %.0f%% of elapsed  ->  %s",
        PERIOD, acct, frac * 100,
        (frac >= 0.8 and frac <= 1.25) and "ACCOUNTS FOR IT - outer retry loop over the watchdog"
          or "DOES NOT ACCOUNT - distinct mechanism"))
    end
    io.flush(); manager.machine:exit()
  end
end)
