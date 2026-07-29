-- pitprog.lua - what the firmware actually programs into the two 8253s.
--
-- The settle (60 units) and op42's guard (70 units) both run off the PIT1 ctr0 system tick.  The
-- model clocks it at 10MHz/4 = 2.5MHz and the measured period is 26.1ms, implying a divisor of
-- ~65280 (0xFF00).  If the FIRMWARE actually programs a much smaller count, the model's assumed
-- 0xFF00 is wrong and both delays are ~26x too long - which would explain the slow boot.
--
-- The PITs are at $8000-$8007 with .mirror(0xff0000); firmware short-absolute lands at $FF800x.
-- pit[0] is the HIGH byte lane (umask 0xff00), pit[1] the LOW byte lane (umask 0x00ff).
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle \
--          -autoboot_script docs/storager-lle/pitprog.lua -flop siemens/set1/mx2-001.imd

local SC   = ":slot1:storager:cpu"
local STOP = 30.0

local cpu, sp, armed = nil, nil, false
local taps, ev = {}, {}

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function PC() local ok,v = pcall(function() return cpu.state["PC"].value end); return (ok and v or 0) & 0xffffff end

local CTR = { [0]="ctr0", [2]="ctr1", [4]="ctr2", [6]="CONTROL" }

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  -- BOTH bases: the boot init uses short-absolute ($8007.w -> $FF8007) but op28/op42 use LONG
  -- absolute ($8007.l -> $00008007).  Tapping only the mirror misses every ctr2 arming.
  local function mk(base)
   return function(off, data, mask)
    -- focus on the command window; the boot-time init is already characterised
    -- focus on the command window, and drop the ctr0 tick reloads at $2BCA (every 26ms) which
    -- otherwise fill the buffer before anything interesting happens.
    if now() < 6.0 or #ev >= 120 then return end
    if (PC() & 0xffffff) == 0x2bca then return end
    local reg = (off & 0x6)
    local lane, val
    if mask == 0xff00 then lane = "pit0(hi)"; val = (data >> 8) & 0xff
    elseif mask == 0x00ff then lane = "pit1(lo)"; val = data & 0xff
    else lane = "BOTH"; val = data & 0xffff end
    local note = ""
    if reg == 6 then
      local sel, rw, mode = (val >> 6) & 3, (val >> 4) & 3, (val >> 1) & 7
      note = string.format("select ctr%d rw=%d mode=%d", sel, rw, mode)
    end
    ev[#ev+1] = string.format("%9.4f  [%06x] %s %-7s <= %02x   pc=%06x  %s",
      now(), base, lane, CTR[reg] or "?", val, PC(), note)
   end
  end
  taps[#taps+1] = sp:install_write_tap(0x008000, 0x008007, "pitlo", mk(0x8000))
  taps[#taps+1] = sp:install_write_tap(0xff8000, 0xff8007, "pithi", mk(0xff8000))
  print("pitprog armed"); io.flush()
  return true
end

emu.register_periodic(function()
  local t = now()
  if not arm() then return end
  if t >= STOP then
    print("=== PIT programming (pit1 ctr0 = the system tick) ===")
    for _, l in ipairs(ev) do print(l) end
    print("=== note: two consecutive ctr writes = LSB then MSB of the divisor ===")
    io.flush(); manager.machine:exit()
  end
end)
