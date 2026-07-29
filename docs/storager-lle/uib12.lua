-- uib12.lua - who writes UIB+$12, the density flag?
--
-- No ROM site can set bit1: the only +$12 writers are $2406 (move.w D0 with D0=0 from $23F2), $3AC4
-- and $3D10 (both move.w #$1), and $3570 (ori.b #$4).  Yet UIB+$12 bit1 is measured SET at cyl 69/83
-- and CLEAR at cyl 0/1.  So the value arrives some other way - the UIB is a control block the gate
-- array DMAs (D000 latch, E800 bit13 clear, 0x20 bytes), so it may be HOST-supplied.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/uib12.lua -flop siemens/set1/mx2-001.imd

local SC, STOP = ":slot1:storager:cpu", 20.0
local cpu, sp, armed = nil, nil, false
local taps, ev = {}, {}
local uib = 0x6e60

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function PC() local ok,v = pcall(function() return cpu.state["PC"].value end); return (ok and v or 0) & 0xffffff end
local function E(s) if #ev < 60 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  -- the whole UIB, so a block DMA is distinguishable from a targeted store
  taps[#taps+1] = sp:install_write_tap(0x6e60, 0x6e81, "uib", function(off, data, mask)
    local a = off & 0xffff
    if a == 0x6e72 then
      E(string.format("UIB+12 [%04x] <= %04x mask=%04x  bit1=%d  pc=%06x",
        a, data & 0xffff, mask, (data >> 1) & 1, PC()))
    end
  end)
  print("uib12 armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  if now() >= STOP then
    print("=== writers of UIB+$12 ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== %d write(s); final UIB+12 = %04x", #ev, W(0x6e72)))
    io.flush(); manager.machine:exit()
  end
end)
