-- posclobber.lua - who overwrites the sector POSPTR cell?
--
-- A1 = $6E60 (the UIB) at $7D04, so the pointer LOAD runs correctly - $7D02 falls through and nothing
-- in the ROM branches past it.  But A0 comes back $71F0 (the node base) on every record after the
-- first, so UIB+$CE ($6F2E) held $71F0 at that instant.  Read later from the periodic it holds
-- $7DAF, the correct sector offset.  The cell is being clobbered and restored.
-- With A0 = the node base, $7D08 move.b (A0),D0 reads node+0 = the COMMAND BYTE ($95), a constant,
-- so $7E58's compare is vacuous, $7E0A re-stamps the constant into [$7428], every sector is accepted
-- by arrival, and the label lands 0x300 into the host buffer.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/posclobber.lua -flop siemens/set1/mx2-001.imd

local SC, STOP = ":slot1:storager:cpu", 14.0
local cpu, sp, armed = nil, nil, false
local taps, ev = {}, {}
local n = 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function PC() local ok,v = pcall(function() return cpu.state["PC"].value end); return (ok and v or 0) & 0xffffff end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  -- UIB+$C8..$CE - the four position pointers
  taps[#taps+1] = sp:install_write_tap(0x6f28, 0x6f2f, "pos", function(off, data, mask)
    n = n + 1
    if now() > 1.0 and #ev < 40 then
      local a = off & 0xffff
      local name = ({ [0x6f28]="X(+c8)", [0x6f2a]="C(+ca)", [0x6f2c]="H(+cc)", [0x6f2e]="S(+ce)" })[a] or "?"
      ev[#ev+1] = string.format("%9.4f  UIB%s [%04x] <= %04x  pc=%06x%s",
        now(), name, a, data & 0xffff, PC(),
        ((data & 0xffff) == 0x71f0) and "   *** the NODE BASE - clobber ***" or
        ((data & 0xffff) == 0x7daf) and "   (correct sector offset)" or "")
    end
  end)
  print("posclobber armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  if now() >= STOP then
    print("=== writes to the position pointers ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== %d total ===", n))
    io.flush(); manager.machine:exit()
  end
end)
