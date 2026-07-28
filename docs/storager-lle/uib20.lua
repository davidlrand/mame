-- uib20.lua - who writes UIB+$20 ($6E80), the template selector word?
--
-- The UIB control block the gate array DMAs in is 0x20 bytes ($6E60-$6E7F).  UIB+$20 = $6E80 is one
-- word PAST it.  If the firmware writes it, it is firmware state.  If nothing writes it during the
-- command, it is stale/uninitialised - and the model may be truncating a longer host-supplied UIB.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--          -autoboot_script docs/storager-lle/uib20.lua -flop siemens/set1/mx2-001.imd

local SC, STOP = ":slot1:storager:cpu", 30.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev = {}, {}

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function B(a) return sp:read_u8(a & 0xffff) end
local function PC() local ok,v = pcall(function() return cpu.state["PC"].value end); return (ok and v or 0) & 0xffffff end
local function E(s) if #ev < 200 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp
  -- the selector word itself, and the last word INSIDE the 0x20-byte block for comparison
  taps[#taps+1] = sp:install_write_tap(0x6e80, 0x6e81, "sel", function(off, data, mask)
    E(string.format("$6E80 (UIB+20) <= %04x mask=%04x pc=%06x  bit14=%d",
      data & 0xffff, mask, PC(), (data >> 14) & 1))
  end)
  taps[#taps+1] = sp:install_write_tap(0x6e7e, 0x6e7f, "last", function(off, data, mask)
    E(string.format("$6E7E (UIB+1E, last word IN block) <= %04x pc=%06x", data & 0xffff, PC()))
  end)
  taps[#taps+1] = osp:install_read_tap(0xa362, 0xa363, "sel2", function()
    E(string.format("   -> op54 reads UIB+20 = %04x (bit14=%d)", W(0x6e80), (W(0x6e80) >> 14) & 1))
  end)
  print("uib20 armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  if now() >= STOP then
    print("=== UIB+$20 writers ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== final: $6E7E=%04x  $6E80=%04x", W(0x6e7e), W(0x6e80)))
    io.flush(); manager.machine:exit()
  end
end)
