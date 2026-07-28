-- esdisel.lua - what distinguishes an ESDI unit select from a floppy one at E804/E802?
--
-- F000 bit1 must become the ESDI transfer-acknowledge for the serial class WITHOUT breaking the
-- floppy seek, which uses the same bit as seek/settle busy.  node+4 = 0x0100 -> [$7a0a] -> bit8 is
-- the selector $A118/$A690 use.  Capture the E804 select and [$7a0a] around both a floppy seek and
-- the 0x98 serial transaction, so the responder can be gated on a measured value.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/esdisel.lua -flop siemens/set1/mx2-001.imd -hard1 <img>

local SC, STOP = ":slot1:storager:cpu", 30.0
local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev = {}, {}
local ne802 = 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function B(a) return sp:read_u8(a & 0xffff) end
local function E(s) if #ev < 160 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp
  for _, base in ipairs({ 0x00e804, 0xffe804 }) do
    taps[#taps+1] = sp:install_write_tap(base, base + 1, "e804", function(off, data, mask)
      E(string.format("E804 <= %04x  cmd=%02x [7a0a]=%04x node+4=%04x  (head=%x drive=%02x)",
        data & 0xffff, B(0x71f0), W(0x7a0a), W(W(0x71bc) + 4),
        (~(data >> 8)) & 0x0f, data & 0xff))
    end)
  end
  -- E802 bit0/bit1 are the serial clock and data; they should move ONLY during $A118/$A1CE
  for _, base in ipairs({ 0x00e802, 0xffe802 }) do
    taps[#taps+1] = sp:install_write_tap(base, base + 1, "e802", function(off, data, mask)
      ne802 = ne802 + 1
      if ne802 <= 40 then
        E(string.format("  E802 <= %04x  clk(bit0)=%d dat(bit1)=%d  cmd=%02x [7a0a]=%04x",
          data & 0xffff, data & 1, (data >> 1) & 1, B(0x71f0), W(0x7a0a)))
      end
    end)
  end
  taps[#taps+1] = osp:install_read_tap(0xa118, 0xa119, "send", function()
    E(string.format("  $A118 SEND entry  cmd=%02x [7a0a]=%04x", B(0x71f0), W(0x7a0a)))
  end)
  taps[#taps+1] = osp:install_read_tap(0xa16a, 0xa16b, "clklo", function()
    E("  $A16A deep: clock-low reached (send loop really running)")
  end)
  print("esdisel armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  if now() >= STOP then
    print("=== unit select / serial lines ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== E802 writes total: %d", ne802))
    io.flush(); manager.machine:exit()
  end
end)
