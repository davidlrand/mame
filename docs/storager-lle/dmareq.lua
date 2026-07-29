-- dmareq.lua - is a host DMA ever REQUESTED during the read?
--
-- IRQ4 = host-transfer complete; the handler ACKs by clearing E800 bit12 ($3C02 andi #$efff,$79f6 ->
-- $6800,A5).  So bit12 rising on E800 is the request.  If it never rises during the data phase then
-- no IRQ4 can exist, and the whole $3C98 -> $3F68 -> $3FFC -> $400C/$4002 chain is unreachable for a
-- reason upstream of anything inside $3Fxx.
--
-- Tap BOTH bases: the firmware reaches E800 A5-relative (lands at $FFE800) but other sites use long
-- absolute.  And take no SRAM reads inside the tap - storager.cpp snoops $4000-$7FFF.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle \
--          -autoboot_script docs/storager-lle/dmareq.lua -flop siemens/set1/mx2-001.imd

local SC   = ":slot1:storager:cpu"
local STOP = 30.0

local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev = {}, {}
local n_e800, n_bit12, n_irq4 = 0, 0, 0
local g_reading = false

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function B(a) return sp:read_u8(a & 0xffff) end
local function PC() local ok,v = pcall(function() return cpu.state["PC"].value end); return (ok and v or 0) & 0xffffff end
local function E(s) if #ev < 120 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp
  for _, base in ipairs({ 0x00e800, 0xffe800 }) do
    taps[#taps+1] = sp:install_write_tap(base, base + 1, "e800", function(off, data, mask)
      if not g_reading then return end
      n_e800 = n_e800 + 1
      local v = data & 0xffff
      if (v & 0x1000) ~= 0 then
        n_bit12 = n_bit12 + 1
        E(string.format("E800 <= %04x  *** bit12 SET - HOST DMA REQUESTED ***  pc=%06x", v, PC()))
      elseif n_e800 <= 12 then
        E(string.format("E800 <= %04x  (bit12 clear)  pc=%06x", v, PC()))
      end
    end)
  end
  -- IRQ4 handler entry, 20 bytes in, clear of any shadow
  taps[#taps+1] = osp:install_read_tap(0x3c12, 0x3c13, "irq4", function()
    n_irq4 = n_irq4 + 1
    E(string.format("IRQ4 handler entry #%d", n_irq4))
  end)
  print("dmareq armed"); io.flush()
  return true
end

emu.register_periodic(function()
  local t = now()
  if not arm() then return end
  g_reading = (B(0x71f0) == 0x95)
  if t >= STOP then
    print("=== host-DMA requests during cmd 0x95 ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== E800 writes: %d ; with bit12 SET: %d ; IRQ4 entries (whole run): %d",
      n_e800, n_bit12, n_irq4))
    print(string.format("=== [743a]=%04x (=$7442 only if $414C ran)  [74b4]=%04x", W(0x743a), W(0x74b4)))
    io.flush(); manager.machine:exit()
  end
end)
