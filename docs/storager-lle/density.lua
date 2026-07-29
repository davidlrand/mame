-- density.lua - does the firmware ever command MFM?
--
-- The model takes density from E800 bit10 (flux_density_fm = !BIT(m_ch[E800], 10)).  Media is MIXED:
-- cyl 0 is 300k FM 16x128 (the label track), cyl 1+ are 300k MFM 16x256.  At the cyl-1 read the model
-- reports density=FM and decodes ZERO sectors - an FM decoder on an MFM track.
-- So either the firmware never sets E800 bit10, or it signals density somewhere else.
-- node+$12 bit1 is the known MFM/double-density flag in the firmware's own work area.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/density.lua -flop siemens/set1/mx2-001.imd

local SC, STOP = ":slot1:storager:cpu", 12.0
local cpu, sp, armed = nil, nil, false
local taps, ev = {}, {}
local last10, lastn12 = -1, -1

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function B(a) return sp:read_u8(a & 0xffff) end
local function W(a) return sp:read_u16(a & 0xffff) end
local function E(s) if #ev < 60 then ev[#ev+1] = string.format("%9.4f  %s", now(), s) end end

local function arm()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  for _, base in ipairs({ 0x00e800, 0xffe800 }) do
    taps[#taps+1] = sp:install_write_tap(base, base + 1, "e800", function(off, data, mask)
      local b10 = (data >> 10) & 1
      if b10 ~= last10 then
        last10 = b10
        E(string.format("E800 <= %04x   bit10=%d -> gate array will decode %s", data & 0xffff, b10,
          (b10 == 1) and "MFM" or "FM"))
      end
    end)
  end
  print("density armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm() then return end
  local node = W(0x71bc); if node < 0x4000 or node >= 0x8000 then node = 0x71f0 end
  local n12 = B(node + 0x12)
  if n12 ~= lastn12 then
    lastn12 = n12
    E(string.format("node+$12 = %02x  bit1(MFM)=%d  bit2=%d", n12, (n12 >> 1) & 1, (n12 >> 2) & 1))
  end
  if now() >= STOP then
    print("=== density signalling ===")
    for _, l in ipairs(ev) do print(l) end
    print(string.format("=== final: E800 bit10 seen set? %s", (last10 == 1) and "YES" or "NEVER"))
    io.flush(); manager.machine:exit()
  end
end)
