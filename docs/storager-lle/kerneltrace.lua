-- kerneltrace.lua — kernel-era C800 ~pair writes + FM/MFM E000 field-word capture.
-- Self-healing taps (the device's handler reinstalls wipe Lua taps once mid-boot).
local CPU = ":slot1:storager:cpu"
local AVDC = ":slot3:serad:port0:s97801:term:avdc"
local cpu = manager.machine.devices[CPU]
local sp  = cpu.spaces["program"]
local ops = cpu.spaces["opcodes"] or sp

local function now() return manager.machine.time:as_double() end
local function pc()  local ok,v = pcall(function() return cpu.state["PC"].value end); return ok and v or 0 end
local function r16(a) local ok,v = pcall(function() return sp:read_u16(a) end); return ok and v or 0 end

_G.keep = {}
local nC8, nE0, n18 = 0, 0, 0

-- C800 file: boundary cells (byte<0x80) logged tersely; higher cells = the ~pair flavor -> decode B
local function cb_c800(offset, data, mask)
  if now() < 6.2 or nC8 > 3000 then return end
  nC8 = nC8 + 1
  local a = offset & 0x1ff              -- byte offset within the file
  if a >= 0x80 then
    local B = (~((((0x800 | a) & 0x1fe) >> 1) << 16 | data)) & 0xffffff   -- if this is counter-B: offset bit11 set
    local B0 = (~(((a & 0x1fe) >> 1) << 16 | data)) & 0xffffff            -- plain decode ignoring the 0x800 base
    print(string.format("C8HI [%03x]=%04x pc=%06x @%.6f decodeB=%06x/%06x", a, data, pc(), now(), B, B0))
  else
    print(string.format("C800[%02x]=%04x pc=%06x @%.6f", a >> 1, data, pc(), now()))
  end
end

-- E000 window: the 16-word channel-control list
local function cb_e000(offset, data, mask)
  if now() < 6.2 or nE0 > 1500 then return end
  nE0 = nE0 + 1
  print(string.format("E000[%02x]=%03x pc=%06x @%.6f", (offset & 0xf) << 1, data & 0xfff, pc(), now()))
end

local function cb_c000(offset, data, mask)
  if now() < 6.2 then return end
  -- model logs presets too; log here for unified timeline
  local a = offset & 0x3ff
  print(string.format("C000[%03x]=%04x pc=%06x @%.6f B=%06x", (a << 1) & 0x7fe, data, pc(), now(),
    (~((((a << 1) & 0x1fe) >> 1) << 16 | data)) & 0xffffff))
end

local function arm()
  for k, t in pairs(_G.keep) do pcall(function() t:remove() end) end
  _G.keep.c8 = sp:install_write_tap(0xc800, 0xc9ff, "t_c8", cb_c800)
  _G.keep.c8m = sp:install_write_tap(0xffc800, 0xffc9ff, "t_c8m", cb_c800)
  _G.keep.c0 = sp:install_write_tap(0xffc000, 0xffc7ff, "t_c0", cb_c000)
  _G.keep.e0 = sp:install_write_tap(0xe000, 0xe01f, "t_e0", cb_e000)
  _G.keep.e0m = sp:install_write_tap(0xffe000, 0xffe01f, "t_e0m", cb_e000)
end
arm()

-- record-area dump at the 0x18/0x1A loaders: triples + the 16 field words at +0x32
local function dump_records(label)
  if n18 >= 16 then return end
  n18 = n18 + 1
  local a2 = r16(0x7938)
  local o = { string.format("%s @%.6f rec=%04x cnt=%d tri:", label, now(), a2, r16(a2)) }
  local p = a2 + 2
  for i = 0, 15 do
    o[#o+1] = string.format(" %02x%02x%02x", sp:read_u8(p), sp:read_u8(p+1), sp:read_u8(p+2)); p = p + 3
  end
  o[#o+1] = " fw:"
  for i = 0, 15 do o[#o+1] = string.format(" %03x", r16(a2 + 0x32 + 2*i) & 0xfff) end
  o[#o+1] = string.format(" | 7a16=%04x 71bc.cmd=%02x", r16(0x7a16), sp:read_u8(r16(0x71bc)))
  print(table.concat(o))
end
_G.x18 = ops:install_read_tap(0x308c, 0x308d, "x_18", function(o, d, m)
  if pc() == 0x308c and now() > 6.2 then dump_records("OP18") end end)
_G.x1a = ops:install_read_tap(0x3182, 0x3183, "x_1a", function(o, d, m)
  if pc() == 0x3182 and now() > 6.2 then dump_records("OP1A") end end)

local prev_tail = ""
emu.register_periodic(function()
  local ok, t = pcall(now); if not ok then return end
  arm()
  local okd, dev = pcall(function() return manager.machine.devices[AVDC] end)
  if okd and dev then
    local s = dev.spaces["charram"]
    if s then
      local o = {}
      for b = 0, 0x1fff, 80 do for i = 0, 79 do local c = s:read_u8(b + i); o[#o+1] = (c >= 0x20 and c < 0x7f) and string.char(c) or " " end end
      local tail = table.concat(o):gsub("%s+", " "):gsub("%s+$", ""):sub(-120)
      if tail ~= prev_tail and #tail > 0 then prev_tail = tail; print(string.format("SCREEN @%.1f |%s", t, tail)) end
    end
  end
  io.flush()
  if t >= 170 then manager.machine:exit() end
end)
print("kerneltrace armed")
io.flush()
