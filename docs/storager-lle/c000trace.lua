-- c000trace.lua — C000-bank write trace: decode each write as a complemented 24-bit host address.
-- Tap is re-installed every periodic tick (the device's handler reinstalls wipe Lua taps once mid-boot).
local CPU = ":slot1:storager:cpu"
local cpu = manager.machine.devices[CPU]
local sp  = cpu.spaces["program"]

local function now() return manager.machine.time:as_double() end
local function pc()  local ok,v = pcall(function() return cpu.state["PC"].value end); return ok and v or 0 end
local function r16(a) local ok,v = pcall(function() return sp:read_u16(a) end); return ok and v or 0 end
local function r32(a) local ok,v = pcall(function() return sp:read_u32(a) end); return ok and v or 0 end

_G.keep = {}
local n = 0

local function cb(offset, data, mask)
  n = n + 1
  if n > 400 then return end
  -- offset is the byte address within the tapped range? MAME passes the absolute address for taps.
  local a = offset & 0x7ff
  local vbar = ((a & 0x1fe) >> 1) << 16 | data          -- ~B as loaded
  local B = (~vbar) & 0xffffff
  print(string.format("C000[%03x]=%04x -> ~load=%06x B=%06x pc=%06x @%.6f | 795e=%08x 79d8=%08x",
    a, data, vbar, B, pc(), now(), r32(0x795e), r32(0x79d8)))
end

local function arm()
  if _G.keep.t then pcall(function() _G.keep.t:remove() end) end
  _G.keep.t = sp:install_write_tap(0xffc000, 0xffc7ff, "t_c000", cb)
end
arm()

emu.register_periodic(function()
  local ok, t = pcall(now); if not ok then return end
  arm()                       -- self-heal against handler reinstalls
  io.flush()
  if t >= 45 then manager.machine:exit() end
end)
print("c000trace armed")
io.flush()
