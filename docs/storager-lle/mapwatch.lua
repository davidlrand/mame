-- mapwatch.lua — who writes the $7654 map, the $7696 chunk links, and the $74AC-$74B8 heads
local CPU = ":slot1:storager:cpu"
local cpu = manager.machine.devices[CPU]
local sp = cpu.spaces["program"]
local function now() return manager.machine.time:as_double() end
local function pc() local ok,v = pcall(function() return cpu.state["PC"].value end); return ok and v or 0 end
_G.keep = {}
local n = 0
local function mk(base, name)
  return function(offset, data, mask)
    local t = now(); if t < 7.9 or t > 8.6 or n > 600 then return end
    n = n + 1
    print(string.format("%s[+%03x]=%04x pc=%06x @%.6f", name, (offset << 1) - base + 0x8000, data, pc(), t))
  end
end
-- note: offset scaling differs per install; print raw offset<<1 and correct in analysis
local function cb(name)
  return function(offset, data, mask)
    local t = now(); if t < 7.9 or t > 8.6 or n > 600 then return end
    n = n + 1
    print(string.format("%s off=%05x =%04x pc=%06x @%.6f", name, offset, data, pc(), t))
  end
end
local function arm()
  for k,tp in pairs(_G.keep) do pcall(function() tp:remove() end) end
  _G.keep.map = sp:install_write_tap(0x7654, 0x76ff, "t_map", cb("MAP"))
  _G.keep.hd  = sp:install_write_tap(0x74ac, 0x74bb, "t_hd", cb("HEAD"))
end
arm()
emu.register_periodic(function()
  local ok,t = pcall(now); if not ok then return end
  arm(); io.flush()
  if t >= 9.3 then manager.machine:exit() end
end)
print("mapwatch armed"); io.flush()
