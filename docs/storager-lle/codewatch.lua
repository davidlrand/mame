-- codewatch.lua — name the INIT's error code: node+2/+3 and node+$18, both channels
local CPU = ":slot1:storager:cpu"
local cpu = manager.machine.devices[CPU]
local sp = cpu.spaces["program"]
local function now() return manager.machine.time:as_double() end
local function pc() local ok,v = pcall(function() return cpu.state["PC"].value end); return ok and v or 0 end
_G.keep = {}
local n = 0
local function mk(name)
  return function(offset, data, mask)
    local t = now(); if t < 8.19 or t > 8.26 or n > 80 then return end
    n = n + 1
    print(string.format("%s off=%05x =%04x m=%04x pc=%06x @%.6f", name, offset, data, mask, pc(), t))
  end
end
local function arm()
  for k,tp in pairs(_G.keep) do pcall(function() tp:remove() end) end
  _G.keep.a = sp:install_write_tap(0x71c8, 0x71c9, "t_a", mk("CH0st"))
  _G.keep.b = sp:install_write_tap(0x71de, 0x71df, "t_b", mk("CH0e18"))
  _G.keep.c = sp:install_write_tap(0x71f2, 0x71f3, "t_c", mk("CH1st"))
  _G.keep.d = sp:install_write_tap(0x7208, 0x7209, "t_d", mk("CH1e18"))
end
arm()
emu.register_periodic(function()
  local ok,t = pcall(now); if not ok then return end
  arm(); io.flush()
  if t >= 9 then manager.machine:exit() end
end)
print("codewatch armed"); io.flush()
