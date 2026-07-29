-- pubwatch.lua — every fw write into the pio-visible window ($7E00-$7FEF) carrying 0x80/0x82-class bytes
local CPU = ":slot1:storager:cpu"
local cpu = manager.machine.devices[CPU]
local sp = cpu.spaces["program"]
local function now() return manager.machine.time:as_double() end
local function pc() local ok,v = pcall(function() return cpu.state["PC"].value end); return ok and v or 0 end
_G.keep = {}
local n = 0
local function cb(offset, data, mask)
  local t = now(); if t < 8.05 or t > 8.5 or n > 80 then return end
  local lo, hi = data & 0xff, (data >> 8) & 0xff
  if lo == 0x80 or lo == 0x82 or hi == 0x80 or hi == 0x82 or lo == 0x81 or hi == 0x81 then
    n = n + 1
    print(string.format("PUB off=%05x =%04x pc=%06x @%.6f", offset, data, pc(), t))
  end
end
local function arm()
  for k,tp in pairs(_G.keep) do pcall(function() tp:remove() end) end
  _G.keep.w = sp:install_write_tap(0x7e00, 0x7fef, "t_pub", cb)
end
arm()
emu.register_periodic(function()
  local ok,t = pcall(now); if not ok then return end
  arm(); io.flush()
  if t >= 9 then manager.machine:exit() end
end)
print("pubwatch armed"); io.flush()
