-- ringwatch.lua — the pio-visible ring entries ($7E20-$7E5F): status/error stamps in the INIT era
local CPU = ":slot1:storager:cpu"
local cpu = manager.machine.devices[CPU]
local sp = cpu.spaces["program"]
local function now() return manager.machine.time:as_double() end
local function pc() local ok,v = pcall(function() return cpu.state["PC"].value end); return ok and v or 0 end
_G.keep = {}
local n = 0
local function cb(offset, data, mask)
  local t = now(); if t < 8.15 or t > 8.6 or n > 120 then return end
  n = n + 1
  print(string.format("RING off=%05x =%04x pc=%06x @%.6f", offset, data, pc(), t))
end
local function arm()
  for k,tp in pairs(_G.keep) do pcall(function() tp:remove() end) end
  _G.keep.r = sp:install_write_tap(0x7e20, 0x7e5f, "t_ring", cb)
end
arm()
emu.register_periodic(function()
  local ok,t = pcall(now); if not ok then return end
  arm(); io.flush()
  if t >= 9.3 then manager.machine:exit() end
end)
print("ringwatch armed"); io.flush()
