-- pullprobe.lua — the fw's E000 data-byte pull loop: does it run, what does it read?
local CPU = ":slot1:storager:cpu"
local cpu = manager.machine.devices[CPU]
local sp = cpu.spaces["program"]
local function now() return manager.machine.time:as_double() end
local function pc() local ok,v = pcall(function() return cpu.state["PC"].value end); return ok and v or 0 end
_G.keep = {}
local n, lastp = 0, 0
local function cb(offset, data, mask)
  local t = now()
  if t < 8.0 or t > 8.12 or n > 250 then return end
  n = n + 1
  local p = pc()
  print(string.format("E000RD =%04x pc=%06x @%.6f", data, p, t))
end
local function arm()
  for k,tp in pairs(_G.keep) do pcall(function() tp:remove() end) end
  _G.keep.r = sp:install_read_tap(0xe000, 0xe001, "t_e0r", cb)
  _G.keep.r2 = sp:install_read_tap(0xffe000, 0xffe001, "t_e0r2", cb)
end
arm()
emu.register_periodic(function()
  local ok,t = pcall(now); if not ok then return end
  arm(); io.flush()
  if t >= 10 then manager.machine:exit() end
end)
print("pullprobe armed"); io.flush()
