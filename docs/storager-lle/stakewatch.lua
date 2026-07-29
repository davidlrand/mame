-- stakewatch.lua — write-taps (reliable) on the stake chain's data cells
local CPU = ":slot1:storager:cpu"
local cpu = manager.machine.devices[CPU]
local sp = cpu.spaces["program"]
local function now() return manager.machine.time:as_double() end
local function pc() local ok,v = pcall(function() return cpu.state["PC"].value end); return ok and v or 0 end
_G.keep = {}
local n = 0
local function cb742c(offset, data, mask)
  local t = now(); if t < 7.9 or t > 8.9 or n > 200 then return end
  n = n + 1
  print(string.format("W742C=%04x pc=%06x @%.6f", data, pc(), t))
end
local function cbledger(offset, data, mask)
  local t = now(); if t < 7.9 or t > 8.9 or n > 200 then return end
  n = n + 1
  print(string.format("WLEDG[%04x]=%04x pc=%06x @%.6f", 0x7654 + ((offset - (0x7654 >> 1)) << 1), data, pc(), t))
end
local function cb7950(offset, data, mask)
  local t = now(); if t < 7.9 or t > 8.9 or n > 200 then return end
  n = n + 1
  print(string.format("W7950=%04x pc=%06x @%.6f", data, pc(), t))
end
local function arm()
  for k,tp in pairs(_G.keep) do pcall(function() tp:remove() end) end
  _G.keep.a = sp:install_write_tap(0x742c, 0x742d, "t_742c", cb742c)
  _G.keep.b = sp:install_write_tap(0x7654, 0x7677, "t_ledg", cbledger)
  _G.keep.c = sp:install_write_tap(0x7950, 0x7951, "t_7950", cb7950)
  _G.keep.f = sp:install_write_tap(0x7428, 0x7429, "t_7428", function(o,d,m) local t=now(); if t<7.9 or t>8.9 or n>250 then return end; n=n+1; print(string.format("W7428=%04x pc=%06x @%.6f", d, pc(), t)) end)
  _G.keep.e = sp:install_write_tap(0x7964, 0x7965, "t_7964", function(o,d,m) local t=now(); if t<7.9 or t>8.9 or n>250 then return end; n=n+1; print(string.format("W7964=%04x pc=%06x @%.6f", d, pc(), t)) end)
  _G.keep.d = sp:install_write_tap(0x7a0c, 0x7a0d, "t_7a0c", function(o,d,m) local t=now(); if t<7.9 or t>8.9 or n>200 then return end; n=n+1; print(string.format("W7A0C=%04x pc=%06x @%.6f", d, pc(), t)) end)
end
arm()
emu.register_periodic(function()
  local ok,t = pcall(now); if not ok then return end
  arm(); io.flush()
  if t >= 10 then manager.machine:exit() end
end)
print("stakewatch armed"); io.flush()
