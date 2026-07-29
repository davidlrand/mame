-- uibwatch.lua — the 0x87 re-INIT: host UIB vs fw UIB + the error code
local CPU = ":slot1:storager:cpu"
local cpu = manager.machine.devices[CPU]
local sp = cpu.spaces["program"]
local cp = manager.machine.devices[":slot6:cpuap:cpu"]
local ps = cp.spaces["program"]
local function now() return manager.machine.time:as_double() end
local function pc() local ok,v = pcall(function() return cpu.state["PC"].value end); return ok and v or 0 end
_G.keep = {}
local n = 0
local function cb(name)
  return function(offset, data, mask)
    local t = now(); if t < 8.18 or t > 8.6 or n > 60 then return end
    n = n + 1
    print(string.format("%s=%04x pc=%06x @%.6f", name, data, pc(), t))
  end
end
local function arm()
  for k,tp in pairs(_G.keep) do pcall(function() tp:remove() end) end
  _G.keep.a = sp:install_write_tap(0x7208, 0x7209, "t_e1", cb("ERR71F0"))
  _G.keep.b = sp:install_write_tap(0x71de, 0x71df, "t_e0", cb("ERR71C6"))
end
arm()
local dumped = false
emu.register_periodic(function()
  local ok, t = pcall(now); if not ok then return end
  arm()
  if not dumped and t > 8.25 then
    dumped = true
    local okd, err = pcall(function()
    local h, f = {"HOSTUIB fe948:"}, {}
    for k = 0, 0x1f do h[#h+1] = string.format(" %02x", ps:read_u8(0xfe948 + k)) end
    print(table.concat(h))
    local uib = sp:read_u16(0x799a)
    f[1] = string.format("FWUIB %04x:", uib)
    for k = 0, 0x1f do f[#f+1] = string.format(" %02x", sp:read_u8(uib + k)) end
    print(table.concat(f))
    end)
    if not okd then print("DUMPERR " .. tostring(err)) end
  end
  io.flush()
  if t >= 9.5 then manager.machine:exit() end
end)
print("uibwatch armed"); io.flush()
