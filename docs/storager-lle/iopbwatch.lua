-- iopbwatch.lua — the host IOPB status bytes: trajectory + writers
local cp = manager.machine.devices[":slot6:cpuap:cpu"]
local ps = cp.spaces["program"]
local function now() return manager.machine.time:as_double() end
local function cpc() local ok,v = pcall(function() return cp.state["PC"].value end); return ok and (v & 0xffffff) or 0 end
_G.keep = {}
local n = 0
local function cb(offset, data, mask)
  local t = now(); if t < 6.3 or t > 8.6 or n > 60 then return end
  n = n + 1
  print(string.format("IOPBW off=%06x =%04x mask=%04x cpupc=%06x @%.6f", offset, data, mask, cpc(), t))
end
local function arm()
  for k,tp in pairs(_G.keep) do pcall(function() tp:remove() end) end
  _G.keep.w = ps:install_write_tap(0xfe780, 0xfe787, "t_iopb", cb)
end
arm()
local lastb = ""
emu.register_periodic(function()
  local ok,t = pcall(now); if not ok then return end
  arm()
  if t > 6.3 and t < 8.6 then
    local b = {}
    for k = 0, 7 do b[#b+1] = string.format("%02x", ps:read_u8(0xfe780 + k)) end
    local line = table.concat(b, " ")
    if line ~= lastb then lastb = line; print(string.format("IOPB @%.4f: %s", t, line)) end
  end
  io.flush()
  if t >= 9 then manager.machine:exit() end
end)
print("iopbwatch armed"); io.flush()
