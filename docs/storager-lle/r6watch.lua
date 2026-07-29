-- r6watch.lua — sample the CPUAP's R6 (the sense/status block pointer) through the INIT era
local cp = manager.machine.devices[":slot6:cpuap:cpu"]
local ps = cp.spaces["program"]
local function now() return manager.machine.time:as_double() end
local last = ""
emu.register_periodic(function()
  local ok, t = pcall(now); if not ok then return end
  if t > 8.15 and t < 8.45 then
    local okr, r6 = pcall(function() return cp.state["R6"].value end)
    local okp, pcv = pcall(function() return cp.state["PC"].value end)
    if okr then
      local b = {}
      for k = 0, 11 do b[#b+1] = string.format("%02x", ps:read_u8((r6 + k) & 0xffffff)) end
      local line = string.format("R6=%06x pc=%06x blk=%s", r6 & 0xffffff, okp and (pcv & 0xffffff) or 0, table.concat(b, " "))
      if line ~= last then last = line; print(string.format("@%.4f %s", t, line)) end
    end
  end
  io.flush()
  if t >= 9 then manager.machine:exit() end
end)
print("r6watch armed"); io.flush()
