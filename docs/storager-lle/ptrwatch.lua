-- ptrwatch.lua — the node's +CA/+CC/+CE capture-field pointers and their targets, sampled through the read era
local CPU = ":slot1:storager:cpu"
local cpu = manager.machine.devices[CPU]
local sp = cpu.spaces["program"]
local function now() return manager.machine.time:as_double() end
local function r16(a) local ok,v = pcall(function() return sp:read_u16(a) end); return ok and v or 0 end
local function r8(a) local ok,v = pcall(function() return sp:read_u8(a) end); return ok and v or 0 end
local last = ""
emu.register_periodic(function()
  local ok, t = pcall(now); if not ok then return end
  if t > 7.96 and t < 8.6 then
    local node = r16(0x799a)
    local pca, pcc, pce = r16(node + 0xca), r16(node + 0xcc), r16(node + 0xce)
    local line = string.format("node=%04x ptrs ca=%04x cc=%04x ce=%04x | [ca]=%02x [cc]=%02x [ce]=%02x | 7dac: %02x %02x %02x %02x %02x %02x %02x %02x | 79a0=%04x",
      node, pca, pcc, pce, r8(pca), r8(pcc), r8(pce),
      r8(0x7dac), r8(0x7dad), r8(0x7dae), r8(0x7daf), r8(0x7db0), r8(0x7db1), r8(0x7db2), r8(0x7db3), r16(0x79a0))
    if line ~= last then last = line; print(string.format("@%.4f %s", t, line)) end
  end
  io.flush()
  if t >= 9 then manager.machine:exit() end
end)
print("ptrwatch armed"); io.flush()
