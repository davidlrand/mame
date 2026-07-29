-- bufwatch.lua — dump the CPUAP label buffer (0FC0DD) around the READ#2 decision
local AVDC = ":slot3:serad:port0:s97801:term:avdc"
local CPUAP = ":slot6:cpuap:cpu"
local cp = manager.machine.devices[CPUAP]
local ps = cp.spaces["program"]
local prev = ""
local dumps = {}
local function dump(tag, t)
  if dumps[tag] then return end
  dumps[tag] = true
  for base = 0, 0x180, 0x80 do
    local o = { string.format("BUF-%s @%.3f 0fc0dd+%03x:", tag, t, base) }
    for k = 0, 0x2f do o[#o+1] = string.format(" %02x", ps:read_u8(0xfc0dd + base + k)) end
    print(table.concat(o))
  end
  io.flush()
end
emu.register_periodic(function()
  local ok, t = pcall(function() return manager.machine.time:as_double() end); if not ok then return end
  -- long arm: read-A completes ~8.37; READ#2 ~8.41; verdict ~9.0. short arm: 3.37/3.41/4.0
  if t > 3.395 then dump("A3", t) end
  if t > 3.6 then dump("B3", t) end
  if t > 8.395 then dump("A8", t) end
  if t > 8.6 then dump("B8", t) end
  local okd, dev = pcall(function() return manager.machine.devices[AVDC] end)
  if okd and dev then
    local s = dev.spaces["charram"]
    if s then
      local o = {}
      for b = 0, 0x1fff, 80 do for i = 0, 79 do local c = s:read_u8(b + i); o[#o+1] = (c >= 0x20 and c < 0x7f) and string.char(c) or " " end end
      local tail = table.concat(o):gsub("%s+", " "):gsub("%s+$", ""):sub(-120)
      if tail ~= prev and #tail > 0 then prev = tail; print(string.format("SCREEN @%.1f |%s", t, tail)) end
    end
  end
  io.flush()
  if t >= 30 then manager.machine:exit() end
end)
print("bufwatch armed")
io.flush()
