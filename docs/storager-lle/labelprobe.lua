-- labelprobe.lua — why does the monitor reject the delivered label?
-- (a) trace the CPUAP's fe4484-fe44d0 check path, (b) log its data reads around the label buffer,
-- (c) dump the buffer at 8.178.
local CPUAP = ":slot6:cpuap:cpu"
local cp = manager.machine.devices[CPUAP]
local ps = cp.spaces["program"]

local function now() return manager.machine.time:as_double() end
local function pcap() local ok,v = pcall(function() return cp.state["PC"].value end); return ok and v or 0 end

_G.keep = {}
local nrd, nx = 0, 0

local function cb_exec(offset, data, mask)
  local p = pcap()
  if p >= 0xfe4480 and p <= 0xfe44d4 and now() > 8.0 and nx < 200 then
    nx = nx + 1
    print(string.format("XPC %06x @%.6f", p, now()))
  end
end

local function cb_read(offset, data, mask)
  if now() < 8.16 or now() > 8.20 or nrd > 300 then return end
  nrd = nrd + 1
  print(string.format("RD [%06x]=%04x pc=%06x @%.6f", offset << 1, data, pcap(), now()))
end

local function arm()
  for k, t in pairs(_G.keep) do pcall(function() t:remove() end) end
  -- exec trace via opcode-fetch tap on the check region
  _G.keep.x = (cp.spaces["opcodes"] or ps):install_read_tap(0xfe4480, 0xfe44d7, "x_chk", cb_exec)
  -- data reads across the label-buffer region (host RAM 0xFC000-0xFC2FF)
  _G.keep.r = ps:install_read_tap(0xfc000, 0xfc2ff, "r_buf", cb_read)
end
arm()

local dumped = false
emu.register_periodic(function()
  local ok, t = pcall(now); if not ok then return end
  arm()
  if not dumped and t > 8.178 then
    dumped = true
    for base = 0xfc0c0, 0xfc1c0, 0x40 do
      local o = { string.format("HOST %06x:", base) }
      for k = 0, 0x3f do o[#o+1] = string.format(" %02x", ps:read_u8(base + k)) end
      print(table.concat(o))
    end
    io.flush()
  end
  io.flush()
  if t >= 20 then manager.machine:exit() end
end)
print("labelprobe armed")
io.flush()
