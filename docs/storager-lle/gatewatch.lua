-- gatewatch.lua — trace the sys-floppy handler's decision points + the second label buffer
local CPUAP = ":slot6:cpuap:cpu"
local AVDC = ":slot3:serad:port0:s97801:term:avdc"
local cp = manager.machine.devices[CPUAP]
local ps = cp.spaces["program"]
local ops = cp.spaces["opcodes"] or ps
local function now() return manager.machine.time:as_double() end
local function pcap() local ok,v = pcall(function() return cp.state["PC"].value end); return ok and v or 0 end
_G.keep = {}
local hits = {}
local sites = {
  [0xfe4484] = "READ1-issue", [0xfe448e] = "S-GATE-chk1", [0xfe4495] = "S-GATE-chk2",
  [0xfe449d] = "READ2-SETUP", [0xfe44af] = "READ2-issue", [0xfe44b9] = "MAGIC-CHK",
  [0xfe44ca] = "NO-SYS-FLOPPY", [0xfe44e5] = "BOOT-LOAD!", [0xfe4364] = "spec-path",
  [0xfe45fc] = "attempt-loop",
}
local nx = 0
local function cb_exec(offset, data, mask)
  local p = pcap()
  -- range covers all sites; log first N transitions with site names (pc-lag tolerant: check offset<<1? opcodes tap offset = byte addr for ns32k? use offset directly)
  local a = offset
  local nm = sites[a]
  if nm and nx < 120 then
    nx = nx + 1
    print(string.format("SITE %s pc=%06x @%.6f", nm, a, now()))
  end
end
local function arm()
  for k, t in pairs(_G.keep) do pcall(function() t:remove() end) end
  _G.keep.x = ops:install_read_tap(0xfe4360, 0xfe4601, "x_gate", cb_exec)
end
arm()
local dumps = {}
local function dump(tag, t)
  if dumps[tag] then return end
  dumps[tag] = true
  for _, base in ipairs({0xfc3dd, 0xfc074 + 0x69}) do
    local o = { string.format("GBUF-%s @%.3f %06x:", tag, t, base) }
    for k = 0, 0x17 do o[#o+1] = string.format(" %02x", ps:read_u8(base + k)) end
    print(table.concat(o))
  end
  io.flush()
end
local prev = ""
emu.register_periodic(function()
  local ok, t = pcall(now); if not ok then return end
  arm()
  if t > 3.40 then dump("A3", t) end
  if t > 8.40 then dump("A8", t) end
  if t > 9.05 then dump("B8", t) end
  local okd, dev = pcall(function() return manager.machine.devices[AVDC] end)
  if okd and dev then
    local s = dev.spaces["charram"]
    if s then
      local o = {}
      for b = 0, 0x1fff, 80 do for i = 0, 79 do local c = s:read_u8(b + i); o[#o+1] = (c >= 0x20 and c < 0x7f) and string.char(c) or " " end end
      local tail = table.concat(o):gsub("%s+", " "):gsub("%s+$", ""):sub(-100)
      if tail ~= prev and #tail > 0 then prev = tail; print(string.format("SCREEN @%.1f |%s", t, tail)) end
    end
  end
  io.flush()
  if t >= 12 then manager.machine:exit() end
end)
print("gatewatch armed")
io.flush()
