-- lightwatch.lua — minimal screen watcher (light load -> short boot arm)
local AVDC = ":slot3:serad:port0:s97801:term:avdc"
local prev = ""
emu.register_periodic(function()
  local ok, t = pcall(function() return manager.machine.time:as_double() end); if not ok then return end
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
  if t >= 100 then manager.machine:exit() end
end)
print("lightwatch armed")
io.flush()
