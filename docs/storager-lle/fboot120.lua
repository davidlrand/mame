-- fboot120: screen-tail printer, full dump @20s, exit @120s
-- (recreated cont.255 after the 2026-07-18 reboot wiped the scratchpad; screen
-- reader cloned from docs/storager-lle/stg2-repro.lua)
local AVDC = ":slot3:serad:port0:s97801:term:avdc"
local function scr()
  local ok, dev = pcall(function() return manager.machine.devices[AVDC] end)
  if not ok or not dev then return "" end
  local s = dev.spaces["charram"]; if not s then return "" end
  local o = {}
  for b = 0, 0x1fff, 80 do
    for i = 0, 79 do
      local c = s:read_u8(b + i)
      o[#o + 1] = (c >= 0x20 and c < 0x7f) and string.char(c) or " "
    end
  end
  return table.concat(o)
end
local prev, dumped = "", false
emu.register_periodic(function()
  local ok, t = pcall(function() return manager.machine.time:as_double() end)
  if not ok then return end
  local s = scr()
  local tail = s:gsub("%s+", " "):gsub("%s+$", ""):sub(-110)
  if tail ~= prev then
    prev = tail
    print(string.format("@%.1f |%s", t, tail)); io.flush()
  end
  if t >= 20 and not dumped then
    dumped = true
    print("==== FULL SCREEN @20s ====")
    for i = 0, (0x2000 / 80) - 1 do
      local row = s:sub(i * 80 + 1, (i + 1) * 80)
      if row:match("%S") then print(string.format("%3d|%s", i, row)) end
    end
    print("==== END FULL SCREEN ===="); io.flush()
  end
  if t >= 120 then manager.machine:exit() end
end)
