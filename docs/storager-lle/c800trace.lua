-- c800trace.lua — full-fidelity C800/D000/D800 write trace during the pcmx2 floppy boot.
-- Goal: pin the C800 scatter/gather file format (spec §3.4 / §6.4 open item).
local CPU = ":slot1:storager:cpu"
local AVDC = ":slot3:serad:port0:s97801:term:avdc"

local cpu = manager.machine.devices[CPU]
local sp  = cpu.spaces["program"]
local ops = cpu.spaces["opcodes"] or sp

local function now() return manager.machine.time:as_double() end
local function pc()  local ok,v = pcall(function() return cpu.state["PC"].value end); return ok and v or 0 end
local function r16(a) local ok,v = pcall(function() return sp:read_u16(a) end); return ok and v or 0xdead end
local function reg(n) local ok,v = pcall(function() return cpu.state[n].value end); return ok and v or 0 end

_G.taps = {}; local taps = _G.taps  -- keep tap handles alive (GLOBAL: locals get GC-d after the chunk ends)
local nc800, ne800, ne802, n18 = 0, 0, 0, 0
local prev_e800, prev_e802 = 0, 0

-- C800 file: log every write; on index 0 add the source-cell context
taps.c800 = sp:install_write_tap(0xc800, 0xc9ff, "t_c800", function(offset, data, mask)
  if now() < 6.2 then return end
  nc800 = nc800 + 1
  if nc800 <= 4000 or (offset & 0x1fe) == 0 then
    local extra = ""
    if (offset & 0x1fe) == 0 then
      extra = string.format(" | 741e=%04x 7434=%04x 7428=%04x 7424=%04x",
        r16(0x741e), r16(0x7434), r16(0x7428), r16(0x7424))
    end
    print(string.format("C800[%02x]=%04x pc=%06x @%.6f%s", (offset >> 1) & 0xff, data, pc(), now(), extra))
  end
end)

taps.d000 = sp:install_write_tap(0xd000, 0xd001, "t_d000", function(offset, data, mask)
  if now() < 6.2 then return end
  print(string.format("D000=%04x (byte %05x) pc=%06x @%.6f", data, (data << 1) & 0x1ffff, pc(), now()))
end)

taps.d800 = sp:install_write_tap(0xd800, 0xd801, "t_d800", function(offset, data, mask)
  if now() < 6.2 then return end
  print(string.format("D800=%04x (byte %05x) pc=%06x @%.6f", data, (data << 1) & 0x1ffff, pc(), now()))
end)

-- E800: kickoff edges (bit12 rising) always; family/arm strobes capped
taps.e800 = sp:install_write_tap(0xe800, 0xe801, "t_e800", function(offset, data, mask)
  if now() < 6.2 then prev_e800 = data; return end
  local rising = data & ~prev_e800
  if (rising & 0x1000) ~= 0 then
    print(string.format("E800 KICK %04x pc=%06x @%.6f", data, pc(), now()))
  elseif (rising & 0x0080) ~= 0 and ne800 < 60 then
    ne800 = ne800 + 1
    print(string.format("E800 arm  %04x pc=%06x @%.6f", data, pc(), now()))
  end
  prev_e800 = data
end)

-- E802: capture-arm bit15 rising, capped
taps.e802 = sp:install_write_tap(0xe802, 0xe803, "t_e802", function(offset, data, mask)
  if now() < 6.2 then prev_e802 = data; return end
  local rising = data & ~prev_e802
  if (rising & 0x8000) ~= 0 and ne802 < 200 then
    ne802 = ne802 + 1
    print(string.format("E802 arm15 %04x pc=%06x @%.6f", data, pc(), now()))
  end
  prev_e802 = data
end)

-- Execution taps: dump the {index,value,translate} record area when the 0x18/0x1A loaders run,
-- and the launch record at $3CD4.
local function dump_records(label)
  local a2 = r16(0x7938)
  local cnt = r16(a2)
  local o = { string.format("%s @%.6f rec=%04x count=%d:", label, now(), a2, cnt) }
  local p = a2 + 2
  for i = 0, math.min(cnt, 15) do
    local b0, b1, b2 = sp:read_u8(p), sp:read_u8(p + 1), sp:read_u8(p + 2)
    o[#o+1] = string.format(" {%02x,%02x,%02x}", b0, b1, b2)
    p = p + 3
  end
  o[#o+1] = string.format(" | 741e=%04x 7434=%04x 7a18=%04x 7a16=%04x",
    r16(0x741e), r16(0x7434), r16(0x7a18), r16(0x7a16))
  print(table.concat(o))
end

taps.x18 = ops:install_read_tap(0x308c, 0x308d, "x_18", function(offset, data, mask)
  if pc() == 0x308c and now() > 6.2 and n18 < 12 then n18 = n18 + 1; dump_records("OP18-ENTRY") end
end)
taps.x1a = ops:install_read_tap(0x3182, 0x3183, "x_1a", function(offset, data, mask)
  if pc() == 0x3182 and now() > 6.2 and n18 < 12 then n18 = n18 + 1; dump_records("OP1A-ENTRY") end
end)

local nlaunch = 0
taps.launch = ops:install_read_tap(0x3cd4, 0x3cd5, "x_launch", function(offset, data, mask)
  if pc() ~= 0x3cd4 or now() < 6.2 or nlaunch >= 24 then return end
  nlaunch = nlaunch + 1
  local rec = r16(0x743a)
  local o = { string.format("LAUNCH @%.6f rec=%04x:", now(), rec) }
  for i = 0, 0x20, 2 do o[#o+1] = string.format(" %04x", r16(rec + i)) end
  print(table.concat(o))
end)

-- $7696 chunk table + microseq snapshot once, at 6.3s (post-setup, pre-read1)
local dumped = false
local prev_tail = ""
emu.register_periodic(function()
  local ok, t = pcall(now); if not ok then return end
  if not dumped and t > 6.3 then
    dumped = true
    local o = { string.format("TBL7696 @%.6f:", t) }
    for n = 0, 15 do o[#o+1] = string.format(" [%d]=%04x,%04x,%04x", n, r16(0x7696+6*n), r16(0x7698+6*n), r16(0x769a+6*n)) end
    print(table.concat(o))
    local m = { "MICROSEQ ch0@722c:" }
    for i = 0, 14 do m[#m+1] = string.format(" %02x", r16(0x722c + 2*i)) end
    m[#m+1] = "  ch1@7250:"
    for i = 0, 14 do m[#m+1] = string.format(" %02x", r16(0x7250 + 2*i)) end
    print(table.concat(m))
  end
  -- screen tail for framing
  local okd, dev = pcall(function() return manager.machine.devices[AVDC] end)
  if okd and dev then
    local s = dev.spaces["charram"]
    if s then
      local o = {}
      for b = 0x1f00, 0x1fff do local c = s:read_u8(b); o[#o+1] = (c >= 0x20 and c < 0x7f) and string.char(c) or " " end
      local tail = table.concat(o):gsub("%s+", " "):gsub("%s+$", "")
      if tail ~= prev_tail and #tail > 0 then prev_tail = tail; print(string.format("SCREEN @%.1f |%s", t, tail)) end
    end
  end
  io.flush()
  if t >= 45 then manager.machine:exit() end
end)
print("c800trace armed")
io.flush()
