-- blocks.lua - dump the live control blocks during the read, with the fields we have decoded.
--
-- A fresh look at the DATA rather than the code.  The firmware indexes the UIB well past the 0x20
-- bytes the model DMAs for it (+$ca..+$dc are read at $7C40/$67EE/$68AC/$93C0), so either those are
-- firmware-maintained scratch or the transfer length is wrong.  This dumps enough to tell.
--
-- All reads happen in the PERIODIC, never inside a tap: a Lua SRAM read inside a CPU write tap
-- trips storager.cpp's own $4000-$7FFF DMA snoop.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle \
--          -autoboot_script docs/storager-lle/blocks.lua -flop siemens/set1/mx2-001.imd

local SC   = ":slot1:storager:cpu"
local STOP = 30.0

local cpu, sp, armed, done = nil, nil, false, false

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function B(a) return sp:read_u8(a & 0xffff) end

local function hex(base, len)
  local out = {}
  for i = 0, len - 1, 16 do
    local line = string.format("  %04x: ", (base + i) & 0xffff)
    local asc = ""
    for j = 0, 15 do
      if i + j < len then
        local b = B(base + i + j)
        line = line .. string.format("%02x ", b)
        asc = asc .. ((b >= 32 and b < 127) and string.char(b) or ".")
      else line = line .. "   " end
    end
    out[#out+1] = line .. " " .. asc
  end
  return table.concat(out, "\n")
end

emu.register_periodic(function()
  local t = now()
  if not armed then
    local ok, d = pcall(function() return manager.machine.devices[SC] end)
    if not ok or not d then return end
    armed = true; cpu = d; sp = d.spaces["program"]
    print("blocks armed"); io.flush()
    return
  end
  -- dump once, while the read is live and past the data
  if not done and B(0x71f0) == 0x95 and t > 8.6 then
    done = true
    local uib, node = W(0x799a), W(0x71bc)
    print(string.format("=== t=%.4f  UIB[799a]=%04x  node[71bc]=%04x  cmd=%02x st=%04x",
      t, uib, node, B(0x71f0), W(0x71f2)))

    print(string.format("--- UIB @ %04x, first 0x20 (what the model DMAs) ---", uib))
    print(hex(uib, 0x20))
    print(string.format("--- UIB @ %04x, +0x20..+0xE0 (offsets the firmware ALSO reads) ---", uib))
    print(hex(uib + 0x20, 0xc0))

    print(string.format("--- node @ %04x, 0x28 bytes (0x18 DMAd + the status tail) ---", node))
    print(hex(node, 0x28))

    print("--- decoded UIB fields ---")
    local u11, u12 = B(uib + 0x11), B(uib + 0x12)
    print(string.format("  +$01 = %02x   (count, read at $7030/$7362)", B(uib + 0x01)))
    print(string.format("  +$03 = %02x   (compared >= 2 at $7D34)", B(uib + 0x03)))
    print(string.format("  +$11 = %02x   bit0=%d -> [796a] (drain loop) | bit1=%d -> [79a0] (record path)",
      u11, u11 & 1, (u11 >> 1) & 1))
    print(string.format("  +$12 = %02x   bit1=%d (MFM density) | bit7=%d (GA op-complete)",
      u12, (u12 >> 1) & 1, (u12 >> 7) & 1))
    print(string.format("  +$18 = %02x   (op42 guard delay, %d units)", B(uib + 0x18), B(uib + 0x18)))
    print(string.format("  +$1a/$1b = %02x %02x  (word limit vs [$7948])", B(uib + 0x1a), B(uib + 0x1b)))
    print(string.format("  +$20 = %04x (flags; bit14=%d template select)", W(uib + 0x20), (W(uib + 0x20) >> 14) & 1))
    print(string.format("  +$ca = %04x  +$cc = %04x  +$ce = %04x   (ID-field pointers)",
      W(uib + 0xca), W(uib + 0xcc), W(uib + 0xce)))
    print(string.format("  +$d0 = %04x  +$d2 = %04x  +$d6 = %04x  +$da = %04x  +$dc = %04x  (position/target)",
      W(uib + 0xd0), W(uib + 0xd2), W(uib + 0xd6), W(uib + 0xda), W(uib + 0xdc)))
    print("--- live state ---")
    print(string.format("  [796a]=%04x [79a0]=%04x [7968]=%04x [742C]=%04x [7956]=%04x [7abc]=%04x [72d8]=%04x",
      W(0x796a), W(0x79a0), W(0x7968), W(0x742c), W(0x7956), W(0x7abc), W(0x72d8)))
    print(string.format("  [7436]=%04x (expected cyl) [7438]=%04x (expected sec) [7428]=%04x (accepted)",
      W(0x7436), W(0x7438), W(0x7428)))
    io.write("  ledger $7654..$7667:")
    for a = 0x7654, 0x7667 do io.write(string.format(" %02x", B(a))) end
    io.write("\n")
    io.flush()
  end
  if t >= STOP then io.flush(); manager.machine:exit() end
end)
