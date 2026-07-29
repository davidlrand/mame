-- op00.lua - WHERE does the walker find the zero that ends the ladder?
--
-- $158C move.w (A1),D0 / $158E bne $1596 / $1590 move.w #$c,D1
-- "op00" is not detected by POSITION - it is detected by (A1) reading ZERO.  So any stray zero word
-- at the list pointer terminates the ladder and stamps node+$26 = $000C, which then completes with
-- 0x80 provided node+$18 is clean.  That is why op00 fires 1/2/2 times rather than once per command,
-- and why a read can complete without op36 ever running.
--
--   A1 at the ladder's real end      -> the built list is SHORTER than the builder emits
--   A1 somewhere else                -> the list pointer is wrong
-- Both decisive, neither needs a comparison, and it is structural within one command - seed-immune.
--
-- The builder emits (cont.422, verified from ROM at $5FE6..$603E):
--   24 28 56 58 [1A] 18 54 4A 42 36 00
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/op00.lua -flop siemens/set1/mx2-001.imd -hard1 <img>

local SC, STOP = ":slot1:storager:cpu", 45.0
local cpu, sp, armed = nil, nil, false
local taps, hits, pending = {}, {}, {}
local builds, build = {}, nil
local nodebase = 0x71f0
local livecmd = 0

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function B(a) return sp:read_u8(a & 0xffff) end
local function W(a) return sp:read_u16(a & 0xffff) end
local function R(n) local ok,v = pcall(function() return cpu.state[n].value end); return ok and v or 0 end

local function arm_taps()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  -- node+$26 <- $000C is the op00 stamp.  Capture A1 (a REGISTER read - safe in a tap); the
  -- surrounding words are read later, in the periodic, never inside the tap.
  taps[#taps+1] = sp:install_write_tap(0x7100, 0x72ff, "op00", function(off, data, mask)
    if (off & 0xffff) ~= ((nodebase + 0x26) & 0xffff) then return end
    if mask ~= 0xffff or (data & 0xffff) ~= 0x000c then return end
    pending[#pending+1] = { t = now(), a1 = R("A1") & 0xffff, a0 = R("A0") & 0xffff,
                            a6 = R("A6") & 0xffff, cmd = livecmd, chan = W(0x721a) }
  end)
  -- BUILD side: the builder writes the list through A6 with move.w #op,(a6)+.  A write tap on the
  -- ladder buffers ($722C and $7250) catches every emitted op, so the build BASE and the built
  -- CONTENT are both observed without an opcode tap.
  taps[#taps+1] = sp:install_write_tap(0x7220, 0x7270, "build", function(off, data, mask)
    local a = off & 0xffff
    if not build or build.cmd ~= livecmd then
      build = { cmd = livecmd, base = a, t = now(), ops = {} }
      builds[#builds+1] = build
    end
    if #build.ops < 14 then build.ops[#build.ops+1] = string.format("%04x", data & 0xffff) end
    build.last = a
  end)
  print("op00 armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm_taps() then return end
  local node = W(0x71bc)
  if node >= 0x4000 and node < 0x8000 then nodebase = node end
  livecmd = B(nodebase)
  -- expand any pending captures now that we are outside the tap
  while #pending > 0 do
    local p = table.remove(pending, 1)
    local ctx = {}
    for i = -8, 8, 2 do
      ctx[#ctx+1] = string.format("%s%04x%s",
        (i == 0) and "[" or "", W((p.a1 + i) & 0xffff), (i == 0) and "]" or "")
    end
    hits[#hits+1] = string.format("%9.4f  cmd=%02x  WALK A1=%04x A0=%04x [721a]=%04x  list: %s",
      p.t, p.cmd, p.a1, p.a0, p.chan, table.concat(ctx, " "))
  end
  if now() >= STOP then
    print("=== where the walker found its terminating zero ===")
    print("  builder emits: 24 28 56 58 [1A] 18 54 4A 42 36 00")
    for _, l in ipairs(hits) do print(l) end
    print(string.format("=== %d op00 stamps ===", #hits))
    print("=== BUILD side: base and content per command ===")
    for _, b in ipairs(builds) do
      print(string.format("%9.4f  cmd=%02x  BUILD base=%04x..%04x  emitted: %s",
        b.t, b.cmd, b.base, b.last or b.base, table.concat(b.ops, " ")))
    end
    io.flush(); manager.machine:exit()
  end
end)
