-- stopfork.lua - what ENDS the re-arm loop, and does it ever fire on the failing reads?
--
-- The count cannot be the stop: $7EBE subq.w #1,$7956 decrements past zero and $7EC2 bne $7ee0 is
-- then satisfied on every subsequent record.  So the stop is exactly one of:
--   $7ED8 via the $AA boundary test at $7ED0        -> [$741c] <- 0
--   $7EF4 via $32AC allocation failure ($7EEE)      -> also [$741c] <- 0  (shares the exit)
-- and $7F14 (allocation OK) writes [$741c] <- 1.  All three are DATA writes, so no opcode taps: the
-- three addresses are branch targets/fall-throughs and their opcode taps fired spuriously earlier.
-- The two [$741c]<-0 paths are separated by the allocator's free-list head [$74ac]: exhausted = 0.
--
-- PREDICTION under test (Static): on the failing reads the ledger stays largely UNSTAKED despite
-- hundreds of arms, and the fork resolves to NEITHER exit - the walk never reaches $AA and the run
-- ends only when something outside this loop intervenes.  If it resolves to $7EF4 instead, the story
-- is allocator exhaustion and the ledger is fine.
-- In-run cheap check that generalises: ARMS-PER-MARK > 1 is the failing signature (measured 1.18 /
-- 1.19 on failures, 0.73 on the good read) and needs no knowledge of the outcome.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle -oslog \
--   -autoboot_script docs/storager-lle/stopfork.lua -flop siemens/set1/mx2-001.imd -hard1 <img>

local SC, STOP = ":slot1:storager:cpu", 45.0
local cpu, sp, armed = nil, nil, false
local taps, rows, cur = {}, {}, nil
local lastcmd, laststat = -1, -1
local nodebase = 0x71f0    -- cached in the periodic; NEVER read SRAM inside a tap

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function B(a) return sp:read_u8(a & 0xffff) end
local function W(a) return sp:read_u16(a & 0xffff) end

local function staked()
  -- ledger $7654..: positive index = live, c0 = consumed, ff = skip, fe = terminator, aa = boundary
  local n, aa, fe, ff = 0, 0, 0, 0
  for i = 0, 63 do
    local v = B(0x7654 + i)
    if v == 0xc0 then n = n + 1
    elseif v == 0xaa then aa = aa + 1
    elseif v == 0xfe then fe = fe + 1
    elseif v == 0xff then ff = ff + 1 end
  end
  return n, aa, fe, ff
end

local function arm_taps()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true; cpu = d; sp = d.spaces["program"]
  taps[#taps+1] = sp:install_write_tap(0x741c, 0x741d, "f41c", function(off, data, mask)
    if not cur then return end
    if (data & 0xffff) == 0 then cur.stop = cur.stop + 1 else cur.alloc_ok = cur.alloc_ok + 1 end
  end)
  for _, base in ipairs({ 0x00c800, 0xffc800 }) do
    taps[#taps+1] = sp:install_write_tap(base, base + 0x1ff, "c800", function()
      if cur then cur.c800 = cur.c800 + 1 end
    end)
  end
  taps[#taps+1] = sp:install_write_tap(0x7dac, 0x7db1, "mark", function()
    if cur then cur.marks = cur.marks + 1 end
  end)
  -- completion chain: if the inversion holds, THIS is where the two classes differ, not the stop fork
  -- node+$26 is written by op36 ($000A) and op00 ($000C); each state lasts ONE walker pass, so a
  -- polling sample cannot see it - it must be a write tap.  Base from [$71bc], cached.
  taps[#taps+1] = sp:install_write_tap(0x7100, 0x72ff, "phase", function(off, data, mask)
    if not cur then return end
    if (off & 0xffff) ~= ((nodebase + 0x26) & 0xffff) then return end
    local v = data & 0xffff
    if mask == 0xffff and v == 0x000a then cur.op36 = cur.op36 + 1 end   -- op42 RETURNED 0
    if mask == 0xffff and v == 0x000c then cur.op00 = cur.op00 + 1 end   -- the terminator
  end)
  taps[#taps+1] = sp:install_read_tap(0x7970, 0x7971, "cb7964", function()
    if cur then cur.cb7964 = cur.cb7964 + 1 end   -- inside $7964, past its entry word
  end)
  print("stopfork armed"); io.flush(); return true
end

emu.register_periodic(function()
  if not arm_taps() then return end
  local node = W(0x71bc)
  if node < 0x4000 or node >= 0x8000 then node = 0x71f0 end
  nodebase = node
  local cmd, stat = B(node), B(node + 2)
  if cmd ~= lastcmd then
    if cmd == 0x95 then cur = { t0 = now(), c800 = 0, marks = 0, stop = 0, alloc_ok = 0, op36 = 0, op00 = 0, cb7964 = 0 }
    else cur = nil end
    lastcmd = cmd
  end
  if cur and stat ~= laststat and (stat == 0x82 or stat == 0x80) then
    cur.t1 = now(); cur.ok = (stat == 0x80)
    cur.staked, cur.aa, cur.fe, cur.ff = staked()
    cur.freelist = W(0x74ac)
    cur.f41c_end = W(0x741c)   -- CLOSES the claim: 1 at exit = the firmware never terminated the loop
    rows[#rows+1] = cur; cur = nil
  end
  laststat = stat
  if now() >= STOP then
    print("=== what ends the re-arm loop ===")
    print("   #  result  elapsed   C800  marks  [741c]END  op36  op00  $7964  staked  verdict")
    for i, r in ipairs(rows) do
      print(string.format("  %2d  %-5s %7.3fs %6d %6d  %6d  %4d  %4d  %5d  %5d   %s",
        i, r.ok and "0x80" or "FAIL", r.t1 - r.t0, r.c800, r.marks,
        r.f41c_end or -1, r.op36, r.op00, r.cb7964, r.staked,
        (r.op36 > 0) and "op42 RETURNED 0 - walker reached its terminator"
                      or "op42 NEVER returned 0 - walker never terminated"))
    end
    print("  INVERSION TEST: [741c] END = 1 means the firmware never terminated the re-arm loop,")
    print("  so a good read stops because the NEXT COMMAND closed the window, not because it decided to.")
    print("  freelist 0000 at exit = allocator exhausted ($7EF4), non-zero = the $AA boundary ($7ED8).")
    io.flush(); manager.machine:exit()
  end
end)
