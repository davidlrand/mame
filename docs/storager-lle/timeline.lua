-- timeline.lua - ONE instrumented run of the Storager read, whole chain, one clock.
--
-- WHY THIS EXISTS: findings assembled from separate runs are not comparable.  The read's arm
-- time is RTC-seeded and moves by seconds between runs, so a fact measured in a run that armed
-- at 6.41s may simply not describe a run that armed at 2.92s.  Several conclusions had to be
-- retracted for exactly that reason.  This captures every link of the completion chain in a
-- single ordered log so a change can be judged against one timeline.
--
-- MEASUREMENT RULES BAKED IN (see the storager-measurement-discipline memory):
--   * tap handles are RETAINED - Lua GC silently removes them otherwise (0 vs 31844).
--   * WRITE taps wherever possible - they have no prefetch semantics.
--   * opcode taps are placed DEEP (>=6 bytes past any branch), never on a routine's first
--     instruction: the 68000 prefetches past branches and rts, faking execution.
--   * paired entry/deep taps where "did it run?" matters.
--   * no time gating - everything is keyed on cmd==0x95, so a late arm cannot hide the event.
--
-- USAGE: SDL_VIDEODRIVER=dummy CPUAP_RTCFIX=1 ./mame pcmx2 -video none -nothrottle \
--          -autoboot_script docs/storager-lle/timeline.lua -flop siemens/set1/mx2-001.imd

local SC   = ":slot1:storager:cpu"
local STOP = 30.0          -- emulated seconds
local MAXEV = 400

local cpu, sp, osp, armed = nil, nil, nil, false
local taps, ev, cnt = {}, {}, {}
local last_state = nil

local function now() local ok,t = pcall(function() return manager.machine.time:as_double() end); return ok and t or 0 end
local function W(a) return sp:read_u16(a & 0xffff) end
local function B(a) return sp:read_u8(a & 0xffff) end
local function PC() local ok,v = pcall(function() return cpu.state["PC"].value end); return (ok and v or 0) & 0xffffff end
local function REG(n) local ok,v = pcall(function() return cpu.state[n].value end); return (ok and v or 0) & 0xffff end
local function reading() return B(0x71f0) == 0x95 end

local opseq, opcnt, lastop = {}, {}, -1   -- ladder trace: sequence of op TRANSITIONS + per-op counts

local PERTAG = 12
-- probes that only matter LATE need a higher cap than the hot early ones
local PERTAG_HI = { ["3F68.reached"] = 40, ["rec1.flags"] = 40, ["72d8.write"] = 40, ["cb.$7964"] = 40 }          -- max logged lines per tag; counts are still exact
local function EV(tag, detail)
  cnt[tag] = (cnt[tag] or 0) + 1
  if cnt[tag] > (PERTAG_HI[tag] or PERTAG) then return end          -- hot tags (pump.dispatch) must not flood
  if #ev < MAXEV then
    ev[#ev+1] = string.format("%9.4f  %-22s %s", now(), tag, detail or "")
    if cnt[tag] == (PERTAG_HI[tag] or PERTAG) then
      ev[#ev+1] = string.format("%9.4f  %-22s ... further occurrences counted only", now(), tag)
    end
  end
end

-- opcode tap, DEEP placement enforced by the caller supplying the address to use
local function otap(addr, tag, fn)
  taps[#taps+1] = osp:install_read_tap(addr, addr+1, tag, function()
    if not reading() then return end
    if fn then fn() else EV(tag, "") end
  end)
end
local function wtap(lo, hi, tag, fn)
  taps[#taps+1] = sp:install_write_tap(lo, hi, tag, function(off, data, mask)
    if not reading() then return end
    fn(off, data, mask)
  end)
end

local function setup()
  if armed then return true end
  local ok, d = pcall(function() return manager.machine.devices[SC] end)
  if not ok or not d then return false end
  armed = true
  cpu, sp = d, d.spaces["program"]
  osp = d.spaces["decrypted_opcodes"] or d.spaces["opcodes"] or sp

  ---------------------------------------------------------------- record capture / staking
  otap(0x8092, "8018.gate", function()                    -- 8 bytes past the $808A branch target
    EV("8018.gate", string.format("[7426]=%04x -> %s  slot[7424]=%04x pos[7428]=%04x",
      REG("D0"), (REG("D0")==0) and "LEDGER WRITE" or "exit $8214", W(0x7424), W(0x7428)))
  end)

  wtap(0x7654, 0x7663, "ledger", function(off, data, mask)
    local pc = PC()
    -- skip the $125E/$1280 command-start teardown loop, which fills the cap with 20 c0's and hides
    -- the per-record stakes we actually want to time against $7EBE.
    -- only the per-record STAKE sites ($8120 c0 / $8128 INDEX); the $125E teardown and the drain's
    -- own $6FE6/$7068 fills are not what we are timing against $7EBE.
    if not (pc >= 0x8118 and pc <= 0x8132) then return end
    local v = (mask == 0x00ff) and (data & 0xff) or ((data >> 8) & 0xff)
    local a = (mask == 0x00ff) and (off | 1) or (off & ~1)
    local src = (v == 0xc0) and "c0 STAKE" or ((v == 0xff) and "ff(drain skip)" or string.format("%02x INDEX", v))
    EV("ledger", string.format("[%2d] <= %-14s pc=%06x", (a & 0xffff) - 0x7654, src, pc))
  end)

  wtap(0x742c, 0x742d, "742C.selector", function(off, data, mask)
    EV("742C.selector", string.format("<= %04x pc=%06x   (1 => $8120 c0 arm, 0 => $8128 INDEX arm)",
      data & 0xffff, PC()))
  end)

  ---------------------------------------------------------------- the OTHER $AA test ($7E9A)
  -- $7E8E forks on [$7968]: ==0 -> $7EB2 (accept/decrement, marker tested once at $7ED0);
  -- !=0 -> $7E90 -> $7E9A cmpi.b #$aa,(A0,D0.w), tested on EVERY record with no count precondition.
  -- Tap at $7EA0 (the beq, 6 bytes past the compare, and 18 past the $7E8E branch - clear of any
  -- prefetch shadow) with A0/D0 still live: this names the cell that arm actually reads.
  otap(0x7ea0, "7E9A.cell", function()
    local a0, d0 = REG("A0"), REG("D0")
    local cell = (a0 + d0) & 0xffff
    EV("7E9A.cell", string.format("A0=%04x D0=%04x -> cell [%d] @%04x = %02x  %s",
      a0, d0, cell - 0x7654, cell, B(cell),
      (B(cell) == 0xaa) and "*** IS $AA -> $7ED8 TERMINATE ***" or "not $AA -> $7EE0"))
  end)
  otap(0x7d4e, "7D4A.reject", function()
    EV("7D4A.reject", string.format("rejected-record leg ([796e]=%04x)", W(0x796e)))
  end)

  ---------------------------------------------------------------- $7AC2: the bootstrap strobe
  -- $82B2 sets [$7968]; $82B8 immediately does bset #0,$7ac2, which nothing in the ROM ever reads.
  -- If that strobe precedes the remain->0 edge it is a usable "the firmware has bootstrapped" signal
  -- for the gate array; if it follows it, no deposit keyed on it can be early enough.
  wtap(0x7ac2, 0x7ac3, "7ac2.strobe", function(off, data, mask)
    EV("7ac2.strobe", string.format("<= %04x pc=%06x  [7968]=%04x [7956]=%04x", data & 0xffff, PC(),
      W(0x7968), W(0x7956)))
  end)

  ---------------------------------------------------------------- WHICH CELL does $7ED0 test?
  -- $7ED0 cmpi.b #$aa,(A0,D0.w) is reached only on the remain->0 pass.  Tap at $7ED6 (8 bytes past
  -- the $7ECE branch, clear of the prefetch shadow) with A0/D0 still live: that names the exact
  -- ledger cell the end marker has to occupy, and when.
  otap(0x7ed6, "7ED0.cell", function()
    local a0 = REG("A0")
    local d0 = REG("D0")
    local cell = (a0 + d0) & 0xffff
    EV("7ED0.cell", string.format("A0=%04x D0=%04x -> cell [%d] @%04x = %02x  %s  [7956]=%04x",
      a0, d0, cell - 0x7654, cell, B(cell), (B(cell) == 0xaa) and "*** IS $AA - would terminate ***" or "not $AA -> re-arm",
      W(0x7956)))
  end)

  ---------------------------------------------------------------- stake vs accept ORDERING
  -- The $AA end-marker must be in the scan's landing cell at the instant $7EBE takes remain to 0
  -- ($7ED0 tests it one instruction later).  So: does the Nth $c0 stake ($8120) land BEFORE the Nth
  -- $7EBE?  If yes, the stake is a usable plant point; if no, the marker can never be early enough.
  wtap(0x7956, 0x7957, "7956.remain", function(off, data, mask)
    EV("7956.remain", string.format("<= %04x pc=%06x %s", data & 0xffff, PC(),
      (PC() >= 0x7ebe and PC() <= 0x7ec6) and "*** $7EBE decrement ***" or ""))
  end)

  ---------------------------------------------------------------- the ledger drain
  otap(0x710c, "drain.arm.full", function() EV("drain.arm.full", "[7968]==0 -> $6F44") end)
  otap(0x7116, "drain.arm.d3walk", function() EV("drain.arm.d3walk", "[7968]!=0 -> $7114") end)
  otap(0x6f50, "drain.enter", function()                  -- 12 bytes into $6F44
    EV("drain.enter", string.format("base[7954]=%04x count[7abc]=%04x", W(0x7954), W(0x7abc)))
  end)
  otap(0x70a0, "drain.sub", function()
    local d3 = REG("D3")
    EV("drain.sub", string.format("D3=%d [7956]=%04x -> %04x  %s", d3, W(0x7956),
      (W(0x7956) - d3) & 0xffff, ((W(0x7956) - d3) & 0xffff) == 0 and "ZERO (completes)" or "NON-ZERO"))
  end)

  ---------------------------------------------------------------- the restart kick
  wtap(0x7a30, 0x7a31, "kick.flag", function(off, data, mask)
    local sz = (mask == 0xffff) and "WORD" or ((mask == 0xff00) and "byte-hi" or "byte-lo")
    EV("kick.flag", string.format("<= %04x %-7s pc=%06x  hi-byte now %02x %s",
      data & 0xffff, sz, PC(), (sz == "WORD") and ((data >> 8) & 0xff) or B(0x7a30),
      (sz == "WORD") and "*** CLOBBERS THE KICK ***" or ""))
  end)

  ---------------------------------------------------------------- node phase + the deposit
  wtap(0x7216, 0x7217, "node+26", function(off, data, mask)
    local sz = (mask == 0xffff) and "WORD" or "BYTE"
    EV("node+26", string.format("<= %04x %s pc=%06x  (byte@+26 now %02x)",
      data & 0xffff, sz, PC(),
      (mask == 0x00ff) and (data & 0xff) or ((data >> 8) & 0xff)))
  end)

  ---------------------------------------------------------------- the settle timer
  -- op42's FM exit is NOT a gate-array write: it waits for [$7a40]->0 and [$7a3e]!=0, both driven
  -- by the $29F8 deferred-write scheduler armed at $6C64 with the delay from UIB+$18.
  wtap(0x7a40, 0x7a41, "7a40.timer", function(off, data, mask)
    EV("7a40.timer", string.format("<= %04x pc=%06x", data & 0xffff, PC()))
  end)
  wtap(0x7a3e, 0x7a3f, "7a3e.expiry", function(off, data, mask)
    EV("7a3e.expiry", string.format("<= %04x pc=%06x  %s", data & 0xffff, PC(),
      ((data & 0xffff) ~= 0) and "*** settle EXPIRED - op42 can advance ***" or ""))
  end)
  otap(0x6c64, "6C64.arm", function()
    local uib = W(0x799a)
    EV("6C64.arm", string.format("UIB[799a]=%04x UIB+18=%02x (the settle delay) D0=%04x",
      uib, B((uib + 0x18) & 0xffff), REG("D0")))
  end)

  ---------------------------------------------------------------- the DESCRIPTOR QUEUE [$74b4]
  -- $81F8/$820A enqueue a descriptor from the per-record ISR - independent of the drain.  The c0 arm
  -- ($8118 (A0)!=0) is the SAME condition that routes to $81B4, so our records should be enqueuing.
  -- [$74b4] != 0 is what $731E/$70EA/$3DE0 test to reach $3DBC's launch leg -> $4062 -> $4122 -> $414C.
  wtap(0x74b4, 0x74b5, "74b4.queue", function(off, data, mask)
    EV("74b4.queue", string.format("<= %04x pc=%06x  %s", data & 0xffff, PC(),
      ((data & 0xffff) ~= 0) and "*** QUEUE NON-EMPTY ***" or "cleared"))
  end)
  otap(0x7326, "731E.test", function()
    EV("731E.test", string.format("[74b4]=%04x -> bsr $3DBC (SR $2400, un-bank leg)", W(0x74b4)))
  end)
  otap(0x3e58, "3DBC.nonempty", function()
    EV("3DBC.nonempty", string.format("*** non-empty leg $3E50 *** [71b2]=%04x [7b0c]=%04x",
      W(0x71b2), W(0x7b0c)))
  end)
  otap(0x4066, "4062.launch", function() EV("4062.launch", "*** reached $4062 - heading for $4122/$414C ***") end)
  wtap(0x743a, 0x743b, "743a.desc", function(off, data, mask)
    EV("743a.desc", string.format("<= %04x pc=%06x  %s", data & 0xffff, PC(),
      ((data & 0xffff) == 0x7442) and "*** $414C RAN - the read's descriptor is live ***" or ""))
  end)

  ---------------------------------------------------------------- host-visible STATUS + $748A's cb
  -- The host polls IOPB+2, which the channel fills from node+3 (the 68000's BE low byte).  Watch the
  -- node's status byte directly: 0x81 = busy, 0x80 = good completion.
  wtap(0x71f2, 0x71f3, "status", function(off, data, mask)
    local sz = (mask == 0xffff) and "WORD" or ((mask == 0xff00) and "byte-hi" or "byte-lo")
    EV("status", string.format("node+2/3 <= %04x %s pc=%06x  %s", data & 0xffff, sz, PC(),
      ((mask == 0x00ff and (data & 0xff) == 0x80) or (mask == 0xffff and (data & 0xff) == 0x80))
        and "*** 0x80 GOOD COMPLETION ***" or ""))
  end)
  -- Q2 dependency: without the preserved kick the first $3DBC never runs, so the launch never happens.
  -- The one other route is the IRQ4 descriptor dispatch - but that reads ($14,[$743a]), and [$743a] is
  -- $748A (the idle descriptor) until $414C.  If $748A+$14 holds a callback, IRQ4 can bootstrap
  -- unaided; if it is zero, the kick really is the only way in.
  otap(0x3c72, "748A.cb", function()
    local d = W(0x743a)
    EV("748A.cb", string.format("[743a]=%04x -> cb at (%04x+14)=%04x%04x  %s",
      d, d, W((d + 0x14) & 0xffff), W((d + 0x16) & 0xffff),
      (W((d + 0x14) & 0xffff) ~= 0 or W((d + 0x16) & 0xffff) ~= 0) and "NON-NULL - dispatches" or "NULL - $3C76 leg"))
  end)

  ---------------------------------------------------------------- IRQ4 descriptor dispatch
  -- $3BFE (IRQ4) forks at $3C1A on [$743a] vs [$7a14].  The NOT-equal leg reaches $3C6E, which
  -- dispatches the callback stored at descriptor+$14 = $7456 - set to $3F68 by the read's launch
  -- at $604C.  $3F68 -> $3FD0 -> $3FFC -> $400C (re-arm kick) or $4002 (un-bank rec1).
  -- $3C12 is 20 bytes into the handler, clear of any shadow.
  otap(0x3c12, "IRQ4.enter", function()
    local d = W(0x743a)
    EV("IRQ4.enter", string.format("[743a]=%04x [7a14]=%04x -> %s | cb long[7456]=%04x%04x",
      d, W(0x7a14), (d ~= W(0x7a14)) and "$3C32 leg (dispatch)" or "$3C1C leg (no dispatch)",
      W(0x7456), W(0x7458)))
  end)
  otap(0x3c3e, "IRQ4.3C32", function() EV("IRQ4.3C32", "took the dispatch leg") end)
  otap(0x3c9a, "IRQ4.jsr", function()
    EV("IRQ4.jsr", string.format("*** dispatched callback via ($14,A0) ***"))
  end)
  otap(0x3fda, "3F68.reached", function()
    EV("3F68.reached", string.format("[7b48]=%04x [74b4]=%04x [7a64]=%04x -> %s",
      W(0x7b48), W(0x74b4), W(0x7a64),
      (W(0x7b48) ~= 0) and "$3FE8" or ((W(0x74b4) ~= 0) and "$3FF0" or "$3FFC *** the un-bank/re-arm fork ***")))
  end)

  ---------------------------------------------------------------- op42's inputs
  -- $6BC2 is entered via `jsr (A1)' from the walker - a real fetch, no prefetch shadow.
  -- Log the first few entries, then only when a control input CHANGES, so the transition is visible.
  local last42 = nil
  otap(0x6bc2, "op42.in", function()
    local node = W(0x799a)
    local n12  = B((node + 0x12) & 0xffff)
    local key = string.format("%04x|%02x|%04x|%04x|%04x", W(0x798e), n12, W(0x7a36), W(0x7a3e), W(0x7a40))
    if key ~= last42 then
      last42 = key
      EV("op42.in", string.format(
        "UIB[799a]=%04x node[71bc]=%04x %s | [798e]=%04x n+12=%02x(bit1=%d bit7=%d) [7a36]=%04x [7a3e]=%04x [7a40]=%04x n+18=%04x F000=%04x -> %s",
        node, W(0x71bc), (node == W(0x71bc)) and "SAME" or "*** DIFFERENT ***",
        W(0x798e), n12, (n12 >> 1) & 1, (n12 >> 7) & 1, W(0x7a36), W(0x7a3e), W(0x7a40),
        W((W(0x71bc) + 0x18) & 0xffff), W(0xf000),
        (W(0x798e) & 0x8000) ~= 0 and "*** blt $6CDA - ADVANCE ***" or
        (((n12 >> 1) & 1) == 1 and "MFM path $6BE8" or
         (W(0x798e) == 0 and "FM: [798e]==0 -> $6C28 -> WAIT $FE" or "FM: [798e]!=0 -> $6C32 settle path"))))
    end
  end)
  -- op28 is what SETS [$798e]; catch every write with its PC so the setter is named.
  wtap(0x798e, 0x798f, "798e.write", function(off, data, mask)
    EV("798e.write", string.format("<= %04x pc=%06x  %s", data & 0xffff, PC(),
      ((data & 0x8000) ~= 0) and "*** NEGATIVE - op42 advances ***" or
      ((data & 0xffff) == 0) and "zero - op42 WAITs" or "positive - settle path"))
  end)

  ---------------------------------------------------------------- THE LADDER TRACE
  -- $15B0 lea $192,A0 / $15B4 move.w (A0,D0.w),D0 / $15BA jsr (A1).  D0 at $15B4 is the byte
  -- offset into the $192 jump table, so the opcode and its handler are both recoverable.
  -- $15B4 is 8 bytes into the routine - clear of any prefetch shadow.
  -- op42 retries ~31.8k times, so log only TRANSITIONS; count everything.
  otap(0x15b4, "ladder", function()
    local off = REG("D0") & 0xff
    local h = W(0x192 + off)
    opcnt[off] = (opcnt[off] or 0) + 1
    if off ~= lastop then
      lastop = off
      if #opseq < 120 then
        opseq[#opseq+1] = string.format("%9.4f  op $%02x -> $%04x %s", now(), off // 2, h,
          (h == 0x6ed2) and "*** op4A DRAIN REQUEST ***" or
          (h == 0x6bc2) and "op42 (guard/wait)" or
          (h == 0xa356) and "op54 (watch-table copy)" or
          (h == 0x8f74) and "op3E (transfer launch)" or "")
      end
    end
  end)

  ---------------------------------------------------------------- [$7968]: the drain DEFERRAL gate
  -- op4A ($6ED6) raises [$7b10] then forks on [$7968]:  ==0 -> drain INLINE now (too early);
  -- !=0 -> leave the request set so the ISR drains AFTER the sector interrupts.
  -- No ROM instruction sets it non-zero, yet it reads 0001 at end of run - find the writer.
  wtap(0x7968, 0x7969, "7968.write", function(off, data, mask)
    local pc = PC()
    EV("7968.write", string.format("<= %04x %s pc=%06x  %s", data & 0xffff,
      (mask == 0xffff) and "WORD" or "BYTE", pc,
      ((data & 0xffff) ~= 0) and "*** SET NON-ZERO - would DEFER the drain ***" or "clear"))
  end)

  ---------------------------------------------------------------- the drain-request flag
  wtap(0x7b10, 0x7b11, "7b10.req", function(off, data, mask)
    local pc = PC()
    EV("7b10.req", string.format("<= %04x pc=%06x  %s", data & 0xffff, pc,
      (pc >= 0x6ed6 and pc <= 0x6ee0) and (function()
        -- op4A's fork inputs, read at the moment it decides inline-vs-deferred
        EV("op4A.fork", string.format("[7968]=%04x %s | gates for $82B2: [796a]=%04x [741c]=%04x | [742C]=%04x",
          W(0x7968), (W(0x7968) == 0) and "== 0 -> INLINE drain (empty ledger, fails)" or "!= 0 -> DEFER to ISR",
          W(0x796a), W(0x741c), W(0x742c)))
        -- m68000 exposes the stack pointer as "SP"; verify it is a live SRAM address before
        -- dereferencing, or we end up reading the reset vector at $0000 and calling it a caller.
        local ok, sv = pcall(function() return cpu.state["SP"].value end)
        local a7 = ok and (sv & 0xffffff) or 0
        if a7 < 0x4000 or a7 >= 0x8000 then
          return string.format("*** $6ED6 REQUEST - SP UNREADABLE (SP=%06x) ***", a7)
        end
        return string.format("*** $6ED6 REQUEST - returns to %04x%04x (SP=%04x) ***",
          W(a7), W(a7 + 2), a7)
      end)() or "clear"))
  end)
  -- entry accounting for the drain: $6F44 body vs the $7106 arm
  otap(0x6f4a, "6F44.body", function() EV("6F44.body", "drain body entered") end)
  otap(0x6f40, "6F3A.fall", function() EV("6F3A.fall", "$6F3A -> falls toward $6F44") end)

  ---------------------------------------------------------------- op42's return: the kick gate
  -- $9492 bsr $6BC2 (op42); D0==0 -> $94A0 arms the kick that lets the ISR run $3DBC.
  -- Tap at $9498 (6 bytes past the $9492 branch target), D0 still live.
  otap(0x9498, "op42.ret", function()
    local d0 = REG("D0") & 0xff
    EV("op42.ret", string.format("D0=%02x -> %s | [798e]=%04x [7a64]=%04x [791a]=%04x",
      d0, (d0 == 0) and "*** $94A0 - ARM THE KICK ***" or
          ((d0 < 0x80) and "$94EA exit (>0)" or "$949C exit (<0, WAIT)"),
      W(0x798e), W(0x7a64), W(0x791a)))
  end)
  otap(0x94ae, "94A0.kickpath", function()
    EV("94A0.kickpath", string.format("reached - n+20=%04x [791a]=%04x", W(W(0x799a) + 0x20), W(0x791a)))
  end)

  ---------------------------------------------------------------- [$79a0]: the record-path selector
  -- The drain computes it from UIB+$11 bit1 ($6F68/$6F6C/$6F70).  UIB+$11 = $97, so bit1 IS set and
  -- [$79a0] should be 2 -> records take $7D1A, not $7E50.  It measures 0: find the clearer.
  wtap(0x79a0, 0x79a1, "79a0.write", function(off, data, mask)
    local pc = PC()
    EV("79a0.write", string.format("<= %04x pc=%06x  %s", data & 0xffff, pc,
      (pc >= 0x6f70 and pc <= 0x6f78) and "*** $6F70 - from UIB+$11 bit1 ***" or
      (pc >= 0x70cc and pc <= 0x70d4) and "$70CC clear (drain tail)" or
      (pc >= 0x70fa and pc <= 0x7102) and "$70FA clear" or
      (pc >= 0x82d6 and pc <= 0x82de) and "$82D6 clear (IRQ5 chain)" or
      (pc >= 0x7232 and pc <= 0x723a) and "$7232 clear" or
      (pc >= 0x373e and pc <= 0x3746) and "$373E clear" or "?"))
  end)

  ---------------------------------------------------------------- [$7a64]: the rec1 unpark enable
  -- Set at $70A6 (drain success) AND at $825C (IRQ5 chain, when [$7956] hits 0).  $3FFC tests it:
  -- non-zero -> $4002 unparks rec1 -> pump dispatches $3DBC -> $3E1C clears $72D8, the last gate.
  wtap(0x7a64, 0x7a65, "7a64.write", function(off, data, mask)
    local pc = PC()
    EV("7a64.write", string.format("<= %04x pc=%06x  %s", data & 0xffff, pc,
      (pc >= 0x8262 and pc <= 0x8268) and "*** $825C - IRQ5 chain, [7956] hit 0 ***" or
      (pc >= 0x70ac and pc <= 0x70b4) and "$70A6 drain success" or
      (pc >= 0x7208 and pc <= 0x7212) and "$7208" or ""))
  end)
  otap(0x4002, "3FFC.unpark", function()
    EV("3FFC.unpark", string.format("[7a64]=%04x -> unparking rec1 via [72d8]=%04x", W(0x7a64), W(0x72d8)))
  end)
  otap(0x8268, "825C.done", function()
    EV("825C.done", string.format("[7956]=%04x [7a64]=%04x [7958]=%08x", W(0x7956), W(0x7a64),
      (W(0x7958) << 16) | W(0x795a)))
  end)

  ---------------------------------------------------------------- rec2: the $72D8 clearer
  -- op54's last act ($A388) patches rec2's WATCH ADDRESS to [$71be]; expected = $0000.  The pump
  -- fires rec2 on MISMATCH, running $9188, which unconditionally clears $72D8/$72D6/$72DA ($91EE).
  -- So the last gate opens when the word at [$71be] becomes NON-ZERO.
  wtap(0x7290, 0x7291, "rec2.watchaddr", function(off, data, mask)
    EV("rec2.watchaddr", string.format("<= %04x pc=%06x  (rec2 now watches that address)", data & 0xffff, PC()))
  end)
  otap(0x89fa, "probe.71be", function()
    local w = W(0x71be)
    EV("probe.71be", string.format("[71be]=%04x  *(that)=%04x  %s  | rec2 flags[728e]=%04x watch[7290]=%04x exp[7292]=%04x",
      w, (w >= 0x4000 and w < 0x8000) and W(w) or 0xdead,
      ((w >= 0x4000 and w < 0x8000) and W(w) or 0) ~= 0 and "*** NON-ZERO - rec2 WOULD FIRE ***" or "zero - rec2 idle",
      W(0x728e), W(0x7290), W(0x7292)))
  end)

  ---------------------------------------------------------------- the $72D8 teardown gate
  -- $72D8 is SEEDED to $7286 by op54's bulk copy and must be CLEARED before $7A7C will let the
  -- completion post.  A write tap names every writer with its PC - no prefetch exposure at all.
  wtap(0x72d8, 0x72d9, "72d8.write", function(off, data, mask)
    local pc = PC()
    local who = (pc >= 0xa374 and pc <= 0xa38a) and "op54 bulk copy (SEED)"
             or (pc >= 0x37a4 and pc <= 0x37b2) and "$37A4 re-seed"
             or (pc >= 0x3976 and pc <= 0x3984) and "$3976 re-seed"
             or (pc >= 0x4396 and pc <= 0x43a0) and "$4396 CLEAR"
             or (pc >= 0x3e1c and pc <= 0x3e24) and "*** $3E1C CLEAR - gate opens ***"
             or "?"
    EV("72d8.write", string.format("<= %04x pc=%06x  %s", data & 0xffff, pc, who))
  end)
  wtap(0x7286, 0x7287, "rec1.flags", function(off, data, mask)
    EV("rec1.flags", string.format("<= %04x pc=%06x  %s", data & 0xffff, PC(),
      (data & 0xffff) == 0 and "*** UNPARKED (via $3E12) ***" or "parked"))
  end)
  otap(0x3dc6, "3DBC.enter", function()
    EV("3DBC.enter", string.format("[7462]=%04x (bne -> $3F54 abort)", W(0x7462)))
  end)
  otap(0x3dee, "3DBC.gates", function()
    local d2 = REG("D2")
    EV("3DBC.gates", string.format("[7a64]=%04x %s | [7454]=%04x | [7958]=%08x | D2 bit10=%d -> %s",
      W(0x7a64), (W(0x7a64) == 0) and "EXIT $3F5E" or "pass", W(0x7454),
      (W(0x7958) << 16) | W(0x795a), (d2 >> 10) & 1,
      ((d2 >> 10) & 1) == 1 and "$3E0E unpark" or "$3E1C CLEAR"))
  end)

  ---------------------------------------------------------------- the $7950 alternator
  -- The IRQ5/IRQ6 stubs do `bchg #0,$7950' and fork on the OLD bit.  The stub entries sit in
  -- prefetch shadow (each follows a `bra'), so instead: a WRITE tap gives the NEW value (old =
  -- NOT new), and DEEP taps in each fork say which one actually ran.
  wtap(0x7950, 0x7951, "alt.bchg", function(off, data, mask)
    local newb = data & 1
    EV("alt.bchg", string.format("<= %04x  old bit0 = %d -> %s  pc=%06x",
      data & 0xffff, 1 - newb, (newb == 1) and "was 0 (verify / setup fork)" or "was 1 (STAKE / done fork)", PC()))
  end)
  -- The six gates that decide whether $7950 STAYS set (arming $92B4/STAKE on the next IRQ6).
  -- Read at $89FA: 8 bytes into the handler, clear of the stub-beq prefetch shadow.
  otap(0x89fa, "irq6.verify $89F2", function()
    local node = W(0x799a)
    local s12, f20 = B(node + 0x12), W(node + 0x20)
    local m = {}
    for i = 0, 4 do m[i+1] = B(0x7dac + i) end
    local orsum = m[1] | m[2] | m[3]
    EV("irq6.verify $89F2", string.format(
      "g1 n+12=%02x bit1=%d | g2 n+20=%04x b14=%d | g3 mark=%02x %02x %02x -> or=%02x | g4=%02x g5=%02x | g6 [7a0c]=%04x  => %s",
      s12, (s12 >> 1) & 1, f20, (f20 >> 14) & 1, m[1], m[2], m[3], orsum, m[4], m[5], W(0x7a0c),
      ((s12 >> 1) & 1) == 0 and "FAIL g1" or
      (((f20 >> 14) & 1) == 1 and "FAIL g2" or
      (orsum ~= 0xa1 and "FAIL g3" or
      (m[4] ~= 0xfe and "FAIL g4" or
      (m[5] ~= 0xff and "FAIL g5" or "g1-g5 PASS - g6 decides"))))))
  end)
  wtap(0x7dac, 0x7db1, "markbuf", function(off, data, mask)
    EV("markbuf", string.format("[%d] <= %04x mask=%04x pc=%06x", (off & 0xffff) - 0x7dac, data & 0xffff, mask, PC()))
  end)
  wtap(0x7a0c, 0x7a0d, "7a0c.passctr", function(off, data, mask)
    EV("7a0c.passctr", string.format("<= %04x pc=%06x", data & 0xffff, PC()))
  end)
  otap(0x92be, "irq6.STAKE $92B4", nil)       -- 10 bytes in
  otap(0x8022, "irq5.done $8018", nil)        -- 10 bytes in
  otap(0x7bb0, "irq5.setup $7BA8", nil)       -- 8 bytes in

  ---------------------------------------------------------------- the watch pump
  otap(0x164c, "pump.dispatch", function() EV("pump.dispatch", "gate3 passed") end)
  otap(0x796c, "cb.$7964", function()
    EV("cb.$7964", string.format("[79b6]=%04x [79ba]=%04x [7956]=%04x [796c]=%04x",
      W(0x79b6), W(0x79ba), W(0x7956), W(0x796c)))
  end)
  otap(0x79ae, "cb.exit.796c=0", function() EV("cb.exit.796c=0", "took the [796c]!=0 leg") end)
  -- $7A4C is the SUCCESS teardown.  Tap at $7A72: 16 bytes past the $7A62 branch target, clear of
  -- the shadow, and before the two gates that decide whether the phase-0x0C write at $7A90 happens.
  otap(0x7a72, "cb.teardown $7A4C", function()
    local n = W(0x71bc)
    EV("cb.teardown $7A4C", string.format(
      "gateA [72d8]=%04x  gateB [791a]=%04x  node[71bc]=%04x n+26=%04x  => %s",
      W(0x72d8), W(0x791a), n, W((n + 0x26) & 0xffff),
      (W(0x72d8) ~= 0) and "SKIP (gateA set)" or
      ((W(0x791a) ~= 0) and "SKIP (gateB set)" or "*** $7A90 RUNS - phase 0x0C ***")))
  end)
  otap(0x7a96, "cb.phase0C", function()
    local n = W(0x71bc)
    EV("cb.phase0C", string.format("wrote #$c to [%04x]+26 = %04x", n, W((n + 0x26) & 0xffff)))
  end)

  otap(0x79ce, "cb.clr7968", function() EV("cb.clr7968", "*** [7968] CLEARED ***") end)
  otap(0x8442, "teardown.$843E", function() EV("teardown.$843E", "*** [72d6] cleared ***") end)

  print("timeline armed"); io.flush()
  return true
end

emu.register_periodic(function()
  local t = now()
  if not setup() then return end

  if reading() then
    local s = string.format("cmd=%02x st=%04x", B(0x71f0), W(0x71f2))
    if s ~= last_state then last_state = s; EV("STATE", s) end
  end

  if t >= STOP then
    print("=== STORAGER READ TIMELINE (one run, one clock) ===")
    for _, l in ipairs(ev) do print(l) end
    if #ev >= MAXEV then print(string.format("  ... TRUNCATED at %d events", MAXEV)) end
    print("=== LADDER: op transitions (cmd 95) ===")
    for _, l in ipairs(opseq) do print(l) end
    print("=== LADDER: per-op execution counts ===")
    local ok2 = {}; for a in pairs(opcnt) do ok2[#ok2+1] = a end; table.sort(ok2)
    for _, a in ipairs(ok2) do
      print(string.format("  op $%02x -> $%04x  %d", a // 2, W(0x192 + a), opcnt[a]))
    end
    print("=== counts ===")
    local k = {}; for a in pairs(cnt) do k[#k+1] = a end; table.sort(k)
    for _, a in ipairs(k) do print(string.format("  %-22s %d", a, cnt[a])) end
    print(string.format("=== final: cmd=%02x st=%04x  [7968]=%04x [72d6]=%04x [796c]=%04x [7a30]hi=%02x byte@7216=%02x",
      B(0x71f0), W(0x71f2), W(0x7968), W(0x72d6), W(0x796c), B(0x7a30), B(0x7216)))
    io.flush(); manager.machine:exit()
  end
end)
