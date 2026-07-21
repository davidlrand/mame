# Storager LLE conversion — design

> Reconciled 2026-07-11 onto the *autonomous gate-array* model after the read-handler
> disassembly. Supersedes the earlier "firmware parses the E000 byte stream" framing.

## Mandate (non-negotiable)
Every board is **low-level emulated**: the board's own firmware runs each transaction.
The emulator provides hardware-level support only; it must **never** take over for the
firmware with a C++ high-level shim (no env-var may change shipped behavior either). For the
storager: the MC68000 firmware runs every disk command end to end; the emulator models only
the **VGC7219 gate array** (74LS1801 ENDEC + 74LS1802 SERDES + support logic **intrinsic**).

## Architecture (confirmed)
- The 68000 talks **only to the VGC7219 gate array**; the 1801/1802 are internal. Bytes/status
  the SERDES would expose are **generated**, not real bit machinery. **Byte streams both sides**
  — no flux/bit cells. Floppy = decoded track (ID + data fields); ESDI = sector stream.
- **The firmware does NOT clock the channel for data.** It configures + arms the gate array and
  the gate array **autonomously** moves the bytes. Proven: with the HLE delivery gated off the
  boot hangs with *zero* firmware channel reads. The read handler (`$5de4`) does:
  1. config the channel: 16× `move #$23f,(A1)+` to `E000` (`$5ea0`), then **one** echo read
     `move $e000,D2` (`$5ea8`) — E000 is **config/status only, never a data stream**;
  2. **arm the gate-array disk→SRAM read**: `E800` bit7 @ `$5ede` → the gate array reads the
     track (ID+data) into SRAM `$4000`;
  3. **pace the fill by reading local SRAM** `$4000..$4fff` (4096× `move.b (A1)+,D0`, `$5ee2`) —
     *local* reads, which is what the `dma_snoop` tap tracks and how `F000 b12` (terminal) latches;
  4. parse SRAM for the wanted sector, then **fire the SRAM→host DMA** (`$13be-$13ce`).
- So there are **two gate-array transfers**: disk→SRAM (the JIT track) and SRAM→host (the DMA to
  the firmware-programmed address). `build_serdes_stream` filling SRAM `$4000` on the E800 arm is
  already the right shape for the first; the second is the keystone we must build.
- **JIT positioning**: serve the *correct* (cyl, head, sector) bytes on demand — head is always
  "there". Position = cylinder (step count) + head (head-select). Seek/rotational latency =
  later refinement (mostly perf).
- **⚠ Transfer DURATION is keystone, not a refinement.** Completion must fire after a non-zero
  emulated time ≈ (bytes × byte-rate), never instantly on the arm. Instant completion is the
  exact too-fast-hardware failure that cost us the SERAD/srinit saga (a ns32k loop had to see
  > ~0.32 s before its first poll; fixed with 2 CPUAP wait-states). The SINIX driver and the
  68000 firmware almost certainly carry timeouts/retry counters assuming real transfer time —
  a zero-time transfer or early completion trips them. Model byte-rate-timed completion from day 1.

## Register model (gate array; 68000 side)
- `E000` — SERDES config/status port. Writes = config/arm (`$23f` idle, `$2ff` read-arm);
  reads = generated status/echo. **Not a data path** (data is DMA'd, fw reads SRAM).
- `E800` — gate-array control: DMA start/stop `0x0cd3`/`0x0c12` (bit6 run), disk→SRAM read arm
  bit7 @ `$5ede`, SRAM→host DMA fire (bit12) @ `$13ce`.
- `C800[0x80]` / `D800` / `D000` — DMA address latches, firmware-programmed: `D000` = local
  (SRAM, word addr<<1); `D800:C800` = host (Multibus); `C800` is the scatter/gather table
  (index = addr bits 7-11, value = addr bits 1-6). A `C800` write while DMA-active latches the
  terminal (`term = 2×C800[0]` in the self-test); the transfer runs `D000` to the terminal,
  `F000 b12` = terminal reached. Self-test reference: fw `$9cb0`(start)/`$9cbe`(stop).
- `F000` / `E01E` — generated status: drive-ready (fw `$767c` btst#5), track0 (`$5de4` btst#$d,
  active-low), WP, INDEX, DMA-terminal (`F000 b12`), timer (b11), CRC/ECC-OK (`E01E b4`).
- `E802` — seek: bit6 rising = one STEP pulse (fw `$24ae`), DIR line; doorbell IRQ ack.
- Doorbell (`PIO 73F8 = 0x13`) → 68000 IRQ2; the gate array (bus master) fetches the 0x1c-byte
  IOPB from the host mailbox — real gate-array behavior, keep as gate-array model.
- Multibus master window (`bus_mem_r/w`, host RAM `0x010000+`) — the gate array's path to host
  memory for the IOPB fetch and the DMA. Already present and correct.

## Host-address source — decision table (this is where the original bug lived)
The gate-array DMA host address is **always** what the firmware programs into `D800:C800`; the
firmware derives it from the command's host-buffer field. The bugs were shims deriving it their
own way. Model = honor `D800:C800`, per trigger:

| trigger / caller                         | how host addr is set                       | notes |
|------------------------------------------|--------------------------------------------|-------|
| firmware read handler (`$5de4`, `$13ce`) | fw programs `D800:C800` from the IOPB buf  | the correct, target path |
| cmd-0x95 doorbell HLE (#2) — to remove   | C++ uses `m_iopb_buffer` (== the IOPB buf) | right address, wrong actor (masks the fw) |
| ioreg #1 HLE — to remove                 | C++ uses `req = i_ma` (ioreg[2,3,6])       | **WRONG addr = the VSN/"Falsche" bug** |

Open (resolve before coding the HD leg): does the **ESDI HD** read also stage through SRAM +
`D800:C800` like the floppy handler, or DMA sectors direct? (cmd-0x95 currently DMAs HD direct.)

## HLE to remove (Dave's inventory — all live paths)
1. `ioreg_w` OS-driver command channel (~943-1289) — **the big one**: the SINIX kernel's whole
   disk transaction in C++ (HD/floppy R/W, FORMAT fill, status synth, INT2, fake `c0`/sense/`87`).
   68000 fully bypassed. Delete.
2. `ch_w` cmd-0x95 BYPASS host-DMA **read** leg (~1943) — C++ does the transfer to `m_iopb_buffer`.
3. `ch_w` cmd-0x94 **write** leg (~2042) — mirror of #2 for writes (mkfs/install/tar).
4. `dataop_tick` host-DMA leg (~1592-1633) — parallel floppy delivery.
5. Doorbell cmd-0xa1 REINIT geometry synthesis (~800-820) — fw's own 0xA1 handler must set UIB.
6. Status/completion synthesis: `mbox_w` tap DONE 0x80 (636-642), doorbell BUSY 0x81 (826-830),
   E802 b7 DONE (2249-2256), kickoff DONE (2144-2150).
7. Gate-array IOPB/UIB fetch as one-off C++ copy (~770-796, 1880-1926) — fold into the gate-array
   bus-master model (lower severity; fw still runs the command after).
8. `m_ch_op_ok` synthesized CRC "no-error" (~1710/1731/1781) — becomes generated `E01E b4`.
- Dead (`!m_fw_driven`, flag permanently true): `read95_deliver` (1304-1380), E802 seek-deadline
  shim, E01E "always forced" branch. Remove.
- **Probes:** `m_trace`/log observers stay. **`STORAGER_NOBYPASS` CHANGES behavior** (gates the
  cmd-0x95 read leg) — a dev-only knob, same class as the removed CPUAP_SCALE/SERAD_SCALE.
  **STRIP before final: no env-var may change shipped behavior.** `STORAGER_DMALOG` is a pure
  observer but also strip before final.

## Build sequence (incremental; boot regresses at the cut, then returns)
1. **SRAM→host DMA engine** (keystone) with **byte-rate-timed completion**: on the fw's DMA fire
   (`$13ce`, bit12) move `D000`(SRAM) → `D800:C800`(host, via C800 scatter/gather) over
   `bus_mem_w` (host→SRAM for writes); raise `F000 b12`/`E01E b4`/IRQ4 after ≈ bytes×rate, not
   instantly. Validate on the fw's own read handler once #2 is gated off.
2. **Gate-array disk→SRAM read** on the `E800` bit7 arm: JIT-serve the current (cyl,head) track
   (ID+data) into SRAM `$4000` (this is `build_serdes_stream`'s job — keep/generalize it; retire
   the `m_sectors` *host-delivery* shim, not the track source). Firmware paces + parses SRAM.
3. **Generated status/completion** (F000/E01E/IRQ) so the fw posts its own DONE/INT2 (#6, #8).
4. **Remove #1 `ioreg_w`** so the SINIX kernel commands run through the fw's IRQ2 handler.
5. **Write path** (#3/#4), FORMAT, seek (E802 step), REINIT (#5) — firmware-driven.
6. Remove dead code; ESDI sector-stream source for the HD (resolve the SRAM-vs-direct question).
7. Validate: floppy boot → stage-2 SINIX1 (VSN lands in flchk's buffer) → HD R/W/format, with
   transfer timing sane (no timeout/retry trips).

## THE keystone — the gate-array IRQ6 pump (static analysis 2026-07-11)
Dispatch reaches the read handler fine; the read does NOT run inline — it is **queued** and
executed later by an IRQ6-driven pump the model never raises. Chain:
- doorbell → IRQ2 → intake `$0BF2` (sets host BUSY 0x81) → dispatch `$0D54` → main table `$92` →
  op 0x95 → `$5FC0`. `$5FC0` validates COUNT (`$5fc8/$5fd0`, err 0x6a if zero), builds the read
  microsequence into `$7940` (`$5fda-$60cc`, branching on cmd 0x93/0x95/0x99 + the `$796e`
  direction flag), then **`bsr $328e` ×2** (`$60aa`,`$60d2`) — `$328e` is a pure RAM store
  (`move.l $7940,(A0,$7942)` into the `$72ec` op table) + rts. It **queues** the op.
- The queued op runs only when the **VGC7219 raises IRQ6** → trampoline `$26ae`
  (`move.l $7304,-(A7); rts`) → soft vector `[$7304]=$9884` (installed at `$9616`) → the parser
  walks the queued steps into `$5e98` (arm E800 `$5ede`, fill `$4000` SRAM, then the `$13ce` DMA).
- **The model never raises IRQ6 → queued op never runs → read never completes → fw waits in its
  main loop = the hang.** So **raising IRQ6 on op-queue is the keystone** that unblocks the whole
  chain; model it like the IRQ4 descriptor engine already driven.
- **Both units** (floppy + ESDI HD) run 0x95 → `$5FC0` → pump → `$5e98`; `$5e98` is unit-conditional
  in exactly one place (`$5ece` reads unit `[$7a18]`, indexes the `$63e` op-table, ORs it into the
  E800 arm word) which selects the ENDEC-floppy vs ESDI-HD interface; everything after (the 4096-B
  SRAM spin-read `$5ee2`, the SRAM→host DMA) is common. **HD is NOT direct-DMA** — build the
  SRAM-staging path once and both units drive it, host addr from the same `D800:C800`/IOPB at `$13ce`.
- **Do NOT complete cmd 0x87** (identify): a completed 0x87 makes the CPUAP HD-boot and never issue
  the floppy 0x95 reads. Keep it timing out.

## Next action (build order, unblocked)
1. Timed **SRAM→host DMA engine** (on the `$13ce` fire): `D000`→`D800:C800`, byte-rate-timed
   completion, `F000 b12`/`E01E b4`/IRQ4.
2. **Raise IRQ6 when a read op is queued** (`$328e` store into `$72ec`) — patterned on the IRQ4
   model. This is what makes the firmware arm E800 → disk→SRAM → reach `$13ce`, and lets us finally
   capture the real `D800:C800` instead of runtime-spelunking.
3. `build_serdes_stream` stays as the disk→SRAM source on the E800 arm. Retire the cmd-0x95/#4
   host-delivery. Validate the floppy boot drives fully.

## Build progress + current blocker (2026-07-11)
- **IRQ6 pump-arm wired**: `install_write_tap($72ec..)` on the `$328e` op-queue store → `m_pump->adjust`
  → `pump_tick` raises IRQ6. Gated on `STORAGER_NOBYPASS` during cutover. (line ~447)
- **Blocker:** with `STORAGER_NOBYPASS=1` (cmd-0x95 HLE off) the CPUAP DOES issue the read —
  command sequence `cmd 87` (identify) → `cmd 89` (restore) → **`cmd 95` (read, buf=0fc0dd)** — and
  the doorbell DOES raise IRQ2 (lines 863/874/885). **But the firmware never reaches `$328e`**
  (`OPQUEUE arm = 0`), so it never queues the op and the pump never fires → boot hangs at "testend".
  So the fw's cmd-0x95 handler (`$5FC0`) stops/branches *before* the `bsr $328e` queue store.
- **ROOT CAUSE (static trace, Dave):** it is NOT a gate inside `$5FC0`. The handler is invoked at
  `$1202` (`movea.l D7,A1; jsr (A1)`, D7=`$5FC0`); the firmware bails *before* that jsr, at the
  **drive-identify precondition in dispatch**:
  ```
  000e8c: cmpi.w #$0,($0,A6)   ; A6 = per-unit UIB = [$20a + unit*2]
  000e92: bne    $eae          ; UIB word0 != 0 (drive identified) -> proceed to the jsr
  000e94: cmpi.b #$87,D0 -> $eae; else only 0x87/0x77/0xA7 allowed
  000ea6: move.w #$40,D0
  000eaa: bra    $1206         ; ERROR 0x40, skipping jsr $5FC0
  ```
  A `0x95` read on a unit with `UIB[0]==0` (not identified) → error 0x40, never reaches `$5FC0`/
  `$328e` → pump never arms → the "testend" hang. (`$5FC0`-internal exits: only COUNT==0 →
  err 0x6a `$5fd0`; the 0x93/95/99 + `$796e` direction branches all fall through to `bsr $328e`.)
  Sibling dispatch gates: unit≥8 → 0x11 `$df8`; unit≥4 & cmd≠0x86 → 0x11 `$e6a`; UIB[$d6]==0xffff
  → drive-init `$651c` `$f3e`.
- **What the HLE was fabricating = drive-identify / UIB setup.** The `#1/#6` synth (op-0x87 identify
  0xc1, c0 status, sense) made the unit read identified so `UIB[0]!=0`. HLE off → the real 0x87
  times out (ESDI drive side unmodeled) → `UIB[0]==0` → 0x95 dies at `$e8c`.
- **0x87 tension resolved:** boot-ROM 0x87 ("HD present, boot it" — must time out) ≠ SINIX 0x87
  (identify the drive). The `87→89→95` seen here is SINIX past the ROM, so 0x87 SHOULD identify.
- **=> Phase C is the unblock:** model enough of the ESDI/floppy 0x87 identify to set `UIB[0]`
  (drive-present/type). Then 0x95 clears `$e8c` → `$5FC0` → `$328e` → the IRQ6 pump-arm fires and
  the fw drives the read (finally programming the real `D800:C800`). Confirm-probe (optional): PC-tap
  `$5fc0` — never hit ⇒ `$e8c`/0x40; hit then `$5fd6` ⇒ COUNT==0.

## Phase C — the 0x87 identify that writes UIB[0] (static trace by Dave, 2026-07-11)
- `0x87 -> $5E64 -> bsr $1242` (drive-scanner). `$1242`: D3 = UIB[$1] (unit count); per unit — D4 =
  `$7654[unit]`, `blt` skip (already scanned), else UIB_struct = `$74c4 + unit*8`, clear `[+2]`,
  `bsr $352e` (**QUEUE**), `$7654[unit] = 0xc0` (mark scanned).
- **`$352e` is a doubly-linked-list insert** (links the op into the work queue at A0+4/+6) — NOT a
  UIB writer. `$355c` is a **UIB validator** (UIB[0]≠0, heads[$1]∈1..0x40, sectors[$6], bytes/sec
  [$2:3]∈0x80..0x800 even, [$4/$5]) — also not a writer.
- **`UIB[0]` is written by the queued interrogation op the IRQ6 pump runs** — from the DRIVE's
  E000/E800 response, NOT copied from the host IOPB. Backing store `$74c4 + unit*8`; pointer table
  `$20a` (`A6 = [$20a+unit*2]`, `UIB[0] = (A6)`).
- **Why C++ geometry never helps:** the 0x87 is still HLE'd (only the read leg is gated), so `$1242`'s
  queued scan op **never runs through the pump** → the firmware's UIB[0] writer is bypassed. Setting
  `m_unit_heads` in C++ never reaches `$20a[unit]->[0]`.
- **Unit = HD 0/1** (0x87 = HD-identify, table param 0820; and by elimination — floppy presence is
  already F000-modeled so it wouldn't hang; the UIB[0]==0 hang is the unmodeled ESDI drive side).
  Confirm exact unit via IOPB[$4] at `0fe780`. Test Phase C with `-hard` present.
- **Phase C = two halves:** (a) un-HLE the 0x87 identify (gate `#7` fetch + the `m_iopb_cmd==0x87`
  block) so `$1242 -> $352e` queues the scan and the pump runs the interrogation; **the pump-arm must
  also fire for the scan-op queue ($352e insert), not only the read op-table ($328e).** (b) model the
  ESDI drive-presence/geometry response the interrogation reads (presence-ready F000 + valid geometry)
  so it writes a non-zero `UIB[0]`. Then `$e8c` passes → `$5FC0` → `$328e` → read chain lights up.
- **RESOLVED (Dave, 2026-07-11): UIB[0] is host-supplied heads, DMA'd host→local by the gate array
  — NOT a drive-side interrogation.** The gate array bus-masters the host UIB (0x1c bytes) from the
  IOPB buffer addr (`m_iopb_buffer` = IOPB words 6-7) → local `D000<<1`; `UIB[0]=heads`. It's the
  same host↔local DMA engine as the read, run host→local. Completion = IRQ4 → `[$71f0+$18]` (=$7208)
  so `$322e` doesn't time out to error 0x60 (fast-exit if `[$749c]==0`).

## Phase-C EMPIRICAL RECONCILIATION (2026-07-11) — the identify already works; the READ leg is the gap
Booted `-hard1` (stage-1 MC1325 install) headless, HLE path vs `STORAGER_NOBYPASS`:
- **HLE path: HD boots SINIX fully** — "no sys-floppy, going to harddisk" → "Boot: sa(1,0)sinix" →
  SINIX-M-C V2.0 (Rev 266). The 0x87 identify fires for **unit0 AND unit1** with valid geometry
  (`heads=8 spt=9 secsize=1024` = MC1325 logical nt=8; and `heads=12 spt=34 secsize=512`). So the
  HD `UIB[0]=heads` IS set under HLE.
- **The HLE 0x87 copy IS the faithful gate-array DMA:** probe `UIB87` showed `D000<<1=$71f0` (the
  channel control block, = the `$322e` channel base) ≠ the fw UIB struct `[$20a+unit*2]=$6e60`, yet
  `$6e60[0]=0x0208` (heads=2 for the floppy) — i.e. **the firmware itself propagates `$71f0 → $6e60`
  after the DMA+completion.** So the identify is NOT separately HLE-broken; the host→local UIB DMA +
  channel-complete is all the gate array owes it, and the fw does the rest.
- **`STORAGER_NOBYPASS` path hangs at the FIRST 0x95 read** (the floppy sys-floppy-check, well before
  any HD access) → screen stuck at "testend". So under the LLE read-gate the boot never reaches a HD
  identify; the `$e8c` UIB[0]==0 → error-0x40 trace was from a partial-LLE state, not a fresh boot.
- **Corrected Phase-C content:** the real gap is the **READ DATA path (0x95)** — disk→SRAM→host DMA +
  channel-complete/IRQ4 — for floppy AND HD. The identify's host→local UIB DMA rides the SAME engine
  (opposite direction) and already works. Build the read channel first (DESIGN steps 1-3); the
  identify falls out for free once the host↔local DMA engine + IRQ4 completion are real.
- The C++ SHADOW latch (`m_unit_heads/_spt/_secsize`) is the only non-faithful part of the 0x87 block:
  the LLE read must consult the firmware's UIB struct (`[$20a+unit*2]`), not the shadow. Retire the
  shadow when the read channel lands.

## Read-channel bring-up — the LLE read hang, localized (2026-07-11)
Ran under `STORAGER_NOBYPASS` with a real boot floppy (`mx2-001.imd`), `STORAGER_TRACE`+`STORAGER_DMALOG`:
- **The fw reaches the real channel.** It issues the full `87→89→95` through the D000 CCB at `$71f0`
  (D000=0x38f8, `D000<<1=$71f0`); the 0x95 CCB shows cmd `9500`, count `0008` blocks, host buf `0xfc0dd`.
- **disk→SRAM works.** `DATAOP` (opfam=2, ID capture) delivers the ID field `C00 H00 R01 N00` (128B FM)
  into SRAM at `$71f0` — the correct LLE direction (into local SRAM, NOT the host buffer).
- **Then it hangs in a seek/data wait at `$369a`/`$36c2`**, toggling the E800 low nibble (step bits) and
  polling **`$71b2`** and **`$7b1a`** for non-zero, for 30+ s. `D800:C800` never programmed (no host DMA).
- **Those flags are set by the IRQ6 pump/parser.** `$71b2` ← `$1ba0` (op-dispatch), `$7b1a` ← `$256c`
  (command completion); both run via IRQ6→`$26ae`→`[$7304]=$9884`. `OPQUEUE→IRQ6=0`: the pump never
  armed (the `$328e`/`$72ec` window doesn't catch this op's queueing).
- **Experiment: fire the pump on DATAOP completion (`m_pump->adjust`, gated NOBYPASS).** Result: the
  `$369a` wait **releases** (fw advances past it) — confirming the pump is the right completion path —
  **but the parser then panics** to the `$24e` blink loop (`move.w #$e1b,$e800; andi #$aaaa,$7d98; bra`).
  So firing IRQ6 after the *ID-capture* phase makes `$9884` process an incomplete command.
- **Conclusion / open question:** the pump is per-COMMAND completion, not per-phase. Either (a) IRQ6 must
  fire only after the data phase (opfam==0) with ID+data both staged in the CCB, or (b) there's a
  per-phase (non-pump) handshake that advances ID→data and the pump fires once at the end. Need the
  `$9884` parser's expected CCB layout at `$71f0` and its ID→data sequencing to place the pump correctly.
- **Note:** the DATAOP `opfam==0` data-phase (storager.cpp ~1617-1653) still writes the decoded sectors
  DIRECTLY to `m_iopb_buffer` (HLE host-DMA) — that's the HLE takeover to replace with a real
  SRAM→host DMA at the fw's `D800:C800` once the ID→data→completion sequencing is right.

### Deeper trace — the parser buffer is D800<<1=$7dac, not D000<<1=$71f0 (2026-07-11, Dave's $9884 trace)
Dave pulled the pump internals: **`$9884` is a per-ID-field matcher** (rte @$9982), reads the field at
`[$7a66]`, checks the ID AM (FM `$9934`: `cmpi.b #$fe,(A3)+`; MFM `$98ac`: A1-triple then `$fe`), matches
C vs `$7438` / H vs `$7436` / sector, sets `$7a68`=match or `$7a6a`=retry (`$2012` seek / `$202a`
wrong-head / `$2029` sync-fail). **One ID per interrupt.** The **data phase `$9984`** (a step-list entry
`$3806`/`$39c0`, NOT an IRQ6 vector) reads the data field (`$6788` transfer) that must sit **immediately
after the matched ID in the same SRAM**. `$369a` is the per-step wait (toggle E800 step bits, spin on
`$71b2`/`$7b1a`, timeout→head-step, D5=2 retries) — the fw walking the track field-by-field.
- **Verdict = per-phase, consuming a CONTIGUOUSLY-STAGED ID+data pair** (not two deliveries). Stage each
  sector as the ENDEC byte stream `sync/AM + ID(C/H/R/N)+CRC + gap + sync/data-AM + data(secsize) + CRC +
  gap` (exactly what `build_serdes_stream` already emits), then IRQ6: `$9884` matches the ID, the fw reads
  the data inline. For the 8-block read, stage the run and let `$369a` walk it, pump scoring each ID.
- **The staging DESTINATION is `[$7a66]=$7dac` = D800<<1** (`$97ae`: `#$7dac→D0→$7a66`; `$97c8`:
  `D0>>1→D800`). The emulator's active `dataop_tick` stages at **D000<<1=`$71f0`** (wrong latch/addr) and
  only the header field — so the parse buffer never gets a coherent sector. Confirmed empirically: in the
  NOBYPASS run **D800 stayed `0000`** the whole time (the `$97c8` D800-program lives inside the pump, which
  panicked first), and IDCAP/E000START/DATAARM all fired 0×; only the wrong-buffer DATAOP fired.
- **Sequencing (open — Dave to confirm via `$1f82→$9984`):** `$369a` waits for the pump, but the
  capture+IRQ6 needs the fw's arm. The real gate array breaks this by having the **E800 step-toggle in
  `$369a` autonomously trigger the capture** (stage next field at D800<<1, raise IRQ6). Need: (1) is
  `$9984` reached synchronously off `$369a`/`$1f82` after `$9884` sets `$7a68` (no 2nd interrupt), and
  (2) where D800 is FIRST set to `$7dac` — in the `$5FC0` read setup before `$369a`, or only inside the
  pump — since that decides whether the step-toggle can stage at `$7dac` on the first pass.
- **Refactor target (single faithful path):** retire the DATAOP/IDCAP/`$4000`-bulk overlap for the
  fw-driven read; on the `$369a` E800 step-toggle, stage the next full `build_serdes_stream` sector at
  **D800<<1**, raise IRQ6; `$9884` matches, `$9984` reads data inline; repeat until count satisfied; then
  the fw fires the SRAM→host DMA at its own `D800:C800` scatter/gather (retiring the `m_iopb_buffer` HLE).

### Refactor attempt #1 + the two-level completion structure (2026-07-11)
Wired the LLE staging (idcap_tick full-sector at D800<<1 + IRQ6, 0x95-scoped; E802 bit15 arm; gated the
D000 dataop for 0x95).  Result: **regressed** — the read hung earlier at `$764a` (t=6.19) instead of
`$369a` (t=6.4).  Cause + the structural lesson:
- **The read completes in TWO levels** (storager.cpp:2231 comment, and confirmed by the hang): a **VERIFY
  CHAIN** where each E800 kick is a channel op that completes on **IRQ4** (what `dataop_tick` supplies),
  followed by a **TRANSFER STAGE** = the **IRQ6** pump (the `$369a` wait).  Gating the D000 dataop killed
  the verify-chain IRQ4 → the read stranded at `$764a` before ever reaching the transfer stage.  So the
  LLE read is NOT a wholesale D000→D800 swap; the per-kick IRQ4 completion is load-bearing.
- **BUT the buffers/IRQ mismatch across the two paths:** the verify chain uses `dataop_tick`→D000<<1
  (`$71f0`) + IRQ4, yet Dave's parser trace says `$9884` (IRQ6) reads the ID at D800<<1 (`$7dac`).  And in
  the run **D800 never became `$7dac`** (the `$28e0/$290c/$2928` read steps that copy `$7434→D800` never
  executed — `DATAARM E802 = 0`, so the E802-bit15 "go" never fired).  So the `$369a` we keep hitting is
  the **verify chain's** wait (pre-`$28e0`), not the transfer stage's — and it needs the pump (IRQ6)
  reading the ID at `$7dac`, which nothing stages because the verify uses the D000/IRQ4 path instead.
- **Tension to resolve with Dave:** the code's 2231 comment says verify=IRQ4 / transfer=IRQ6, but Dave's
  `$9884` trace says the IRQ6 pump IS the per-ID matcher used in the ID/verify walk.  If the latter, the
  verify chain's `dataop_tick`→D000+IRQ4 is itself the mismodel, and the verify should stage the ID at
  `$7dac` + IRQ6.  Either way I need the **read-phase map**: (1) does the verify chain precede the
  `$28e0` read steps; (2) which buffer + IRQ level does the verify use vs the transfer; (3) when/where
  D800 first becomes `$7dac` for the verify (since `$28e0` doesn't run before the verify `$369a`); (4) is
  the per-sector capture armed by the E800 step-toggle inside `$369a` (Dave's earlier note) or by the
  E000#$2ff/E802-bit15 read-step arm (Dave's later note) — these fire at different phases.
- **State:** reverted to known-good (`$369a` at t~18, verify chain via DATAOP intact, 87/89 untouched).
  The LLE staging (idcap_tick 0x95 branch + E802-bit15 arm) is left in but **dormant** (E802 bit15 never
  rises during the read) as scaffolding for attempt #2 once the phase map is settled.

### Runtime phase trace (STORAGER_PHASELOG) — ground truth, contradicts the static map (2026-07-11)
AS_OPCODES fetch taps ($5fc0/$60d6/$89f2/$9884/$9984/$3bfe/$26ae/$369a/$28e0/$290c/$2928/$92b4/$7ba8/
$1f82) + D800/E000/E802 arm logs, one "PHASE" stream.  HLE first: only IRQ4-entry fires — the shim
short-circuits the fw read parser, so HLE gives NO read-code phase map (the read code only runs under
NOBYPASS).  NOBYPASS (boot floppy, sys-floppy-check read), full 12s run — see
`docs/storager-lle/phase-trace-nobypass.txt`:
```
cmd=87 CHANSVC($1f82) ×1 → cmd=89 IRQ4 ×3 → cmd=95 IRQ4 ×3 → cmd=95 RDSTEP-op0($28e0) ×1
   → cmd=95 SEEKWAIT($369a) ×11992 (hang)
```
- **`$5FC0` (READ-START) and `$60d6` (the `$7434=$7dac>>1` setter) fire 0× the whole run.** So
  **`$7434` stays `0000`**, and when `RDSTEP-op0` (`$28e0`) does `move.w $7434,$d800`, **D800 = 0**.
  D800 never becomes `$7dac`, `$7dac` stays all-zero, and the read step is mis-set-up from the start.
- **No `$89f2`, no `$9884`, no `IRQ6`, no `$290c`/`$2928`, no op-handlers** — the fw hangs in `$369a`
  BEFORE any ID check.  So the `$369a` we kept hitting is reached with a **null D800**, not a
  transfer-stage wait on a staged sector.
- **This contradicts the static model** (`$5FC0` sets `$7434` before `$369a`): in the runtime, `$5FC0`
  is never reached.  Either `$5FC0` is not this read's entry (the sys-floppy-check read dispatches
  elsewhere), or the `$7434`/D800 setup lives on a path the fw skips here.  Open for Dave against the
  trace: **where does this read's `$7434`/D800 setup actually run**, given `RDSTEP-op0` executes but
  `$5FC0`/`$60d6` do not?  That's the true first domino — nothing downstream (ID stage, IRQ6) can be
  right while D800=0.

### ROOT CAUSE FOUND — the fw command dispatch is intercepted for 0x87/0x95 (2026-07-11)
Dispatch table at `$92` (4-byte entries indexed by `cmd-0x70`, read at `$0D54`: `mulu #4,D1; lea (A1,D1),A1;
move.w (A1)+,D7`):
```
0x87 -> $5e64 (identify)   0x89 -> $5f74 (restore)   0x93/0x94/0x95/0x97 -> $5FC0 (read, flags $8c27)
0x88 -> $3736 (a DIFFERENT command's descriptor constructor, flags $b124)
```
So the 0x95 read handler IS `$5FC0` (static model correct). Runtime dispatch taps (`$0D54`/`$5e64`/`$5f74`):
- **`$0D54` (CMD-DISPATCH) fires ONCE — for cmd=89** → `$5f74` runs. **0x89 dispatches cleanly.**
- **For 0x87 and 0x95, `$0D54` NEVER fires; `$5e64`/`$5FC0` never run.** The emulator's HLE intercepts
  0x87 (the `m_iopb_cmd==0x87` INIT block) and 0x95 (the doorbell/DATAOP/CHANKICK path) **before** the fw
  command dispatch — so `$5FC0` never builds the read descriptor (`$7434=$7dac>>1` never set) → `$7434=0`
  → `RDSTEP-op0` sets D800=0 → the fw's IRQ4 engine, kicked by the emulator's 3× premature DATAOP IRQ4s,
  walks the step machine (`$28e0`→`$369a`) on an **unbuilt descriptor** → SEEKWAIT hang, no ID check, no
  IRQ6.
- **The fix (next):** under NOBYPASS, stop intercepting the 0x95 doorbell so the fw's IRQ2 → `$0D54` →
  `$5FC0` runs and builds the descriptor itself. That means retiring the emulator's premature DATAOP/
  CHANKICK IRQ4 for 0x95 AND whatever doorbell/IOPB HLE prevents the fw dispatch (the same class as the
  0x87 INIT interception).  Open for Dave: where in the fw IRQ2/doorbell handler the 0x95 path diverges
  from 0x89 (0x89 reaches `$0D54`, 0x95 doesn't) — i.e. what the emulator does on the 0x95 doorbell that
  0x89 avoids.  Trace: `docs/storager-lle/phase-trace-nobypass.txt`.

## ============ CHECKPOINT (2026-07-11): LLE THESIS PROVEN + task-#4 spec ============
After a long measure-refute arc, the storager LLE goal is **proven reachable** and every upstream contract
is measured. This section is the durable checkpoint + the spec for the remaining build. The `getenv`
scaffold that proved it is NOT the implementation (see "Scaffold" below).

### The proven result (measured, not asserted)
**The firmware drives its own transactions when fed faithful per-op gate-array completions.** Under
`STORAGER_NOBYPASS` (all HLE data-writes off, all completion signals on), posting the gate-array
channel-complete per op advanced the firmware through its OWN command sequence: **`DISP-PRE` 1→9**,
running dispatch → seek-done → restore-status, each step unblocked by one faithful signal. This is the
mandate demonstrated end-to-end for the upstream path.

### Root reframe: two DONEs, and who posts each (the decomposition that unlocked it)
- **Seek/restore completion is EMULATOR-posted** (gate-array does the physical seek, posts channel-complete).
- **Read completion is FIRMWARE-posted** (the fw reads its data and posts its own host-DONE at `$0BF6`).
- The HLE shim (site 2131) conflated them - it posted both. `STORAGER_NOBYPASS` wrongly killed BOTH the
  data-write (correct to remove) AND the completion signals (WRONG - they're HW-level support the mandate
  requires). The fix is to keep the gate-array completion layer live and remove only the data-write.

### Measured per-op contract table
| cmd | who posts completion | contract (measured) |
|-----|----------------------|---------------------|
| 0x89 RESTORE | emulator (on `m_seek_deadline` settle) | ring-DONE (b: IOPB `+2`/`0x0fe782`=0x80, or e: `m_read_pending=false` → `mbox_w` local-DONE) releases `$369a`; **E01E bit4 (c: `m_ch_op_ok`) gates retry** - without it the fw reads the op errored and re-restores (measured: adding it cut retries 9→1). Contract = ring-DONE + E01E-OK + track0 status. **NOT `$7654`** (constant 0xc0). |
| ~~0x02 / 0x0c INIT~~ | — | **RESOLVED (task #2, 2026-07-11): NOT commands, NO contract.** They dispatch to the error path (`$d54`→`$d58/$d5e`→`$d60`, result `$14` = "command out of range"). Measured `D0=$14` at the post-handler epilogue `$1206` for every 0x02/0x0c; `D0=0` (OK) for every 0x89. **They are the high byte of gate-array channel words** (`0x0200`/`0x0c00`) written to the `$d000` window at `pc=$3d4a` (routine `$3d30`-`$3d4c`) — and `$71f0` is **aliased** (D000<<1 = the CCB base = the dispatcher's `A0`). When the dispatcher runs on that transient window value it misreads `0x02`/`0x0c` as a command. **Static proof they can never be valid: the dispatch table only covers cmd 0x70-0xAF.** This is Dave's *error-fallback* case, not an init step — a **corruption symptom of the crude per-spin scaffold**, which drives a spurious restore→identify(0x87)→channel-program loop that thrashes `$71f0`. Fix = task #4 (stop over-firing DONE); no 0x02/0x0c contract to implement. |
| 0x95 READ | **FIRMWARE** (`$0BF6`) | emulator provides DATA-READY ONLY: idcap stages sector at `$7dac` (wired, site 1646) + IRQ6 pump + E01E-OK. **Post NO crude read-DONE** (= the data-less-read trap, measured out). The fw's `$5FC0`→E802-bit15→`$9884`→`$9984`→`$0BF6` runs itself. |

### The inversion (the key structural finding)
**Read tail (#3) is gated BEHIND the per-op model (#4), not the reverse.** `$5FC0` only runs after the
restore→read sequence is CORRECTLY sequenced - and correct sequencing IS the per-op model.
Proof: a single fixed completion word on every `$369a` spin cannot be right for the in-flight op -
always-on E01E over-completes (hangs after 1 dispatch), no-E01E under-completes, neither reaches `$5FC0`.
So the read microsequence + the already-wired `$7dac` tail light up ONLY after `$5FC0` runs, which needs
the real model.

**Task-#2 measurement sharpened this (2026-07-11):** the "89/02/0c" loop is NOT three ops cycling — it is
`0x89` (restore, `D0=0` OK) plus **spurious error dispatches** where the dispatcher misreads the aliased
gate-array `$d000`/`$71f0` window (channel words `0x0200`/`0x0c00`) as commands `0x02`/`0x0c` (see table).
The read command `0x95` **is staged into `$71f0` four times** (pc `$1264`/`$156a`/`$203e`/`$23da`) but is
overwritten by the identify/channel churn before dispatch — `$5FC0` runs **0** times. So the concrete
task-#4 win-condition is now exact: **stop the over-firing so a staged `0x95` survives to dispatch.**

### Task-#4 spec (the per-op completion model — the next build, do from THIS doc)
> **DISCIPLINE (load-bearing, not a slogan): MEASURE each contract, don't infer it.** Every mechanism
> inferred this session was wrong and cost turns (F000-track0, the IRQ4 prefetch artifact, `[$791a]`-is-a-
> param-bit, (b)/(e)-not-(c)/(d)); every measured contract advanced us. The build WILL tempt the same
> inference - especially the **0x02/0x0c contracts (task #2): trace them exactly as the restore's E01E
> bit4 was traced (working-default diff + isolate-each), do NOT reason them out.** The reclassification
> that makes this a build not a question - "748a is the read node, completion is the mailbox-DONE" - came
> from two measurements (the working-default diff + the SHIMTEST isolate-each), not from analysis.

1. **Track the in-flight op** per channel (from the E802 arm / the CCB command byte at `$71f0`/the node),
   not `m_iopb_cmd` (a transient host shadow).
2. **Map op-type → completion-word** (the table above): restore→ring-DONE+E01E-OK+track0; read→data-ready-only.
   (There is **no init/0x02/0x0c op** — task #2 proved those are error-dispatch artifacts of over-firing, not
   real ops; when the model stops over-firing they disappear.)
3. **Trigger on the op's ACTUAL settle event** (seek: `m_seek_deadline`; read: idcap staged), NOT on
   every `$369a` spin. This is the structural fix the crude scaffold can't do.
4. Then `$5FC0` runs → E802-bit15 → idcap stages `$7dac` → `$9884` matches ID → `$9984` reads data →
   `$0BF6` posts fw DONE → bump. Read-tail validation open items (task #3, when reached): does `$9884`
   match idcap's rotationally-delivered sector vs a target R (likely first gap - a target-sector staging
   fix, not a completion word); does `$9984` pull the data behind the matched ID (format at idcap
   1668-1669); does `$0BF6` post.

## ============ BUILD #4 IN PROGRESS (2026-07-11): per-op model in place; blocker re-localized ============
The crude per-spin `SEEK-DONE-POST` scaffold is REPLACED by a per-op completion model in `ch_w`:
- **What was built (works):** latch the armed seek's own IOPB (`m_seek_iopb`) + arm flag (`m_seek_fired`,
  now `true`=idle) at the 0x89/0x98 doorbell; post the completion (b host-IOPB DONE on `m_seek_iopb`, b
  local mailbox `$fe782/3`, c `m_ch_op_ok`, e `m_read_pending=false`) EXACTLY ONCE per armed seek, gated by
  `!m_seek_fired && time>=m_seek_deadline`, pc in `$3690-$36d2`. **Result: the over-firing garbage is GONE**
  - DISP-PRE is a clean single `0x89`, zero `0x02/0x0c` churn (task-#2 mechanism confirmed by its absence).
- **Blocker (measured, re-localized - NOT what task-#4 spec assumed):** `$5FC0` still 0. Two facts:
  1. **Delivery is too sparse.** The fw does the whole restore→read handoff in a ~55ms BURST (6.402-6.457)
     then pure-spins in `$369a` (reads `[$71b2]`, no E800 writes). The E800-write hook can't deliver at the
     75ms deadline (6.475) - it misfired at 7.31 (next E800 write in-range). Delivery must be timer+IRQ
     driven, not "fw happens to write E800."
  2. **GATE NAMED (locate-then-diff, measured): `node+26` = `[$71f0+0x26]` = `[$7216]` is a STATE-MACHINE
     field, not a done flag.** `$369a` releases on `[$71b2]!=0`, bumped only by the fw's `$24fe` (CH-PROCESS)
     walk reaching `$260c`. In `$24fe` all three walks are identical except one operand: `$251e: tst.w
     ($26,A0)` (A0=`$71f0`) then `$2522: bne $2698` (skip). Measured at CH-PROCESS: **0x87 identify node+26=
     `0000` -> falls through -> `$260c` BUMP; 0x89 restore=`0006`, 0x95 read=`000c` -> `bne $2698` skip ->
     read falls to `$1c8c/$1d66` decrement.** ([$7ff8]=0x13/bit0-set and [$71b2]!=2 for ALL three - not the
     differentiator.) `($26,A0)` is written across the ROM with 0/2/4/6/8/0xa/0xc (`$2000`->6, `$17f8`->0xc,
     `$2244`->8, `$1fcc`/`$2626`->4, several `clr`->0): it is the channel op's STEP/STATE. The bump fires only
     from state 0 (identify = single-step). **Restore(6)/read(0xc) are MULTI-STEP ops parked mid-sequence -
     the walk correctly skips them; they are NOT done.** `$7e1e`/D0 is a red herring (measured 0=success by
     default; the skip happens at `$251e`, BEFORE the `$260c` D0 test is ever reached).
- **This settles the (a)-vs-(b) sequencing question:** the read's pre-settle walk took `$1d66` NOT because it
  ran too early but because the node is at a genuine intermediate STATE (6/0xc). So (b) is NOT "timer -> raise
  IRQ2 at settle" - a bare IRQ2 would re-walk into the SAME skip. The seek-complete signal must make the fw
  **ADVANCE node+26's state machine** (restore: 6 -> ... -> a completable state), which the fw does itself in
  response to a hardware step-completion. Guardrail extends: node+26 must be advanced BY THE FIRMWARE via a
  faithful gate-array/channel signal, never poked from C++.
- **RUNTIME REFRAME (2026-07-11) - the state machine is NOT the blocker; the `[$71b2]` SEMAPHORE is.** A live
  write-tap on node+26=`[$7216]` shows it is a GENERIC channel phase field (written by cmd 87/89/95 at fixed
  per-state pcs - `$2006`->6, `$224a`->8, `$17fe`->c, `$1d0e`/`$3c8c`->0, `$2626`->4) that runs cleanly
  `4->6->8->c->0` in the 6.40 burst - it REACHES 0 (@6.40200) and STAYS 0 through the spin. Last turn's "parked
  at state 6" was a single-sample artifact. **The actual wait is `$369a`: `tst.w $71b2; bne $36de` OR `tst.w
  $7b1a; bne $36de`** (release if EITHER != 0; a step-generator pulsing E800 between polls). `[$71b2]` is a
  completion SEMAPHORE: bumped +1 at `$2632` (0x87 identify complete @6.40031), DECREMENTED -1 at `$1d6c` (the
  read's `$24ea` consumes it @6.40212) -> 0; `[$7b1a]` flat 0. The loop spins because the one bump (0x87's) was
  consumed by the read and no fresh bump follows.
- **Generic vs op-specific: GENERIC** (answers the scoping question) - one shared phase machine; restore and
  read run the same sequencer. Task-#4 remainder = model the channel machine's step-events ONCE, not per cmd.
- **The fix is VALIDATED, not hypothetical: timer -> IRQ2 at seek-settle WILL bump.** Because node+26=0 during
  the spin, a post-settle `$24ea` walk reaches `$260c` and bumps (the `$251e` gate passes). Proof: the current
  per-op SEEK-DONE (memory-write path) DID cause a `$2632` bump when it finally fired (`71b2<-0001 pc=002632
  @7.31534`) - just too late (E800-hook is sparse post-burst). This refutes last turn's "bare IRQ2 re-walks
  into the same skip" concern - node+26 is 0 by the time the timer would fire.
- **Next action (build #4 cont.):** replace the E800-write-hook delivery with a TIMER armed at the seek-settle
  (reuse `m_dataop`/`m_hd_chan` alloc); the callback raises **IRQ2** -> fw runs `$24ea` -> node+26=0 -> `$260c`
  bumps `[$71b2]` -> `$369a` releases. Arm for whatever seek is waiting - restore's AND the read's internal
  seek (the read's `$1d6c` decrement depletes the semaphore, so the read's seek needs its own bump). Then watch
  `$5FC0` run + the read data tail. GUARDRAIL: the timer raises IRQ2; the FIRMWARE bumps `[$71b2]` - never C++.
  Probes: `SEEK-DONE(per-op)`, `STATE26` `[$7216]` tap, `71b2<-`/`7b1a<-` taps, GATE260c RING, CH-* SEL dump.

## ============ BUILD #5 (2026-07-12): LOCK-ON PASSES; blocker = null continuation cells ============
Session recovered after a machine crash; the 0xFF-marker edit had NOT landed - re-derived from the
disasm, not memory. Repro reconstruction: the "read transfers" state needs the FULL knob stack
`STORAGER_NOBYPASS=1 STORAGER_PHASELOG=1 STORAGER_BUSYHOLD=1 STORAGER_STEPIRQ=1 STORAGER_SEEKACTIVE=1
STORAGER_STEPBIT0=1` + boot from the mx2-001 floppy (the ~6.5s sys-floppy-check read IS the LLE read).
The stg2 mount-at-prompt flow can't even reach SINIX1 under NOBYPASS (the HD label read rides 0x95 too).
⚠ `STORAGER_A118=1` SEGFAULTS (its E802-write log block; __dynamic_cast in the 4-connector
get_device() loop) - leave it off; it's strip-listed anyway.
- **The $89f2 lock-on ISR check, measured + confirmed live ($8a14-$8a30):** buffer at $7dac must be
  `[A1 A1 A1][FE][FF]` - OR of bytes 0-2 == $A1 (sync, REQUIRED EVEN FOR FM, same as $9884), byte 3
  == $FE (IDAM), byte 4 == $FF (ID-valid marker). Each valid ID decrements the $7a0c lock-on
  countdown (seeded #$3 at $6ab2; node[$20] bit14 RESETS it at $8a0e); $7a0c==0 -> the $8a42
  success path. $79a4 = retry budget; exhausted+unlocked -> 0x2029 at $8ab2.
- **Fix landed in idcap_tick LLE branch (2 edits): insert $FF after the $FE IDAM; write the A1 A1 A1
  sync run UNCONDITIONALLY (FM included).** mx2-001 track 0 decodes FM -> the old `if (!trk.fm)` skip
  delivered `FE FF ..` at $7dac[0] and the sync OR got $FF != $A1 - every ID rejected (that was the
  1398-iteration stall). RESULT (run6): 27 IDCHK, lock-on completes, fw takes $8a42 -> $8a54 -> $8aa4.
- **NEW BLOCKER, measured: lock-on success lands in NULL continuation cells.** $8aa4 does
  `movea [$72de],A0; clr (A0)` - run6: [$72de]=0, the clr hit unmapped 0 ("write to 000000"). $7968=0
  (skips the $352e queue arm), $7986=0. Nobody was waiting on the lock-on result -> node+26 parks at
  $000a, $9884/$9984 (DATASTEP) never run, host retries 95/89/95 then "no sys-floppy".
- **Decoded (static): the $72xx waiter-pointer table + descriptor blocks.** The setup builders
  ($3736 and $390e; run6's MICROSEQ list `0058 0018 0054 004a 0042 0036` = a third sibling) build
  8-byte blocks {gate, mode $0246/$0248, 0, HANDLER} at $727e/$7286/$728e/$7296/$729e/$72a6/$72ae and
  register them in $72d6/d8/da/dc/de/e0/e2. **[$72de]'s block carries handler $9984 = DATASTEP**: the
  lock-on success clr of its gate word IS the data-phase GO. $3736 also seeds $7428 (target R) from
  IOPB[9] at $3962. Cleared-by: $7a6a/$8446/$91fe (clr $72de). Open Q (probe added, run7): was $72de
  registered-then-cleared, or did the registering builder never run?
- **Model bug found: BUSYHOLD's SEEK-DONE(timer,host-post) posts host DONE on the READ's IOPB
  (0fe780) MID-READ** (@6.5774, while the fw is in the ID scan) - the premature completion that
  drives the host's 95->89->95 churn. The seek latch (m_seek_iopb) matches because the earlier 0x89
  used the same IOPB address. Needs a cmd guard (don't host-post a latched seek whose IOPB now
  carries 0x95) or the real fw-posted completion.
- Probes added this build (TEMP, STRIP): `WAITER[$72d6-$72e5]<-` write tap, `HOOK 7968<-`/`HOOK
  7986<-` write taps, `RING`/`RINGDONE` taps ($1bbe/$209c/$223c/$2244/$2056 + cursor dumps) (all in
  the m_fw_driven device_start block).

### Build #5 cont. - the $7e00 ring is a PARK/RESUME coroutine mechanism (decoded, runs 7-11)
- **Waiter probe (run7): $72de NEVER registered** - only boot RAM-test writes (pc $9cea/$9d14,
  <@0.14). The $3736 builder never runs for the sys-floppy check; the $8aa4 clr-to-null is likely
  a NORMAL no-waiter case on real HW (write to $0 is harmless there). NOT the blocker.
- **Stale-seek guard added (KEEPER):** seek_done_tick now reads the IOPB's current cmd byte and
  skips the host post if it is not 0x89/0x98 (the host reuses the IOPB slot; run6's post landed on
  the 0x95 READ mid-scan). Run8 result: host churn GONE - host waits correctly; run6's entire
  "ID-scan progress" is now explained as an ACCIDENT of the churn (the re-doorbell edge re-drove
  the dispatch at a moment the seek had settled).
- **The remaining loop (runs 8-11, 746 cycles of 31.6ms): the fw's step-generator protocol, now
  fully decoded static+live:**
  - $369a wait: poll [$71b2]/[$71a]; on TIMEOUT fall to $36ac = the REAL head-step pulse train
    (D0's nibbles eor/ror'd into E802's low nibble incl bit0, 2 pulses per nibble). The E800=2250
    writes at $3694 are the wait loop's own polling waggle, NOT op submissions.
  - CHANSVC $1f82: after claiming [$7b1a], polls bit7 of the ring entry at [[$7b1c]] (bclr on
    consume). $7e00-$7e1a = ring entry words; $7e1c = hardware-owned nonzero status gate ($205e,
    only ever tst'd); $7e20+ = 0x20-stride payload blocks ([$7b20]/[$7b2c] cursors).
  - $bba (entered from $2000 with the ch0 mailbox bytes): node[$1c]long = ~inverted status; ZERO
    (= mailbox 01/ff/ff/ff, our stamp is CORRECT) -> happy path: A2=[$7b20]; (A2)==0x8f -> quick
    $700; else COPY the 12-byte payload block -> [$7b1a] node and RE-DISPATCH via $d54. It does
    NOT return to $2000 ($2056/$2244 taps: 0 hits, all 746 cycles took the $bba continuation).
  - $1b90 = the PARK: consume a [$71b2] bump, clr node+26, copy 24 BYTES OF THE NODE into the
    payload block at [$7b20], advance $7b1c/$7b20 (wrap $1bd6). So the fw parks a continuation
    snapshot at SUBMIT; the hardware executes the op and stamps entry bit7; $bba RESUMES by
    copying the snapshot back and dispatching. Classic coroutine park/resume over dual-port RAM.
  - The IRQ2 walk ($2570-$25e8) positions the ring window: $7b1a=node, $7b1c/$7b1e/$7b2a/$7b2c
    from clamped indices (0..$d/$e) computed from the completion - slot range comes from the
    completion event.
- **ROOT DEFECT (model): the STEPIRQ trigger is too broad.** stepdone_tick re-arms on EVERY E800
  write with pc in $3690-$36c0 - i.e. on the wait loop's own waggle - manufacturing 746 spurious
  completions; each releases $369a instantly, so the $36ac pulse train NEVER RUNS, the head never
  moves, nothing is ever parked ($1b90: 0 hits), and my RINGDONE stamped an EMPTY slot whose
  zero-payload resume re-dispatches junk. The 1ms ring-stamp phase-2 + $7e1c!=0 stamp are the
  right hardware behaviors but at the wrong (spurious) times.
- **NEXT (design fork for Dave):** model the step generator honestly: (a) discriminate SUBMIT
  (which E800 op writes queue work + a $1b90 park) from the wait-waggle; (b) complete only after
  the physical step train / recal actually runs (drive stp_w edges from the $36ac nibble train
  are already wired under STEPBIT0); (c) on completion write the mailbox (01/ff/ff/ff), stamp
  entry bit7 + $7e1c, and let $bba resume the fw's OWN parked snapshot (never synthesize payload).
  Open q: whether the sys-floppy-check restore even NEEDS the park path, or completes inline via
  $36de when the completion timing is right (park may be the long-seek path only).
- Validated intact: default HLE boot untouched (all changes NOBYPASS/knob-gated except the two
  idcap format bytes + the stale-seek guard, which are NOBYPASS-gated too).

### Build #5 cont.2 - FORK RESOLVED (INLINE, no park); seek stage COMPLETES honestly (runs 13-17)
Dave's two-step ran: fix completion discrimination -> measure inline-vs-park. Results:
- **FORK ANSWER (run13): INLINE-36de fires, PARK-1b90 NEVER (0 hits).** The restore/seek completes
  inline; the $7e00 park/resume coroutine is NOT used by this op - machinery stays UNBUILT (the
  ring-stamp phase-2 was removed; see the stepdone_tick NOTE).
- **Completion discrimination (KEEPER, stepdone_tick + ch_w):** completion keys on the PHYSICAL
  step event, never E800 polls: (a) stp_w edges re-arm a 30ms quiesce (train done + settle);
  (b) the $3694 E800 loop-ENTRY write arms a ONE-SHOT settle for ZERO-step seeks (run13: read at
  cyl0/head at cyl0 - no edge ever fires, but the wait still needs its completion); the old
  any-E800-write trigger is log-only (E800-OPENTRY/WAGGLE).
- **Mailbox signature CALIBRATED (KEEPER): [$7ff2/4/6] = e7/00/00** (copied from the measured
  working 0x87 completion), which steers the IRQ2 walk to the $2632 BUMP path ([$71b2]++ -> the
  $369a inline release). The old ff/ff/ff triple steered to the $2570 RING-CLAIM path = the
  coroutine this op never uses (that mis-steer, not the ring itself, caused the run8 CHANSVC spin).
- **Settle = 1ms, not 30 (KEEPER):** run14 measured the 30ms bump landing AFTER the fw's short
  inline poll window closed (fw parks to the router idle; bump sat unconsumed; pipeline degraded
  to a 2.86s phase-mismatch cycle). With 1ms (run17): op-entry @6.40223 -> STEPDONE @6.40323 ->
  bump @6.40327 -> INLINE-36de @6.40331. The seek stage completes synchronously in 1.1ms - the
  55ms-burst shape. stp_w-train quiesce stays 30ms (must exceed inter-pulse gap; revisit when a
  real train is measured).
- **NEW FRONTIER (measured, run17): the pipeline self-advances but each SUBSEQUENT stage times
  out at ~2.6s** (consume $1d30 -> RDSTEP -> op-entry -> 1ms complete -> inline release ->
  2.6s SILENT gap -> next consume; 10 stages in 29s). The 2.6s = a $201c-class poll timeout on a
  hardware event not yet delivered. The ID-scan module ($89xx: A118/E802-bit15/IDCAP/$89f2) never
  runs on this honest path (run6 reached it only via the churn re-dispatch). NEXT: name the poll
  target in the 6.403->9.007 gap - STORAGER_PCHIST over the gap window, then model that one event.
- **Repro note: pcmx2 boot has a wall-clock-seeded intermittent** (runs 15/16: CPUAP diverges at
  ~1.34s, never reaches testend, screen empty - same class as the pc532 launch beachball). On a
  silent boot (no "testend" by ~7s), just relaunch; do NOT debug the storager for it.

### Build #5 cont.3 - WIDE PCHIST MAP (run19): it is ONE stage RETRYING, not ten stages
`STORAGER_PCHIST_WIDE=1 STORAGER_PCHIST_AT=6.35` (4us continuous sampling, histogram dumped+reset
at each INLINE-36de boundary - the sampler's wide mode + boundary dump are in pcsamp_tick/the
inline_36de tap, TEMP). Result: every window after the first is BYTE-IDENTICAL (n=652890, same
counts): ~50% in the ROUTER `$cd8-$ce0`, ~35% in the channel-walk scan `$1d24-$1d2a`. So the
"pipeline of ~10 stages" was ONE seek-retry loop with a ~2.61s period:
- The fw is NOT polling a device register; it is EVENT-IDLE (router + node scan). After our seek
  completion releases the $369a wait, the op RE-PARKS and waits for a NEXT channel event that
  never arrives; a ~2.61s timeout (0x201c-class) then restarts the read op from scratch -
  RDSTEP-op0 re-runs each cycle instead of advancing to op1.
- => The whole frontier is ONE missing hardware event: the one that advances the read op's
  microsequence from seek-done to the capture-arm phase (the $89xx module / E802-bit15 arm /
  $9400 rw-setup - all 0 hits on the honest path; run6 only reached them via churn re-dispatch).
- First window (6.35-6.4033) is the real dispatch burst ($b3a/$b90/$b4c/$bb0 doorbell/dispatch
  region) - healthy.
- NEXT: name the 2.61s retry timer's owner (who posts the timeout, what event would have
  preempted it): trace the retry edge - what runs at consume ($1d30) time and which timer/event
  source fires it; then follow the working direction: after $36de returns to $1dfa, what channel
  event does the op wait for (node+26 state at re-park + which walk transition would advance it).
  One event, then the proven lock-on template applies downstream.

### Build #5 cont.4 - TIMEOUT OWNER NAMED (runs 20-21): fw timer-list entry posts 0x18 into ch0-node[$18]
- **RETRYEDGE tap ($1d2a error-branch, prefetch-gated on the live exit condition): A0=$71c6 (the
  CH0/internal-disk-channel node), [A0+$18]=$71de takes error code 0x0018, with [$749c]=1 still
  BUSY and node+26=0** - every 2.61s, all cycles identical.
- **The poster is the fw's TIMER-TICK ISR ($2b58-$2bd4, rte):** walks a linked timer list at
  [$736c]; entry = {+0 countdown, +2 VALUE, +4 TARGET-CELL ptr, +8 next}; on expiry $2b84 posts
  VALUE->[TARGET] (here 0x18 -> $71de) + bsr $2aba (requeue). ($7b12/$7b14/$7b16 = a separate
  one-shot slot. Tail $2bb6+ = E800 watchdog kick + rte.) The 0x18 entry is armed when the ch0
  op starts ($3aca/$3d16 BUSY-set era); the wait at $1d1a exits on [$749c]==0 (SUCCESS: the $3c76
  queue-walker cleared it = the WHOLE op completed) or node[$18]!=0 (this timeout).
- **=> The one missing event, named from both sides: the ch0 channel op never completes because
  it never advances past its SEEK step.** Our seek-complete releases the $369a wait (measured,
  1.1ms), but the op's MICROSEQUENCE (the queued list 0058 0018 0054 004a 0042 0036) must then
  execute its NEXT step (op1 = the capture-arm) for $3c76 to eventually clear $749c. The
  executor (EXEC31f0/$26ae/$9884 family) runs op steps on the gate array's IRQ6 pump - the
  faithful IRQ6-at-the-right-time that the old model armed prematurely ($328e shortcut,
  retired). NEXT SESSION - ORDER IS THE TRAP (Dave): the executor ALREADY ran once without
  advancing (EXEC31f0 @6.43241, run14), so IRQ6 alone may NOT suffice - pumping an executor
  that spins on a bad descriptor is a no-op. (1) FIRST measure the EXEC31f0 stall: what it
  read (MICROSEQ walk entries vs the $72xx descriptor gate words) that made it not step;
  (2) THEN deliver IRQ6 at stepdone-completes-a-step - NEVER on enqueue (the $328e prematurity
  lesson: right mechanism, wrong timing).
- Probes added (TEMP, STRIP): NODE18ERR ($74a2 - note: that cell is a POINTER field on the $748a
  node, value $7440, NOT an error cell), NODE26ST ($74b0), RETRYEDGE ($1d2a).

### Build #5 cont.5 (runs 22-25) - dispatch/completion architecture FULLY DECODED; FRAME-IRQ4 pump
### built + measured; one remaining race
Static + live decode (probes: WALKENTRY $156a, WKR-ENTRY/CONT/CLR12 $3bfe/$3c6e/$3c76, RETRYEDGE
+q748a fields, FRAME-IRQ4):
- **The microseq WALK ($156a, caller $1942 + boot $dca/$e4e):** entries = offsets into the $192
  jump table, jsr per op, advance on D0==0 gated by [$71b2]==1 && [$71b6]!=0 ($15ec); entry $36 =
  WAIT op -> node+26=$a, cursor past it, stop ($15a0-$15a8 = run6's STATE26 pc=15a4); entry 0 =
  list end -> node+26=$c. Results $fe/$fd/$ff = wait-exits.
- **The honest path walks only the SHORT preamble list `0024 0026 0000` @$7250** (WALKENTRY:
  cursor $7224->$7256=past-end, ch0 state $c). The LONG read list (0058 0018 0054 004a 0042 0036,
  seen @$7256+ in run6) is built ONLY by the $5FC0 read-setup - which has NEVER dispatched on the
  honest path: **DISP-DD6 total = 1 (the 0x89); the 0x95 has never reached the $92 table.** The
  ROUTER ($cd4) has NEVER seen IDLE with the 95 pending (749c=1 at every 95 pass; run6's $5FC0
  dispatch came only from the churn's re-doorbell hitting an idle router).
- **The $748a QUEUE node's contract ($3c6e):** [+$12] IS $749c (the busy flag), [+$14].l =
  continuation fn ptr (jsr'd while nonzero; measured NULL), [+$18] = its timer-list entry ($7440
  = the 0x18-posting timeout). Busy-clear path $3c76 (clr + timer cancel at $2aba) is PROVEN
  executable (749c<-0000 pc=3c7c, run24/25 @6.40075/6.40204).
- **$3bfe = the IRQ4 walker** ($3c12: A0=[$743a]; ==[$7a14] pending-mark -> $1310/$1348 dispatch
  branch; else all roads reach the $3c6e completion processing - no early exits in $3c32-$3c6c).
  [$7a14] pending-mark set at $c88/$1418, cleared at $cfa/$133e/$13c6/$14d2. In steady state NO
  real IRQ4 passes occur (WALKENTRY/WKR-ENTRY steady-state lines = prefetch ghosts off the $3bfc
  rts; 7a18=1 vs 2 distinguishes two IRQ4 sources).
- **FRAME-IRQ4 pump BUILT (m_framedone, 2ms after stepdone if no new op-entry - m_stepdone_armed
  cancels): fires (11x) but is EATEN by a race:** by +2ms the retry flow has re-marked [$7a14]=
  $748a, so the walker takes the $3c16 dispatch branch (re-cycles the preamble op) instead of the
  $3c6e completion path. Also measured: the one mid-flow busy-clear (inside the IRQ4 ISR) doesn't
  help - the interrupted chan-svc context resumes and re-busies ($3aca) before the main loop
  router ever runs, so the router never sees the IDLE window.
- **THE REMAINING QUESTION (for Dave, with the full map now in hand):** what is the fw's intended
  main-loop moment for dispatching the still-queued 0x95? The candidates: (a) the FRAME-IRQ4 must
  land in the micro-window between the walk consuming the last completion and the retry re-mark
  (re-time the pump: fire from stepdone's IRQ2 completion IMMEDIATELY after the walk's own pass,
  not +2ms); (b) the read is MEANT to run as the chan-svc continuation and the long-list append
  happens from a continuation we haven't found (the [$14] ptr is NULL though); (c) the 0x95
  should never have been doorbelled during the restore (BUSYHOLD spacing artifact) - on real HW
  the CPUAP waits for restore DONE, doorbells the 95 at an IDLE router, and $5FC0 dispatches
  first try (matching run6's churn accidentally reproducing the REAL sequence!). (c) would mean
  the fix is in the completion-to-host timing (restore DONE post), not another 68000-side event.
- Probes added (TEMP, STRIP): WALKENTRY, WKR-ENTRY/WKR-CONT/WKR-CLR12, RETRYEDGE q748a fields,
  FRAME-IRQ4 (m_framedone timer + frame_done_tick - keep the mechanism if re-timed, strip the log).

### Build #5 cont.6 - Dave's two measurements RAN (runs 26-27 + run6 diff); fork settled to a PICKUP RACE
- **M1 (run6 diff): the 0x95 dispatched at [$749c]==0**, cleared by the WALKER (pc=3c7c) driven by
  REAL completion-IRQ4s: `IRQ4fire pc=003d4a` = the per-submit CHANCOMPLETE (today's L3039) firing
  for the CPUAP's retry-batch 87s, + hd_chan (L355). (a)+(c)'s (a) half confirmed: a 68000-side
  completion IRQ4 is required and sufficient to idle the router.
- **M2 (run27, direct host-IOPB write tap): NO host DONE is EVER posted honest-path** (the
  stale-guard suppressed the model poster; fw $0BF6 never runs). AND the CPUAP does NOT wait:
  87@6.4003 -> 89@6.4006 -> 95@6.4013, fire-and-forget within 1ms. STRONG-(c) REFUTED for the
  first attempt. MORE: **the 95's doorbell arrived at [$749c]==0** (walker had cleared the 89 at
  6.40071) - the router just never picked it up before the restore's chan-svc continuation
  re-busied (6.40188).
- **=> The residual defect is a PICKUP RACE, now exactly bounded:** idle windows with the 95
  pending EXIST (post-walker-clear 6.4007-6.4018; post-retry-cleanup 9.0068-9.0080, 130ms wide)
  but no router-IDLE dispatch occurs in them. Unmeasured last branch: does the main loop reach
  $cd4 in those windows at all (and if so which gate diverts), or is it stuck inside the
  doorbell-ISR/chan-svc flow for the whole window? ROUTER-cd4 tap was cap-exhausted (80) before
  the windows - NEXT: time-gate/un-cap ROUTER-cd4 around a retry cleanup + log its branch.
- **Also re-opened on measured ground: the cmd-95 CHANCOMPLETE SUPPRESSION block (ch_w ~2962-2998,
  "LLE EXPERIMENT: suppress premature channel-completion IRQ4s").** Its rationale ("they hold
  [$749c]=1 so 0x95 never dispatches") is inverted by run6's measurement: the per-submit
  CHANCOMPLETE is what CLEARS 749c via the walker. The suppression is why the honest path's
  re-submits ($3d4a, every retry) get no completion. Candidate fix: un-suppress for the re-submit
  case (or replace with the honest FRAME-IRQ4 timing) - re-evaluate together with the pickup-race
  finding; they are probably the same fix from two sides.
- Probe added (TEMP, STRIP): HOSTIOPB (m_bus write tap 0xfe780-7; NOTE it only sees
  STORAGER-side writes - CPUAP writes don't traverse the storager's bus space view).

### Build #5 cont.7 (runs 28-31) - WINDOW MEASURED (Dave's lean confirmed) -> SUPPRESSION REMOVED
### -> the 95 DISPATCHES; new gate = 0x14 stale-cmd at $d54
- **Window measurement (run29, un-capped ROUTER-cd4): exactly ONE idle-pass all run (the 89's).
  The 95 NEVER gets a router pass at 749c==0; in the 9.00-9.05 retry window all 399 passes are
  BUSY, starting only AFTER the re-busy.** The main loop never reaches $cd4 during the idle
  windows - trapped in the retry/chan-svc flow. Dave's lean confirmed: the suppression is the
  root, the pickup race its consequence.
- **CHANCOMPLETE suppression REMOVED for NOBYPASS** (ch_w cmd-95 kick: the HLE-only branch keeps
  the old m_dataop/hd_chan flow + early return; NOBYPASS falls through to the generic per-submit
  CHANCOMPLETE IRQ4 - the completion M1 measured clearing [$749c] via the walker). RESULT
  (run31): **the 2.61s retry loop is GONE (RETRYEDGE=0) and the router dispatches the 95**
  (ROUTER-cd4 IDLE with 71f0[0]=95 @6.40370 - first time ever on the honest path).
- **NEW GATE, measured + decoded: ERR1206 D0=0x14 at pc=$d66, re-hit every 2.35ms** (a fast
  fw-internal dispatch-retry loop replaced the 2.61s stall). $d54-$d60 decoded: 0x14 = the
  COMMAND-RANGE check (`cmd-0x70` must be in [0,0x3f)) - **a genuine 0x95 PASSES it (0x25)**. So
  the dispatcher is reading a STALE/GARBAGE cmd byte at gate time, 10us after the router pass saw
  [$71f0]=95. Suspect class: the CCB/D000-window clobber family (task-#2's "MODEL-doorbell-iopb
  writes 71f0 via m_d000<<1"; two such model DMA-target bugs were already fixed 2026-07-12).
  NEXT: tap $d54 with live D1 + its source register/address (prefetch-gate on the 0x14 outcome),
  then find WHO overwrote the cmd byte between the router pass and the gate - likely one more
  model write into the aliased $71f0 window.
- Boot-flake tally now 5 (runs 15/16/26/28/30) - every-other-run frequency; still relaunch-only.

### Build #5 cont.9 (runs 33-35) - 0x14 was NOT a clobber: the PENDING-IOPB RE-FETCH (KEEPER);
### $5FC0 RUNS honest-path; new frontier = the scan's ID-check MODE BIT
- **STALECMD tap (run33): D1=00 read from A0=$71c6 (the ch0 CCB) while [$71f0] held 9500 intact.**
  Not a clobber - the fw's pickup preamble ($c84-$c96) re-runs on a dispatch retry with the
  channel rotation on the OTHER node, stores the NEW dst into [$7a06] ($c8e), and expects the
  gate array to deliver the still-pending doorbell IOPB there (mailbox ptr + latch still live on
  real HW). The model fetched only once, at GO time, into the then-current [$7a06].
- **KEEPER: the pending-IOPB re-fetch** (write-tap on $7a06, pc-gated to the $c84-$c9a preamble,
  NOBYPASS-gated): on the fw's re-point, re-deliver the pending IOPB (0x1c bytes) to the declared
  dst. RESULT (run34): 0x14 loop GONE, **DISP-DD6 cmd=95 fires, $5FC0 RUNS (READ-START=1),
  READ-IOPB dumps the true IOPB (count=8 blocks, buf 0fc0dd) - the read dispatches through the
  $92 table on the honest path for the first time.**
- **New frontier (runs 34-35, measured): the scan loop runs but in NON-ID-CHECK MODE.** 457k
  E802-bit15 re-arms over the whole run (~80us cycle, avg speed drops to 55%), deliveries
  perfectly formatted (a1a1a1feff + rotating sectors) - but the $89f2 ISR's mode gate
  ($89fa-$8a02: btst #1 of [$799a]+$12) is CLEAR every pass -> the $8a38 re-arm path; the ID
  check ($8a14+) NEVER executes (LOCKDEC/LOCKRST/LOCKSEED all 0; the LOCKED-ON hits were
  prefetch ghosts past the taken $8a40 bne - tap discipline again). Run6's churn path had the
  bit SET. NEXT: who sets [$799a]+$12 bit1 (the ID-check mode arm) - which op of the
  $5FC0-built sequence, and what hardware event gates it. Also: the 80us re-arm storm is
  wasteful - the idcap delivery latency (30us) may be pacing it; revisit delivery timing once
  the mode bit is understood.
- Probes (TEMP, STRIP): STALECMD ($d54), lock-on tracer set ($6ab2/$8a32/$8a0e/$8a42), the
  IOPB-REFETCH logerror (mechanism itself is KEEPER).
- **UNIT-SELECT FOUND (Dave's lead question answered: FIRMWARE writes [$799a], not the model).**
  $e28-$e62 in the dispatch path: `cmpa $799a,A6; beq skip` (already-current -> no repoint);
  else btst #1,($12,A6) [the mode bit checked AT select], clr $7b22, bsr $14e2+$1242, E804
  unit-select hw write (ori #$f000,$79fa -> E804), then `[$799a]=A6` at $e5e. ($2218/$2230 = a
  temporary swap pair around bsr $6788 in the CH walk.) The model's only IOPB-area writes are
  the +2/+3 BUSY stamps (1573-1576, cmd-0x87 block) = the confirmed 8181 source.
- **THE DIVERGENCE IS ONE BYTE: host-IOPB byte[4] (the unit field).** Honest path's refetched
  IOPB: byte[4]=0x02 -> unit 2 -> UIB $6f90 (bit1 CLEAR) and $e2e's beq skips the repoint
  (already current). Run6 pristine: byte[4]=0x00 -> unit 0 -> floppy UIB $6c00 (0x37, bit1 SET).
  NEXT (one instrument): write-tap host IOPB 0fe784/5 + compare byte[4] at doorbell-time vs
  refetch-time - host-genuine (CPUAP wrote a different IOPB this attempt) vs model residue the
  re-fetch propagates. If residue: fix = the re-fetch delivers the pristine command image (or
  the dirtying write is removed); if host-genuine: decode why the CPUAP addresses unit 2 for
  the sys-floppy check on this path (its own retry state after the earlier BUSY/error views?).
### Build #5 cont.10 (run36) - byte[4] is HOST-GENUINE unit 2 (= floppy 0); the gate is the
### IDENTIFY's UIB flag byte
- **Three-way answered: byte[4]=0x02 is written by the CPUAP's own IOPB template builder
  (pc=fe3c0d, every command incl. the 87s).** Not residue, not a clobber. Unit derivation
  decoded ($dee-$e0a): unit = IOPB byte[4]; A6 = [$20a + unit*2] (the per-unit UIB table);
  err 0x11 gates at >=0x80/>=8. **Units 0-1 = HDs, 2-3 = floppies (IOREG comments) - unit 2 =
  floppy 0 is CORRECT for the sys-floppy check.**
- **Run6 reconciled: its "working" scan (byte[4]=0x00) was the HD-LABEL read for unit 0, not the
  floppy check.** The honest path is the FIRST time the floppy-check read itself has reached the
  scan loop. Its unit-2 UIB carries [$12]=0x45 (bit1 CLEAR) where the ID-check mode needs bit1
  SET - and UIB content is filled by the fw's OWN 0x87 identify interrogating the drive over
  the MODELED signals (f000 status bits / E01E / the interrogation data path).
- **NEXT SESSION (the gate, one cell): write-tap UIB@unit2+$12 ($6f90+$12=$6fa2 - confirm $6f90
  is unit 2's UIB via [$20a+4] first), name the fw pc that writes 0x45, read that interrogation
  code, and find which modeled drive-status input produces bit1 (floppy-type/ID-capable flag)
  on real hardware. Deliver that input faithfully; the identify then writes 0x37-class flags,
  the mode gate opens, and the scan's ID check (the proven lock-on) runs.
- Also confirmed in run36: the refetch propagated a model DONE-post (80 80 at bytes[2,3] between
  doorbell and refetch, cmd=87 era) - the pristine-image question stays open but secondary.
### Build #5 cont.11 (run37) - ARCHITECTURAL REFRAME: bit1 = HD-CLASS flag (ROM constant);
### the FLOPPY read uses the E000/SERDES + $9884 IDPUMP path, NOT the $7dac idcap channel
- **UIBFLAG taps: the flag bytes are ROM DEFAULTS copied at boot ($86e, the $84a init's 0x20-byte
  copy from $5de/$5fe/$61e). Unit 0 (HD): [$12]=0x06 - bit1 SET. Unit 2 (floppy): [$12]=0x44 -
  bit1 CLEAR. NO runtime writer.** Bit1 is a per-unit-TYPE constant, not an identify result:
  the ID-CHECK-counted lock-on ($89f2's $8a14 path + $7a0c countdown) is the HD/ESDI ID-CAPTURE
  mechanism. UIB table @$20a: $6c00/$6d30/$6e60/$6f90 (units 0-3); unit2 flag cell = $6e72.
- **Everything reconciles:** run6's scan = the unit-0 HD-label read -> bit1 set -> $7dac idcap +
  counted lock-on, working as designed. The floppy read (unit 2, bit1 clear) takes the $8a38
  re-arm path BY DESIGN - no ID examination in that ISR. **The floppy's ID matching is the
  $9884 IDPUMP parsing the E000/SERDES BYTE STREAM** (buf ptr from [$7a66]; "buf[1]==A1 then FE"
  - the comment on build_serdes_stream all along). Our idcap-to-$7dac deliveries feed the HD
  channel to a floppy read - wrong channel.
- **NEXT SESSION (the redirect, machinery already exists):** the model's SERDES path
  (m_serdes_stream, built lazily, served on E000 reads when m_serdes_active - ch_r ~2274) is
  the floppy's data path. Find the fw's floppy read-arm (E000 <- $2ff read-arm per the E000
  register notes; who sets m_serdes_active) and the IRQ6-pump-to-$9884 transition (the fw swaps
  the level-6 vector per phase - currently IRQ6 lands in the $89f2-family ISR). Deliver: arm
  the serdes stream on the fw's E000 read-arm + pump IRQ6 so $9884 parses; the E802-bit15/idcap
  LLE branch is the HD path - gate it to HD units (or by UIB bit1) rather than feeding $7dac
  for floppies. The [A1 A1 A1][FE][FF]-with-FF question REOPENS for the $9884 parser (it may
  want the raw stream format from build_serdes_stream - A1 A1 A1 FE c h r n crc, NO 0xFF - the
  $89f2 FF marker was the HD capture format; do NOT assume, measure $9884's live parse).
### Build #5 cont.12 (run38) - THE FLOPPY SCAN PHASE COMPLETES on its own channel (KEEPER:
### channel-select by UIB bit1); next = the scan->data phase advance
- **KEEPER (E802-bit15 LLE arm): channel select by the fw's own UIB flag** - bit1 of [[$799a]]+$12:
  SET -> the HD $7dac idcap path (run6-validated, unchanged); CLEAR -> the FLOPPY SERDES path:
  build_serdes_stream(cyl, side from [$7436]) + m_serdes_active=true, NO model IRQ, no $7dac.
- **RESULT: the fw drank the stream and SUCCEEDED** - $8924 pull loop reads E000 with serdes=1,
  op_ok=1 (E01E bit4 per byte), and the handler epilogue posted **D0=0000 SUCCESS @6.40414**
  (ERR1206 tap, ret=71c6). build_serdes_stream's format (A1 A1 A1 FE c h r n crc / FB data crc,
  NO 0xFF) passed the live parse at first contact - the pre-crash comment's belief VALIDATED
  (Dave's existence-isn't-validation check ran and the machinery passed). 2304B/16-sector FM
  track 0 from the real mx2-001 image.
- **NEW PARK (the next gate, same completion-advance class as the seek bump + FRAME-IRQ4):**
  after the scan success, FRAME-IRQ4's walker ran @6.40520, then ONE periodic EXEC31f0 @7.96
  and silence - no retry (the success path canceled the 0x18 timer), no host post, screen
  never even prints "no sys-floppy". The op parks awaiting its SCAN->DATA advance (the sector
  data pull + host DMA + $0BF6). IDPUMP-irq6 ($9884) still 0 - the data-phase parser hasn't
  run. NEXT SESSION: (1) did run38's read go through $5FC0 (DISP-DD6/READ-START - ungrepped) or
  chan-svc? (2) node+26 / cursor state at the 6.406 park; (3) what advances scan->data on the
  SERDES channel - candidates: the fw re-arms E000 for the DATA window (a second E802-bit15 /
  E000 mode write the model should answer), or the IRQ6 pump to $9884 for the sector-data
  parse; measure which arm the fw performs after the scan success, then serve THAT channel.
### Build #5 cont.13 (run39) - record-ready IRQ6 pump BUILT (repeats while SERDES armed); IRQ6
### lands in the scan ISR, not $9884 - the ISR's internal phase branch is the next gate
- pump_tick now repeats (~700us record time) while m_serdes_active; armed at the floppy bit15 arm.
  Run39: pump fires (speed 80% = churn) but IDPUMP-irq6/$9884 = 0 - the level-6 delivery lands in
  the $89f2-family ISR which (floppy, bit1 clear) takes the $8a38 re-arm path. The scan->data
  branch is selected INSIDE the IRQ6 ISR by a phase cell (old notes: "$9984 reached synchronously
  off $369a/$1f82 after $9884 sets $7a68"; $89f2's $8a04 branch tests node[$20] bit14; $7950
  bchg at $28e0-era...). NEXT SESSION: trace the IRQ6 ISR entry ($89f2 head) with the data-arm
  state live - which cell must change after the $89b6 data-arm so the ISR routes to $9884
  (candidates: $7a68, node[$20] bit14, the $79f8/E802 mode shadow) - find the fw's own writer of
  that cell in the working direction (who sets it between arm and parse on the intended flow),
  deliver ITS gating hw event if one is missing. Also verify the pump doesn't regress the scan
  phase (IDCHK count in run39 - ungrepped) and consider closing the window (E000<-$23f) handling.
### Build #5 cont.14 (run40 + statics) - THE LEVEL-6 CONTRACT FULLY DECODED: soft-vector $298c
### toggler + C800 count-expiry interrupts (the pump's correct timing)
- **TRAMP6: [$7304] = $298c constantly; DSETUP-$9602 never runs.** $298c = the IRQ6 PHASE
  TOGGLER: `bchg #0,$7950; beq $89f2 (ID) / bra $92b4 (DATA)` - records alternate ID/data down
  the track and so do the handlers. The scan ISR's $8a38 `clr $7950` = "stay in ID phase" while
  searching. ($9602/$9884 = the OTHER data path's vector swap - $99xx/DATASTEP family; not this
  flow's first stage.)
- **The data-arm window ($8990-$89f0) fully decoded:** D800<-[$7434]; **C800<-[$741e] = the
  COUNT/TERMINAL**; E802 word rebuilt with bit15; then per $796e: >0 -> **the fw OPENS ITS OWN
  E000 window (`E000<-$2ff`)** - the model's existing serdes-arm condition! - reads E000, writes
  [$7a48] mode back; $742e<-1; arms the $7986 timeout {0x32, 0x201c}; rte.
- **=> The ENDEC's IRQ6 contract is COUNT-EXPIRY, not free-running:** the gate array interrupts
  when the C800-programmed byte count has been clocked from the stream; each expiry toggles the
  $298c phase. The blind 700us pump is the right line with the wrong timing - it fires
  regardless of count, and each $89f2 landing clr's the toggle so $92b4 never runs.
- **NEXT SESSION (the pump correction, one mechanism):** model the C800 counter: latch the count
  at the C800 write; on the fw's E000 $2ff arm (or bit15 arm), schedule IRQ6 at count x
  byte-time (FM ~32us/byte at 250kbps; measure against the fw's $7986 timeout 0x201c scale);
  serve stream bytes against the SAME counter so the E000 pulls and the interrupt agree.
  Then: does the toggle survive (the arm path doesn't clr $7950 - check who last wrote it
  before the data-arm), does $92b4 run on the second expiry, and validate $92b4's data-record
  parse live (the FB record check - Dave's sharpening #2, still unearned). Strip the 700us
  free-run from pump_tick when the count model lands.
### Build #5 cont.15 (run41) - count model built; C800-count=0 -> the source is the $7696
### PER-UNIT GEOMETRY TABLE, empty for unit 2 (the real identify-content gap)
- Count model (KEEPER shape, single-shot, coherent cursor): m_sd_count latched from m_c800[0] at
  the floppy arm, m_sd_base = cursor, expiry at count x 87us (provisional), cursor forced to
  base+count at expiry. RUN41: **C800-count=0000 at both arms - [$741e] was empty.**
- **The count's source, decoded:** the scan-phase setup ($79d0-$7a06; twin at $7ef8-$7f1e) does
  `[$7424]=unit; D0=[$7696 + unit*6]; [$741e]=D0>>1` - **$7696 = the per-unit GEOMETRY table
  (6 bytes/unit)**, plus `$7968=1` (the hook cell!), `clr $7950` (the toggle clear lives HERE,
  in the scan setup - not the ISR alone), `[$79a4]=[$79a2]` (retry budget seed).
- **Unit 2's $7696 entry is EMPTY -> count 0 -> no expiry -> the park.** The table is populated
  by the 0x87 IDENTIFY's drive interrogation - and the old Phase-C note named this exact gap:
  "the C++ SHADOW latch (m_unit_heads/_spt/_secsize) is the only non-faithful part of the 0x87
  block". The model kept the floppy's geometry in C++ members; the fw's own interrogation never
  landed it in [$7696+12].
- **NEXT SESSION:** (1) grep $7696's writers (the identify's geometry store) + read what the
  interrogation computes it FROM (the drive's identify response bytes over the modeled
  channel); (2) measure the honest-path 0x87 for unit 2 - where its interrogation got its
  answers and which response bytes are missing/wrong; (3) serve the floppy's identify response
  faithfully (geometry: 16 spt x 128B FM track 0 era) and retire the C++ shadow latch per the
  LLE mandate. Then the count arms, the expiry fires, the toggle survives (verify the $7a02
  clr $7950 interaction - the SCAN setup clears it; what re-sets it for the data phase), and
  $92b4's FB-record parse gets its live validation.
### Build #5 cont.16 (run42) - the $7696 table has NO runtime writer at all; the fw has a
### DEFAULT-GEOMETRY fallback ([$77c6], the $555e entry) - the count fork is table-vs-default
- GEOM taps (units 0 AND 2): only the fw startup clear ($73e) + RAM-test ($9cea/$9d14). **No
  identify, no anything, writes the geometry table on the honest path - for ANY unit.** So the
  table is host-populated later (cmd 0x86 SET-PARAMS, the cmd the dispatch gates special-case -
  SINIX-era) and the BOOT-era reads must use the fw's DEFAULTS: $5548 = the table-lookup entry,
  **$555e = the alternate entry reading [$77c6] (a default-geometry block ptr)** - two entry
  points, caller chooses. The scan setup ($79d0/$7ef8 twins) read the EMPTY table -> count 0.
- **NEXT SESSION:** trace the $79d0/$7ef8 scan-setup CALLERS - is there a sibling setup that
  routes via the [$77c6] default path for un-parameterized units (the boot floppy check MUST
  work pre-0x86 on real HW), or does [$741e] get seeded from the default block by a path we
  haven't run? Read [$77c6]'s pointee + its ROM default content; check the five [$741e] writers'
  ($79ee/$7afe/$7f10/$84a6/$864e) enclosing routines for a default-vs-table branch. The fix will
  be routing/serving whatever input selects the default path - NOT poking the table (mandate).
  ALSO still queued: the shadow-latch retirement part 2 (m_unit_* readers re-home), toggle
  re-set namer, 87us validation, $92b4 FB-record check.
### Build #5 cont.17 - PREMISE CORRECTED: [$77c6] = the SECTOR-BUFFER FREE LIST; C800 = DMA
### TERMINAL ADDRESS (not a count); the gap = unit 2's BUFFER ALLOCATION never ran
- **[$77c6] decoded ($5cc6-$5d02): a free-list builder** - 10 x 0x200-byte buffers at $5800+,
  linked {bufptr,next} nodes at $77ca+; [$77c6]=head, [$77c8]=tail. NOT default geometry.
- **Field map corrected:** $7696 entry word[0] = the unit's ALLOCATED BUFFER ptr (>>1 = the
  word-address convention, same as D800/D000); [$741e] = that buffer word-addr; **C800 = the
  DMA TERMINAL ADDRESS** (the model's own comment: "C800 scatter/gather translation register
  file"; the F000 bit12 machinery: "DMA transfer address reached the terminal" - m_dma_term!).
  The $5548/$555e twins = "set D000 from the unit's buffer (table) vs from the free-list head".
  So the count-expiry premise revises to TERMINAL-REACHED: the ENDEC DMAs stream bytes toward
  the C800 terminal; IRQ6 when the address hits it; count = terminal - base.
- **The actual gap: unit 2's buffer ALLOCATION** - whoever pops the free list into the unit's
  $7696 entry (the open/first-use flow) never ran on the honest path. NEXT SESSION: find the
  allocator (the free-list POP: reads [$77c6], follows ->next, stores bufptr into the unit
  entry - likely in the 0x87/open or first-read path; grep readers of $77c8/$77ca-chain +
  the $5658 lea site); measure why it doesn't run for unit 2 (another gated hw input?); and
  REVISE pump_tick to terminal-distance timing once [$741e]/C800 carry real addresses. The
  87us/byte constant survives; the multiplicand becomes (terminal - base) bytes.
### Build #5 cont.18 - the ALLOCATOR FOUND: free-list node-move $5b5c, staged from an ISR gated
### on F000 bit11 = the 8253 TIMER OUT (modeled: m_timer_out)
- **$5b5c = the buffer node MOVE:** unlink head from source list (A1=cell), append to dest
  (A2): $77c6 free -> $77c4 ACTIVE queue; under $791a -> the per-unit slots $74c4+[$7926]*8
  (the kickoff table!). $5b08/$5ba2 = more list ops in the same family.
- **The pop site ($5658, inside an rte ISR): gated on D0 bit $b = F000 bit11 = the 8253 TIMER
  OUT** (the model's f000 handler: "bit11 = 8253 timer OUT" - m_timer_out, and m_pit exists,
  gate1 already wired in the step path) - or the $791a flag. Then bsr $48ea.
- **NEXT SESSION (the coupling check, Dave):** (1) name the $56xx ISR's vector + when the fw
  programs the 8253 so timer-out stages buffers (the open/identify flow? - if yes, the
  allocation and the shadow-latch retirement are ONE gated input as suspected); (2) does the
  honest-path fw ever program the PIT for this (m_pit writes in the run logs) and does
  m_timer_out ever assert; (3) then deliver the PIT/timer behavior faithfully and watch the
  free-list pop populate the unit's buffer -> [$741e]/C800 carry real addresses -> revise
  pump_tick to terminal-distance -> the chain runs.
### Build #5 cont.19 - the READ'S OP-QUEUE BUILDER decoded ($3a30-$3aba): $55d8 is a QUEUED OP;
### plumbing verified live (PIT1-OUT0 -> m_timer_out -> F000 bit11 all working); the circle
- **Plumbing check (Dave's routing question): BOTH modes pass** - the fw programs PIT1 (OUT0
  toggling ~76ms in run42, driving IRQ1 -> the $2b58 tick ISR), and m_timer_out IS assigned
  from OUT0 -> F000 bit11 live. But NOTE: the $5658 pop gate tests bit11 of the SOFT status
  [$792e] (seeded 0x2000 by the builder), NOT F000 - the earlier F000-bit11 identification was
  wrong one register over; [$792e] is fw-owned state.
- **$3a30-$3aba = the read's OP-QUEUE BUILDER** (the $328e enqueuer the retired pump-shortcut
  hooked): enqueues [$7940]=handler/[$7942]=slot pairs - $299a @$18, $29ea/$29dc/$29ce @$14
  per [$796e] (density variants - the $29xx bchg-toggler TABLE, each op a two-phase toggler),
  **$55d8 @$c (the tick/buffer-stage "ISR" is a QUEUED OP)**; seeds [$792e]=0x2000; `clr $77c6`
  (free-list re-init); [$7434] = ($7d9d+1[+2 if UIB bit1])>>1 (the parse-buffer math - the HD/
  floppy +2 offset!); [$7450]=1, [$7a34]=1, [$7432]=UIB[$e6].
- **THE CIRCLE to resolve next session:** queued ops execute on the level-6 machinery; IRQ6 =
  terminal-expiry; the terminal needs the unit's buffer; the buffer pops inside queued op $55d8.
  The circle breaks somewhere - candidates: the fw's SYNCHRONOUS executor (EXEC31f0/$352e runs
  the queue head without IRQ6 - the MICROSEQ walk!), or the $2b58 IRQ1 tick (runs $736c
  timer-list callbacks - could dispatch the eligible op), or the first IRQ6 comes from a
  different source (the scan phase's synchronous completion?). MEASURE: did $3a30 run in
  run38-42 (tap it); is the op table ($72ec+) populated with $299a/$55d8 after the read setup;
  what consumes op slots $c/$14/$18 (the MICROSEQ list offsets are $58/$18/$54/$4a/$42/$36 -
  SLOT NUMBERS! the walk jsr's [$192+slot]... no - the walk used the $192 table; the $328e
  ops land in $72ec+... reconcile the two op systems: $192-table ops (walk) vs $72ec queue
  ($328e) - likely setup-ops vs runtime-ops).
### Build #5 cont.20 - THE MAP COMPLETES: three interrupt levels, three soft-vector slots, all
### builder-installed; the missing inputs are IRQ3 (index) and IRQ5
- **$328e slots = the soft-vector FILE: slot $c = $72f8, $14 = $7300, $18 = $7304.** The ROM
  vector table: **$6c (IRQ3 autovector) -> $26a2 -> [[$72f8]]; $74 (IRQ5) -> $26a8 -> [[$7300]];
  $78 (IRQ6) -> $26ae -> [[$7304]]**. The builder ($3a30) INSTALLS: IRQ3 <- $55d8 (the
  buffer-stager - counts revolutions, runs the $7922/$791e divider, clears descriptor gates,
  pops buffers on [$792e] bit11 = **the INDEX-PULSE handler**); IRQ5 <- $29ea/$29dc/$29ce
  (density-variant toggler); IRQ6 <- $299a (toggler).
- **The model has NEVER raised IRQ3 or IRQ5 on the floppy path.** The circle dissolves: nothing
  bootstraps off IRQ6 alone - the per-revolution IRQ3 stages buffers (op $55d8), and the
  record-class events land on 5/6 (likely ID-record vs DATA-record completion, or terminal vs
  record - MEASURE which is which before wiring: the $29xx entries' two branch targets name
  their consumers).
- **NEXT SESSION (the last input set):** deliver IRQ3 per index pulse while the armed floppy
  spins (200ms/rev at 300rpm; the model's fdd idx_r already feeds F000 bit4 - wire the same
  source to IRQ3, window-gated like the SERDES); determine IRQ5's event (the $29ea/$29dc/$29ce
  variants' targets) and deliver it; retire the blind IRQ6-only pump; then the builder's whole
  op set runs on its real events - buffer pops, [$741e]/C800 arm, the togglers alternate, $92b4
  parses (FB-record validation!), host DMA, $0BF6. Everything downstream measured or one
  record-check away.
### Build #5 cont.21 - IRQ5 NAMED; THE WIRING SPEC (the next session's build, complete)
- **IRQ5's toggler branches ($29ce family): phase0 = $8552** (strobe/prep: E802 bit11/12 pulses,
  density-selected by [$796e]; re-arms the $7986 timeout {0x32, $201d}); **phase1 = $8690 = THE
  DATA-TRANSFER KICKOFF** (gated on [$741c] from the scan setup): programs D800<-[$7434],
  C800<-[$741e], E000<-$2af (mode pulse), read, E000<-$2ff (window open). IRQ5 = the
  RECORD-BOUNDARY event; its phase1 IS the scan->data advance.
- **THE WIRING SPEC (Dave's coherence principle - ONE rotational source, three consumers):**
  1. A track-position clock: pos(t) = ((t - t_index_origin) / BYTE_US) mod stream_size, with
     BYTE_US = 200ms / stream_size (300rpm) - rotation-faithful by construction. The E000 read
     handler serves stream[pos(t)] (RETIRE the pull-incremented m_serdes_ptr); the sector/record
     boundary positions come from build_serdes_stream's own layout (record offsets recorded at
     build time).
  2. IRQ3 at pos==0 (the index) while the window is armed - same physical source as F000 bit4.
  3. IRQ5 at each record boundary crossing (ID-record starts) while armed.
  4. IRQ6 at the OTHER record event (data-record complete / terminal-reached - C800-derived);
     RETIRE the blind 700us pump and the m_sd_count expiry.
  5. All three window-gated (m_serdes_active; E000<-$23f closes); per-unit only when the floppy
     channel is selected (UIB bit1 clear).
- **BUILD CAUTIONS (Dave, don't relax the rule on the last wire):**
  (a) The record-boundary offsets and the E000 pull positions must be THE SAME positions -
      build_serdes_stream records the boundaries AT BUILD TIME and pos(t) serves the same
      array; never two computations of the layout (the free-run's mid-record failure class).
      Verify IRQ5 phase-1 fires where $92b4 expects the data record to START.
  (b) Confirm the scan setup sets [$741c] on the honest floppy path BEFORE assuming $8690's
      gate opens (it's fw-managed; the scan setup at $79f6-era does `$741c=1` - verify live).
  (c) Watch the phase-0/phase-1 alternation - the toggler must land phase-1 ON the data
      boundary (coherence one interrupt in); log the toggle state per IRQ5 on first flight.
- **Flake coupling: a HEDGE, not a plan (Dave).** The flake lives in the CPUAP's first ~1ms at
  boot; this wiring is read-time (t~6.4). It closes task #7 only if the leak lived in the
  STEP/timer inputs this clock replaces. Re-run the 3s flake pre-check batch after the wiring
  and let it say - do NOT mark #7 closed on the hypothesis.
- Everything downstream of these three wires is measured working except $92b4's FB-record parse
  (validate live when it first runs - the last unearned record).

### Build #5 cont.22 (runs 43-44) - THE WIRING IS BUILT AND PLAYING: all three keys live,
### handlers cycling per-record on real rotational events; $92b4 runs (first time ever)
- **Run43 (first flight) taught the load-bearing lesson:** the $89xx scan/arm code is ITSELF
  interrupt-handler-phase code - runs 38-41's "scan" was started by our stray IRQ6s. Fix:
  rotation starts at READ ACCEPTANCE (the doorbell, floppy-unit test = IOPB byte[4]&3 >= 2;
  [$799a] is stale at doorbell time), not at the arm. ROT-START keeper in the doorbell block.
- **Run44: the machine CYCLES on the three-interrupt instrument.** ROT-START @6.4013 (2304B,
  33 marks); 12 IRQ5s + IRQ6s consumed via [[$7304]]=$298c; **$92b4 (OPH1) EXECUTES - first
  time ever - once per data record (12.5ms cadence), each followed by a D0=0 success epilogue.**
  The cross-level toggle alternation measured: $7950=0 at every IRQ5 fire -> IRQ5 always lands
  $8552 (prep), IRQ6 always $92b4 (parse) - a LOCKED alternation by construction. **$8690 (the
  transfer kickoff) requires the fw to set $7950=1 out-of-band - which it does when $92b4
  MATCHES THE TARGET SECTOR.** $92b4 runs with D800<<1=0000 (no transfer buffer yet - correct,
  pre-match) and returns keep-going each record: THE PARSER ISN'T MATCHING.
- **NEXT (Dave's sharpening #2, now live): validate $92b4's parse against the stream.** Read
  $92b4's code: what buffer/stream position it examines ([$7a66]-pointed?), what record format
  it expects (the FB data record? or the ID it matches C/H/R against $7438/$7436/$7428?), and
  diff against what the rotational E000/stream serves at its execution moment. The mismatch is
  the last gap: fix the SERVED record (or the position sync), never the parser. Note the mark
  map may need the ID/DATA irq assignment swapped (5<->6) once $92b4's true consumption is
  known - measure first.
- Strip note: the ROT-IRQ5 caution-c log + TRAMP6/ROT-START logs join the strip list.

### Build #5 cont.23 (run45) - THE READ IS BUFFERING SECTORS and WALKING ITS PHASES
- **OPH1CAP: [$742c]=1 (path B - the counting path) every record; [$741e] WALKS BUFFER ADDRESSES
  0x2000->0x2900 (+0x100 word-addr = +0x200 bytes = the free-list buffer stride) per data
  record** - the fw is buffering successive sectors into local SRAM $4000+; $79a8 (seeded from
  the IOPB's 8-block count) counts them down. Position coherence EXACT: pos +144/record,
  mark# +2, byte=A1 (next record's sync) at every fire. Targets C/H/R = 0 (not consulted in
  this phase).
- **After the countdown: the fw ADVANCES PHASES** - new per-record handlers cycle at the same
  12.5ms cadence: $937c/$9384/$938c (the $9370 path-B continuation) + $7f78/$7f84/$7f90 +
  $7bdc = the phase-0 targets of the OTHER toggler family ($29a4/$29b2/$29c0 -> $7fc6/$7fee/
  $8018 phase-1s) - the vectors were RE-INSTALLED by another $328e builder pass. The machine
  is walking its phase sequence on the honest three-key instrument. Screen still testend: no
  host completion yet - the walk hasn't reached the host-transfer/$0BF6 phase (or its phase
  awaits another input - likely the SRAM->host Multibus DMA the fw fires itself at
  $13be-$13ce per the old notes).
- **NEXT SESSION:** same method, next phase: capture the $7f78-family's execution moment (what
  the phase-0 preps examine/await; what flips their togglers to the $7fc6-family phase-1s);
  check whether the SRAM->host DMA path ($13be-$13ce) runs and whether the model's Multibus
  master-write side serves it; watch for the completion chain ($0BF6/host DONE/E802 bit7) and
  the CPUAP's "no sys-floppy"/label-read verdict on REAL DATA for the first time.
- 5<->6 mark note (Dave's re-check): the observed consumption is CONSISTENT (IRQ5-marks drive
  the prep/counting handlers, IRQ6-marks the $92b4-class) - no swap indicated by the live
  cadence; the record content (byte=A1 at the mark) serves the NEXT record's start, which the
  handlers tolerate. Revisit only if a phase shows a content mismatch.

### Build #5 cont.24 (run46) - the transfer phase PARKS in the $808a check chain; instruments set
- DMAFIRE ($13be) = 0, HOSTDATA (bus writes to the 0fc0dd buffer) = 0 - the SRAM->host transfer
  has not run. The $7f78 phase-0 flows into $808a, whose gate chain routes $8214 (transfer-ish)
  vs $80c0 (the $74c4 kickoff-slot walk, slot statuses $40/$80): checks $79b8, $79b6, $79a8,
  $79b0, UIB[9]. One of these (or the slot statuses - populated by the $5b92 buffer-to-slot
  move under $791a) blocks. **NEXT SESSION (one tap): capture $808a's execution moment - all
  five flags + the unit's $74c4 slot words - the blocking flag names the next (likely gated,
  fork-2-shaped) input.** DMAFIRE/HOSTDATA taps stay armed to catch the transfer the moment it
  runs; expect the completion chain ($0BF6/host DONE/E802 bit7) to reconnect to the
  already-built model machinery per Dave (no new completion logic - "does the transfer run,
  then does the built completion fire on real bytes").
- Boot-flake tally: 2 more in run46's wrapper (auto-retried). The post-wiring flake re-test
  (the hedge) still queued - run the 3s pre-check batch when the read completes.

### Build #5 cont.25 (run47) - transfer starves at a UIB CONFIG BIT: [$792e] = UIB[$20] & 0x2800
- **XFERGATE whole-vector: $79a8=0 (buffering COMPLETE - transfer-phase gate, no upstream
  regression, Dave's split answered).** The slot walk runs per record hungry: slot0 arrived
  staged (status 0080), consumed on pass 1; statuses 0000 thereafter - the $5658/$5b5c stager
  never runs again ([$792e] bit11 clear; seeded 0x2000).
- **[$792e]'s real source ($4a66 builder - the transfer-phase setup, and the SECOND $55d8
  install site): [$792e] = UIB[$20] & 0x2800 (via [$799c], the SECOND unit ptr from dispatch)
  | 0x200 if [$790e]==0.** Bit11 of the pop gate = **UIB word $20 bit 11 - a per-unit RUNTIME
  config bit** (beyond the 0x20-byte ROM-default copy; writers: $3abc-era sets UIB[$20]=1 -
  bit11 clear). Unit 2's bit11=0 -> one-buffer staging -> the transfer starves after slot 0.
- **NEXT SESSION:** who sets UIB[$20] bit 11 (0x0800) on the intended flow - per-unit runtime
  config: the 0x86 SET-PARAMS, the identify, or a mode the multi-sector read op sets (bit11
  smells like "multi-buffer/queued-transfer enable"); find its writer + the input it derives
  from; also decode [$799c] (the second-unit/dual-op pointer semantics) while there. Then the
  pop stages all 8 slots, the walk feeds the master-write, DMAFIRE/HOSTDATA (still armed)
  catch the transfer, and the built completion chain carries real bytes to the CPUAP.
### Build #5 cont.27 (run50 + statics) - the EXECUTOR RUNS TOO (SLOTWR's "append" = the walker
### marking nodes at $32da); widened HOSTDATA still 0; NEXT = the model's DMA SERVICE question
- **$32ac = the executor entry and it ALREADY RUNS** (run49's pc=$32e0 writes = the $32da
  `($2,A2)=0x80` node-activate; the tap logs the next pc). The walk dequeues ($32e6-$3314 link
  surgery) and dispatches nodes to the PROCESSOR $3474: `bclr #7,$7b0e` -> first-entry work
  path $34c8 (UNREAD); else re-list onto $74ae (a further queue).
- **HOSTDATA widened to 0fc000-0fc9ff (the full 8-block target): STILL 0. DMAFIRE 0.**
- **NEXT SESSION - CHECK THE MODEL'S SIDE FIRST (the structural candidate):** when the fw fires
  its SRAM->host DMA (the $13be-$13ce E-register writes, or wherever the op chain lands), the
  COPY is gate-array work = MODEL work. Does ch_w service that fire with an actual
  SRAM->m_bus copy? If the master-write DMA service is unimplemented, no firmware correctness
  lands a byte - "everything runs but nothing arrives" is exactly its signature. Search the
  model for the DMA-fire service before reading more op-chain ($34c8+) statically. Then:
  DMAFIRE/HOSTDATA catch the first bytes, the built completion carries the verdict.

### Build #5 cont.28 (runs 52-53) - THE DEATH NAMED AND FIXED (ungated IRQ3 at the index wrap);
### the ENABLE MAP MEASURED; the machine now runs its fullest honest sequence
- **Run52 frame: SP=7d8a {SR=2308, PC=$0000008c}** - the CPU was executing IN THE VECTOR TABLE:
  the FIRST IRQ3 ever delivered (at 6.60005 = exactly the 33rd 200ms rotation wrap) jumped
  through the uninstalled [[$72f8]] -> wild execution -> exception -> the $25e self-loop. The
  fw hadn't installed the IRQ3 handler because IT NEVER ENABLED index interrupts in this flow.
- **KEEPER: the rotational IRQs gate on the fw's E802 enable shadow [$79f8]** (the ENDEC's
  interrupt enables - hardware truth, not a workaround). MEASURED map (run53 ROT-MARK):
  IRQ5 fires under 79f8=42d7 (bit9), IRQ6 under 4ad7 (bits 9+11) - the handlers toggle each
  other's enables per phase ($8552 ori #$800, $92b4 ori #$8a00); IRQ3's family (bits 8/10,
  from $55d8's own ack andi #$faff) is NEVER set in this flow -> the index gate (en & 0x0500)
  is correct. TODO next: gate IRQ5/6 on bits 9/11 too (currently delivered ungated; the fw's
  masks align in this flow but gate them for faithfulness).
- **Run53: DEATH-25e = 0; the machine survives and runs the FULLEST honest sequence yet:**
  IRQ6 -> $92b4 -> Q352e -> Xdisp -> EXEC31f0 -> IRQ4 -> Xdecr -> RDSTEP-op0 (legitimately!
  the op that entered this saga as a prefetch phantom) -> IRQ2 -> CH-PROCESS -> GATE260c,
  cycling; D000 re-pointed to the ch0 CCB ($71c6); alive past 7.97s. HOSTDATA/DMAFIRE still 0
  - the transfer is further down this now-running sequence. NEXT SESSION: follow where this
  deeper flow settles (PCHIST_WIDE again if needed), gate 5/6 on their bits, and the transfer
  should be stages - not mysteries - away.

### Build #5 cont.29 (run54) - all three levels fw-gated (KEEPER complete); the machine settles
### in the $16xx MAILBOX-SERVICE loop
- IRQ5 gated on bit9, IRQ6 on bit11 (measured map now fully applied). DEATH=0; healthy.
- **The settle map: $1608/$160c/$1612/$1634/$1638/$163c dominate (33%+) with 218 distinct pcs**
  = the fw's ch1 MAILBOX-SERVICE loop (the same pcs that wrote 7ff8 in the run6-era logs:
  "7ff8<-1300 pc=001608", "7ff8<-000f pc=00163c"). The fw finished its disk-side work and is
  cycling the host-facing handshake. HOSTDATA/DMAFIRE still 0; "no sys-floppy" not printed.
- **NEXT SESSION:** read $1608-$1640 (what the mailbox loop writes/awaits - the CPUAP-facing
  completion handshake?); whether it's POSTING (the completion trying to reach the host - then
  check the host-side visibility of 7ff8-family writes) or WAITING (on a host ack the CPUAP
  never sends because it never saw a DONE); the transfer's place in this sequence (before or
  after the handshake). The machine is running and host-facing - the remaining distance is
  handshake decode, not machinery.
- **cont.29 CORRECTION (the read ran): $15fe-$1644 is NOT a mailbox service - it is the WAITER
  WAIT-SCAN.** A0 walks [$727c] through the waiter-pointer table $72d6-$72ec (11 cells - the
  $37xx builders' registry): registered cell + gate word [[cell]]==0 + parked node (state $a
  at [$71b6]/[$71bc]) -> RESUME at $1646 (cursor update + descriptor dispatch). The run6-era
  "7ff8-writer pc=1608/163c" association was a WRITER-ATTRIBUTION ARTIFACT (host pio writes
  logged with the 68000's incidental scan pc). So the machine is PARKED AT A WAIT (state $a)
  on a registered descriptor whose GATE never clears - the descriptor handlers include $9984
  (DATASTEP). NEXT SESSION OPENER (one tap): dump the waiter cells $72d6-$72e2 + each gate
  word + [$71b6]/[$71bc]/their node states in the settle era - names WHICH descriptor is
  registered and what event should clear its gate (the $55d8-tick cleared [[$72e2]]/[[$72e6]]
  statically; the $89f2 success cleared [[$72de]]; per-descriptor events, one of which is owed).

### Build #5 cont.30 (runs 55-56) - the WAITDUMP + the FRAME-IRQ4 A/B
- **WAITDUMP (settle era):** [72d6]=727e{g=0000,h=$7964}, [72d8]=7286{g=ffff,h=$3dbc},
  [72da]=728e{g=0000,m=$71de(!the ch0 ERROR cell as its mode),h=$9188}, [72dc]=7296{g=ffff,
  h=$94ec}, **[72de]=729e{g=ffff,h=$9984 DATASTEP}**, [72e0]=72a6{g=ffff,h=$9398}; cells
  $72e2+ empty. [$71bc]=$71c6 st=00 (not $a); [$71b6]=0.
- **FRAME-IRQ4 A/B (STORAGER_NOFRAME4): identical dump, no regression anywhere** - FRAME-IRQ4
  is REDUNDANT since the CHANCOMPLETE un-suppression (real per-submit IRQ4s) -> STRIP CANDIDATE
  (keep the env gate til cutover), but NOT the state-cleaner.
- **Reading:** the two gate-0 descriptors = likely POST-CONSUMPTION residue (resumed earlier
  while state was $a); the wait-scan = the fw's IDLE loop; the LIVE question = which events
  clear the four ffff gates ($3dbc/$94ec/$9984/$9398 - mode-$0248-class; the $9188 waiter
  watches [$71de] the ch0 error cell). The DATASTEP gate [[$72de]] is cleared by the
  ID-MATCH-SUCCESS event - on the HD channel that's the $89f2 counted lock-on's $8aa4; the
  FLOPPY channel's equivalent clear site is the next decode (the $8924-pull/$92b4 world's
  success path - which of them clears [[$72de]]-class gates and on what condition).
- **NEXT SESSION:** (1) static: find all writers of the gate words $727e/$7296/$729e/$72a6
  word[0] (the clr sites) - the floppy-path clear for the $9984 gate names the owed condition;
  (2) live: gate-word write-taps to catch any clear attempt + its pc; (3) decode mode $0246 vs
  $0248 (the $37xx builders used both); (4) strip FRAME-IRQ4 at cutover.

### Build #5 cont.31 - THE FINAL CHAIN DECODED END-TO-END (the DATASTEP gate's clear path)
- **$9884's parse loop = $98xx-$9982 (the disasm header was right all along):** walks the
  capture buffer (A3 from [$7a66]-family): `cmpi.b #$fe,(A3)+` (IDAM), **`cmpi.b #$ff,($4,A1)`
  - THE 0xFF ID-VALID MARKER at +4, the SAME format validated on the HD channel**, C-match vs
  [$7438], R-sequencing (D1/D2 compare-increment), $202a posted on fail via [$71be]; on match:
  $7a0e=1, node[$20] bits 7/14 branch, **[[$72d6]]=ffff + [[$72de]]=0 at $9968 - THE DATASTEP
  GATE CLEAR** - then $7a68=1, rte. $9a5c-$9a90 = DATASTEP's completion re-arming the gate
  (the ping-pong).
- **The full remaining chain:** [something invokes $9602] -> installs $9884 on level-6 ->
  IRQ6 (record marks, bit11-gated) -> $9884 parses the CAPTURE BUFFER -> ID-match -> gate
  clear -> wait-scan resumes DATASTEP ($9984) -> ... -> transfer -> $0BF6. STILL MISSING on
  the honest path (run56: DSETUP=0, [$7304]=$298c): (1) what invokes $9602 - callers: the
  $7492-walker (whose $74a6 region tests $7a0e - interplay with $9884's own $7a0e=1) + the
  $99xx family; (2) the CAPTURE BUFFER feed - the floppy channel needs the ID record STAGED
  (with the 0xFF marker!) where [$7a66] points, i.e. the D800-programmed capture the $92b4
  path-A/$8690 arms perform - THE TWO CHANNELS CONVERGE HERE: the idcap-style staged record
  (validated format) serves BOTH; the model's floppy path must stage the ID record at the
  D800 destination when the capture window arms (the $8690/$92b4-path-A D800/C800+E000
  pulse!), not only serve the E000 stream.
- **NEXT SESSION:** (1) name $9602's honest invoker (tap $7492 + the $74a6 gate); (2) stage
  the ID record (A1A1A1 FE FF c h r n crc - the validated capture format) at D800<<1 on the
  floppy's capture-window arm ($8690-class: D800+C800 programmed + E000 $2af/$2ff pulse) -
  the convergence of the two channels' capture machinery; (3) then $9884 parses, the gate
  clears, DATASTEP wakes, and the chain runs. The FB-record question resolved: the DATA phase
  parses the FE-record (the ID) - the FB record is consumed by the E000 stream pull, already
  validated.

### Build #5 cont.32 (runs 57-58) - the CONVERGENCE STAGING BUILT AND FIRING; the last piece =
### $9602's invoker (the WHEN)
- **stage_next_id() (KEEPER-candidate): stages the validated [A1 A1 A1][FE][FF] c h r n crc
  record at the fw-programmed D800 destination, picking the next ID mark rotationally from the
  stream's own layout.** Hooked at BOTH capture-arm sites: the E000 $2ff window-open (didn't
  fire this flow - the fw's $2ff pulse is $796e-gated) and the floppy bit15 arm (fires).
  Run58: IDSTAGE fired (id r=0f, idpos=2026, rp=1922 - rotationally correct) at the 7.97 arm.
- **But no consumer yet: [$7304] still $298c - $9602 (which installs $9884 on level-6) never
  runs.** The WHAT is built; the WHEN is the one piece left: $9602's honest invoker. Callers:
  $7492 (in the $7440-$74c0 walker whose $74a6 gate tests [$7a0e] - and $9884's own match sets
  $7a0e=1: the interplay to decode) + the $99xx family (post-install sites). NEXT SESSION:
  read the $7440-$7492 walker (entry conditions, who calls IT, the [$7a0e] gate direction);
  tap $7492; the invoker's gate is the last owed event before: $9884 installed -> parses the
  staged record -> [[$72de]] clears -> DATASTEP -> transfer -> $0BF6 -> the CPUAP's verdict.
- Note: only ONE IDSTAGE fired (the 7.97 late arm) - the 6.4-era arms should also stage
  (m_d800=$3ed6 then); verify the early-arm path takes the floppy branch with m_d800 set (or
  whether hd_chan/UIB-bit1 state at that instant diverted) when re-measuring.

### Build #5 cont.33 - NO CIRCLE: the $7a0e gate is first-pass-friendly; the walker's invoker
### is built by the read handler's own $a44a setup (the last decode)
- **$7a0e writers: ONLY $9948 (=1, the $9884 match), $9622 (=0, inside $9602 itself), $a47e
  (=0, the read handler's data-phase init).** And $74a6 re-read with the first-iteration
  question: `tst $7a0e; beq $74ce` = zero takes the CONTINUE path - not a block. The $7492
  `bsr $9602` install is upstream and unconditional when the walker runs. Dave's bootstrap
  question answered: no priming needed - the walker just has to RUN.
- **The walker's invoker: the read handler's $a44a setup builds it** - the $7444 control block
  {1, [$79d8].l, $d000, [$79ce], [$79d0]}, [$745c]=[$71be], **[$745a]=#$743c** - timer-list
  -entry shapes ($2b70-walker family: {countdown, value, target, next}). The $74xx walker is
  triggered via these blocks (a timer/completion edge the $a44a setup arms).
- **cont.33 addendum - the $7492 install is UIB-bit1 (CHANNEL) GATED; the FLOPPY BRANCH SKIPS
  IT.** $7482: `btst #1,($12,A0); bne $7568` (HD) `/ bra $7522` (FLOPPY). $748e/$7492 (the
  `bsr $9602` that installs $9884 on lvl-6) sits on the HD fall-through; the floppy path
  ($7522 = skip-factor geometry math -> $7578) NEVER reaches it. This is the SAME channel-select
  bit ($89f2-vs-SERDES, run37) one level up: the HD channel installs the $9884 ID parser here;
  the FLOPPY channel installs it elsewhere - the other $9602 callers are $999e/$99d8/$9a56/$9df0
  (the $99xx DATASTEP family = post-first-install re-arms) -> so the floppy's FIRST install is
  either one of those reached differently, or a site not yet found.
- **NEXT SESSION (the precise last decode):** (1) trace the FLOPPY path from $7522/$7578: where
  does it install $9884 (or its SERDES-parse equivalent) on level-6? Read $7578+; check the
  $99xx callers' reach-conditions. If the floppy genuinely never installs $9884, the parse is
  a DIFFERENT handler for the SERDES channel (the E000-stream ID scan, not the $7dac buffer
  parse) - which would mean the IDSTAGE-to-$7dac approach is the HD model and the floppy wants
  its ID matched IN THE STREAM ($9884 reading via [$7a66] pointed at... the stream? re-check
  what [$7a66] points to on the floppy path). (2) early-arm staging check. Chain otherwise
  intact: install -> parse -> [[$72de]] clear -> DATASTEP -> transfer -> $0BF6 -> verdict.
- ⚠ HOLD on trusting IDSTAGE-to-$7dac until (1) resolves - it may be the HD staging model
  mis-applied to a floppy channel that matches IDs differently (in-stream, not in-buffer). The
  format is validated; the DESTINATION/mechanism for the floppy is the open question.
- **cont.34b (the $7522-$75c0 read): the floppy branch = the SECTOR-SEQUENCE LIST BUILDER**
  (interleave-aware ascending-R writes via A1, [$7ac8] cursor) + $727e-gate management
  ($75a6: [$7b40]!=0 -> [$727e]=ffff; else [$7a0e]==0 -> [$727e]=0) - the $7964 waiter's gate,
  NOT DATASTEP's. No parser install here. $75c2+ = a separate round-robin unit scanner
  ([$79fe] over the $20a UIB table). **The floppy's parser-install hunt continues at the four
  unexplored $9602 callers: $999e/$99d8/$9a56/$9df0 (the $99xx family) - their
  reach-conditions are the next read.**
- **cont.35 (THE INSTALL-SITE ANSWER): $999e/$99d8 are INSIDE $9984 - DATASTEP IS THE
  INSTALLER.** $9602 = the RUN-ONE-CAPTURE subroutine (saves vectors, installs $9884 on lvl-6,
  clears $7a0e, captures, restores, returns D0 status) - called repeatedly from DATASTEP's own
  flow. $9984's opening: cancel the $7986 timeout -> bsr $9602 -> build the EXPECTED-ID
  TEMPLATE from UIB[$ce] (or UIB[$e4] if UIB[$20] bit14) copied 12 bytes into $7a5a-down ->
  bsr $9602 again (capture) -> **cmpm.b template-vs-captured at $99ea = THE ID MATCH**. The
  floppy matches IN-BUFFER (template vs capture) driven FROM DATASTEP; $9a56's variant gates
  on UIB[$20] bit14 + UIB[$11] bit1. So the chain is: wait-scan resumes DATASTEP (gate
  [[$72de]] - first-clear question REOPENS: who clears it before DATASTEP's first entry;
  candidates: the op-list walk dispatches $9984 DIRECTLY as a $192-table op, or a $39xx-
  builder variant seeds the gate 0) -> DATASTEP drives capture cycles itself.
- **THE ONE UNREAD PIECE: $9602's body $9636+ - the capture-destination setup (sets [$7a66],
  presumably to the $7dac-class buffer)** - read it, and the IDSTAGE HOLD resolves: if $9602
  points [$7a66] at a capture buffer the gate array fills, the staging approach is CORRECT
  (possibly with the destination read from [$7a66]/D800 at capture time); the format ($9884
  wants FF at +4) already validated. Then: DATASTEP's template-match succeeds on the staged
  record -> the transfer machinery it drives -> $0BF6.
  The in-buffer-vs-in-stream question is DOWNSTREAM of the missing install: the same absent
  setup that would install the parser would set the pointer. Also banked (Dave's lean
  corrected): build_serdes_stream emits NO 0xFF (A1A1A1 FE chrn crc - the run38-validated
  SCAN format); the FF-at-+4 is $9884's requirement only. TWO parsers, TWO formats - do not
  cross-apply. THE ONE DECODE LEFT: the floppy branch $7522 -> $7578+ - where the floppy's
  read flow installs its data-phase parse machinery (and sets [$7a66], and by which format).
- **cont.36 (the wake-up read + a FLAGGED CONTRADICTION).** Reference map of $9984/$729e/$72de
  is small: two builders ($37f2 and $39ac, both seed $729e={ffff,$0246,0,$9984} - gate SEALED
  in both), plus:
  - **$74a6-$74c4 = THE WAKE SEQUENCE (HD-side flow)**: after $7492's bsr $9602 succeeds
    (D0=0) AND [$7a0e] set (the match flag), the code sets [$72d6]'s gate=ffff (sleep),
    clears [$72d8]'s gate, and **clears [$72de]'s gate ($74c4) = DATASTEP WAKE**. The first
    capture happens in the SETUP path itself; its match opens DATASTEP's gate. First-clear
    answered for the HD shape.
  - $7a62-$7a6e (in the $7964 handler's flow) clears the registry SLOTS $72d6/$72dc/$72de/
    $72e0 themselves = deregistration (end-of-op), not gate wakes.
  - $8aa4/$91fe/$8446: HD lock-on-area and other flows also wake/deregister via the pointers.
  - **$7964 (per the ROM listing) = the floppy PER-SECTOR DRIVER** - and it is the LIVE
    handler in the parked WAITDUMP ([$72d6]=$727e{g=0000 h=$7964} - gate OPEN, dispatched
    every scan pass): tst $79b6/$79ba/$7956 outs; else dequeue next sector via bsr $32ac
    (empty -> clr $7968, park); else stamp $7424=sector, $741e=size/2 (the measured buffer-
    walker seed!), **clr $7a0e**, $741c=1, clr $7950 (phase toggle), SR=$2700, arm capture
    via the $63e-table E802 idiom, bsr $88ac. Continuation $7a4c+: cancel $7986 timer via
    $2aba, then the deregistration block.
  - **⚠ CONTRADICTION raised then DISSOLVED**: the listing decodes CODE at $7964-$7bxx, but
    that range is measured VARIABLE space ($7a0e/$7a16/$7a66 fall inside instruction
    operands; live RAM $7a66=0000 vs ROM bytes 4278). Resolution was already IN THE MODEL
    (mem_map ~line 3603): **fetch/data space split** - opcodes_map fetches ROM across
    $0-$ffff, mem_map overlays RAM at $4000-$7fff for DATA accesses. Handler $7964 executes
    from ROM while clr.w $7a0e writes RAM - same numeric addresses, disjoint spaces. The
    firmware deliberately doubles the range. CONSEQUENCES: cont.34's [$7a66]=0000 is a VALID
    variable read (no artifact); the $7964 per-sector-driver decode STANDS; move.w D1,$7a16
    is a RAM variable write, not self-modification. OVERLAYCHK (reads AS_PROGRAM) should
    show RAM variables at $7964, NOT 4a78 79b6 - confirmation only. ROM binary verified =
    listing at both offsets (xxd).
  - **METHOD NEAR-MISS (runs 60/61)**: reran with ONLY STORAGER_NOBYPASS=1 - read failed
    fast (@6.41 "no sys-floppy") and I nearly chased a phantom firmware divergence. The
    event-diff vs run59 named it: run59 had `BUSY-HOLD: suppressed premature DONE` (env
    STORAGER_BUSYHOLD) + STEPIRQ machinery; without them the DONE posts at +0.3ms. THE FULL
    SIX-KNOB STACK (line ~461) IS THE REPRO - a partial env is a DIFFERENT MACHINE. (This is
    also more cutover-audit evidence: behavior depends on env presence.)
  - **Free measurement from the fast-fail**: the ERROR path works end-to-end LLE - fw posts
    error DONE ($28b0), CPUAP reads it, RECONFIGURES the unit (UIB[$12] 40->44 i.e. sets
    bit2, INIT unit2 heads=2 spt=16 secsize=256 via cmd 0x87), resubmits 0x95 with count=4
    (node[$20]=0820 vs 0026 first try), second fail -> honest "no sys-floppy, going to
    harddisk" verdict. The CPUAP-side negative-verdict flow is now measured.
- **cont.37 (THE STARVATION ROOT + THE CAPTURE CONTRACT WIRED).** The park chain, fully
  measured (runs 62/63):
  1. Wait-scan dispatch condition decoded ($1606-$1676): slot populated AND gate==0 AND
     **the anchored node's status byte (+$26) == $0a**. Status $0a is stamped ONLY by
     **op-code $36 in the node's op list** ($1596-$15a0) - the WAIT op. Live park: node
     $71c6 status stays $00, op/continuation cells at $7256 are ZEROS (the build#5-header
     "null continuation cells") -> scan never dispatches $7964 despite its open gate.
  2. The ring ($7224, appended by $32da from IRQ handlers, drained by $32ac from $7964) is
     empty because NO CAPTURE EVER COMPLETES: E000MODE (run63) caught the model killing the
     rotation inside the fw's own capture-arm - $88ac primes (E802 $8200 pulse around an
     E000 $22f window w/ 16 flush reads), closes **$23f at pc $892e -> the model stopped
     the PUMP**, then restores the $0a6d operating mode (= [$7a48], which the model's open
     predicate didn't recognize) and finally arms E802=aad3 (bit15 HELD + bit11 IRQ6-enable)
     - an armed capture the dead rotation could never complete. 1.3s of live engine
     ping-pong ($7ba8/$7fee) ran under $0a6d before this arm.
  3. OVERLAYCHK (run62): RAM $7964 = variables (0000 0080...), NOT the ROM's code bytes -
     the fetch/data SPACE SPLIT (opcodes_map ROM / mem_map RAM at $4000-$7fff) is confirmed
     live; RAM $7a16 even held the $0010 the engine's own move.w stamps. cont.34 valid.
  - **WIRED (keepers)**: (a) the rotational tick = the SPINDLE - pump gates on stream-
    nonempty, never on the E000 window; $23f closes only the read mux. (b) held E802 bit15 =
    capture PENDING (m_idcap_pending; falling edge cancels - the prime pulse is µs); at the
    next id_end mark pump_tick stages the record at the fw-programmed D800 dst
    (stage_next_id(dst, mk.pos)), posts E01E bit4 (m_ch_op_ok), delivers IRQ6 per [$79f8]
    bit11. Replaces cont.31's instant-stage-at-arm.
  - E000 mode values (measured + $88ac/$7fd6 static): $22f = capture/flush window, $23f =
    idle/close, $2af+$2ff = data-phase arm pair, $0a6d = [$7a48] operating mode.
- **cont.38 (THE RECORD LAYOUT, from the fw's own pointers).** Run64 post-wiring: the engine
  went LIVE - 1743 captures/22s at the fw's 12.5ms retry cadence, r walking 01..10 (all 16
  FM sectors), no fast-fail - but every record REJECTED without $202a (the C-compare RAN and
  PASSED; the reject was downstream). $7ce6 decoded = the real floppy match mechanism: read
  captured R via [UIB+$ce], index the $7654 SECTOR MAP ($ff=not-wanted -> $7e0a re-arm,
  $fe -> $7e1e, else accept -> data-phase setup $7d4a). CYCLESTATE (run65) gave the verdict:
  **[ca]=$7db0 [cc]=$7db1(C) [ce]=$7db2(R) - CONSECUTIVE. The gate array's record drops H**
  (hardware checks it against side-select): layout = A1 A1 A1 FE | FF@+4 | C@+5 | R@+6 |
  N@+7(provisional). My stage put h at +6 -> fw read R=00 forever -> $7654[0] never wanted.
  ONE layout reconciles every reader: $9884's FF@+4, $7c46's C@[cc], $7ce6's R@[ce]. The UIB
  pointers ARE the format spec (CPUAP configured them knowing the hardware). Stage fixed
  (H dropped); run66 pending.
- **cont.38c/d (runs 74-76): the interrupt contract SPLIT + typed captures + THE $89f2
  DECODE THAT CLOSES THE MAP.**
  - Mode-split wired: STREAM mode (window open) = per-record strobes (IRQ6/5/3, the
    $29xx-toggler network); CAPTURE mode (window closed) = completion-only, ALWAYS via IRQ6
    (bit11) - $298c's phase fork: phase0 take -> $89f2, phase1 -> $92b4. Run75 (untyped)
    killed the 1738-churn (11 captures) but fed the hunt data records it never asked for;
    run76 added TYPING: **$22f-primed arm = ID capture; bare $8a00 re-arm ($92b4) = DATA
    capture** - clean id walk returned but at HALF rate (odd sectors only) and no accept.
  - **$92b4 decoded**: phase-1 = capture-complete consumer/data-armer: [$742c]==0 ->
    D800/C800 load + E000 ack + $7b18=1 + re-arm E802 $8a00; [$742c]!=0 -> $92f6 fail path
    ($9318: map[[$7428]]<-$f0 + $79b8=1).
  - **$89f2 DECODED - THE GATE-CLEAR SITE FOUND ($8aa4)**: HD branch (UIB[$12] bit1) = the
    counted lock-on ([A1A1A1][FE][FF] check at $7dac, $7a0c countdown) -> at zero: E01E ack,
    mask enables, and **if [$7a0c]==0: movea [$72de],A0; clr (A0) = WAKE DATASTEP** (else
    error $2029 to the node). FLOPPY branch (bit1 clear) = NO buffer check: clr $7950,
    **decrement $79a4** (seeded $0100 from $79a2), re-arm via $8ab8; on EXPIRY falls into
    the SAME $8a42 -> $8aa4 DATASTEP wake. The capture-mode loop is a SETTLING COUNTDOWN
    that hands off to DATASTEP ($9984 = cont.35's $9602 capture + template-match engine).
    THE MARATHON'S CHAIN CLOSES HERE: $79a4 expiry -> [[$72de]] clear -> wait-scan (node
    status $0a via op $36) -> DATASTEP -> template match -> transfer -> $0BF6.
  - **WHY IT NEVER FIRES: $79a4 is REFRESHED from $79a2 on every tracked event** ($7e60,
    $7e0e - the $7exx map-walk paths) - the countdown never runs dry (run76: 1638 id
    captures over 22s, no expiry). NEXT INSTRUMENT: log $79a4 writers in the retry era -
    name who refreshes it and under what condition the refresh STOPS on real flow (likely:
    the refresh happens only while IDs keep matching expectations; the model's staged walk
    satisfies it forever - or the countdown is meant to expire in ONE rev because the
    hunt should ACCEPT within a rev and stop counting).
  - STANDING HYPOTHESIS (flagged, unproven): the 6.41-6.7 STREAM era = the real read op
    (ping-pong + map + sector buffering; XFERGATE slots 0-2 walked then stalled; $9318
    f0-stamps from 6.41339 = per-sector failures from the start); the capture-mode era =
    the fw's RECOVERY loop. DISSONANCE TO RESOLVE: the pre-cont.37 "eight-sector buffering
    measured working" era vs the current 6.4x stall - what regressed or what was
    misattributed. Do not wire against this hypothesis until the dissonance is measured.
- **cont.38e/f (runs 77-80): THE STREAM-ERA FAILURE CHAIN, fully named.**
  1. $8ab8 = bare re-arm (ack + bsr $88ac + rte): the capture-mode network for the FLOPPY
     is TIMEOUT SCAFFOLDING ONLY - count [$79a4] (seed $0100 from $79a2, decremented at
     $8a3c/$7d9c, ~2/25ms, NO refresher - run77) down to expiry -> $8a42 with [$7a0c]=3
     (seeded at $6ab2, floppy never touches it) -> the $2029 error fork, never the $8aa4
     DATASTEP wake (that fork is the HD's: 3 lock-ons clear $7a0c -> wake). The capture
     mode = the fw's retry/timeout loop, NOT the floppy's read path.
  2. THE REAL FAILURE (stream era, from the first sector): $92b4 phase-1 entered with
     **[$742c]=1** -> $92f6 fail path -> $9318 f0-stamp -> teardown/rebuild, every 12.5ms
     from 6.41338. Setter measured (run78): **$7eb2 (logged pc $7eb8), the $7e8a fork:
     [$7968]==0 -> retry-flag** - and $7968 is never set nonzero (run79 SEQACT: zero hits).
  3. $7968=0 because the $32ac dequeue finds the $7224 ring dry - SAME starvation as the
     park, seen from the stream side.
  4. THE OP LIST (runs 79/80): the dispatch built the SHORT list **{$24, $26} at $7250 via
     the $5f74 builder** (pcs $5f86/8a/8e, once @6.40100); the FULL read list ($5fe0:
     $24 $28 $56 $58...,) never built; each retry cycle re-enters via the [$79e2]!=0 path
     ($6024 region) and writes ONLY a terminator at $7240 (pc $6044, every 25ms).
     [$79e2]'s single writer = $f8a (init/config).
  5. $192 OP TABLE resolved: $24->$651c, **$26->$6ce4 (HOT in run65's PCHIST - the retry
     loop IS op $26 executing and failing)**, $28->$6788, $36->$15fe (the wait-scan = the
     wait op, confirming the $36/status-$0a mechanism), $56->$a392, $58->$7346.
  NEXT FRONTIER (the one decode): **op $26 = $6ce4, the floppy read-step op** - what it
  needs to succeed (it works the $7654 map region, $6b58-family), and why the dispatch
  chose the {$24,$26} short list vs $5fc0's full list (entry-point/gate question: who
  jumps to $5f74 - likely the cmd-0x95 $92-table entry itself; and [$79e2]'s role).
- **cont.38g-l (runs 81-82): THE TIMELINE REWRITTEN - the read RUNS; the gap is the final
  data-phase arm.**
  - OPWALK (the $15b0 dispatcher trace): the restore ran {$24,$26} on node $71f0 with
    **[$79e2] ALREADY 0** (recal proven at boot - the fourth dead premise); the read built
    its FULL list on node $71c6 ($24 $28 $56 $58 $1a $18 $54 $4a $42 $36) - the $6024 path
    is a list VARIANT, not a defer (ops $28 $54 $4a $42 $36; my $7240 tap saw only its
    terminator).
  - op $28 ($6788) = SEEK: returns $fe (re-run) until [$7a36] (seek-complete); spins 85us/
    pass at 6.40x until stepdone. op $42 ($6bc2) = **MOTOR SPIN-UP WAIT**: arms a $29f8
    timer, countdown=UIB[$18], completion sets [$7a3e] (via [$7a40] node bookkeeping).
  - **SPINFLAG (run82): armed ONCE @7.96397, FIRED @9.79017** (~70 ticks x 26ms PIT1-ctr0
    = 1.8s spin-up). The eternal WAITDUMP@9.7903 timestamp = the spin-up completion moment.
    THE 6.4-9.79 ERA IS THE SPIN-UP CHOREOGRAPHY (the $89f2 $79a4-countdown = rotation-
    speed verification, NOT a sector hunt - reframe of the reframe); the capture/typed
    wiring (cont.37-38d) serves it faithfully.
  - **POST-SPIN-UP THE READ RUNS (9.79048 READ-START)**: $88ac arms, id captures, the
    $7ba8/$7fee/$8018 ping-pong live, **phase-1 ($92b4) ACCEPTS each target sector and
    calls the $352e queue-append** (9.83838 r=01 ... 10.41338 r=08 - all EIGHT sectors
    id-accepted at ~1/rev), slots walking with $78c4 buffer pointers + $0080 stamps.
  - **THE ONE REMAINING GAP: the data phase never delivers** - no $fe done-stamps ($8174),
    no host DMA (HOSTDATA/DMAFIRE 0), D800 stays $7dac ([$7434]=3ed6) - the sector DATA
    never lands in $78c4, so after r=08 a SECOND pass begins (7428 wraps to 2 @10.62).
    NEXT DECODE (bounded): after the id-accept, who/what arms the DATA capture in this
    mode - the $801c/$8022 D800/C800 loads (pc $8022 D800wr seen @9.82501), which record
    the data capture consumes, and the path to $8174's $fe-stamp + the $74ac/$74ae host-
    transfer queue -> DMA -> DONE -> verdict.
- **cont.38m (runs 83-86): the ARM-SIGNAL question resolved to its final shape - NEXT
  SESSION'S FIRST WIRE.** The post-accept DATA arm was invisible to the bit15-edge hook
  ($92b4's $8a00 makes no edge when bit15 is held). Tried: arm-on-C800-load (runs 83-85) -
  collides with the $30e4/$31d0 FREE-LIST writes (C800 = ONE RAM: terminal-count register
  AND the fw's buffer free list; offset 0 doesn't split them). The lifecycle trace (run86,
  CAPCANCEL) killed the bit15-fall cancel too: **the fw drops bit15 INCIDENTALLY in its
  routine enable juggles ($67ff/$77ff masks - $7cca after the r=01 accept, $7f84 in the
  data flow) - bit15 is a shadow enable bit, NOT the arm**. THE CORRECT CONTRACT (the
  model's own logs have always shown it): **C800/D800 = parameters; the ARM COMMIT = the
  E800 kick** (DISKOP-kick lines, op-family bits 5-7 from the $63e table) - the classic
  load-params-then-kick controller idiom. Wire: pending=1 (typed by the $22f tracker) on
  the E800 kick writes; NO bit15 cancels (disarm = completion, or $23f idle/E800 reset if
  measured to need it). Current state left in tree: C800-offset-0 arm + bit15-fall cancel
  WITH lifecycle logging (CAPARM/CAPCANCEL) - replace with the E800-kick arm first thing.
  Read-era measured (runs 82-86): READ-START @9.79 post-spin-up, all 8 sectors ID-accepted
  (~1/rev), ring fed via $352e; the DATA captures still never complete (DATASTAGE=0 in the
  read era) - the data record never lands, the op re-passes after r=08.
- **cont.38n (runs 87-90): THE DATA RECORD LANDS.** The E800-kick arm hypothesis DIED
  measured (runs 87-89: ZERO E800 activity in the per-record capture cycles - only the
  26ms timer-ISR touches at $2bc6/$2bd2; E800 kicks are op-level channel starts, and the
  22bd/221d pc-$31cc/$31d8 churn = the fw programming the C800 free-list through the E800
  bit7 aperture strobe). THE SURVIVING CONTRACT, simpler than every hypothesis: **the arm
  = the E802 bit15 RISING WRITE** (both flavors produce one at write granularity - even
  $92b4's andi #$77ff / ori #$8a00 pair), **typed by the $22f tracker, and NOTHING disarms
  a pending capture but its completion**. The cont.37 bit15-fall cancel was the DATA-
  capture murderer (the routine $67ff/$77ff juggles drop bit15 incidentally - run86's
  CAPCANCEL type=DATA pc=$7f84 was the kill on camera); the run83 C800-load arm collided
  with the free list. Both RETIRED (tombstone comments at the sites).
  **RUN90: DATASTAGE fires - sector 1's REAL 128 data bytes (stream 14..142) staged at
  $7dac at its data mark, once per revolution, 7 times.** Both record types now delivered
  on the fw's own arms. REMAINING (next bounded question): the CONSUMER'S verdict - the
  fw re-arms for the SAME sector each rev (no $8174 $fe-stamp, no $74ac queue-advance, no
  host DMA; stages stop ~11.0s). Decode/measure what the post-data-capture round ($92b4's
  [$742c]==0 branch -> $7b18=1 path, or the $7ba8 flow) checks on the staged record and
  why it doesn't advance - then the mapped tail: $fe-stamp -> $74ac/$74ae -> DMA ->
  CMDDONE -> the CPUAP verdict.
  OPENING FRAME for the verdict question (decoded post-run90): the sector-completion
  writer at $8140-$81b2 FORKS at $8158 on [$79b8]: NONZERO -> the $8170 path ($fe map
  stamp + $74ac/$74ae queue link) - **$79b8=1 belongs to the ERROR/FLUSH eras ($9304
  setter), so $fe likely means CLOSED-OUT/FLUSHED, not done-queued (REVISE the cont.38
  map-code table)**; ZERO (the read era) -> $815e: slot+2=0002 + slot+4=0.l = the happy
  completion flavor. Run90: NEITHER ran - no slot+2<-0002, no $fe - the $8140 writer is
  never reached post-data-capture; the staged record is not consumed at all. The chain
  into $8140 (from $810e's $7430-list walk / the $8106-era) is the next decode, alongside
  what the $7f52 `tst $7b18` consumer does after $92b4 sets $7b18=1.
- **cont.38o (runs 91-92): THE VERDICT CHAIN LOCALIZED.** DREADY: [$7b18] set-then-cleared
  every round, but the setter is **$92fc = the $92f6 FAIL path** - phase-1 never examines
  the record because **[$742c]=1 at every round**. FLAG742C (read-era window): the happy
  clear ran ONCE ($7cae, the good-ID continuation at the r=01 accept @9.81255), then
  **$7eb2 re-poisons it every 25ms, 80us before each phase-1 round, on [$7968]==0** - the
  ring-consumer flag only $7964's dequeue-success sets ($79d6). The ring HAS entries now
  ($352e appends run) - the consumer never runs. FULL CIRCLE to the wait-scan dispatch:
  with spin-up complete, does the op walk reach $36 -> status $0a -> dispatch $7964?
  ONE-TAP QUESTIONS for next session: (1) node $71c6 +$26 status WORD post-9.79 (NOTE: the
  WAITDUMP read byte(+$26) = the HIGH byte of a word status - $000a would read as 00; the
  scan's cmpi.b #$a,($26) vs the $15a0 move.w setter is a byte/word encoding PUZZLE -
  my st=00 readouts may have been misread all along); (2) opcode tap at $7964 entry
  t>9.79 (does the driver EVER dispatch); (3) if yes, does its dequeue set $7968=1 and
  break the re-poison loop -> $742c stays clear -> phase-1 happy -> $815e slot-status-2.
- **cont.38p/q (runs 93-94): THE STAMP FIRES; THE SCAN-DISPATCH MODEL DIES (premise #8).**
  NSTAT (byte-resolved): the $36 op stamps node $71c6 status **word $000a at $71ec**
  (pc $15a4, @9.79188 - 140us after spin-up) - the walk DOES reach its wait op. The
  encoding resolves per Dave's constraint: $0a lands in the LOW byte ($71ed); the scan's
  cmpi.b #$a,($26,A1) reads the HIGH byte ($71ec)=00 - and the fw's own WORD reader
  ($7432 cmpi.w #$a) matches the stamp, so the field is a word and the scan's byte test
  can never match any normal status. MEASURED CONFIRMATION: **SCANDISP ($1646 opcode tap)
  = ZERO across the entire run - the wait-scan's dispatch path NEVER executes, in any
  era. DRV7964 = ZERO - $7964 has never run in any measured boot.** The "wait-scan
  dispatches $7964 on status $0a" model (cont.36-era) is DEAD; the $72d6-$72ea registry
  entries are dead weight in every run taken; the live floppy engine is purely the
  IRQ-driven ping-pong ($89f2/$92b4/$7ba8/$7fee/$8018 + the togglers).
  REFRAMED NEXT QUESTIONS: (1) who consumes the $000a status/the registry handlers if
  not $15fe - the second cmpi site ($16ba region) or another descriptor walker; (2) the
  $7968/$742c chain: its assumed setter ($79d6 inside $7964) is unreachable - either
  another $7968 setter exists, or the HAPPY flow routes around the $7e8a fork entirely
  (the $7eb2 re-poison may itself be an artifact of a state the real flow never enters);
  (3) what ACTUALLY gates $92b4's [$742c]==0 happy branch on working flow. The scan
  ($15fe) may serve a different command class whose statuses are $0aXX words.
- **cont.38r (run 95 + exhaustive static): THE REAL WAIT/RESUME CONTRACT; the registry is
  VESTIGIAL in v2.60.** STATRD: the $000a consumers = the op-walk dispatcher ($1586) and
  THE MAIN LOOP ($229a/$23ba - the PCHIST-hottest code). The main loop dispatches node
  status through **the $222 STATUS TABLE**: 8 -> $156a (list-anchor swap), $a -> $15fe,
  $c -> $23e6 (op-walk continue), 4 -> $1f82, else table-jsr. Status-$a's handler IS the
  wait-scan - which NO-OPS: its cmpi.b #$a,($26) reads the status word's HIGH byte, and
  NO writer (word-immediate, dynamic, or byte - exhaustive grep) ever puts $0a there.
  **The $15fe registry dispatch never fires on this firmware; the $72d6-$72ea registry +
  its handlers ($7964/$3dbc/$94ec/$9984/$9398) are VESTIGIAL in v2.60** (maybe live in
  v1.80/SGIC). THE REAL CONTRACT: park = status $000a (main loop spins past); **RESUME =
  an ISR re-stamps the node runnable ($0c via $17f8-family / $08 via $2244) and the main
  loop's next poll continues the op walk PAST the $36 op to the transfer ops**. Run93
  NSTAT: after the $000a stamp, NO status write for 20s - the post-data-capture re-stamp
  never happens. SEQACT2: $7968 NEVER written nonzero (its only setter is in the dead
  registry code) - so on real flow the $7e8a/[$7968]==0 fork should ALSO never poison
  $742c the way we measure... yet $7eb2 runs every 25ms in our boot. THE ONE REMAINING
  QUESTION, now approached from both ends: what re-stamps the node runnable after a data
  capture on the happy path, and what keeps [$742c] clear there (i.e., what state on real
  flow routes AROUND the $7e8a fork / satisfies it - $7968 being dead suggests the fork's
  guard chain ($79b6/$79a0/$7d16-era) differs on working flow). NEXT TAP: who calls the
  $7exx flow containing $7eb2 (backtrace its entry per 25ms cycle) + the $17f8 re-stamp
  site's context (what event/conditions drive it).
- **cont.38s (run 96): THE DIFFERENTIAL GUARD READ - the poison's causal order + the two
  wires it names.** Side-by-side (am=FE id rounds vs am=FB data rounds): first post-spin-up
  round CLEAN (742c=0); **the poison fires at the SEQUENCE-ADVANCE - the r=02 id round
  after sector 1's accept takes $7e50 (r==[$7428] sequential), advances 7428->2, hits
  $7e8a with [$7968]=0 -> $7eb2**; every later round carries 742c=1; the am=FB rounds
  entering the ID guard chain are downstream noise of the already-poisoned phase-1. KEY
  ABSENCE: **no DATASTAGE at the r=01 data mark (9.8125)** - the post-accept re-arm went
  through $88ac (ID-typed), the data record passed UNCAPTURED. RESTAMP-17f8: never runs.
  THE MODEL'S TWO WIRES (the accept installs [[$7300]]=$7ba8 + leaves bit9 enabled - the
  fw expects the DATA record as an IRQ5 STROBE into the ping-pong/E000-pull, not a bit15
  capture): (1) restore per-mark IRQ5 strobes per bit9 in ALL modes (cont.38c's mode-split
  killed them in capture mode; the fw's enable masking is the gate; the run73-era garbage
  came from spurious COMPLETIONS which typed captures already fixed); (2) the E000 mux
  open-set: every $88ac arm's $23f closes it and the restored $0a6d operating mode never
  reopens it in the model -> read-era data pulls would read channel status, not stream.
  $23f closes; operating-mode writes ($a6d/$2ff families) open. INTENDED FLOW once wired:
  accept -> IRQ5/$7ba8 data strobe -> E000 pull serves sector bytes -> consume (742c
  clear, no $7e8a poison) -> $17f8-family re-stamp -> walk continues past $36 to the
  transfer ops -> DMA -> DONE -> verdict.
- **cont.38t/u/v (runs 97-100): THE FULL ARM/DELIVERY BRACKET - four regimes measured,
  none consume; the unexplored selector = the E802 OP-FAMILY BITS.**
  - run97 (strobes-all-modes + mux-open-on-any-mode): the mux half re-created the refuted
    ungated-streaming failure (fw status reads got stream bytes; 126 per-rev READ-STARTs).
    REVERTED - the old open-set ($2ff/$xfff) was always sufficient (the data pull's $7fc6
    issues $2af+$2ff itself).
  - run98 (strobes-all-modes alone): pre-poisons the hunt (742c=1 before any advance) -
    per-mark IRQ5s in capture mode refuted AGAIN, now in isolation.
  - run99 ($22f-typed captures + typed-LEVEL delivery id->IRQ6/data->IRQ5): clean start,
    same advance-poison as run96 - the post-accept $88ac arm is $22f-primed so the r=01
    data mark still passes uncaptured.
  - run100 (TYPE-AGNOSTIC capture + typed-level delivery): data captures fire (11), the
    $7e8a advance-fork never reached (3rd regime), but per-rev READ-STARTs persist (127)
    and no consume/re-stamp.
  ALL FOUR combinations of {typed-by-$22f, agnostic} x {IRQ6-only, typed-level} measured;
  none reach the $17f8-family re-stamp. NEXT SESSION'S OPENER: **decode the $63e op-family
  table** - every arm builds its E802 value from $63e entries (bits 5-7 = the op family,
  |$80 etc.: $88ac/$7fc6/$7fee/$92b4 all index it via $7a16/$7a18/$7432) - the op-family
  bits are almost certainly the hardware's REAL capture-type/mode selector; the $22f proxy
  is a stand-in that fails exactly at the post-accept arm. Static read of $63e + one run
  logging the E802 op-family bits per arm alongside the record type the fw then expects.
  Tree state: cont.38v (agnostic + typed-level) left in; regime same-family as run90
  (data lands, no consume). Default HLE boot re-verified GREEN after cont.38t/u/v (4FFFFC x2, no knobs).
- **cont.39p (run 144): THE FULL-TRACK PICTURE - everything read, nothing delivered.**
  The 12s accounting: **map@win = c0 x8 (sectors 1-8, consumed/claimed) | 07 08 09 0a 0b
  0c 0d 0e (sectors 9-16 READ AHEAD, slots 7-14 filled) | fe fe fe fe (END-MARKERS -
  the ledger TERMINATED PROPERLY)**; [$7428/[$7430] = $11 = the first end-marker (the
  loop correctly reached the end - the "stuck at 9" read at 10.5 was mid-progress);
  the $74ac/$74ae queue grown to slot 16; $7968=1; **[$79a8] = 0008 - EIGHT HOST
  TRANSFERS OWED, the counter never decremented**; [$79a4] = $fd70 (wrapped past zero -
  the expiry crossing happened ~10.9s; whichever drain-variant ran did not trigger the
  host phase; $741c still 1); **nstat = $000a - THE NODE STILL PARKED. The resume for
  the TRANSFER PHASE never comes.** THE LAST LINK, precise: (a) the transfer-phase
  resume (who re-stamps the parked node when the batch completes - the $8a5a/$7db4
  content-fork drain variants, the $79b6-gated $9328 counter block ($79b6 setters
  unread)); (b) THE TRUCK ITSELF: the model's host-DMA transfer engine under NOBYPASS -
  the HLE delivery shim is explicitly DISABLED under NOBYPASS ("the read is FW-DRIVEN,
  no shim") - when the fw finally kicks the host DMA (D800:C800 = host addr + E800
  kick), the model must serve the bus copy as pure hardware. Verify the kick-transfer
  path exists un-gated; it may be the model's one remaining un-built hardware duty.
- **cont.39m/n/o (runs 139-143): THE READ RUNS - sector-by-sector, full speed, the
  shelf stocked, the dead flag alive.** THE ROOT (run139 micro-trace): the model's
  ROT-START opened the stream window at the doorbell (model-initiated) - the first
  data-AM strobe arrived BEFORE the fw's first arm (idcap=0, buffer empty), the walk
  matched R=0 to virgin [$7428]=0, ran the scan on the fill-less map, poisoned $7430,
  and THE DOOR'S OWN CONTINUATION copied it into $7428. THREE KEEPERS WIRED:
  (39m) ROT-START builds the stream + starts the CLOCK only - the window opens on the
  fw's own writes; (39n) the fw's $a6d-class OPERATING write (uniquely bit11 among the
  mode words) = its channel engagement - opens the window; (39o) the bit15-arm branch
  rebuilds the stream ONLY on track change (the unconditional rebuild was resetting
  m_seen_id 90us after every id, gating every data strobe).
  RUN143, ON TAPE: $7428<-1 ($70e4, the legit first target) -> map[R]=$ff matched
  ($7e0a) -> scan next=2 (a REAL result) -> **THE DOOR advancing the target 1->2->3->
  ...->8, ONE SECTOR PER 12.5ms, consecutive, no wasted revolutions** -> at sector 8:
  $7430<-9 (end territory) -> **$8290-CONT RUNS (first time in 143 runs) -> $7968<-1 AT
  $82b2 (the un-buried setter fires!) -> HAPPYSTAMPs ($815e slot-status-2) -> THE FILL
  LEDGER STOCKED (SECMAP [765c]<-$07, [765e]<-$08/$09 - POSITIVE SLOT INDICES, pc $812c
  = the shelf-stocker, on camera)** -> the $74ac/$74ae transfer queue populated, the
  node processor walking it. THE REMAINING TAIL: the host DELIVERY - the queued slots
  never DMA to the host buffer (HOSTDATA=0, no CMDDONE post; the machine reads ahead
  past the wanted set and the $79a4 countdown resumes). NEXT: the queue-drain trigger -
  what should fire the host DMA from the populated $74ac queue (the E800-kick/CHANSTART
  path under NOBYPASS; the $23e6/status-flow that converts queued slots into bus
  writes to the IOPB buffer 0fc0dd) - the LAST phase between here and CMDDONE/verdict.
- **cont.39k (the $94d8/$9a3e reads, DAY CLOSE): CMD $95 IS THE NO-LEDGER CLASS - the
  week's biggest correct-else-branch.** BOTH builder routes are bit14-gated ($94ca and
  $9a42: btst #$e of node[$20]=the param; $8c27 bit14=0 skips, $c827 bit14=1 calls):
  **the $73fa/$7522 sequence builder, the $ff/$fe ledger, and op $48 belong to the
  $96/$82/$72 class BY DESIGN. Cmd $95 never builds a ledger because it never uses one.**
  The "builder never runs" chase closes as correct behavior. ($94c2 bonus: that wake
  routine clears the [$72d6] gate - the wake writes live there.) THE $95 PICTURE: its
  data phase = the $8018 door chain (IRQ5 taken bit-was-1 -> $8018 -> D800/C800 -> E000
  -> the $802c/$804c tail -> slot stamps -> $808a walk -> $8140 writer) - PROVEN WORKING,
  opened once per boot on camera (run121). The $7ba8-side map-walk rounds (the $7ce6/
  $7e50/$7e6c scans, the $44 overrun, the $7eb2 poison) are WRONG-PHASE symptoms - the
  fw driven into the other class's rounds by events arriving with phase 0.
  **THE FRONTIER, FINAL FORM (cont.38ee's question with a fully clean perimeter): the
  $7950 phase toggle must survive from an id event to the data event for cmd $95's
  consume. The clearers: $8a3e ($89f2's count-branch exit) and $88d6 ($88ac's re-arm).
  The escape: the setup era's first pair - the walk exiting via the $7exx/$7f22 tail
  WITHOUT re-arming. What distinguishes a no-re-arm id round on real $95 flow - THE one
  remaining question, with nothing anywhere beside it.**
- **cont.39i/j (runs 134-137): the truncated-UIB keeper + the param-word decode; the
  config layer FULLY closes.**
  (1) REAL DEFECT FOUND AND FIXED (keeper): the model's 0x87 UIB copy was hardcoded to
  0x1c bytes, dropping [$1c..$21] of the host image. Fixed: IOPB-count driven, capped
  0x22 (run135: the raw count $100 over-copies and clobbers the fw fields past the
  image - pointers destroyed, door lost; the DMA-length semantics deserve a later
  decode). Run136: [1c..1f] now delivered (ff ff ff ff), pointers intact, door restored.
  (2) THE $68-vs-$8c DIVERGENCE FULLY EXPLAINED (candidate (c) dies completely):
  **UIB[$20..21] = THE CURRENT COMMAND'S PARAM WORD, stamped by the dispatcher ($e82)
  at every dispatch** - $0026 at the restore, $8c27 at the read = the $92-table params
  verbatim. The host image's $68 was a dormant default. Every [$20]-gate reads the
  command's own class descriptor; every measured value was authentic. PARAM SEMANTICS:
  $95->$8c27 vs $96->$c827 (the $5fc0-flavor vs $6102-flavor high bytes), $94->$8c26,
  $82->$c826, $72->$ca22, $89/$98->$0026 - the param = the command-class descriptor
  feeding node[$28], the template select (bit14), the unit fork (bit4), the $1718 gate.
  CONFIG LAYER: COMPLETELY CLEARED (one keeper extracted). THE FRONTIER RETURNS, with
  nothing left beside it, to the dynamic question: how real flow reaches the builder
  dispatch ($94d8/$9a5e via the scan) - the wall, now provably alone.
- **cont.39g/h (runs 132-133): the side-quests - one cleared, one confounded.**
  (1) GUARD-READER AUDIT: both decodes SURVIVE - the $21c0 guard reads ($d0/$d2,A4)
  as decoded; the seek updates [$d2] ($6ac6) and [$d4/$d6] ($680a/$6938-family); the
  $365a/$365e pair (op-walk router region) writes BOTH d0/d2 = THE INVALIDATOR (parked
  $ffff = post-teardown invalidation; the read-start match happened while [$d2] still
  held the restore's 0). Two-cache semantics: current-position (invalidatable d0/d2) vs
  last-sought (d4/d6). No correction needed either side.
  (2) COPY-FIDELITY: storager vs host divergence at **[$1c..1d] (00 00 vs ff ff) and
  [$20] ($8c vs $68)** - and [$20] is LOADED: the $1718 re-stamp gate (&$30) FLIPS with
  the host value ($8c->0 skips, $68->$20 passes!). BUT the check is CONFOUNDED: 0fe948
  is the 0x87's host buffer and the CPUAP INITs multiple units - by dump time it may
  hold ANOTHER unit's image ($68 could be unit-0's byte). SURGICAL NEXT: snapshot host
  buffer + dest at the DMA moment for unit 2 (hook at the model's UIB87 log site); if
  unit-2's image really carries $68 and the copy delivers $8c, the copy path is the
  defect and the $1718 gate (and possibly the whole class routing) flips with the fix.
  If the DMA-moment images match at $8c, the divergence is buffer reuse and the config
  layer stays fully cleared.
- **cont.39f (run 131): THE UIB, COMPLETE.** @6e60: [00..20] = 02 10 80 00 07 07 01 08
  02 01 00 01 00 00 00 00 | 10 97 40 18 06 02 00 02 | 46 03 50 00 00 00 ff ff | 8c;
  [ca..d7] = 7db0 7db1 7db2 | ff ff ff ff | 00 00 00 00. THE AUDIT:
  (1) **UIB[$0e] = $00 - the $743a builder-gate byte (tst.b; beq skip) is ZERO** - and
  it is AUTHENTIC host data (the 0x87 INIT DMAs the CPUAP's own UIB image; the model
  fabricates nothing) - so the $73fa-block via $7432/$743a legitimately skips for this
  config: that block is NOT cmd-95's builder path either. Hypothesis (c) narrows: a
  mis-model would have to live in the DMA/copy itself - VERIFY by comparing the
  host-side image bytes against $6e60 (one dump).
  (2) **UIB[$18] = $46 = 70 ticks x 26ms = 1.82s - the measured 1.8s spin-up EXACTLY**
  - a perfect independent cross-check of the cont.38g spin-up decode and the PIT quantum.
  (3) NEW STRUCTURE: TWO position caches - [d0..d3] = $ffff (the $21c0-guard's compare
  fields, never updated = "position unknown", forces seek) vs [d4..d7] = 0 (updated by
  op $28's seek, $680a-family). Either intentional (target-vs-current) or an offset
  misread in one decode - audit both readers.
  ALSO: [$04]=7 (FM sec0 ✓ matches the model), [$01]=$10 spt=16 ✓, [$02..03]=128 ✓,
  [$1a..1b]=80 tracks. Sec0=7 + spt=16 + FM-128 = the geometry the model already runs.
  NEXT: (a) the host-side UIB image vs $6e60 comparison (the copy-fidelity check);
  (b) the [d0/d2]-vs-[d4/d6] reader audit; (c) with the $743a block cleared as
  not-our-path, the builder dispatch question returns to the registry/scan pair
  ($94d8/$9a5e) EXCLUSIVELY - and the wall with it.
- **cont.39d/e (runs 128-130): THE POISONER CAUGHT + the probe series' verdict.**
  (1) First-writer taps: the poison has TWO pens - **$7430<-$44 at pc $7e8a (the $7e6c
  scan's store - the overrun mechanism confirmed) and $7428<-$44 at pc $8114 (the DOOR'S
  OWN CONTINUATION, $810e's (A0)+ walk, copying the poisoned $7430 into $7428)**. Boot-era
  writers are init/RAM-test only - not stale.
  (2) MAPPROBE v1 (doorbell-time): the fw's dispatch RE-FILLS the map with $c0 AFTER the
  doorbell (only the probe's [17]=$fe survived, scan terminated at $11 not $44 - clean
  confirmation of the scan mechanism). v2 (first-id-completion): $44 RETURNED - the map
  is re-initialized AGAIN after that ($1242/$1280-family fills run repeatedly through
  setup). VERDICT: the builder's dispatch slot is precise, the map lifecycle is actively
  managed, THERE IS NO SIDE DOOR - cmd $95's ledger population genuinely rides the
  registry/scan dispatch ($94d8/$9a5e -> $73fa), and the byte-test wall is THE blocker.
  (3) Since v2.60 works on real HW, the wall must pass on real flow. HYPOTHESIS INVENTORY
  (fresh-eyes, for next session): (a) a $0aXX dynamic stamp in an unexercised flow; (b)
  [$71b6] populated via boot state we never reach; **(c) - NEW AND TESTABLE - the CPUAP's
  0x87 INIT writes UIB fields our model mishandles: the $7432 builder-gate checks
  UIB[$e]!=0 (never read!), UIB[$11]&2, and class bits - a mis-modeled INIT byte would
  shift the whole class selection far upstream of everything measured.** OPENER: dump
  UIB[$00..$20] complete at 9.79 + audit the 0x87-INIT sequence against every UIB-field
  gate on the scan/builder path (UIB[$e], [$11], [$12], [$20], [$1a/$1b], [$d0-$d6]).
- **cont.39c (runs 126-127): the IDAM-sync gate + the poison persists - the poisoner
  needs its own tap.** The op census at $22a2 was sparse (op 8 only, 3x/era - the $2290
  loop is signal-driven, not the hot path; the hot idle runs the $23b0/$7b1a side).
  WIRED (hardware-true, keeper-shaped): **m_seen_id - the ENDEC cannot recognize a data
  AM before syncing an IDAM; data-boundary events gated until an id boundary fires since
  stream (re)build.** Effects: read-era level-5 takes CEASED (run119-122's IRQ5 source
  now an open oddity - only two model sources exist, both gated; suspect stale HOLD_LINE
  latches or the per-arm stream-rebuild interplay); DOOR-8018 still opens once; IDSTAGE
  1887. **BUT [$7428]=0044 persists by 6.425** - the overrun-entry theory (first-data-
  before-first-id) is DEMOTED to one candidate; others: the $82e2 pending-scan's $8318
  store (7428=D0 on $f0-found... or its D0 leak on not-found), an id-round path into the
  $7exx scan, or the pre-gate first round via another route. MAPSNAP/BGATE still never
  fire. NEXT: first-writer taps on $7430 AND $7428 (value+pc+t, from 6.39) - catch the
  poisoner in the act; then re-run the poison-free question. (Also queued: the run119-122
  IRQ5-source oddity; the $23b0/$7b1a hot-idle census as the scheduler's real hot path.)
- **cont.39 (2026-07-14): THE SCHEDULER REFRAME - node status = NEXT-OP NUMBER; the
  main loop is the op scheduler.** The chain that got here: (1) triangle differential
  (runs 119-122): $7950 diverges - setup's first data-event PAIR escapes the re-arm
  (walk exits via the $7exx tail/$7f22-clear), second event lands phase-1 -> DOOR-8018
  OPENED ONCE (@6.41689, slot stamps ran, the completion writer wrote once at pc $8126);
  read era: every IRQ5's walk exits $7d4a -> re-arm within 5us. (2) The walk rejects on
  **[$7428]=$0044 - poisoned by the $7e6c scan OVERRUNNING a terminator-less map** (the
  $c0-fill has no $ff/$fe; the scan runs into the $7696 geometry table; $7430=$44 from
  the op's FIRST round). (3) The map's $ff/$fe writer = the $7522 SEQUENCE BUILDER -
  NEVER RUNS (SEQBUILD=0 unwindowed). Its routine head = $73fa = **OP $48** (table $1da)
  + two callers inside registry handlers ($94d8/$9a5e). Our 0x95 list has NO op $48;
  **$6102 (cmds $72/$82/$96) builds the FULL list WITH $48** ($42/$48 ordered by
  UIB[$11] bit1) + installs the $299a/$29ea/$29dc/$29ce togglers. (4) THE MERGE: run112
  measured the main-loop dispatcher using the $192 OP TABLE on node "statuses" ->
  **statuses ARE op numbers: $000a = "next op $0a" = $15fe (the wait op - the park is
  the scheduler running the node's wait op each pass); $0048 would dispatch the builder;
  $0008 = $156a; the $17f8/$2244 stamps write the node's NEXT OP.** The resume = an ISR
  stamping the next op number. The byte-vs-word "wall" dissolves into the scheduler
  frame (the scan IS op $a; its byte test is one branch of the wait op's own logic).
  NEXT INSTRUMENT: log the main-loop dispatcher's (D0=status/op, target) pairs per era
  ($230a tap) - setup shows the op progression on tape; the read era shows the exact op
  number where the progression stops, and whoever should stamp the next one.
- **cont.38xx (run 118, SESSION END): THE CLASS NEVER MODE-SELECTS DATA.** SYNCSEL
  transitions: ZERO, both eras - no $2af/$2ff-family (bit7=1) E000 write ever executes
  in this boot. For the $796e==0 class the data phase is E000-PULL + bit15-arm driven:
  $8018 READS E000 under the $a6d operating mode and writes no sync select ($7fc6's
  $2af/$2ff pair belongs to the $796e>0 flavors). CONSEQUENCE: the data-typed pending
  for our class must arise from the bit15/$8a00 chain = $92b4's [$742c]==0 happy branch
  = the $7e8a fork = the phase triangle ($742c/$7950/$7a0e) - cert-2's chain is circular
  at the same three cells, NOW WITH THE COMPLETE MAP SURROUNDING THEM (the mode latch,
  strobes, completions, doorstep, guard, stamp, queue, ledger all measured). FRESH-
  CONTEXT SESSION OPENER: with cert-1 green as the live baseline, take one read-era
  cycle end-to-end (id capture -> $89f2 -> the $7cxx/$7exx rounds -> where $742c/$7950/
  $7a0e each flip) against one setup-era cycle in the same run - the phase triangle's
  first divergent write is the whole remaining question. Tree: v3 + SYNCSEL instrument,
  cert-1 green (IDSTAGE=1888), DATASTAGE=0, default-HLE last verified green cont.38t-v.
- **cont.38vv/ww (runs 116-117: the latch bracket closes on v3 - CERT 1 RECOVERED).**
  v2 (bit7-of-every-mode-write latch + pending-type override): STILL IDSTAGE=0 - the
  gate was choking the STREAM-ERA STROBES (the 6.4x ping-pong runs under $2ff/$a6d and
  needs BOTH mark types - measured working since the 6.6-era). **v3 (the correct form):
  the mode gate governs the CAPTURE ENGINE ONLY (which sync it hunts -> which
  completions fire); the per-record strobes are unconditional ENDEC boundary pulses,
  stream-gated, both types. RUN117: IDSTAGE=1888 - CERT 1 RECOVERED**, the hunt alive
  with the data-AM mark in-tree and the bit7 latch correct. CERT 2 still fails: $8229 x2
  and DATASTAGE=0 - the data-typed completions never fire (no data-typed pending arms,
  or the accept never happens in this regime). NEXT SESSION: diff run117's setup era
  (alive baseline) against its own read era - where the 8229s post (which round, which
  guard) and why no $92b4/$8a00-family data arm ever raises a data-typed pending. ALSO:
  process notes - run114 void (stale anchor), run116 initially void (a grep-gate
  swallowed the run; the run-was-actually-executed check joins the build-mtime check).
  Tree: v3 in, cert-1 green, data-AM mark live, bit7 latch live.
- **cont.38uu (runs 114-115: the mode-gate v1 - FAILED CERT 1, diagnosis in hand).**
  Run114 = VOID (the edit script aborted at a stale anchor; the unchanged run113 tree
  rebuilt - check assert-failures before trusting a "build"). Tree-drift found in the
  re-apply: **the cont.38z bit9-alone IRQ5 strobe survived under the 38u block through
  runs 99-114** (retired now into the mode-gated switch; those runs' regimes carried an
  extra all-modes IRQ5 strobe - re-audit any conclusion that leaned on their IRQ5
  timing). Run115 (mode-gate wired clean): **CERT 1 FAILED - IDSTAGE=0, the id hunt
  itself silenced.** Prime suspect: the $2ff->data-mode latch fires on window-opens that
  are not sync selections (the GATE-READ-DMA $2ff, the boot-era opens) and sticks
  data-mode before the first $22f; irq6 marks invisible -> no id captures ever. THE
  LATCH SEMANTICS TO DECODE NEXT: the real ENDEC almost certainly AUTO-REVERTS to IDAM
  hunt after each data record completes (one data record per $2af/$2ff selection - the
  fw re-selects per record); model: data-mode as a ONE-RECORD state (set by $2af/$2ff,
  cleared at the data-end mark/completion), id-mode otherwise. Also decide $2ff-the-
  window-open vs $2af-the-sync-select roles separately (they may not both latch).
  SPINFLAG still fires (the op-$42 timer path, independent ✓). Tree state: mode-gate v1
  in, cert-failed; next session opens on the latch semantics, re-certifies setup FIRST
  (IDSTAGE alive, no $8229, READ-START born), then reads the data phase.
- **cont.38tt (run 113): the edge unconditional BREAKS THE HUNT - the boundaries are
  MODE-SELECTED.** With the data-AM irq5 mark added per-record unconditionally: the node
  completes EARLY with status $8229 (done + $29-class lock-on error) in the setup era;
  the read era never happens; screen parks. Same shape as runs 75/98: right event,
  unconditional where the hardware is selective. THE SELECTOR (measured all along):
  **the E000 mode register - $22f (IDAM-sync) armed before every id hunt; $2af/$2ff
  (data mode) before data operations. The ENDEC's boundary interrupts follow the mode:
  id-mode -> id boundaries (irq6) only; data-mode -> data boundaries (irq5, BOTH AM and
  end) only.** NEXT SESSION'S WIRE (the final form of the owed edge): track the last
  E000 mode write ($22f vs $2af/$2ff family) as m_endec_mode; pump_tick delivers irq6
  marks only in id-mode and irq5 marks (incl. the new data-AM) only in data-mode, all
  still [$79f8]-enable-gated. The fw's own mode choreography (measured: $22f at each
  $88ac hunt arm, $2af+$2ff at each $7fc6 data arm) then sequences the boundaries
  exactly as the real ENDEC would. Revert nothing - the data-AM mark stays; only its
  delivery becomes mode-gated (and the id marks gain the same gate).
- **cont.38ss (the doorstep): THE SECTOR-DONE BOOKKEEPING = the $802c/$8044/$804c
  common tail** (slot stamp $40, $742e, $808a walk -> $8140 writer) - reached by
  $7fee/$7fc6/$8018, NOT by $7ba8 (which exits via $7f5a). The chain: **$7ba8 (first
  data-side IRQ5, phase-0) installs [$7302]=$7fee -> the NEXT IRQ5 dispatches $7fee
  DIRECTLY -> data D800/C800 setup -> the bookkeeping tail.** IMPLIED LISTEN CONDITION:
  **TWO IRQ5-class events per data record** (data-AM detect + data-end - the real
  ENDEC's two boundaries); the model's serdes has ONE irq5 mark per sector = HALF the
  signal - every read-era IRQ5 lands as a "first" event, $7ba8 forever, tail never
  reached. CAUTION (decode before wiring): the two-boundary reading is inferred from
  handler-chain shape; the $7420/$7422 mode pairs installed per $796e flavor ($1066:
  b7ff/ba00, $10a0: d7ff/da00, $10b6: 97ff/9a00, $10dc: f7ff/fa00, $110e: a7ff/aa00,
  $1128: c7ff/ca00) look like PER-FLAVOR ENABLE/EVENT masks - decode their consumers
  ($7420/$7422 readers) to confirm which boundaries each flavor arms; then add the
  data-AM mark to build_serdes_stream (a second irq5 mark at the record's START) and
  let the fw's own vector chain do the rest. THE MODEL'S OWED EDGE, final form: the
  data-record START boundary (data-AM detect) as an IRQ5-class event.
- **cont.38rr (the $21c0 guard decode): STATUS 8 = "TARGET TRACK UNDER THE HEAD" - and
  the circle closes at one wire.** The guard: $1210 converts the request's next block
  address to C/H/S; cmp vs **UIB[$d2]/[$d0] (the current track/head cache** - the fields
  $6ce4's timeout poisons with $ff/$ffff); match -> stamp 8 directly; differ -> retarget
  + inline seek ($6788) -> stamp 8. THE SPECIMEN EXPLAINED: at read-start the restore
  had the head on track 0, block 1 computed to it -> one match, one stamp, the walk
  proceeded to its $36 park. PER-SECTOR: the guard must be RE-ENTERED with the next
  block - its re-entry is the completion-status post from the sector-done path = the
  $8140 writer chain = the [$7426]==0 window = the consumer = the sector-done event.
  **EVERY link is now decoded or measured; they all terminate at the single unlit wire:
  the per-sector done event entering the status system.** The model's side of that event
  - what the gate array signals when a sector's data transfer completes (E01E bit4 +
  WHAT ELSE: an IRQ4 channel-complete? the F000 status? the $7ff8-family mailbox?) - is
  the one contract left to fix, with the firmware's entire consumption path lit from
  the stamp backward. NEXT SESSION: find the sector-done poster (who writes the node
  status/work item from IRQ context after a data transfer - the $8140 chain's ENTRY
  from interrupt side), and deliver the model event that reaches it.
- **cont.38qq (run 112: THE SHOUT WAS IDLE NOISE - and the label half-dies).** The
  branch-log: D0=$0000 at every read-era dispatch, A3 stale ($6ed6, op-$4a leftovers),
  ALL deeper-exit taps ZERO. Resolution: the main loop's $2304-area dispatcher indexes
  **the $192 OP TABLE (entry 0 = $16c6)** - the 110us calls are the STATUS-0 IDLE
  SERVICE. "$16c6 = resume wrapper" half-dies: $16c6 = the idle/status-0 handler that
  CONTAINS the re-stamp machinery, reached with real work context only from the
  setup-era $241c path (status-8/seek chain). The read-era calls take the earliest exit.
  THE HONEST RESTATEMENT: the read era idles at main-loop speed; the $0a-parked node is
  dispatched by NOTHING (the scan wall stands as the reason); the resume can only enter
  via a STATUS CHANGE posted by an event - the per-sector data-landing event - now with
  every consumer, gate, stamp, and table in the entire resume architecture lit and
  measured. THE SAGA'S QUESTION IN ITS FINAL FORM: what hardware event, in the read era,
  is supposed to change the parked node's status word from $000a - find the WRITER SITES
  that write a node's +$26 with a runnable value from IRQ context (the $224a-family
  stamp measured once at READ-START: "[71ec]<-0008 pc=00224a" - ITS guard/trigger is the
  event; decode $2240-$224a's caller chain next), and which model signal fires it.
- **cont.38pp (runs 110-111: THE TWO-ERA DIFFERENTIAL, self-certified and decisive).**
  SETUP ERA (certified reference): the full chain per 12.5ms - STAT8 ($2244) -> RESWRAP
  (caller $241c) -> RESTAMP2 ($17f8). Its EVENT SOURCE decoded ($2214-$2244): **the main
  loop calls op $28 (the SEEK op, $6788) INLINE per step** (A3->[$799a], returns ->
  $7946/$7948/$7a0a) and stamps status 8 - the setup heartbeat = the SEEK PROGRESSION,
  powered by the model's (working) step events. Read era posts no status-8 because the
  seek is DONE - correct, not the missing event.
  READ ERA: **$16c6 dispatched EVERY 110us from caller $2314 = the $222 STATUS-TABLE JSR
  itself** - a node carries a status >= $10 (the table extends past $232; $16c6's entry
  lives in the extension) - and the wrapper DIVERTS before $17f8 every time (RESTAMP2=0).
  THE LAST DARK FORK: the read-era wrapper's exit path. NEXT SESSION'S ONE INSTRUMENT:
  inside $16c6 when caller==$2314, log {node ptr, node status word, node[$28], which UIB
  selected, exit taken ($17f8 / $1718-deeper / early-out)} - names the divert branch and
  the >= $10 status driving it. Also read the table extension at $232+ (which statuses
  map to $16c6) and the $16f2-$170c early-outs. Everything else in both eras: measured.
- **cont.38oo (THE RESUME LOOP IS ALIVE - the morning's convergence).** RESTAMP=0 was
  the THIRD WINDOW ARTIFACT (tap gated t>9.79; the 8 wrapper calls were at 6.40-6.49).
  PROOF the re-stamp runs: the ancient cont.37-era logs - "STATE26 [7216]<-000c
  pc=0017fe" = $17f8's write, attribution-shifted, in the FIRST measured cycles. **The
  complete status-driven resume loop (park $0a -> event -> status handler ($204c/$2418
  sites) -> $16c6 -> UIB gates -> $17f8 stamps $0c -> main loop dispatches $23e6 -> op
  walk continues) IS ALIVE and cycles at 12.5ms through the entire setup era.** The
  read-era park = the loop receiving NO EVENTS after the $36 op - with every piece of
  machinery now identified and measured-working where events flow. THE SAGA'S FINAL
  QUESTION, exact: what EVENT re-enters the status system per-sector in the read era
  (the setup era's 12.5ms event source vs the read era's silence - diff THOSE two eras'
  event chains: what posts the status/work item at 6.4x that stops being posted at
  9.8x), and which model signal owes it. Everything downstream of that event: measured.
- **cont.38mm (morning session: $a356 decoded + the execution census + the re-stamp
  caller emerges).**
  1. **$a356 = op $54 in full**: copy 24 words from template ($66a, or $6a6 on node[$20]
     bit14) -> $727e-$72ad (six descriptors), 6 words -> $72d6-$72e1 (the slots), patch
     [$7290]=[$71be]. Template A ($66a, OURS - bit14=0 measured) matches the live
     WAITDUMP VERBATIM ($727e{g=0000,m=$248,h=$7964} etc.). Handlers ours by install:
     $7964/$3dbc/$9188/$94ec/$9984/$9398. Template B ($6a6): $836c/$4362, no slot 3.
  2. **THE CENSUS (run 108): none of the installed handlers runs via the registry** -
     $3dbc's 6 hits are its address doubling as $3dxx IRQ4-walker code; $9188/$94ec/
     $9398/$8140/$8290/$7964: ZERO. Dave's caution = the measured verdict: installed !=
     running. The registry is stamped into RAM and never consumed.
  3. **[$71b6] = opts-bit4-gated** (single writer $1144, behind btst #4; our opts=$02 ->
     null BY DESIGN). The wait-scan's first test reads ROM for our class; the second test
     hits the byte/word wall. The second scan entry ($16a8-$16c0) = same byte test.
  4. **$16c6 = a descriptor-called dispatch wrapper** (enters with A3 past a slot; unit
     context by node[$28] bit4 -> $799a/$7442 vs $799c/$7468) that reaches **$17f8 - THE
     RUNNABLE RE-STAMP - when UIB[$11]&4 == 0**. Our UIB[$11]=$97 (bit2 SET) -> skips to
     $1718+ (unread). The re-stamp's caller-side machinery is found; its gates are
     per-unit config bits. NEXT: (a) who calls $16c6 (descriptor-relative entry - find
     the jsr/dispatch that passes A3); (b) read $1718+ (our UIB's path past the skipped
     immediate re-stamp); (c) the scan byte/word wall stands unexplained on real HW -
     hold it as the residual anomaly (v2.60 may genuinely never scan-dispatch for opts=2
     and the $16c6-path re-stamp may be the ONLY resume mechanism - which would make (a)
     and (b) the whole remaining frontier).
- **cont.38kk (the caller grep, TOP OF TOMORROW'S PAGE): OP $54 INSTALLS THE REGISTRY.**
  $8290 has NO textual caller (computed dispatch - a handler continuation). And op $54's
  handler $a356 - IN OUR READ'S OP LIST, measured executing @7.99151 - loads **lea $66a
  (or $6a6, UIB[$20]-bit14-selected) + lea $727e**: $66a = the ROM registry TEMPLATE
  (right after the $63e table), $727e = the first registry slot. **The registry is
  OP-INSTALLED per command by op $54 - NOT vestigial; premise 8's verdict overturns one
  level up.** The wait-scan question REOPENS on the right class. Op $58 ($7346) sets up
  the $7abe/$7abc/$7954 window that $6f44 consumes. TOMORROW: decode $a356 fully (which
  template rows install into which slots, gates), then the registry handlers' continu-
  ations for the $8290 entry (the landing handler + shelf-stocker likely live in the
  $54-installed handler set - $836c and friends), with the whole night's map re-audited
  against "op-installed, class-selected" instead of "vestigial".
- **cont.38jj (the $82e2 round-read, FINAL): $7968 HAS A SETTER - $82b2 - and the ledger
  lifecycle completes.** (1) **$82b2: move.w #$1,$7968** exists 20 bytes above the scan
  site - never executed in our runs (SEQACT zero ✓ execution-dead, not static-dead). The
  class-discriminator relabel PARTLY REVERTS: $7968 = "batch has content"; every $7968
  fork tonight is the CONTENT fork; our side always empty because $82b2 is never reached.
  (2) $82e2 = the PENDING-SCANNER: scans the map for **$f0 = wanted/pending** (NOT a
  failure stamp - the $9312 relabel), found -> $7428 = index; none -> error $69. THE
  LEDGER LIFECYCLE, coherent at last: **$c0 init -> $f0 wanted -> slot# filled (positive)
  -> $ff queued/consumed; $fe = end-marker.** (3) The $8290-$8352 block = the per-segment
  continuation engine (set content flag, cancel $7986 timer, rescan next $f0, $7b10-gated
  $7106 queue-rebuild; $82c0-$82cc: content-flag handoff to $8352).
  THE FRONTIER, TWO NAMED WRITES WIDE, plausibly one caller: **who calls $8290/$82b2 (the
  landing-event handler) and who writes map[r]=slot# (the fill)** - both unexecuted in
  100+ runs, both signatures exact. NEXT SESSION: (1) find $8290's callers (static: what
  branches/jumps into $8290/$82b2/$8296; it is a continuation entry - check the $328e
  soft-vector enqueuer's targets and the $192-op table's unread handlers $a356/$7346/
  $6788-tails for it); (2) grep register-writes into $7654-indexed (the fill's move.b);
  (3) then the landing->fill->$82b2 chain is the whole remaining read, with everything
  downstream ($6f44 queue -> $702e -> DMA -> CMDDONE -> verdict) already decoded.
- **cont.38ii (the $6f44 read, TRUE NIGHT CLOSE): $6f44 = OUR CLASS'S TRANSFER-QUEUE
  BUILDER - and the map relabels one last time.** Setup: [$7956]=[$7abc], [$7958].l=
  [$7abe], **[$79a0] = UIB[$11]&2** (the measured 0002 ✓ - source found). Body (when
  [$796a]==0): walk $7654 from the [$7954] window, [$7956]-1 entries: **POSITIVE byte =
  a SLOT INDEX** (x8+$74c4, linked head/tail into $74ac/$74ae - the $8180-style linkage),
  map entry consumed to $ff; **NEGATIVE byte = skipped** (stamped $ff if it was the
  in-walk marker). THE MAP IS A FILL LEDGER for this consumer: positive = "data in slot
  N, queue it"; negative = unfilled ($c0 init, $f0, $fe - all markers on the negative
  side; the status-code readings were the OTHER class's semantics or wrong). EVERY MAP
  DUMP ALL DAY HELD ONLY NEGATIVES: nothing was ever filled; $6f44 runs correctly and
  queues nothing. THE DARK SEGMENT'S TRUE LOCATION: **the positive-entry writer -
  map[r] = slot# when a sector's data lands.** SECMAP never saw a positive write in a
  hundred runs. Unread candidate: the $82e2 region (lea $7654, never decoded). NEXT
  SESSION: (1) read $82e2-region; (2) grep any move.b of a REGISTER (not immediate) into
  $7654-indexed; (3) the writer's trigger = the per-sector data-landing event = the same
  per-sector consume question, now with its exact bookkeeping target known. Chain: fill
  writes map[r]=slot# -> op $4a arms $7b10 -> $7106 -> $6f44 queues -> $702e (unread
  tail) -> DMA -> verdict.
- **cont.38hh (the $7b10/$7106 reads, NIGHT CLOSE): $7b10 IS ALIVE - and the trail ends
  at $6f44, unread.** $7b10's ONE setter = **$6ed6, inside op $4a's handler ($6ed2)** -
  the op right before $42 in the read's own list, measured executing (OPWALK). The op
  walk arms $7b10=ffff; the first data round after consumes it ($7d6e clears) and calls
  $7106 once per cycle. NOT the next $7968 - a live, op-armed, once-per-walk gate.
  $7106's head: **tst $7968 at the FIRST instruction** (the third $7968 fork of the
  night): !=0 -> the $7654 map-scan body ($7114+, the $7954/$7abc-windowed walk decoded
  earlier); **==0 (OURS) -> SR=$2400, bra $6f44** - the map scan is skipped for our
  class and the flow continues at $6f44, UNREAD, in op $4a's own region ($6ed2-$6f4x,
  beside the $6f22/$6f30 $7b10 tests/clears). NEXT SESSION'S FIRST READ: $6f44 - our
  class's once-per-walk data-round continuation; the $7968-fork pattern says it is the
  path that matters, and it has never been read. The chain: op $4a arms $7b10 -> first
  data round -> $7106 -> $6f44 (dark) -> ??? -> the consume. $7968 now reads as a CLASS
  DISCRIMINATOR (set only by the other command class's machinery; ==0 = our class's
  branch selector at every fork), not a dead flag - relabel pending $6f44's content.
- **cont.38gg (the $7dfe read): BATCH-AT-EXPIRY REVISED WITHIN THE HOUR - expiry is the
  FAILURE exit on BOTH sides; the consume is PER-SECTOR, DURING the batch.** $7dfe =
  three instructions: post error $2029 to [$71be], bra $7c92. With [$7968]==0 (our
  measured state, always), BOTH expiry sides error: id-side -> $8a42/$2029, data-side ->
  $7dae/beq $7dfe/$2029. The $7db4 slot-drain requires $7968!=0 - and $7968's only setter
  is in vestigial code: EITHER the drain path is $99-cmd/other-class machinery (like the
  registry) OR $7968 has a setter the taps missed (SEQACT measured zero nonzero writes -
  strong). WORKING READING: $79a4 = the FAILURE budget ("256 events without completing ->
  $2029"), the read must COMPLETE during the batch, and **the mid-batch consume candidate
  = the [$7b10]-gated block in the data round ($7d68-$7d98: enable juggle, E01E ack,
  bsr $7106)** - $7106 unread, $7b10's setters unknown. NEXT SESSION, reordered: (1) read
  $7106 + find $7b10's writers (the new consume path); (2) the parity/landing instrument
  DEMOTED to the failure-path audit it is; (3) keep the phase question live ($8018's door
  still needs bit-was-1 - whether $7106/$7b10 is the same chain from the other end or a
  third entrance). The day's last lesson banked twice in one hour: error-shaped code
  isn't an error path (cont.38z) AND completion-shaped structure isn't a completion path
  - only the measured round settles either.
- **cont.38ff (the $7d68 read): THE TRIGGER IS THE BUDGET - the consume is
  BATCH-AT-EXPIRY.** The data-round exit: $7d9c subq $79a4; beq $7dae; else bsr $88ac
  (re-arm). At EXPIRY: no re-arm (phase stands) and the SLOT PROCESSOR runs ($7db4+:
  walk $74c4 slots by [$742a], status $40/$80 -> clear -> **bsr $352e queue-append**,
  D1=2 loop). THE READ'S RHYTHM: hunt/collect for [$79a2]=256 events (~1.6s at 12.5ms -
  the measured pacing), then stop re-arming and drain. $79a4 was never a timeout - it is
  the BATCH WINDOW. BOTH rounds decrement it ($8a3c id-side, $7d9c data-side): id-side
  expiry -> $8a42 -> $7a0c!=0 -> $2029 ERROR; data-side expiry -> $7dae -> consume -
  **which side receives the 256th event decides error-vs-consume: a PARITY question**
  (run77's expiry inference "always $2029" was never measured for landing side). RESIDUE
  (next read): $7dae gates the slot walk on tst $7968; beq $7dfe - the dead flag again;
  $7dfe UNREAD - skip vs a different consume flavor. NEXT SESSION, in order: (1) read
  $7dfe-$7e0a (the $7968==0 expiry side - OUR side, since $7968 is measured dead);
  (2) instrument the expiry landing (which round takes $79a4 to 0, both sides' pcs);
  (3) the parity/alternation audit: what the model's event pattern does to the landing
  side vs real alternation. The chain now reads: hunt 256 events -> data-side expiry ->
  slot drain ($352e) -> $74ac queue -> DMA -> CMDDONE -> verdict, with $7dfe the one
  unread branch on OUR flag state.
- **cont.38ee (run 107): THE PHASE TRACE - both clearers named; the no-re-arm exit is
  the shape of the fix.** Per cycle, on camera: data-mark IRQ5 -> $29c8 toggles 0->1
  (bit-was-0 -> $7ba8, wrong side) -> 6us later **$88d6 clears** (a clr $7950 INSIDE
  $88ac's entry gap $88c8-$88f0 - every re-arm zeroes the phase); id mark -> $2994
  toggles 0->1 -> $89f2 -> **$8a3e clears** ($8a38, the count-branch exit) -> $88d6
  clears again on the re-arm. The hunt NEEDS phase-0 at id marks (consistent by design);
  the consume needs phase-1 standing at the data mark. **The correct flow must exit an
  id round WITHOUT re-arming** - and that exit exists: $8a42 (lock-on complete) skips
  $8ab8's re-arm and leaves the toggle standing. HD trigger = the $7a0c count; THE
  FLOPPY TRIGGER IS THE QUESTION. Candidates for next session, in order: (1) re-read
  $89f2's floppy branch for a match/skip condition upstream of $8a38 (the $8a14-$8a36
  buffer check is bit1-gated HD-only as decoded - verify no floppy analog hides in the
  $8ab8/$88ac path); (2) the $7ba8-side data-mark round (the map-walk on the RETAINED id
  - the GRD am=fe rounds ARE the data-mark rounds, clarified) - does its wanted-sector
  path ($7e0a/$7e50) have a no-re-arm exit that leaves phase for the NEXT data mark;
  (3) whether the DATA-typed capture completion (in-tree, IRQ5-delivered) is itself the
  event that should arrive bit-was-1 - i.e. the consume rides the COMPLETION after an
  accept round that skipped the re-arm. The sequence hunt->accept->no-re-arm->phase
  stands->data event->$8018->consume is the full remaining shape; every element is now
  named and only the accept's no-re-arm trigger is unmeasured.
- **cont.38cc/dd (run 106 + the $1042 decode): THE ROUTING NAMED - one phase bit from the
  consume.** VEC5WR: ZERO writes ([[$7300]] never re-installed in the read era); TRAMP5:
  every IRQ5 take routes $29c0. THE $796e SELECTOR DECODED ($1042-$113a): IOPB opts&3==3
  -> UIB[$12]-gated {+1/-$10/-1} flavors (each installing its $7420/$7422 mode pair and
  enable bits); **opts&3 != 3 (OURS: opts=2, the sys-floppy-check) -> $1134: $796e=0 BY
  DESIGN.** $796e=0 is not a missing state - it selects the $8018 dispatch class with NO
  $7ba8 install. **$8018 IS the consumer chain's head**: D800/C800 loads, E000 read, the
  $802c bit11 pulse, then $8044 tst $741c; beq $7f6c = the $7f22 region -> $7426 clear ->
  $808a walk -> $8140 -> stamp -> queue. THE LAST LINK: $29c0's bchg fork routes IRQ5 to
  $8018 only when [$7950] bit0 == 1 at the take (bit-was-1); every measured take had 0 -
  the id-side rounds clear $7950 on their way out ($8a38 clr in $89f2's floppy branch;
  also $7b3a/$7d56 in other flavors). ONE RUN DECIDES: trace [$7950] writes between an id
  capture and the following data mark (who clears it, on which branch) - the correct flow
  must leave the $298c id-toggle's 1 standing so the data-mark IRQ5 lands bit-was-1 ->
  $8018 -> the mapped chain to the verdict. (Note the earlier "$8018 = third OPH/data-pull"
  and "$7ba8 = data handler" labels partially swap under this reading - $7ba8 is the
  id-side/hunt service for this command class; re-audit those labels with the phase trace.)
- **cont.38bb (run 105): THE STOPWATCH - not late, ABSENT (regime-dependent).** In the
  current regime (bit9-alone strobes + typed captures): CNT7426 shows ONLY sets (<-0001,
  pc $7d68 = the $7d62/$7d4a-path setter, every 12.5ms) and **ZERO clears; WALKGATE
  ($8092) and CONSUMER ($7f1a) NEVER EXECUTE in the read era.** The 24.9ms "lag" was
  run91's regime (where the $7f22-flow ran every ~25ms - on what clock? almost certainly
  the $2b58/PIT tick); here the consumer never runs at all. SHARPENED READING (Dave's
  poll-vs-event, resolved by regime comparison): the $7f22 consumer flow is an IRQ5-side
  branch ($7f0e-$7f5c region, beside the $7fee/$7fc6/$8018 dispatch) - on real flow it
  rides the DATA-RECORD IRQ5; in regimes where the model withholds that event it either
  rides the 25ms timer (run91) or starves entirely (run105). ALSO: this regime's rounds
  run the $7d4a path (setter $7d62), never phase-1/$92f6 ($79b8 stays 0) - the round
  content is regime-dependent too. EVERYTHING converges on the single unpaid debt: **the
  data-record IRQ5, delivered at the record boundary, wakes the consumer chain
  ($7f22-flow -> $7426 clear -> $808a walk -> $8140 writer -> $8170/$815e stamp)**. The
  next session's task is no longer diagnostic: deliver that one event faithfully (the
  bit9-alone gate is already wired in-tree; what remains is verifying the consumer chain
  fires from it and the spin-up era tolerates it - the run104 markers say spin-up
  completed fine under it: SPINFLAG fired, no fast-fail, boot green).
- **cont.38aa (static, session end): THE GATE PAST $79b8 = [$7426]==0, AND IT'S A RACE.**
  The $808a walk's approach to the $8140 completion writer: $8092 tst [$7426]; beq $810e -
  the writer needs [$7426]==0 with $79b8=1 (already available). $7426's writers, decoded:
  SET=1 at $7cac (accept continuation), $7d62 ($7d4a path), $92fe/$9342 (the $92b4 rounds
  - every 12.5ms); CLEAR=0 at $7b0c ($7b02 match), **$7f1a (the $7b18-consumer flow that
  MEASURABLY RUNS - DREADY's clear at pc $7f28 is $7f22's write, 4 bytes past $7f1a)**,
  $7c0e ($99 cmd), $84b0, $95ce, $79fc (dead registry code). READING: $7426 = "record
  event outstanding" - set per round, cleared by the consumer. THE RACE: DREADY timing
  shows the consumer's clear lands ~24.9ms after each round's set - i.e. ~74us BEFORE the
  NEXT set; the [$7426]==0 window is ~74us/12.5ms, and the $808a walks (IRQ-tail side)
  apparently never land inside it. On real flow the consumer presumably runs promptly
  after each round (not at a 24.9ms lag) - the lag itself may be the model's last artifact
  (what dispatches the $7f22-consumer, and why 24.9ms late?). NEXT SESSION'S OPENER, in
  order: (1) tap $7426 writes with timestamps against $808a-walk entries (the race, on
  camera); (2) what dispatches the $7f22-consumer and why it lags 24.9ms (the main-loop
  status poll? an event the model delivers late?); (3) the full $92f6-round measure (the
  38z reinterpretation audit) folds into the same log.
- **cont.38z (run 104): THE POISON IS THE STEADY STATE - and the $742c interpretation
  itself is now the open question.** bit9-alone strobes on the current base: HAPPYSTAMP=0,
  RESTAMP=0, DATASTAGE=8 (the run98-family regime reproduced). THE EARLY WINDOW's finding:
  **FLAG742C $7eb8 fires from 6.40453 - one event after the doorbell - every 12.5ms, in
  EVERY regime, before spin-up, before everything.** [$742c]=1 is not a strike; it is how
  the machine runs from the op's first moment (run78's identical 6.40453 timeline confirms
  across regimes). REINTERPRETATION CANDIDATES (flagged, unbanked): if the fw sets $742c
  on the first event BY DESIGN, then $92b4's [$742c]!=0 branch ($92f6) may be the NORMAL
  per-sector path, not the fail path - and it sets $79b8=1, which is EXACTLY what gates
  $8158 toward the $8170 $fe-stamp + $74ac queue-link. The $fe interpretation may flip
  back to done-queued; the "$9318 f0-stamp" may be normal per-target bookkeeping. The
  whole fail-path reading (cont.38o-s) inherited the $742c=fail assumption - audit it the
  way premise 8 was audited: what does the $92f6 path DO measured end-to-end (its $9318
  stamp, its $79b8=1, whether the $8140 completion writer is reachable from its
  continuation), not what its error-shaped code suggests. NEXT SESSION'S ORDER: (1) tap
  the $8140-writer's CALLERS (the $810e $7430-walk entry) - why it never runs is now THE
  question, with $79b8=1 available from the $92f6 rounds; (2) the consume-side reading of
  one full $92f6 round (what it writes, where it exits); (3) only then re-order the
  IRQ5-gate A/B - the gate question may dissolve if the consume path was always reachable
  through the "fail" branch.
- **cont.38y (run 103): PULL-ABSENT confirmed + THE CIRCULAR GATE.** The E000 tap in the
  accept->pull window (9.798-9.845): the ONLY reads are $8924's sixteen $22f-prime flush
  reads - no $7fc6, no pull loop. WHY: the pull lives in $7fc6, an IRQ5-DISPATCHED handler
  ([[$7300]]-side togglers $29a4/$29c0 route phase->$7fc6/$7fee/$8018) - the fw expects
  the data-record boundary IRQ5 to CALL the puller, and the puller's own $2af/$2ff is
  what opens the mux; my model gates IRQ5 strobes ON the mux being open (stream-only
  strobes, cont.38u). THE DEADLOCK: fw waits for the interrupt that runs the opener; the
  model waits for the opener before delivering the interrupt.
  THE REMAINING CONTRACT QUESTION (one A/B): on real HW the data-record IRQ5 = the
  ENDEC's record-boundary detector, gated by bit9 ALONE (independent of the read mux).
  Run98 refuted bit9-alone - BUT its poison hit at 9.79994, i.e. DURING SPIN-UP, where
  bit9 is also enabled; re-examine whether run98's spin-up poison is a model artifact
  (e.g. phase-state mismatch in the 6.4-6.7 setup vs the togglers) rather than a real
  refutation of bit9-gated data-boundary IRQ5s. Candidate narrower gates to A/B if
  bit9-alone stays refuted: IRQ5-at-data-marks only while [$741c]=1 (op active) and
  post-spin-up ([$7a3e] fired), or only while [[$7300]] holds a $29xx toggler (the fw's
  own vector state = the era signal). The ID-accept -> IRQ5($7fc6 pull) -> synchronous
  E000 data read -> consume -> $17f8-family re-stamp chain is the last dark segment;
  everything on both sides of it is measured.
- **cont.38x (runs 101-102): THE EMPTY DIFFERENTIAL - the arm carries NO type.**
  run101: the agnostic regime (38v) kills the op cycle at 8.10 DURING SPIN-UP (data
  records fed to ID-intent arms; 127 READ-STARTs all pre-8.1; no read era) - agnostic
  breaks spin-up, $22f-typing breaks post-accept: the selector differs per era. run102
  (typed regime restored, tuple across both eras): **ARMTUPLE IDENTICAL AT EVERY ARM -
  {7a16=0010, 7a18=0000, tbl[10]=00, e802lo=d3} - spin-up and read era, hunt and
  post-accept, all three arm pcs ($891a/$89b6/$7f78).** No firmware-visible state
  distinguishes the arms. THE READING THIS FORCES (bank as the working model, verify
  next): captures are ID-ONLY by hardware semantics - the $22f prime IS sync-to-IDAM -
  and **the DATA phase is the SYNCHRONOUS E000 STREAM PULL** (the saga's oldest measured
  mechanism: the 6.6-era "scan = synchronous pull, D0=0"); the post-accept flow arms the
  pull via $7fc6's $2af/$2ff (which opens the mux legitimately in the existing model).
  The r=01 data record "sliding past uncaptured" was CORRECT behavior; what must be
  verified is whether the fw issues E000 pulls at the data-record time post-accept and
  whether the mux serves stream bytes there. NEXT SESSION'S ONE INSTRUMENT: an E000
  READ-tap in the post-accept window (t 9.80-9.83): does the fw pull, what does it get
  (stream bytes vs status), and does the consume follow. If the pull is there and starved,
  the fix is in the mux/serve; if absent, the accept path's pull-arm ($7b4x/$7fc6 chain)
  is where to look.
- **cont.38w (the $63e decode, static half).** The table = OP-FAMILY BIT PATTERNS: three
  16-entry unit-class rows of {$00,$20,$40,$42,$60,$62} - bits 6/5/1 of the E802 LOW byte
  (measured arms aad3/42d7 carry family $42, matching row-1 entries). Indexed
  table[$10 + [$7a18]] | $80 by every arm builder. **[$7a18] = the PHASE INDEX, loaded
  from the queued WORK NODE at each channel kick** (move.w (A3),$7a18 at $3d2c/$4196/
  $4318/$4542/$4868 - the $3d24-$3d4a kick sequence; the node's +$1c field carries it,
  the "748a+1c<-0015" logs). The selector chain is FIRMWARE-SIDE end to end: op walk
  stamps the node's family index per phase -> kick loads $7a18 -> arm writes the family
  into E802's low byte. THE MODEL'S JOB: interpret the family bits it receives at each
  arm (which family = which record type/mode). BONUS: the registry's static init template
  ({0,$248,ffff,$7964}...) sits immediately after the table at $66a - the vestigial
  structure is a data image, confirming cont.38r.
  NEXT SESSION'S SINGLE INSTRUMENT: log per arm (bit15 rise / E802 write in the accept
  window) the tuple {[$7a16], [$7a18], table-entry, E802 low byte} against the round that
  follows (id round / data round / consume) - the family-to-record-type map falls out of
  one run, and with it the correct capture-type wire replacing the retired $22f proxy.
  (cont.38b context below.) Post-layout-
  fix runs (66-72), the chain measured end-to-end:
  1. Marks at the TRUE 12.5ms ID cadence (16 IDs/200ms rev - the "retry cadence" WAS the mark
     cadence); IRQ6 enabled (aad3/4ad7 bit11) and TAKEN per mark via [[$7304]]=$298c; TRAMP5
     shows [[$7300]]=$29c0.
  2. B15EDGE per cycle: FALL@$88c6 (entry andi #$67ff clears bit15) / prime pulse $891a-$8936
     / final arm RISE aad3 @$89b6 held -> pending=1 SPANS each id mark -> capture stages AT
     the mark -> IRQ6 -> $89f2 processes -> re-arm 3us later. The machine walks ALL 16 IDs.
  3. $298c = phase fork: bchg $7950, phase0 -> $89f2 (ID processor), phase1 -> $92b4 (data
     processor, leads to the $9318 map[$7428]<-$f0 retry stamp on failure). $9318's index =
     [$7428] (expected-sector), stamped every failed data phase.
  4. **THE ACCEPT MEASURED (run72 @8.00088): capture r=01 == [$7428]=1 -> phase switches;
     12.5ms later $92b4 examines the buffer expecting SECTOR 1's DATA and finds the next ID
     record (r=02) the id-only model staged.** [$7428] advances one sector per REVOLUTION
     (accept -> data-fail -> $f0 retry -> re-hunt) - the definitive signature.
  5. THE CONTRACT (final form): **the armed capture is RECORD-TYPE-AGNOSTIC - it completes
     at the NEXT record boundary, either type.** irq6 mark -> ID record; irq5 mark -> the
     sector's DATA record. The fw re-arms per record ($88ac after an accepted ID arms for
     the DATA record that follows).
  - WIRED: stage_data_record(dst, mark) - the stream bytes between the preceding id_end+4
    and data_end-2, framed A1 A1 A1 FB FF <data> 00 00 (the ID capture's measured valid-byte
    idiom); pump case 5 completes a pending capture with it (op_ok + IRQ5 per bit9); dst
    read LIVE at the mark (the phase handlers rewrite D800 per record). Sector-map codes
    ($7654): $c0 init ($1280 fill + $9362), $f0 = retry-stamp ($9312, idx [$7428]), $fe =
    data-captured + slot queued to $74ac/$74ae ($8174), $ff/$aa checked by the walkers.
    Run73 pending.
  - **Parked-state reading of $7964 (why dispatched-every-scan but no progress)**: with
    $796c=0 and $7986=0 the driver would fall to $7a62 (deregister + node status $c =
    COMPLETE) - but live registry stays populated, so the park must route via $79b6!=0 ->
    $7a9a: tst $7a0e (no match) -> node-cmd $71/$99 checks -> $74ac/$74c4-table work via
    $34ee = the RETRY path. The park is $7964 retrying the capture that never matches -
    exactly where the staged record (stage_next_id on the fw's own E802-bit15 arm) closes
    the loop. The floppy-side gate-clear for [$72de] should then follow the same
    match->wake idiom measured at $74a6-$74c4.
  Everything else behind it is measured.

### Build #5 cont.26 (run49) - the STARVATION REFRAME OVERTURNED TOO: the pipeline FLOWS; the
### frontier = the $74ac op-queue's WALKER
- **Dave's count-not-presence instrument: the chain is NOT starving.** SLOTWR shows the full
  cycle per record: APPEND ($32e0 writes status 0080 into slot N), CONSUME ($80f8-era clears +
  `bsr $352e` re-queues), RELINK ($3554/$34bc) - slot0, slot1, slot2... flowing. Run47's
  "empty slots" was a SAMPLING ARTIFACT (XFERGATE read at walk-entry = post-consume/pre-append,
  always between meals). The starvation theory is dead; so is UIB-bit11 before it.
- **The consume RE-QUEUES each slot via $352e onto the $74ac-headed op queue** (QUEUE-INSERT
  logs: nodes 74c4/74cc/74d4... head=$74ac, per record) and branches $8214 when the count
  drains. The HOST TRANSFER = those queued ops EXECUTING. $1242 (called at unit-select) = a
  queue-primer, not the walker. **The walker = the $3282/$32ca op-queue engine (unread)** -
  the same machinery whose premature IRQ6-on-enqueue pump was retired; its FAITHFUL execution
  trigger is the one thing left. NEXT SESSION: read $3282-$32ca + who reaches it (main-loop
  poll? an IRQ?); find the honest trigger; DMAFIRE ($13be) and HOSTDATA (0fc0dd - WIDEN the
  tap window to 0fc0d0-0fc2ff for the full first block) catch the transfer; then the built
  completion carries the verdict.
- **cont.25 addendum (run48): the [$792e] recompute is NOT this read's mechanism.** SRCGATE
  ($4a7c) = 0 hits; $4a66 is an entry in the $152 dispatch table = the 0xA0+ COMMAND family's
  handler (the $d88 `subi #$30; lea $152` second-table path), not the 95's flow. ALSO: the
  dispatch DOES stamp UIB[$20]=0x8c27 (bit11 SET) at $e7e - the config was never missing.
  **REFRAME: the starvation = the kickoff CHAIN breaks after slot0.** Slot0 was staged once
  (stager unknown - pre-transfer setup?), consumed, links reset to the $78c4 EMPTY sentinel;
  the per-record buffering (path B, $92fe-$9330) should APPEND each buffered sector to the
  chain and doesn't visibly - the appender is in $9330's continuation ($9370+) or elsewhere,
  with its own gate. NEXT: write-tap the slot words ($74c4-$74cb) to name slot0's stager +
  every appender attempt; read $9370+ (path B's continuation); the missing append's gate is
  the one input left.

- **MODE BIT DECODED (static, earlier): [$799a] = the CURRENT UNIT'S UIB pointer** (single
  writer $84a = the boot UIB-init, called 3x with A0=$6c00-family/$6e60/$6f90; also inits
  UIB[$d0-$d6]=ffff + copies 0x20 defaults from $5de/$5fe/$61e). The ISR's mode gate reads
  **UIB[$12] = a per-unit config FLAG byte**: the floppy unit's UIB ($6c00 dumps) = 0x37 (bit1
  SET = ID-check mode); UIB@$6f90 = 0x45 (bit1 CLEAR). [$799a] parks on $6f90 (last boot init)
  and run6's working scans ran with UIB@6c00 active - the honest path scans with the WRONG
  UNIT'S UIB selected. NEXT SESSION: (1) log [$799a] live + dump [$799a]+$10..$17 at scan time;
  (2) find the per-command unit-select that should re-point [$799a] (or copy the unit's flags
  into the current UIB) - no other $799a writer exists, so suspect the fw copies INTO the
  common UIB, or the model's one-off UIB-fetch C++ copy (~770-796, 1880-1926 per the old notes)
  targets the wrong UIB honest-path; (3) also check the refetched IOPB's 8181 status bytes
  (stale completion markers - the re-fetch may need to deliver the PRISTINE IOPB image, check
  what the host actually has at 0fe780+2 vs what an earlier model DONE-post wrote there).

### Scaffold status (superseded by the per-op model above; STRIP the old proof-of-thesis notes)
The old `SEEK-DONE-POST` per-spin block is GONE (replaced). It PROVED option 1 but could not sequence.
All checkpoint probes (`SEEK-DONE-POST`, `SHIMTEST` gates on site 2131, the F000rd/GATE260c/NODEWALK/
71b2<-/Xdisp/Xdecr/DISP-PRE/E800kick/DONE-POST taps, `STORAGER_WALKTRACE`, the spin.lua) are
`STORAGER_NOBYPASS`/`STORAGER_PHASELOG`/`STORAGER_SHIMTEST`-gated and dormant by default.

### Shipped behavior: INTACT throughout — default (no NOBYPASS) boots SINIX-M-C V2.0 on `-hard1`. Verified repeatedly.

## CUTOVER BLOCKERS (running list - the robustness pass before the LLE signals go default)
None block the frontier work; ALL must land before de-gating NOBYPASS/STEPIRQ/SEEKACTIVE/STEPBIT0:
1. **A118 probe segfault** - `STORAGER_A118` E802-write log block crashes MAME (__dynamic_cast in
   its 4-connector get_device() loop). Fix or delete WITH the strip.
2. **Wall-clock-seeded boot intermittent** (runs 15/16) - CPUAP diverges ~1.34s, never reaches
   testend (pc532-beachball class, presumably RTC-seeded). Harmless for dev (relaunch), but must
   be root-caused or confirmed pre-existing (test: does the DEFAULT HLE boot also flake?) before
   the LLE path ships - a default-path user hits it cold.
3. **Repro fragility** - the LLE read path currently requires the FULL knob stack to be reachable
   at all, and the HD-label read also rides cmd 0x95 (the stg2 HD flow dies under NOBYPASS before
   SINIX1). Once de-gated these are one path: every 0x95 issuer (boot floppy check, HD label read,
   SINIX driver I/O) must work through the same LLE machinery, not just the floppy-check repro.
4. **Experiment-comment audit (Dave, post-M1):** the cmd-95 CHANCOMPLETE suppression's rationale
   was measured BACKWARDS (it claimed the IRQ4s hold [$749c] busy; M1 proved they CLEAR it) and it
   strangled every honest-path completion. Before de-gating, audit EVERY env-gated experiment
   block's comment against measured behavior - one backwards belief survived this long; others may
   be wrong too.
5. **Boot flake: MEASURED NOBYPASS-SPECIFIC (focused pass, build#5 cont.8). DEFAULT HLE: 0/12
   flakes; knob stack: ~20% (stable across all pins).** Storager-introduced, NOT inherited - a
   robustness bug in our env-gated signal stack. Detection recipe: 2s boot, `grep -c 4FFFFC
   error.log` (0 = flake; the CPUAP's FE02B6 RAM-size probe) - a full test cycle is ~3s.
   ELIMINATED as the varying input: MC146818 RTC registers (CPUAP_RTCFIX pin: still 2/14 - but
   VERIFY mc146818::device_reset doesn't re-seed AFTER the cpuap poke, which would void this),
   the RTC-IRQ line phase (identical 2 toggles @t=0 in flake and good), NVRAM oscillation
   (wiped per-run: 2/12), heap-uninit (MallocPreScribble: 2/8), entropy calls in
   ns32000/ns32082/ns32202 (none). Divergence: the CPUAP's SILENT first ~1ms (before its first
   logged event; the sizing probe simply never happens in a flake; everything else in the logs
   is timestamp-identical). NEXT: (a) knob-bisect - 6 knobs x 12 cheap runs names the injecting
   signal (prime suspect: STEPIRQ's boot-time stepdone chain - STEPARM fires @0.00001 from the
   B0-init E802 write, mailbox+IRQ2 @0.030 + FRAME-IRQ4 @0.032 into the storager 68000 MID-INIT);
   (b) verify the RTCFIX poke ordering; (c) if needed, disassemble the CPUAP ROM's FE02xx sizing
   loop for its exit condition's data source. TEMP probes for this hunt (STRIP): CPUAP_RTCLOG
   tap + RTC-IRQ lambda log + CPUAP_RTCFIX poke (all in cpuap.cpp, env-gated).
   **Bisection attempted, INCONCLUSIVE at affordable n:** single-knob drops at n=8 suggested
   STEPIRQ/STEPBIT0, but n=16 refuted (minus-STEPIRQ 2/16 vs full 1/16 - the 0/8s were luck).
   The rate is NON-STATIONARY (2/8 -> 1/16 same config across an hour) at pinned inputs ->
   suspect per-exec host state (ASLR-class) reaching an emulation decision; statistics can't
   bisect this at 3s/run - needs the deterministic divergence instrument (CPUAP early-window
   instruction trace; the headless -debug/-debugger none + debugscript path produced no trace
   file, debug why, or use a gdbstub session). **PRAGMATIC INTERIM (stops the run bleed): every
   scripted measurement run should pre-check `grep -c 4FFFFC error.log` at ~5s wall and
   auto-relaunch on 0 - a 3s retry beats a wasted 35s run.**
6. **Strip list below** (all TEMP probes + the env gates themselves at cutover).

### TEMP probes to STRIP (added this session)
- ⚠ `STORAGER_A118` E802-write log block SEGFAULTS (\_\_dynamic_cast in its 4-connector get_device()
  loop) - fix or delete WITH the strip; do not leave it to bite the cutover.
- `STORAGER_PHASELOG` block (device_start): the AS_OPCODES fetch taps + the D800/E000/E802 PHASE logs.
- Task-#2 additions: phase-array entries `DISP-ERR`(`$d60`)/`ERR1206`(`$1206`); the `HANDLER-RESULT`
  (`nm[0]=='E'&&nm[1]=='R'`) case; the `ccbcmd` `$71f0` write-tap; the `STORAGER_ALLCMD` scaffold gate
  (`m_iopb_cmd != 0x95 || getenv("STORAGER_ALLCMD")` — revert to plain `!= 0x95` on strip).
- Build-#4 additions: the `GATE260c` RING dump's extra `7e1e`/`node+28` fields (diagnostic only). The per-op
  model itself (`m_seek_iopb` latch, `m_seek_fired` semantics, the `ch_w` per-op completion block) is the
  REAL implementation, NOT a probe - it stays, but its `logerror` + the `STORAGER_NOBYPASS` env gate come
  off when the LLE read is done and NOBYPASS becomes the default path.
- `storager.cpp` UIB87 `logerror` (in the 0x87 block, gated `STORAGER_NOBYPASS`).
- The `$352e` pc-range in the `opqueue_irq6` pump-arm (identify uses host→local DMA + IRQ4, NOT the
  IRQ6 pump — the `$352e` extension is unneeded for the identify; keep only if a read op ever queues
  via `$352e`).

## Resources
- ROM `siemens/storager/storager_v260.bin` (`v260`); disasm `siemens/disasm/storager/storager_v260.asm`.
  Entry points: read handler `$5de4`, channel config `$5e98`, disk→SRAM arm `$5ede`, SRAM→host
  DMA fire `$13be-$13ce`, step loop `$24ae`, DMA self-test `$9cb0`.
- **Durable HLE reference:** `docs/storager-lle/storager.cpp.hle-reference` (pre-cutover known-good).
- Repro: `docs/storager-lle/stg2-repro.lua` (SINIX1 = `siemens/set1/mx2-002.imd`); flchk-print
  image `/tmp/stg2-patched.img` (redirect stripped so verlangt/verfügbar print — regenerate from
  `stage1.img` by blanking `>/dev/null 2>/dev/null` at HD offset 227243).
- Probes (env-gated, STRIP before final): `STORAGER_DMALOG` (observer), `STORAGER_NOBYPASS`
  (behavior-changing dev knob — gates cmd-0x95 read leg ~line 1943).

## cont.40 — the delivery architecture, fully mapped (runs 145-157, 2026-07-14)

**Dave's two questions answered.**

### Q1: $79b6's writers → the entire transfer subsystem decoded
- $79b6 (transfer-active) is armed at $95e4 inside $94ec, guarded by `tst $79a8` (owed count,
  seeded = IOPB byte 7 = 8 by op $56 at $a486). $94ec is a MINI-TASK (record $7296, cell [$72dc]).
- Six 4-word task records {park, watch-ptr, last-seen, handler} at $727e..$72a6, installed by
  op $54 from ROM template $66a (cmd $95 class; $6a6 for bit14 class). Handlers: $7964 (batch
  driver), $3dbc (delivery driver), $9188 (finalizer), $94ec (transfer), $9984, $9398 (sequencer).
- Dispatcher = the $15fe scanner (= op-$a handler, table $222: op8=$156a walker, op$a=$15fe),
  cursor [$727c] over cells [$72d6..$72e0]. Gate: park==0 AND `cmpi.b #$a,($26,node)` —
  **the byte test reads the status word's HIGH byte; every stamp in the ROM writes word $000X
  ($2/4/6/8/a/c), so the gate can NEVER pass. Measured: SCANMATCH tap = zero hits all boot.**
  A passing state ($0aXX) would crash the word-reading op scheduler ($2304 table jsr) — the
  scanner's node gate is vestigial (SMD-family inheritance). All real task activity = direct
  calls (door tail $8338 bclr $7a30 → bsr $3dbc; IRQ-side wake writes).
- THE RESUME (the working path, no tasks needed): batch end → $7968=0 → next walk's $7e6c scan
  sets $742c=1 ($7eb2/$7ea4) → door completion takes the $81b4 DELIVERY flavor ($8152: [$742c]≠0)
  → slot queued to $74b4 (status $10/$18) → door tail's $3dbc drains it: $3e50 builds the $7442
  channel descriptor (buffer addr = $77f8 slot table[slot], wordcount from $7696) → E800 kick per
  slot → IRQ4 chain → queue dry + $7958 drained → $3dbc from MAIN-LINE ctx ($3e08 btst #10,SR:
  clear = mainline) → $3e1c teardown → node←$c → CMDDONE. For the floppy class $7958=0 is
  AUTHENTIC ($7abe gets IOPB bytes $a-$b = the count, then op $58 ($7392) zeroes it; the host
  address 0fc0dd lives in IOPB bytes $d-$f and reaches the gate array elsewhere).

### Q2: the truck → CONFIRMED MISSING, semantics decoded
- The model's ONLY data delivery is the HLE shim (`!STORAGER_NOBYPASS`-gated, serves m_sectors).
  Under NOBYPASS the fw's E800 kicks would fire into nothing. NEEDS BUILDING.
- Hardware semantics measured/decoded: D000 = local word addr ($13be: byte addr >>1); E800 =
  control + bit12 GO (edge); C800 = 0x80-entry per-page host translation file, LOADED BY THE FW
  via the $308c script walker ($3066 splits a word into {reg index=(v&$f80)>>6, ...}); host addr
  = C800[page]-based. E807 = write-side data byte (not address). Direction/family = E800 bits
  5-7 from the $63e table ([$7a16]+1 vs +2 = the two directions, $1390/$146a primitives).

### The current wedge (root cause, measured to the microsecond)
- run154: drain (IRQ6 $79a4-expiry, $8a42) clears $741c at 9.26343; the ADJACENT data mark's
  IRQ5 walk re-arms it at 9.26376 (33us; no door can interleave — pending IRQ5 taken at rte).
  The race is AUTHENTIC (marks 33us apart), so the design cannot depend on a door seeing
  $741c==0 there. The REAL batch-end signal is upstream: the $92xx sector-engine's $933c side
  ($9342: $7426=1, $79b8=1, clr $741c → end-door $8170 flavor) — which never runs because the
  $2970-family IRQ thunk continuations (bchg $7950 phase → $925a/$9284/$92b4) are never
  installed in [$7300]/[$7302] during our read. NEXT PROBE: [$7300]/[$7302] write history —
  what installs the engine continuation, and why not here.
- run157 forensics: walks post-9.2 hunt target [$7428]=$0011 (17!) = the $fe terminator cell —
  the $7e6c scan's $7968≠0 side checks for $aa (HD marker), ignores $fe, promotes past the end.
  Target 17 = mechanical consequence of $7968 never dropping (the upstream signal above).
- Also: $7968's droppers enumerated: $155c/$14e2-family (batch-closer: called at NEW command
  $dc6/$e4a, SEEK $6806, cleanup $193e), $9342-era, $79ca+$82c6 (blocked: $796a≠0/$741c=1).

### Keepers/probes this session
- cont.40a KEEPER: level-at-mark capture eligibility (E802 bit15 live at the mark, from m_ch) —
  hardware-correct (tolerates $92ce's us-scale re-arm dips, honors sustained disarm). Did not
  change run156 (the ghost walk's completion predates the disarm) but stays.
- cont.39q-39x TEMP taps (ALL STRIP): X-*/R-* pc taps, NSTATWR, 7ABEWR, W7968/W741C/W742C
  (fixed cap-on-logged-changes after the cap-exhaustion re-lesson), SCANMATCH/SCANDISPATCH,
  DOORTAIL/F8268/F826E/F82C0, PDW-*, OPLIST/NODE71C6/NODE71F0/RELAYFLAGS in waitdump shot 2.
- Op list for cmd $95 (live dump): $24 $28 $56 $58 $18 $54 $4a $42 $36 0000; cursor parked at
  the 0-op (DONE) — op-0 stamps node $c the moment the node ever re-enters op 8.

## cont.41 — reference confirmed, three hypotheses measured dead, the fork named (runs 158-162)

**Dave's reference-gap check: a delivering flow EXISTS** — the 6.40-era CCB/UIB kicks (TRUCKDRY):
fam=1 (host->local, 0x87 UIB fetch iopb_buf=0fe948), fam=2 (CCB fetches), D000=local word addr,
D800=0 at all kicks. C800 INGEST PROVEN (C800WR): file loaded at boot, stable at every kick:
[01]=0003 [02]=0003 [04]=0004 [08]=0004 [10]=0005. The host pointer for these transfers is NOT
in D800/C800[0] — the real host-address path remains the one open register question for the
truck (model currently HLE-peeks the mailbox).

**Soft-vector architecture (SOFTVEC, run158):** $328e = "register microsequence" = writes
per-family thunks INTO the IRQ soft vectors (table base $72ee + family: $14->$7302=IRQ5,
$18->$7306=IRQ6, $c->$72fa). One install per command @6.4039: IRQ5=$29c0 (walk/door alternator:
phase-odd->$8018 door, even->$7ba8 walk), IRQ6=$298c (phase-odd->$92b4 engine, even->$89f2
budget). Variant picked by [$796e] sign. Phase = steering bit: the hunt's clr $7950 ($88d0)
re-routes; ids reach the ENGINE ($9342 batch-end: $7426/$79b8/clr $741c) only when the walk
STOPS re-hunting = the designed batch-end relay.

**Three hypotheses measured dead this cycle:**
1. cont.40a level-at-mark did NOT kill the 33us ghost — the ghost's data completion is at the
   ADJACENT mark (id/data 33us apart), pre-disarm; the pending IRQ5 is taken at the drain's rte
   with zero mainline instructions between. The race is AUTHENTIC on real hardware too.
2. $796a=1 CONFIRMED authentic (W796A @7.96312: guard UIB[$11]=$97 bit0 + node[0]=$95 PASSED);
   $79a0 position-mode CLEARED at first content ($82d6 via the $8290 first-content fall-through)
   — AUTHENTIC. R-compare walks are the design, not a defect.
3. E804 side-switch REFUTED: ONE write all read-era (bfd8 @7.96226 = select drive 2 + motor);
   the fw NEVER switches sides. (Model defect noted anyway: stream `side` derives from fw cell
   [$7436] — an HLE peek; should be E804's side bit. Not the current blocker.)

**The exact wedge instruction:** $7e58 `cmp.w $7428,D0; bne $7d4a` — captured R (1-16) vs
target 17. Target 17 = authentic ($7e86 promotes to the first $ff/$fe cell — the terminator IS
the target; "arrive there = batch done"). In R-mode arrival-by-match is impossible; the
$79a4/$7a0c expiry drain ($8a42) is the terminator's arrival mechanism (fired once, correctly).
Post-drain the design NEEDS either the $74ac queue consumed (only consumer: task $7964 via
$32ac) or the window re-based ($9462, inside task $9398) — BOTH scanner-task-gated.

**The wall, formalized:** the $15fe scanner's dispatch gate (`cmpi.b #$a` on the status HIGH
byte) can never pass — no ROM writer stamps $0aXX, and a passing state would crash the word-
reading scheduler ($2304 jsr table[$0aXX]). SCANMATCH: zero hits, all boot, every command.
Yet multi-batch + delivery + window-rebase all live behind tasks. The fork for Dave:
(a) the tasks are dispatched by a writer the MODEL fails to produce (a hardware-originated
    status write? gate-array/DMA stamp into $71ec's high byte? — nothing in the 68000 ROM does
    it, so it would have to be BUS-side), or
(b) the floppy cmd $95 class completes by a non-task path not yet found (the $71f0 second
    node / second op list [$7224] is the least-explored corner — its cursor cells $7224/$7228
    and the $2258-$2288 dual-node wait-exit juggle).

## cont.42 — (a) sealed, (b) confirmed twice, the host road cracked (runs 163-164)

**(a) CLOSED (static):** the scanner's `cmpi.b #$a` targets ($26,[$71bc]/[$71b6]) — the SAME
pointers + offset the op scheduler word-reads ($2298/$229e). Byte $71ec IS the op word's high
byte; no separate task cell; the crash-argument stands. No bus-side stamp to invent.

**(b) POSITIVELY CONFIRMED from the live reference (N2STAT, node $71f0):** the 0x87/0x89
completions run 4->6->8->$c->0: the $c stamped TWICE - by the op-list walker ($15a4, op 0) and
by the $16c6 CHANNEL WORKER's tail ($17f8) - then the node is RELEASED (status 0, $1d0e).
Scanner never touched. The $16c6 worker (= the WKR-ENTRY taps' subject, running since 0.41)
IS the E800-channel executor: it interprets the $7442 descriptor script and stamps node $c on
channel completion. Delivery architecture, final: $3dbc builds $7442 -> worker programs the
channel -> kick -> IRQ4 -> ... -> worker stamps $c. NO TASKS ANYWHERE in a real completion.

**The truck's host road (cracked at $12f0):** $36e6 assembles the 24-bit host address from the
IOPB; then [$795e].l=addr, D0=~addr (not.l), [$79da]=~addr.lo16 (measured $3f22 = ~0fc0dd lo ✓),
[$79d8]=(~addr>>15)&$1fe = the C800 REGISTER INDEX ($1e0 measured). The $7444 descriptor embeds
[$79d8].l ($a452) - {C800 index, ~addr low} = what the worker programs at delivery. The C800
file is 0x100 WORDS ($c800-$c9ff), one register per 32KB host page (inverted addressing).

**cont.41c KEEPER:** m_c800 0x80->0x100, map $c800-$c8ff -> $c800-$c9ff. The old half-map
silently dropped every host-page write ($c9xx); the mailbox HLE-peek masked the loss. run164
green, no regression; the $c9e0 write will appear at first real delivery.

**Also measured:** the door bumps C800[00] by $40 (=128 bytes = FM sector) per sector during
collection ($8028, $2000->$23c0 over 16 sectors) - the transfer counter is maintained
throughout; collection and delivery are designed to overlap. The hunt writes C800[$1f]=0 +
C800[00]=[$741e] per pass ($88da/$899e).

**Remaining single unknown:** the $7968-drop / $742c=1 transition that stocks $3dbc's $74b4
queue (the delivery-flavor doors). New lead: task 3 ($9188 finalizer, born park=0) watches
node result cell [$71de] (node+$18) via the scanner's change-detector - same byte-gate wall;
more promising: the reference walker/worker path suggests the op-8 re-stamp after the $36-wait
comes from the worker/IRQ tails ($2244-family) once a channel op completes.

## cont.43 — the resume circuit RUNS; the bit6-ack defect; the two-node choreography (runs 166-169)

**cont.43a KEEPER — E802 bit6 = IRQ2 ack/mask level.** The model cleared IRQ2 on EVERY E802
write; the fw's real ack is the bit6 LOW pulse ($24ae andi #$ffbf / ori #$40). The hunt loop's
re-arms (bit6 high, several/ms) ate every pending IRQ2 - probes 166-168 proved the line was
never taken. Fix: CLEAR only when the written value has bit6==0. Boot green.

**The IRQ2 handler ($24aa) decoded exactly:** dedupe per channel (ch0 $7ff0 vs [$71e0]=node+$1a;
ch1 $7ff8 vs [$720a]); ch0 -> node $71c6, ch1 -> node $71f0. Event byte: bit0 (doorbell/accept
flavor) REQUIRES the target node IDLE ($251e: tst ($26,A0); bne skip) - a stale ch0=01 vs a
busy node sits unconsumed forever (measured: since the seek era). bit1-only = the OP-RING pump
($2634: ring $737c x20, count [$7376], index [$7378]; each completion pops + $27be submits the
next op). The calibrated seek release always ran via ch1 -> the IDLE node.

**PROBE RESULT (run169, the decisive one):** posting the calibrated ch1 event (13/e7/00/00 +
IRQ2, two edges - pass 1 spends the ch0 dedupe, pass 2 reaches ch1) ran THE WHOLE CIRCUIT on
firmware legs: idle node 4->6->8->$c->0, worker passes, REAL fw-issued E800 kicks (fam=1
D000=$71f0, D800=$3ed6, C800[00]=$23c0 - the per-sector counter accumulated during collection),
$2244 re-stamp, $17fe worker-tail $c. The resume mechanism is real and works. BUT it cycled the
IDLE node's leftover 3-op list; the READ node stayed parked at $000a - the busy node is never
stamped by IRQ2 (idle-gate), so the read's own resume remains unexplained.

**The delivery executor found:** $3dbc's slot loop ($3f1e/$3f9c): pop $74b4 -> slot status $40
-> arm [$7442]=1 -> bsr $3abc (THE PER-SLOT HOST-TRANSFER EXECUTOR) -> disarm -> next
($3f6e/$3f68 continuation). $414c points the worker list head at $7442. Op $56 pre-stages the
$7444 payload; the header stays 0 until $3dbc arms it. $74b4 stocking = the $81b4 door flavor =
[$742c] - the transition still ungated in our flow.

**Still open (tightly bounded now):** (1) what re-stamps the PARKED read node (op $a) to 8/$c -
not IRQ2 (idle-gate), not the scanner (dead), not measured in any probe; candidates: the op-$c
completion poster $23e6 path, ops $44/$46 ($9602/$8ac8, other classes?), or the possibility
that on real HW the park never happens this way because collection-era ch0 events keep the
op-8 walker live ([$7220]=$fe wait-exit $226e juggle). (2) $742c=1 for the read class.

## cont.44 — four trigger shapes dead, the IRQ4 pump decoded, the machine complete (runs 170-174)

**The capture-channel post (Dave's park-dissolves build): four trigger shapes, each measured dead:**
- v1 completion-dry 30ms: the post-park hunt captures every ~1.1ms - the channel NEVER goes dry.
- v2 raw bit15 falling edge + 1ms confirm: fired on the BOOT SELF-TEST twiddle (0.41) -> permanent
  kick-retry storm, blank screen (run171 - the whole boot died; >9.0-windowed greps hid the start:
  ANOTHER windowing lesson).
- v3 50ms-high-hold qualifier: the fw's ~12.5ms re-arm dips reset the rise clock - never matches.
- v4 cmd-gated (m_iopb_cmd!=0) 1ms confirm: the post-drain ghost hunt re-arms bit15 0.3ms after
  the $8a48 disarm - a SUSTAINED low never exists in this machine state. Zero posts, boot green.
CONCLUSION: the capture channel's completion event cannot be expressed as an E802 write pattern;
the correct hardware condition is still unidentified. The park-dissolves hypothesis is untested,
not refuted. (Note: even per-capture ch-events would only cycle the IDLE node - all op-4 stamps
are idle-gated - so [$71b2] pumping alone never moves the READ node.)

**$3bfe (IRQ4) fully decoded - the delivery pump:** ack E800 bit12 (via $79f6), then:
[$743a]==[$7a14] -> the $1310/$1348 submitters (IOPB-post flavors); else per descriptor A0:
A1=($14,A0).l = THE CONTINUATION -> jsr (A1); NULL -> clr ($12,A0) + result->($18,A0) + $2aba
timeout-cancel (= the measured WKR-CLR12 busy-clear). The read's pre-staged $7444 payload has
$3f68 at +$14 (DESCAREA [$7458]) = the delivery continuation ($3f6e: pop next $74b4 slot, arm,
bsr $3abc...). $16c6 head sets [$7b0c]=ffff (worker-busy); $3dbc's $3e66 gates its $4062 branch
($414c: [$743a]<-$7442 + $4208: node<-$c) on [$7b0c]==0.

**The complete read-delivery ignition, all decoded, one unlit fuse:** $74b4 stocked ($81b4 doors,
[$742c]=1) -> $3dbc slot loop -> arm $7442 -> $3abc per slot (C800[$1e0]/host-page programming)
-> kick -> IRQ4 -> $3bfe -> continuation $3f68 -> next slot -> ... -> $4208/$3e2c node<-$c.
The fuse = $742c=1 = the scan's $7968==0 side = the batch-end transition - THE single open cell,
now with every downstream gear proven. Candidate re-focus: the door's $82c0 clear needs a door
pass with [$741c]==0 - doors stopped at ~9.2 because walks stopped MATCHING (phantom-17 R-mode
hunt) - which re-roots in the $79a0 position-mode clear at first content ($82d6, authentic) and
the R-mode/$fe-target protocol. The remaining decode target: how R-mode arrival at the $fe
target is SUPPOSED to work ($7e50: cmp [$7428] vs captured R with target 17 - either the target
cell should have been re-based (window slide $9462, task-gated) or captured "R" at that stage
is position-mapped by hardware the model doesn't provide (H/side folding? the gate array's
C800[00] counter?).

## cont.45 — the record-layout defect (real, fixed, masked); the arithmetic still open (run175)

**KEEPER cont.45: the capture record layout was off by one byte.** The fw's own readers prove
the natural ID field A1A1A1 FE | C H R N: the $9884 parser reads +4 as C directly (+4..+5 for
the UIB[$12]-bit1 16-bit-cylinder class, $98ce); the verify reads [[ca]]=+4 as C ($7c52, same
fold) and [[cc]]=+5 vs [$7436] the HEAD target ($7c40) - H is captured, NOT dropped. The old
FF@+4 pad made the fw read C=$ff / H=our-C, and stage_data_record's FF shifted every DATA byte
one high (slot buffers held corrupt sectors - invisible only because delivery never ran).
Fixed both. run175: byte-identical behavior (cyl0/h0: C=H=0 mask the shift; R lands at +6
either way - the lucky alignment that let collection work). The 9.263 drain is BUDGET-driven
($79a4 at $8a3c), not virgin-driven ($8a14/$7a0c is a second, faster door) - unchanged by the fix.

**Dave's arithmetic (R-compare vs target 17) still unsatisfied.** With H now captured, the
natural fold R + spt*H = 17 needs H=1 = side 1 - which needs the E804 side switch the fw never
issues in R-mode. New thread: op $58 ($7392) ZEROES $7abe.l (measured: op $56 seeds it = the
IOPB count 8, then $7392 clears both words) -> [$7958]=0 -> the $82d0 fork clears $79a0
(position mode) at first content -> R-mode -> the $fe-target protocol jams. NEXT: read $7346
(op $58) in full - is the $7abe zeroing class-authentic or a status-driven branch we mis-feed
(op $58 reads E01E/f000 state)? If a model-fed status bit steers op $58 wrong, position-mode
survives, the walk indexes the ledger by position, hits the $fe cell, and the batch closes at
$7e1e - the whole R-mode phantom would be one more masked-signal artifact.

## cont.46 — THE WEDGE IS BROKEN: the IAM is the 17th address mark (runs 176-177)

**Dave's inference CONFIRMED EMPIRICALLY: 17 = SPT+1 = the INDEX ADDRESS MARK, the 17th
ID-class address-mark event per revolution.** Op $58's own bound ($7366-$7372: UIB[$01]+1,
the +1 gated on the UIB[$12] format bit) is the same 17 the $fe terminator target carries.
R was the ordinal's non-interleaved proxy for one pass, diverging exactly at the wrap.

**cont.46 build (STORAGER_IAM, KEEPER candidate pending count-vs-code):** an armed id-typed
capture crossing the index completes on the IAM; record = A1A1A1 FE | C H (SPT+1) N.
run176/177 results:
- IAMSTAGE fires once per revolution (every 200ms) ✓
- THE CHAIN LIT at ~9.2-9.4: walks match at the crossing -> the scan runs -> $7968 drops ->
  $742c=1 -> DOORS TAKE THE $81b4 DELIVERY FLAVOR (T-81B4 x8, one per sector, textbook state
  7968=0/742c=1) -> $74b4 stocked.
- THE READ NODE COMPLETED: NSTATWR 9.16: $c (pc $17fe = the WORKER-TAIL stamp!) -> 0 (released)
  -> 4 -> 6 -> 8 (A NEW COMMAND ACCEPTED - the CPUAP got a completion and sent the next one).
- The machine is ALIVE at 12s: batch nodes advancing (7424 = $16/$17 between dumps), $74ac
  moving, commands flowing. No "no sys floppy" declared. Park at 9.79 = a LATER command's wait.

**Not yet moving: THE DATA.** No E800 kicks post-9.2, no C800[$c9xx] writes, T-3E50/T-3ABC
(the $3dbc slot-pop/executor) silent - the $74b4 queue stocks but never drains, so the 9.16
completion carried no data (likely an error/short completion the CPUAP retries - hence the
command loop). The remaining work: why $3dbc's slot side never pops (door-tail $7a30 armed but
unconsumed at 12s), then THE TRUCK (bus copy at the kicks, host addresses from the $77f8 table
= C800-encoded [$7a1e]+n*secsize, decoded at op $58's tail).

Open question for the keeper: count-vs-code (index-anchored ordinal vs stamped code) - the
data-separator's address-mark spec decides; behavior identical for this flow.

## cont.47 — Piece 1 measured (the arm dies authentically); the truck built; completion-by-retry (runs 178-179)

**Piece 1 (the $8338/$3dbc hand-off) — Dave's consume-condition fork, measured exactly:**
[$7a30] = TWO byte flags. The door tail's $833c is a BYTE bclr on the HIGH byte; $6042/$604a
(read dispatch) arms that byte; but $70d4 (the $6f44/$7106 region, op-$4a era) word-writes
#$0001 - clobbering the arm 1.5s BEFORE the first door can consume it (measured both command
cycles: arm 6.40389/9.16660, clobber 7.96368/9.16789, first bclr 8.02509/9.22509 - always
finds it dead). Same instructions on real hardware -> the door-tail is AUTHENTICALLY not the
floppy read's delivery entry (that arm serves another class). Every decoded $3dbc entry is now
dead in this flow: door-tail (clobbered), $3ff2 (circular - the delivery continuation), task 2
(scanner byte-gate). Yet the fw shipped: a fifth entry exists, undecoded.

**The 9.16 completion decoded: COMPLETION-BY-RETRY.** Command #1's $c (worker-tail $17fe) was
a worker NULL-pass ([$743a]=$748a, continuation 0) triggered by command #2's DOORBELL op-4 -
the CPUAP timed out, re-sent, and the retry's accept flow posted #1's (empty) result. #2 then
wedges at the same park (hunts to 30s). No data moved; the CPUAP is cycling timeouts.

**Piece 2 built - THE TRUCK stands at the dock (cont.47):** at kick under NOBYPASS, armed by
the $3abc signature (any HIGH-file C800 cell nonzero): host = (~cell_index)<<16 | ~value
(the $12f0 inverted-counter decode; verified against 0fc0dd), src = D000<<1, len = secsize v1,
counter cell advanced ~(host+len). TRUCKRUN logs everything; no kicks yet to validate.

**Next thread: the [$743c] wait-descriptor list.** $29f8/$2aba (the link-frame helpers used
EVERYWHERE) build 3-word wait-records at $7b12+ and register [$743c]<-$7b12 ($2a20) - a SECOND
worker list ($3c12's [$743a]==[$7a14] fork, the $2a06 $743c-frame check). The op-$42 era
registers {#$32, [$71be], #$7986} via $29f8 ($7b28-$7b34). The worker walking $743c-chains is
the last unexplored dispatch surface - the read's delivery entry may live there.

## cont.48 — the timer system decoded and eliminated; the fifth road survives enumeration (run180)

**The deferred-poke pump ($2b58, IRQ1 tick) fully decoded and taped:** records = {delay, VALUE,
CELL, next} on the $736c chain; expiry = write VALUE to CELL. Complete traffic across the read:
- {$32=1.3s, $18, node-result} per accept - THE WATCHDOG (result $18 = timeout; the worker
  checks it at $1762). Canceled in time in every cycle (never fired).
- {$3c, 1, $7a36} - seek settle (fired 7.96208 = the op-$28 release ✓).
- {$46=70 ticks=1.82s, 1, $7a3e} - motor spin-up (fired 9.79017 = the park's timing source ✓).
- The $743c/$7b12 special record: NEVER registered in this flow (no $743c-key registrations).
**Timers are NOT the fifth road.**

**Elimination table (all dispatch surfaces traced, all dead for the read's delivery):**
scanner (byte-gate, proven), door-tail $8344 ($7a30 high-byte arm authentically clobbered by
$70d4 1.5s pre-door), $3ff2 (circular), task-2 wake (scanner), IRQ2 stamps (idle-gate),
IRQ4 continuation (needs a kick first), timers (above). $7b10/$7106 = one-shot consumed 8.0-era.
The fw ships, so the road exists in the ~unread regions: $4bxx-$5bxx (write-side/op $30-$34
handlers, heavy $29f8 users at $4c10-$4f30) and $9600-$a2xx.

**Standing state:** the IAM keeper holds (chain lights, $74b4 stocks 8 slots/command, commands
cycle by CPUAP timeout-retry); the truck built and idle; the drain/$3abc never entered.

## cont.49 — op-$32 read; THE DELIVERY DISPATCHER FOUND at $58xx (2026-07-15)

**Op $32 ($4ec2) = the idle node's drive-attention COROUTINE** (resumable via [$7b38]; polls
$F000 bits 7/13; handlers $5156/$4cc6 with continuation re-entry; unit mask from [$792a]
changes; $798a-timers). Not itself the road - but its region led to it.

**THE DELIVERY DISPATCHER ($58xx, inside the bounded set):**
- $58a2: works when [$77c2]!=0 (FLOPPY work) OR (HD, $791a!=0) $74b4/$74ac non-empty
- $58c8: programs the FAMILY-8 transfer channel ([$7a18]=8, $63e byte, E802 ori $2010 family)
- $5906: pops $74b4 (or $74ac when [$790e]!=0) -> [$7926]=node, unlink (THE HD POP)
- $5970: movea [$77c2] -> THE FLOPPY POP
**[$77c2]/[$77c4] = the floppy delivery queue head/tail** ($5b02 append region; [$7930]<-$77c2
registered at $4b38). Dave's Q1 (queue-consume): YES. Q2 ($3abc-reach): the dispatcher may kick
the channel DIRECTLY (family-8 program at $58d4-$58fa) - $3abc-vs-direct still to trace.

NEXT: (a) the $58xx dispatcher's ENTRY (who calls it - op $30/$34's body? the $4cc6 coroutine?),
(b) $77c2's STOCKER (who moves the read's slots onto the floppy queue - the $5b02 region),
(c) $5970+ (the floppy pop's kick path). The road is one stocker + one entry from lit.

## cont.49b — world (iii); the op list GREW; op $1a = the transfer-side C800 script (run 181-182)

**Dave's discriminator, verdict: world (iii).** $77c2 is NEVER stocked (boot noise only) AND the
$58xx dispatcher NEVER RUNS - that whole surface belongs to yet another class (op $30/$34 era).
Neither a floppy stocker to find nor mis-routed doors: the read's road is elsewhere.

**The submit-pair tape ($743a/$7a14):** accept-era transfers ride {$c8e: $7a14<-$748a} + $27be
(ring submit) + $1344 (kick-clear) per command; $3abc RAN at 6.4005 ($3b92 = its own submit,
caller pre-$3dbc) - the executor is general-purpose. Callers: $3dac/$3f36/$3fb2 ($3dbc-family),
$44a2, $496e, $8fea (INSIDE op $3e = $8f74!), $9e86 (beside the $743c lea). Executor ops exist
in other classes' lists.

**THE OP LIST CHANGED IN THE NEW REGIME (post-IAM + record-layout keepers):**
old: $24 $28 $56 $58 $18 $54 $4a $42 $36 0
new: $24 $28 $56 $58 **$1a** $18 $54 $4a $42 $36 0
Op $1a = $3182 = the SECOND C800 script loader: walks [$793a] (op $18/$308c walks [$7938]),
writes C800 cells + reads the $4000 table + pours E000 $23f - THE TRANSFER-SIDE channel
program. The cont.45 layout fix un-masked a class-byte gate in the $5fc0 list builder.
NEXT: tape op $1a's C800 writes (pc $31cc) - do the HIGH-file cells ($c9xx = the truck's arm)
light now; then the family-8 channel kick that consumes the [$793a] script.

## cont.50 — the builder decoded; host-shipped-script DISPROVED by experiment (runs 183-184)

**The $5fc0 list builder ($5fc0-$6060), decoded:**
- emits $24 $28 $56 $58; then [$793e]: ==1 -> NEITHER loader op; ==0 -> $18 only; >=2 -> $1a
  THEN $18 (the order is AUTHENTIC); then $54 $4a $42 $36 0.
- [$7938] <- UIB+$24 (A0=[$799a]): THE SCRIPT LIVES IN FW WORKSPACE AT UIB+$24, fw-built.
- $6042: the bset (the door-tail arm we traced); $604c: [$7456].l=$3f68 - THE BUILDER installs
  the $7444-descriptor's continuation; $6054: [$7450]=2.

**Host-shipped-script hypothesis: DISPROVED by the 0x60-cap experiment** - host bytes [24..5f]
are a pointer table (0fe968+4n, LE longs), and copying them clobbers the fw's script workspace
(flaky boots, op-18's pour shrank 16->1). Cap reverted to 0x22 (CORRECT; baseline verified
run184). run135's clobber now fully explained: the raw $100 copy overwrote the script workspace
+ beyond.

**Frontier (two named cells):** (a) who fills the part-2 script - [$793a] holds op-18's STOP
position ($3090: op-18 deposits it); with the $1a-before-$18 order, op-$1a's script = the
PREVIOUS command's part-2 leftovers (cross-command pipelining?) or a separate fill not yet run;
(b) [$793e] semantics (>=2 admits $1a; what sets it - satisfied in the new regime).

## cont.51 — pipelining half-confirmed, then re-read: op $1a is a RE-LOAD op (runs 185-186)

**The full-chain tape (Dave's both-halves discipline):** #1's op-$18 DOES write [$793a]
($3090, 7.963) and #2's op-$1a DOES read it (count=$10, 9.167) - but the deposit is made AT
ENTRY: [$793a] = the script START (=[$7938]=UIB+$24), not op-18's stop position. Op-$1a's pour
(isolated past the C800WR cap - the cap swallowed it in run185, the windowing toll AGAIN) is
byte-identical to op-18's part-1 disk-side program ([00]=19/26/28, [03], [04], [3f]x8; the
only difference: op-$1a echoes through the $4000 table, op-18 through $6000). **Op $1a = re-pour
the channel program (a re-load op for the retry/second-engagement), NOT a transfer-side script.
The two-part-script reading is dead.** The high C800 file stays dark; the truck stays idle.

**[$793e] provenance (the op-$1a admission gate):** dispatcher $e44/$ebe set $ffff (>=2 ->
builder emits $1a); op-18's $3094 sets [$793e]<-[$793c] (=1 -> next build emits NEITHER);
$313e/$3178 (op-$1a-adjacent) set $ffff; $2752 clears. A cross-command handshake - the old
regime's lists lacked $1a because [$793e] was 1 at build time; the new regime's $ffff at
dispatch admits it. Lifecycle decoded enough for the record.

**Standing:** the delivery road remains the $3abc executor's un-found invocation. Eliminated
this arc: every dispatch surface + op $1a-as-transfer. The $79d8/$79da host-address pair and
the $7444 descriptor (continuation $3f68 installed by the BUILDER at $604c) still sit staged
and unconsumed - the machine builds the delivery's ingredients at every command and never
cooks. What consumes $7444/[$79d8] is THE question, now with everything else dead.

## cont.52 — the reader-tap verdict; the 17-match LIVES; the grind loop (runs 187-188)

**The inversion's verdict: NOBODY reads the staged cells.** After filtering self-noise (the
pc=$1606 hits at 9.79 = the waitdump's own C++ reads firing on the scanner-fetch tap - flag
for the record), the command-era reader census: $79d8 read ONLY by its own builder ($a456 =
op-$56's descriptor build, both commands); $7444/$743c read by NOTHING. The consumer provably
never runs - read-side proof matching 180 runs of writer-side silence.

**The gift on the same tape: THE 17-MATCH HAPPENS NOW.** Post-park, the deep walk sees the IAM
ordinal: r=$10, r=$11 vs [$7428]=$0011 - MATCH at 9.80124 -> the scan runs ($7e8a) -> $32ac
DEQUEUES a $74ac batch node -> re-batch -> target promotes to $0012 (18) = unreachable
(ordinal max 17) -> hunt until the next 17-crossing -> dequeue another node. THE GRIND LOOP:
one queued node consumed per crossing. The close fork ($7ed8: $741c=0, no re-arm -> the clean
resume) fires only when $32ac finds the queue EMPTY - convergence = does the grind drain
faster than drains restock. run188 (120s): instruments all capped out by ~10s (the aging-taps
gap - next instrument must be BUCKETED COUNTERS, not capped logs); no "no sys" banner in 120s.

**Next: (1) bucketed grind counters (17-matches, dequeues, queue depth per 10s) - does $74ac
empty; (2) if it converges, the $7ed8 close should light $742c cleanly on an EMPTY queue and
the delivery flavor doors + $3dbc drain may finally sequence in the designed order.**

## cont.53 — the rates: HALT, not convergence; hollow drain confirmed (run189, 120s)

GRIND buckets: t=10: match17=9 dequeue=24 restock=7 kick=9; t=20: restock=1 only;
t=30..110: ALL ZERO. Queue frozen at 7584/757c, batch node $17, forever.

**Reading:** (1) Dave's hollow-drain discriminator CONFIRMED in his exact terms - dequeues
climb (24) while delivery kicks stay zero (the 9 kicks = accept-era CCB fetches, 2-3 per
command x ~3 commands); dequeued nodes RECIRCULATE as re-batches whose sectors are already
consumed - the drain never finishes because nothing carries the bytes. (2) The fourth outcome:
the CPUAP STOPS COMMANDING after ~2-3 retries (~13s) - no error banner in 120s, just silence.
The grind cannot converge because it only advances ~4-5 crossings per command era and the
commander quits first.

**The hypothesis this forces (for Dave):** nothing in the 68000 ever reads the staged
descriptor ($7444/$79d8 - reader-tap proven cold) because THE CONSUMER IS THE GATE ARRAY:
the $7442+ area is a hardware channel-program block (V/SMD 3200 descriptor-chain heritage);
the fw stages the program, the header arm ($7442=1) is the hardware GO, and the gate array
itself fetches {type, C800-idx, ~addr-low, D000-val, ...} and runs the bus transfer. The model
must BE that consumer - the truck's real form: not a kick-time copy but a descriptor-block
processor watching the $7442 arm. The remaining fw-side gap: the arm sites ($3f26/$3fa2)
still never run - unless the ARM itself is also simpler than decoded (op-56's staging IS the
program and the GO is elsewhere - e.g. the E802 family-8 bits or [$7450]=2 from the builder).

## cont.54 — the descriptor engine LIVES; v1 self-diagnosed its own timing (run192)

DESCGO fired (after a run191 placement bug - the engine was nested dead under a==0xe800):
hdr=0000 e802=aad7 idx=01e0 ~lo=3f22 -> **host=0fc0dd** srcD000=5ffe cnt=0002 @7.96385.
**The address math VALIDATED ON LIVE DATA** - the staged descriptor reconstructs the host
buffer exactly. But v1 delivered 1024 ZERO bytes: the GO fired at staging time (7.96, before
any collection - the hunt's $aad7 writes carry $2010 constantly) and all three candidate
sources are virgin at that instant.

**v2's shape, named by the machine's own counter:** the door's C800[00] += $40-per-captured-
sector (measured back in run163: $2000 -> $23c0) is the fw TRACKING the hardware engine's
concurrent per-sector progress. C800[00]=$2000 = local word addr = **$4000 - the bounce buffer**
(the $4000 table op-$1a echoes through!). The real engine runs CONCURRENTLY with collection,
one sector behind the captures: disk-side DMA deposits at [C800-counter], host-side engine
carries from there to host+progress. v2 = per-sector transfer at each capture/counter bump,
source (C800[00]<<1), dest descriptor-host + progress; completion event when the count
exhausts (then the fw's DONE post path - the remaining unbuilt half).

## cont.56 — FIRST REAL CARGO: VOL1 CROSSED THE BUS (run197, 2026-07-15)

**The pipeline works end to end: disk -> bounce -> host, with real bytes.**
DEPOSIT-C800 4200 first4=56 4f 4c 31 ("VOL1") / 4280 = 48 44 52 31 ("HDR1");
CARRY sec@4200 -> host+512 VOL1, sec@4280 -> host+640 HDR1 - the ANSI floppy label landed
in CPUAP memory at 0fc0dd+. The truck delivered its first real cargo.

The build chain that got here: v1 bulk-at-GO (kilobyte of zeros, self-diagnosed timing);
v2 counter-bump concurrent carry (the door's C800[00] +$40/sector = the shared pipeline clock;
Dave's off-by-one + DONE-ordering disciplines built in); v3 deposit-at-counter in
stage_data_record (never ran - no data-typed arms in this regime); v4 deposit-at-D800 (D800
tracks the record buffer, not the bounce); v4b deposit-at-C800[00] at every data mark = THE
HARDWARE SHAPE: C800[00] is the disk-side DMA's own address counter, fw-aimed per sector.

**Three named wrinkles (next work):**
1. ORDERING: VOL1 (physical R7 = logical 0, sec0=7) landed at host+512, not host+0 - the
   deposits/carries ran rotational-order while the CPUAP wants logical order. The fw's
   aim values vs our carry sequencing need reconciling (the carry should follow the AIM, i.e.
   host offset = (aim - base), not arrival order - the fw's re-aims ARE the logical map).
2. The $4300 duplicates (counter parked while awaiting the next want): benign re-deposits,
   but the carry fired on arrival not on aim-advance for those.
3. The DRAIN's stale $a0 tail (host+896): Dave's predicted off-by-one hole - the final stage
   drained a never-deposited position.
All three are one fix in shape: host offset must derive from the AIM (C800[00] - $2000), and
the carry/drain must move aimed positions, not arrival slots.

## cont.57 — carry-by-aim clean; the aim is ARRIVAL, not logical (run198)

Mechanics all green: 8 CARRY-AIM slots, zero duplicates (the $4300 parked-counter re-deposits
self-skip), no drain hole (each sector carries itself at deposit), DESCDONE after slot 7.
BUT: VOL1 still at host+512. The fw aims the counter SEQUENTIALLY PER ARRIVAL ($2000, $2040,
...) for EVERY capture including unwanted pre-window sectors (R3-R6 arrive before R7=VOL1 and
get aims 0-3). The aim = the collection slot, not the logical position. The logical map lives
in the fw's ledger/window bookkeeping (position p <-> R = ((sec0-1+p-1)%spt)+1: position 1 =
R7 = VOL1 - the POSITIONS are logical; the ledger binds positions to slots).

TWO open semantics (Dave's domain):
(1) Does the CPUAP's NON-ADDRESSED read (IOPB addr=0, opts=0) even require VOL1-first, or is
    current-position/arrival order correct for it (the HLE's sec0 rule may belong to ADDRESSED
    reads only)? If arrival order is right, run198's cargo is ALREADY correctly shelved.
(2) The DONE race is now structural: slot 7's aim only advanced in command #2's era (the fw
    stopped aiming after 7 in cmd #1) -> our DONE at 9.2375 always trails the 9.16 completion-
    by-retry. Either the total is wrong (cnt=2 blocks=1024 bytes=8 sectors but the fw's cmd-#1
    window only aims 7?) or DONE-per-descriptor vs DONE-per-command needs the fw's own
    accounting ([$79a8]) rather than the descriptor cnt.

## cont.60 — THE VERDICT: the sys-floppy check ran to a DECISION (run201, 2026-07-15)

**Screen @8.2: "no sys-floppy, going to harddisk" — the CPUAP read the delivered buffer and
JUDGED it.** The full cycle, first time in 201 runs: doorbell 6.40 -> collection 8.03-8.17
(carry-by-R: VOL1 at host+0, all eight logical) -> DESCDONE 8.175 (hold released + DONE
posted) -> fw completes the node at 8.176 ($c -> 0 -> next accepts at 8.18-8.19, THREE
commands in 20ms - the pipeline FLOWS) -> the CPUAP checks and moves on at 8.2. The boot
PROGRESSES past the floppy check instead of hanging - the machine's behavior is now
decision-shaped, not wedge-shaped.

**cont.60 KEEPER: DESCDONE = the data-delivered event.** STEP-324's m_read_pending hold (which
suppressed EVERY host-visible completion under NOBYPASS - only the retired HLE legs cleared
it) releases at DESCDONE; the completion status posts (the descriptor tail's own wiring).

**The verdict itself is the next question:** "no sys-floppy" with VOL1 correctly at +0 means
either (a) a cargo-fidelity defect (our 128-byte payload vs the IMD's true R7 content -
deposit boundary/CRC-trim off-by-something), or (b) the check's criteria live deeper (HDR1
boot-file fields), or (c) a correct negative for this disk under fw criteria (though the HLE
era booted from this same image to "Boot: sa(22,0)sinix" - so (a)/(b) favored). NEXT: byte-diff
host 0fc0dd..+3ff against the IMD's true sectors R7-R14. The truck's cargo-fidelity test - one
diff names the defect or clears the cargo.

## cont.61 — CARGO CERTIFIED BYTE-EXACT; the verdict is semantic (run202)

**The diff: ZERO. All 1024 delivered bytes match the IMD's decoded R7-R14 exactly.** The
truck's cargo-fidelity certification passes - the LLE mandate's truest test: the bytes the
host received are the disk's own. Dave's fork resolves to (b): "no sys-floppy" is a judgment
on CORRECT data - the check's criteria live deeper than the label bytes.

**The named suspect: THE QUOTA.** The HLE (which booted this same image to "Boot:") delivered
count(8) x 512 = 4096 bytes = 32 media sectors; our descriptor cnt=2 x 512 = 1024 = 8. Dave's
"the descriptor's count is a channel-program unit, not the command's quota" - exactly. The
CPUAP asked 8 LOGICAL BLOCKS (4KB); the check likely reads boot-loader fields beyond +1024
(stale garbage there -> "no sys"). The fw-side accounting ([$7abc]=8, [$7966] units, op-58's
split logic sized for a 17-bound window) needs re-reading in 512-block units - a 32-sector
read spans TWO track-sides (the E804 side-switch that "never came" may be exactly what a
4KB read triggers).

**NEXT: read the judge.** Find "no sys-floppy" in the CPUAP monitor ROM; decode the check -
which offsets/fields it reads, how many bytes it expects delivered. The consumer's own code
is the contract; measure it before resizing the quota.

## cont.62 — the judge's contract QUOTED; the verdict is STATUS-driven (2026-07-15)

**The monitor's checks, off its own code (d53+d54 interleaved, 32KB at FE0000):** the label
checks address a struct with the buffer base at +0x65 - 'S'@+0x69, 'I'@+0x6A, 'X'@+0x6D all
resolve to **"SINIX" at buffer+4 = the VOL1 volume-ID field**. Our delivered R7
("VOL1SINIX0...") SATISFIES the content contract. (The old PCMX2-CPUAP-DISASM.md notes'
fe3686 HD-check offsets now fully explained by the same struct layout.)

**The real reason for "no sys-floppy": the COMPLETION STATUS.** run200's node dump: result
field (+$18) = $2029 - the expiry-drain's error ($8aae branch: [$7a0c] still nonzero at drain
= "batch incomplete"). The CPUAP checks the read's STATUS before/with the label; an errored
read = no sys-floppy regardless of buffer content. THE FW DOESN'T BELIEVE ITS OWN READ
SUCCEEDED - its internal transfer accounting (the task-side $79b6/$9328 chain, dead for this
class) never ran, so its own receipt says failed even as the physical delivery lands perfect.

**NEXT: the fw's clean-success accounting.** [$7a0c] (the valid-id countdown, $8a32) history:
what seeds it, why it doesn't reach zero before the expiry, and what the CLEAN drain branch
([$7a0c]==0 -> no $2029) requires. The goods pass inspection; the controller's own paperwork
is the last thing between here and the label check saying yes.

## cont.63-64 — the paperwork CLEARED; DONE reaches the CPUAP; the verdict is the QUOTA after all (runs 203-204)

**[$7a0c]'s three numbers (run203): a FOURTH world.** Seed = 3 ($6ab8, 7.962) - a sync-
confirmation counter, NOT the quota. Decrements 3->2->1->0 at 8.19614-635 (pc $8a38), the
drain firing ON THE COUNTED CLEAN PATH (X-8a5a at the zero, 8.19635), then re-seed 3. Command
#1's own bookkeeping completes CLEAN in the full-pipeline regime - the $2029 seen at 12s
belongs to a LATER command's cycle. The fw believes this read now.

**IOPBDUMP (run204, t=8.3): DONE REACHED THE CPUAP.** The current IOPB at 8.3 = cmd 95,
unit 0 (THE HARD DISK), count 2 blocks, same buffer - "going to harddisk" LITERALLY: the CPUAP
processed #1's completion, judged, and MOVED ON to the HD probe. Status 81 81 = the HD
command's fresh BUSY (the model's accept re-post), not #1's. buf@0fc0dd still holds
"....SINIX0" (VOL1's first 4 bytes overwritten post-verdict; label was present at check time).

**The verdict's remaining explanation = THE QUOTA (Dave's number, one level up):** with status
DONE and the label present, the check still said no - the criteria read DEEPER IN THE BUFFER
than our 1024 bytes (the +0x69-family offsets against a 4KB buffer, and/or the NSC-Boot load-
map fields from the sectors we short-delivered). The CPUAP asked 8 x 512 = 4096 = 32 sectors
= TWO SIDES; we deliver 1024. NEXT (the one remaining delivery-side fix): size the engine to
the IOPB count (cnt from the IOPB, not the descriptor's channel-unit 2), let the read cross
to side 1 (the E804 switch arrives as a consequence), deliver 32 sectors, re-run the verdict.

## cont.65 — THE JUDGE'S CONTRACT ON REAL DISASSEMBLY (run206, MAME debugger dasm)

**The fe44xx sys-check, quoted (NS32016):**
  FE4484 ADDR @8,(SP); MOVD R6,TOS; CXP 0x2B      ; READ 8 units (the disk-read proc)
  FE448E CMPB 'S', 0x69(R6); BEQ +                ; label test path A (struct field)
  FE4495 CMPB 'S', 0x4(-0x94(FP)); BNE fail-path  ; path B: 'S' AT BUFFER+4 (our SINIX0 ✓)
  FE449D (matched): 0x65(R6) += 0x400; CXP 0x2B   ; ADVANCE +1KB, READ THE NEXT 8 UNITS
  FE44B9 CMPD 0x10B, (R4); BEQ loader             ; THE MAGIC: a.out ZMAGIC 0413 = 0x10B
  FE44CA (else) print "no sys-floppy"             ; at the retry counter's exhaustion

**THE QUOTA STORY, FINAL FORM: 1KB units, CHAINED.** The CPUAP reads count=8 x 128 = 1KB per
command, advancing the source address +0x400 each read (0x65(R6) += 0x400). The fw's 1KB
window per command was RIGHT all along; the HLE's x512 over-delivery masked the real protocol.
The check: read1 = label (S at +4 PASSED - run204's buffer proves the CPUAP matched and
proceeded); read2 = the NSC-Boot a.out header, expecting magic 0x10B (ZMAGIC 0413).

**The actual failure: READ2.** Commands at 8.18136 and 8.19519 = the CPUAP CHAINING reads
after the label matched. Verdict <=8.2 = read2 completed within ~20ms = served from the fw's
READ-AHEAD CACHE (positions 9-16, collected during cmd#1) with NO NEW CAPTURES - and our
engine only carries on fresh captures -> read2 delivered NOTHING -> (R4) stale != 0x10B ->
"no sys-floppy". THE LAST GAP: the cache-served read's host transfer (the fw's own
serve-from-buffer path - its aims/descriptor for a no-capture command).
NEXT: trace cmd#2 (8.18) - its IOPB addr field, the fw's aims/descriptor for it, what the
engine must key on when the data is already in the bounce.

## cont.66 — read2 on tape: the chain is 8/4/2, non-addressed CONTINUE; the monitor pre-clears the magic slot (run207)

**Cmd#2's IOPB (8.185):** 95 / unit byte 02 (EXPLICIT floppy) / count 00 04 (FOUR blocks) /
addr 00000000 (NON-ADDRESSED - the chain CONTINUES from current position; the +0x400 advance
is the monitor's own source pointer, not the IOPB) / buffer SAME 0fc0dd. Status 80 80 = #1's
DONE visible (our completion REACHED the host ✓). Buffer at 8.185: VOL1SINIX0 INTACT.
**Cmd#3 at 8.195: count 00 02** - the chain halves 8/4/2 (the FE44C2 retry loop). Buffer at
8.196: bytes 0-3 ZEROED with SINIX0 intact behind - **the MONITOR PRE-CLEARS (R4).D = buffer+0
(the ZMAGIC slot) before each chained read** - fail-safe against stale matches; read2 must
land 0x10B there and landed nothing.

**Read2's lifecycle: accept 8.178 -> op-56 restage ([$7abe]<-4 ✓, [$79d8] recomputed same
host) -> [$7a0c] full clean cycle 3->0 + drain at 8.196 -> completes in ~18ms** (1.4 sectors
of rotation - no real recollection; the fw satisfied its bookkeeping from the already-
collected state). NO deposits (except the parked $4300 dribble), NO carries: the engine's
desc_done latch is a plain bug (one-shot for the RUN - can never serve read2), AND the cache
sectors aren't in the bounce anyway (positions 9-16's deposits all landed at the PARKED $4300
aim, mutually overwritten - the bounce holds one of them).

**OPEN (the next tape):** read2's RESULT byte - clean-but-undelivered vs errored (the $2029
seen at 12s in run200 is a strong candidate for READ2's result, not #1's). And the fw's
actual serve source for a non-addressed continue over consumed positions (fresh re-collect vs
ledger-cache) - the answer defines the engine's cache-carry source. Engine fixes queued:
per-staging re-arm (kill the one-shot), carry-on-staging per Dave's generalization, source =
whatever the read2 trace names.

## cont.67 — re-arm lands (early); read2 confirmed no-disk; the blind-0x80 translation exposed (run208)

- Per-staging re-arm works (DESCARM #2 @8.17502) but arms EARLY: it re-arms on cmd#1's still-
  staged descriptor 2ms before cmd#1's host post, then s_desc.active pins across cmd#2's
  restage (8.1785). REFINEMENT QUEUED: invalidate/re-arm on the op-56 restage ($79d8 write)
  when done==0.
- READ2 IS NOT A DISK OPERATION: no new wants, no aims, no deposits after 8.178 - the fw
  completes it in ~18ms internally. Errored-vs-cache still hinges on THE RESULT BYTE.
- NEW HLE-ISM EXPOSED: the model's $7fe8 tap translates EVERY fw post to 0x80/0x80 - the fw's
  real status values ($00ba for read1's completion, $0101 for accepts) never reach the host
  faithfully. The CPUAP's CXP-2B return tests these. RETIREMENT QUEUED: map the fw's posted
  value to the host status faithfully; the read1 $ba and read2's (unseen) post are the codes
  the monitor actually branches on.
NEXT TAPE: read2's result (node result cell writes 8.17-8.20 + the raw $7fe8 value for its
completion) + [$7954] per staging. Then: errored -> stream-state fix (let the continue
re-collect); clean-internal -> name the serve source and carry from it.

## cont.68 — read2 is CLEAN-INTERNAL; the fw already sent the second kilobyte (run209)

**The arbiter: $98E7 = the NORMAL completion word** - the healthy 0x87/0x89 reference cycles
post the identical hi=$98/lo=$e7 composite throughout 6.40. Read1 AND read2 complete clean in
the fw's own vocabulary. (The blind-0x80 translation is thus FAITHFUL for successes - it only
lies for errors; retirement deprioritized until an error-path flow needs it.)

**Dave's fork resolves: clean-but-internal.** And the serve source was on the tape since
run163: the fw's cmd#1 aims walked $2000->$23c0 = SIXTEEN sectors = 2KB - the fw's transfer
intent covered positions 1-16 (wanted + read-ahead), pushing BOTH kilobytes in cmd#1. Our
engine truncated at the descriptor's cnt*512 = 1KB (slots 8-15 carried nothing; their
deposits also parked at $4300 as the fw stopped re-aiming once ITS transfer view was
satisfied... to re-measure under the span fix). Read2 = the second kilobyte, which the fw
believes is already at the host - hence the 18ms clean internal completion, no recollect.

**THE SPAN FIX (next build):** (1) cmd#1: honor the full aim span - carry all aimed slots
while the staging lives (up to 16/2KB), DONE still posted at the command's ask (count x 128);
(2) read2's restage (the queued restage-keyed re-arm): re-carry the second kilobyte to the
new staging's base (the monitor pre-clears +0 and expects KB2 there). Open question to watch:
whether the fw's re-aims resume for slots 8-15 once the engine keeps carrying (the $4300
parking may have been DOWNSTREAM of our truncation - the fw stops aiming when its counter
view stalls).

## cont.69 — span + re-carry land mechanically; the ZMAGIC ISN'T ON TRACK 0 (run210)

Span works: 14 slots carried (incl. pre-window R3-R6 to scratch offsets; R1/R2 missed - their
marks pass before the first door aims, content zeros anyway). Slots 8/9 (R15/R16) carried to
host+1024/+1152 as they arrived. Re-carry works: RESTAGE-INVAL -> RECARRY slot8 -> host+0.
BUT the cargo is honest zeros: **positions 9-16 of track 0 ARE zeros on the disk** - and the
$4300-parked deposits only held one survivor anyway. The verdict is unchanged because the
monitor's ZMAGIC target was never track 0's second half.

**THE REFRAME (up one protocol layer):** FE449D parses HDR1 (file "NSC Boot", field "00256")
and sets the NEXT read's unit/address (0x5D(R6) <- label field = THE UNIT - and cmd2's IOPB
unit byte measured 0 -> 2 on tape ✓; 0x61(R6) <- R4 = the file-start). The load-map lives at
CYL 1 (PCMX2-FLOPPY-ID). So read2 is supposed to fetch the FILE START, positioned from HDR1 -
not the label track's tail. OPEN: cmd2's IOPB semantics (unit=2/count=4/addr=0 - what does
the fw do with them; cmd1's unit byte 0 read the floppy fine, so the byte-4 unit decode is
suspect), and how the fw positions for the file (seek to cyl1? the 00256 units?).
NEXT: decode the fw's cmd-95 handling of the unit/addr fields (the $5fc0 head + $a3xx op-56
addressed-path) + verify where the ZMAGIC actually sits on the disk (dump cyl1 from the IMD).

## cont.70 — THE ZMAGIC IS AT CYL 1; the protocol is label -> re-INIT -> file-read

**ZMAGIC 0x10B found on the IMD: cyl 1, head 0, R1, LITTLE-ENDIAN** (the NS32016's byte
order) - the NSC-Boot load-map, per PCMX2-FLOPPY-ID's cyl-1 claim. Track 0's second half is
honestly zeros; read2 was never meant to read it.

**The boot protocol, final shape:** 0x95 (label, count 8 FM sectors) -> monitor parses HDR1
(file "NSC Boot", start field "00256", unit/density) -> **0x87 RE-INIT** (UIB = file-region
geometry + base: MFM 256B at cyl 1 - "geometry is label-driven", the old notes' own words;
the HLE-era comment knew it: "the ANSI flow re-INITs at the file start") -> 0x95 (file,
count 4 x 256 = 1KB MFM, non-addressed from the NEW base). The tape matches: two accepts
post-read1 (8.181, 8.195) with 0x87-flavored posts between; cmd#2's count=4 = MFM-256 units.

**The gap:** the post-INIT read completed in 18ms with NO SEEK - the fw never stepped to
cyl 1. Suspects: (a) the 8.18-era 0x87's UIB copy under NOBYPASS (geometry/base fields for
the MFM region - the uibcnt/dst handling for a SECOND INIT), (b) the fw's position/seek
computation for the re-INIT'd read (op-28's target from the new UIB), (c) our seek/settle
model shortcuts. NEXT TAPE: the second 0x87's UIB content + op-28/E804/step activity at
8.18-8.20. When the seek runs, cyl 1's MFM stream builds, the ZMAGIC collects, and read2
delivers 0x10B to host+0.

## cont.71 — the second INIT on tape: no seek attempted; the cyl-1 address is in the IOPB (run210 re-read)

- **DOUBLE re-INIT at 8.177** (two 0x87s, 8.17699/8.17748), src = THE SAME host UIB 0fe948 as
  the first INIT. UIB[0] = $0210 (2 heads, 16 spt) - geometry word unchanged. (Whether the
  monitor REWROTE 0fe948's other fields between 6.40 and 8.177 = still to diff.)
- **NO STEP ATTEMPT** in the window: E804 select churn only (ffd8@$1adc deselect, ffc8/bfc8
  re-select at the op-$24 pcs; low byte $d8->$c8, bit4 dropped). Suspect (c) RULED OUT - the
  fw never wanted to seek. (a)/(b) live: its UIB/base said stay-put.
- **The cyl-1 address is plausibly IN cmd#2's IOPB**: bytes 4-5 = "02 00" - as a block address
  multiple natural decodes land on byte 4096 = CYL 1 H0 exactly (2x2KB track-side units;
  or HDR1's "00256" x 16). The model's byte4-as-unit parse (&3 -> "unit 2" = floppy,
  coincidentally right!) is an HLE-era guess now suspect for reads - cmd#1's byte4 was 00
  ("unit 0" that read the floppy anyway). BYTES 4-5 MAY BE THE FILE-START BLOCK ADDRESS.
- NEXT TAPE: (1) diff host 0fe948 image at 8.177 vs 6.40 (did the monitor rewrite the UIB);
  (2) the fw's own parse of cmd#2's bytes 4-7 (where does the fw put them - the seek-target
  cells [$7436]/[$7438]/op-28's goal); (3) then feed the seek whatever it's starving for.

## cont.72 — no address anywhere: the protocol is POSITION-CONSUMPTION; the side-switch returns (run211)

- UIB image BYTE-IDENTICAL across all three INITs (monitor rewrote nothing).
- The ROM has exactly TWO node-byte-4 readers: $dee (dispatcher unit-select) and $1a90 (the
  E804 drive-select at $1adc). Bytes 4-5 are unit-class after all; no IOPB address exists.
- **THE SYNTHESIS: the monitor's chain is POSITION-CONSUMING.** The HLE booted because its
  count units (512-byte blocks) made read1 consume count(8) x 512 = 4KB = 32 sectors = BOTH
  SIDES of cyl 0, leaving the head AT CYL 1 - so read2's non-addressed fetch read the ZMAGIC
  at the head's position. Dave's 4096 was right at the CONSUMPTION level. The LLE fw's read1
  consumed only 16 sectors (one side, 2KB) - HALF - because the collection never crossed to
  side 1. **The $fe-terminator at position 17 = "cross to side 1 and continue"** - the walk's
  17-arrival should trigger the E804 side-switch + window re-base and collect positions 17-32;
  ours dequeues-and-regrinds instead. Dave's flagged side-switch risk, returned as the gap.
- NEXT: the position-17 arrival's INTENDED full path - the $7e1e/$7e50-match continuations'
  deeper branches (side-switch + re-base + continue) vs our dequeue-regrind; what the switch
  needs that the model must provide (E804 side bit -> stream side, the build already keys
  (cyl<<1)|side). The last mile is one head-switch wide, again - this time with the protocol
  demanding it.

## cont.73 — sequential consumption CONFIRMED from the monitor's own math; the fw's abbreviated list is the 18ms (run211 + statics)

- $dee: IOPB byte 4 = THE UNIT (validated < 8, indexes [$20a+u*2] UIB ptrs) - for ALL commands.
  cmd#2's 02 = unit 2 explicit (from the label parse: FE449D's MOVXBD label-struct field ->
  0x5D(R6)). The bytes-4-5-as-address numerology is dead twice over.
- The monitor's 0x61(R6)/0x65(R6) = BUFFER + OFFSET (their sum prints as "max addr= %x",
  FE4468-77) - the chain accumulates reads into memory at advancing offsets; NO disk address
  exists anywhere. Sequential consumption is the whole protocol, confirmed from the monitor's
  own arithmetic.
- The 8/4/2 chain = the FE44C2 RETRY-PROBE loop: FE44D4 indexes table FE2C30 by retry number -
  the monitor RE-READS with different counts/strides hunting the ZMAGIC. On real hw the probes
  CONSUME disk each pass, eventually crossing to cyl 1 where the header lands. Our reads 2/3
  consumed NOTHING (no collection).
- **Why no collection: the ABBREVIATED OP LIST.** The $5fc0 builder's first branch ($5fda:
  tst $79e2; bne $6024) emits a SEEK-ONLY list (just op $28) when [$79e2]!=0 - no loaders, no
  window, no collection: an 18ms completion. Read2/3 almost certainly took it.
  NEXT: [$79e2] semantics - what sets it (position-known? same-track shortcut?), whether its
  branch is authentic for a continue-read, and where the CONSUMPTION (head advance by count)
  is supposed to happen in that branch's flow.

## cont.74 — [$79e2] is ALWAYS ZERO (single clr-writer); the 18ms re-opens, sharpened

**The tape-by-grep: [$79e2]'s only writer is $f8a - clr.l D0 then move - it writes ZERO,
always.** No setter exists. The abbreviated-list branch ($5fda) never fires; reads 2/3 built
FULL op lists. My 18ms-via-diet-list theory is dead at the grep (Dave's stale-state lean and
my authentic-branch alternative BOTH die - the branch is unreachable).

**The 18ms, re-opened with sharper suspects:** reads 2/3 accepted 8.181/8.195 and completed
~14ms apart, doorbell-cascade style ($c stamps at pc $17fe = worker passes on the NEXT
accept). Their full lists would run: $24 select (E804 churn at 8.178 ✓ matches), $28 seek -
**[$7a36] is STALE 1 from cmd#1's settle poke (7.96208, never re-cleared?)** -> instant
seek-complete; $42 motor - possibly instant if $6bc2 checks motor-already-on (E804 bit4 set
since 7.962) rather than the timer. If every wait-op returns instantly on stale state, the
list runs to $36 in microseconds - and the node's $c comes from the next doorbell's worker
pass (completion-by-retry at accept cadence). The suspect: STALE WAIT-STATE between commands
([$7a36]/[$7a3e] not re-cleared) collapsing reads 2/3's op lists into no-ops - no window era
long enough to collect.
NEXT TAPE: per-read op-list cursor trace (OPLIST cursor + [$7a36]/[$7a3e]/[$741c] at
8.17-8.25) - which ops reads 2/3 actually executed and which waits returned instantly.

## cont.76-77 — two shim limbs retired; THE FAILURE BANNER IS GONE (runs 214-215)

**cont.76:** the cont.69 continue re-carry RETIRED (built for the dead cache-serve theory; its
instant DONE-at-arm denied every continue-read its collection time).
**cont.77 KEEPER:** the $7fe8 translation gated on BIT 7 of the posted value - the fw's own
vocabulary: completion posts = 0x80|code ($ba), accept posts = $01. The blind translation had
been posting PHANTOM host-DONEs at every accept once cont.60 released m_read_pending -
driving the CPUAP's 10ms probe cascade (each probe "completed" at its own accept). Dave's
$7fe8 ghost was the cascade-driver after all - one layer deeper than either diagnosis (not
masking errors, not masking successes: MANUFACTURING completions).

**run215: NO "no sys-floppy" in 120 seconds - first time ever.** read1 completes genuinely
(bit7-gated $ba post), read2 accepts at 8.1836, arms, and the monitor WAITS honestly (screen
static at "testend" = blocked in CXP-2B on read2's DONE). Read2's collection never carried in
the visible window - the next forensics: the 8.2-120s era (greps capped early). The machine
is honest end-to-end for the first time; read2 is genuinely collecting-or-stuck on its own
merits, with no phantom interference.
NEXT: read2's collection era - deposits/walks/wants at 8.2+ (why no carries; the second
window's targets vs the consumed ledger; where the fw's continue-collection stalls, if it does).

## cont.78 — the seek fork decoded; the protocol completes itself; read2's window anomaly is the block (run215 forensics)

**$6788 (seek) decoded:** entry gate [$7a36] (prior-settle, stale-1 = proceed, harmless);
bounds from UIB[$1a-1b]; THE FORK at $67ee: UIB[$d0/$d2] position cache vs [$7946]/[$7948] -
THE TARGET FROM THE IOPB (dispatcher $f9e loads bytes 5+). cmd#2's bytes 5-7 = 0 -> target
track 0 -> equal -> NO SEEK, correct per the IOPB. On a real seek: $14e2 batch-close (!),
UIB[$d4] last-sought, step sequence.

**The protocol's final piece: THE MONITOR ADVANCES THE TRACK ITSELF.** Read2 legitimately
reads track 0 positions 9+ (zeros - a VALID read of empty sectors); the probe loop's later
reads carry advancing track fields (IOPB bytes 5-7) until cyl 1 delivers the ZMAGIC. The fw
seeks where told; no fw-side magic, no side-crossing continuation needed - the CPUAP drives
the geometry. Read2 just needs to COMPLETE HONESTLY WITH ITS ZEROS.

**The block: read2's window.** WINPARM @12: fresh ledger (c0 x16 + fe), NO wants staked
(no $f0), no hunt arms, no walks, no IAM - and [$7428]=[$7430]=$0104 (260 - scan-overrun
flavored; the $7e6c scan promoting over a c0-only map past the terminator into the $77f8
region?). The collection never launches: the stocker/want-staking for cmd#2's window never
ran (cmd#1's launch chain: op-list -> first hunt-arm -> capture -> door -> $7106 stock; cmd#2's
equivalent bootstrap = the open question).
NEXT: cmd#2's post-op-list era - why no hunt arm ($891a-family absent >8.2), the $0104
promote's provenance, and the first-batch bootstrap for a second command.

## cont.79 — the bootstrap differential: the launch lives in op-42's SPIN-UP branch (run215 standing tape)

**cmd#1:** OPDISP $6bc2 (op-42) @7.96371 -> $741c=1 @7.96378 (pc $7d62 - the walk re-arm tail,
CALLED from op-42's spin branch) -> first hunt-arm @7.96380 ($891a). THE COLLECTION LAUNCH =
op-42's motor-spin-up side, started to overlap the 1.82s wait.
**cmd#2:** OPDISP $6bc2 @8.17927/8.18347 - dispatched, but the motor is WARM (on since 7.962,
E804 bit4) -> the instant-return branch -> NO LAUNCH. No wants staked, no arm, no hunt, no
strobes (E000 closed at $23f from op-$1a's pour; the window-open $a6d only ever comes from
the hunt's own cycle - chicken-egg for a warm start). The $0104 target = downstream garble
(the scan over a wants-less map), symptom not clue, exactly as Dave framed.

**The open question, one branch wide:** how does the REAL fw launch a WARM command's
collection - op-42's warm path must still reach the launch (a branch we haven't read), or
the launch lives one op earlier/later on a path our state skips. NEXT READ: $6bc2's full
branch structure - the spin branch's launch call ($7d4a-family per pc $7d62) vs the warm
branch's exit; what the warm side calls or doesn't.

## cont.80 — op-42 exonerated; THE LAUNCH IS EVENT-DRIVEN (run215 standing tape)

**$6bc2 (op-42) contains NO launch call** - pure motor logic (class checks, F000 ready tests,
the $7a3e/$7a40 timer, clean exits). The "launch in the spin-up branch" was an artifact of
timing coincidence: the launch at 7.96378 was an **IRQ6 WALK INTERRUPTING op-42's execution**.

**The egg-breaker, on tape:** PHASE IRQ6-entry @7.96372 (bare strobe - 7dac all zeros, no
capture) -> OPH2($7ba8) @7.96373 (the walk via the IRQ6 soft-vector, installed by op-54 at
7.96346) -> $741c=1 @7.96378 -> first hunt-arm @7.96380. **The collection bootstrap = the
FIRST FREE ID-STROBE after the thunk install enters the walk.** Not an op branch, not fw
state: an EVENT. Dave's fork (starved-signal vs early-exit) resolves to a third arm - the
warm launch is event-driven and the event is HARDWARE (the gate array's free id-strobes,
ours to deliver).

**cmd#2 starves because no strobes fire post-8.19** (no IRQ6 entries, no IAMSTAGE, no
ROT-MARK deliveries). The strobes' visibility chain: pump fires marks (stream-gated ✓ stream
still built) -> mark_visible (irq6: !data_mode...) -> line raise gated on en=[$79f8] bit11.
The state at 8.2+ (en/bit11, m_endec_data_mode, the pump's mark cursor) = unmeasured - the
named next tape. cmd#1's strobe at 7.96372 fired under the same post-pour conditions
(E000=$23f closed - the mux close does NOT gate strobes, only stream-byte reads), so the
cmd#2-era difference is in the enables/mode/pump state, one dump wide.

## cont.81 — THE DIFFERENTIAL LANDED: the sole differing gate is the WINDOW; the fw is listening (run216)

**STROBEGATE, both windows (the crash ate the write-up; run216 preserved the tape):**
- **cmd#1 firing window (7.960-7.972):** en=42d7/42d3 (bit9 SET, bit11 CLEAR), dmode=0,
  pend=0, active flips 0->1 between 7.96250 and 7.96337 - the irq6 mark at 7.96337 rides
  active=1 into the walk (IRQ6-entry 7.96372, OPH2 7.96373, first hunt-arm $891a 7.96380).
- **cmd#2 silent era (8.19-8.40):** en=aad3 - **bits 9 AND 11 BOTH SET (+13,15): the fw
  fully enabled both strobe classes and is LISTENING** - dmode=0 (ID side), pend=0, and
  active=0 for the ENTIRE era. Marks fire on schedule (irq6 at 8.20087/8.21337/8.22587/
  8.23837..., one per 11.25ms rev as always). Every mark dies at the one gate:
  `if (m_serdes_active)`.
- **No E000 mode write ever comes** (E000MODE: one line all run - the $23f close at
  6.40110): the fw never re-opens the window itself; the open only comes from E802 arms /
  the fw's $2ff pulls INSIDE a running hunt. Chicken-egg, now measured shut.
- E804 select/motor register, changes-only from 7.9: bf78 @7.96218 (cmd#1's op-$24 era),
  ff78 @8.17664 (INIT-era churn, pc $dd6) - low byte 78 CONSTANT (bit4 = motor stays ON
  through the warm era); only high-byte bit14 flips. No re-select ever follows.
- OPDISP walker tail: 6bc2 (op-42) dispatched 7.96357, then x2488 repeats (the op list
  parks on op-42 polling while the IRQ-driven hunt reads - 88us cadence), handler changes
  to $355c at 8.18320 and NOTHING after: read2's list dispatches no select/motor/seek ops
  (or parks instantly in $355c, its read op) - consistent with a list waiting on a
  collection that never launches.

**THE SYNTHESIS (cont.80 closed):** the warm-start bootstrap event the fw waits for is the
free ENDEC boundary strobe, and in cmd#2's era EVERY prerequisite the FIRMWARE controls is
satisfied (enables aad3, ID mode, handlers installed). The one blocker is OUR window gate -
a model artifact. Physical reading: the ENDEC's boundary pulses free-run whenever the
selected drive is at speed; the $23f close gates E000 BYTE READS only (cont.37's own
spindle rule, now extended to the strobes). Setup-era safety (runs 97/98's no-strobes-in-
capture-mode) is preserved by the MOTOR, not the window: before ~7.96 the drive isn't up
to speed, so no pulses exist to deliver. And cmd#1's own bootstrap timing corroborates:
the first strobe entered the walk ~1ms after the motor came ready - the strobes BEGAN with
drive-ready, not with a window-open.

## cont.82 — the warm-strobe gate: free-run on motor-on (E804 bit4) OR window

Change at the pump's strobe switch: `if (m_serdes_active)` -> `if (m_serdes_active ||
BIT(E804, 4))`. The en-gates ([$79f8] bits 9/11) still discipline each level; m_seen_id
still qualifies irq5. A/B risk, named: if E804 bit4 is ALSO set during the 6.4x setup era
(E804WR probe widened to t>6.0 to measure exactly this), free irq5 strobes (bit9 set in
42d7, seen=1 from 6.41372) could re-create the run72-74 phase-wreck signature (READ-START
storm / IDSTAGE=0) - a visible, diagnostic failure either way. Expected on success:
post-8.19 IRQ6 entries return, read2 arms + collects its track-0 zeros, completes honestly
($ba), the monitor's FE44C2 retry-probe chain advances the track itself (cont.78), cyl 1
delivers the ZMAGIC, boot proceeds past "testend".

## cont.83 — the cont.82 regression EXONERATES the gate: it never delivered; the wreck is the BOOT FLAKE + a lost recipe (runs 217-222)

**The gate is runtime-inert, proven:** run220's FREESTROBE probe (logs every delivery the
new bit4 path would add) = ZERO lines in 120s; E804-B4 (bit4 edges, whole run) = ZERO -
**bit4 NEVER goes set**, in any era. E804 low bytes on tape: 0c/0b/c8/88 (init drive scan,
0.42+), 0d (6.4x), 78 (run216's 7.96+) - bit4 was set only in run216's warm era values,
and whatever it means, the new path delivered nothing in 217-220. The cont.82 change
cannot have caused the 6.4x wreck.
- (The run216-era bf78/ff78 values DO carry bit4; the 0.4-6.4 values don't - bit4 may yet
  be motor-command, set at op-$24/$42 time - but with the flow wrecked pre-floppy in
  217-220, cmd#1 never ran, so the bit4-timing question (Dave's motor-command-vs-ready
  fork) is still open, unmeasured.)

**What actually differed - TWO lost-recipe pieces + the known boot flake:**
1. My post-crash runs dropped `-oslog` (217: logerror went nowhere) and added
   `-sound none -nothrottle` (217-220). run215/216's exact recipe was lost with the crash.
2. Runs 217-220 (sound none, nothrottle): "no sys-floppy" at 6.4 - and run219's tape shows
   the host hammering $7ff8 UNANSWERED from 6.40002, the storager never consuming the
   mailbox: the board wedged BEFORE read1's doorbell, during its own init - with the gate
   delivering nothing. First 1.0s of storager events = IDENTICAL to run216 (CMDDONE/WAITER/
   OPCUR counts equal, 0.1ms drift); divergence lies in 1.0-6.4, unlocalized.
3. Run221 (sound on, throttled - closest to the old recipe): a THIRD behavior - the CPUAP
   self-test ends ~5s EARLY (0x95 accepted at 1.348 vs 6.41), floppy read fails fast,
   straight to "waiting for harddisk ready".
**cpuap.cpp:168 names the mechanism (pre-crash knowledge):** the boot flake "diverges in
the CPUAP's first ms, before the FE02B6 RAM-size probe"; the MC146818 is HOST-TIME-SEEDED -
the only nondeterministic input. CPUAP_RTCLOG (tap the RTC-window reads) and CPUAP_RTCFIX
(pin the seed) were already built for exactly this A/B. The run215/216 baseline was almost
certainly pinned (or lucky); every post-crash run rolled the seed.
NEXT: re-run the cont.82 A/B under CPUAP_RTCFIX=1 - if the run216 timeline returns
(testend 6.4, read1 6.41-7.96, read2 8.18+), the FREESTROBE/E804-B4 probes finally measure
the gate against the real warm era. Success/failure criteria unchanged from cont.82.

## cont.84 — cont.83 CORRECTED: the wreck was MY OWN incomplete recipe; the verbatim wrapper recovered from the transcript

The crashed session's transcript (c5ad6074...jsonl) preserved the exact run215/216 launch
wrapper, and it convicts the recovery-era runs, not the code:
- **SEVEN STORAGER_* env gates**, not two: NOBYPASS, IAM, PHASELOG, BUSYHOLD, STEPIRQ,
  SEEKACTIVE, STEPBIT0. The five I dropped are BEHAVIORAL feature gates - without them the
  model runs different seek/step/busy/IAM hardware, and read1's 6.4x choreography fails
  instantly ("no sys-floppy" right after testend). Runs 217-220, 222, 223 all ran this
  crippled config; the "regression" was never real. (The gate stayed inert throughout -
  FREESTROBE=0 holds in every run; cont.83's exoneration stands.)
- `-log` (error.log, copied per-run), `-nothrottle`, sound default, scratchpad flop copy
  (byte-identical to siemens/set1). No CPUAP_RTCFIX: the wrapper RETRIES up to 3x, gated
  on `4FFFFC` in error.log + `testend` on screen - the boot flake was handled by retry.
  ("try1: BOOTED" in the old task outputs = this loop. Runs 221 vs 222 with identical
  commands = the two flake arms, live.)
- The transcript also settles the tree question: the LAST source edit before the crash was
  the cont.81 STROBEGATE probe at 18:27:06, immediately followed by build-c81 + run216.
  No hidden edits; the tree is exactly the run216 tree.
The verbatim wrapper is preserved as run-storager.sh (session scratchpad) and in the
storager-run-recipe memory. cont.82's A/B now re-runs under it as run224: success criteria
unchanged (post-8.19 strobes -> read2 honest zeros -> monitor self-steers to cyl 1);
E804-B4 additionally answers Dave's motor-command-vs-ready fork on a healthy cmd#1.

## cont.85 — THE REAL READ2 BLOCKER: the fw has been DEAD since 8.1844 in every run; null node-status dispatch -> vector-table execution -> fatal park (runs 224/225 + baseline re-read)

**cont.82 verdict first (run224, verbatim recipe, try1 BOOTED):**
- E804 bit4 rises at 7.96218 (pc $6b24, op-42 era) - AFTER the 1.55s spin wait, exactly when
  strobes historically began. NOT set during spin-up; not set at 6.4x. Dave's cold-side risk
  (motor-command vs ready) resolves clean: bit4 tracks READY. The 6.4x setup era ran
  undisturbed with the gate in (clean boot, no banner) - run97/98 safety intact.
- FREESTROBE delivers in BOTH windows: cmd#1's bootstrap (7.9625+, bit4 path, window still
  closed) and cmd#2's era (8.1875+, en=aad3, 12.5ms cadence). THE GATE WORKS AS DESIGNED.
- And it doesn't matter for read2, because:

**THE PARK (run225 PARKTRAP + the pre-existing DEATH-25e tap, re-read into runs 215/216):**
- $24a = `move #$2600,SR` + spin at $256 (E800<-0e1b / andi $7d98 / bra) - the fw's FATAL
  TRAP. ROM vectors 2 (bus err), 3 (addr err), 15 (uninit int), 24 (spurious) ALL point at
  it. The $256 "idle loop" at 9.8 in every tape = a parked corpse, IPL masked to 6.
- Frame, identical in runs 215/216/224/225: SR=2008 PC=$8c ret=$2314 at sp=7d8a,
  **@8.18442-8.18444 - 0.8ms after read2's accept (8.1836), BEFORE the first free strobe
  (8.1875)**. The park is in the BASELINE: runs 215/216 died the same way at the same
  address; DEATH-25e was on their tapes, unexamined. Every read2 forensic from cont.77's
  "monitor waits honestly" through cont.80's strobe starvation was performed on a machine
  that had been dead since +0.8ms. (The strobe gating was real but IRRELEVANT: a parked fw
  at IPL6 can't take IRQ5/6 anyway.)
- **Mechanism, fully decoded:** the channel-service event dispatcher $2290-$2312 loads
  D0 = node+26 (status/event code) from the node at [$71bc] (fallback [$71b6]), special-
  cases 4/$c, table-dispatches the rest through ROM table $222: entries {0:0000, 2:0000,
  6:0000, 8:$156a, a:$15fe}. **node+26 was 0000** (tape: 71b2<-0001 stamp at 8.18431 shows
  n[+26]=0000, 0.13ms pre-death) -> jsr 0 -> executes the vector table as code from $0 ->
  ILLEGAL at $8c -> park. (Same PC=$8c as run52's wild-execution frame - one mechanism,
  now named: ANY jsr-through-null lands there.)
- The same RDSTEP-op0 path ran at 6.40906 (cmd#1 accept era) and SURVIVED (7940/7942 zero
  then vs 29c0/0014 in the fatal instance) - the asymmetry between a first command and a
  back-to-back second command's node state is the live question.

NEXT TAPE: node+26 provenance for the 8.184 dispatch - tap $2290/$2298 (log [$71bc],
[$71b6], both nodes' +26 live) + static sweep of all node+26 writers (the $2244 `move #$8`
stamp, the $17fe $c stamp, the WKR-CLR12 clear path) to name what SHOULD have stamped it
before this dispatch and who consumed/cleared it early. Suspects: the model's doorbell-era
CCB stamps (71f0 MODEL-writer) racing the fw's own node setup for a second command, or a
worker pass consuming the node before the dispatcher's read (the doorbell-cascade
choreography cont.74 flagged). The fw is blameless until the stamp trail says otherwise.

## cont.86-87 — the park mechanism caught live; ordering confirmed as the lever; the completion belongs on the ROTATIONAL CLOCK (runs 226-227)

**run226 (DISP26 tap):** the fatal $2290 dispatch runs with **BOTH queue heads null**
(71bc=0000 71b6=0000 @8.18440) - the code skips the +26 load when [$71bc]=0, falls through
with the CALLER'S STALE D0, and table-dispatches it: stale 0 -> null entry -> jsr 0. On the
healthy path the queue is NEVER empty mid-era: node 71c6 sits at status 8 from 6.411
through 8.176 (DISP26 x19) - status 8 = in-progress, re-dispatched every pass. The fw
assumes a non-empty queue whenever the main loop makes a dispatcher pass; the walker's
WKR-CLR12 consume (driven by our synchronous CHANCOMPLETE IRQ4, raised mid-ISR at the $3d4a
kick, pending to re-enter at RTE) emptied it 15us before the main line's read.
Static: table $222 = {0:null, 2:null, 4:$1f82(special), 6:null, 8:$156a, a:$15fe,
c:special}; the fw itself stamps 2 ($261a) and 6 ($2000) as intermediate states consumed by
the special paths - reaching the table with 0/2/6 is by-design-impossible on real hw.

**run227 (cont.87, CHANCOMPLETE deferred 300us via m_chancomplete):** the park MOVES
(8.18444 -> 8.18684, +2.4ms) and read2 now completes AS AN ERROR (monitor prints "no
sys-floppy, going to harddisk" at 8.2 - first read2 completion of any kind on the tape).
Ordering is the lever - but 300us is ~40x short of a real ID-verify (a mark passage,
us..12.5ms). The walker still consumes the read node mid-continuation; the RDSTEP/$1dfa
main-line pass still eventually finds an empty queue and parks.

**THE MODEL FIX, named:** disk-class channel-op completion (the E800 kick at $3d4a) must be
raised from the PUMP'S MARK SCHEDULE - the completion event IS the mark passage the op
waits for (ID-verify: the next id mark matching want-C/H/R; the infrastructure exists:
m_serdes_marks, serdes_pos, IDVFY's want-cells) - not from a wall-clock timer (100us
placeholder, 300us experiment, both wrong). Then the dispatcher always reads a queued node
(the op is genuinely in flight for a revolution-scale interval), the walker consumes on the
TRUE completion, and the queue never empties mid-flow - the invariant the fw's dispatcher
is built on. NEXT: wire chancomplete for cmd-0x95 kicks to the pump (complete at the next
qualifying id mark), delete the 300us experiment, re-run; then read2's real result (zeros
from track 0) and the monitor's probe-chain advance are finally reachable. The cont.82
strobe gate stays (validated, cold-path-safe, bit4=ready).

## cont.88 — mark-scheduled completion alone does NOT cure the park (run228)

CHANCOMPLETE moved to the rotational clock (next id-mark passage, 0.09-12.5ms genuine
in-flight time; 300us fallback). Boot try1 clean (setup tolerates revolution-scale
completions - the fw's "1.5ms window" fear was unfounded). But the park persists
(@8.18731, same frame) and read2 still errors out at 8.2. The 8.184-era queue-emptying is
therefore NOT (only) the walker racing on our early IRQ4 - with completions now
revolution-scale, something still zeroes BOTH heads ([$71bc]/[$71b6]) before a main-line
dispatcher pass. NEXT TAPE (one run): write-trace $71bc and $71b6 (installer write taps,
pc+value+time, windowed 8.17-8.19 + the 6.40-6.41 healthy accept for contrast) - name the
emptier and the flow making the fatal pass; then decide whether the model owes a re-queue
(a completion class that should leave/put a node on the head) or owes suppressing the pass
(an event the fw wouldn't see on real hw). Park frame decode + table map: cont.85/86.
Keep: cont.82 gate (validated), cont.87/88 rotational completion (correct physics
regardless; the wall-clock guesses were wrong on their own terms).

## cont.89 — the head-emptier is $1b70 (fw dequeue); the missing piece is the RE-POPULATOR (run229)

HEADWR tape: [$71bc]<-0000 at pc $1b70 - x3 in the healthy accept era (6.40144/293/563,
each re-populated before the next $2290 pass: DISP26 saw 71f0 at 6.40741) and x1 in the
fatal era (8.18614; the 8.18684+ pass then found BOTH heads null -> park 8.18731). $1b70 =
the fw's dequeue (the $1ba4 +26 clear is adjacent). So the invariant is: after $1b70's
zero, EITHER a re-populator runs before the next main-line pass, or no pass happens. In
the fatal era neither held.
CAVEAT (probe method): the head re-population (71c6->71f0 between 8.17576 and 8.18319) was
NOT caught by the AS_PROGRAM write tap - it happens through a different space handle
(model/bus-side writes bypass CPU-space taps). Future head/queue traces must tap the
MULTIBUS-side path too, or instrument the model's own writers.
NEXT: disasm $1b40-$1bb0 (the dequeuer: what advances the head - a node link field?) +
name the re-populator (fw code via another pc, or the MODEL's doorbell/completion writers);
then diff healthy-vs-fatal: why no re-population before the fatal pass (does the fw expect
a node from the NEXT channel op's accept, which our completion timing suppressed?). Park
mechanism fully mapped: cont.85 (frame/table), cont.86 (null heads live), cont.87/88
(completions on the rotational clock - KEEP), cont.89 (dequeuer named).

## cont.90 — FORENSIC DISCIPLINE + the dequeuer decoded; the head change-poller (task #8, Dave's ordering)

**PROCESS RULE, permanent (Dave): LIVENESS-GATE EVERY READ2 FORENSIC.** Before reading any
tape region, check the CPU is alive at that timestamp - grep PARKTRAP (tap stays in the
probe set permanently) and confirm no park precedes the region. A parked CPU at IPL6
produces downstream silence that narrates as "waiting"/"starved". This single check would
have saved the cont.77-80 autopsies-read-as-vital-signs.

**$1b42-$1ba4 decoded = COMMAND TEARDOWN:** clears ALL wait/settle flags ($7a36/$7a3a/
$7a42/$7a3e - so cont.74's "stale $7a36" gets cleared HERE, at teardown, not leaked),
ori #$c into [$79f8] + writes it to E802 (bits 2+3 - the between-commands enable state),
then clears the head selected by D0 bit4 ($1b6a: [$71bc], $1b74: [$71b6]), decrements the
active count [$71b2], clears node+26. The zero is TEARDOWN; the re-populator is the ACCEPT
path.

**Instrument blind spot, symmetric:** the 71f0 head refills were uncaught by the AS_PROGRAM
write tap in BOTH eras (healthy 6.40563->6.40741 AND fatal-era 8.17576->8.18319, both
in-window, cap not hit) - and the host physically cannot address $71bc (host window maps
pio 7200-73ff -> fw 7e00+ only; the model never writes the heads: grep clean). Per Dave's
ordering: characterize the HEALTHY refill on the wire first - cont.90 adds a writer-
agnostic CHANGE-POLLER at pump-mark cadence (HEADPOLL, 6.39-8.30, change-only) to bracket
every head transition regardless of writer; only after the healthy refill is understood
does the fatal absence count as real. Run230.

## cont.90 results — the re-populator is the fw ACCEPT PATH ($ef0); read2's accept queued NO node; teardown-vs-accept ordering is the kill (run230)

HEADPOLL + HEADWR + DISP26, one run, self-consistent (the run226 "71f0 head" was flake
variance between runs, NOT a tap bypass - the AS_PROGRAM tap catches the real writer fine):
- 6.401-6.406: three $1b70 teardowns (the 87/89 setup commands); head empty between.
- **6.41368: pc $ef0 writes [$71bc]=71c6 - THE RE-POPULATOR = the accept path queuing the
  command's node.** 71c6 stays queued at status 8 for the whole of read1 (6.41-8.17;
  DISP26 x100s), with one transient teardown+requeue at 7.975->7.989 (read1's completion-
  era continuation re-queue, also via $ef0-class flow).
- **read2's accept (8.1836) NEVER ran $ef0** - no head write after 8.1836 (window covers
  it). The accept found the head still occupied by read1's continuation node.
- 8.18614: teardown ($1b70) finally clears 71c6. Head empty, no re-queue owed by anyone.
- 8.18684+: main-line dispatcher pass -> stale D0 -> park 8.18731.
**The kill is teardown-AFTER-accept:** on real hw the prior command's node tears down
before the next accept processes (the monitor rings the next doorbell only after DONE, and
the fw completes teardown in its DONE flow), so the accept queues into an empty head. In
our flow the accept ran with the head busy (queued nothing), teardown came 2.8ms later,
and the fw's non-empty-queue invariant broke. Note the park exists at EVERY completion
timing tried (sync/300us/rotational) - the teardown lateness is not (only) IRQ4 pacing.
NEXT (cont.91, one run): tap $ef0 (accept-queue: D0/node/[$71b2]/[$743a] + head state) and
$1b42 (teardown entry: what triggered it, caller/stack) with wide windows 6.40-6.42 +
7.96-8.00 + 8.17-8.19 - name (a) the accept path's queue-or-skip decision input, and
(b) the teardown's trigger event; then determine which side the model owes: an earlier
teardown (read1's DONE flow blocked on something model-owed?) or a deferred/second accept
pass the fw expects (doorbell re-ring? the 0x95 retry cadence?).

## cont.91 — read2's dispatch never reaches the node-build path; the fork is UPSTREAM of $ee4 (run231)

ACCEPTQ ($ee4, the btst #5,D5 queue decision): TWO visits all run - 6.41368 (read1's
initial queue) and 7.98860 (read1's continuation re-queue), both D5=8c27 (the UIB[$20]
flags word; bit5 SET -> queue; ret stack [2038 71c6 71f0] = called from $2038-era with the
node list). **Read2's era (8.17-8.19): ZERO visits - the 0x95 dispatch for a back-to-back
command never enters the $ec0+ node-build/queue path at all.** The fork is upstream:
between the doorbell dispatch ($dee/$d54-family, DOORCMD pc 1982/1994) and $ec0. (Probe
note: the $1b42 TEARDOWN tap caught nothing - $1b42 is the PREVIOUS function's epilogue
(unlk); the teardown body at $1b48+ is a branch target; re-tap at $1b48.)
NEXT (cont.92): static-trace the 0x95 dispatch path from $dee to the $ec0 node-build -
find the branch that diverts a second 0x95 (queue-for-later? busy check? [$71b2] count?
the $17fe worker-pass route from cont.74), and tap ITS decision input. Also re-tap
teardown at $1b48 for the trigger. The invariant to restore: a back-to-back 0x95 must
either build+queue its node ($ee4 path) or the prior node's teardown must not strand it.

## cont.92 — THE NODE MAP DECODED: two nodes, a status ladder, and read2's node built-but-never-queued (run231 + statics)

**The key that reframes every prior tape: node+26 cells resolve as 71c6+$26=$71ec and
71f0+$26=$7216 - the "$71ec/$7216 status cells" ARE the +26 fields of exactly TWO command
nodes (71c6 and 71f0).** Rereading with the key:
- The node-build routine entry = $bba (runs through $e82-$113a): validates cmd vs UIB[0]
  (uninitialized unit admits only 87/77/a7; else error $40 -> $1206/ERR1206), loads
  **D5 = UIB[$20]** (the flags word, 8c27 for our unit; bit9/bit5/bit4 branches at
  $eb2/$ec6/$ee4), and bit5-set QUEUES the node onto [$71bc] ($eec).
- Caller: $2034 (bsr $bba) inside the STATUS-6 era code ($2000 stamps 6, then A4=node+24,
  D0-D3 from the request bytes, build+queue). The ladder: **4 -> 6 -> (build+queue via
  $bba) -> 8 (in-flight, re-dispatched every pass) -> c (complete) -> teardown ($1b48+)**.
  Table $222's nulls (0/2/6) are ladder states consumed by dedicated code, never table-
  dispatched - confirming reaching the table with them is by-design-impossible.
- **The tape, re-read: read1's node 71c6 hits status c at 8.18334 ($17fe stamp). Read2's
  node 71f0 WAS BUILT and stamped status 8 at 8.18319 (pc $224a) - the node exists! - but
  never landed on the head** ([$71bc] stayed 71c6 until teardown zeroed it; ACCEPTQ/$ee4
  never ran in the era; [$71b6] zero throughout). A status-8 node that is not on a head is
  invisible to the dispatcher: when 71c6 tore down (8.18614), the walker had nothing, and
  the next pass parked.
**The remaining one-branch question, sharpened: what puts a SECOND command's node on the
head.** Read1's path was ladder-6 -> $bba -> $eec (head write). Read2's node got to
status 8 WITHOUT the $bba path (stamp pc $224a, the $22xx walker's own flow) - meaning the
fw has a second-command flow that stamps 8 expecting the node to be (or get) head-queued
by something else - OR read2's node was stamped 8 spuriously by the walker treating it as
already-current. NEXT: (a) disasm the $2244-stamp's surrounding flow ($21xx->$224a: what
A0 is it stamping and what put A0=71f0 there with [$71bc]=71c6); (b) re-tap teardown at
$1b48 (real entry; $1b42 was the prior fn's epilogue) + tap the status-c handler chain
($22fe->$23e6, the chain-to-successor: does it run at 8.1834+, and what does it need to
hand off to node 71f0). The invariant candidates stand (queue read2's node vs teardown
hands off); the $23e6 chain is now the prime suspect for the model-owed event's consumer.

## cont.92b — the successor-handoff contract: the status-c chain hands off ONLY to a status-4 node (statics)

$23e6 decoded (the status-c/completion chain): node+18==$18 gate -> clear anchor fields ->
bsr $16c6 (completion post) -> on success, **tst [$71b2]; beq $b00 (no actives -> idle);
else read [$71ec] (node 71c6's status): ==4 -> $1f82 (LADDER START for the successor);
$2442 then checks [$7216] (node 71f0's status) the same way.** THE CONTRACT: chain-to-
successor requires the successor node's +26 == 4 (fresh-accepted). At the fatal handoff,
node 71f0 held 8 - no 4 anywhere - so the chain fell through, nothing was queued, and the
teardown stranded the head.
Also decoded: the $2214-$2244 flow = the ladder's LAUNCH leg (movem, bsr $6788 SEEK,
restore, stamp status 8 on A0) - so 8-stamps mark launch. The 8.18319 stamp on 71f0
PRECEDES read2's doorbell (8.1836): node-to-command attribution in the 8.17-8.19 era needs
tape, not inference (the 87/89 INITs at 8.1758/8.1775 are in play for 71f0).
**NEXT TAPE (fresh session, one run): the node-status movie.** Write taps on $71ec and
$7216 (every transition, pc+value) + ACCEPTQ($ee4) + teardown at $1b48 (real entry) +
DOORCMD, window 8.17-8.19 with the 6.40-6.42 healthy contrast. That assembles: which node
served which command, who stamped what when, and why no node was at 4 when the chain
looked. Then the fix falls out: either the model owes the event that would have left the
successor at 4 (likeliest - the doorbell/accept flow for a back-to-back command), or the
teardown/chain timing is model-skewed. LIVENESS-GATE the reading (PARKTRAP first).

## cont.92c — movie reading order (Dave, session close): attribution BEFORE absence

The movie's FIRST job is not "why no 4" - it is **"which node was supposed to carry read2,
and was it ever a candidate for 4 at all."** Read the build-and-first-stamp sequence for
read2's node BEFORE reading the chain's look, because "no 4 existed" has two upstream
causes only transition ORDER separates:
- read2's node never entered the ladder (consistent with $ee4 zero-visits) -> the missing
  4 is downstream of a missing BUILD -> back to the diversion branch;
- read2's node built but got stamped PAST 4 too early by an INIT-era event (71f0 hit 8 at
  8.18319, before read2's 8.1836 doorbell, with the 87/89s in play) -> the model-TIMING arm.
The healthy 6.40-6.42 contrast supplies the presence, not just the absence: find the pc
that stamps status-4 on the healthy successor, then watch that pc not fire / fire on the
wrong node / fire late in the fatal window. Every frame liveness-gated (PARKTRAP first).
One-line bug statement, current best: THE SUCCESSOR NEVER REACHED 4 WHEN THE CHAIN NEEDED IT.

## cont.93 — THE MOVIE (run232): the arc unified. Read2 = 71c6's continuation, ALIVE and hunting; the park is the RECOVERY path; the live bug is HUNT CADENCE

**The stamp grammar (healthy contrast, complete):** $262c stamps 4 (ACCEPT - right after
each DOORCMD), $2006 stamps 6, $224a stamps 8 (launch, post-seek), $17fe stamps c
(complete), $1d0e clears. INITs (87/89/80/87) cycle node 71f0 (4->6->c->0, ms-scale);
read1 (0x95) rides node 71c6: 4@6.40461 -> 6 -> queue@6.41368 -> 8@6.41420 -> c@7.96505
(the genuine $ba completion) -> 0@7.96575 -> **RE-4@7.97717 = READ2 IS 71C6'S SECOND PASS**
-> 6 -> queue@7.98860 -> 8@7.98894 -> stuck in-flight 200ms (never c).
**The fatal sequence:** read2 hunts 7.99-8.18 ON A LIVE CPU (45 IRQ6 entries, 16 walks, 32
hunt-arms - the collection machinery RUNS; the corpse era is over). The monitor times out
(~200ms), fires recovery INIT 87@8.1846 (71f0: 4->6) + NEW cmd 0x98@8.1852 (status/reset
class, bytes 80 80 00 01) -> 71f0 c@8.18533, cleared@8.18620 -> ITS completion chain looks
for a successor at 4, finds 71c6 legitimately at 8 (mid-flight) and 71f0 at 0 -> no-4
fall-through -> stale-D0 null dispatch -> PARK 8.18731. Post-8.19 "DOORCMDs" at pc
$250-$25e = the host ringing a corpse (liveness gate catching post-mortem frames).
**The LIVE bug, isolated: hunt cadence.** Read2's want is CLEAN (C=0 H=0 R=0001 - the
cont.78 "$0104 garbled want" was a corpse artifact). The capture chain steps sector-by-
sector (7dac R: 0d -> 0e -> 0f in ~0.86ms steps = per-mark, correct) then STALLS a full
revolution (11.25ms) before the next step-burst. On real hw the hunt advances every id
mark (1/16 rev); ours averages ~2-3 sectors/rev - too slow to crawl from R=0d to R=01
(wrap) inside the monitor's timeout. 200ms was enough for ~5 revs of honest per-mark
stepping; we delivered ~16 revs of stalling.
NEXT (fresh context): the stall anatomy - between a step-burst's last capture and the next
burst, what is the fw waiting for that only comes once per revolution? Candidates: the
walk's re-arm path gated on something the model posts once/rev (IRQ3 index? the $7e6c-scan
ledger cycle?); or the free-strobe delivery pattern (cont.82 gate) interacting with the
fw's enable toggling (en 42d7<->4ad7 per arm) such that only ~2-3 of 16 marks get through.
Tape: per-mark delivery-vs-consumption ledger for one stall revolution (every irq6 mark:
en at mark, delivered?, IRQ6-entry?, arm?) in 8.00-8.03. ALSO fw-blameless check: the
recovery-path park (no-4 fall-through) may still be model-owed - on real hw the monitor's
INIT-era recovery presumably finds the in-flight read ABORTABLE (the 0x98 class?) - but
that's second in line; fix the cadence and the recovery path never fires.

## cont.94 — THE LEDGER (run233): the stall is THE PUMP ITSELF - a model scheduling bug, reproducible with the fw dead

The delivery column answered with a third arm neither candidate named: **the pump fired
only THREE id marks in 35ms (pos 10 -> 154 -> 298, successive sectors, 12.5ms apart)** -
the other ~15 id marks per revolution NEVER TICKED. Each mark that did fire was a capture-
completion (cmpl=1, pend=1, e802b15=1), correctly suppressing the free strobe (no-double-
fire), with the CPU at IPL7 at mark time for the first two (completion pends, delivers
late - secondary). The fw consumes everything it is given; it is given one mark per rev.
**Proof of model-ownership: run216's 9.795-9.81 window (POST-PARK, fw dead, zero arms) has
the same signature** - marks at pos 2303 -> 0 -> 10 -> 14 in a 1.3ms burst, then silence
for the rest of the revolution. No firmware interaction: the pump alone fires a short
burst of consecutive marks then stalls one full rev. serdes_align_and_start is correct
(aligns to next mark ahead); serdes_schedule_next's math reads correct in isolation; the
defect is in the tick/schedule cycle (m_next_mark advance vs the mark list's ordering/
content, or a mid-cycle re-adjust pushing next_t a revolution out).
NEXT (one run, fw-independent): PUMPTRACE - log every pump_tick entry (m_next_mark index,
mark pos+irq, now%size, computed next delay) across 9.79-9.83 (the dead-fw window - zero
interference) plus 8.00-8.02 (the hunt era). The wrong step will be visible in one
revolution of frames: which index/delay computation jumps a rev. Fix shape: make the pump
fire ALL 49 marks per revolution; read2 then steps per-mark, completes in ~5 revs, no
timeout, no recovery, no park. (Hygiene note for the fix era: the IPL7-at-completion
observation may matter next - completions pending against a masked CPU - but cadence
first.)

## cont.95 — PUMP EXONERATED (my unit error); the hunt sees EVERY sector including the wanted one and doesn't take it (run234)

**The "one mark per rev stall" was a UNITS mistake, mine: a 5.25" rev at 300rpm = 200ms;
12.5ms is the SECTOR pitch (144 bytes x 86.8us/byte - PUMPTRACE confirms every mark fires
at its exact byte position, all 49 per rev, now%=pos-1 benign truncation).** The pump is
healthy; the hunt steps ONE SECTOR PER SECTOR-TIME = correct per-mark cadence. cont.94's
"model pump bug" is RETRACTED; the run216 9.8-window "burst then stall" was the same unit
error read into a healthy tape.
**The real finding (run234 hunt window):** the captures present R = 01,02,...,0f,10
sequentially - **including want R=01, three walk-entries on it - and the fw does not take
the match**; the scan runs the full track (~195ms) into the monitor's ~200ms patience ->
recovery INITs -> no-4 park. Captured record: a1a1a1fe 00 00 01 00 (C=0 H=0 R=1 N=0);
want C=0 H=0 R=1. N comes from the STREAM (stage_next_id reads [idpos-3]) - so N=0 is
build_serdes_stream's track content (possibly correct if track 0 is FM/128; the UIB
re-INIT switches FM128<->MFM256 - VERIFY against the IMD's real track-0 sector size).
NEXT (one tape): what the fw does with the R=01 capture - the $7e1e/$7e50 match-
continuation vs re-arm at those three walk entries: (a) if the compare REJECTS: which
field (N? C-16-bit fold? the virgin test?) - dump the compare operands at $7c40/$7c52-era
for the R=01 pass; (b) if the match FIRES: the data-stage after it (irq5 data capture,
stage_data_record) failed silently - tape the data-capture arm/completion for sector 1.
Also check the IMD: is track 0 side 0 truly 16 x 256B MFM (then N must be 1 and the
BUILDER is the bug) or FM/128 (N=0 right, rejector elsewhere).

## cont.96 — THE JUDGMENT DUMP (run235): read2 COLLECTS PERFECTLY; the missing piece is the BATCH-CLOSE at the fe terminator

The compare-reject theory dies at the first two lines: **R=01 MATCHED at 8.00123 and the
want advanced** (7428: 0001 -> 0002 -> 0003 -> 0004 -> 0005 in step with captures R=01..05,
12.5ms apart = adjacent sectors at disk speed), the ledger consuming ff->c0 per matched
sector. The IMD adjudicated N offline first (track 0 = FM 16x128, N=0 CORRECT; cyl 1+ =
MFM 256 - the builder is faithful). The virgin/consumed lean dies too: the ledger's ffs
are WANTS, consumed cleanly in order.
**Read2 = the probe-loop's count-4 pass (initial ledger: c0 + ff x4 + fe x13 - FOUR wants,
sectors 1-4), fully collected by ~8.051 - 130ms INSIDE the monitor's patience.** The bug
is the BATCH-END: at slot 5 the walk hit the fe TERMINATOR and kept going - want advanced
to 0005, 0006, ... hunting R=05..10+ past its own batch instead of CLOSING (the $8a48
sustained-disarm -> drain -> $ba DONE post never fired). cont.72 saw this exact shape on
the corpse ("dequeues-and-regrinds at the fe-terminator"); it is the live bug: collection
succeeds, batch-close judgment fails, DONE never posts, monitor times out at ~8.185,
recovery INITs, no-4 park.
NEXT (one tape): the fe-encounter (~8.0512-8.0637) - what the walk does at ledger slot 5:
which code reads the fe (the $7e6c scan / the $7e1e/$7e50 continuations' deeper branches),
and why it re-arms/advances the want instead of entering the batch-close ($8a48 area /
drain / $7fe8 $ba post). JUDGE window moved to 8.045-8.075 + opcode taps on the fe-branch
candidates. Read1's SUCCESSFUL batch-close (~7.96-7.97, its count-8 batch on side 0/1) is
the healthy contrast on the same tape (window 7.955-7.975).

## cont.97 — THE DISARM SHAPE (run236): the fw never commands the close; its want-advance rolls past the staked batch, overwriting the fe terminators

**Dave's fork, answered:** read1's close = bit15 SUSTAINED LOW 20ms+ (silence across
7.955-7.975, re-arm only at 7.975087 for the next era) - the close mechanism works. Read2's
fe era = dip-and-re-arm ONLY (fall $88c6 -> re-arm $891a 12us later, per sector, 12.5ms
cadence) - **the sustained disarm never fires, and the pcs are the FIRMWARE'S OWN hunt
cycle: the fw never commands the close.** Not candidate (a) (no close-path entry to bounce)
and not pure (b) (the model isn't swallowing an event): the fw doesn't think it's done.
**Caught in the act:** past slot 4, the walk OVERWRITES the ledger's fe terminators with
the captured sector IDs (slot5<-05 @8.063, slot6<-06 @8.076...) - the want-advance treats
the fe region as more batch and records as it grinds. The "dequeue-regrind" is a
batch-limit failure: the fw's continue-vs-close decision does not key on the staked ff
count; it keys on a LIMIT/COUNT cell - prime suspect [$7430], which reads 0044 at the
batch's first judgment (7.99992) and then mirrors the want thereafter (0002, 0003, ... -
i.e., it is REWRITTEN per match; its initial 0044 is the untouched limit encoding).
Read2's true count is 4 (the ledger staked ff x4); if the fw's limit comparison reads
0x44 (68) - or a misparsed/mistranslated count cell - the batch never looks complete.
NEXT: (a) static - the want-advance code (who increments [$7428] and rewrites [$7430]; the
$7e50-continuation's deeper branch) and the close-decision's exact operands: what
comparison read1 satisfied at its close (~7.9650) that read2 never satisfies; (b) the
count's PROVENANCE: [$7430]'s 0044 vs the IOPB count field for the probe-loop's second
pass (FE2C30 says 4) - if the model's IOPB/UIB staging mis-scaled the count (sectors vs
128B-units vs 512B-blocks - the FM128 track makes unit errors cheap), the fw is blameless
and the fix is the staging. Healthy contrast: read1's close context at 7.9650 (its limit
cell at close time) on the same tape via a widened JUDGE window next run.

## cont.98 — [$7430]'s plumbing + a hot lead: the walk checks the ledger for an $aa terminator; ours holds $fe (statics)

**[$7430] writers, complete:** $7e86 (D0 -> $7430, inside the walk = the per-match rewrite
seen on tape), **$849a (move ($2,A0) -> $7430 - the BATCH-START LOAD, from a struct's +2
field)**, $84c6 (from [$7a62]), $862e (D0), and **$8628 copies [$7430] -> [$7428] (limit
seeds the want!)**. So the 0044 provenance = whatever struct $849a's A0 points at (request/
UIB field +2) or the [$7a62] path - decode those callers next; Dave's caution stands (0x44
is not a clean scale of 4: not scaled, likely stale or a mis-filled source cell; read1's
count was 8 - 0x44 is not an obvious function of 8 either, so "stale ghost of read1"
needs the load-site trace, not arithmetic numerology).
**The hot lead ($7e20-$7e6e, the walk's continue/close region):**
- $7e20-2e: D1 = (A1+1 byte) x3 vs [$79a4] (the run counter: 00d1/00fd/0100 on tape);
  counter <= SPT*3-ish -> continue via $7d4a (the re-arm). A three-revolutions-of-sectors
  bound - the walk's own give-up horizon?
- $7e50-5c: ledger byte D0: ==$ff -> continue (consume a stake); == want -> continue;
- **$7e66-6e: the deeper branch checks the NEXT ledger byte for $aa (cmpi.b #$aa,(A0,D0.w))
  - the fw's batch-end marker may be $AA, and our ledger's terminator region holds $FE.**
  If the close path keys on finding $aa past the last stake, and the stocker (fw's $7106?
  or model staging?) filled $fe, the close can NEVER match - the walk consumes the fe as
  not-aa and grinds on, overwriting them with sector IDs exactly as run236 showed.
NEXT: (a) static - who writes the ledger's terminator bytes (fw stocker vs model staging;
grep model for 0xfe fills - the MAPPROBE probe itself writes fe 9..17!, and stage/dispatch
paths may too) and the $7e6e branch's full context (is not-aa the continue and aa the
close?); (b) the $849a/$7a62 load-site callers for [$7430]'s 0044; (c) run: dump read1's
close-time [$7430]/[$79a4]/ledger tail vs read2's - the differential Dave specified. If
the fw's own stocker writes $aa terminators on real batches and OUR c0-fill/stake staging
wrote $fe (an HLE-era guess), this is a one-byte model fix: stake the terminator as $aa.

## cont.99b — $849a NEVER RUNS (run237, zero SEGLOAD visits); the live loader is $84c6: [$7430] <- [$7A62], the INCREMENT-MODE counter

The segment-table path was a decoy for OUR flow: zero $849a executions all run. The
$8608 branch: **[$791a] != 0 (UIB-flags bit8, staged at accept $ed6) selects INCREMENT
MODE** - want-advance bumps [$7a62] and chains it ($8612 -> $8628: 7428 <- old 7430,
7430 <- [$7a62]) - matching the tape's want marching 0005, 0006... The batch-start loader
$84c6 does [$7430] <- [$7a62] directly. **The 0x44 = [$7a62] at read2's batch start - a
RUNNING COUNTER; if read1's pass left it at 68, this is the stale-leftover to the letter,
carried in [$7a62].** (68 as read1's legacy: its count-8 batch + continuation hunting
could plausibly advance a per-capture counter to 68 by 7.977 - measure, don't bet.)
NEXT (one tape): [$7a62]'s biography - write-trace it (all writers, pc+value) from 6.40
to 8.06: its value at read1's batch start (expect a RESET to read1's base), through
read1's collection, at read1's close, and at read2's batch start (expect the MISSING
reset). The writer list names the resetter read2 skips - then the fix fork: fw's reset
path blocked by something model-owed vs model staging owing the reset. Also confirm
[$791a] mode selection for both reads (same mode?).

## cont.100 — [$7A62] holds read2's CORRECT count (4), pre-staged by the fw at read1's completion (run238)

One write all window: **[$7a62] <- 0004 at pc $73ac @7.96446 - read1's completion era
(0.6ms before its c-stamp). The fw pre-stages the NEXT probe pass's count (the 8/4/2
ladder: after count-8, stage 4).** No later writes: no reset is missing, no increments
followed (the want-advance's 0005/0006 did not route through $7a62). Read2's correct
count EXISTS, staged by the fw itself, in the cell the increment-mode branch reads.
CONSEQUENCE: [$7430]'s 0044 did not come from [$7a62] (0x44 != 0x04) - the $84c6 loader
either ran at a different time/source or isn't the loader for this batch either. The
close comparison's true operands remain the open decode - but the frontier is now:
**the fw staged 4; SOMETHING reads 68. Find the read path between $7a62/staging and the
[$7430] limit** - candidates: (a) the $84c6/$8612 mode paths' actual execution (tap them:
which fires for read2, what D0 carries); (b) a byte/word aliasing on the 0044 (00|44 as
two byte fields?); (c) [$7430]'s remaining writers ($7e86 D0 provenance at the FIRST
write of read2's era - the initial 0044 may itself be $7e86's rewrite from a garbled D0
before the first JUDGE sample). NEXT TAPE: write-trace [$7430] itself (all writers,
pc+value, 7.96-8.01) - the 0044's writer pc closes the question the same way $73ac
closed [$7a62]'s.

## cont.101 — THE 0044 CONVICTED (run239): read1's close-era walk leftover; read2's BATCH-INIT NEVER RUNS

LIM7430 tape: [$7430] <- 0044 at pc $7e8a (the walk's $7e86 rewrite) at 7.96349 and
7.96424 - READ1'S CLOSE ERA, before read2's re-accept (7.977). Then nothing until 8.00127:
<- 0002 (right after read2's first match at 8.00123) - read2's walk chaining from the
STALE SEED (limit 2, 3, 4, 5... the mirror pattern on the earlier tape).
**The complete mechanism: read2's BATCH-START INIT (the $8480-$84c6 block: index the
request, load [$7430] from the $7696 table or [$7a62]) NEVER EXECUTES** - run237 zero
$849a visits + run239's no-load-before-first-walk agree. Read2 walks on read1's leftovers;
the fw's own correctly-pre-staged count 4 ([$7a62], cont.100) is never consulted; the
close condition (whatever compares against the batch-init's outputs) can never fire.
Dave's stale-leftover lean lands - as a MISSING INIT, not a missing reset.
NEXT (the last link): WHO invokes $8480/$846a-era batch-init for read1, and why not for
read2. $846a's entry writes SR (#$2000) and reads F000 (drive status) then btst #2 -
ISR-flavored, possibly the FIRST-CAPTURE handler's init leg or an op-list op's entry.
Tape: opcode tap at $846a + $8480 (all visits 6.40-8.06, pc-stack context) - read1's
visits mark the init's trigger; read2's absence + the trigger's identity names what the
model owes (the same reactive-event species as every keeper: likely an event read2's era
doesn't get because our completion/strobe timing differs from real hw at one point).

## cont.102 — the $84xx init paths are DECOYS (run240: zero visits for EITHER read); re-aim at the CLOSE SITE $8a48

BATCHINIT tape empty across read1's entire batch AND read2's era: $846a/$8480 never
execute. The limit-provenance hunt (three probes: $849a table, $84c6/[$7a62], $8480 init)
was chasing paths our flow never takes; [$7430]/[$7428] are the walk's own bookkeeping
cells and 0044 was read1's own late-chain value, not a loaded limit. THE CLOSE DECISION
LIVES ELSEWHERE - and the direct question was always: WHO ISSUES THE CLOSE. Run236 proved
read1's close happens (bit15 sustained low 20ms+ from ~7.955); cont.44 named the writer:
the $8a48 batch-close (andi #$67ff on E802, the sustained disarm).
NEXT (run241): opcode tap at $8a48, all visits 7.94-8.06 + return stack + the counter
cells ([$79a4], [$7a0c], [$741c]) - read1's visit(s) name the close's trigger context
directly; read2's absence + that trigger = the missing event, no more plumbing inference.

## cont.103 — $8a48 NEVER RUNS (run242, full window): there is NO explicit close command; read1's "close" = THE WALK GOING QUIET

CLOSESITE empty across 6.40-8.20: the cont.44 "batch-close $8a48 sustained disarm" is not
part of this flow either (cont.44's model-side capdone keyed on an E802 pattern that came
from somewhere else). **Read1's 20ms bit15-low = the walk's last dip simply never
re-arming - the walk STOPPED WALKING; completion posted via the ladder ($16c6 -> c-stamp
-> $ba). There is no close command to miss: read2's bug is the walk never deciding to
STOP.** The stop-decision = the $7e20-$7ea4 branch structure (already half-decoded,
cont.98): every exit avoiding the $7d4a re-arm is a stop candidate - the want-advance's
$7e4c -> $7ee0 exit, the $7e60+/$aa region, the $7e2a SPT*3-vs-[$79a4] horizon.
NEXT (static first): decode $7ee0 and $7ea4 (the non-re-arm exits) - which is "batch
done -> post" and what condition selects it; then ONE tape: [$79a4] + the $7e2a/$7e58/
$7e6e branch operands at read1's LAST arm (~7.94?) vs read2's grind - the operand that
differs is the stop condition read2 never meets. (Note read1 timing puzzle for the
decode: first hunt-arm 7.9638, c-stamp 7.9650 - 1.2ms apart; its 8-sector collection
cannot fit there physically; the walk's quiet point predates 7.955 - read1's real
collection/stop era needs locating on the next tape: widen B15EDGE-style arm tracking to
6.41-8.0 to find read1's LAST re-arm.)

## cont.104 — $7EE0 DECODED: the stop = the segment queue running dry ($32ac); the $7696 table returns, consulted per-segment (statics)

$7ee0: link frame -> **bsr $32ac (the NEXT-SEGMENT QUERY: returns a flag at frame-6, a
segment index at frame-8)** -> flag==0 -> unlk, bra $7ed8 = THE QUIET PATH (the stop);
flag!=0 -> D1=index -> [$7424]<-D1 -> D1*6 -> **$7696 TABLE lookup** (word0 asr#1 at
$7f0a-0e...) and the walk continues on the next segment. The table wasn't a decoy - it's
consulted HERE, per segment, not at batch init. THE STOP CONDITION = $32ac answering
"no more segments."
**The unified picture: read1 stops when its segment queue runs dry; read2 grinds because
the queue keeps answering MORE - the segment queue (fed to $32ac; [$74b8]-family) still
holds read1's count-8 two-side segments. Dave's ghost, at the queue level: the 0x44
boundaries live in read1's leftover segment entries.** The want-vs-limit surface behavior
(0044, chaining) falls out of walking those stale segments.
NEXT: (a) static - $32ac (the queue it walks: head/tail cells, entry format, and WHO
ENQUEUES segments per command - the enqueue site is the thing read2's accept must drive);
(b) read1's true quiet-point (its last re-arm, arm-trace 6.41-8.0) for the differential's
valid sample moment (Dave: sample the DECISION, not the post - read1's collection cannot
fit 7.9638-7.9650, its real era is earlier); (c) then the one-tape differential: $32ac's
queue state at read1's dry-out vs read2's era - whose segments are in it. Fix shape once
confirmed: the segment-queue rebuild for a back-to-back command - who owes it (fw path
blocked vs model staging) decided by the enqueue-site trace.

## cont.105 — $32AC DECODED: the queue head is [$74AC]; empty = stop; the ENQUEUER is the fe-stamp path (pre-crash lore connects)

$32ac: stamp [$7b0e]<-$80; UIB+$21 bit6 selects the mask (IPL7 vs +$400); **tst [$74AC]:
zero -> $3320 = the NO-MORE answer (the stop); else A2 = head entry, stamp entry+2 <- $80,
walk entry fields (+6, +4...).** THE SEGMENT QUEUE HEAD = [$74AC] - and the pre-crash
notes already mapped its feeder: "the $fe-stamp -> $74ac/$74ae -> DMA -> CMDDONE" (the
2026-07-13 session's tail lore). THE ENQUEUER IS THE FE-STAMP PATH.
**The fork this sharpens (Dave's tell, upgraded):** read2's never-empty queue is either
(a) INHERITED - read1's segments still enqueued at read2's start - or (b) SELF-FED - the
grind's own want-staking over the fe region re-enqueues a segment per stake (producer-
consumer with itself: each fe consumed stakes a new want AND a new queue entry), a
self-sustaining loop that would grind forever regardless of inheritance. The tape hints
(b) is at least active: the walk overwrites fe with sector IDs as it goes (run236). Both
arms are one differential apart: sample [$74ac] + the entry chain at (1) read1's true
dry-out (locate its last re-arm first - the timing-hole discipline), (2) read2's batch
start (inherited entries? whose?), (3) read2's grind era (does the chain GROW?). A growing
chain = self-fed; a static stale chain = inherited; both = both.
NEXT: that three-point queue-state tape + the enqueue-site pc (find the $74ac WRITERS
statically first - the enqueue instruction names the producer; then does read2's accept/
flow reach a FLUSH site at all). Fix shape holds: flush-and-rebuild on accept (who owes it
= the enqueue/flush-site trace).

## cont.106 — THE QUEUE ANATOMY (statics): pool $74C4 (8-byte entries, cap ~50), init $a90, release $3270, list-op $352e; enqueue sites all OUTSIDE the walk

- **$a84-$a90 = THE INITIALIZER**: computes [$7996]/[$7998] (a #$32=50 clamp - pool cap),
  points [$74AC] at the pool base $74C4, zeroes entry fields. A SETUP, not a per-command
  flush - find its caller(s): power-on only, or per-INIT/per-command?
- **$3270-$328c = THE RELEASE** (consumer side): entry = pool + [$7926]*8, clear entry+2,
  lea $74ac -> bsr $352e (the UNLINK list-op). The queue drains here, entry by entry.
  Adjacent: $328e = the FWPUMP ARM (the known "queues a disk op" site) - same module, the
  fw's central event/op queue.
- **Candidate ENQUEUE call-sites (lea $74ac + list-op):** $1250, $1544, $1970, $43bc,
  $5518, $5910, $5c2e, $6b4a - ALL in dispatcher/completion/op-handler territory; NONE in
  the walk's own $7xxx-$8xxx region. Dave's locational tell leans INHERITED/dispatcher-fed
  (the walk doesn't obviously enqueue in-line) - but bsr chains from the walk could reach
  them; classification needs one context read per site (which are enqueue vs dequeue-scan,
  and which the walk's call graph reaches).
NEXT: (1) classify the 8 sites (enqueue vs scan; walk-reachable?); (2) find $a90's
callers (the initializer = the rebuild the fix may need to drive); (3) then the 3-point
queue tape (read1 dry-out at its TRUE quiet-point / read2 start / read2 grind - chain
growing vs static) with entries decoded (8-byte format: +2 flag/state, +4/+6 fields per
$32ac's walk). The fork stands: inherited -> flush/rebuild at accept; self-fed -> stop
the walk's producer. The site classification decides before the tape runs.

## cont.107 — THE REBUILD DIFFERENTIAL ANSWERED (run243): one rebuild at read1's launch, ZERO for read2; the rebuild lives under the OP-EXECUTION chain

QREBUILD: **exactly ONE visit all window - @7.96234 (read1's launch era, between op-24
select 7.96218 and op-42 7.96357), stack carries $15bc (the $15ba op-walker's return) -
the queue rebuild is invoked UNDER AN OP HANDLER during op-list execution.** Sizing input
[$7966]=0080 (128 bytes -> 1 entry by the /0x80 divide; the per-entry unit semantics need
one more read - entry-per-128B-transfer-unit is the shape). Head was at pool base
($74ac=74c4) going in. **READ2 NEVER GETS A REBUILD.**
**This closes the arc's OLDEST open observation:** read2's era dispatches no device ops
(the $355c-only OPDISP tape, cont.74/90 era) - its continuation re-enters via the ladder's
short path (re-4 -> 6 -> queue -> bsr $6788 seek -> stamp 8) WITHOUT the full op-list
execution of a first pass - and the queue rebuild rides the op list. No ops -> no rebuild
-> stale queue -> $32ac never answers dry -> no quiet -> no c-stamp -> timeout -> park.
NEXT (fix-shaping): (a) identify the exact op whose handler calls $a4c (the $15ba-walker
dispatch around 7.96234 - correlate OPDISP tape; likely the read op $355c's setup leg or
op-$58's) and [$7966]'s loader (the 0080's provenance - per-op transfer sizing); (b) the
REAL question for the fix: on real hw does the continuation re-run that op (i.e., the
monitor's retry-probe protocol expects a FULL re-dispatch, and our model's continuation
path shortcuts it - find what event makes the fw re-run the op list on a continuation:
suspect the completion/DONE handshake the continuation skips because cont.77's accept-
posts let the RE-ACCEPT ride the old op list), or does the fw legitimately reuse the
queue and something else drains it. Dave's semantics-matching caution governs: copy the
working first-pass flow, don't invent a flush.

## cont.108 — CORRECTIONS + THE GUARD: the continuation DOES dispatch ops; the $1922 rebuild is [$791a]-gated; read1's actual rebuild caller is UNIDENTIFIED (statics + run232 re-read)

**Correction 1 (windowing artifact, again):** run232's OPDISP tape shows read2's
continuation dispatching a FULL op list at 7.98912-7.99019 (651c, 6788 seek, a392 SELECT,
7346, 3182, 308c, a356, 6ed2, 6bc2) - the ancient "$355c-only" reading came from run216's
capped/deduped probe. So read2's op-24 RAN and still didn't rebuild: **the rebuild is
CONDITIONAL inside the select chain.**
**The guard at $191c-$1922:** `tst.w $791a; beq skip; bsr $a4c` - the $1922 rebuild call
is gated on [$791A] (the accept-staged UIB[$20]&$100 cell, cont.99b). BUT 8c27 has bit8
CLEAR - both accepts staged [$791a]=0 - so the $1922 path should skip for BOTH reads.
**Correction 2: read1's 7.96234 rebuild caller is NOT identified** - run243's "return
stack" words are movem'd register saves, not a return address (none of the four grep'd
callers return to $2010). There is likely a FIFTH caller (register-indirect jsr, or a
bsr my grep pattern missed).
NEXT (the two-instruction finish line): (a) identify read1's actual rebuild caller - tap
each candidate caller site individually (or log the true return: capture SP at $a4c entry
+8/+10 past the movem) - and read ITS guard; (b) the differential is then guard-condition
vs guard-condition: what read1's caller-condition satisfied at 7.96234 that read2's
op-24 chain didn't at 7.9893. The fix = make read2's continuation satisfy the same
condition the fw's own first pass does (semantics-matched, Dave's constraint) - likely
one flag the model's accept/completion staging sets differently on the encore.

## cont.109 — THE TRUE CALLER IS $A3BE: rebuild-on-SIZE-CHANGE; same-size reuse is BY DESIGN; the real bug = THE QUEUE NEVER DRAINS (run244)

QREBUILD (true-return, past the movem, windows-from-zero): boot rebuild via $8e6
(ret $8ea, [$7966]=0200), **read1's rebuild via $A3BE (ret $a3c2, [$7966]=0080) at
7.96234 - and $a3be sits directly after the [$7966] LOADER ($a3ae cmp new-size vs old,
$a3b4 store): THE GUARD IS SIZE-CHANGE.** Read2 computes the same 0x80 -> no rebuild,
QUEUE REUSED - the fw's own design: a same-size encore expects the queue ALREADY DRAINED
by the prior command's consumption. ($1922 QCALLER lines = PREFETCH GHOSTS - guard-failed
branch fetches, no QREBUILD follows; third prefetch trap this arc, caught by doctrine.)
**THE REAL BUG, final form: the queue never drains.** The chain the pre-crash lore mapped
- "fe-stamp -> $74ac enqueue -> DMA -> CMDDONE" - has its RELEASE leg ($3270: entry
cleared + unlinked, keyed via [$7926]) driven by the per-entry DMA/CMDDONE completion.
Read1's era released at least its own entries... read2's era enqueues (its collection's
fe-stamps) but the per-entry completion chain never fires the releases - producer without
consumer - so [$74ac] never reaches zero, $32ac never answers dry, no quiet, no $ba.
THE MODEL OWES THE PER-ENTRY CONSUMPTION EVENT (the DMA/CMDDONE leg) - Dave's species,
one final time, now at the exact wire: what completes a $74c4 queue entry on real hw
(the gate-array DMA-done per 128B transfer unit?) and where our model drops it in the
continuation era (read1's first pass got them - find ITS release driver on tape: $3270
visits 6.41-7.97 vs their absence 7.99-8.18).
NEXT (one tape): tap $3270 (the release) windows-from-zero - read1's releases (when, how
many, driven by what: correlate with DISKOP/IRQ4/CMDDONE events) vs read2's era (expected
zero). The differential names the consumption event the model owes the encore. Fix =
deliver that event per-entry as the first pass gets it (semantics-matched; NOT a hand
flush - the fw releases its own entries when the hardware completes them).

## cont.110 — $3270 NEVER RUNS EITHER (run245, doctrine-proof): fifth decoy; FLIP THE METHOD - tape $32ac's ANSWERS

QRELEASE: zero lines, windows-from-zero, site-tap - a GENUINE never-runs (unlike the
capped artifacts): the $3270 release path is not part of this flow, even for read1's
healthy pass. So the queue does not drain by per-entry release-events; read1's dry-out
happened some other way (candidates: $32ac's own walk consumes/advances the chain
in-place; or the chain's link-end IS the dry answer and read2's chain differs in
CONTENT not length-management).
**Method flip after five statically-guessed never-runs sites ($849a, $846a/$8480, $8a48,
$1922-as-caller, $3270): stop inferring from statics - tape the decision's ANSWERS.**
$32ac's no-more exit = $3320 (the tst.w $74ac beq target). NEXT (one tape): tap $3320
(the DRY answer) and the yes-path exit, windows-from-zero: (a) read1's quiet point =
$3320 firing (when, and [$74ac]/chain state at that moment); (b) read2's era = $3320
never firing + the yes-path's handed-out ENTRY (address + 8 bytes) per call - what entry
keeps answering MORE and what its fields say (whose segment, what state). The entry
contents at read2's grind vs read1's dry are the final differential - no more site
guessing; read the machine's own answers.

## cont.111 — THE SEGMENT-ANSWER TAPE (run247): no dry answer EXISTS; read1 stops by COMPLETION-BETWEEN-ASKS; [$74ac] is a pool CURSOR; the full vocabulary lands

**Read1's era: exactly TWO asks** (idx=0 @7.96354 ledger all-c0 pristine; idx=1 @7.96429
ledger f0+c0s - $f0 = the staked-want mark, cont.78's lore vocabulary confirmed), both
answered MORE (flag=1). **No flag=0 exists on the tape - read1 NEVER receives a dry
answer. Its stop = THE COMPLETION FIRING BETWEEN ASKS: the c-stamp at 7.96505, 0.8ms
after its second segment - the ask-loop is exited FROM ABOVE by count-satisfied
completion, not from below by exhaustion.** (cont.104's "stop = queue dry" model retired;
the $32ac answer is always MORE while the pool lasts.)
- **[$74ac] = a CURSOR through the pool** (74cc -> 74d4 -> ... +8 per answer): entries
  handed out consecutively, 50 deep. Read2's asks march it 74dc -> 754c, one ask per
  consumed sector (12.5ms cadence, 1:1 with matches), each answer staking the next want
  (the ledger's 05/06/07... overwrites = per-segment want-stakes, not corruption!).
- Ledger vocabulary complete: c0=clean, f0/ff=staked want, fe=terminator, sector-ID=
  consumed-recorded. Read2's grind is WELL-FORMED segment processing - of segments it
  should never have asked for.
**THE FINAL DIFFERENTIAL, exact: read1's completion check fired after segment 2 (its
count: 2x128B); read2's never fired after segment 4 (4x128B, done by 8.05).** The event
read1's stop rides on = the count-satisfied COMPLETION CHECK between asks (the $17fe
c-stamp driver at 7.96505 - its caller context vs read2's silent post-segment-4 moment
at ~8.051). NEXT (one tape): $17fe's stamp with caller stack + the count/consumed cells
([$79a4], [$7a0c], [$7966], segments-consumed) at read1's 7.96505 vs read2's 8.051 era -
the operand that satisfies read1's check and not read2's is THE BUG'S FINAL CELL.

## cont.112 — THE EVENT, NAMED (run248 + SECMAP): completion rides the $9318 f0-STAKE; read2's launch never stakes one; its walk eats read1's ff-LOOKAHEAD forever

**CSTAMP tape:** read1's c-stamp @7.96505, ret=$7d90 (THE WALK'S OWN completion branch,
same one the INITs use via $7d8e/$7d92) - and every counter cell ([$79a4]=0100,
[$7a0c]=0003, [$79a2]=0100) IDENTICAL between read1's completion instant and read2's
entire grind: the check keys on NONE of them. The deciding operand is THE STAKE CLASS in
the ledger: $ff -> the $7e50 cmpi.b #$ff -> continue; $f0 -> falls to the want-match ->
the completion branch.
**SECMAP names all three stakers, all at READ1'S LAUNCH:** $9318 stakes f0 (slot 0, the
command's OWN completion-class want, 7.96358); $6fe6 stakes ff x4 (slots 1-4, 7.96482 -
LOOKAHEAD, continue-class); $7068 stakes fe (terminators, slots 5+). Read1 completes on
its f0 (7.96505) leaving the lookahead unconsumed. **Read2's launch stakes NOTHING (no f0
in its entire era; the model's only ledger writer is env-gated off - all authentic fw) -
its walk consumes read1's leftover ff-lookahead, each correctly answering CONTINUE per
its own vocabulary, forever.** The grind, the no-quiet, the timeout, the recovery park -
all downstream of one missing $9318 invocation at the continuation's launch.
**THE FINAL QUESTION (one caller-trace, doctrine-windowed): why $9318 runs at read1's
launch and not at read2's.** $9318 sits in the read-setup/capture-request region
($9384/$9400/$9884 family). Candidates by the arc's pattern: the launch leg the
continuation skips (its short ladder path re-4->6->queue->seek->8 vs the first pass's
full $5fc0-builder chain), or a guard satisfied only on a size/geometry change (like
$a3be's rebuild). Trace $9318's caller at 7.96358 (true-return, past any movem) + its
guard's operands, then read2's equivalent moment (7.9889 launch era). FIX = make the
continuation's launch stake its own f0 the way the first pass does - the fw's own act,
model-owed only if an event we withhold gates the $9318 path.

## cont.113 — THE STAKE'S TRUE MEANING (statics + run248): f0 = DATA LANDED, staked by the $92f6 DATA-COMPLETION handler; the missing event = the per-sector DATA-phase completion (IRQ5 -> $92b4/$92f6)

$9312's context: it is INSIDE $92f6 - the data-capture COMPLETION handler (the cont.38-era
"$92b4 consumes: $7b18=1, re-arms $8a00" family; $92e0-$92f0's andi/ori $8a00 on E802 sits
right above). `move.b #$f0, ($7654 + [want])` = **f0 IS THE "DATA COLLECTED" MARK, staked
per-sector when the sector's DATA record lands.** Vocabulary final: c0 clean, ff want-
staked (data needed), f0 data-landed (the walk's want-match on f0 = sector done ->
completion branch), fe terminator.
So the completion chain per sector: id-match -> data-phase -> DATA COMPLETION (IRQ5 ->
$92b4/$92f6) -> f0 -> the walk's next match sees f0 -> count/complete. **Read1's data
completions happened (its f0 at 7.96358); read2's NEVER do - its ID hunts succeed forever
but the data phase never completes, no f0 is ever staked, the walk never sees done.**
run248 doctrine notes: DATASTAGE fires ZERO times for EITHER read (the line-578 comment
already knew: this regime streams sector data via E000/DMA, not the capture-stager) and
SYNCSEL logs zero flips - the ID->DATA mode transition itself needs its own tape.
**THE FINAL QUESTION, one tape: what delivers read1's IRQ5 data-phase completion (the
$92f6 entry at ~7.9635) and why read2's sectors never get one.** Candidates: the pump's
completed-branch IRQ5 (data-typed capture at an irq5 mark - requires m_idcap_id_typed
FALSE + pend: the $22f-prime vocabulary), or the free-strobe irq5 path, or an E800/DMA-
completion leg. Tap $92f6 entry (windows-from-zero, true-return) + the pump's data-
completion branch state at read1's 7.9635 vs read2's matches (8.001+). The fix delivers
read2's sectors the same data-completion read1's get - the hardware's own signal, the
LLE mandate's species, one final level down.

## cont.114 — READ1'S REAL LEG CAUGHT (run249): ONE f0, staked at LAUNCH (7.96358), synchronously, before any data could exist; the per-sector-data-completion model dies too

F0LEG: **exactly one entry all run - @7.96358, sp=7d6e (deep straight-line chain, NOT an
interrupt fork: vec7300=0000 vec7304=0000 both UNINSTALLED), idcap=0, e802=4ad7 (bit15
disarmed), window closed, want=0.** 0.2ms after the first bootstrap strobes (FREESTROBE
irq5@7.96250/irq6@7.96337), 2ms after motor-ready - NO sector data existed yet. **Read1's
ledger-completion = consuming ONE launch-era pre-staked f0 (want 0); its actual 8 sectors
ride the E000/DMA channel separately. The ledger walk is the VERIFY/position protocol,
not the data path** - cont.113's per-sector-data-completion model dies on read1's own
tape (Dave's red-herring warning was exact: the assumed leg was wrong, and so was the
next assumption).
**The frontier, final form of this stretch: what synchronous launch-era chain reaches
$92f6 and stakes want-0's f0** - prime suspect: the FIRST free strobe burst's hard-handler
fork (the cont.82 gate's own deliveries at 7.9625-7.9637; the fw's IRQ5/IRQ6 hard handlers
run even with soft-vectors empty - their straight-line legs), or the launch choreography's
own id-verify concluding. And symmetrically: read2's re-launch (7.977-7.989, which DID
get strobes and DID re-arm) never reaches it. NEXT TAPE (fresh session): pc-chain into
$92f6 - tap the IRQ5/IRQ6 hard handlers ($26a8/$26ae) + $92b4 with the same columns,
windows-from-zero, and walk the 7.9635 chain link by link; then read2's 7.989 launch era
for where the same chain diverges. The fix still obeys cont.98's constraint - whatever
event lets read1's launch stake its f0, read2's launch must receive it the same way.

## cont.115 — READ1'S CHAIN, WALKED: the f0-stake is gated on [$742C] ("final-delivery mode"); the walk's own exits set it; the launch-era setter is the last link (statics)

Link-by-link from $92f6 backward:
- $92f6 is entered from THREE conditional branches, all in the capture-completion handler
  family ($925a / $92a8 / $92b4 - the data-fork included), and all through ONE GATE:
  **`tst.w $742c; bne $92f6`**. [$742c]==0 -> the pull/re-arm path ($92ce, E000 $2ff
  data-pull etc.); [$742c]!=0 -> STAKE THE f0 (delivery complete for this want).
- **[$742C] = "final-delivery mode"** (run154's relay lore: $741c=0 -> $7968=0 -> $742c=1
  -> $81b4). Setters: **$7ea4/$7eb2 = THE WALK'S OWN non-re-arm exits** (routed by
  $7e66's `tst [$79b6]; bne $7ea4` and the $7e6e $aa-check - the "this want is the final
  one" arms), plus launch-era candidates **$7162 (the $7106-stocker leg, cont.79's
  door->stock), $7bf6, $95da**. Clears: $7ca8, $7e42 (the want-advance - every mid-batch
  advance re-clears it), $7e90.
- So the per-want protocol: match a non-final want -> advance (clr $742c) -> pull -> next.
  Match the FINAL want ($79b6/$aa route) -> set $742c -> the NEXT capture-completion
  stakes f0 -> the walk's next match sees f0 -> completion branch -> c-stamp -> $ba.
- Read1's launch staked its f0 at 7.96358 with [$742c] ALREADY SET (want 0, before any
  matching) - a launch-era setter ($7162/$7bf6/$95da class) armed final-delivery mode
  during the launch choreography (read1's batch effectively completes on its first
  verify-capture - consistent with its 2-segment/instant profile).
RUN250 (in flight): FLAGWR biography of [$742c] + [$79b6], windows-from-zero - read1's
setter pc (the launch link), read2's era (never set? cleared and never re-set by the
$79b6/$aa route?). The divergence = the withheld confirmation's exact address.

## cont.116 — THE DIVERGENCE ON TAPE (run250): read2 ARMED final-delivery FOUR TIMES; the completion handler never came; the withheld event is ITS INVOCATION

FLAGWR biography:
- **Read1: $742c<-1 at 7.96349/7.96424 (pc $7eb8, the walk's final-want arm) -> the
  completion handler consumed it 0.09ms later ($92f6/f0 at 7.96358).** The protocol works.
- **Read2: THE SAME ARM FIRED FOR ALL FOUR SECTORS (8.00127/8.01379/8.02627/8.03878, same
  pc $7eb8) - final-delivery mode SET, the f0-stake gate OPEN for 50ms - and NO capture-
  completion handler EVER ran** (zero $92f6 entries in the era; the gate stood open,
  nobody walked through). At 8.05011 ($82e2) [$79b6] cleared - the wants exhausted - and
  every later match took the $7e96 CLEAR arm: the fw stopped arming and settled into the
  lookahead grind. The park followed downstream.
**THE WITHHELD EVENT, FINAL LINK: the invocation of the capture-completion handler family
($925a/$92a8/$92b4) - i.e., the DATA-CLASS capture completion.** Read1's launch cycle
receives it within 90us of arming; read2's per-sector cycle NEVER does - and run236's
edge tape already shows why in outline: read2's cycle re-arms E802 ID-TYPED every sector
(the $891a/$22f-primed hunt arm), so the model's capture engine only ever completes
ID-typed captures (-> the walk), never a DATA-typed one (-> $92b4/$92f6). Read1's launch
chain must arm (or receive) a DATA-class completion through a leg read2's cycle doesn't
use.
NEXT (one tape): the ARM-TYPE trace - every E802 bit15 arm with its prime state (was the
$22f sync-select written before it? m_idcap_id_typed at arm time) + every capture
COMPLETION with its type, read1's 7.9634-7.9637 vs read2's armed windows (8.001-8.051).
Name read1's data-class arm/completion leg; then the fix: the model's capture engine must
complete read2's armed-final captures the same way - the LLE species at its true floor:
the DATA-completion the gate array owes an armed final-delivery capture.

## cont.117 — THE LATCH-AT-ARM FIX (KEEPER) + run252: data-typed completions now FIRE; the remaining gap is the IRQ5 delivery chain

**Run251 caught the model retyping an IN-FLIGHT capture:** the fw's per-sector choreography
arms BARE at $891a (data-typed, [$742c] set - the f0-delivery request!) then primes $22f +
re-arms at $89b6 44us later - and the model's live-read global type flag retyped the
flying capture to ID before its data mark (~1ms out). The real gate array latches type at
arm; the prime programs the NEXT arm. **FIX (KEEPER): m_idcap_armed_id latched on a FRESH
arm (re-arm dips while pending keep the armed type); mark_visible/completed/IAM terms use
the latched type.**
**Run252: the capture level is FIXED** - CMPLTYPE irq5 idtyp=0 completions fire at data
marks with 742c=0001 (7.999913, 8.001215, 8.012500 - the armed-final captures complete as
DATA class at last). But still no f0/no park-change: the completion raises IRQ5 (en=aad3
bit9 set) and the chain from the $26a8 hard handler to the $92xx family is not reaching
$92f6 in read2's era (read1's identical completion runs it synchronously). Run253 (in
flight): DLVCHAIN taps on $26a8/$925a/$92a8/$92b4 - where read2's raised IRQ5 dies
(not taken - IPL? taken but forked elsewhere - [[$7300]] uninstalled?).

## cont.118 — READ1'S DATA-CLASS LEG NAMED (runs 251/252 head-read): the SYNCHRONOUS STAGE-AT-OPEN path; the machine has TWO serving paths

The arm-type tape's head answers Dave's question and kills both suspects (fourth pillar,
again): **read1's entire first pass has ZERO E802 arm edges and ZERO pump completions**
(only boot-era arms at 0.4147 exist before 7.975; bit15 sat HIGH from boot through the
pass). Read1's records are served by **the SYNCHRONOUS STAGE-AT-OPEN leg** - the cont.31
convergence path in ch_w's E000 handler: on the fw's own $2ff window-open (NOBYPASS
branch), stage_next_id(m_d800<<1) stages the record ON THE SPOT, and the fw's launch code
- already executing - finds it and walks straight-line into $92f6 (sp=7d6e deep, no IRQ,
no arm edge: F0LEG's evidence exactly).
**TWO SERVING PATHS: launch-era = synchronous stage-at-open; continuation-era = the
rotational pump.** The cont.117 latch fix made the pump side complete data-class correctly
(run252 ✓) - the remaining gap is that pump completions deliver by IRQ5 into a chain that
differs from read1's synchronous one (run253 DLVCHAIN pending: not-taken vs forked-short).
Note the design smell for the eventual cleanup: the synchronous stage-at-open is an
HLE-flavored shortcut (instant staging at the open) that happens to serve launches; the
pump is the faithful rotational engine. The likely end-state per the LLE mandate: ONE
path (the pump) serving both eras, with the launch's expectations met by honest rotational
latency - but only after the delivery chain question is answered; change one thing at a
time.

## cont.119 — TWO REGIMES, FINAL ARCHITECTURE (run253 + statics): the toggler network vs the WALK regime; in the walk regime DATA IS PULLED, not captured

Run253: read2's IRQ5s ALL TAKEN ($26a8 within 4us of every data completion, sr=25xx) and
NEVER dispatch to $92xx - because the IRQ5-raise is a model-invented signal in this
regime. The ONE $92b4 entry (read1, 7.963573) is at **sr=2600 - inside an IRQ6/walk
context** - and statics close the loop: $92b4's only reference is the $2996 trampoline in
the $2970/$297e/$298c PING-PONG TOGGLER bank (cont.38c verbatim: bchg #0,$7950; phase0 ->
$89f2 ID, phase1 -> $92b4 DATA), installed via the [$7940]-pump/builder network ($3a34/
$6192 install $299a).
**THE TWO REGIMES:** (1) the SETUP/TOGGLER regime (the $3a30-builder's strobe network -
captures alternate ID/DATA via [$7950]); (2) the WALK regime (the accept installs OPH2
$7ba8 on the IRQ6 path; IRQ5 has no consumer) - **in the walk regime, sector DATA is not
captured: the walk PULLS it through the E000 window synchronously** (the $7f6c/$7fc6
data-pull legs - "issues $2af then $2ff then pulls", cont.38t; $2964's fork: tst $741c ->
$804c / $7f6c). Read1's f0 = its walk's pull chain reaching the $92xx stake ($92b4 at
sr=2600, 7us before the f0). The cont.117 latch fix remains correct for the capture
engine, but read2's missing f0 is on the PULL side, not the capture side.
**FINAL TAPE (next session): the pull-path differential.** Tap $7f6c, $7fc6, $804c (and
the $2964 fork) in read2's armed windows (8.000-8.052) vs read1's launch (7.9634-7.9637):
does read2's walk (a) never branch to the pull (the $741c/$742c fork state), or (b) pull
and starve (E000 mux closed - e000mux=0 all era; the pull's own $2ff should open it -
does the model's window-open serve the pull's reads with stream bytes at the right
rotational position?). The fix lands wherever the pull dies - most likely the model's
E000 window/stream serving the pull's synchronous reads (the LLE species, in the E000 mux
this time).

## cont.120 — THE PULL FORK ANSWERS: CONTROL FLOW (run254); read2 is locked in hunt mode ([$741c]=1) and never routes to the pull leg

Run254: **read1's launch reaches the pull - PULLPATH $7f6c @7.963597 with [$741c]=0000,
[$742c]=1, mux=0, pos=1884 - microseconds around its $92b4/f0. Read2's armed windows:
ZERO pull-path entries** (not $2964, not $7f6c/$7fc6/$804c - and $2964 itself logged zero
in BOTH eras, so $7f6c has another entry route; the fork family is $741c-gated: tst
$741c -> hunt leg ($804c-class) vs pull leg ($7f6c)). The E000-stream suspect never gets
questioned - READ2'S WALK NEVER ROUTES TO THE PULL AT ALL.
**The state tells the story: read1's pull ran BEFORE its first hunt-arm (741c=0 at
7.9636; the arm at 7.96380 set 741c=1). Read2's era holds [$741c]=1 permanently (armed/
hunting every sector) - the walk's routing never leaves hunt mode.** The missing
transition: "final want matched ([$742c] set) -> STOP HUNTING (clear/route past $741c)
-> take the pull -> $92xx -> f0." Read2 sets $742c four times and stays in hunt mode
through every one.
NEXT (fresh eyes): (a) find read1's ACTUAL entry route into $7f6c (true-return at $7f6c
entry - $2964 logged zero, another router exists) and the exact state it tests; (b) who
performs the hunt->pull transition on real hw for an in-flight command (a $741c clear
between the final-want match and the next walk pass? a router that tests $742c FIRST?);
(c) then the model question: whether an event we deliver (or withhold/mistime) drives
that transition - the arm/disarm cycle's interaction with [$741c] (the $7d62 walk re-arm
tail sets it; what clears it mid-command?). $9318's own block clears $741c AFTER staking
- the pull is upstream of that clear, so the circularity resolves somewhere else.
KEEPERS unchanged: latch-at-arm (cont.117), strobe gate, rotational completions.

## cont.121 — DAVE'S CLOSING DECODE: the countable invariant + the bifurcating tap (statics, run255 in flight)

**The pop is count-gated ($797e):** `tst [$7956] (remaining count); bne $7a9a (keep
serving current, DON'T pop); ... $79be: bsr $32ac (pop next want - reached ONLY at
count==0); $79c6: bne -> [$7968]=1 (want present, re-arm); else [$7968]=0 (no want,
FINAL).** So [$7968]=0 (final delivery) <=> count spent AND the pop returns empty - and
the $7e8a fork routes on it. Read2 spends its count-4 and the pop STILL RETURNS A WANT.
**$32ac's two want-sources:** [$74ac]!=0 -> $32d0 (unlink + return a queued segment);
empty -> $3320 (LEDGER SCAN over $7654: find a staked mark, re-stake $c0, return it).
**Launch ($7030-$70da) stakes the ledger from the IOPB count fields** (A6[1], +1 if
two-sided per A6[$12] bit1, minus [$7954]/[$7956]), lays the $fe terminator + **the $aa
END-MARKER (the $7e6e check's meaning, closed at last)**, clears [$7968]. Both reads
start drained; read2 RE-FILLS during the hunt.
**REDUCED INVARIANT: read2 exits correctly <=> at [$7956]==0, $32ac returns no-want.**
The extra want has exactly two possible sources = ONE measurement (the $32ac branch on
the count-zero pass):
- QUEUE branch ($32d0): the hunt queued capture segments PAST the count -> the model
  over-delivers ID captures beyond read2's count (real hw: the fw had stopped arming) ->
  fix: gate capture/segment delivery on the fw's E802 arm state (the $23f/motor-gate
  family, keeper species).
- LEDGER branch ($3320): an unconsumed staked mark remains because the want-retirer (the
  DATA completion $92f6: stakes f0, decrements owed [$79a8]) never fired -> fix: deliver
  read2's matched-sector DATA-phase completion the way read1's final sector gets it.
Correct-exit signature: at [$7956]==0, [$74ac]==0 and no staked ledger mark -> $32ac
empty -> [$7968]=0 -> $7e8a -> $7eb2 -> [$742c]=1 -> [$741c]=0 -> next completion passes
tst $742c -> f0 -> $ba. Tap set: $7e8a fork ($7968), $7956 writes, $32d0/$3320 branch +
slot, $74ac/$74ae pushes, $92f6 count, $79a8 writes. THE $32AC BRANCH IS THE WHOLE REPORT.

## cont.121b — LATCH v2 (run255 read): latch at EVERY arm write; v1's fresh-arm guard starved the hunt

Run255 under v1: read2's era went QUIET (zero SEGANSWER asks, zero serve-loop activity,
31 data-typed completions raising unconsumed IRQ5s) - the fw's dip-and-re-arm cycle
(bare rise -> $22f prime -> $89b6 second rise) is a DELIBERATE re-arm with the new type,
and v1's fresh-arm-only guard blocked the $89b6 ID re-latch: captures completed
data-class, stage_next_id stopped feeding the walk, the hunt starved. **v2: latch at
EVERY arm write (the operative capture is the last-armed one). The semantic keeper
stands: a PRIME ALONE (no arm write) cannot retype an in-flight capture** - v2 is
behaviorally the honest pre-fix baseline with clean latching semantics.
Also from run255's read1-era lines: the launch's serve loop ran its fork->QUEUE-pop
cycle TWICE (7.9635, 7.9642 - the two segments), count decrements at $7ec4
(0000 -> ffff -> fffe... sign convention pending), read2's count staged 0004 at 7.9648
(pc $6f62/$70a6 - the launch staker, matching Dave's $7030-$70da read).
Run256 (in flight): the bifurcating tap set on the v2/honest baseline - THE $32AC BRANCH
ON READ2'S COUNT-ZERO PASS IS THE WHOLE REPORT (QUEUE = over-delivered ID captures, gate
on E802 arm state; LEDGER = missing DATA completion, deliver it).

## cont.122 — THE BIFURCATION ANSWERS: QUEUE — with two complications the tape adds to the invariant (run256)

**The measurement (v2/honest baseline): read2's count decrements 4->3->2->1->0 cleanly
($7ec4, last at 8.038778). At the count-zero pass the pop serves FROM THE SEGMENT QUEUE
(POPSRC QUEUE, [$74ac]=74f4 - a fifth entry) and [$7968] flips to 1 -> re-arm -> grind.**
Dave's QUEUE arm selected.
**Complication 1: the fw NEVER DISARMS.** At/around count-zero the fw re-arms per its walk
cycle (8.03841/8.03846, 8.05091 - the $891a/$89b6 pair, ID-typed) with the count spent.
Sector-5's ID capture (8.038368) completed with bit15 legitimately HELD - the capture
delivery per se matches the arm state; "the firmware had stopped arming" does not appear
on this tape. The self-sustaining loop: capture -> walk -> serve -> pop(QUEUE) -> 7968=1
-> re-arm -> capture...
**Complication 2: the pop's queue-nonempty test may never fail structurally** - the $3306
writes are the pop's own head-advance through the pool (a cursor, never null), and
**read1 never pop-empties either** (run247: no flag=0 ever; read1 exits via COMPLETION-
BETWEEN-ASKS - the f0-consumption at 7.965 - not via pop-empty). The correct-exit
signature (pop empty -> 7968=0 -> final) is not read1's observed exit; [$7968]=0 in
read1's era came from launch's clear and was never re-set because read1's f0 completed
the command BEFORE any count-zero pop cycle could run.
**The reconciliation question for Dave: the invariant's correct-exit may be the f0 path
(complete before the pop matters), not the pop-empty path** - read1's two pops both
returned QUEUE wants too (7.9635/7.9642) and its exit came from the f0/c-stamp
short-circuiting the loop. If so, the QUEUE finding relocates: the fifth entry isn't the
bug - the missing f0 (still the pull/$742c/$741c transition of cont.120) is - and the
count-zero pop cycle is a path real commands never reach because they complete first.
Tapes attached: CNT7956, POPSRC, PUSH74AC ($3306 = pop's own advance), B15EDGE/ARMTYPE
at count-zero. All windows honest, caps unhit, liveness-gated (single park at 8.187-era
recovery as always).

## cont.124 — THE RING CLOSES (run258): the hunt's own re-arms clear the phase; the pop's QUEUE answer restarts the hunt; ONE question left — the queue's missing terminator

PHASE7950 tape: the phase is SET by the toggler bank ($29c8-family per capture) and
**CLEARED BY THE HUNT'S OWN RE-ARM CHOREOGRAPHY - pc $88d6 (the $88ac capture-arm) and
$8a3e (the $8a00 re-arm), 10-70us after every set.** Read1's launch flip survived 24us to
its DATA pass because its hunt hadn't started (first hunt-arm 7.96380 > its f0 7.96358);
read2's phase cannot survive 12.5ms under a running hunt - by the fw's own design (arming
resets the ping-pong to expect-ID).
**THE COMPLETE CAUSAL RING (all measured):** count-zero -> serve loop falls toward the
no-rearm exit -> pop ($79be/$32ac) -> QUEUE returns a 5th entry -> [$7968]=1 -> re-arm ->
phase cleared -> ID-only togglers -> no $92b4 -> no f0 -> ring repeats. **If the pop
returned EMPTY at count-zero: no re-arm -> the hunt stops -> the phase survives -> the
next toggler pass takes $92b4 -> f0 -> c -> $ba.** Dave's correct-exit signature is
read2's designed exit (cont.122's complication was about READ1's different, launch-era
exit; both are real exits).
**THE LAST QUESTION: why does the segment queue hold a valid 5th entry at count-zero.**
The launch builds the batch's queue (the $a4c/$70xx builders, count-sized); the pop's
cursor found 74f4 linked and valid. Either (a) the launch never terminates the chain
(entry-4's link left pointing into the pool - then the fix/finding is the launch-vs-model
staging of the chain end), or (b) something extends it per pop (the $34c4 tail-write
[$74ae]<-78c4 every pop - maintenance or extension?). NEXT (the true last tape): dump
entry-4's link field at launch-end and at the count-zero pop; if it changed, trace the
writer; if the pool is pre-chained circular from boot, decode the pop's end-test
($32d0-$3320 full walk) for the terminator it expects and never finds.

## cont.125 — THE ENTRY-FIELDS TAPE (run259): the pool is launch-built as one long chain; THE RE-APPEND FIRES ON EVERY POP (Dave's default-skip gate is being cleared mid-pop)

- POOLWR: entries 5-6 (74f4/74fc) built AT LAUNCH by pc $a9e-$aaa (the $a4c initializer's
  loop, 7.96243-4) - {index, 0, fwd-link, back-link} - the pool is pre-linked long (8+
  entries on tape; the 0x32 clamp suggests all 50). The serve clears +6 (back-link) at
  $3302; the $32d0 walk's validity tests are the +6/+4 fields.
- **THE KILLER (existing PUSH74AE tape reread against Dave's pop-tail decode): [$74AE]
  (the tail) <- 78c4 at pc $34c4 ON EVERY POP - the re-append path ($3474-$34c2), which
  Dave's decode shows gated on [$7b0e] bit7 CLEAR with $32ac setting $80 on entry
  (default skip), FIRES EVERY TIME.** Every popped entry is re-queued; head can never
  reach tail; the destructive pop's drain-to-zero ($3306 zeroing head+tail on the last)
  can never trigger; the queue is never empty; [$7968] never 0; the ring spins.
- The final cell: [$7B0E] bit7's mid-pop clearer (run260 in flight, write-tap). If the
  clearer is model code (a tap side-effect? a status write?), it's ours; if fw, the
  question is what state makes the fw recycle-vs-retire (a flag read1's exit never
  exercises - read1 exits via f0 pre-drain, so the re-append may be wrong for both reads
  with only read2's exit depending on the drain).

## cont.126 — THE GATE DECODED (run260 + statics): the re-append is UNCONDITIONAL under all taped state; the skip needs a >=$8000 write into [$7B0E] mid-pop that NOTHING makes — the suspected hardware retire-mark

$3474-$34c2 exact: **$347e `bclr #7, $7b0e` is a BYTE op on the HIGH byte; the $32ac
entry-stamp `move.w #$0080, $7b0e` leaves that byte 00** - the skip branch ($3484 bne
$34c8, taken when bit7 WAS set) can never fire from the stamp alone. The fall-through:
D0 <- [$7b0e] (the word 0080, read as an ENTRY INDEX), clear the cell, entry = pool +
idx*8 = 74c4 + 0x400 = **78C4 - the constant tail on every tape** - append it; head
(74c4+8n) reaches 78c4 after 128 pops = 1.6s of grind > the monitor's patience. The
GATE7B0E tape confirms per-pop: 0080 stamp ($32b2) -> bclr RMW 0000 ($3486-pc) -> $348a
clear ($3490-pc) - the fall-through path, every pop, both reads.
**The skip requires a >=$8000-class value in [$7B0E] at the bclr - deposited between the
stamp and the test - and NO tape ever shows one.** The fw stamps and tests the same cell
200us apart; firmware doesn't fight its own stamp: [$7B0E] is a MAILBOX, and on real
hardware the retire-vs-recycle mark (bit15/high-bit7 set = retire; an index = recycle
that entry) must arrive from outside the CPU - **the gate array's status deposit, a
hardware write the model never makes.** The species, at the true bottom: deliver the
retire-mark deposit at the right moment (per served segment? per capture completion?),
the skip fires, the re-append stops, the pop drains at entry 4, returns empty, [$7968]=0,
the hunt stops, the phase survives, $92b4, f0, c, $ba.
FOR DAVE'S ADJUDICATION: (a) is [$7b0e] plausibly a gate-array-visible cell (the $7b0e
region vs the hw-deposit map)? (b) what event on real hw writes the retire-mark - the
per-segment DMA-done? (c) read1's exit never needs the skip (f0 pre-drain), so the
missing deposit is invisible to the first pass - consistent with every keeper's shape.
Alternative reading to rule out: the disasm's move.w #$0080 might be a mis-disassembled
byte op (verify opcode bytes 31fc 0080 7b0e = move.w, confirmed word) or [$7b0e] holds a
deliberate always-recycle in SOME modes (then the terminator lives elsewhere and the
0080/idx-0x80/78c4 numerology is the fw's own bounded-spill design - the 1.6s bound).

## cont.127 — DAVE'S ADJUDICATION + THE FIX (KEEPER): [$7B0E] is the disk-side status mailbox; the withheld write is the per-segment DMA-done retire-mark

**Adjudicated on the sixth writer ($8814, the ARM side):** D2 = segment idx; [$7b0e]==0 ->
append now; else raise the STACKED SR to 7 (IRQ context) and DEFER by stashing the idx in
[$7b0e]. So [$7B0E] = a lock/handoff mailbox with three states: $00xx small idx = arm's
deferred append (pop's fall-through serves it - the $3486 handler's REAL purpose); $0080 =
the pop's idle stamp (its degenerate misfire appends pool[128]=78c4 - never meant to reach
the handler); **bit15 set = SEGMENT RETIRED -> the $34c8 skip = the drain.** No firmware
writer ever sets bit15 (the deferred idx is structurally small) - the consumer polls a
bit its every fw-side producer leaves clear = by construction waiting on a SECOND
producer = the gate array. Q2: the real event = per-segment DMA-completion (the model
streams payload at the IRQ5 mark but posts status only at whole-command descriptor-done
to the HOST side - the disk-side per-segment status post was the missing write). Q3
always-recycle ruled out (loses to its own supervisor's timeout; the fall-through is the
deferred-append handler, not a recycle primitive).
**THE FIX (cont.127, KEEPER): in the per-segment data-completion (the irq5 completed
branch, with the payload deposit), OR bit15 into [$7B0E].** Data modeled, acknowledge
withheld - the signature species; this delivers the acknowledge. Expected tape (run261):
the count-zero pop sees bit15 at $347e -> $34c8 -> queue drains at 4 -> pop empty ->
[$7968]=0 -> hunt stops -> phase survives -> $92b4 -> f0 -> c-stamp -> $ba -> NO timeout,
NO recovery INITs, NO park - and the monitor's probe ladder continues (reads 3+, the
track-advance toward cyl 1's ZMAGIC). Read1 stays green (its f0 is pre-drain).
Confirmation frame: [$7b0e] bit15 set at $347e, branch to $34c8 - the ring running
FORWARD for the first time.

## cont.128 — v3 MERGE LANDS MECHANICALLY; THE RING STILL SPINS: the serve's end-test is not head-meets-tail-simple (run263, for Dave's read)

- RETMERGE: 8080 at every pop-stamp (8.0013+); re-appends STOPPED (zero $34c4 tail-pushes
  after 7.9643 - the two that exist are READ1's OWN first pops, pre-carry, misfiring the
  degenerate append harmlessly). The retire-mark is visible to every $347e test and the
  $34c8 skip is being taken.
- **BUT the pops continue past count-zero unchanged** (QUEUE serves 74f4/74fc/7504...,
  [$7968]=1) - the retire governs the RE-APPEND only; the unlink-and-return at $32d0
  already served the next entry before the tail-handler runs. The drain is NOT simple
  head-meets-tail:
- Tail biography (run263): launch $ac2 tail<-764c (pool[49] - the chain terminates at the
  POOL end, not the staked count); the $354c writer (in the $352e list-op region) rewrites
  tail per-pop to the JUST-POPPED entry (74c4, 74cc... and 74ec at 8.1854 recovery-era) -
  semantics unclear (the unlink's own maintenance? a ring-buffer wrap?).
**FOR DAVE'S NEXT READ: the pop's actual END condition.** With re-appends stopped and the
retire-mark delivered, what should make $32ac return empty at the staked count? Candidates
now on tape: (a) the $32d0 walk's +6/+4 field tests against the served entries' cleared
back-links (the $3302 clears - does "empty" = next entry's +6 already cleared? our serves
clear +6 one entry behind the head); (b) the $354c tail rewrite = the REAL drain pointer
(tail chases head; empty = head==tail after N serves - but then the launch's 764c and the
per-pop rewrite need his semantics); (c) the retire-mark's consumer isn't the re-append
gate alone - the $34c8 exit path may set the drain state we haven't decoded ($34c8: move
D2,SR + rts-era - check what the SKIP path's caller does differently with the mailbox
result). Tapes: RETMERGE, PUSH74AE ($354c biography), POPSRC (unchanged ring), ENTRYF.
KEEPER status: the merge tap + m_seg_retired stay (mechanically correct, needed); the
missing piece is one more end-test decode.

## cont.129 — FULL STATIC ANALYSIS (no run): the pop is a doubly-linked-list SERVE with drain-at-last; the launch links ALL 50; the architectural question is WHAT $74AC'S LIST IS

**The pop walk $32d0-$331c, exact:** entries are {+0 idx, +2 status, +4 NEXT, +6 PREV}
(addresses, A0=0 absolute). $32da stamps status $80 ("serving"). Unlink by case:
middle (+6,+4 both set) -> link prev<->next; first (+6==0) -> next.prev<-0, HEAD<-next;
last (+4==0) -> prev.next<-0, TAIL<-prev; **ONLY entry (+6==+4==0) -> $3306: head+tail
<- 0.L = THE DRAIN.** Return D0 = idx, always, at $3318 -> the $3474 mailbox block. So
the drain fires only when the list is exhausted - the retire-mark cannot shorten it.
**The $34c8 skip path: `move D2,SR; rts` - NOTHING else.** The retire's only effect is
skipping the re-append. Confirmed insufficient alone: with the merge in, the list drains
by exhaustion after ~50 pops = 625ms, still losing to the ~200ms patience.
**The $352e list-op = a proper APPEND** (empty: head=tail=D4; else old-tail.next<-D4,
D4.prev<-old-tail, D4.next<-0, tail<-D4). Run263's $354c-pc tail-writes were APPENDS
(read1's pre-merge deferred-append misfires + one recovery-era). PUSH74AE's "tail<-popped
entry" misread RETRACTED - the tap caught $3548's tail<-D4 with pc at $354c.
**The builder tail ($aa2-$ac6):** links D0 entries; at end: last.next<-0, TAIL<-last
($abe), zero 8 words after the head-pair. Run263: launch tail = 764c = pool[49] -> the
launch LINKED ALL 50 (despite [$7966]=0080 = 1 segment at the $a3be call) - the count
computation ([$7966]/0x80, clamp $32) and the observed 50-link disagree: the link loop's
bound is NOT the segment count (likely fixed pool-size = a FREE-LIST init).
**THE ARCHITECTURAL QUESTION (Dave adjudicates before any dynamic):** is [$74AC]'s list
(a) the batch's WORK QUEUE (then the launch over-populates it - it should hold COUNT
entries, and the fix is why the model's inputs make the fw link 50), or (b) the FREE
POOL (then serving "free" entries as wants is the bug's surface, the WORK source is the
LEDGER-SCAN branch ($3320 - staked wants), and $32ac's queue-first order means the queue
must be EMPTY in steady state - emptied by... the arm's deferred-append consuming free
entries and the pop retiring them back? Then the launch pointing head at the full pool
is boot-init semantics and the per-batch state we mis-feed is elsewhere).
Candidate dynamics ONCE ADJUDICATED (not before): (a-arm) tap $a4c's entry registers
(the link-loop bound's source - one frame); (b-arm) read1's own pops popped 74c4/74cc
(free entries!) while its WANTS came via... re-read run247's read1 SEGANSWERs (flag=1
idx=0/1 - from WHICH branch? the POPSRC run256 tape says QUEUE for read1 too) - if
read1's two pops were ALSO queue-first free entries and its real wants rode the ledger,
the queue-first order itself needs the (b) reading. All tapes exist; no new run needed
for the first adjudication pass.

## cont.130 — RUN264 + STATICS: the byte-count lead dies ([$7966] = SECTOR SIZE, correct); cont.120's pure form fully assembled; ONE tap recommended

**Run264:** [$7966]'s writers = boot $8b4 (0200 = 512B boot-unit sector) and the rebuild
gate's own $a3b8 (0080 = 128 = FM sector size, from UIB[2:3] '80 00' in the UIBCOPY
image). BOTH reads present 0080 = the correct FM sector size. The pool sizing
($2c00/sector-size), the reserve split, and read2's rebuild-skip (same geometry) are all
CORRECT. Dave's outcome (i): back to cont.120 pure. (My "read1 inconsistency" dissolves:
read1 is a genuine 1-sector probe; [$7956]=4 staged at 7.9648 was read2's.)
**The pure form, statically assembled:**
- The DATA-SERVICE = $9602: scoped install of the $9884 record-parser onto BOTH soft
  vectors ([[$7300]] IRQ5 + [[$7304]] IRQ6; old vectors saved, $987e restores) - the
  capture-service scope for a sector's data phase.
- ONE walk-side call: $748e/$7492 (movem + bsr $9602), branch-target-only, reached from
  the $7402 routine's fork ($7438 bne / $743e beq).
- $7402's GATES ALL PASS for read2 on paper: [$791a]==0 ✓ (both accepts stage 0);
  UIB[$11] bit1 SET ✓ (image byte 0x97); node+26 != $0a ✓ (read2 serves at 8).
- $7402 is a dispatch-table target (not inline-called; not in the $63e attr stretch) -
  read1's launch chain very plausibly runs $7402 -> $748e -> $9602 -> $9884-parse -> ...
  -> $92f6 (the F0LEG deep straight-line stack fits).
**THE QUESTION IN FINAL STATIC FORM: what dispatches $7402 - for read1's launch, and why
never for read2's serve.** RECOMMENDED SINGLE TAP (catch-the-leg, post-statics): $7402
entry, windows-from-zero, with SP-window + [$71bc]/node+26 + [$7956] - read1's entry
context names the dispatcher; read2's absence against passing gates names the withheld
dispatch. All other machinery (pool, counts, sizes, gates) is now verified correct.

## cont.131 — THE STOP MACHINERY: [$7A0C] = the consecutive-VIRGIN-capture countdown; three virgins -> skip the phase-clear -> phase survives -> DATA pass -> f0 (statics + 30 runs of frozen 7a0c=0003)

- Run265: $7402 NEVER RUNS (either read; windows-from-zero) - the $7402->$9602 hypothesis
  dies; $9602's scoped install is not part of this flow (consistent with every vec=0000
  frame). Read1's f0 rides the TOGGLER phase-1 entry, period.
- **The $8a00-region arm sequence decodes as the batch-stop detector:**
  $8a02 beq -> skip; UIB[$20] bit14 SET -> clr [$7a0c] (countdown killed; a no-stop mode);
  bit14 CLEAR (our 8c27) -> the $8a14 VIRGIN TEST: OR of $7dac bytes 0-2 == $a1(each),
  +3 == $fe, (+4 == $ff per the cont.31 lore; $8a2a-$8a30 = the one unread instruction) ->
  **subq [$7A0C]; beq $8a42 = SKIP THE PHASE-CLEAR** -> fall into the $8a48 sustained
  batch-disarm -> the phase bit survives to the next toggler entry -> $92b4 -> f0 -> c.
  Non-virgin capture (real sector, C=00) -> $8a38 clr [$7950] -> keep hunting.
- **[$7A0C]=0003 frozen on EVERY tape line of read2's era = its captures are never
  virgin: it reads real recorded sectors, so the end-of-recorded-data stop never arms.**
  The probe-loop semantics this implies: the monitor's probe passes may be designed as
  "read from sector N until the recorded area ends (3 virgins)" on a PARTIALLY-RECORDED
  track - and our synthesized FM track has ALL 16 sectors recorded (build_serdes_stream
  fills the full track from the IMD), so the virgin run never comes.
**FOR DAVE: the design question this bottoms into.** The IMD's track-0 has 16 real
sectors (run234's sector map 1-16, all present). If the REAL boot floppy's track 0 is
also fully recorded, the virgin-stop can't be the real exit for these probes either -
then the remaining stop arms are the $8a02 beq's source-cell, UIB[$20] bit14 (a no-stop
mode read2 maybe SHOULD have staged: 8c27 is the UNIT's flags - is the PROBE's UIB
different?), or the wants-exhausted $7eb2 path setting state that reroutes $8a0e. If the
real track 0 has FEWER recorded sectors than our synthesis (the IMD records what WAS
readable; virgin sectors on real media = unformatted gaps the IMD can't represent as
absent), our full-track synthesis is the model divergence - the LLE answer would be
faithful virgin captures past the recorded extent. The one unread instruction $8a2a-$8a30
prints above this entry for the read.

## cont.132 — DAVE'S ADJUDICATION: fork B; NO virgin synthesis (IMD-refuted); read2's only exit = the $79A4 count path; the reload race is the frame (run266 in flight)

**The IMD settles fork A against the medium:** track 0 = FM 16x128, NO cylinder map ->
every ID's C = 0, not one $ff; sectors 7-8 recorded (VOL1/HDR1), the rest formatted-EMPTY
(IMD type 2, compressed) - **empty != virgin; both classes carry C=0; the model's
synthesis (C from the stream = 0) is FAITHFUL. Do not synthesize virgin captures** - it
would invent an erased region the real disk does not have.
**The consequence: the phase-survives->f0 route is the VIRGIN exit's property ONLY.** On
a fully-formatted track the sole reachable stop is the COUNT path - $8a3c: [$79A4]--
per real capture; $8a40 ==0 -> $8a42 STOP (through the phase-clear; no f0 from here) -
**and [$79A4] is RELOADED from [$79A2] on every want-match at $7e60** (the $7e3c/$7e60
`move.w $79a2,$79a4` sites, decoded back in cont.98!). Read2's grind re-matches faster
than the count drains -> never 0 -> no stop. Same root as the free-pool grind, seen from
the count side.
Run266 (in flight): CNT79A4 trajectory (decrement-$8a3c vs reload-$7e60 pcs) + UIBGATES
(the probe's UIB[$12] bit1 / UIB[$20] bit14 live at $89f2, vs the unit's 8c27 - a
model-fed UIB field would bypass or force no-stop). The decisive read: does [$79a4] ever
approach 0 between reloads, and what the reload cadence is.

## cont.133 — THE COUNT RACE MEASURED (run266): lockstep reload, exactly as Dave called; plus three new facts for the synthesis

- **Read2's grind: [$79A4] oscillates ff<->0100 in PERFECT LOCKSTEP** - every want-match
  reloads 0100 (pcs $7e14 and $7e66 - TWO reload sites, both $79a2-sourced), every real
  capture decrements once (pc $8a42). The count can never approach 0; and the reload
  VALUE [$79A2]=0100=256 means even unreloaded it exceeds the patience window. The
  count-stop is unreachable for read2 (as the virgin-stop already was - fork B).
- Between-commands era (7.975-8.000, no matches): the count DRAINS monotonically
  (ff->f7, decrementers $7da2 AND $8a42 - a second decrementer $7da2, the walk's own?).
- **Gate anomaly for Dave: the probe's UIB[$12]=0x40 -> bit1 CLEAR -> per the $89fa
  decode the whole virgin/count handler should be SKIPPED - yet the $8a42 decrements
  demonstrably run.** Either the $89fa sense inverts (bne vs beq), the [12] offset
  differs, or the count block has a second entry ($7da2's region?). One static.
- The synthesis this leaves (Dave's eyes, fresh): with virgin-stop unreachable (fully-
  formatted disk), count-stop unreachable (lockstep reload by design while matches
  continue), the fw's remaining exit signal is the WANTS-EXHAUSTED arm ($7eb2, fires 4x
  on tape) whose consumer needs the toggler phase - and the first four wants were served
  from the QUEUE branch (the free-pool entries) with [$7968]=0 keeping the ledger-scan's
  MANUFACTURE arm unconditional ($339a: 7968==0 -> $33ae mint). The [$7968]/cursor gate
  ($33a6 cmpa $7428 -> ble EMPTY) is the one decoded exit that stops the minting - its
  preconditions ([$7968]!=0 = re-arm state; found-slot behind cursor) vs read2's era
  state = the next question. All tapes: run266 CNT79A4/UIBGATES + the standing corpus.

## cont.134 — DAVE'S ADJUDICATION: (B), the alternator's starved DATA fork; fix = the data-record capture completion strobe, WIRE-CHECK FIRST (run267 in flight)

**The phase machinery grounded:** the toggler bank is a clean ID/DATA ALTERNATOR - every
capture handler bchg's [$7950] and forks on the OLD phase: 0 -> $89f2 (ID), 1 -> $925a/
$9284/$92b4 (DATA - the $742c consumers). **THE KILL: $89f2 clears the phase at $8a38,
undoing its own toggle - after any ID capture the phase is 0 again, the next capture is
ID again, the DATA fork is never taken. The only escape is the virgin path's skip-clear -
unreachable on this disk (fork B).** That is why the $742c consumer fired zero times
against four arms. The UIB[$12] bit1 anomaly DISSOLVES: bit1-clear's $8a02 beq targets
$8a38 INSIDE the handler (skipping only the virgin sub-test) - correct marking of read2
as a normal read; the decrements we measured are the bit1-clear branch itself.
**ROOT (the signature species at the floor): the model deposits the sector data payload
(cont.56's own note: "zero DATASTAGE all regime") but never raises the DATA-RECORD
capture completion.** Read2 receives only ID captures; the phase never reaches the DATA
fork; $92b4 never runs; $742c never consumed; f0 never staked. Data modeled, completion
strobe withheld - the strobes/IAM/free-run shape, one final level.
**THE FIX (directed): after depositing a served sector's payload, raise that record's
DATA-record capture completion (the data-AM event driving $92b4/$8690).** One per served
sector -> phase reaches the DATA fork -> consumes a $742c -> stakes f0 -> the measured
chain runs forward on a fully-formatted disk, no invented virgin region.
**WIRE-CHECK FIRST (run267, per Dave - don't feed the completion blind):** at the served-
sector instant, which handler sits on the IRQ vectors - the phase-forking toggler bank or
the unconditional $299a -> ID (which would swallow even a perfect DATA completion)?
Frame: [$7300].l/[$7304].l + [$7940]/[$7950]/[$742c] at each carry; then one confirmation
that a phase-1 capture reaches $92b4 with $742c set. Wire verified -> issue the strobe.

## cont.135 — THE STROBE LANDS: FOUR f0 STAKES IN READ2'S ERA (run268); the consumption lap is the last unlit segment (run269 widening in flight)

**Run267 wire-check: the wire was ARMED** - [[$7300]]=$29C0 and [[$7304]]=$298C (phase-
forking togglers on BOTH vectors; every earlier "vec=0000" frame was a WORD-read of a
long's zero high-word - one more byte/word doctrine entry), and at every served-sector
carry: phase=1, [$742c]=1. The alternator stood in its data window at the exact delivery
instant.
**Run268, the fix run: FOUR f0 STAKES** - 8.01252/8.02502/8.03751/8.05002, wants 1-4, one
per served sector - THE DATA FORK RUNS (cont.135 KEEPER: at the carry, raise IRQ6 per
en bit11 - the data-record capture completion). The ledger reaches `c0 f0 f0 f0 f0 fe...`
and the walk REWINDS to want=1 with limit=5 - re-walking to consume/verify the four
data-landed marks. The captures passing during the tape's window are R=04-07; the R=01
lap arrives ~8.10 - and the 8.08-8.18 era was UNINSTRUMENTED (JUDGE/FLAGWR windows ended
8.078/8.070 - a self-manufactured absence, caught by doctrine). The banner still fires at
8.2: the consumption lap didn't complete in run268 - WHY is the run269 question (windows
widened to 8.19 across JUDGE/FLAGWR/GATE7B0E).

## cont.136 — THE RESIDUE IS A ROTATIONAL RACE (run270): four f0s banked, want rewound to 1, the consumption lap needs sector 1's re-pass at ~8.213 vs the timeout at ~8.185

Run270 (windows verified wide): after the 4th stake (~8.050) the fw REWINDS want to 1
(limit 5) and walks the passing captures R=05..0a+ with want frozen - the consumption
pass matches CAPTURES against wants, so consuming f0-slot 1 needs SECTOR 1's physical
re-pass: R=01 arrives ~8.2126 (12.5ms x the remaining sectors), the monitor's INITs fire
at ~8.1846 (patience ~207ms from the 7.977 accept). **28ms short. The staking half of
the fix is verified; the consumption half loses a rotational race.**
THE DESIGN QUESTION (Dave): is consumption LAP-DRIVEN by design (then real hw faces the
same geometry - and either the real monitor's patience is longer than our CPUAP-side
timing produces, or the real consumption starts mid-lap where the head already is
(wants consumed in R-order encountered: 2,3,4 then 1 - not rewound-to-1-first: the
REWIND-to-want-1 ordering is the extra ~1 revolution)) - or INSTANT by design: the $92f6
block also sets [$7B18]=1 / [$7426]=1 / [$79B8]=1 (the DATA-READY flags; the old lore's
"$7f52 tst-$7b18 consumer") - if the $7b18-consumer completes the want without a
physical re-lap, the rewind we observe is a fallback for a consumer that didn't fire,
and the missing piece is that consumer's trigger. Static next: $7f52's context (the
tst $7b18 branch and what it completes), and whether read1's launch consumption used it
(read1's f0-consume at 7.965 was 1.2ms after its stake - NO physical lap - INSTANT -
which answers the design question already: read1 consumed via the instant path. Read2's
rewind-and-relap = the fallback. THE $7B18-CONSUMER IS THE LAST GAP.)

## cont.137 — THE $7B18 CONSUMER LOCALIZED: set by the stake ($92f6), CONSUMED at $7f22, guarded at $7f52 — one handler tail; read1's instant path confirmed; the last tap specified

$7b18's complete map: SET by $92f6 (the stake, with $7426/$79b8); **CLEARED at $7F22 -
the consumption act** - in the handler tail $7f22..$7f52 (clear, work, nop pad, the
$7f52 guard: tst $7b18; set -> rte; clear-at-guard -> trap #1 diagnostic), directly
above the pull legs ($7f5c+/$7f6c). Read1's instant consume (stake 7.96358 -> c-stamp
7.96505, 1.5ms, no lap) rode this path. Read2's rewind-and-relap (run270: 28ms short of
sector 1's re-pass) = the fallback of this consumer not firing.
**THE LAST TAP: $7f22 execution, windows-from-zero** - read1's firing context (which
walk entry routes there) vs read2's era post-stakes (8.0125+): never-fires -> the
diverting branch in the walk body $7ee0-$7f22 (one static once the absence is doctrine-
confirmed); fires-but-incomplete -> the work between $7f22 and $7f52. The measured
chain's final segment: stake ($92f6, WORKING) -> data-ready ($7b18=1, WORKING) ->
consume ($7f22, THE GAP) -> c-stamp -> $ba.

## cont.138 — DAVE'S STATIC CHAIN: $7f22 has ONE predecessor ($7ed8, the hunt-exit); three candidate diverts; prediction = the $7968/free-pool cell (run271 paired tap in flight)

**The consumer statically end-to-end:** $92f6 stakes + sets $7b18, then returns INTO the
walk ($9370 -> $7f6c) - stake and consume are TWO events. The consume = the walk
subsequently reaching **$7ed8 (hunt-exit: clr $741c -> $7ede bra $7f1a -> $7f22 clr
$7b18 = CONSUME -> complete -> rte $7f5a)**, with the $7f52 guard's trap #1 expecting the
completion inside the 12-nop settle window. $7ed8 is $7f22's ONLY predecessor. Read1
reaches it; read2's walk re-arms ($7d5c: $741c<-1) and relaps - miss the exit once and
the next chance is a full revolution (+12.5ms x remaining, the 28ms loss).
**Three candidate diverts (the walk body's gates to $7ed8):**
1. $7d16/$7d1e: [$79a0]==0 OR [$79b6]!=0 -> the $7e50 match regime (can reach the exit);
   else the $7d22 ledger-index scan (knows only ff/fe, NEVER f0) -> $7d4a continue.
2. $7e58: the position match (cmp [$7428],D0; bne $7d4a) - capture/want mismatch.
3. $7e8a: tst [$7968] -> $7eb2 (-> $7ed8 exit) vs $7e90 (clr $742c, continue) -
   **[$7968]!=0 (queue-non-empty, the cont.129 free pool) diverts.**
**Prediction (Dave): #3 - the pool that never drains keeps [$7968]!=0, sending every
post-stake pass down continue: four f0s staked, zero exits taken. If read1 hits $7ed8
with [$7968]==0 and read2 sits at [$7968]!=0, the consume-residue and the cont.129
free-pool root are ONE CELL - and the fix is draining the pool so the fourth stake's
pass reads 0 and exits.** Run271: paired $7ed8+$7f22 tap with [$79a0]/[$79b6]/[$7968]/
[$7428]/ledger[want]/[$7b18] per firing.

## cont.139 — PHYSICS CORRECTION + THE REFINED RESIDUE (runs 271/272): the completions are honestly timed; the premature exit at match-4 and its REWIND are the last question

**Correction: the "one sector late" framing was wrong.** The carry fires at the sector's
data-field END (the following boundary mark) = IDAM + 11.4ms - honest FM physics (128
data bytes = 11ms under the head; run272's CARRY-R R=1@8.01250 vs IDAM@8.0013 ✓). The
cont.139 irq5-switch in the chancomplete defer was a NO-OP (stakes identical) - left in
place, harmless (the boundary marks are irq5-class anyway).
**The refined residue:** completions 1-3 land before match-4 (each at IDAM_N+11.4ms,
1.1ms before IDAM_N+1 ✓); completion 4 lands at 8.0500, AFTER match-4 (8.0388) BY
PHYSICS - no timing fix can put 4 completions before the 4th match. **Run271/272: the
hunt-exit ($7ed8 -> $7f22 consume) fires AT match-4 with 3/4 f0s banked; its completion
body judges not-done and REWINDS want to 1 (run270: want=0001/limit=0005 from 8.05015),
and consuming want-1 then needs sector 1's physical re-pass (8.213) - the fatal relap.**
Also measured: [$7968]=0 throughout (Dave's divert prediction inverted - the exit ROUTE
works; the pool question is moot for the exit); the count hit 0 exactly at match-4
(run256) - the exit's early firing coincides with count-zero/wants-exhausted at the
LAST want's match.
**FOR DAVE (static): the exit body between $7ed8/$7f22 and the c-stamp** - what it
checks (3/4 vs 4/4), what the REWIND is (its writer pc ~8.0502; a verify-pass semantics?
an error-retry?), and whether the real design exits at match-5 (first pass after the
last data - 8.0513, 130ms inside patience) with our match-4 exit being the artifact
(routed by count-zero at the last want) - or the rewind is correct and the consumption
is meant to ride something faster than a relap (the data-ready $7b18 chain consuming
want-4's f0 the instant its completion lands at 8.0500, no relap - in which case the
8.0500 stake's INSTANT consumption is the one unlit link).

## cont.140 — DAVE'S EXIT-BODY STATICS: the done-check is op-scheduled and CORRECT; there is NO rewind; the residue is ordering-or-coupling, one tape apart (run273 in flight)

- **The done-check is an OP** ($94ec/$9398, installed into the dispatch table at $37d6/
  $381e) - $9506 runs when the op-loop dispatches the node's done-check op, NOT at the
  fourth match directly. **The check itself is CORRECT: $9506 tst [$79A8] (owed count) /
  bne $95e4 - requires zero.** Candidate 1 sound.
- **There is NO rewind:** the not-done path $95e4 = set [$79b6] (keep-hunting), clear
  [$72d6], [$72dc]<-ffff, rts - the relap is the EMERGENT consequence of continuing to
  hunt, not an instruction. Candidate 2 dissolves; nothing to model differently.
- **The subtlety (measure, don't assume): the $932c owed-decrement is GATED at $9322 on
  [$79b6]!=0** - a completion landing while keep-hunting is CLEAR drops its decrement.
  "3/4" was an assumption about [$79a8]; the coupling could drop a decrement independent
  of the 4th f0's timing.
**Run273 (Dave's ordering tap): [$79A8] writes + the $9322 gate state + the $9506 firing
([$79a8] value, timestamp vs the 4th $932c).** The frame decides: (i) done-check
dispatched BEFORE the 4th decrement = genuine ordering bug -> wire the 4th completion's
$7b18 data-ready edge to re-dispatch the done-check op (LLE-honest: on real hw the 4th
completion carries the node into the done op); (ii) [$79a8] never zeroes = the $79b6
coupling drops a decrement -> the fix is upstream in the keep-hunting/owed-count
coupling, and the exit body is exonerated. Don't build the re-trigger until the tape
says which.

## cont.141 — RUN273: the done-check op NEVER DISPATCHES (either read); [$79b6] never set; owed staged 8 at $a492 for read2; the consume body $7f22-$7f52 is the last unread stretch

- ZERO DONECHK firings all run - the $94ec/$9398 op route is a third completion path
  neither read uses. Read1's c-stamp = the walk's $7d90 branch (f0-consumption), owed
  coincidentally unstaged (0) at its era.
- [$79B6] never set anywhere (its only setter $95e4 lives in the never-dispatched done
  op) -> the $9322 gate drops EVERY decrement by construction; [$79A8] frozen at its
  staged value.
- **[$79A8]<-0008 staged at pc $A492** - fresh at 7.9643 AND 7.9887 (read2's launch):
  read2's owed = 8 (unit? 2 per sector - id+data? - needs naming); even ungated, 4
  decrements != 0.
- The rewind (want<-1 at ~8.0502) lands 11.4ms after the 8.0388 exit - possibly the 4th
  stake's own processing, not the exit body.
**FOR DAVE (the last unread stretch): the consume body $7f22..$7f52** - what the work
between the $7b18 clear and the guard actually does (read1: it reached the c-stamp; read2
at 8.0388: it did NOT - what operand differed), plus the $a492 stager's context (what 8
counts) and whether the walk's $7d90 c-stamp branch (read1's route) has a gate read2's
8.0388 consume failed. The completion architecture now has THREE routes on the map:
(1) the walk's $7d90 f0-branch (read1's, WORKS), (2) the $7ed8->$7f22 hunt-exit consume
(read2 reaches it, doesn't complete), (3) the $94ec/$9398 done-op (never dispatches).
Which one read2 is DESIGNED to ride - and what its 8.0388 pass lacked - is the question
the $7f22-body read answers.

## cont.142 — DAVE'S THREE READS + the pivot tap (run274 in flight): owed = block[7], copied; the $7d90 arm is [$7b10], disarmed by [$7968]==0; the consume body is a DEAD-END

- **The owed-stager ($a486-$a48e): [$79A8] = block[7] via A6 - COPIED, not computed.**
  Read1 never runs it (owed unstaged -> its done-side trivially satisfied). Read2 carries
  an 8-unit obligation read1 doesn't - THE TWO READS ARE DIFFERENT COMMAND SHAPES.
- **The $7d90 c-stamp gate = [$7B10]** ($7d68: tst/beq $7d9c), armed at $6ed6
  (move #$ffff) and **DISARMED at $6f3a when [$7968]==0** ($6edc/$6ee0). Read1's
  [$7968]!=0 kept it armed; read2's [$7968]==0 - the very state cont.139 called moot -
  STRIPS ITS COMPLETION ARM.
- **The consume body $7f22-$7f5a: clr $7b18 -> SR $2500 -> F001-bit6/[$796e] gates ->
  nops -> re-check -> trap#1/rte. NO C-STAMP - a consume, not a completion.** $7f22 was
  never a finish line; read2 is DESIGNED to ride $7d90, same as read1.
**THE PIVOT: name block[7]'s 8** (run274: the block at the stager, both eras). Fork per
Dave: (i) 8 = 2 sides x 4 (A6[2:3]=1024, two-sided set) -> read2 legitimately owes 8 and
exits prematurely at the side-0 boundary -> the fix is the CROSS-TO-SIDE-1 leg (the $fe
terminator path - cont.72's ORIGINAL reading, the arc's first silhouette); (ii) 8 vs
A6[2:3]=512 -> inconsistent with its own transfer length -> a model-fed block field.
PRE-STAGED NOTE: run225's UIBCOPY image (host-sourced, 0fe948) shows byte7=08 - if A6 =
the UIB, the 8 is the REAL monitor's own INIT staging (authentic fw-to-fw data), and
fork (i) is the lean. Don't wire either fix until the block frame lands.

## cont.143 — RUN274: THE TRANSFER COMPLETES. DESCDONE 8.17500, 1024/1024, VOL1 DELIVERED, 9.6ms IN-PATIENCE; the last cell is the c-stamp arm's [$7968] strip

**The pivot named:** A6 = the UIB (6e60); block[7]=08 = SECTORS-PER-1KB-BLOCK (1024/128 -
the descriptor's own /1024 window on every carry). [12]=40 two-sided CLEAR -> fork (i)
dead; single-sided. **Read2's true obligation: 8 sectors, one 1KB block, window at
sec0=7: R=7 = VOL1 ("56 4f 4c 31" - the label itself, on the wire!) + R=8 HDR1 + six
empties. The R=1-4 want-stakes were out-of-window sectors, carried and self-filtered
(slots 10-13, host+1280+) exactly per cont.59's logical-map design.**
**RUN274's headline: the in-window carries run R=7..14, 128/1024 -> 1024/1024, and
DESCDONE posts at 8.17500 - the full block delivered to the host NINE POINT SIX
MILLISECONDS BEFORE the 8.1846 recovery.** The transfer beats the timeout. The sole
missing act = the fw's own completion: the walk's $7d90 c-stamp -> $ba - DISARMED
([$7B10] stripped at the $6ed2 op because [$7968]==0, Dave's read #2). Read1's pass kept
the arm ([$7968]!=0 at its $6ed2); read2's lost it.
Run275 (in flight): [$7968]/[$7B10] biographies - the last cell. Once the arm survives,
the fw's own c/$ba lands ~8.176 and beats the recovery by 8ms - the command completes,
the monitor advances, and the cascade above (INITs, no-4 chain, park) never fires.

## cont.144 — RUN275: the [$7B10]/[$7968] theory FAILS THE DIFFERENTIAL; both reads stripped identically and read1 c-stamps anyway; the discriminator is inside the f0-consumption chain

ARM7B10/FLAG7968 biographies: **BOTH launches run the identical arm-strip** ($6edc arm
ffff -> $6f40 disarm 0000, 3us apart; [$7968]==0 at both $6ed2 passes) - read1 at
7.96476, read2 at 7.98941 - **and read1's c-stamp lands at 7.96505 REGARDLESS, 0.3ms
after its own disarm.** Dave's read #2's gate is real code but does not discriminate;
read1's completing route does not pass through the $7d68 test as read (or [$7b10] is
re-armed by an untapped path, or the $7d90-block's c-chain has an ungated entry).
**The live differential remains the f0-consumption -> $17fe chain:** read1's ONE f0
(want-0, SLOT 0, staked at launch by the synchronous chain) fired $17fe 0.8ms later;
read2's FOUR f0s (wants/slots 1-4, staked by the strobe fix) never fired it. Salient
asymmetries for the next read: (a) slot-0/want-0 vs slots-1-4 (a slot-0-special
consumption path? the $7d22 scan's base case?); (b) read1's f0 lands pre-hunt (walk
idle) vs read2's mid-hunt; (c) read1's consumption used the $7e50-family ledger-byte
route with want=0 (no capture-R match needed?) vs read2's want-match regime.
NEXT (Dave or fine-trace): read1's 7.9647-7.9651 pc-chain (the 4ms from its $6ed2 to its
c-stamp - what routes into $17fe with the arm formally disarmed) vs read2's post-f0
passes. The transfer is DONE and in-patience (cont.143); the c-stamp chain is the whole
remaining distance.

## cont.145 — DAVE'S TWO-BIT END-CONDITION: DONE = descriptor[$18]!=0 AND D1.bit4 at $17fe/$1810; read2 brings one bit, not both (run276 trace in flight)

**The $17xx c-stamp routine builds the completion descriptor from walk state ($7428/
$7430/$7466/$7696/$790e/$7908 -> A0's fields) and gates the DONE stamp on a two-condition
AND: $17fe `move ($18,A0),D0; bne` AND $1810 `btst #4,D1; bne` -> $184a/$184e status $82
posted; miss either -> $1816 partial/not-done reset (clr $74c0/$74c2/$7b40-53) - the
silent outcome.** Neither field is written by the stamp routine - both are upstream
inputs that must COINCIDE. The slot-0-walk-idle vs slots-1-4-mid-hunt asymmetry maps:
one bit = the data-landed/f0 signal (read2 has it, four times); the other = a walk-state/
quiet-point condition only read1's launch-synchronous firing holds.
**Run276 (Dave's three-step): (1) reach-check - do read2's post-f0 passes reach $17f8 at
all (chain: $6ed2 -> $7102/$7106 ledger-accounting bset#6 -> $17xx)? (2) if reached:
desc[$18] + D1.bit4 both reads (read1 expect both -> $82; read2 exactly one names the
missing bit); (3) one static: the missing bit's setter (desc[$18]'s prior-op writer;
D1.bit4 from the $7a74/$76d8 status-nibble region's $f0 mask).** Trace before assuming -
the [$7b10] walk-back is one message old; a two-bit AND fails two ways only tape
distinguishes.

## cont.146 — RUN276: READ2 NEVER REACHES THE STAMP; the divergence is bracketed to the $7106->$17f8 body (137us); plus the $82-vs-$ba wrinkle

- **Reach-check: NEVER-REACH.** Zero CSTAMPGATE in read2's post-f0 era. Both reads enter
  the accounting step at launch (CHAIN7106: A0=$7654, d18=c0c0, D1=ffff - IDENTICAL
  visible state); read1 proceeds to $17f8 137us later (7.964877 -> 7.965014); read2
  diverts inside the $7106->$17f8 body (7.989531 -> nothing). THE BRANCH IS IN THAT BODY;
  the discriminating operand is deeper than the tapped frame - one static with the two
  frames in hand.
- **The wrinkle: read1's own stamp frame is d18=$0069 (!=0) with D1=$8c27 - bit4 CLEAR -
  and its command completed anyway** (the monitor proceeded at 7.977). Per Dave's decode
  that takes the $1816 arm, not $184a/$82 - so the $82 branch may be a completion VARIANT
  (the monitor's own DONE = the $ba/0x80|code posts of cont.77 lore), and the two-bit AND
  gates that variant rather than done-itself. Fold into the body read.
- Reference frames: the INIT-era stamps (6.401x, 8.1853: A0=71f0, d18=98e7, D1=0000) =
  the 87/89 command class through the same stamp.
NEXT (one static): the $7106..$17f8 body - its branches and exit conditions against the
identical-at-entry frames; the operand that routes read1 onward and read2 out is THE
final cell. (All infrastructure below verified: transfer complete in-patience, marks
staked, chain entered by both reads.)

## cont.147 — THE FINAL CELL CONFIRMED FROM THE CHAIN SIDE: $7106's FIRST INSTRUCTION is `tst [$7968]; ==0 -> $6f44 divert`; the root is the POPS-BEFORE-OPS ordering

**$7106 opens with the branch:** tst.w [$7968]; bne $7114 (the accounting body, the road
to $17f8); ==0 -> move #$2400,SR; bra $6f44 - OUT. [$7968] (want-present) gates the
c-stamp chain at its first instruction. Dave's read #2 had the right cell at a non-
discriminating gate ($6ed2/[$7b10]); the same flag guards the chain itself - and BOTH of
read2's failed gates ([$7b10] stripped at $6ed2, $7106 diverted) key on the same flag at
the same era.
**THE ORDERING ROOT (already on tape):** read1's launch POPS FIRST (asks/pops at 7.9635/
7.9642 - the ledger/queue path, no match needed, riding the cont.82 bootstrap strobes)
THEN ops ($6ed2 7.96476, $7106 7.96488) - flag UP, chain proceeds 137us to the stamp.
Read2's continuation runs OPS FIRST (7.9894-7.9895) with its first pop only at its first
HUNT MATCH (8.0013) - flag DOWN at both gates, divert. Pops-before-ops vs ops-before-
pops; the entire two-gate failure is one choreography inversion.
**THE REMAINING QUESTION (the true root): why read2's first ask waits for a hunt match
while read1's launch asks immediately** - the ladder's ask ($7ee0/$32ac) at launch vs
re-launch; what makes the continuation's serve defer its first pop until a capture
arrives (fw design = the re-launch legitimately re-asks only on captures, and the
completion accounting is meant to ride the LAST match's serve - in which case the 4th
match's pass at 8.0388 had a want in hand ([$7968] semantics at mid-hunt?) and the gates
should have passed THERE - measure [$7968] at read2's four matches; if 1 at the matches,
the $7d68/[$7b10]-armed walk pass with flag up = the designed stamp moment, and the only
missing piece is [$7b10]'s re-arm between launch and the matches).
NEXT: one frame - [$7968] + [$7b10] live at read2's four match/consume instants
(8.0013/8.0138/8.0263/8.0388). If both up at any match: the stamp should have fired
there - trace that pass's divert. If [$7b10] down: its re-arm ($6ed6) never re-ran -
the $6ed2 op dispatches once per launch - and the design question is what re-arms it
mid-command on real hw.

## cont.148 — RUN277 + SESSION CLOSE: the $7d68 walk-gate is nobody's route (both flags down at every test, read1's era included); the frontier is the SERVE CYCLE's flag lifecycle

WALKGATE: [$7b10]=0 AND [$7968]=0 at every $7d68 test, both eras - and no frame exists at
read1's completing chain (7.9648-7.9650): **read1's $7106 entry came from the door/serve
path directly (the launch's synchronous chain), NOT the $7d90 walk block - the $7d68
gate is nobody's route.** The completion accounting rides the SERVE CYCLE: pop sets
[$7968] -> the serve's accounting ($7106 chain) runs INSIDE that window -> the $70e0-era
clear -> next. Read1's entry lands inside the window; read2's launch entry (7.98953)
lands outside (ops-before-pops, cont.147), and its four match-era serves never re-enter
$7106 at all (zero CHAIN7106 post-7.99).
**FOR THE NEXT SESSION (fresh eyes, full corpus): the serve cycle's flag lifecycle** -
the exact sequence pop($79c6 set) -> serve -> $7106-entry -> $70e0 clear for read1's
completing cycle (7.9635-7.9650, all on tape) vs read2's cycles (8.0013+, pops returning
wants but 7968 reading 0 at the forks - the set/clear ordering within its cycle), and
why read2's serves skip the $7106 accounting entirely. The answer routes one of: the
serve's accounting call is conditional on a state read2's cycle lacks; or the pop's
7968-set doesn't fire for queue-branch pops (the $79c6 gate's operand); or the
continuation's serve uses a different cycle body. ALL infrastructure below this is
DONE: transfer complete in-patience (DESCDONE 8.1750, VOL1 on the wire), marks staked,
consumption chain mapped, five keepers standing.

## cont.149 — THE LIFECYCLE READ FROM EXISTING FRAMES (no run): the complete mechanism; the "rewind" was the f0-SCAN AIMING; the single missing arm is [$7B10] at the $8320 gate

**$8300-$8350 = THE f0-CONSUMPTION HANDLER (IRQ-tail):** scan the ledger for f0 ($8302
cmpi.b #$f0); not-found -> node status $69 (the error); **FOUND -> [$7428] <- the slot
($8318) - READ2'S 8.0502 "REWIND TO WANT-1" WAS THE SCAN AIMING AT ITS FIRST f0** -
clr [$79b6] -> **$8320: tst [$7B10]; ==0 -> SKIP** -> (armed: consume the arm, bsr $7106
- the stocker reads the f0 at the cursor -> C-STAMP).
**[$7B10]'s lifecycle (all on tape):** only arm = $6ed6 inside the $6ed2 op (arm -> tst
[$7968] -> no-want-in-hand: disarm at $6f3a AND fall into the INLINE stocker $6f44).
Read1's launch: inline stocker met its launch-f0 at the cursor -> the bail arm -> c-stamp
at 7.96501 (its complete route, no arm needed). Read2's launch: inline stocker staked its
four wants (the ff x4 on tape) - CORRECT, arm consumed. Its four f0s landed (the cont.135
strobe); the scan ran at 8.0502 and AIMED; **the arm was down; the stocker call skipped;
c never stamped; the monitor's recovery fired 134ms of unused patience later.**
**The design's mid-command arm: a $6ed2 re-dispatch during the serve with [$7968]!=0 (a
want in hand) keeps the arm standing for the scan.** Our tapes: 6ed2 dispatches once per
launch, never re-runs (the op-walker parks); read2's pops (which set the want-in-hand
window) all post-date its one 6ed2 pass - cont.147's ordering, now with its consequence
fully mechanized. THE FIX CANDIDATES (for Dave): (a) what re-dispatches the op list
mid-command on real hw (the doorbell-cascade? an event our completion timing suppresses);
(b) whether the $8352+ block (a SECOND tst $7b10 immediately after) is an alternate
consumer with its own arm; (c) whether the real [$7968] lifecycle keeps a want in hand
ACROSS the launch's 6ed2 (the pop-set surviving into the op) - one static each. The
mechanism is closed: every tape frame from forty runs now has a named place in it.

## cont.150 — THE BIT DELIVERED, THE CHAIN UNMOVED (run278): [$F000].4 up from 8.0125, the $6bc2 poll live, no re-init — the AND's second conjunct (descriptor[$18]) is the next frame

**Wired (KEEPER candidate, pending): m_want_ready latched at the carry, presented as
[$F000] bit4 (OR'd with the index pulse), cleared at the DONE post.** Run278: no c-stamp,
no CMDDONE, banner unchanged. The level WAS up (first carry 8.0125) and the $6bc2 poll
WAS live (the op-walker's x2488 park spans the era on every tape) - so the $6c10 bit4
test now passes and the chain proceeded to $6c18: **D0 = descriptor[$18] (A1 = IOPB+$18)
- the AND's second conjunct - and the re-init didn't fire: either desc[$18] reads wrong
at the poll, or the $6c14-onward shape differs from the static.**
NEXT FRAME (one tap): $6c18 execution - A1, the word at (A1) = IOPB+$18's live value,
[$f000] at the moment, [$7a60] (the 0->2->1 re-init machine's state), timestamps - read2's
era polls vs read1's. Also untapped so far: $94a0 (the re-init) and [$7a60] itself. The
one-condition-two-gates reading stands (Dave's convergence); the remaining question is
purely what IOPB+$18 holds at read2's polls and who was supposed to write it (the
host-side IOPB field? the fw's own descriptor build? - the $17xx routine COPIES walk
state into the descriptor, so [$18] has a builder somewhere in that family).

## cont.151 — RUN279: zero CONJ2 (the $6c18 fall-through never taken within visibility); the F000 reader census names the WALKER'S OWN POLL ($15b2) as the re-dispatch driver candidate; the era past 8.0125 is CAP-BLIND

- CONJ2 empty windows-from-zero: the $6c10 bit4 fall-through never executes IN THE
  VISIBLE ERA - but the F000TERM probe's 40-line cap exhausted at 7.9906 (36 reads at pc
  $15B2, the op-walker's pre-dispatch poll at 80us cadence, + $654c/$65a8/$7f32) - **the
  8.0125+ era (the bit UP) is CAP-BLIND for F000 readers. Doctrine: a capped absence
  decides nothing.**
- **pc $15B2 = THE OP-WALKER POLLS [$F000] BEFORE EVERY DISPATCH** - Dave's candidate (a)
  re-dispatch driver, on tape: the walker's op re-dispatch is F000-gated. What bit(s)
  $15b2's code tests = one static; with bit4 now delivered, the walker's post-8.0125
  polls may re-dispatch the $6ed2/$6bc2 family - or test a different bit.
- $7f32 = the consume body's F001 btst#6 (Dave's decode ✓ live).
NEXT (one run + one static): (a) static $15a0-$15c0 (what the walker's F000 poll tests
and how it selects the re-dispatch); (b) re-run with the F000TERM probe re-windowed to
8.01-8.06 (or a dedicated $6c04/$6c10 tap) - does the $6bc2 chain see the bit and reach
$6c18, and does the walker's poll change behavior with bit4 up. The fix's hardware half
is delivered and held; its consumption path is one cap-widening from visible.

## cont.152 — RUN280: the $6bc2 chain is the NINTH never-runs (zero POLLBTST, both windows, launch included); the 80us F000-poller is unidentified; the decisive instrument specified

- POLLBTST empty in BOTH windows (7.99-7.995 launch-polls AND 8.012-8.30): **the
  $6c00/$6c10/$6c18 chain never executes in this flow** - Dave's $6bc2 re-dispatch gate
  is real code on an untaken path (joining the done-op, the walk-gate, $9602, $7402...).
  The "pc=15b2" F000 reads were NOT $6c00's - the F000TERM probe's m_cpu->pc() at
  read-cycle time is UNRELIABLE (add to doctrine: read-tap pc attribution needs
  corroboration by an opcode-tap at the presumed site).
- STANDING: the [$f000].4 want-ready level (delivered, held, harmless - and still the
  $17fe done-stamp's D1.4 conjunct, so likely still half the fix); the launch era has a
  REAL 80us-cadence F000-poller (36 reads, 7.990-7.9906) whose identity is unknown.
**THE DECISIVE INSTRUMENT (next session): tap the $15b4 op-table read (`move.w
(A0,D0.w),D0` with A0=$192) logging D0 (the op index) and the fetched handler, uncapped,
windowed 7.99-8.30** - names the dispatched/parked op stream outright: which op polls
F000 at 80us, whether the walker re-dispatches post-8.0125 with the bit up, and where
the $17fe-family consumer actually lives in this flow. One table-read tap = the whole op
choreography, no pc skew (the operands are registers at a known instruction).

## cont.153 — THE OP TABLE ANSWERS IN ONE LINE (run281): the park = op-42 re-dispatched forever on its WARM path; the front returns to cont.38h's [$7A3E] timer flag

**OPTAB: `idx=42 hdlr=6bc2 (prev x0) wr=1 @7.990049` - AND NOTHING ELSE THROUGH 8.30.**
The walker re-dispatches op-42 forever (index frozen; the register-level tap cannot be
skewed). The $6bc2 chain IS the poller after all - but our flow rides its WARM
instant-return (motor already on, cont.79's decode): the deep body (the $6c00 F000/bit4
reads, the $94a0 re-init scaffolding) is the COLD path's, which only read1's launch
(motor spin-up, the 18583-poll era) ever ran. Run280's zero POLLBTST ✓ consistent: the
warm path exits before $6c10.
**The warm path's wait = [$7A3E] (the spin/settle timer flag, cont.74's op-42 decode) -
THE ANCIENT cont.38h QUESTION ("does [$7a3e] EVER get set in the retry era") IS THE
FRONT:** the chain PIT tick -> IRQ1 -> $2b58 handler -> the $736c timeout-queue walk ->
$7a3e-family set -> op-42 completes -> THE WALKER ADVANCES to the subsequent ops (the
completion scaffolding) with the bit up. The model's PIT/IRQ1 wiring is self-documented
as half-trusted (timer0_out: clock divisor a guess; the tick fires on tape but whether
$2b58's walk ever sets [$7a3e] is UNMEASURED - every tape shows 7a3e=0000).
**Bonus defect on the same line: wr=1 at 7.990 = READ1'S STALE want-ready level** - my
DONE-post clear (the $7fe8 CMDDONE branch) never fires on read1's completion route; the
level needs its clear at the true completion post (or at doorbell-accept).
NEXT: (a) static - op-42's warm-exit condition ($6bc2's early body: what it tests beyond
motor - the [$7a3e] wait's exact shape) and what its DONE-return needs; (b) the $2b58
IRQ1 handler's timeout-queue walk - does it ever set [$7a3e], and what queues the record
([$736c] entries); (c) fix the stale-level clear. The op choreography is ground truth
now: one op, one flag, one timer tick between the walker and the completion scaffolding.

## cont.154 — THE CLOCK IS THE DEFECT, MEASURED (run282): tick 26.116ms x count ~70 = 1.83s settle vs 200ms patience; the model's self-documented PIT-clock guess is off ~10-25x

TICKWALK: the record EXISTS (rec=$7360, cell=$7a3e ✓, [$7a40]=$7360 registered), the
IRQ1 tick FIRES and decrements - **once per 26.116ms exactly** (the model's pit[1] ctr0:
count 0xFF00 @ 10MHz/4=2.5MHz = 26.1ms - the divisor its own comment calls A GUESS) -
count observed 0x46->0x3e across the era: **~70 ticks to expiry = 1.83 SECONDS, against
the monitor's ~200ms patience. The warm settle can never land, by arithmetic, on any
run.** The firmware+host contract (a 70-tick settle inside 200ms patience) implies a
~1-2.8ms real tick.
THE RESOLUTION SPACE: (a) the PIT input clock divisor (the guess) - but 0xFF00 @ any
8253-legal clock (<=2.6MHz) cannot tick faster than ~25ms, SO (b) the fw's actual ctr0
RELOAD in the running era is probably NOT 0xFF00 (the tick handler "re-arms via gate0 +
the $8001 count reload" - the model observed ONE programming; the steady-state reload
may be small/fast), or (c) the tick rides a different counter/output than modeled.
DECIDING FACTS (one frame each): the fw's actual $8001/$8007 reload writes in the
running era (a PIT-write tap or the pit device's own logging), and UIB[$18] (the settle
count's source byte - the register site scales from it). THEN wire the clock the reads
dictate - never drive [$7a3e] from C++ (Dave's mandate line). Also standing: the wr=1
stale-clear fix (independent, owed).

## cont.155 — DAVE'S INVERSION: descriptor[$18] IS THE SETTLE DELAY; zero = set-the-flag-NOW ($6c5c); the fix relocates from the PIT to the field (run283 in flight)

**$6c40-$6c64 decoded end to end:** [$7a3e] set -> proceed; else D0 = desc[$18]:
**==0 -> $6c5c: [$7a3e]<-ffff IMMEDIATELY (no timer - the warm path's rightful
completion)**; !=0 -> $6c64: queue a $736c timeout of duration [$18] (the $29f8 register
- read1's legitimate ~1.55s cold spin). The flag's two setters exactly as suspected: a
completion set and a timeout set, one field between them - **and it's the SAME $18
offset the $17fe done-gate keys on: the second conjunct and the settle delay are one
byte.**
**The inversion: delivering the tick would faithfully model a STALL** (read2 "finishing"
by waiting out a delay it shouldn't have). Read2 is on the timer arm because its
desc[$18] != 0; a warm motor needs no settle, so the field should be ZERO and the flag
set on the spot - a live instruction the firmware already has. Read1 (cold) legitimately
has [$18] != 0. Same code, correct on cold, wrong-INPUT on warm. **The model isn't
withholding a tick; it's presenting a cold-motor settle requirement to a warm motor.**
Run283 (in flight): the $6c54 read's A0 (which structure) + the byte, cold (6.41) vs
warm (7.99). Then the writer: if computed from motor state, the LLE fix = present the
motor as already warm so the fw writes zero and takes $6c5c - never touch the field or
the flag from C++.

## cont.156 — RUN284: THE WRONG-COUNTER ATTRIBUTION. The fw's tick = pit[1] CTR2 (mode-2 rate generator); the model raises IRQ1 from CTR0's boot 0xFF00 (the 26ms guess)

PITWR (windows-from-zero): the running era's ONLY PIT writes are CONTROL bytes at $8006 -
**0x9A = ctr2, LSB, MODE 5 (hw-triggered one-shot) at pc $6854 (the seek era - the settle
one-shot the model's comment admits faking via m_settle_out)** and **0xB4 = ctr2,
LSB+MSB, MODE 2 (RATE GENERATOR) at pc $1ae4 (the teardown)**. NO count reloads in-era:
the model's 26.1ms tick rides ctr0's boot 0xFF00 - and the model's OWN line-273 comment
says the system tick is **OUT2** ("pit[1] OUT2 level (system tick edge -> IRQ1)") while
the code raises IRQ1 from timer0_out = **OUT0**. THE FIRMWARE'S TICK IS COUNTER 2's RATE
GENERATOR; the model wired the wrong output.
**The missing number: ctr2's boot-loaded count** (its data-port write predates the run's
visible frames; recover via an early-window PITWR rerun or the pit device's state). Then
the LLE wiring: IRQ1 <- pit[1] OUT2 edge (per the fw's own mode-2 programming), retire
the OUT0 test-wire and the m_settle_out direct-drive (the fw's mode-5 ctr2 one-shot IS
the settle hardware - both self-documented fakes replaced by the real counter). With
ctr2's real rate, the 70-tick settle expires in the ~70ms class, op-42 completes ~8.06,
the walker advances, the scaffolding runs with the bit up. Also owed: the wr=1 stale
clear.

## cont.157 — THE COMPLETE PIT PROGRAMMING (run285, mirror-tapped): ctr0 = a 102us system tick (LSB-only 255, mode 3, handler-reloaded at $2bca); the model honors it as 0xFF00 — A FACTOR-OF-256 BYTE-LANE DEFECT

The fw's programming, complete and on tape:
- **ctr0 (pit1) = THE SYSTEM TICK: ctrl 0x26 (LSB-only, MODE 3), data 0xFF = COUNT 255,
  and THE IRQ1 HANDLER ITSELF RELOADS IT ($2bca, inside $2b58, every tick)** - at the
  modeled 2.5MHz: ~102us period. THE MODEL TICKS AT 26.112ms = 255 x 256 x 409.6ns -
  **the LSB byte lands as the count's HIGH byte (0xFF00) - a factor-of-256 byte-lane/
  access-mode defect in the pit wiring** (the model's own boot-era "count 0xFF00" note
  was this same mis-landing, observed and believed).
- ctr1 = the seek-settle one-shot: ctrl 0x7A (mode 5), count $024A=586 (the $3ce4-$3d00
  boot loads) ✓ matches the model's own ch1 comment.
- ctr2 = mode-2 rate-gen / mode-5 one-shot pair (the $6854/$1ae4 seek-era toggles), boot
  count 0x0000=65536.
**With the true 102us tick: the 70-tick settle expires in ~7ms; op-42 completes ~7.997;
the walker advances to the completion scaffolding (want-ready bit up and holding) 190ms
inside patience.** The fix = the pit byte-lane/access-mode handling (find why the
LSB-only 0xFF is honored as 0xFF00 - the umask16 interleave's delivery into the MAME
pit8253's count latch), THEN the OUT-wiring question (IRQ1 <- which OUT) resolves
naturally since ctr0 IS the tick per the fw's own reload-in-handler. Also owed: wr=1
stale clear. FOR DAVE: the ×256 mechanism (one C++ read of the pit map/handlers) + the
green-run projection.

## cont.158 — THE LANES EXONERATE THE INTERLEAVE (run286): the fw's programming reaches pit[1] ctr0 intact; the x256 lives in the GATE/DEVICE layer — the last C++ read

Lane-tagged stream: **ctrl 0x26 (ctr0, LSB-only, mode 3) rides pit1/odd (mask=00ff)** at
0.398407 and 0.414740; the boot sequence's later pit1 words (0x66=ctr1, 0xb8/0xb4/0xb0=
ctr2) never re-touch ctr0's mode; the $2bca reloads ([8000], mask=00ff) ride pit1 ctr0
data. **Device right, counter right, mode right, count 255, clk 2.5MHz - and the
IRQ1/OUT0 cadence is 26.116ms = 255 x 256 input clocks.** Dave's counter-misalignment
arm dies with the interleave; the x256 is BETWEEN the measured inputs and the device
output: (a) the model's m_gate0 handling (the fw's handler "re-arms via gate0 + the
$8001 reload" - E800 bit9 toggles the gate per tick; a gate-freeze/retrigger interaction
could multiply the period), or (b) the MAME pit8253's mode-3 + LSB-only + gate semantics
as wired (set_clk, the out_handler edge condition in timer0_out: rising && m_gate0).
ONE C++/device READ closes it - every input is now on tape. Also note pit0/even receives
its own program (0x38/0x7a/0xb8 + ctr1 count 0x024a at $3ce4-$3d00 = the seek-settle
one-shot lives on PIT0 ctr1, not pit1 - the m_settle_out fake's real home).
FOR DAVE / NEXT: the timer0_out + gate0 + pit8253 wiring read with these inputs; then
the alignment fix; then the green-run. Owed: wr=1 stale clear.

cont.159 (2026-07-16) — THE ×256 DISSOLVES: ctrl 0x26 is MSB-ONLY, the tick is authentic.
Run287/288 (gate-vs-OUT frame Dave assigned): the run287 edit aborted (anchor miss) so the
gate probe never landed, but the OUT tape decides it anyway — PIT1-OUT0 half-period is
13.06ms from the INSTANT of the first fw programming at 0.398, i.e. the counter ran 65280
counts from its first load, before any gate-edge reload could have staled anything.
The decode check closes it: 8254 control word 0x26 = 0b00100110 → SC=00 (ctr0),
RW=bits5:4=10 = **MSB only** (LSB-only would be 0x16), mode 3, binary. The firmware writes
0xFF as the MSB → count 0xFF00 = 65280 ÷ 2.5MHz = 26.112ms — the observed 26.116ms tick to
within rounding. My PITWR tap's "LSB-only" label was a decode error; "count=255 → 102µs
expected" was built on it, and the whole ×256-defect frame with it. VERDICT: the pit8253 is
byte-faithful, the clk at 5801 stays, neither of the four-line arms (stale reload latch /
&&m_gate0 edge-eating) is a defect at the counter level. Dave's split is answered "neither
— the premise was wrong," which his directive's own doctrine (measure before wiring)
demands we honor over the directive itself.
COHERENCE CHECK: 70 settle ticks × 26.112ms = 1.828s = a textbook 5.25" cold-motor spin-up,
and exactly the old tapes' 7.96→9.79 settle-record expiry. The firmware is running a
1.83s motor-settle because it believes the motor just started.
REOPENED FRAME (Dave's own, cont.~150 era): "a cold-motor settle requirement presented to
a warm motor." Read1 ran 6.41→7.96 without paying 1.83s → its warm flag [$7a3e] was up;
read2's op-42 armed a fresh 70-tick record at ~7.96 → [$7a3e] read cold. The leg to catch:
who clears [$7a3e] (or never set it) between read1 and read2.
Run289 (c151): GATE0 edge log (whole run, cap 120) + WARM7A3E writer tap on $7a3e
(pc/data/mem_mask/time, cap 120). No fix until the tape names the leg and Dave adjudicates.

cont.160 (2026-07-16) — [$7a3e] IS THE HARD-DISK PATH; the floppy warm gate is an F000 HARDWARE READ.
Run289 tapes: WARM7A3E = six writes, ALL in the first 0.163s (RAM test 5959/5a5a at $9cea/$9d14 +
init clears $73e/$9d4a) — [$7a3e] is NEVER touched again, at either read or any expiry. GATE0 =
pc $2bc6 (drop) / $2bd2 (raise) pairs every 26.112ms from 0.44 — the IRQ1 handler's own mode-3
reload pulse choreography, alive and periodic; the tick machinery is healthy end to end.
STATIC (op-42, $6bc2, read direct): UIB flag byte [$12] bit1 forks the op.
  bit1 CLEAR (hard disk, flags $45): the $7a3e/$7a40/$29f8 settle-record machinery — cell $7a3e,
  value 1, count UIB[$18], flag-cell $7a40; UIB[$18]==0 → $7a3e<-ffff set-now (Dave's $6c40
  inversion, correctly placed HERE). This path never runs on our floppy flow — tape-consistent.
  bit1 SET (floppy, flags $37): (flags & $23)==$23 → mask = $0120, or $0220 if [$7a0a]!=0;
  **gate = (F000 & mask) == $0020 — bit5 (drive READY) SET, bit8 (or bit9) CLEAR.** Pass → $6cac
  done-exit (cancels any [$7986] timer via $2aba, arms pit1-ctr2 ctrl $9a mode 5, count UIB[$13]
  if [$796e]==-$10, returns 0). Fail → node[$18] word: nonzero → returned as code; zero → $fe =
  re-dispatch. THE SPIN = this gate failing, nothing else.
  (The (flags&$23)!=$23 sibling branch does btst #4 — cont.150's "f000.4 gate" reading came from
  there/17fe; the bit4 want-ready hack does not feed THIS gate. The "waits [$7a3e]" framing in
  cont.158 was the HD path misattributed to the floppy flow.)
cont.161: OP42POLL probe in the F000 read handler (pc $6bf0-$6c1a): full d + [$7a0a] + flags +
node[$18] per poll, both read eras. Names the failing bit (b5/b8/b9) and the mask choice. Run290.

cont.162 (2026-07-16) — READ1'S LOST 1.55s = OP-28 SEEK, NOT OP-42; the motor lifecycle fully decoded.
Run290 spine: op-42's FIRST-EVER dispatch in the whole boot = 7.9649 (read1's tail; "read1 6.41->7.96"
was never a completion - the monitor timed it out and read2 is the RETRY on the same node $71c6, st=08
second pass). 7.9649 + 70x26.112ms = 9.792 = the famous 9.79 fire. Both op-42 dispatches find
[$7a3e]=0 cold. OP42POLL=0 because UIB=$6e60 flags[$12]=$40, bit1 CLEAR -> the settle-record class
branch (my floppy-flags-0x37 read was a different UIB; the $0120/F000 gate belongs to bit1-SET units).
STATICS (seek op $6788 region + $1ab2 + $b8c): [$798e] = seek state (-1 = on-track/no-seek -> op-42
INSTANT DONE; 0 = settling; 1 = settled). For bit1-CLEAR units even the on-track path exits $798e=1,
so op-42 always reaches the [$7a3e] gate. THE MOTOR LIFECYCLE (all firmware, host-parameterized):
op-42 cold: E802 bit2 CLEAR (motor ON) + record{cell $7a3e, val 1, count UIB[$18]=70t=1.83s} -> warm;
at command completion $1aee stages [$7a42]=(UIB[$10]&0xF)x0x30000 main-loop passes (motor-off idle
delay), [$7a3a]=(UIB[$10]>>4)x0x30000; main-loop $b8c expiry -> E802 bit2 SET (motor OFF) + $7a3e=0.
$1b48 = abort path (cancels $7a38 record, clears all, ori #$c). Real-HW reconciliation: read1 pays
1.83s ONCE; the monitor retry rides warm and completes fast. Our failure: the spin-up armed 1.55s LATE.
RUN290'S REAL DIVERGENCE: **op $28 SEEK ($6788) spins $fe 6.4134->7.9622 waiting [$7a36]** - the whole
1.55s - completing exactly on a ~59-tick record expiry. Fw's seek-settle count = (steps x UIB[$19])/10+1
ticks ($6904); ~59 ticks implies a huge steps-x-rate product for a boot seek to cyl 0. Authentic
full-stroke restore vs step-engine-inflated = run291: W7A36 writer tap + RECQ ($736c-$73a3 queue-write
tap = every record's {count,value,cell,handler} at arm) + UIBTIME one-shot ($6e60 bytes $0e-$19).

cont.163 (2026-07-16) — THE 1.55s NAMED: HEAD-LOAD SETTLE, 60 TICKS; and the tick-rate indictment.
Static ($65bc-$661a): op-28's [$7a36] wait is armed at $65f0 = the HEAD-LOAD settle - clears E802
bit3 (head-load line), arms record {cell $7a36, val 1, count = (UIB[$14]&0xF) x 10, flag $7a38};
UIB[$14]=0x06 -> 60 ticks x 26.112ms = 1.567s = run290's 6.4134->7.9621 spin EXACTLY (walker write
pc $2b8a). UIB[$14]&f==0 -> $7a36=1 instant. Floppy-class (flags bit1 SET) skips (bne $6620).
Run291 UIBTIME: $6e60[$10]=0x10 (motor-off idle nibble=0 -> motor NEVER idles off once warm),
[$13]=0x18, [$14]=0x06, [$18]=0x46=70, [$19]=0x03.
THE INDICTMENT (three independent timeout classes, one clock):
  head-load 60t: 2.5MHz clk -> 1.57s (absurd) | 10MHz clk -> 392ms (plausible)
  spin-up   70t: 1.83s (long)                 | 457ms (textbook 5.25")
  seek settle steps x 3/10 t: 7.8ms/step      | ~2ms/step (textbook buffered seek)
The host monitor (works on real HW) staged these UIB values expecting a ~6.5ms tick. Model line
5845: set_clk<0>(10MHz/4) with its own comment "at the 8253's ~2.6MHz limit" - the /4 was chosen
for the 8253 ceiling; an 8254-2 runs to 10MHz. If the board's PITs see 10MHz, tick=6.528ms and
read1 completes in-patience (head+spin done ~7.3), no retry, no cold overshoot.
HARDWARE CALL FOR DAVE: is the part an 8254 and what does the clock tree feed it (CPU is 50MHz/4
= 12.5MHz; candidates 10MHz -> 6.53ms tick, 6.25MHz -> 10.4ms, 5MHz -> 13.1ms)? Differential
run293 (clk<0> -> 10MHz) measures self-test survival + read1 completion either way.

cont.164 (2026-07-16) — THE 10MHz DIFFERENTIAL (run293): the tick rate is load-bearing in BOTH directions.
clk<0>=10MHz took (OUT half 3.264ms, tick 6.528ms). Result: boot green (testend, no PARKTRAP) but the
floppy probe NEVER RAN - no doorbell-95 era, zero OPWALK/OPDISP, zero carries; at 116-119s the fw is
still churning cmd=87 INIT. Signature: STEPDONE -> FRAME-IRQ4 retry cycle every 2.8626s, forever - the
EXACT run13-17-era phase-mismatch loop ("model step completion lands outside the fw's inline poll
window"). The 2.86s period is UNCHANGED from the 26ms-tick era => that retry cadence is tick-independent
(CPU-counted or another timer), and the INIT-era stall mechanism at the fast tick is real but unmeasured.
Reading: at 26ms tick the model's fixed synthetic delays (seek_done 75ms, step quiesce, settle 1ms,
m_settle_out fake) happen to fit the fw's windows; at 6.5ms at least one no longer does. On real HW both
eras fit because the real drive's physical timings are what the fw was tuned for. The coherent end-state
per the LLE mandate: pick the tick from the real clock tree (Dave: is the part an 8254? what feeds it -
CPU is 50MHz/4=12.5MHz; 10MHz -> 6.53ms, 6.25MHz -> 10.4ms, 5MHz -> 13.1ms), then dissolve the model's
synthetic delays into real drive/pit timings (step 2-3ms class, head-load ~35-75ms physical, index/rev
200ms) so both INIT and read choreography fit the same tick, as they do on hardware.
ACTION: clk REVERTED to /4 (known-good baseline; run294 confirm). Differential banked. Next session:
Dave adjudicates the clock tree; then the INIT-era stall tape at the chosen tick (what the fw waits on
after each STEPDONE), then rescale the synthetic delays and retire the fakes (m_settle_out -> pit0 ctr1
mode-5 count 586; the OUT0-TEST wire; want-ready wr=1 stale clear still owed).

cont.165/166 (2026-07-16) — DAVE'S ADJUDICATION LANDS: THE E807 CHANNEL-OP PROTOCOL IS THE MISSING EVENT LAYER.
Dave: "the ops complete on physical events, and every one is falling through to a timer because the model
doesn't deliver the event... the watchdogs are never reached and the clock rate is a number nobody reads.
Drop the clock tree - the tick will have been irrelevant, exactly as the tick-independent INIT period told us."
THE HUNT (runs 294-296 + statics): host stages flags=$40 for the floppy AUTHENTICALLY (UIBCOPY source
byte [$12]=40; also unit labels were flipped in memory - $6c00/flags-$37 = unit0 = ESDI HD, $6e60 = unit2 =
floppy). $2aba (cancel) never writes cells; no cancel/complete callers for the settle flag-cells; IRQ3
([$72f8]) NEVER installed on this flow (VECINST: only IRQ5<-$29c0/IRQ6<-$298c at $32a6). The wait-for-ready
subroutine ($65b4-$6786) = watchdog {val $202d error, count $12c} + F000 hardware POLLING for the bit1-SET
class; head-load record arms only when [$7a34]!=0, and [$7a34] loads from THE COMMAND IOPB's byte7 bit0
($64c6) - all host-choreographed. THE MECHANISM ($27be/$283c/$24aa): fw queues 8-byte channel-op descriptors
{+0 param, +2 mailbox ($7ff0 ch0/$7ff8 ch1), +4 aux, +6 cmd&7} in the $737c ring (20 slots, head $7378/tail
$737a/count $7376); $283c dispatches: updates control shadow (mbox-$10 = $7fe0/$7fe8), pulses E802 bit6,
writes THE COMMAND BYTE TO E807; the gate array executes the PHYSICAL op on the drive's schedule and posts
the mailbox; IRQ2 ($24aa) matches desc (cmp mbox addr at $265e), acts (E806 write, status bits), advances.
Run291 caught descriptors armed at 6.4018 {cmd4, mbox $7ff8, param $010a} AND at 7.9753 (inside the
head-load spin) - the fw QUEUED the physical op; nothing answered.
THE MODEL GAP: E807 is entirely unhandled (grep: one comment). The stepdone/capdone heuristics (E800
pattern + quiesce -> $7ff0<-01 / $7ff8<-13) bypass the real protocol and only complete the op classes they
were tuned on; the settle-class channel ops fall to their tick watchdogs. THE FIX (Dave's "only fix"):
implement the E807 protocol - on command write, execute the physical op per cmd code on the drive's real
schedule, post the mailbox with the right status, raise IRQ2; retire the heuristics into it; leave the
watchdog clock rate alone (nobody reads it on healthy flow).
cont.166: census first - E807CMD tap (every command byte w/ pc) + OPRING tap (full descriptors at the
$27ee-$2800 arm pcs). Run296. Then decode cmd-code semantics and wire the settle-class completions.

cont.167 (2026-07-16, SESSION CLOSE) — THE JAM PROVEN; the whole stall is ONE unanswered channel op.
Run296 census (pc-skew-corrected descriptors {+0 param, +2 mbox, +4 aux, +6 cmd}):
  desc0 @6.4018 {cmd 4, aux 0,     mbox $7ff8, param $010a} -> E807 $0a @6.4018 -> COMPLETED (IRQ2
    match-action E806-clear @6.4042, pc $2668) - the model's boot-era heuristics answer this class.
  desc1 @6.4035 {cmd 2, aux $00ba, mbox $7ff8, param $010a} -> E807 $0a @6.4043 -> NEVER ANSWERED.
  desc2 @6.4067 {cmd 2, aux $00ba, mbox $7ff8} - queued behind desc1, never dispatched.
  desc3 @7.9753 {cmd 2, aux $01b1, mbox $7ff0} - the head-load-era op, queued, never dispatched.
  desc4 @8.1870 {cmd 0, aux $00ba, mbox $7ff8} - the retry's, same.
The ring only advances when IRQ2 consumes the head ($267a dec + $2684 re-dispatch via $27be flag=1).
$7ff8 write history: fw itself CLEARS it @6.40566 (pc $1cc2) at the 0x95 dispatch; ZERO writes 6.4057
-> 7.99 (grep-proven). The 60-tick head-load record firing at 7.9621 and the 70-tick spin-up at 9.79
eras are the WATCHDOGS on this jam, exactly per Dave's adjudication. Dispatch internals ($283c):
cmd&7 -> bit mask (1<<cmd rol.b 4: cmd0->bit4, cmd2->bit6, cmd4->bit0); cmd bit2 selects shadow
(-$c = $7fe4/$7fec) vs (-$10 = $7fe0/$7fe8); aux's masked bit -> set/clear; shadow low nibble <-
param low bits; E807 <- param low byte ($0a), E802-bit6 ack pulse when ring-head dispatch.
OPEN FOR DAVE'S 3030 DOC: cmd-code physical semantics (what IS ch-op cmd 2 with shadow bit6 per aux -
motor/head/select class per the settle context?) + the completion VALUE each posts (IRQ2's $2540 wants
the ff/ff/ff triple at mbox+2/4/6 for the SUCCESS steer; the $2632 BUMP path took e7/00/00 mailboxes).
FIX SHAPE (bounded): model services E807 dispatches; per cmd class, schedule the physical op on real
drive timing; post mbox {value, ff/ff/ff} + IRQ2; ring un-jams; watchdogs never fire; clock stays /4
untouched (nobody reads it on healthy flow). Then retire stepdone/capdone heuristics INTO this protocol.

cont.168 (2026-07-16) — THE JAM WAS REAL BUT NOT BINDING; the settle waits are truly timer-completed;
new hardware frame: ctr0 = WATCHDOG (pet + IPL7-reset), IRQ1 <- an unfound faster source.
Run298 (STORAGER_E807ACK=1, verified completion contract: mbox[0]=0x02 bit1 "channel-op complete" ->
$2510/$2634 ring-consume; stamp != last per the $24de gate): THE RING UN-JAMS PERFECTLY - desc1 answered
6.4243 -> desc2 dispatches instantly -> answered 6.4444 -> head 0->4 through the run. OUTCOME IDENTICAL:
desc3 isn't even ENQUEUED until 7.9753 (it belongs to the post-head-load flow), the 60t head-load record
runs its full 1.55s with the ring flowing, give-up at 8.2 unchanged. VERDICT: the channel-op ring and the
settle timers are SEPARATE mechanisms; the settle waits ($7a36/$7a3e/$798e) have NO event that beats them
- differential-proven, not just static. E807DISP context frames banked (shadows+E802/E804 per dispatch)
as cross-check material for Dave's static table. The E807ACK probe stays env-gated in-tree (off=baseline).
THE RECONCILIATION THAT REMAINS: host-staged counts (head-load 60t, spin-up 70t, seek 0.3t/step, watchdog
300t) demand a ~1ms-class tick for physical sanity; run293 proved the 2.86s INIT retry is tick-independent
(doesn't ride ctr0). BOTH fit ONE wiring: **ctr0-OUT is NOT IRQ1.** The fw's IRQ1 handler PETS ctr0
(0xFF reload at $2bc4 + gate pulse $2bbe-$2bce, every pass) = watchdog choreography; ctrl 0x26 mode 3
MSB-only = 26ms window; av7 = $26b4 = FULL REINIT (E800/E802, mailboxes, all four UIBs) = the classic
watchdog-reset target. IRQ1's real source is elsewhere - fw programs NO other periodic rate ($976-$98a
= B4/B0/B4 counter PARKING, no counts; no ctr1 control words exist; $1adc's B4 shows no count write) ->
likely the VGC7219 generates the tick internally (it owns interrupt generation per STORAGER-CHIPS.md).
Sibling manuals (2180/2190/4201 txt) = OCR junk, unusable.
FOR DAVE: (1) board photo trace - what feeds the IPL encoder? Is ctr0-OUT on IPL7 (watchdog reset) and
what drives IPL1? (2) the E807->shadow->E802/E804 static table (E807DISP frames in run298-error.log to
cross-check); (3) if IRQ1 = gate-array-internal tick, the model wire is: periodic IRQ1 at the derived
rate (from the host counts: ~1ms), ctr0 untouched (petted forever, never fires), ctr0-OUT -> IPL7.

cont.169 (2026-07-16, FINAL) — head-load arms from INTERNAL state (not the IOPB); the op-42 label is
suspect; the tick-budget arithmetic; 2190 manual datum.
Run299: W7A34 setters = $f56/$102a/$60f4 at the dispatch moment (6.4128-6.4132), NOT $64ce - staged
IOPB bytes "95 00 82 e7 02 00 00 00": byte7=00, the monitor stages NO head-load hint; the $64ce
IOPB-loader never runs. So [$7a34]=1 comes from fw-internal state at first-command time (plausibly
authentic: head genuinely unloaded at power-on). Read those three sites next session.
STRUCTURAL: op-42 is the NINTH op ($24 $28 $56 $58 $1a $18 $54 $4a $42 $36) - AFTER the data ops; in
run290 all four sectors were collected (7.99-8.08 carries) WHILE op-42 spun. A motor-spin-up gate after
successful reads is physically incoherent -> the cont.38 "spin-up" label is suspect; op-42 GATES THE
DONE POST (motor-state bookkeeping: clear E802 bit2 + record -> $7a3e=1 'motor known-on'; the $b8c idle
expiry does the reverse). The reads never needed it; the CMDDONE does.
2190 Users Guide p.25 (PDF readable; txt is junk): same-family firmware polls drive status "approximately
every 16 milliseconds" - a 16ms-ORDER background tick, corroborating the 26ms-order tick, NOT a 1ms one.
TICK BUDGET: monitor total patience ~1.79s (doorbell 6.41 -> give-up 8.2). First-command cost =
head-load 60t + (hunt, parallel) + op-42 70t ≈ 130t + mechanics. T=26.1ms: 3.4s FAIL (measured).
T=13.06ms (clk 5MHz = 10MHz/2): 1.70s - fits, marginally. T=6.5ms broke INIT (run293). UNTESTED: 2x
(5MHz). But the cleaner resolution may be op-42/head-load not needing their full counts on real flow
(events? the three $7a34 setters' conditions? [$7a3e] persistence across boots?) - Dave's statics call.
SESSION TOTALS: runs 287-299, cont.159-169. Keepers: E807ACK probe (env-gated, off=baseline). Tree
green at baseline (run299 try1). All new taps STRIP-tagged.

cont.170 (2026-07-16, THE CONVERGENCE) — THE TICK IS ROM-PROVEN; the bug moves ACROSS THE BUS.
Run300 (clk10M + E807ACK): INIT still stalls - E807ACK never exercised (stall is pre-ring). Run301:
STEPDONE guards ALL CLEAN at every delivery (node+26=0, 71b2=0, stamps differ) - the fw never enters
the consume branch. Run302: ZERO E802 writes in the 0.40-0.47 window; GATE0 stops at pc $a0ec =
**the boot timer self-test's FAIL path.** THE SELF-TEST ($a0a8-$a116, decoded): programs ctr0 (0x26,
0xFF00), pulses the gate, polls F000 bit11 counting loop iterations while OUT is high; PASS band
0xA00..0x4F000 iterations (~4.4us each on the 12.5MHz 68k) -> demands half-period >= ~11ms. At clk
2.5MHz (13.06ms half) it passes with ~15% margin; at 10MHz (3.26ms) it fails "too fast," error $6b,
park $367a. **clk = 10MHz/4 is AUTHENTIC, proven by the firmware's own acceptance band. The 26.112ms
tick stands. The fast-tick theory is dead - killed by the ROM.** (STORAGER_CLK10M knob stays for
reference; never set it.)
FINAL COHERENCE: at the true tick, first-command-after-power-on cost = head-load 60t (1.57s) +
op-42 70t (1.83s) = ~3.4s - REAL behavior for this drive class (FD-55F-family: head-loads per
command class, motor-managed; flags $40 host-authored; PCMX2-FLOPPY-ID: 360RPM 5.25" QD). Real boot
ROMs wait 5-30s for floppy spin-up. OUR monitor: doorbell 6.41 -> retry 7.98 -> give-up 8.2 = 1.79s
patience, ON SILENCE (no IOPB status ever posted in the window - checked; the fw posts nothing, the
monitor times out). 1.79s would fail against REAL hardware identically. The old 9.79s "ghost" = read1's
true completion time; in current runs no 9.79 expiry occurs because the monitor's 8.18 harddisk re-INIT
runs the $1b48 abort (cancels the settle records) first.
=> THE DEFECT IS THE CPUAP MONITOR'S PREMATURE TIMEOUT - NS32016-side emulated timing (its patience
loop / NS32202 ICU / RTC timebase running fast). The storager's remaining timing is EXONERATED.
NEXT SESSION: find the monitor's floppy-probe timeout mechanism (Dave's CPUAP disasm + cpuap.cpp;
memory: 20MHz VALVO osc confirmed, CPUAP_SCALE removed) and verify its authentic duration; predicted
green chain: monitor waits >= 3.5s -> read1 completes ~9.8 (or retry rides warm flags in ~300ms) ->
probe ladder proceeds to NSC Boot load.
Also banked this arc: restore (0x89) completes in 4ms via the mailbox INLINE/BUMP path (fw behavior,
mailbox-triggered) - the restore ladder never runs settle ops, so the READ always pays the cold cost;
consistent on real HW, reinforcing that the monitor must tolerate a slow FIRST READ specifically.

cont.171 (2026-07-16, TEST #1 CLOSED ON ALL BRANCHES) — the fw holds the DONE honestly; the restore is
exonerated by inertness; the monitor's deadline is ONE command, ~1.78s.
Statics: the ROM has exactly TWO IOPB status stamps - 0x82+errcode at $184e, 0x80 at $1a54 - one
publication chain ($17fe->$1966->$1a54->$1ab2 motor-off staging), reached only via the ladder's
completion stamp => the DONE is GENUINELY serialized behind op-42. The HLE's data-complete DONE was a
shortcut, not the fw's release point. Warm-flag lifecycle: [$7a36]/[$7a3e] persist across commands
(op-24 skips at $65d8 when settled; only the IDLE UNLOAD $b74 / abort $1b48 clear them) => the 3.4s
cold cost is a FIRST-TOUCH one-off, livable on real flow.
cmd-89 exoneration (run303, STORAGER_FWRESTORE): suppressing the C++ seek/restore host-post changed
NOTHING - the shim was already inert (fires at +75ms, finds the IOPB reused, retires STALE) because
the monitor doesn't wait for the restore at all. The fw's 0x89 = fast logical reset (no ladder; the
{24,26} builder at $5f74 serves other flows); the physical warm-up belongs to the FIRST READ's
virgin-path dispatch ($f46, [$d6]==-1) BY THE FIRMWARE'S OWN DESIGN.
Doorbell census (run303): SIX doorbells 6.4002-6.4051, NONE after => the monitor queued its burst
(87/87/89/87/95) and set ONE deadline; the 7.98 "read2" = fw-internal re-dispatch. MONITOR FLOPPY-READ
TIMEOUT = ~1.78s (6.405 -> 8.19), measured.
THE CONTRADICTION, FINAL FORM: fw needs >= 3.4s (ROM-proven tick x host-authored counts x serialized
DONE - all three links proven); monitor gives 1.78s (measured); real silicon runs the same fw and the
machine boots => the monitor-side 1.78s must differ on silicon (timeout longer / CPUAP timebase slower
in some emulated respect) OR a pre-6.4 warming touch exists that we don't emulate. TEST #2 (Dave's
ordering, now unlocked): the 1.78s deadline's provenance in the CPUAP monitor - delay loop @20MHz
(authentic -> hunt the pre-warm) vs NS32202-ICU / MC146818-RTC driven (compressible -> CPUAP bug).
Lives in siemens/PCMX2-CPUAP-DISASM.md + cpuap.cpp. The storager side of this investigation is DONE.
Knobs in tree (all env-gated, off=baseline): STORAGER_E807ACK, STORAGER_CLK10M (reference-only,
ROM-refuted), STORAGER_FWRESTORE (no-op, shim already stale). All session taps STRIP-tagged.

cont.172 (2026-07-17) — TEST #2 MECHANISM READ: the monitor NEVER TIMES OUT; the "1.78s deadline" was
my inference and it is DEAD. The poll loop (cpuap_fe.asm fe36f6-fe372a, NS32016): counter = 0xF00000
(15.7M) or caller-count x 0x30000; read the status byte (R6)=@0xF00800 over the Multibus; bit7 SET ->
done-exit; counter exhausted -> PRINT a timeout message (FE2CD0) + RESET the counter + KEEP WAITING
FOREVER. Cycle-counted, no ICU, no RTC - and unbounded. Arithmetic: 15.7M iterations x (8 insns +
Multibus crossing) = tens of seconds minimum - CANNOT fit 1.78s => in our runs the poll EXITS VIA THE
DONE BIT almost instantly. Six doorbells in 5ms = six instant false-DONEs. The monitor's give-up at
8.19 = its RETRY COUNTER exhausting (the -0x10(FP) boot-source index at fe44c2, label check 0x10B at
fe44b9), not a clock. THE REAL QUESTION: what does the model serve for the R0 status byte (host PIO ->
fw $7e00 window via host_win_r) and why does bit7 read DONE while the fw is still grinding. Run304:
CPUAP-POLL censuses (6.39-6.46 + 7.9-8.3 windows, STORAGER_POLL=1).
Dave's discipline note honored both ways: no ICU-divider tuning (there is no ICU in the path), and the
"pre-warming" branch is moot - the monitor doesn't need patience; it needs a TRUTHFUL BUSY.

cont.173 (2026-07-17, SESSION CLOSE) — the monitor's patience is INFINITE (rev9 fe3e85-fe3ec3 decoded:
doorbell GO -> spin-until-0x81 unbounded -> TBITB bit0 poll, counter exhaustion just PRINTS and re-arms);
the "1.78s deadline" is DEAD - the give-up is the fe44c2 RETRY LADDER cycling on instant false-DONEs.
ROM = rev9 (cpuap_monitor.bin == cpuap_rev9_monitor.bin; rev3's fe3exx disasm was out-of-sync data).
PC-sampled the silent wait (run305, Lua sampler in fboot-pcsample.lua): all samples in fe3e99-fe3ec3.
Status protocol: 0x81 busy (bit0 set) -> 0x80 done / 0x82 error (bit0 clear) - matches the fw's own two
stamps ($bf6/$1314/$1356 accept-0x81; $1a54/$184e complete). Monitor polls EXT(6)+0x22 = HOST RAM
0xfe782 (the FIXED status slot) + per-IOPB +2.
FALSE-DONE CENSUS (run306, all nine C++ 0x80-post sites line-tagged): L5498 x6 in the burst (non-95
instant DONE), L3553, L706 (dbs - posts the READ's DONE at data-complete 8.175 while the fw owes op-42),
L993 (seek shim), + L5889 = the fw-$7fe8-edge transcription leg (KEPT - that IS the hardware's job).
RUN307 (STORAGER_FWDONE gates all but L5889): gates verified firing (SKIPPED lines) - OUTCOME
UNCHANGED. Doorbell burst still 5ms; give-up still 8.2. => ONE MORE status path exists that the nine
sites don't cover. Candidates for next session: (a) the 3810-3813 accept-stamp block (writes 0x81 to
IOPB+2 AND fixed slot fe782 - who transitions it to bit0-clear?); (b) a read-through window serving fw
RAM ($71f0+2, where the fw's OWN $bf6/$1a54 stamps land) to host polls of fe780+; (c) L5889 firing early
on a fw $7fe8 write that isn't the real completion. Find the writer/reader of host 0xfe782 in the
6.400-6.404 window - a bus-side write tap on fe780-fe784 + a host_win-style read trace decides in one run.
THE END-STATE (unchanged, now fully specified): monitor has infinite patience; fw posts honest
0x81->0x80; the model's only job is transcribing the fw's stamps + the accept; every premature C++
DONE dies; the read's true DONE ~9.8 arrives; boot proceeds. Knobs: STORAGER_FWDONE joins E807ACK/
CLK10M/FWRESTORE (all env-gated, off=baseline; baseline still green run307-try1).

cont.174 (2026-07-17, SESSION CLOSE) — THE LAST LIAR LOCATED: the FIXED-SLOT posts (0x0fe782/3 <- 0x80),
outside the FWDONE gates.
Run308 (fe780-fe787 both-direction taps, Dave's sharpening): the monitor's IOPB is AT host 0xfe780
(build fe3c01-fe3cd7, cmd stamp fe3e6d, options fe3e7f); the poll (fe3e90 accept-spin + fe3ebe TBITB)
reads 0xfe782 and the tape shows 0x8181 (busy) flipping to **0x8080 at 6.400542 WITH NO LOGGED WRITER**
on either tap. Run307 cross-check: ALL nine gated posts SKIPPED at 8.17-8.26, L5889 never fired - yet
the monitor still advanced to "no sys-floppy" at 8.2 => an UNGATED writer exists. FOUND (source read):
the shim sites continue past the per-IOPB {+2,+3} pair with FIXED-SLOT writes - dbs.write_byte(0x0fe782,
0x80) at the L706-region (and the 3810-3813 accept block writes 0x81 to BOTH per-IOPB and fixed slot) -
and the FWDONE wrapper closed after the "+3" line, leaving every fixed-slot 0x80 write OUTSIDE the gate.
The monitor's actual poll target IS the fixed slot (EXT(6)+0x22 = fe782, run308-confirmed). (Why the
bus-side tap missed those writes remains unexplained - audit the tap/space routing next session; the
CPU-side read tape is authoritative regardless.)
NEXT SESSION, FIRST MOVE: extend the FWDONE gate to EVERY fixed-slot 0x80/0x82 write (grep 0x0fe782 /
0x0fe783 in storager.cpp), keep the fw-edge transcription (L5889 + its fixed-slot mirror if the real
gate array mirrors it - decide from the fw's $bf6 protocol), re-run. Predicted: the monitor spins
0x8181 at fe3ebe through the fw's honest 3.4s first-touch, the fw's $1a54/$7fe8-edge DONE lands ~9.8,
the label check sees real VOL1 data, and the sys-floppy probe finally proceeds. The monitor's patience
is infinite (cont.173); nothing else is owed.

cont.175 (2026-07-17, MILESTONE) — THE MONITOR'S REAL COMPLETION PATH RUNS FOR THE FIRST TIME.
Statics (Dave's faithfulness check): the fw NEVER writes host RAM - $bf6 stamps 0x81 at ring+2
(A2=[$7b20], dual-port window; $c06 fast-reset 0x80 there), $1a54 stamps the local IOPB copy ($71f2
for the live command). Host delivery = the bus interface's write-through of those stamps => the
mirror is TRANSCRIPTION of real fw writes, LLE-honest.
Run309 (FWDONE: fixed-slot 0x80 pairs gated + transcription taps on $71f2 + $7e22 mirroring fw stamps
{80,81,82} -> host iopb+2/3 + fe782/3): **screen shows "READ -compl-stat: 13 sensb: 80, Cyld=0 Head=0
Sect=0 Log. Blocknr=0" - the monitor ran its completion evaluation AND ISSUED A SENSE for the first
time in 300+ runs.** 25 mirrors fired. The false-DONE fabric is broken open; the host-controller
dialogue is real now.
REMAINING (next session): compl-stat 0x13 = the wrong value transcribed (likely $7e22 carrying the
mailbox GO 0x13, not a status stamp - my fixed $7e22 tap approximates the REAL ring slot, which is
ring+2 off [$7b20] and MOVES per command). Refine: tap ($2 offset of [[.$7b20]]) dynamically, or tap
the $bf6/$c06/$1a54 pc-sites' writes directly; transcribe only genuine status stamps. Then the sense
path (fw's sense data delivery) gets its own look. ALSO OWED: the bus-side write tap silently missing
writes (cont.174) - Dave: LANDMINE, fix the tap/space routing before trusting it again.
Knobs: FWDONE now carries the gates + transcription (env-gated, off=baseline-green).

cont.176 (2026-07-17, SESSION END) — transcription pc-gated + slot-dynamic; bus tap self-tests (FIRES);
the remaining wrongness = CROSS-COMMAND SMEAR in the mirror's binding.
Run310: BUSTAP-SELFTEST logs per-run (fires when probed through the shim path - run308's misses took
some other path; the instrument now announces itself, never fails quietly again). Transcription refined
per Dave: WHEN = pc-gated ($bf6/$c06 ring-stamps, $1a54/$184e local-copy stamps); WHERE = follows
[[$7b20]]+2 dynamically (the fixed-$7e22 trap retired). 9 mirrors (down from 25 - gating correct).
STILL "compl-stat: 13" + a second sense (sensb: 82). DIAGNOSIS: (a) compl-stat = the monitor scraping
the ring/mailbox via PIO where 0x13 = the UNCONSUMED GO of the never-finished read - stale state, not
a status; (b) what breaks the busy-spin = a fast command's HONEST 0x80 stamp mirrored while
m_iopb_addr already points at the READ's IOPB - a true stamp delivered to the wrong command = the
CROSS-COMMAND SMEAR. FIX (next session): bind the mirror to the command the FW is completing - the
fw's own IOPB backlink ([$7a06] / the node's host-iopb field), never the model's latest doorbell
latch. Static-binding = the same wrong-because-static trap in its fourth costume (head-refill word
read, fixed count, fixed slot, now fixed binding).
STATE: FWDONE fabric = gates + pc/slot-gated transcription (env, off=baseline-green). The monitor's
completion+sense machinery is exercising real dialogue; the conversation is honest; one binding from
the label check. Runs 287-310 = cont.159-176.

cont.177 (2026-07-17, FINAL) — NO SMEAR: the host reuses ONE IOPB (0fe780, every mirror); the dialogue
is honest END TO END. Run310 mirror tape: 80@6.413 (INIT), 80@6.427 (RESTORE), **82@8.025 (the fw
HONESTLY ERRORS THE READ)**, 80s@8.026-8.101 (the monitor's SENSE commands completing honestly), 82s
@8.125/8.182/8.226 (retries erroring). The monitor's full error machinery is running a REAL
conversation. Binding fix moot; Dave's "latest-X" AUDIT still owed (rule: any model state meaning
"the latest X" instead of "the X the fw is acting on" is a cross-command landmine - sweep the model).
NEW FRONTIER (single question): why does the fw error the read at 8.025 (~35ms after op-28's head-load
completes at 7.99)? $17fe stamps 0x82 with node[$18] = the error code. ONE TAPE: node[$18] at the
$184e execution + which op/record stored it ($20xx-family watchdog vs an op's stored code). Existing
taps (RECQ/OPWALK/OPDISP) cover the suspects. Then: fix whatever hardware event the fw is missing
(per the season's rule: deliver the event, never fake the flag), and the read completes to $1a54.

cont.178 (2026-07-17, FINAL) — THE ERROR CODE NAMED: $69 at pc $8318 = the $82e2 PENDING-SCANNER'S
"no f0 stake found" exit (cont.38jj vocabulary: $f0=WANTED; none -> error $69). Stored 8.025173,
stamped 0x82 254us later. Dave's lean CONFIRMED at exactly its stated strength: not the data path -
the f0 STAKING of the data's arrival. The frontier returns to the season's deepest subsystem - the
f0 verify-stake machinery (cont.114 read-dispatch chain, cont.135 carry strobe, cont.117/127 keepers)
- now evaluated under an HONEST fabric where every completion is fw-authored and the monitor retries
for real. Question for next session: at 8.025, were the f0 stakes (a) not yet landed (race: carries
run 7.99-8.08, scan at 8.025 mid-stream - why does the fw scan before its own completion event?) or
(b) never planted (the cont.135 strobe path inert in this flow)? The ledger dump at the $8318 moment
+ the stake-write tape (existing SECMAP taps) decide in one run. Then: deliver the event that stakes
honestly (never plant the flag from C++ - the season's rule, now with the fabric to honor it).
Runs 287-312 = cont.159-178. The dialogue is honest; the machine tells the truth; the truth says $69.

cont.179 (2026-07-17, SESSION END) — THE DUMP SAYS ABSENT. LEDGER@69 (8.025173): c0 c0 ff x7 fe x17
aa - NO f0 anywhere; aim [$7428]=$00fe, [$7430]=0104, [$79a8]=0008 (transfers owed), [$74ac]=74d4.
The ff/fe pattern = the LEDGER-INIT's own writes at 7.9902 (SECMAP: pc $6fe6/$7068, the $6f44-chain
map-fill) - NOT consumption. Between init (7.990) and scan (8.025): ZERO stake writes, while the
carries delivered on schedule (R=15@7.9875, R=1@8.0125, R=2@8.0250 - 173us before the scan). VERDICT:
ABSENT STAKE - the byte arrives, nothing marks the ledger. The fw's f0-stamper (the $92b4-happy/$9318
capture-completion bookkeeping, cont.38z/38jj vocabulary) never runs in this flow. Dave's branch (a):
the withheld event = the data-record completion that drives the fw's OWN stamper. NEXT SESSION: decode
what gates the stamper ($92b4's [$742c]==0 happy branch / the $9318 entry conditions) in THIS flow and
deliver the hardware event it waits on - never write f0 from C++. The second LEDGER@69 (8.0378, retry):
all-c0 + fe tail = a FRESH re-init, same absence - the retry reproduces the miss deterministically
under the honest fabric, which makes the stamper-gate tape clean to take.
Runs 287-313 = cont.159-179. Season state: dialogue honest end-to-end; the truth says $69; $69 says
"stake me honestly."

cont.180 (2026-07-17, TRUE END) — reach-vs-gate: the EXISTING tape partially answers, leaning GATE.
Run313 read-era (7.98-8.03): ARMTYPE idtyp=0 x2 among 17 (TWO data-typed arms - the alternator DOES
leave ID; reach occurs) but 7950=0000 at all 17 logged arm-instants (type classifier vs phase bit
disagree - note for the entry tape). [$742c]=0001 at the DLVCHAIN delivery instants bracketing the
scan (7.9999/8.0012/8.0126) - the $92b4 happy branch (needs ==0 per the season decode) would fall to
$92fc and stake nothing. LEANS GATE (inverting Dave's reach lean) - WITH cont.38z's reinterpretation
flag attached: "[$742c]!=0 may be the NORMAL per-sector path; the fail-path reading inherited
$742c=fail - audit like premise 8." If that holds, neither reach nor gate - a third path stakes f0.
OWED (next session's opening tape): the $92b4 ENTRY frame - pc-gated tap on its own tst $742c: entry
instants, [$742c] at each, branch taken. Delivery-instant values are NOT entry-instant values - this
arc has paid for that distinction (pc-skew, prefetch-shadow) before.
Runs 287-313 = cont.159-180. The stake goes in by the firmware's own hand or not at all.

cont.181 (2026-07-17, ABSOLUTE END) — THE FOURTH DOOR: the stake is planted NOWHERE. Run314 entry
frame (at $92bc's own pc): ONE entry @8.025014 (159us before the $69), [$742c]=1 -> the !=0 branch ->
$9312 stakes f0 at ($7654 + [$7428]) with **[$7428]=$00fe** = the terminator sentinel = $7752, outside
the ledger the scan walks. Not reach (entered), not gate (the !=0 branch IS the staker), not a third
path - THE AIM WAS NEVER LOADED. 7950=0000 even at entry (router note). [$7430]=0104 (suspect too).
NEXT SESSION: who loads [$7428] with the wanted sector index on healthy flow ($82e2's found-arm, the
$8114 [$7430]->[$7428] copy, the id-accept bookkeeping) and what event drives the loader. The season's
poison-saga cells ($7428/$7430, cont.38y/39c/39d) return as the frontier - now under an honest fabric,
with a deterministic reproduction and the entry frame as yardstick.
Runs 287-314 = cont.159-181. The stamper is honest; the aim is empty; load the aim by the fw's own
hand - the season's rule, one cell deeper.

cont.182 (2026-07-17, SEASON CLOSE) — THE POISONER CAUGHT BY NAME. W7428 chain (run314): 0001@pc70e4
(7.99026 - the op-$4a loader runs and loads CORRECTLY: wanted index 1) -> 0044@pc8114 (7.99997 - THE
POISONER: the $810e-continuation's [$7430]->[$7428] copy delivering $7430's overrun residue - the
cont.38y poison, red-handed) -> 00fe@pc7e0e (8.01376 - the $7e-scan advances the poisoned aim to the
terminator) -> the stake lands nowhere -> $69. The aim was loaded right and CLOBBERED 79ms before the
kill. NEXT: one cell up - [$7430]'s writers ($7e6c/$7e8a scan), why it holds $44/$0104 (the
"terminator-less map overrun into the geometry table" of cont.39c - but run313's ledger HAS its aa
marker; re-examine), and what the $8114 copy SHOULD deliver on healthy flow. Same instrument, same
discipline: the branch decides, not the lean. The season closes on its own beginning - the poison
cells, with sight.
Runs 287-314 = cont.159-182.

cont.183 (2026-07-17, FINAL WORD) — [$7430]'s writer census was already on tape: $7e8a ALONE (the
$7e6c-scan store, cont.39d pen #1). THE ORDERING FACT: 0044 stored at 7.98959 - BEFORE the ledger
init (7.9902) - the scan walked a PRE-INIT map and stored residue; $8114 then copied that stale $44
over the freshly-correct aim (loaded 0001 at 7.99026, poisoned 7.99997). Second store 0104@8.0138,
same pen. Dave's two framings INTERSECT: the copy fires after the loader (the WHEN) carrying a value
from before the map existed (the WHAT). NEXT CELL = the ordering: on healthy flow does the $7e6c scan
run pre- or post-init, and what arms it at 7.98959 (an event delivered too early? the scan's own
trigger mis-sequenced?). Yardstick = the five-line timeline: 7e8a-store / init / loader / poison-copy
/ second-store. Same rule, one cell higher: keep the SEQUENCE honest by the fw's own hand - deliver
events in the order the hardware would, never reorder from C++.
Runs 287-314 = cont.159-183. Season closed: the fabric is honest, the poisoner is named, the residue
is dated, and the next question is a single ordering.

cont.184 (2026-07-17, LAST TAPE) — THE TRIGGER'S SPECIES: sr=2700 at both $7e8a stores - the $7e6c
scan runs MAINLINE under a full interrupt mask (the move #$2700,SR critical-section idiom), NOT in
any interrupt handler. Dave's capture-strobe lean takes its priced-in hit: no IRQ delivers this scan;
a mainline op calls it pre-init. INSTRUMENT ARTIFACT (own-medicine note): a7 read 000000 and the
"stack" dump returned the vector table - M68K_A7 under supervisor mode is the wrong alias; use
M68K_SP/ISP. The caller chain is ONE CORRECTED REGISTER away. NEXT SESSION OPENS: rerun the SCANTRIG
tap with the honest SP, read the stacked return addresses, name the mainline caller that runs the
scan before the init - then re-order by the fw's own flow (deliver whatever event/gate the caller
checks, in hardware order; never reorder from C++).
Runs 287-315 = cont.159-184. The season ends mid-stride, instruments honest, one register from the
caller's name.

cont.185 (2026-07-17, THE LAST FRAME) — the corrected stack (M68K_SP) names the chain: store#2's
returns = $6bc2 (OP-42'S ENTRY - the scan's containing routine returns INTO it: the fall-through/
tail-call ending at ~$6bbc) and $156a (the STATUS-8 LAUNCH HANDLER, $222 table), with 000c (status-c
word) + $6e60 (UIB ptr) as data. Store#1's frame = locals (scan running deep). THE SCAN RUNS INSIDE
THE OP-WALK'S OWN LAUNCH CHAIN, mainline under the $2700 mask - the ordering fault is INSIDE the
ladder: some launch-chain routine (ending ~$6bbc, called from $156a's body) runs the $7e6c scan
BEFORE the op that inits the ledger. NEXT SESSION (bounded statics, frames as ground truth): read
$6bb0-$6bc2 (the routine ending at op-42's entry), the $156a call site, the scan's containing entry;
name the caller's wait; then order the flow by the fw's own hand - the season's rule at its final
depth: never let the model's timing run the firmware's flow ahead of itself.
Runs 287-316 = cont.159-185. Season closed mid-decode with the chain IN HAND: every actor honest,
the fault a 600us ordering inside the launch ladder, both stack frames banked as ground truth.

cont.186 (2026-07-17, THE THREE READS) — (1) The routine ending at $6bc0 IS the $c0 MAP-INITIALIZER
(move.b #$c0,(A2)+ dbra loop under $2700, SR restore $6bbc, rts $6bc0) - init and scan share the
launch chain. (2) $156a = the status-8/status-c stamp dispatcher (walks $721a queue, stamps node+$26
c-or-a, extends into the $192 OP TABLE dispatch at $15b0). (3) THE $7e8a STORE IS THE SEQUENCE-ADVANCE
(cont.38s): $7e58 cmp [$7428] vs delivered r; on MATCH: reload batch window ($79a2->$79a4), step the
aim past consumed/ff entries to next unconsumed, store at $7e8a. At 7.98959 the advance ran ON A STALE
AIM OVER A PRE-INIT MAP - consuming against read1-era state before the $c0 fill + fresh load existed.
AMBIGUITY NAMED NOT RESOLVED: stacked {0000 6bc2, 0000 156a} = return addresses OR the walker's saved
next-op/status-entry POINTERS in stacked registers - the two readings place the scan's caller
differently. NEXT SESSION MOVE #1: resolve it (one more frame with more stack depth, or the static
call-graph of the $7e50 round's entry). Then: what should be true at the advance's ladder position
(a fresh aim + inited map) and which model input lets the round fire early - never reorder from C++.
Runs 287-316 = cont.159-186. Season truly closed: three reads done, one ambiguity honestly parked,
the rule whole: the ladder is honest; make the model's inputs meet it in hardware order.

cont.187 (2026-07-17, RESOLVED) — THE DEEP FRAME DECIDES: at sp+12 both frames carry an EXCEPTION
FRAME {SR=2010/2000, PC=$a424/$6bdc} with a return to $15bc (the op-walk jsr site) beneath. The
sequence-advance runs INSIDE AN INTERRUPT HANDLER that raises the mask to $2700 internally - the
"mainline" reading was the handler's own mask; the stacked SRs show IPL0/1 interrupted MID-WALK
(frame2 interrupted INSIDE op-42's body at $6bdc). Shallow {6bc2,156a} = the handler's saved register
pointers - both prior readings half-right, the frame whole-right. VERDICT: an IRQ5-class data-round
delivery at 7.98959 interrupts the launch ladder BEFORE the $c0 init + aim load - Dave's ORIGINAL
capture-strobe lean, surface-refuted by sr=2700, resurrected by the exception frame beneath it. The
model presents a matchable data event against a ladder still mid-launch; hardware would not - the fw
hasn't armed the round. THE FIX'S SPECIES, named by the stack: strobe delivery must follow the fw's
arming state (the E802/window/serdes gating at delivery time), in hardware order. NEXT SESSION: which
strobe fired at 7.98959 (the serdes mark tape at that instant), what fw arming state it should have
been gated on, and the gate - delivered by the fw's own signals, never a C++ reorder.
Runs 287-317 = cont.159-187. The season ends with the fault's species named by the machine itself:
an event delivered before its arming - the thesis, proven at the last cell.

cont.188 (2026-07-17, THE TAPE NAMES BOTH) — THE STROBE: a serdes mark interrupt @7.989089 (DLVCHAIN
26a8 sr=2510 -> $29c8 pump [$7950]=0101 -> OPH2/$7ba8 -> $7e66 batch reload -> $7e8a advance) with
D800<<1=0000 and 7dac all-zero - NO record behind it: a mark riding the PREVIOUS era's stream enables,
the serdes rotation free-running across the command boundary. THE ARMING STATE: the fw's FRESH
per-command arm (the $88ac-chain E802 bit15 writes - on tape only AFTER this instant). The model's
mark gate rides persistent serdes-active/enable residue; hardware's window opens at the fw's arm.
THE SINGLE GATE (next session builds it): mark delivery honors the CURRENT command's arm - the
cont.117 latch-at-arm class, extended to the delivery side. The fw's own signal, hardware order,
no C++ reorder. Dave's method note banked verbatim: a refutation deserves the identical skepticism
as the claim it kills - the strobe species was right, its refutation was the misread.
Runs 287-317 = cont.159-188. SEASON END. The chain, complete: stale-enable mark -> pre-arm round ->
stale-aim advance -> poisoned copy -> nowhere stake -> honest $69. One gate closes it all.

cont.189 (2026-07-17, THE GATE'S FIRST CUT) — run318 (ARMGATE): THE POISON CHAIN IS BROKEN (40 marks
withheld; the 8.025 error-82 GONE from the mirrors) AND the read never completes (no 80/82 by 120s;
monitor honestly waiting forever). Dave's over-correction discipline fired exactly as priced: the
per-command arm gate blocks the stale mark AND the live ones - possibly a deadlock if the fw's first
$88ac arm itself follows a mark. THE REFINEMENT (next session): read run318's arm-write timeline
("LLE READ arm" lines) vs the withheld marks - did the $88ac arm ever fire? If never: the fw's arm
depends on an earlier signal the gate must key on instead - the $a6d window-open (cont.39m: "the fw's
$a6d write (bit11-unique) opens the window") is the standing candidate; the discriminator between
stale-era and live-era marks is finer than "armed since doorbell." Same rule, finer signal: the fw
tells us when the window opens; read THAT.
Baseline (no ARMGATE) unchanged-green. Gates: FWDONE + ARMGATE both env-gated, both off = baseline.
Runs 287-318 = cont.159-189. The season ends one refinement from the chain: poison provably broken,
delivery gate provably too wide, the finer fw signal named as the next read.

cont.191 (2026-07-17, THE GATE HOLDS) — ARMGATE v3 (engagement-keyed, per spec §3.1/cont.39n): run320
= the correct species. ENGAGED at 7.989407 (the fw's own $a6d write, ~300us after the stale mark's
old slot); only 2 marks withheld (surgical: stale blocked, live admitted); THE POISON CHAIN IS DEAD
(no $69, no 8.025 error). The read ran 800ms FURTHER and failed at **$2029 (pc $7e08) = the
batch-window expiry** (cont.38ff's id-side fork): the hunt hunts honestly for its full [$79a4]
window; the DATA-PHASE CONSUMMATION never happens - the season's OLDEST frontier (cont.38ee/38ss:
the data-record event on the retained id; for the $796e==0 class the data phase = the synchronous
E000 pull + bit15 re-arm, NOT IRQ5 marks) now reachable with an honest fabric and a surgical gate.
Gate lineage: v1 E802-bit15 (mark-circular, run318), v2 E000 arm codes (boot-only, run319), v3
engagement (HOLDS, run320) - each killed/kept by its own tape, per the method.
Screen: the monitor moved to "waiting for harddisk ready" (no floppy retry visible) - the give-up
flavor changed with the honest 82s at 8.81/8.85.
NEXT SESSION: the data-phase delivery under the gate - the $7950 phase toggle surviving an id round
(the $8a42 no-re-arm exit), the E000-pull serve, the four-stakes machinery - all decoded, waiting.
Runs 287-320 = cont.159-191. The chain's head is gated; the next link is the data phase.

cont.192 (2026-07-17, THE TOGGLE IN CLEAN AIR) — run320's tape, 7.99-8.81: the $7950 toggle SETS
(pump $29c8/$2994) and BOTH cont.38ee clearers kill it within ~100us, EVERY round, 645 times across
the honest 800ms hunt: $88d6 (the re-arm pen) and $8a3e (the count-branch pen). The $8a42 no-re-arm
exit NEVER runs. THE FROZEN CELL: [$7a0c]=0003 in every frame - cont.38ee's "HD trigger = $7a0c
count; the floppy trigger = the one unmeasured element" - the count never decrements through 645
rounds; the fork's condition never ripens. NEXT (bounded statics, 645 rounds as yardstick): the
$8a30-$8a50 fork condition + [$7a0c]'s decrementer - what ripens the lock-on-complete exit on the
$95 floppy flow. The data phase consummates when the toggle survives; the toggle survives when
$8a42 runs; $8a42 runs when the fork's condition - read it, don't lean it.
Runs 287-320 = cont.159-192. Frontier: one fork's condition, with the spec as reference.

cont.193 (2026-07-17, THE CONDITION READ) — THE FORK'S CONDITION: $8a14-$8a30 = the captured record
at $7dac must read A1 A1 A1 FE FF (sync run OR'd == $a1; IDAM $fe; ID-VALID MARKER $ff) - qualify ->
$7a0c--; THREE CONSECUTIVE well-formed IDs -> $8a42 lock-on exit, toggle survives. Writers: $6ab2
seeds 3; $8a0e clears; $8a32 (in-fork) decrements. THE STARVATION, on existing tape: every era frame
shows 7dac[0:7] = a1 a1 a1 fe 00 ... - BYTE 4 = 00. **The 0xFF ID-valid marker - the season's FIRST
keeper ("[A1 A1 A1][FE][FF] c h r n", the idcap_tick insertion that made $89f2 lock-on pass in the
run251 era) - is MISSING from the staged records.** The check fails 645/645; the decrementer never
runs; [$7a0c] freezes at 3; the toggle dies every round; the data phase never consummates; $2029.
Dave's lean lands as STARVED - but on a MODEL REGRESSION, not a missing event class: the staging path
dropped the FF somewhere in the last ~70 continuations (candidates: the stream rebuild eras, the
cont.117/135/150 keeper interactions, the honest-fabric refactors). NEXT SESSION: grep the model's
id-record staging for the marker insertion; find where byte 4 stopped being $ff; restore the keeper;
then the chain - lock-on -> toggle survives -> E000 pull -> f0 stake -> scan -> $1a54 -> VOL1.
Runs 287-320 = cont.159-193. The frontier is a regression hunt with a one-byte signature.

cont.194 (2026-07-17, HANDOFF NOTES from Dave) — (1) INSTRUMENTATION: Lua WRITE taps get wiped by the
device's handler reinstalls (opcode read-taps survive; PHASELOG/in-model logerror = the reliable
channel) - prefer C++ install_write_tap; a silent Lua write tap = suspect the wipe before the theory.
This likely explains run308's bus-tap silent miss (the cont.174 landmine). (2) MODEL GAPS for the
next touch: map the C000 bank; retire C800's host-address role (C800 = the DMA/scatter-gather
programming file per spec §3.4/§6.6 - the residual host-address use is a leftover to dissolve).

## ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★ cont.260 (2026-07-19) — THE READS ARE IN NATIVE SECTORS; the label read COMPLETES byte-perfect (READVERIFY OK)

**Dave's decisive question settled the whole size saga: "closely examine cylinder 0 head 0 and head 1's data — is there truly data on all 32 sectors?"** Decoding `siemens/set1/mx2-001.imd`:
- **cyl0 head0** (FM, 16×128B): R1-6 = 00, **R7 = `VOL1SINIX0`** (ANSI volume label), **R8 = `HDR1 NSC Boot`** (file header), R9-16 = 00.
- **cyl0 head1**: **all 16 sectors are `0xe5`** — UNFORMATTED. There is no data on head 1 at all.

So the label read is NOT 32 sectors across two tracks. **The count is in NATIVE SECTORS, not 512-byte blocks** (Dave: "the reads are native sectors"). The IOPB count 8 = 8 native sectors = logical 0-7 = R7..R14, entirely on cyl0 head0, holding VOL1+HDR1. The firmware's `[$7956]=8` decrementing per sector and completing at 8 was **correct all along**; the model had OVER-sized the window.

**cont.256f's `ask×512` (4096B / 32 sectors / 2 tracks) was the error** — it mistook the HLE's harmless over-delivery of the blank head 1 (`count×sec_per_blk`) for the ask. Sizing the window to 4096 made `m_read_hostmap` wait for 32 sectors that don't exist, so `DESCDONE` never fired and the fw — done at `7968=1` — hung waiting for a channel completion. The whole "premature completion / head switch / per-block cadence" investigation (cont.258-259 head-switch fixes, the F000-bit6/PAIR attempt) was chasing a phantom created by the oversize; the head-switch code is now dormant (single-track read).

**FIXES (cont.260, KEEPERS):**
1. `s_desc.total = ask × the live sector size` (`m_unit_secsize[2]`), not `× 512`. Generalizes to any unit's native sector size — 128B FM, 256B MFM, and (to verify at the ESDI section) the HD's 1024B.
2. DESCDONE-POS coverage: treat bit 0 as covered — the read's first sector (VOL1) is always delivered as a read-ahead a few µs BEFORE the model detects DESCARM (which resets the bitmap), so its coverage bit was lost and the window sat at `0xfe` forever.

**RESULT (run348):** `DESCARM total=1024`; 8 trucks (host+0..+896, VOL1/HDR1 + 6 empty, byte-exact); **`DESCDONE-POS 1024 bytes / 8 blocks`** and **`READVERIFY OK: 1024 bytes / 8 sectors match the media exactly`**. The read data phase is DONE.

**NEXT FRONTIER:** the read completes but the CPUAP monitor stays at "testend" — the read's `0x80` completion status is not reaching the host IOPB (the read-era CMDDONE shows no host post). Plus a read-ahead 2nd DESCARM at host+896. So the frontier moves past the read to: post the command completion to the monitor → the monitor parses VOL1/HDR1 → issues the boot-file read. Also: the ESDI HD path must be checked against the native-sector rule (1024B sectors) when reached.

## cont.262 (2026-07-20) — THE DROPPED KEEPERS RESTORED + THE FM DATA PATH RESOLVED (IRQ5→$7ba8, not $92b4)

**Dave's frame: completion WAS solved; find what we stopped doing.** The env-freeze (cont.255l)
rebuilt `storager_getenv`'s lists and silently DROPPED three keepers. Restored to passthrough:
- **STORAGER_FWDONE** (cont.175 KEEPER) — the honest stamp transcription (fw's own $1a54/$184e
  writes to its local IOPB copy $71f2, mirrored to host slot fe782). With it, the monitor's
  completion+sense machinery runs again (screen advances past "testend" to READ compl-stat / sense
  / "going to harddisk"). Without it NOTHING posts status (NOBYPASS-gated direct posts are dead).
- **STORAGER_ARMGATE** (cont.191 KEEPER) — engagement-keyed on the fw's own $a6d window-open write;
  kills the $69 poison (blocks the stale free-running mark that lands before the fresh arm).
  Reproduced cont.191 exactly: engaged @7.9894, ~2 marks withheld, no $69.
- **STORAGER_IDFF** — RED HERRING for this read. It feeds the MFM `a1a1a1fe`**`ff`** lock-on
  ($8a14), which $89fe gates OFF for FM (UIB[$12] bit1 clear -> beq $8a38). The label read is FM
  (cyl0), so IDFF/$7a0c/$8a42 never apply. Dave: "look for the address mark in FM, not MFM."

**BYTE/WORD PHASE BUG (fixed):** the alternator's `bchg #0,$7950.w` is a BYTE op on the big-endian
HIGH byte, so the phase bit is **bit8 of the word** (0x0100), phase=1 = $0101. Prior phase-hold
code using `& 1` / `|= 1` touched the low byte the alternator never reads. (Doctrine trap cont.135
warned of.)

**THE DEADLOCK (mapped):** no f0 stake -> ledger stays `ff` -> $7e8a aim-advance always finds
position 1 as next-unconsumed -> aim [$7428] stuck at 1 -> the want-match ($7e58 cmp $7428,r) fires
once (aim=1) -> [$742c] set once (at $7eb2, NOT the serve $7162 which is blocked by [$7968]=0) ->
but no DATA path stakes ledger[1] -> back to no stake.

**THE $92b4 RED HERRING RESOLVED.** Reliable AS_OPCODES taps (same pattern as CLOSER-810E/idmatch
which fire): **$92b4 / $92f6 / $9312 fire 0x — the IRQ6 phase-1 DATA fork NEVER runs.** The cont.22
"$92b4 = data" and cont.135 "raise IRQ6 at carry" mechanism is a PRIOR ERA's data path. The CURRENT
FM data path is IRQ5-driven: build_serdes_stream emits per sector {IRQ6 ID, IRQ5 data-AM, IRQ5
data-end}; the fw chain (cont.38ss) is first-IRQ5 -> $29c0 -> $7ba8 (setup), second-IRQ5 -> $7fee
(sector-done) = the SYNCHRONOUS E000 pull (cont.191). The pump ($29c8/$2994) runs constantly. So the
phase-hold / C135PH work chased the wrong fork.

**THE REAL DEFECT = ASYNC FLOOD vs SYNCHRONOUS PER-READ (Dave's frame).** Measured IRQ5:IRQ6 = 67:4.
The free-running rotational spindle (serdes_schedule_next's non-arm branch) floods IRQ5 marks; the
data pull runs 67x without completing a sector. Dave: NOT "throttling" - "synchronous": each read is
independent; deliver each sector's read as ONE self-contained synchronous cycle (ID verify -> match
aim -> [$742c] -> IRQ5 pull -> stake f0 -> advance aim) that completes before the next, so the
chain-of-reads completes. The arm-deliver mode (m_arm_deliver, cont.257q "one sector per arm, no
rotation") is the vehicle; the fix is to make each arm emit exactly the aimed sector's locked mark
sequence and complete it, instead of the IRQ5 flood.

**FRONTIER:** rework the per-arm mark emission so each read synchronously drives the IRQ5->$7ba8->
$7fee pull to a sector-done (f0 stake + aim advance), one sector per arm, chaining all 8. Knobs
added (env, off=baseline): STORAGER_FWDONE, STORAGER_ARMGATE, STORAGER_IDFF, STORAGER_C135PH
(phase-hold, WRONG-PATH - retire), STORAGER_FMVFY/SERVE/STAKEV (diagnostics). FWDONE+ARMGATE are
confirmed keepers to promote to hard_on once the read completes.

## cont.262b (2026-07-20) — THE ROOT = the missing $9318/$92f6 LAUNCH STAKE (the log's own final question); + a TAP-RELIABILITY impediment

**The f0 staker is $9318/$92f6 at the command's LAUNCH, not per-data-mark.** SECMAP (cont.~110,
DESIGN L3951): read1's launch invokes $9318 -> f0 in slot 0 (7.96358); read2/the-continuation's
launch stakes NOTHING; "the grind... all downstream of one missing $9318 invocation at the
continuation's launch." Our read matches exactly: $92f6/$9312/$9318 never run -> no f0 -> the
deadlock (ledger stays ff, aim stuck at 1, want-match fires once, no stake). cont.124's read1-vs-read2
divergence is the same cell: read1's f0 (7.96358) lands BEFORE its hunt starts (7.96380) so its phase
survives to the DATA fork; our read's hunt is already running and clears the phase ($88d6/$8a3e)
before the fork can consume it. So "$9318 at launch" == "phase survives to the DATA fork" == the
same phase-survival root, under the restored FWDONE+ARMGATE fabric.

**The DATA fork ($92b4, reached only via the $298c alternator on IRQ6 phase-1) appears structurally
unreached for our read:** $298c/$299a/$2996 taps fire 0x while $89f2 (reachable as the density-fork
subroutine independent of the toggler) fires 14x. INTERPRET WITH CAUTION (below).

**IMPEDIMENT (must resolve before more deep instrumentation): opcode read-taps are UNRELIABLE for a
subset of addresses.** Taps on $92b4/$92f6/$9312/$9884/$7106/$7150/$7162/$716a/$298c/$299a/$2996 all
fire 0x; taps on $810e/$16c6/$1a54/$89f2/$15a0/$9342 fire fine - SAME install pattern (AS_OPCODES
pair-loop), same device_start region. So some 0-counts may be tap artifacts, not "never executes."
The CLOSER/idmatch block (installed ~L2555-2567) works; SERVE/STAKEV/TOG additions do not. Root
unknown (tap-count limit? a handler reinstall wiping a sub-range? address-region dependence?). Next
session MUST fix instrumentation first (e.g. a single AS_PROGRAM catch-all PC tap, or the debugger's
bpset, instead of many narrow AS_OPCODES taps) before trusting reachability 0-counts.

**CHECKPOINT.** Confirmed keepers restored (FWDONE, ARMGATE) + byte/word phase fix are real, durable
wins. The remaining core = get the launch f0 stake ($9318/$92f6) to fire for the continuation's read,
i.e. make its phase survive to the DATA fork the way read1's does - the project's deepest, most-
iterated cell (cont.38-193). Retire STORAGER_C135PH (wrong fork). Do NOT trust the $92b4/$298c
0-counts until the tap-reliability issue is fixed.

## cont.262c (2026-07-20) — CORRECTION (reliable instrumentation): the DATA fork WORKS; the flaky-tap "never runs" claims were artifacts

**Instrumentation fixed:** early write-taps on RAM ($4000-$7fff, incl. $7654 ledger / $7950 / $742c)
and narrow AS_OPCODES taps get WIPED by the device's RAM-handler reinstall. ROM/vector taps ($74/$78
autovectors) and LATE-installed taps (near phase7950 @L4286) SURVIVE. So cont.262/262b's "$92b4 /
$92f6 / $9312 never run" and "no f0 stake" were TAP ARTIFACTS, now RETRACTED.

**RELIABLE truth (autovector IRQ census + late wide stake-tap):**
- IRQ6 IS taken (~3 ACKs in the read), dispatches to vec6=$298c (the alternator). At 8.000828: IRQ6
  taken with phase=$0101 (bit8=1) AND [$742c]=1 -> the DATA-fork conditions ARE met.
- **The DATA fork RAN and STAKED f0:** STAKE-F0 addr=$7656 (slot 2) pc=$9318 aim=2 742c=1 @8.000847.
  So $92b4->$92f6->$9318 works; it is reachable and fires.
- The aim is NOT poisoned to $fe (ARMGATE fixed the cont.181/182 poison): AIMW = 1 (pc70e4/7e0e) ->
  2 (pc8114/831c). Sane. The stake lands IN-ledger (slot 2), correctly.

**THE TWO REAL GAPS (cleanly isolated):**
1. **Fires only once.** Only ~3-4 IRQ6 data-completions exist (the C135 carry strobe) vs 67 IRQ5;
   the phase-1 alignment happened once (at aim=2). Slots 1, 3-8 never got their aligned IRQ6. Need
   ONE data-completion per sector, aligned to that sector's aim + phase-1 (the synchronous per-read).
2. **The staked f0 is an ORPHAN.** Slot 2 stays f0, never -> c0 (the transfer-complete $933c doesn't
   fire), so the aim sticks at 2 (f0 = captured-pending, no advance). Need the f0->c0 conversion per
   sector.

**FRONTIER:** make the data-completion IRQ6 fire once per delivered sector (aim-aligned, phase held
by C135PH which is now VINDICATED - it produced the one real stake), then deliver each sector's
transfer-complete (f0->c0). NOT a data-mark/IRQ-assignment problem - the fork works; it needs to fire
per-sector and each stake needs its conversion. STORAGER_STAKEV late taps (autovector IRQ census +
wide stake + aimwide) are the reliable instruments - keep them.

## cont.262d (2026-07-20) — THE PRECISE, RELIABLE DIAGNOSIS: data-completion IRQ6 fires 2x, needs per-sector (two-event stake/convert); phase-hold vindicated

**Baseline (FWDONE+ARMGATE, no C135PH) vs C135PH, reliable taps:**
- Baseline: ZERO f0 stakes - [$7950]=0000 (phase 0) at EVERY IRQ6 -> all fork $89f2 (ID). Aim
  advances cleanly 1->2->3->4->5->6->...->0x14 (over-read). The want-match/aim machinery is HEALTHY.
- C135PH: ONE f0 stake (slot 2, $9318, aim=2) - the phase-hold put phase=1 at one IRQ6 -> $92b4 ->
  $92f6 -> stake. VINDICATED: holding the phase is the mechanism that lets the DATA fork stake.

**THE TWO-EVENT CONTRACT is BOTH via $92b4, differentiated by [$742c]:**
- IRQ6 phase-1 with [$742c]=1 -> $92b4 bne $92f6 -> $9318 STAKE f0[slot].
- IRQ6 phase-1 with [$742c]=0 -> $92b4 fall-through $92f4 bra $933c -> CONVERT f0->c0[slot].
So each sector needs TWO phase-1 IRQ6 (stake then convert). 8 sectors = ~16 phase-1 IRQ6.

**THE LIMIT (measured): the data-completion IRQ6 (the C135 carry strobe) fires only 2x** (gated on
E802 bit11 + the per-slot dmap + rec_last), and the phase is 0 at nearly all IRQ6 without the hold.
So: (a) too few data-completion IRQ6 (2 vs ~16 needed), (b) phase not held at each. The one C135PH
stake orphaned (no convert) -> stuck the aim at 2; that is why baseline (no stake) advances further.

**THE FIX (precise): fire the data-completion IRQ6 PER SECTOR in the window (aim-aligned), phase
held by C135PH, so each sector gets its stake (742c=1) then convert (742c=0) -> ledger fills c0 ->
completion.** This is the synchronous per-read Dave called for, now reduced to a delivery-cadence
change (per-slot carry already exists at L1529 s_desc.dmap; the strobe needs to ride THAT, not
rec_last/E802-bit11). NOT a machinery/reachability bug - the fork works; it needs per-sector cadence.

**INSTRUMENTATION (reliable, keep): autovector IRQ census ($74/$78), LATE wide stake-tap
($7654-$775f) + aimwide ($7428), all env STORAGER_STAKEV. Early RAM taps + narrow AS_OPCODES taps
are WIPED by the RAM-handler reinstall - do not trust their 0-counts (this invalidated cont.262/262b
"$92b4 never runs"). Confirmed keepers: FWDONE, ARMGATE, C135PH (+ its byte8 phase-bit fix).

## cont.262e (2026-07-20) — THE REAL ROOT (reliable): the over-read corrupts the ledger TERMINATOR; the 0x82 "timing gap" was a C135PH artifact

**The 0x82 @8.062 is CAUSED BY C135PH, not natural.** Clean baseline (FWDONE+ARMGATE, NO C135PH/
PERSEC): NO 0x82 - the read HANGS at "testend" (only the two 6.4 setup 0x80 stamps). C135PH's one
orphan stake perturbs the fw into the 0x82. So cont.262c/d's "82 pre-empts at 8.062" and the
"8.062-vs-8.096 timing gap" were INTERVENTION ARTIFACTS. RETRACTED.

**THE REAL ROOT (reliable, baseline): the WINDOW COMPLETES but the over-read CORRUPTS THE TERMINATOR.**
Baseline ledger @8.017: `c0 c0 c0 c0 c0 c0 c0 c0 c0 07 08 09 0a 0b 0c 0d 0e 0f` - positions 0-8 = c0
(the 8 window sectors + read1's slot0 ALL DONE), but positions 9+ = `07 08 09...` (over-read garbage,
the aim/R indices of aim 9-25). The completion check ($7078 scan -> $70ba wants $aa END-MARKER) scans
from [$7954]=1 past the c0s, hits position 9 = $07 (not $aa/$ff/$fe), keeps scanning into the
corruption, NEVER finds the terminator -> [$796c]=1 "more" -> the fw keeps over-arming -> aim 9-25
delivered -> MORE terminator corruption. Self-feeding.

**So the completion machinery WORKS (window all-c0); it fails ONLY because the over-read overwrites
the $aa/$fe terminator slot (position 9 = [$7954]+count).** This IS Dave's synchronous point: the read
over-delivers past its 8-sector window (aim 9-25 = the async free-running spindle), and that clobbers
the terminator. A synchronous per-read stopping at the window's last sector leaves position 9's
terminator intact -> the scan finds it -> completion -> 0x80.

**FIX DIRECTION:** stop the delivery/over-arm from writing past the window terminator (position
[$7954]+[$7abc]). Either (a) make delivery synchronous (arm-deliver stops at the window's 8th sector,
no free-run to aim 25), or (b) protect the terminator slot from the over-read's writes. The window
sectors already complete - only the terminator survival is missing.

**RETIRE C135PH + PERSEC (they cause the 0x82 artifact, wrong approach - forcing phase != natural
alternation). Keep FWDONE + ARMGATE + reliable STAKEV instruments.**

---

## cont.264 (2026-07-20) — JIT/synthetic layer RIPPED OUT; raw-bitstream path stands alone

Executed the migration Dave called for ("Start on the 74LS1811 PLL separator over
get_next_transition. Rip out all of the JIT code — it's in github already."). The raw-bitstream read
path (cont.263: 74LS1811 PLL separator + 74LS1812 live-run over real floppy flux) is now the SOLE
read path, and the entire synthetic/JIT delivery layer is deleted. **storager.cpp: 9163 → 7297 lines.**

Removed, in verified checkpoints (each built + oracle-green; branch `storager-lle-wip`):
1. **Made the raw path unconditional** — dropped the `STORAGER_RAWFLUX` gate at all three engagement
   points ($a6d window-open, per-arm start, E000 serve) and deleted the ~1220-line synthetic
   `pump_tick` body; `pump_tick` is now just the SERDES mark clock advancing the live-run. Removed 7
   orphaned JIT fields (m_arm_pos, m_deliver_aim, m_jit_pending/_wait_ord, m_rec_first, m_idx3_hold,
   m_field_captured).
2. **Removed the live synthetic delivery calls** — the PC-snoop SRAM prefill (@0x5ed0 → 0x4000), the
   $02ff window-open `serdes_align`, the arm-time stream rebuild. All were redundant: the flux path
   serves every read (proven — the read still completes with them gone).
3. **Deleted the synthetic cluster** — functions build_serdes_stream, jit_warp_to, aim_to_target,
   stage_next_id, stage_data_record, serdes_pos, serdes_rate, serdes_schedule_next,
   serdes_align_and_start; fields m_serdes_stream, m_serdes_marks, m_next_mark, m_serdes_warp,
   m_serdes_ptr, m_serdes_fm, m_arm_deliver, m_serdes_track. READVERIFY + `s_desc.base_trk` now read
   the live `m_flux_track` (identical (cyl<<1)|side key). The mark-derived completion-delay math is
   gone — m_dataop/m_chancomplete now schedule on their documented 300µs floor.
4. **Pruned dead env knobs** — removed the FLUX-ID/DATA trace logs and the env-table entries that no
   longer gate anything (JIT, NATLAYOUT, MARKSTAGE, POSPTR, IDX5, NOIRQ3, IAM, TRUCK, QPROMOTE,
   AIMDATA, MAPPROBE, ARMGATE, IDFF, SYNCWIN, RAWFLUX).

**Oracle throughout (STORAGER_FWDONE=1, real mx2-001.imd):** FLUX-START:1, ledger all-C0, 0x80
stamped — the read completes end-to-end over real flux. `-validate pcmx2` clean.

**FWDONE is BEHAVIORAL, not a diagnostic** — it governs whether the model transcribes completion
status to the host IOPB (the completion path itself). Left in place; it is the next thing to resolve,
not remove.

**Open frontier (unchanged by this migration):** the read now takes real rotational time (~1.8s to
gather 8 sectors at 300 RPM); completion lands ~9.7s but the CPUAP monitor has already moved on
("going to harddisk"). This is the honest read-time vs monitor-patience reconciliation (cont.170-171),
now made real. Remaining: that timing reconciliation, verifying data lands at the host, the MFM/HD
read paths through the same PLL+live-run engine, and a comment/doc sweep of the historical
synthetic-era annotations still scattered in the source.

---

## cont.265 (2026-07-20) — the raw-flux read DELIVERS the data to the host (VOL1/HDR1 byte-exact)

Dave's steer: "this worked (under HLE), so a timeout means we are not supplying something in a
timely fashion." Chased it with a reliable ledger-change watch (polled in pump_tick, no fragile tap)
and a per-mark cadence log. The chain:

- The screen's "READ -compl-stat: 13 sensb: 82" is the READ (cmd=95) ERRORING; the 0x80 stamps were
  cmd=87/89 (INIT/mode), never the read. So the floppy read has NEVER completed to the monitor - the
  monitor reports the error and moves to the hard disk. (The earlier "all-C0 / 0x80" was the error
  teardown resetting the ledger + other commands' stamps, not the read succeeding.)
- Captures stake f0 promptly and in one pass (aim advances 1..8), but nothing converts f0->c0, so the
  verify hunt exhausts (5 revs) -> 0x2029 -> sense 0x82. This is the cont.242 orphan-f0.
- ROOT of the empty transfer: **the DATA field was never deposited to the host.** `DESCGO` DMA'd
  `src: 00 00 00 00 ...` - the ID was staged ($7dac) but the data field never was (only ID staging was
  wired in cont.263; the old `stage_data_record` was deleted in the cont.264 rip-out). The `TRUCK`
  m_sectors carry was inert (m_c000 = 0xe70000, outside the [0x080000,0x100000) host window; the read's
  real buffer is iopb_buf = 0x0fc0dd).

**FIX (faithful, "detection is capture"):** at the flux DATA mark, deliver the just-recovered field
(m_flux_buf) to the host at its logical position - `s_desc.host + n*ssz`, `n = ntrk*spt +
(R - sec0) mod spt` - and fire the transfer-complete IRQ4 once every window block has landed. One
revolution delivers the whole window (R07..R0e -> host+0..host+896). **VERIFIED byte-exact:** R07 ->
host+0 = `56 4f 4c 31 53 49 4e 49` ("VOL1SINI"), R08 -> host+128 = `48 44 52 31 20 4e 53 43`
("HDR1 NSC"). The correct label now lands at the host over the pure raw-flux path (was zeros before).

**STILL OPEN (the remaining blocker):** f0->c0 conversion. Even with the data landing and IRQ4 firing,
the firmware's ledger stays f0: the scan finds `LEGCEN NOHANDLER` for f0 at aim=1, `[$742c]` stays 0001
(clears only briefly - 3 samples vs 12), gating off $933C, and the aim can't advance past the f0.
The two-event contract ($92F6 stake / $933C convert, gated by [$742c] and the $7950 phase alternator)
needs the transfer-complete interleaved per sector at [$742c]==0, not one bulk IRQ4 at window-done
(which also races the per-rev DESCARM re-arm). Next: drive the per-sector convert - fire the
transfer-complete at each in-window data mark timed to the [$742c]==0 / phase window, or find why the
firmware isn't reaching its $FE-leg to clear [$742c]. The data path is now proven; the conversion
handshake is the last mile.

---

## cont.266 (2026-07-20) — the DATA PHASE COMPLETES (data-AM IRQ5 from cont.262); frontier = the transfer phase ([$79b6]/[$79a8])

Per Dave ("this has been solved repeatedly; review the history before debugging"), mined the log
instead of re-deriving. Two history-grounded results:

**FIXED — the f0->c0 conversion (the cont.242 orphan-f0).** cont.262 documented the FM data path as a
TWO-IRQ5 contract: first-IRQ5 at the data address mark -> $29c0 -> $7ba8 (setup), second-IRQ5 at the
data-record end -> $7fee (sector-done). The raw-flux path raised IRQ5 only once (data-end), so the
$7ba8 setup leg never ran and every sector orphaned at f0 -> sense 0x82. Added flux_data_am() at the
DAM detection ($f56f FM / $fb/$f8 MFM). Result: the ledger now converts f0->c0 incrementally to all-c0
over one revolution; the 0x82 error is GONE; the read data phase completes. This reaches the exact
cont.260 state via the pure raw-flux path.

**FRONTIER — the 0x80 completion post ([$79a8] owed never zeroes).** The monitor has infinite patience
(cont.173 end-state) and now WAITS at "testend" (no more 0x82 give-up) for a 0x80 that never comes.
The completion chain is ladder-end -> $17fe done-gate -> $1966 -> $1a54 (0x80) -> $1ab2 motor-off,
gated on [$79a8]==0 (transfers owed, seeded 8 = IOPB byte7 by op $56 at $a486). The owed-decrement tap
comment (storager.cpp ~L2634) names the mechanism: [$79a8] decrements at $9322 (DECGATE) but the
decrement is DROPPED when [$79b6]==0; [$79b6] (transfer-active) is armed at $95e4 inside $94ec. In the
runs [$79b6]=0 and 0 E000 pulls occur in the data phase -> the transfer phase never arms -> [$79a8]
stays 8 -> $9506 done-check never passes -> no $1a54. So: capture done, transfer phase not started.

**NEXT:** find the event that arms [$79b6]/starts the transfer phase ($94ec/$95e4/$8214 vs $80c0 fork,
gated on $79b8/$79b6/$79a8/$79b0/UIB[9] per cont.24). Candidate: the model owes the per-record transfer
strobe that $94ec waits on (the "buffering complete -> transfer active" edge), distinct from the
capture IRQ5s. The data content already lands at the host (cont.265); this is purely the fw's
owed-transfer accounting reaching zero so it posts its own honest 0x80.

---

## cont.267 (2026-07-20) — transfer-phase arming: [$79b6] never touched; over-scan is a red herring for the error

Drilled the transfer phase (Dave: "keep going on the transfer-phase arming"). Findings, all from the
current v2.60 flow with the cont.266 data-AM fix in place (data phase completes, ledger 1-8 -> c0):

- **[$79a8] (owed) is RE-SEEDED to 8 at $81d0 every data mark, never decremented** (write-tap OWED79A8:
  15x <-0008 pc=$81d0, 1x seed <-0008 pc=$a492). The tap comment (storager.cpp ~L2634) matches: the
  $9322 DECGATE drops the decrement when [$79b6]==0, so $81d0/$7fee take the re-seed branch.
- **[$79b6] (transfer-active) is NEVER written anywhere in the whole run** (write-tap: 0 hits). So the
  transfer op that arms it ($94ec/$95e4 - VESTIGIAL in v2.60 per cont.106-141) is completely unreached.
  cont.141 saw the identical state. The transfer op / done-check never dispatches.
- **The Build#5 addresses are wrong for v2.60**: $7f78/$808a/$8214/$80c0/$4a66/$55d8/$94ec taps all fire
  0x. The v2.60 transfer entry is elsewhere (unknown). IRQ3 index is a dead end: [$72f8] (IRQ3 vector)
  = 0, the fw never installs the index handler for the floppy read (IRQ5/IRQ6 vectors ARE installed).
- **[$7956] (sector countdown) DOES reach 0** (capture complete). So the capture-exit gate is satisfied.
- **Over-scan is NOT the error trigger**: without gating, the fw over-scans (aim 9..25) and positions
  9-16 of the ledger read 07..0d instead of the fe/aa terminator - BUT the fw just WAITS at testend
  (infinite patience), no 0x82. Gating the out-of-window DATA marks (sector-keyed) instead caused 0x82;
  gating on s_desc.active broke the capture (no ID -> no arm deadlock). Both REVERTED. So terminator
  clobbering is a symptom, not the completion blocker.

**STATE:** the read data phase completes (1-8 c0, VOL1/HDR1 at host, [$7956]->0), the fw waits patiently
at testend, but the transfer/done phase ([$79b6] arm -> [$79a8] dec -> $1a54 0x80) NEVER DISPATCHES.
This is the cont.141 frontier, unchanged. NEXT: find the v2.60 code that arms [$79b6] / dispatches the
done-check op - it's not IRQ3-index, not the Build#5 $94ec. Candidate approach: forward-trace from the
$7fee sector-done ($81d0 re-seed context) via a reliable AS_PROGRAM tap (AS_OPCODES taps at $81xx are
wiped), OR find how the working HLE (read95_deliver) posted 0x80 and what fw event it stood in for.
Diagnostic taps added under STORAGER_XFER (off by default): W79B6, RESEED81D0.

---

## cont.268 (2026-07-20) — DISASM DECODE: [$79b6] was the HD path; the floppy completes via $7fee->$808a->$810e; the exit blocker is the un-draining [$74ac] slot queue

Dave: "when you find a block ALWAYS search the history - it's been solved before"; and the disassembly
EXISTS (siemens/disasm/storager/storager_v260.asm). Decoded the completion directly:

**CORRECTION - I was drilling the wrong path.** cont.267's [$79b6]/[$79a8]/$9948/[$7a0e]/$94ec chain is
the **HD path** (UIB[$12] bit1 SET). The FLOPPY (unit 2, UIB[$12]=0x45, **bit1 CLEAR** - a per-unit-TYPE
ROM constant, no runtime writer; Build#5 cont.11) takes a DIFFERENT path. $9884 reads D0=UIB[$12]; the
$9906 `btst #1,D0 -> $9948 (set [$7a0e]=1, wake the HD transfer)` is only reached with bit1 SET. For the
floppy the verify goes $98a4 `btst #2` -> $9934 FM-parse -> $9914 match loop. Confirmed in-model: [$7a0e]
is NEVER written, [$79b6] is NEVER written - both dead because they are the HD branch.

**The floppy completion path (decoded):** second-IRQ5 -> $7fee (sector-done: writes C800/D800, arms E802,
installs [$7302]=$7ba8, reads $E000 once at $8028) -> $808a (the transfer GATE, present in v2.60):
`tst [$7426]; beq $810e`(closer) / `tst [$79b8]; beq $8214` / `tst [$79b6]; beq $80c0`(HD slot-walk).
For the floppy [$7426] reaches 0, so it goes $810e. **$810e RUNS and is the per-sector completer:**
`$8120 move.b #$c0, ledger[aim]` (this is why the ledger reaches c0!), records C/H/R to the $7696 table
($8140), then $81b4: re-seeds [$79a8]=[$79a6] at $81ca (the $81d0 re-seed observed in cont.267) and
enqueues the sector's slot.

**THE EXIT BLOCKER: [$74ac] never drains.** [$74ac] is the completed-sector slot queue; it advances one
8-byte slot per sector-complete (74c4 -> 74cc -> ... -> 7504+ in-run) and NEVER returns to 0. The command
exit ($32ac returns no-want -> [$7968]=0 -> ... -> $1a54 0x80) requires **[$74ac]==0** (board-notes
REDUCED INVARIANT). Because the disk keeps spinning past the 8-sector window (over-scan), sectors keep
completing, the queue keeps growing, and $32ac always finds a queued want -> no exit -> no 0x80. [$7956]
DOES reach 0 (capture count satisfied); the queue is the remaining gate.

**NEXT:** the [$74ac] slots must be CONSUMED (drained) - find the v2.60 consumer (the $32ac $32d0
unlink / the transfer that releases slots) and what event drives it for the floppy, OR stop the
over-scan cleanly so exactly the 8 window slots enqueue and then drain. cont.267's mark-gating was the
right instinct (stop over-scan) but broke on the $7ba8<->$7fee ping-pong; the gate must respect the
ping-pong (suppress only the ENQUEUE past the window, not the IRQ5 alternation). Search the disasm for
the $74ac consumer next.

---

## cont.269 (2026-07-20) — the [$7956] over-scan gate PRESERVES the terminators; exit chain = $7e8a->$32ac drain

Decoded the exit/drain path (v2.60 disasm). $32ac IS the [$74ac] consumer: `$32ca tst $74ac; beq $3320`
(empty->ledger scan) else `$32d0` dequeues+unlinks one slot. It is called from the EXIT CHAIN at $7eea:
$7e8a `tst $7968; beq $7eb2` -> $7eb2 sets [$742c]=1, `subq #1,$7956` (this is where [$7956] decrements),
`bne $7ee0` -> $7ee0 `bsr $32ac`; if $32ac returns a want set [$7424] and continue, else $7ed8 sets
[$741c]=0 (the correct exit) -> $1a54 0x80.

**KEEPER (cont.269): the [$7956] over-scan gate.** flux marks (ID+both IRQ5) for a sector are now
suppressed once [$7956]==0 at that sector's ID (m_flux_skip). This models the fw's own closed read
window: it STOPS the over-scan cleanly - ledger terminators at pos 9-16 stay $fe (were clobbered 07-0d),
ledger 1-8 still -> c0, NO 0x82 (unlike cont.267's sector-window gate which broke the $7ba8<->$7fee
ping-pong). [$74ac] stops growing (parks at the 8th slot 7504).

**REMAINING: [$74ac] still doesn't DRAIN to 0.** The 8 window slots enqueue but $32ac isn't draining
them (7504 parks). So the exit-chain drain loop ($7e8a->$7eb2->$7ee0->$32ac) isn't cycling enough - it
either isn't reached, or the in-window sectors re-enqueue as fast as it drains. NEXT: tap $7e8a/$7eea
(reliable AS_PROGRAM or the $7ed8 [$741c]=0 exit) to see if the drain loop runs and why [$74ac] parks;
check whether the still-firing in-window marks (they re-fire every rev until [$7956] hits 0) re-enqueue
via $810e faster than $7eea drains. Likely fix: also gate an in-window sector's marks once its ledger
block is already c0 (don't re-complete a captured sector), so only truly-wanted sectors enqueue.

---

## cont.270 (2026-07-20) — already-c0 skip shrinks the [$74ac] queue but it still won't fully drain

Extended the over-scan gate (cont.269): also skip a sector's marks when its ledger block is ALREADY c0
(captured) - don't re-complete/re-enqueue a done sector every revolution. RESULT: [$74ac] now parks at
74d4 (~3 slots) instead of 7504 (8 slots), and [$741c] reaches 0 (the exit marker) intermittently - so
the drain loop IS cycling and consuming, just not to empty. Data phase still completes (1-8 -> c0,
terminators $fe intact), no 0x82.

**REMAINING (the last gap): [$74ac] parks at ~3 slots, never 0.** The drain loop $7e8a->$7eb2->$7ee0->
$32ac drains 7 of 8 (the last [$7956] 1->0 decrement takes the $7ec4 branch, NOT $32ac), and a couple
slots re-enqueue. The exit ($7ed8 [$741c]=0 -> $1a54) needs [$74ac]==0. NEXT (do NOT re-derive - the
history/disasm has it): find how a WORKING past run drained [$74ac] to 0 - tap $7eea's return
((-6,A3)==0 = no-want) vs the enqueue rate; or check whether the floppy is supposed to drain via a
DIFFERENT consumer than the $7eea exit-chain $32ac (the $79be/$7ad6/$a2de callers). The gate work
(cont.269/270) is a real keeper (terminators preserved, queue bounded); the drain-to-zero is the final
piece.

---

## cont.271 (2026-07-20) — the $aa terminator IS the exit key (AAFIX validated); [$74ac] drain is the last piece

Followed the disasm to the exit decision. $32ac's ledger scan ($3320) walks from [$7954] for a byte with
BIT 6 SET; $ff/$f0/$c0/$fe ALL have bit6 set, only **$aa (bit6 clear)** stops it. So "no-want" (-> exit
$7ed8 [$741c]=0 -> $1a54 0x80) needs the scan to reach the $aa END-MARKER at position [$7954]+[$7abc]=9.
**The $aa was MISSING** (position 9 read $fe): the fw stakes it at LAUNCH ($706c move.b #$aa,(A0)) but
the read clobbers it (the scan's own $33ae `move.b #$fe,(A2)` over-writes the boundary).

**VALIDATED: STORAGER_AAFIX (cont.262f) places the $aa correctly** - with it the ledger reads
`c0..c0 ff.. aa fe fe..` (position 9 = $aa). This is the exact history-sanctioned terminator-protect fix.

**STILL BLOCKED: [$74ac] won't drain to 0.** The exit also needs [$74ac]==0 (the completed-slot queue).
Even with the $aa placed and the cont.269/270 over-scan gates, [$74ac] parks at 74d4 (~2-3 slots): $32ac
dequeues one slot per call ($32d0) but the queue re-fills / isn't cycled to empty. The drain loop is
$7e8a(tst[$7968] beq $7eb2)->$7eb2(subq#1,[$7956])->$7ee0->bsr $32ac; it drains 7 of 8 (the last [$7956]
1->0 skips $32ac) and a couple re-enqueue. So: $aa + gates give a CORRECT ledger; the final piece is
draining [$74ac] to 0 so $32ac reaches its $3320 scan and the $aa yields no-want.

**NEXT:** find why [$74ac] parks - who RE-enqueues after the window is c0+$aa (tap $810e/$81b4 enqueue
vs $32ac dequeue counts), and whether the drain loop ($7e8a) stops being called once the flux marks
stop (may need to keep one event flowing to cycle the drain). This is the whole boot, minus the drain.

---

## cont.272 (2026-07-20) — the [$74ac] drain is in ENQUEUE/DEQUEUE EQUILIBRIUM; lever = stop the per-rev capture re-arm

Decoded the drain coupling. $7fee (sector-done, my 2nd IRQ5) FORKS on [$741c] ($8044 tst $741c):
[$741c]==0 -> the CAPTURE path ($8028.. -> $808a -> $810e stakes c0 + ENQUEUES a [$74ac] slot);
[$741c]!=0 -> the TRANSFER/DRAIN path ($804c.. walk $7e6c -> $7e8a -> $32ac DEQUEUES one slot; $7ef8
processes the want, sets [$741c]=1). So EVERY IRQ5 mark does exactly one enqueue OR one dequeue,
selected by [$741c], and the fw's own walk alternates [$741c] 0<->1. Net: the queue stays in
equilibrium and never drains to 0.

Tested cont.272 (keep firing marks when [$741c]!=0 to cycle the drain): WRONG - it just fired more
marks -> more captures -> [$74ac] GREW (74fc). Reverted. The problem is not too-few drains; it is that
the fw KEEPS RE-ARMING CAPTURES (DESCARM per revolution) after the window is already c0+$aa, so [$741c]
keeps returning to 0 and $810e re-enqueues. As long as the disk spins and marks arrive, the fw re-opens
a capture window every rev.

**THE LEVER (next):** stop the fw's per-rev capture re-arm once the window is satisfied (all c0 + $aa
placed), so [$741c] stays 1 (transfer) and the queue drains monotonically to 0 -> $32ac hits its $3320
scan -> $aa -> no-want -> $7ed8 [$741c]=0 exit -> $1a54. Candidates: (1) the ARMGATE/$a6d engagement -
close the mark window (m_serdes_active=false) when the read is done so no more capture marks arrive;
(2) find the fw's own read-complete gate that should stop DESCARM (the $23f close / the [$7956]==0 +
[$74ac] drained condition) and deliver whatever event flips it. This is the whole boot minus this one
equilibrium-break. AAFIX + cont.269/270 give the correct ledger; this closes it.

---

## cont.273 (2026-07-20) — the completion WALK/exit chain is never TRIGGERED (not a queue problem)

Corrected the whole [$74ac] framing. Reliable AS_PROGRAM write-taps show: **[$74ac] is NEVER written
during the read** (no enqueue/dequeue) and **[$7956] is never written to 0** (CNT0 tap: 0 hits). So the
"[$74ac] parks / queue equilibrium" reads (cont.268-272) were STALE values, not live churn. The real
finding: the completion/exit chain does not run at all.

**The exit decoded (floppy, UIB[$12] bit1 clear):** $32ac's scan takes the $33d2 branch (not $3320).
"No-want" ($346c) needs the backward scan of ledger pos 2..8 to find no bit6-set byte, OR ($3448)
[$7968]!=0 AND found-pos <= aim [$7428]. Since the window is c0 (bit6 set), only the [$7968]!=0 path
exits. [$7968]=1 is set at $82b2, reached $8214->$823c([$7956]==0)->$825c->..->$8296([$727e]==0)->$82b2,
gated on: [$79b6]==0 & [$79ba]==0 & [$7956]==0 & [$7958]==0 & [$741c]!=0 & [$796a]!=0 & [$727e]==0. And
the whole $8214/$82b2/$7e8a chain hangs off the WALK $7e58->$7e6c (scan ledger for $aa/$ff/$fe -> $7e8a
-> $7eb2 subq [$7956] -> $32ac). **That walk is not being triggered** - [$7956] never decrements to 0
via $7ebe, $32ac never runs, [$7968] never goes 1.

So the missing gate-array state is whatever TRIGGERS the completion walk ($7e58/$7d.. routine) once the
capture is done. It is NOT the $aa (AAFIX places it correctly), NOT the [$74ac] queue (never churns).
NEXT: find the caller/trigger of the $7e58 walk routine (its entry is in the $7d00-$7e58 block; find
bsr/jsr to it or the interrupt/flag that dispatches it), and what event the gate array must raise so the
fw runs its completion walk. Likely an IRQ (IRQ4 channel-complete?) or a status edge the model omits.
Data phase + ledger (c0 window + $aa via AAFIX) are correct; only the completion-walk trigger is absent.

---

## cont.274 (2026-07-20) — the CONSUMPTION/exit chain stops after 2 sectors ([$7956] parks at 6)

Reliable taps: [$7956] parks at **6** (795 samples), never 0, INDEPENDENT of the c0-skip gate (removing
it did not change this - the earlier min=0000 was a pre/post sample). [$7956] is decremented by the exit
chain $7ebe (`subq #1,$7956`), so it ran exactly TWICE (8->7->6) then STOPPED. Only 2 of 8 sectors are
CONSUMED (transferred/exited), even though all 8 are CAPTURED (ledger 1-8 = c0) and the $aa is placed
(AAFIX). [$79ba]=0 throughout; [$741c] toggles 0/1; the exit-enable [$7968]=1 ($82b2) never fires.

So the real blocker is NOT the ledger (correct) nor the $aa (placed) nor a queue (never churns): it is
that the fw's per-sector CONSUMPTION (the $7e58/$7e6c walk -> $7e8a -> $7eb2 -> $32ac chain) runs for
only 2 sectors then stalls. The gate array must be raising the per-sector consumption event (each
IRQ5-driven walk consuming one sector) but after 2 it stops being satisfied. NEXT: instrument the walk
$7e58/$7d02 per IRQ5 - why does it stop reaching $7eb2 after the 2nd sector (which condition in the
$7d22 scan / the $7e58 want-match fails from the 3rd on)? Correlate with what changes at [$7956]=6:
[$79a0]/[$7428] aim, [$742c], the ledger scan cursor. The 2-vs-8 split is the exact signature to chase.

**State kept (keepers):** cont.269/270 over-scan gates + STORAGER_AAFIX give the correct ledger (c0
window + $aa terminator, no 0x82, data byte-exact at host). The completion walk stalling at sector 2 of
8 is the one unresolved gate-array-signalling gap between here and boot.

---

## cont.275 (2026-07-20) — DECODED $6f44 (the history's UNREAD routine): the ledger needs POSITIVE slot-index writes

Dave: this is solved; the history shows it. cont.38hh/ii had reached but LEFT UNREAD $6f44 (the floppy
class's transfer-queue builder). Decoded it now:

**$6f44 (floppy, [$796a]==0, [$7968]==0 path):** re-seed [$7956]=[$7abc]=8; A0 = $7654+[$7954]; loop
[$7956]-1 entries: `move.b (A0)+, D4; bge $6f92`. **D4 >= 0 (bit7 CLEAR = a POSITIVE SLOT INDEX)** ->
slot = $74c4 + D4*8, stamp map entry $ff (consumed), LINK the slot into the [$74ac]/[$74ae] transfer
queue (head/tail). **D4 < 0 (bit7 SET = c0/f0/fe/ff, a NEGATIVE marker)** -> stamp $ff, skip. Then
$702e (the queued transfer runs) -> DMA -> CMDDONE -> verdict.

So (confirming cont.38ii) the ledger is a FILL-MAP for the transfer: a **positive byte = "sector data is
in slot N, queue it for host transfer"**; c0/f0/fe/ff are all negative = nothing to transfer. **The model
only ever writes NEGATIVE markers** (c0 via $810e $8120, f0 at capture), so $6f44 queues NOTHING, no
transfer runs, [$7956] never drains, no $1a54. "SECMAP never saw a positive write in a hundred runs"
(cont.38ii) - the POSITIVE-ENTRY WRITER is the per-sector data-landing event the model omits.

**THE MISSING GATE-ARRAY BEHAVIOR:** when the SERDES captures a sector's data, the gate array DMAs it
into a local SLOT (the $74c4 pool, 8-byte entries) and writes the ledger position = that SLOT INDEX (a
positive byte). $6f44 then queues the slot, $702e transfers slot->host, and the sector completes ($c0).
The model currently short-circuits (flux->host direct + c0 stamp), so the fill-map never gets a positive
entry. NEXT: model the capture as SERDES->slot DMA + ledger[pos]=slot#, and the transfer as slot->host
DMA (replacing the flux->host shortcut), so the fw's own $6f44->$702e transfer/consume runs. This is the
faithful gate-array DMA-into-slots the whole read has been missing. AAFIX(pos9=$aa) + cont.269/270 gates
give a correct-looking ledger but the wrong POLARITY (negative markers where positive slot-indices belong).

---

## cont.276 (2026-07-20) — Dave's FILL-MAP validated (correct ledger shape); walk decrements [$7956] only twice/pass

Implemented Dave's guidance: the gate-array fill-map. STORAGER_FILLMAP writes ledger[pos] = the SECTOR
NUMBER R (positive byte, per Dave "the slot is the sector, not the sector-since-index") when a sector's
data lands. **VALIDATED SHAPE:** the ledger now reads `c0 07 08 09 0a 0b 0c 0d 0e aa fe..` - R7-R14 as
positive slot-indices at pos 1-8, $aa at pos 9 - exactly the fill-map $6f44 needs (was all-negative
c0/f0/fe). This was the missing polarity (cont.275).

BUT [$7956] still parks at 6 - the fill-map is NECESSARY but not SUFFICIENT. Decoded the consumption
walk fully: it is the ID-VERIFY handler at $7c34 (per IRQ6): `tst $79ae; beq $7ce6` else verify captured
HEAD [[UIB+$cc]] vs [$7436] (err $202a) + CYL [[UIB+$ca]] vs [$7438] (err $2012), pass -> $7ce6 -> $7d02
walk (cursor D0 = [[UIB+$ce]] = captured POSITION [$7daf]) -> scan $7654 for $aa/$ff/$fe ($7e6c, skips
positives+c0) -> $7e8a `tst $7968; beq $7eb2` -> $7eb2 `subq #1,$7956`. The sibling handler $7bf0 CLEARS
[$7956]=0 (the true completion/reset). $6f44 re-seeds [$7956]=[$7abc]=8 each pass; the walk decrements it
TWICE (8->6) then stalls; so 2 of 8 consumed per pass, repeating.

**ISOLATED REMAINING BLOCKER:** why the walk reaches its $7eb2 decrement only twice per pass (or why
$7bf0 - the [$7956]=0 completion - never runs). Candidates to chase next: the $7e8a `tst $7968` gate
(does [$7968] flip !=0 after 2?), the walk cursor [$7daf]/[[UIB+$ce]] (does it advance so the scan stops
finding the terminator?), the POSITION VERIFY passing for only 2 (the captured C/H/R staging for the 6
empty sectors R9-14). The full chain is now decoded end to end; this decrement-count is the one gap.

**KEEPERS:** STORAGER_FILLMAP (correct ledger polarity, Dave-guided) + STORAGER_AAFIX ($aa terminator) +
cont.269/270 over-scan gates. Data still byte-exact at host (VOL1 host+0, HDR1 host+128).

---

## cont.277-278 (2026-07-20) — Dave's GATE 2 SOLVED: [$7daf] ordinal fix -> [$7956] grinds to 0; Gate 1 (the $99/$82b2 completion) is the last mile

Dave's fresh static analysis (disasm + model source + IMD) named two gates the log had circled but never
pinned. Ran ONE authoritative reliable trace (STORAGER_TR6, read at the IRQ6 raise, this side of the
RAM-handler reinstall) and both resolved:

**GATE 2 (SOLVED):** the POSITION cell [$7daf] was staged as the raw sector R (7..14) while the aim
[$7428] is position-space; $7e58 (cmp $7428,[$7daf]) never matched -> aim stuck at 1, [$7956] decremented
only twice/pass. FIX (Dave's one-liner): stage [$7daf] = ((R-sec0) mod spt)+1 (the ordinal). RESULT
(verified TR6): aim now sequences 1->8, [$7daf]=01..08, **[$7956] grinds 8->0** - the full per-sector
consumption runs. Data still byte-exact (VOL1 host+0, HDR1 host+128).

**GATE 1 (the last mile):** [$7956] reaching 0 via the subq walk only DISARMS; the honest 0x80 needs the
completion. Two forms, both blocked:
- $82b2 exit-enable: ALL its flags are satisfied at [$7956]==0 ([$79ba]=0 [$7958]=0 [$741c]=1 [$796a]=1
  [$727e]=0), and (after cont.278 relaxed the mark gate to out-of-window-only so marks arrive at
  [$7956]==0) marks DO reach the check - but the flow forks at $808a `tst [$7426]; beq $810e` to the
  per-sector closer instead of $8214->$82b2, because [$7426]==0. [$7426]=1 is set only at $7cac (the
  $7ca8/$7bf0 completion region).
- $7bf0 $99-path: [[$71bc]]==$99 is the TRUE completion; [[71bc]] stays $95 (the read cmd) - the fw never
  presents a $99 request-type. $0eec sets [$71bc]=node; a $99-command node must be dispatched at
  window-end, triggered by a gate-array edge the model omits (Dave's candidate: IRQ4/channel-complete or
  DESCDONE).

So Gate 2 opened the whole per-sector consumption; Gate 1 is a single window-end completion event (set
[$7426] / present a $99 request) that the gate array supplies on real hardware. cont.278 relaxed the
over-scan gate to out-of-window-only (AAFIX protects $aa) so the completion mark can arrive. KEEPERS:
STORAGER_FILLMAP (positive slot-index ledger writes + [$7daf] ordinal) + STORAGER_AAFIX + the relaxed gate.

---

## cont.279 (2026-07-20) — TRACE confirms Dave's Gate-1 root: DESCDONE IRQ4 fires with a NULL descriptor callback

Dave's fresh disasm pass reframed Gate 1: $99 CANNOT be firmware-written (the ROM never stores $99/$71
as a request-type; [$71bc] is set only at $0eec = the fetched IOPB). [[71bc]]==$99 happens only when the
GATE ARRAY auto-fetches an internal continuation IOPB. The three finalize routes ($82b2 phase-path;
$3bfe IRQ4-descriptor; $7bf0 $99-IOPB) all hinge on the channel/DMA-transfer-complete IRQ4 carrying a
REAL descriptor node: $3bfe reads [$743a] and jsr's its +$14 callback, which advances the ladder to its
transfer step (installs $9188/$9398, latches [$7426]=1). And ($34ee is NOT a DMA - just a linked-list
unlink stamping slot status $80; no data-corruption hazard in the completion path.) Subtlety: the Gate-2
scan path CLEARS [$7426] at $7f1a, so window-close must come from the IRQ4 ladder step, not consumption.

**AUTHORITATIVE TRACE (reliable, at my own DESCDONE IRQ4 raise):**
`DESCDONE-IRQ4 [743a]=748a +14=00000000 | [[71bc]]=95 +26=00 | 7424=0006 7426=0000`
- [$743a]=748a (fw set a node at $3b92 @6.40) but its **+$14 callback = 00000000 (NULL)**.
- ladder phase [[71bc]]+26 = 00 (not advanced); [$7426]=0 (never latched); [$7424] steps 6..9/rev.
So my bare DESCDONE IRQ4 makes $3bfe jsr a NULL callback -> advances nothing -> [$7426] never latches
-> $808a forks $810e forever. CONFIRMS Dave: raising IRQ4 without the fw's built descriptor.

**DECISION: Option A (finish cont.275's second half - the faithful transfer path).** The +$14 callback
is null because the flux->host short-circuit (line 758) PRE-EMPTED the transfer phase; the fw never built
the transfer descriptor. The smaller "right node in [$743a]" fix is insufficient (+$14 must be installed
by the transfer step actually running). NEXT (the migration): (1) stop flux->host; stage recovered
sectors into the local buffer ($4000-$7fff); (2) let the fw's ladder build its transfer descriptor +
kick the channel (E800 bit6/bit12; model tracks m_dma_active + C000 counter); (3) model the
descriptor-driven buffer->host DMA, raising IRQ4 with [$743a] carrying the real +$14 so $3bfe advances
the ladder -> [$7426] latches -> $82b2/completion natively. This is also LLE-correct (the flux->host
write is the last HLE-shim doing the fw's transfer).

---

## cont.280 (2026-07-21) — DESCTRACE: arm-REACHED but descriptor PARTIAL ([$7a14]=0); Dave's $748A contract

Dave's disasm agent CORRECTED cont.279: node+$14=NULL is BY DESIGN (only ever clr.l at $3D58). $3BFE does
NOT jsr(node+$14) for the read; it branches on IDENTITY first ($3c16: cmpa [$7a14],[$743a] both==$748A ->
$1310/$1348 re-arm selected by [$7a76]; mismatch -> generic $3c32 jsr(+$14)=terminal). So no callback to
synthesize - the completion is the fw's own $1310/$1348 re-arm; the model must guarantee the IDENTITY at
IRQ4.

**THE $748A LAUNCH-RECORD CONTRACT (Dave, spec §6.4; built $3D4E, consumed $3CD4):**
+04/06 $748E/90 = ~hostaddr (one's-comp 24b, the C000 preset; <-[$79DC]=not.l IOPB buf) -> latch C000 @DESCGO
+0a/0c $7494/96 = E800 AND/OR masks -> apply to E800
+0e $7498 launch/op-state (1->2->3); +10 $749A = IOPB>>1 (D000 word addr); +12 $749C = BUSY (clear releases fg)
+14 $749E = callback ALWAYS 0 (leave it); +18 $74A2 = status-post $7440; +1c $74A6 = class->PIT count; +1e $74A8 = PIT preset(=3)
$7A14 = launch id (=$748A, IRQ4 identity); $743A = completing node (=$748A); $7A76 = continuation-select ($1310 vs $1348).
($74AC/AE = desc-queue head/tail, NOT node fields.) PORT: count is 8253 PIT LSB/MSB @$8004 ($0003->$8002, ctrl $7A->$8006);
~host rides C000 (node+04, latched DESCGO, no move,$c000); D000=IOPB>>1; E800 &=AND |=OR|op |=$1000 bit12 kick. Arm=$13D2/$3CD4.
Park proven ($159C sets IOPB+$26=$A); unpark ($3E30 +$26=$C) done by $3DBC gated [$7a64]set & [$7454]==0 & [$72d6]drained.

**DESCTRACE RESULT (>7.9):** BUILD($3d4e):0 ARM($3cd4):**921** PARK36:0 UNPARK:0 DESCGO:20 IRQ4id($3c16):**920**
DONE10:0 DONE48:0. Snapshot: **743a=748a but 7a14=0000** (identity BROKEN), 7a76=0, ~host=0030ffff (wrong),
7a64=1, 749c=0, ph7216=00. So the read REACHES the arm ($3cd4, 921x) and my IRQ4 reaches $3c16 (920x), but the
$748A descriptor is only PARTIALLY populated ([$7a14]/~host/[$7a76] unset) because the flux->host + bare-DESCDONE
pre-emption short-circuits the BUILD - so the identity fails -> generic null-callback -> no $1310/$1348, no unpark.

**=> SMALL MIGRATION (arm-reached), precondition pinned: stop pre-empting so the fw fully builds $748A
([$743a]==[$7a14]==$748A, real ~host, [$7a76]); then per-sector: stage flux->local buffer ($4000-$7fff), do the
buffer->host DMA on the E800 bit12 kick honoring C000/PIT, raise IRQ4 with the identity intact -> $3bfe->$1310
drains [$74B4] -> $3DBC posts +$26=$C -> $749C clear -> fg release -> honest 0x80.** Adds STORAGER_DESCTRACE.

---

## cont.281 (2026-07-21) — FAITHXFER migration applied: pre-emption removed -> read PARKS at 0x36 (Dave's larger case)

Implemented Dave's line-758 diff (STORAGER_FAITHXFER, 3 pieces, baseline byte-exact): (1) capture->local
window buffer m_win_buf not host, suppress the bare window-done DESCDONE; (2) on the E800 bit12 kick, if
[$743a]==[$7a14]==$748a and C000 presets in-window, DMA buffer->host + raise identity-gated IRQ4; (3)
clear m_win_buf at DESCARM. Registered STORAGER_FAITHXFER.

**RUN (FAITHXFER+FILLMAP+AAFIX+DESCTRACE):** removing the pre-emption changed the state fundamentally -
BUILD 0->1, DONE10 ($1310) 0->1 (the identity re-arm DID route once - progress the pre-emption blocked),
but the read now PARKS HARD at 0x36 (PARK36 0->21982, $159c +$26=$A), UNPARK never fires. During the read
[$7a14]=0000 and [$7a64]=0000 (both were 748a/1 at the 6.41 setup); ARM ($3cd4) runs 1298x but the $748a
identity/window are NOT set in the read window. My Piece-2 kick NEVER fires during the read: no E800 bit12
RISING EDGE (matches cont.278's "0 kicks during read") - the $3cd4 arm doesn't present a bit12 edge my
handler sees, and c000=e70000 (out of window) at the only 2 kicks (@6.41 setup, NOT-ARMED).

So this is Dave's "parks-before-arming (larger)" case: the ladder reaches $3cd4 but [$7a14]/[$7a64] aren't
set during the read and the transfer is never edge-kicked -> the read sits at 0x36. OPEN QUESTIONS for the
next ground-truth pass: (1) why [$7a14]=0000 during the read (the $3cd4 arm runs but $0c88/$1418 that set
[$7a14]=$748a don't, or it's cleared)? (2) why no E800 bit12 edge - does the $3cd4 arm's "E800 |= $1000"
reach the model's E800 handler, or is bit12 a stuck level (m_e800_bit12_prev)? (3) what does the fw wait
on at 0x36 (21982 park spins)? Baseline (FAITHXFER off) unchanged: VOL1/HDR1 byte-exact, no 0x82.

## cont.282 (2026-07-21) — the 0x36 wait is the segment engine, not the pump counter; seek is a resolved transient

Ran Dave's STORAGER_PUMP836 tap set (STORAGER_FAITHXFER=1). Findings, in order of discovery:

1. The op-ladder $156A is the spin loop; op $0028 dispatches (table $192[$28]) to handler
   **$6788 = the floppy SEEK/POSITION op**, NOT $836C. ($836c disasm = `move #$2700,SR`, an
   IRQ-disable epilogue; Dave's $836C label is mislabeled/off. P836 tap fired 0x.)
2. $6788 gate: A6=[$799a]=UIB $6e60; **UIB[$12]=0x40** -> `0x40 & 3 == 0` -> takes the early
   leg $67a2/$67a8: proceed only if [$7a36]!=0 OR byte[$0014].4 (dead: $0014 has no writer,
   vector space). Target cyl0/hd0 == current cyl0/hd0 (position already matched) but the early
   gate returns **$FE** before checking position -> re-enter spin.
3. **[$7a36] is a SEEK-SETTLE-COMPLETE flag.** Set immediately at $661a if settle param
   (A1[$14]&$f)==0, else scheduled via a TIMER ($6612: bsr $29f8, delay = param*10). In the
   run it is set **once, at t=7.962077, by the timer callback (pc=$2b8a)** -> SEEK->positioned
   ($683e) at 7.962163. So the $FE spin 7.90->7.962 is a **normal ~62 ms seek settle**, a
   resolved transient. ($65d8 scheduler path never taken in-window; the immediate-$661a path
   never taken; the write comes from the timer callback via the `lea $7a36,A0` pointer.)
4. **Post-settle (t=7.963) the segment engine is the true steady-state blocker**, confirming
   Dave's model. PUMP836 `956--` ($70a0) snapshot @7.963537:
     74b8=0000  7b42=0000  7956=0008  74c0=0000  7abc=0008(seed confirmed)
     7302=29c0(IRQ6=ID-walk, pre-arm; NOT $8552)  7a64=0000  F000=08a0(bit2=0)  743a=748a  7a14=0000
   Gates A ([$74b8]) / B ([$7b42]) / C ([$7956]) all dead; **W7B42 never fires in the read
   window** (only at t=0.01-0.16 setup) -> no IRQ4 segment-complete. [$74c0]/[$74b8] never
   populate; F000 bit2 (segment/drive ready) is clear. Engine is starved of the IRQ4 bootstrap
   exactly as Dave predicted: IRQ4 -> $4696 sets [$7b42]=1 + refills [$74c0] -> $83c8 copies
   [$74c0]->[$74b8], [$7956]-=[$7b46] -> arm leg $846a checks F000 bit2, writes E800, installs
   [$7302]=$8552 -> IRQ6 data pump paces [$7956]->0 -> [$7a64]=1 -> $3dbc unpark +$26=$c ->
   $13d2 sets [$7a14]/kicks bit12.

Decision inputs for the fix (Dave to adjudicate): (a) raise a descriptor-engine IRQ4 once the
Gate-2/fill-map queue is built (so $3bfe->$4696 sets [$7b42]/[$74c0]); AND (b) F000 bit2 must
read ready or the arm leg $846a bails (board.yaml currently tags F000 bit2 "write-fault sense,
conf R" -- likely wrong). Piece 2 should then key on the E800 write from the SEGMENT arm
($846a), not the $13d2 bit12 (which is the later host-DMA arm, downstream of unpark).
Instrumentation added (all env-gated STORAGER_PUMP836 / STORAGER_DESCTRACE, baseline untouched):
PUMP836 six-site tap set + W7B42; DESCTRACE SEEK/SEEKok/SEEKgo/SEEKret, SETTLECHK/TIMERSCHD/
SET7NOW/W7a36, LADOP handler-resolve, GATEA/GATEB/PUMPFOUND/PUMP36/OP36.

## cont.283 (2026-07-21) — UNPARK trace: outcome #1 (handler off-route); [$7956] underflows past 0

Ran Dave's STORAGER_UNPARK on BASELINE (FAITHXFER off, the case where the walk drains [$7956]).
Killed the IRQ4-bootstrap/F000-bit2 story (Dave's correction): the unpark at $845c/$8460 fires
the instant [$7956]==0 && [$7958]==0 && [$72e2]==0 -> +$26=$c on [$71bc]; F000 bit2 ($8472) is
on the $846a still-working branch, NOT a completion gate.

Result = Dave's outcome #1: **PUMPH ($8416/$836c) never fires; the pump $15fe never runs in the
read window at all** (PSELECT/PUMP36 empty). The unpark handler is simply not on the read's
executed route. W7956<-0 only at setup (t=0.01-0.14); the drain-to-0 is the parallel $7c34 IRQ6
ID-walk. CPU spins in the $2axx scheduler/timer area (SPINFLAG pc=$2aac/$2afa/$2b8a).

Authoritative RUNTIME record layout (WAITDUMP @12s) -- SUPERSEDES the disasm read of $3990 (that
was the BUILD-time +6=$836c; firmware phase-swaps the handler):
  [$72d6]=727e {+0=0000 +2=0248 +6=$7964}   pump-eligible (+0==0) but phase-gated
  [$72d8]=7286 {+0=ffff +2=0248 +6=$3dbc}   <- the UNPARK-POSTER, but +0=$ffff -> pump SKIPS it ($160e tst+bne)
  [$72da]=728e {+0=0000 +2=7208 +6=$9188}   pump-eligible but phase-gated
  [$72dc]=7296 {+0=ffff +2=0248 +6=$94ec}   skipped (+0=ffff)
  [$72de]=729e {+0=ffff +2=0248 +6=$9984}   skipped
  [$72e0]=72a6 {+0=ffff +2=0248 +6=$9398}   skipped
  71b6=0000 st=02  71bc=71f0 st=00  727c=72d6

Phase-gate dead: NODE71F0+$26 = word $000a -> byte[$26]=$00 (st=00). op-ladder op-$36 park writes
the phase as a WORD ($159c/$15a0 move.w #$a,($26,A2)); the pump reads it as a BYTE
(cmpi.b #$a,($26,A1)) at offset $26 = big-endian MSB = $00. Never matches.

[$7956]=$ffed at 12s -- underflowed PAST 0 into negative: the walk decrements but the ==0 unpark
check lives in a handler that never runs, so it over-decrements (two-consumer split, at the limit).

OPEN question for Dave (the real dispatch question outcome #1 points to): what is supposed to
(a) run the pump $15fe during the read, (b) ACTIVATE the $3dbc unpark record (+0: ffff->0) and/or
(c) satisfy the pump phase-gate (byte[$71f0+$26]==$a) -- so the pump calls $3dbc / the [$7956]==0
unpark actually executes. The $3dbc record being +0=$ffff (pump-skipped) is the cleanest lead.

## cont.284 (2026-07-21) — LADWAIT: [$71b6] never installed; op-42 re-dispatched then scheduler idles

Ran Dave's STORAGER_LADWAIT on baseline. Answers to his three questions:

Q3 (does [$71b6] install for this read?): NO. W71B6 fires only at setup (t<=0.099, memset
c8c8/c9c9). For the monitor's options-0x00 read, [$71b6]=0000 all read-window. Confirms the
secondary gate (board notes: [$71b6] installed at $1144 only for IOPB options bit4) -- the pump's
low-record eligibility (cmpi.b #$a,($26,[$71b6])) would read a null node and skip every record.

Q2 (which completion cell does $6bc2 poll, satisfied?): op-42 handler $6bc2 = `tst.w $798e;
blt $6cda` then channel/transfer setup off [$71bc]+$18 / [$799a] UIB. The ladder RE-DISPATCHES
op-42 ~200x at 7.90-7.92; across all samples the cells [$7a3e]/[$7a40]/[$7a36] stay 0000 and
never satisfy. (They ARE later written by the $2axx seek-settle timer callbacks -- SPINFLAG
[$7a40]<-7360 @7.96 pc=$2aac, [$7a3e]<-0001 @9.79 pc=$2b8a -- but after $6bc2 stops looking.)

Q1 (does [$7424] advance to 0x42/0x36 or stuck early?): the ladder walker $156a runs until ~8.3s
and advances through the op-list toward the end (OPLIST@12: [cur]=7262, list tail ..$42 $36 $00).
After ~8.3 NEITHER $6bc2 nor $156a runs -- the CPU idles in the $2axx TIMER-LIST scheduler
($2aba/$2aac/$2afa), i.e. the scheduler has gone quiescent and nothing re-queues the parked
(phase-$A) command into the dispatcher. Late activity reaches pc=$8344 ($3dbc-adjacent) @11.94.

Net (Dave's doctrine read, confirmed): the firmware armed a read channel op and its scheduler
parked waiting to be woken by a channel/operation-complete the model never returns; the pump /
$3dbc / $7286-activation are all downstream of that one missing return-of-state. TWO things are
off the normal path for this options-0x00 read: (primary) the re-dispatch wake, (secondary) the
[$71b6] install. Open for Dave: name the exact completion the op-42 channel arm waits on (the
$6bc2/[$798e] path and what a real gate array returns to wake the scheduler).

## cont.285 (2026-07-21) — REDISP: re-dispatch WORKS; the sole blocker is the pump's byte-vs-word phase-gate

Ran Dave's STORAGER_REDISP on baseline. Read-window (t>=7.9) counts:
  DISP-ENTER($2244)=0  DISP-LOOP($2250)=250  PH-READ($2258)=250(ph7216=000a=$A)
  PARK36($159c)=0  PUMP($15fe)=250
Setup phase (6.40-6.43) all fire too incl PUMP; PARK36 stamped phase $A back there.

RESULT = a THIRD outcome, cleaner than Dave's A/B: the dispatcher IS looping, reads the phase as
a WORD and correctly gets $A ($2258 move.w ($26,A0),D0 -> $000A), and DISPATCHES THE PUMP -- the
pump runs 250x. So re-dispatch WORKS (Outcome B "dispatcher exits / needs a re-queue trigger" is
REFUTED; no interrupt/wake needed) and the dispatcher's own phase read is fine. The block is
INSIDE the pump: PUMPFOUND($1646)=0 -- it never selects a record.

Mechanism (same run, DESCTRACE cross-check): the pump's record-select gate is a BYTE read
  $161c/$162a: cmpi.b #$a,($26,A1)
For the read's record slots (all A0<=$72e2) the LOW gate is used, A1=[$71bc]=$71f0, and
GATEB shows byte[$71f0+$26]=$00 (40x). But the op-36 park wrote the phase as a WORD $000A
(big-endian: byte[$26]=$00 MSB, byte[$27]=$0A LSB). So: dispatcher WORD-reads $71f0+$26 -> $A
(works); pump BYTE-reads $71f0+$26 -> $00 (fails, never == $a) -> PUMPFOUND=0 -> no +6 handler,
no $3dbc unpark. The HIGH gate ([$71b6]) is never reached (walk stays low) and is null anyway
(options-0x00, $1144 install skipped).

So Gate 1 = the cont.253 phase-byte-gate, now the PRIMARY and only remaining blocker. It is pure
state, NO interrupt -- the clean floor Dave predicted. OPEN for Dave: the park writes the phase as
a WORD at $71f0+$26 (byte[$26]=$00, byte[$27]=$0A); the pump byte-reads offset $26 and needs $a.
Is the model placing/parking the phase at the wrong offset/width, should the pump node [$71bc]
point one byte higher (so +$26 hits the LSB $0A), or is [$71b6] supposed to be installed for this
read and point at a node whose byte[$26]=$a? Name which, and it's a small pure-state edit.

(Note: RTC-seeded flake -- cont.284's run had PUMP absent at t>=7.9; this run PUMP fires 250x.
But when it runs it selects nothing, so the phase-byte-gate is the true blocker either way.)

## cont.286 (2026-07-21) — OPTBIT4: bit4 is a per-COMMAND ROM flag, CLEAR for the read -> not a fetch bug

Dave's OPTBIT4 check, adapted after tracing the guard: the [$71b6] install ($1144) is gated by
btst #4,D5 where D5=[UIB+$20], written at $0e7e from D6 = the $92 op-descriptor FLAGS word
(A1=$92+cmd*4, D6=second word of the entry). So bit4 is a per-COMMAND ROM constant, NOT a
host-IOPB options field -- the auto-fetch-fidelity theory does not apply.

Runtime (baseline):
  DESC-BIT4 cmd=95 D6=8c27 bit4=0    (read)
  INST-BIT4 cmd=95 D5=8c27 bit4=0 -> SKIP 71b6
  cmd=87 D6=0820 bit4=0 ; cmd=89 D6=0026 bit4=0    (none set bit4)
  INSTALL1144 = 0 (never installs)

=> [$71b6] is never installed for the read command BY ROM DESIGN. The read is NOT supposed to use
the pump's high-gate. This is Dave's SECOND outcome: the options-0x00 / bit4-clear read does not
complete via the pump/high-gate path at all.

So the contradiction resolves the other way: the LOW gate byte-reads $71f0+$26 = word $000A ->
$00 and structurally cannot match cmpi.b #$a either. Neither gate is the consumer for a bit4-clear
read. Yet the firmware DID build op-36 into the read's ladder (OPLIST tail ..$42 $36 $00) and it
parks at phase $A. OPEN, sharpened for Dave: what consumes the op-36 phase-$A park for a
bit4-clear read (0x95), given neither the high-gate ([$71b6] uninstalled by ROM) nor the low-gate
(byte[$71f0+$26]=$00 from the word write) can select the record? i.e. what is the real completion
/ unpark path for a options-0x00 read, and is the model steering it onto the pump path wrongly, or
starving that real path of a state it should return? The $3dbc unpark record ($7286) is activated
at $3ef6/$40f8/$840a (channel-completion sites) -- likely the real path is a channel-completion
that activates $3dbc directly, bypassing the pump.

## cont.287 (2026-07-21) — XFERDRAIN: $6f44 consumes an EMPTY fill-map before capture; walk then steals [$7956]

Ran Dave's XFERDRAIN on baseline. The read's completion path ($6f44 fill-map consumer, NOT the
pump) DID execute -- once -- and it lands both of Dave's failure modes at once:

  BUILD ($6f44)  7956=0000 fillpos=0  @7.963413  (x2, never again)
  XFER  ($702e)  7956=0008 fillpos=0  @7.963464
  DRAIN ($70a0)  7956=0008 fillpos=0  @7.963537   sub.w D3(=0),$7956 -> no change
  WIN64 ($70a6)  7956=0008 fillpos=0  @7.963538   7956!=0 -> [$7a64] NOT set
  WALK-- ($7ebe) 7956=0008->0003 @8.05-8.11 ... 7956=fff0 fillpos=7 7a64=0001 @11.5-11.9

Outcome #1 (dominant): $6f44 runs at 7.963 with fillpos=0 -- the fill-map has NO positive slots
yet -- so D3=0, $70a0 drains nothing, $70a6 sees 7956=8 and never sets [$7a64]. $6f44 fires
exactly 2x (both @7.963413) and NEVER re-runs.
Outcome #2 (compounding): the $7ebe walk then owns [$7956], draining 8->0->underflow fff0 over
8.05-11.9, and the fill-map only reaches fillpos=7 by ~11.5 -- long after $6f44 consumed it empty.
[$7a64] does reach 1 (via the walk path $825c, not $70a6) but the $3dbc unpark in the $6f44 path
($70f4) never runs with a populated map, so no completion.

Root: STATE-ORDERING. The model's flux capture populates the fill-map (via the IRQ6 $7ebe walk)
AFTER the firmware's ladder reaches op-4a/$6f44, so $6f44 consumes an empty map. On real hardware
capture-complete precedes $6f44. Two coupled fixes to weigh (Dave to pick): (a) the fill-map /
capture-complete must precede $6f44's drain (deliver captured slots before op-4a runs, or gate
op-4a on capture-complete); (b) [$7956] is the transfer counter $70a0 owns -- Gate 2's $7c34/$7ebe
walk decrementing it is on the wrong path and steals the zero-landing; the walk should move off
[$7956] (or not run once the transfer path is active). #1 is primary: even without the walk, $6f44
at fillpos=0 drains nothing.

## cont.288 (2026-07-21) — R7426: completion needs the walk's $fe-terminator MISMATCH path; Gate 2's aim-match starves it

Ran Dave's STORAGER_R7426 on baseline + traced the walk. The single completion gate is $808a
(routes to $82e2 ledger-scan ISR -> $6f44 re-run -> $3dbc unpark IF [$7426]!=0; else $810e
dead-end). Findings:
  GATE808a  27  (all see 7426=0 -> DEAD810e 27x; boundary hit fillpos=8 7dac=fe 7daf=08 7426=0)
  SET-7cac   0  (the TERMINAL accept that STICKS -- never fires)
  SET-7d62 268  (sets 7426=1 but $7f1a clr.w $7426 on the scan path wipes it before $808a)
  SCAN82e2   1  (ran once @8.144 with fillpos=0 -- empty map)
  $7f1a: clr.w $7426  (confirmed the Gate-2 scan clear)

The terminal accept $7cac is reached ONLY via the walk's MISMATCH path:
  $7c34: tst [$79ae]; beq $7ce6            <- [$79ae]==0 -> generic path (taken 249/250)
  $7c40: A0=capture ptr; D0=captured byte
  $7c46: cmp [$7436],D0; beq $7c52         <- MATCH stays in walk
  $7c50: bra $7c70                         <- MISMATCH -> $7c70
  $7c70: cmpi.b #$fe,$7daf; beq $7c84      <- terminator
  $7c7a: cmpi.b #$fe,$7dac; bne $7d4a      <- ([$7dac]==$fe also routes to accept)
  $7c84 -> $7ca4 -> $7cac: move.w #$1,$7426  (TERMINAL, bypasses $7f1a)
Runtime: WALK34=250, P-7ce6=249, MATCH52=1, MISS50=0, DEC7c70=0. The walk bails to $7ce6 via
[$79ae]==0 EVERY sector -- never reaches the compare, so never mismatches, so never reaches the
terminator accept. And even the compare path needs a MISMATCH ($fe terminator) which Gate 2's
aim-match would deny.

ROOT (the Gate-2 coupling, exact): completion requires the window-boundary sector to (a) have
[$79ae]!=0 so the walk does the terminator compare, and (b) present the $fe terminator (a
MISMATCH vs [$7436], and/or [$7daf]/[$7dac]==$fe) so it routes $7c50->$7c70->$7c84->$7cac, setting
[$7426]=1 on the STICKING path (not $7d62, which $7f1a clears). Gate 2 (cont.282) made every sector
aim-MATCH to grind [$7956]/fix the aim -- which is precisely what denies the boundary mismatch and
starves $7cac. This is Dave's shape-1 "detection is capture": the model must stage the $fe
terminator mark at capture-complete/window-close (and ensure [$79ae] gates the compare) so the
firmware's own walk completes. Pure boundary-state, no injected interrupt. Await Dave's exact
one-site edit (couples with Gate 2 -- must not re-break the aim).

## cont.289 (2026-07-21) — faithful IAM at the physical index: presents correctly, but [$79ae] is never armed for this read

Implemented Dave's faithful boundary edit (replaces the rejected cont.288 $79ae/fabricated-$fe
steer): in flux_advance_to, at the physical index crossing (m_flux_next_index) during an armed
read (m_serdes_active, cmd 0x95/0x94), DMA the real Index Address Mark {A1 A1 A1 FE FF} to the
fw-published capture cells [$7dac..$7db0] and raise IRQ6 -- the same pointer-directed capture path
as every IDAM. NO [$79ae]/[$7426] poke, no fabricated sector. Env-gated STORAGER_IAM (registered
in the passthrough whitelist); baseline untouched.

RESULT: the IAM presents correctly -- 56x over 14s at 200ms spacing (once/rev @300RPM), 6 of them
IN the read window (7.57-8.57). But completion does NOT fire: DEC7c70=0, SET-7cac=0, [$7426]=0.

ROOT (traced): [$79ae] is NEVER armed for this read. $739a (its ONLY writer, unconditional once
reached) never runs -- ARM79ae=0 even ungated from t=0. And the entire arm path is off-route:
$9400 rw-setup=0, $947c bsr $7346=0, $9412 arm-READ=0, $7346 resid-setup=0. So the walk $7c34
takes the [$79ae]==0 bail to $7ce6 EVERY pass (P-7ce6=249/250), never reaching the terminator
compare -- regardless of the IAM. Dave's premise "the firmware's residual math has already armed
[$79ae]" does NOT hold: $739a is a strict one-shot reached only via $9400/$7346 rw-setup, which
never executes for the options-0x00 / bit4-clear label read. ($7ce6, the generic path, drains the
ledger but never re-arms [$79ae]; there is no other [$79ae]=1 site in the ROM.)

So the completion path we mapped ($7c34 armed -> compare -> $fe mismatch -> $7c70 -> $7cac ->
[$7426]=1 -> $808a -> $82e2 -> $6f44 re-run -> $3dbc) is NOT on this read's route, because its
entry gate [$79ae] is never armed. OPEN for Dave: (a) is $9400 rw-setup supposed to run for this
read (what state is the model failing to return that keeps it off-route)? or (b) does the
bit4-clear read complete via a DIFFERENT path (not $7c70/$7cac)? or (c) is a model arm of [$79ae]
at the IAM the faithful stand-in for a boundary re-arm the firmware genuinely doesn't do for this
variant (the rejected-patch fallback Dave flagged)? The IAM presentation itself is correct and
kept; the blocker is strictly upstream at the [$79ae] arm.

## cont.290 (2026-07-21) — post-capture PC census (Dave's method: read the route, don't forward-map)

Pointed STORAGER_PCHIST_WIDE at the post-capture window (PCHIST_AT=8.3 PCHIST_END=9.5) on baseline.
Top PCs (491 distinct, 300k samples) cluster in THREE regions - all machinery we already mapped:
  ~25%  $22xx-$23xx : the TOP-LEVEL op-dispatch scheduler $2290-$23e6 (reads phase D0=[$71bc+$26],
                      dispatches via table $222[phase] at $2312, then runs op-ladder [$721a]+6 at $2342)
  ~15%  $15xx       : the op-ladder walker $156A + pump $15fe
  ~7%   $6bxx-$6cxx : op-42 ($6bc2 wait region)
No unmapped completion poster appears. Per Dave's read: "spinning in scheduler + $810e => the read
reaches no completion poster on its own route; the missing state is owed much earlier (arm/setup)."

Steady state = continuous ID-search RE-ARM: the fw executes $891a (E000=$22f ID-arm -> delay ->
$23f park) over and over; the model defers an IRQ4 (300us) each time (IRQ4defer-ARM pc=00891a/$89b6).
So capture keeps re-arming but nothing posts completion.

CONCRETE LEAD (model-caused, from the census tail): **LEGCEN NOHANDLER aim=0008 (128x)** with
L1-4=07 08 09 0a (the fill-map positive slots) and 7956=fffa. The ledger SCAN ($7e58/$32ac) reaches
the last position aim=8 but the byte there is a POSITIVE fill-map slot# (07/08/09/0a, written by the
cont.276 FILLMAP change), which is NOT one of the scan's recognized handler bytes (ff re-aim / f0
pending / c0 done / fe beyond-window / aa) -> NOHANDLER -> the scan does nothing and cannot advance
or complete at the boundary. So there are TWO INCOMPATIBLE consumers of the $7654 ledger: $6f44
(fill-map) WANTS positive slot# bytes; the scan/$7e58 WANTS handler bytes. The FILLMAP positive
bytes satisfy $6f44 but break the scan's boundary dispatch. Also seen: $7d62 sets [$741c]=1 (walk
tail runs); 7956 underflowed to fffa. Open for Dave to read: whether the real route is the scan
(and FILLMAP's positive bytes are actively breaking it), or the model owes an arm/setup-time state
that never got returned (why $9400 rw-setup / the $6f44 consumer branch is off-route). No forward-map.

## cont.291 (2026-07-21) — CORRECTION: NOHANDLER is a code label ($7d4a), not a fault; FILLMAP-off makes capture WORSE

Ran Dave's step-1 A/B: FILLMAP OFF (AAFIX/FWDONE kept), census 8.3-9.5. Refutes the cont.290
interpretation AND Dave's prediction:

1. CORRECTION (mine): "LEGCEN NOHANDLER" is the model's LABEL for PC=$7d4a (source line 3804/3833:
   {0x7d4a,"NOHANDLER"} / {0x7d4a,"IDAM-DEFAULT"}) = the walk's GENERIC/DEFAULT branch ($7c80 bne
   $7d4a -> $7d62). It is NOT a fault and NOT caused by FILLMAP positive bytes. It fires 156x
   (FILLMAP off) vs 128x (on) because it's just the normal walk path. cont.290's "positive bytes
   break the scan" was a misread of an instrumentation label.

2. FILLMAP-off ledger $7654: c0 c0 ff ff ff ff ff c0 c0 aa -- INCOMPLETE. positions ~2-6 stay ff
   (wanted/uncaptured), never fully c0. The scan re-aims to the ff gaps forever; no clean c0..c0 aa.
   With FILLMAP capture reached fillpos=8; without it capture is INCOMPLETE. So FILLMAP was helping
   the capture COMPLETE, not merely adding positive bytes. Removing it is strictly worse.

3. $1a54 (STAMP80) fired only at 6.4s (setup, aim=0), never in the read window. No 0x80, no boot.

So the "simple scan route completes once freed of FILLMAP's positive bytes" hypothesis is refuted:
the scan ($7d4a/$7d62) IS on-route (fires either way) but never completes because the ledger never
reaches the terminal state - WITH FILLMAP the bytes are positive (not c0), WITHOUT it there are ff
gaps. Neither is the clean c0..c0 aa the scan needs. The blocker is that CAPTURE never stamps all 8
window positions to c0 on this route (positions 2-6 = the R9-R13 "00" data sectors stay ff without
FILLMAP; with FILLMAP they went positive not c0). Open, factual: why do the middle window positions
never reach c0 via the firmware's own $810e stamp? That is the next thing to READ (not forward-map).

## cont.292 (2026-07-21) — REFUTES model-side hypothesis: model captures ALL 8; the ledger is FROZEN during the read

Dave's census (STORAGER_C0CENSUS: DAM/1st-IRQ5 + DATADONE + m_flux_skip, correlated by R):

1. MODEL COMPLETES ALL 8 in-window sectors, INCLUDING the blanks R9-R13:
   DATADONE R=07..0e nb=128 want=128 skip=0 (data-record-end fires, full 128B field recovered);
   DAM (1st IRQ5) fires for each; m_flux_skip=0 for all R7-R14. So the SERDES recovers the all-0x00
   FM data field and raises the completion mark for the blank sectors exactly as for VOL1/HDR1.
   => Dave's leading hypothesis (blank sectors' record-end never fires in the flux path) is REFUTED.
   The "2 vs 8" is NOT a model capture failure. Delivery to host works; recovery is honest.

2. The firmware's f0/c0 CAPTURE-STAKE path ($92b4/$92f6/$9312 = move.b #$f0,ledger[aim]) NEVER runs
   during the read (STK-* = 0 in 7.99-9.85). Off-route, like the pump / $7c34 / $9400 rw-setup.

3. THE LEDGER $7654 IS NOT WRITTEN DURING THE READ. Two independent write-taps (mine + the existing
   IAMRD "LEDGWR") both = 0 across 3.5-11.0s. The ledger state (c0 c0 ff ff ff ff ff c0 c0 aa) is
   FROZEN from setup (<3.5s); the read's capture does not reach it. ($810e, where the IRQ5 dead-ends,
   is `move.w $7430,(A0)+` - NOT a ledger write; my earlier "$810e stamps c0" was wrong.)

CONSISTENT PICTURE (consolidates cont.286-291): the model captures + delivers all 8 sectors, but the
firmware's IRQ5 handler routes every mark to $808a -> $810e (dead-end) because [$7426]=0, and never
stamps the ledger. Every [$7426]-setter is off-route for this read variant: $7cac needs [$79ae]
(never armed - $9400 rw-setup off-route), $7d62 sets it but $7f1a wipes it, $933c/$9342 need $92xx
(off-route). So completion is blocked at [$7426]=0, the ledger stays frozen, and $1a54 (0x80) only
fires at 6.4s setup, never in the read. The blocker is NOT capture and NOT the flux path - it is that
NO on-route path durably sets [$7426], so the ledger-scan completion never engages. Open, factual:
what SETS [$7426] on the variant this read actually takes (the scheduler $2290-$23e6 / op-ladder /
op-42 / walk $7ce6-$7d62 route from cont.290) - since the three setters we mapped are all off-route.

## cont.293 (2026-07-21) — CONFIRMS cont.262 root: the f0-stake never fires ($7950 fork pinned no-stake). Retraction of failed-tap results.

RETRACTION: several instrumentation results earlier this turn came from Python tap-installs that
FAILED SILENTLY (wrong anchor -> file written unchanged, git diff empty). Invalid, disregard:
"$92b4=0 / W7426=0 / LEDGWR=0 / ledger frozen" from those non-installed taps. Re-read below uses
the PRE-EXISTING taps (PHFORK line 4434, secmap line 4445), which are real.

CONFIRMED (Dave's cont.262 root, clean instrumentation): the IRQ6 alternator fork is pinned to the
NO-STAKE path.
  PHFORK: $89f2 (no-stake ID) = 30x ; $92b4 (f0-stake) = 0x ; [$7950] = 0000 (21x) / 0100 (9x),
  bit0 = 0 in all 30. So every data-record IRQ6 forks $89f2, the move.b #$f0,ledger[aim] stake at
  $92b4/$92f6 NEVER runs, and NO capture is ever recorded. Everything downstream (ledger scan,
  [$7426], the terminator, the pump, FILLMAP, Gate-2) is compensation built on an unrecorded capture
  - which is exactly why each piece turned out off-route. cont.262 named this; we walked away for 30
  continuations; it is the root.

CORRECTION to cont.292 "ledger frozen": the ledger IS written, but only at SETUP -- secmap shows
c0c0 word-fills at pc=$1280 (the #$c0c0 init loop) @6.40s. The read's capture writes NOTHING to the
ledger (because the stake never fires). The c0 c0 ff .. aa state is the setup c0-fill with some
positions reset to ff/aa. So "capture never writes the ledger during the read" holds; "frozen" was
imprecise (setup writes it once).

STILL VALID (committed C0CENSUS tap, cont.292): the MODEL completes all 8 sectors (DATADONE nb=128
skip=0, incl blanks) -- the flux path is honest; the 2-vs-8 is not a capture failure.

ROOT, faithful framing (Dave): the $7950 alternator parity is determined by the count/order of
IRQ5/IRQ6 the model emits per sector (model cadence = 1 IRQ6 ID + 2 IRQ5 [data-AM + data-done]).
That cadence pins the data-record IRQ6 at phase-0 (no-stake). Open, faithful question: does the
model's per-sector mark cadence match a real VGC7219's, such that the data-record IRQ6 would
naturally land at phase-1 (stake)? NOT force the phase (C135PH proved forcing -> 0x82); check the
cadence/parity. This is upstream of everything touched since cont.262.

## cont.294 (2026-07-21) — BREAKTHROUGH: one IRQ5/sector unpins the $7950 parity; the f0-stake FIRES

Dave's parity math (worked out): every IRQ6 leaves $7950 bit0 at 0 (stake's own toggle-from-1, or
the ID-round $8A38 clr). So the bit between two ID marks = parity of the data-IRQ5 count in the gap.
The model emitted 2 IRQ5/sector (data-AM + data-done) = EVEN -> next ID-IRQ6 always old-bit 0 ->
$89f2 no-stake -> $92b4 never. Self-reinforcing PARITY pin (not timing) - which is also why C135PH
(force the bit) failed.

A/B: STORAGER_ONEIRQ5 suppresses the SECOND data-IRQ5 (the cont.266 data-done at line 830), keeping
the first (data-AM setup, needed). Result (verified edit, git diff non-empty, build clean):
  PHFORK: $92b4 (STAKE) = 3x  (was 0!) ; $89f2 (no-stake) = 30x
  Ledger $7654: c0 f0 ff ff ff ff ff f0  -- positions 1 and 7 now hold f0 (STAKED), where with
  2 IRQ5 the ledger was frozen at the setup c0-fill. The f0 capture-stake FIRES for the first time.
=> Dave's hypothesis (1) CONFIRMED: the cont.266 second IRQ5 was a PARITY regression; it flipped the
gap parity even and starved the stake. Dropping to one IRQ5 -> odd parity -> the stake fires.

PARTIAL, not full boot yet: only 2-3 of 8 positions stake (positions 1,7 got f0; 0=c0 setup, 2-6=ff);
no 0x80 in the read window (STAMP80 still only the 2 setup stamps @6.4s). So the parity root is
proven and the stake is unpinned, but the cadence isn't cleanly staking all 8. Open (Dave's (1) vs
(2)): is it hypothesis (1) needing refinement (the per-sector IRQ5/IRQ6 interleave across sectors
isn't uniformly 1-per-gap, so only some ID-IRQ6 land odd), or (2) the data-record-end should be an
IRQ6 (carry-strobe) rather than merely dropping the second IRQ5? Next: census the exact per-mark
{level, old-bit, fork} sequence across sectors to see why only 2-3 gaps reach odd parity.

## cont.295 (2026-07-21) — full mark census: parity PROVEN (3 stakes), remaining blocker is the AIM advance

Extended PHFORK to all 4 forks (89f2/92b4=IRQ6, 7ba8/8018=IRQ5; verified installed). ONEIRQ5 on,
IAM off. Per-mark sequence {fork, old-bit, aim}:
  92b4(stake) 7ba8  aim=1     first 3 sectors: 2 marks each (1 IRQ6 + 1 IRQ5), odd parity -> STAKE
  92b4(stake) 7ba8  aim=7
  92b4(stake) 7ba8  aim=8
  [89f2 x3, 7ba8 x1] repeating, aim STUCK at 1     -> 4 marks/group (even) -> no more stakes
Counts: 92b4=3, 89f2=30, 7ba8=30, 8018=0.

READING (Dave's decision, both separated):
- Parity is PROVEN: the first 3 ID-IRQ6 land old-bit 1 and stake (aims 1,7,8). ONEIRQ5 works.
- The remaining blocker is the AIM ADVANCE: [$7428] goes 1->7->8 then STICKS at 1 instead of
  walking the 8-sector window. Once stuck, the firmware RE-READS aim=1 every rev, and each re-read
  emits 3 IRQ6 (no-stake) + 1 IRQ5 = 4 marks (even parity) -> the extra IRQ6 flip parity back even
  -> no further stakes. So the aim-stick and the late parity-drift are the SAME fault: the stuck
  aim generates the extra marks. (8018 = IRQ5 old-bit 1 never fires; all IRQ5 -> 7ba8 old-bit 0.)

So the two couplings tangled since Gate-2 are now cleanly separated: PARITY (solved by ONEIRQ5,
the cont.266 second-IRQ5 was the regression) and AIM-ADVANCE (the next lever). The stake now
records captures (ledger f0 at the staked positions); the read stops because the aim won't walk
0..7. Next lever: why [$7428] sticks at 1 after 1->7->8 (the $7e58 aim-match / [$7daf] cursor /
$8114-vs-$831c aim traffic - Gate-2 territory), now that the parity confound is removed. NOT a
phase poke; the aim advance is firmware-walk state driven by the mark cadence/ledger the model
returns. Keep ONEIRQ5 for aim work (it's the correct cadence for the stake).

## cont.297 (2026-07-21) — stake fires, CONVERT doesn't: f0 never becomes c0, aim locks. [$742c] NOT pinned.

Control-checked census (CTL-89f2=20, taps in the PROVEN PHFORK ~4438 block; retract cont.296's
dead-block zeros). 3 runs (RTC flake: 2 stake, 1 doesn't):
  run1/3: PHFORK-stake=3  STAKE-92f6=3  CONVERT-933c=0  [$742c]={0,1}  ledger c0 f0 ff.. (f0 stays)
  run2:   stake=0 (flaked - RTC-seeded)
=> CONFIRMED (Dave's extension): the STAKE (event 1, $92f6/$9312 move.b #$f0,ledger[aim]) FIRES
(3x, writes f0), but the CONVERT (event 2, f0->c0) NEVER fires - the ledger stays f0, so the aim
never advances off the staked position. CORRECTION to Dave's leading hypothesis: [$742c] is NOT
pinned at 1 - it alternates {0,1} (the FE-leg does clear it). So the convert is starved for a
different reason than [$742c] stuck.

DISASM (the convert path): the stake $9304-$9326 tests [$79b6] (HD-flag) at $9322; beq $9370 for
the floppy ([$79b6]=0) SKIPS the $933c convert region ($9328-$9362, incl the $9362 c0-write). So
$933c is HD-gated - NOT the floppy's convert. The floppy c0-convert is $6ff0 (move.b #$c0,(-1,A0))
inside the $6f44 TRANSFER BUILDER (the fill-map consumer from cont.287, which then ran on an EMPTY
map). Now the stake writes f0, so the fill-map has entries - the question is whether $6f44/$6ff0
now consumes them to c0. That couples the parity win (stake->f0) directly to the cont.287 transfer
builder: $6f44 was starved of a populated map; ONEIRQ5 now populates it via the stake.

NEXT: census $6ff0/$726c (c0-convert) + $6f44 entry with the f0 fill-map present - does the transfer
builder now convert f0->c0 and advance the aim? Control-checked taps in the ~4438 block.

## cont.298 (2026-07-21) — the convert's ENCODING MISMATCH: stake writes f0 (negative), $6f44 wants POSITIVE

Convert-path census (control CTL-89f2=20; staking runs): STAKE-92f6=3, BUILD-6f44=2 (the transfer
builder RUNS), but CONV-6ff0=0 / CONV-726c=0 / CONV-9362=0 - NO c0-convert fires. Ledger stays f0.

DISASM (the convert gate $6fda-$6ff0, the floppy c0-convert inside $6f44):
  $6fdc: D4 = (A0)+ (ledger byte); $6fde: bge $6fe8  -> POSITIVE (bit7=0) converts to c0 at $6ff0
  $6fe0: move.b #$ff,(-1,A0)  -> NEGATIVE resets to ff (re-wanted)
=> $6f44/$6ff0 converts only POSITIVE ledger entries. The stake $9312 writes #$f0 = NEGATIVE. So
$6f44 sees the f0 stakes, takes the negative branch, and RESETS them to ff. The capture cycles:
stake->f0 -> $6f44 resets->ff -> re-stake->f0 -> ... never c0, aim never advances. This is the
mechanism behind "stake fires but convert doesn't" (cont.297) and the ff-dominated ledger.

So the two-event contract has an ENCODING GAP for the floppy: STAKE writes f0 (negative) but the
TRANSFER-BUILDER $6f44 wants a POSITIVE slot number. cont.276 FILLMAP (write positive slot#) was
papering over exactly this - which is also why FILLMAP "helped capture complete" (cont.291): it fed
$6f44 the positive bytes it wants. The real question (Dave's ledger-lifecycle domain): where is f0
SUPPOSED to become positive between the stake and $6f44? Candidates: an intermediate FE-leg/scan
step ($7e1e/$7e90) that rewrites f0->slot#, or the capture-stake for the floppy should write the
slot# not f0, or $6f44's input pointer/base is off so it reads the wrong cells. NEXT: find who (if
anyone) writes a POSITIVE slot# to $7654 on the floppy path - that's the missing f0->positive step.

## cont.299 (2026-07-21) — DECISIVE: ZERO firmware positive-writers; f0->positive is the gate-array's capture-into-slot event

Census (secmap tap, proven block, control CTL-89f2 passing; staking seed). The ONLY $7654 writes in
the read window (t>7.9):
  pc=$9318  <-f0  : the STAKE ("capture noticed" - address mark passed)
  pc=$6fe6  <-ff  : $6f44 RESETTING the rejected f0 (its negative branch $6fe0)
  pc=$7068  <-fe  : beyond-window
  => NO POSITIVE write anywhere (count=0). Confirms Dave's prediction.

So f0->positive (the "data landed in slot N" event that $6f44 consumes) is NOT a firmware
instruction - it is the GATE ARRAY's capture-into-slot event (cont.275): at the sector's data
record-end, the GA DMAs the recovered sector into a local slot and writes ledger[pos]=slot#. The
model fires event one (the stake, parity-fixed) but never event two, because it SHORT-CIRCUITS
flux->host and never uses the slot pool. So $6f44 keeps seeing f0 (negative) and resetting to ff.

THE FAITHFUL FIX (picks the whole path, doctrine-clean): at the DATA record-end (flux_advance_to,
the second-IRQ5 site - which under ONEIRQ5 is where the data-done lands), the model DMAs the
recovered sector into a slot and writes ledger[pos]=slot# (positive), REPLACING the flux->host
short-circuit. This is FILLMAP-done-right (cont.276 wrote the positive byte but at the wrong
layer/time -> collided with the scan). Dave's caveat: the slot# must land at ledger[AIM] (the pos
the stake used, [$7428]), not the geometric position, or $6f44 and the scan disagree again (the
FILLMAP collision). So: record-end -> slot DMA -> ledger[aim]=slot#. Retires the flux->host shim
flagged at the very start.

NEXT: implement the capture-into-slot at record-end (ledger[aim]=positive slot#), env-gated, and
verify $6f44 now converts f0->c0 and the aim advances. Await Dave's exact edit (pos=aim vs geometric,
slot# source, timing relative to the stake).

## cont.300 (2026-07-21) — SLOTMAP encoding fix WORKS, but $6f44 runs once (pre-capture) and never re-runs

Implemented Dave's STORAGER_SLOTMAP (env-gated; verified): at DATA record-end write ledger[aim] =
(m_flux_r & 0x7f) = the positive slot#, overwriting the stake's f0. Result: the write LANDS - ledger
becomes "c0 0d ff ff ff ff ff" (position 1 = 0d = R13, positive, at [$7428]=aim=1). Encoding fix
confirmed correct.

BUT the convert still doesn't fire (CONV-6ff0=0). TIMING is the cause:
  $6f44 (BUILD/scan)  @7.96341  x2 (one pass) - scans ledger, SCAN-6fde reads D4=0000 at all 8
                                  positions (A0=7655..765c) - the ledger is EMPTY at scan time
  first STAKE-92f6    @8.04908  - captures start ~90ms AFTER $6f44
  SLOTMAP 0d writes   @8.04-8.07 - the positive slot#s land AFTER $6f44's only scan
  last BUILD-6f44     @7.96341  - $6f44 NEVER re-runs
So $6f44 scans the empty ledger once at 7.963, finds nothing, and never re-runs after the captures
populate it. The encoding is now correct but unconsumed - same single-early-run as cont.287.

=> The blocker is no longer encoding; it's the $6f44 RE-RUN. $6f44 must run (or re-run) AFTER the
record-ends populate the ledger. What re-triggers $6f44? It's the op-4a dispatch / transfer-complete
path. Leading hypothesis: $6f44 re-runs on the transfer-complete IRQ4, which the flux->host
short-circuit SUPPRESSES (no real transfer -> no completion -> no re-run). If so, Dave's step-2 (the
faithful slot-DMA replacing flux->host, which raises the real transfer-complete) is what re-triggers
$6f44 - meaning steps 1 and 2 may not be separable: the encoding needs the slot-DMA's completion to
drive the re-run. NEXT: census what triggers $6f44 (op-4a dispatch) and whether a transfer-complete
IRQ4 is what re-runs it - that decides whether step-2 (slot-DMA) is required now or a lighter re-run
trigger exists.

## cont.301 (2026-07-21) — the $6f44 re-run is WALK-driven (not IRQ4) but gated by [$7b10], a one-shot armed at setup

Control-checked walk census (CTL-89f2=20; ONEIRQ5+SLOTMAP). Dave's correction confirmed: the re-run
is walk-driven, not IRQ4. Findings:
  WALK-7c34=20 WALK-7d02=20 RERUN-7d4a=20 (walk RUNS, reaches $7d4a 20x @8.07-8.51 AFTER captures,
    cursor [$7daf] ADVANCING 9->14->7->8, aim stuck=1) ; GATE-7e58=0 ($7d4a reached via other entries,
    NOT the cursor-vs-aim gate - so it's NOT outcome-2/Gate-2 cursor==aim)
  BUILD-6f44=2 @7.96341 (scans EMPTY, SCAN-6fde D4=0000 all 8) ; RE7106=1 @7.96355 ; CONV-6ff0=0
  first stake @8.049 - captures land AFTER the only $6f44 pass

THE GATE (disasm): $7d68 tst $7b10; beq $7d9c -> the re-run $7d94 bsr $7106 fires ONLY if [$7b10]!=0
(and $7d6e clears it - one-shot). [$7b10] is set to $ffff at EXACTLY ONE site: $6ed6, called from
$948e (the read-arm/$9400 setup). So [$7b10] is armed ONCE at read-setup, consumed by the FIRST
mark's walk (the single $6f44 re-run @7.96, EMPTY ledger), never re-armed. The 20 post-capture
$7d4a walk hits all see [$7b10]==0 -> skip the re-run.

=> ROOT of the missing convert: the $6f44 re-run trigger [$7b10] is a ONE-SHOT armed at read-setup
that fires on the first mark (before captures), scans the empty ledger, and is never re-armed after
the captures populate it. Encoding (SLOTMAP) is correct; parity (ONEIRQ5) is correct; the re-run
just fires too early and once. NEXT (Dave's domain): what should re-arm [$7b10] after each capture /
after the window fills? Is the arm ($6ed2/$948e) supposed to run per-capture, or should a mark
handler re-set [$7b10] so the re-run fires against the populated ledger? That is the last coupling -
the convert is one re-arm away.

## cont.302 (2026-07-21) — OUTCOME 2: stuck at the first op-36 park; op-4a never re-dispatches -> no re-run

Control-checked (CTL-89f2=20; ONEIRQ5+SLOTMAP). Ladder-progression census:
  OP4A-6ed2 (op-4a dispatch)  = 1  (only @7.96341 - the single early pass, empty ledger)
  PARK36-159c (op-36 park)    = 20 (STUCK at the first data-phase park)
  DESCGO-417a                 = 0
  BUILD-6f44=2 / CONV-6ff0=0
=> Dave's OUTCOME 2 confirmed: the read is stuck at the first op-36 park. The read ladder
(24 28 56 58 1A 18 54 4A 42 36 00, continuation 28 54 4A 42 36 00) never reaches the continuation's
2nd op-4a, so [$7b10] never re-arms and $6f44 never re-runs against the SLOTMAP-populated ledger.
No [$7b10] poke - the faithful re-run IS op-4a recurring, which requires leaving the first park.

So the LAST lever is the first op-36 park's EXIT condition - the unpark question from cont.283-289,
re-read now that the two things it ultimately needs are SOLVED this session: parity (ONEIRQ5) gives
the stake, SLOTMAP gives the positive slot#. A recorded, correctly-encoded capture now sits below
the park for the first time. The park exit ($3dbc +$26=$c, gated on [$7a64]<-[$7956]==0 etc.) may
now be satisfiable where it wasn't three sessions ago. NEXT: census the op-36 park exit condition
(what pass-one waits on: [$7956]/[$7a64]/$3dbc/the pump select), now with stakes+slot# present.
Session arc: parity (cont.294) -> stake fires; SLOTMAP (cont.299) -> encoding correct; re-run is
op-4a recurring (cont.301/302) -> gated on leaving the first op-36 park. The park exit is the finish.

## cont.303 (2026-07-21) — FINISH LINE: [$7956] now DRAINS to 0; the $8460 unpark CONDITION is met, but the handler doesn't run

Control-checked park-exit census (CTL-89f2=20; ONEIRQ5+SLOTMAP). THE KEY COLUMN:
  [$7956] MOVES across the read: 8(17x) 7(7x) 6(7x) 5(91x) 0(23x) - it DRAINS now (impossible before
  this session: nothing staked -> nothing to drain). At ALL 20 PARK36 hits: 7956=0000.
  Park snapshot (x20): 7956=0000 7958=00000000 72e2=0000 7a64=0000 7426=0000 7428=0000
=> Dave's outcome 1's key is MET: [$7956] drains to 0. And the $8460 unpark CONDITION
([$7956]==0 && [$7958]==0 && [$72e2]==0, cont.283) is now SATISFIED at the park - the first time
in the saga. BUT the read still parks, and [$7a64]=0 (the $3dbc gate).

The nuance (cont.287 two-drain coupling, now decisive): [$7956] drained via the WALK subq ($7eb2),
NOT via $6f44's $70a0 tail. $70a0 is what sets [$7a64] on its zero-landing (sub.w D3,[$7956]==0);
the walk pre-drained [$7956] to 0, so $70a0 (if it ran) subtracts into negative and never sets
[$7a64]. So the $3dbc path ([$7a64]-gated) can't fire. Meanwhile the $8460 path's condition IS met
but its handler ($836c) doesn't run (cont.283: PUMPH=0, the handler is off-route).

So the finish is ONE of: (a) make the $8460 unpark handler run now that its condition
([$7956]/[$7958]/[$72e2] all 0) is finally met - re-read cont.283's "why $836c never runs" with the
drained state; or (b) route [$7956] drain through $6f44/$70a0 (not the walk) so [$7a64] sets and
$3dbc fires - the cont.287 "walk steals the counter" fix. Both are firmware-path reads, not pokes.
Session arc COMPLETE to the gate: parity->stake, SLOTMAP->slot#, drain->0, unpark CONDITION met.
The only thing between here and boot is which unpark handler acts on the now-satisfied condition.

## cont.304 (2026-07-21) — park-loop census: pump out (structural), $70a0 re-run gap; A/B refutes "walk subq steals", real zeroer is $7c02 RESET

Control-checked park-loop census (CTL-89f2=20; ONEIRQ5+SLOTMAP):
1. PUMPSEL-1646 = 0 - the pump NEVER selects a record. Option 1 ($8460 via pump/$836c) is
   structurally out for bit4-clear (cont.286 byte-vs-word gate). Confirmed.
2. DRAIN-70a0 = 1 @7.96, D3=0000 - $70a0 runs once (empty ledger, $6f44's only pass), never again.
   The re-run gap. Confirmed.
3. WSUBQ-7ebe = 3 (8->5); DISARM-7ed8 = 0.
4. A/B (STORAGER_NOWALKSUBQ blocks the $7ebe subq): [$7956] STILL 0 at all 20 PARK36, [$7a64] STILL
   0. => "the walk subq steals the counter" is REFUTED - the subq isn't the zeroer.

The REAL zeroer: $7c02 move.w #$0,$7956 (in the walk, beside $7bf0 move #$0,$7428 the aim reset). So
the walk RESETS [$7956] (and the aim) to 0 - it is NOT a capture-count drain. CORRECTION to cont.303:
[$7956]=0 at the park is 8->5 (subq x3) then 5->0 by the $7c02 RESET, not 8 sectors counted down. The
"$8460 unpark condition met" is thus partly a reset artifact, not a genuine capture completion - less
close than cont.303 read it.

HONEST STATE: both unpark paths remain blocked. $3dbc needs [$7a64] via $70a0's zero-LANDING (sub.w
D3 hitting 0 with D3>0), but $70a0 runs once with D3=0 (empty) and $6f44 never re-runs (park gate);
$8460's condition is (artifactually) met but its handler $836c is off-route (pump never selects,
structural). Neither Dave option is clean: option 1 structural-out, option 2's "walk steals" refuted
+ the re-run gap remains. NEXT (fresh read, not derived): what gates $7c02 (why the walk resets
[$7956]/aim mid-read), and whether $70a0 could ever land the zero with a populated ledger - since the
walk reset + the D3=0 empty pass are the two things keeping [$7a64] unset. This region has been
mis-mapped repeatedly; read $7bf0-$7c34 (the walk reset gate) before deriving the next edge.

## cont.305 (2026-07-21) — Part A read inventory: $9400 NEVER reached (outcome 2); INIT(87)/89 COMPLETE, read(95) doesn't; 95 IS armed

Control-checked (PA-CTL=24). Part A (doorbell + completion + arming census):
A1 - the monitor issues 4 commands: 00, 87(INIT), 89, 95(READ) @0.4-1.35s. ONLY ONE read (0x95).
A2 - cmd=87(INIT) and cmd=89 COMPLETE (0x80 at $1a54); cmd=95(READ) does NOT. Zero 0x82. => 87/89
     are the golden reference we've lacked - firmware commands that complete.
A3 - $9400 (PA-ARM9400) is NEVER reached by ANY command; $5fc0/$6102 never installed via $9412/$940a.
     => Dave's OUTCOME 2: $9400 is NOT this firmware's read path; the arming-via-$9400 hypothesis is
     REFUTED. BUT the read IS armed: [$79ae] (PA-ARM79ae=1) and [$7b10] (PA-ARM7b10=2) fire for cmd=95
     via NON-$9400 paths (so $6ed2/$739a have callers other than $948e - corrects cont.301). [$71b6]
     never installs (bit4-clear, cont.286).

So the divergence is NOT arming - the read IS armed. 87/89 complete WITHOUT data transfer (INIT/seek
class); the read (95) is armed but its DATA-TRANSFER completion stalls (the park/pump/re-run maze).
The reference (87/89 -> $1a54) is a NON-data completion, so it may not diff usefully against a data
read. NEXT: (a) is there a completing DATA path anywhere (v1.80/sgic, or does 0x95 ever complete in
any config)? (b) trace 87/89's path to $1a54 and see which step the armed-but-stalled 95 diverges at.
The read being ARMED (flags set) yet stalled at data-transfer completion re-centers on the transfer
itself, not the dispatch/arm - i.e. back to the two-event contract (stake OK, convert blocked by the
$6f44 re-run gap, cont.301-304), now known to be downstream of a correctly-armed read.

## cont.306 (2026-07-21) — IAM re-test broke the [$7426] barrier; the divert is the STAKE's own side effects, not an HD leak

IAM re-test (ONEIRQ5+SLOTMAP+IAM, control CTL=20): the [$7426] BARRIER IS BROKEN - [$7426]=1 (3x),
$7cac fires, $82e2 ledger-scan ISR runs 3x. cont.288/289's "dead" conclusion OVERTURNED (cont.289's
[$79ae]-never-armed was the Part A mismeasurement). (The IAM itself didn't fire=0; [$7426]=1 came via
the natural SLOTMAP path, so the IAM isn't needed for the trigger.) BUT still no convert: op-4a
doesn't re-dispatch (OP4A=1), park doesn't exit (PARK36=20), [$7b10] never re-arms, $82e2's own $6f44
re-run ($8330) is also [$7b10]-gated, so BUILD-6f44 stays at 7.96 (empty). CONV-6ff0=0.

Dave's [$79ba]-divert lead (7-gate census at $822c, control-checked): CONFIRMED the evaluator diverts
at [$79ba]!=0 - but the OWNER is NOT an HD leak. [$79ba]=1 is set at $931c, which is the STAKE's OWN
TAIL: $9312 stake f0 -> $9318 clr $741c -> $931c set $79ba=1 -> $9322 tst $79b6 (floppy skips $933c).
So the stake (this session's parity fix) sets [$79ba]=1 AND clears [$741c]=0 as side effects, every
time it fires, for the floppy too. The 7 $82b2 gates at EVAL-822c: THREE fail - 79ba=1 (stake-set),
7956=5-7 (draining, not 0 yet), 741c=0 (stake-cleared, must be !=0); the other 4 hold (79b6=0 7958=0
796a=1 727e=0). ENABLE-82b2=0 (exit-enable never fires).

So it's NOT one stale flag - it's the STAKE's own [$79ba]=1 / [$741c]=0 writes (plus 7956 not-yet-0)
diverting the completion evaluator. On real HW the stake sets these and the read still completes, so
either the HD-convert path (floppy skips it) clears them, or the evaluator's gate timing differs, or
$82b2 isn't the floppy's completion path. OPEN (Dave's domain): after the stake sets [$79ba]=1 /
[$741c]=0, what restores them for the floppy so $822c passes to $82b2 -> [$7968]=1 -> deferred re-run
-> convert? The stake<->evaluator interaction is the seam. Session net: barrier broken, completion
path live, diverted by the stake's own side effects at the first evaluator gate.

## cont.307 (2026-07-21) — TOPOLOGY CORRECTION: the floppy convert is $933c/$9354 (direct IRQ6), starved on [$742c]==0 - NOT $6f44

Dave's topology correction, measured (control CTL=20): $92b4 tst $742c; bne $92f6(STAKE) / fall-through
$92be->$92f4 bra $933c->$9354(CONVERT ledger[aim]=c0). $933c is reachable for the FLOPPY via [$742c]==0
(the $9322 HD-gate is inside the STAKE tail only, downstream - cont.298/306 mislabeled it).

Measurement:
  FORK-92b4 = 3, [$742c]=0001 at ALL 3 -> every fork takes the STAKE branch; NONE the convert.
  C0WRITE-9354 = 0 -> the real floppy convert NEVER fires. FELEG-7e1e = 3 (the clear-leg runs).
=> Dave's OUTCOME 1: the floppy convert is $933c/$9354 - a DIRECT IRQ6 handler, NO re-run, NO [$7b10],
NO continuation, NO park-exit. It is starved because [$742c] is never 0 at the $92b4 fork. THE WHOLE
cont.297-304 $6f44/re-run/park/[$7b10]/circularity was chasing the WRONG convert ($6f44 = the host-DMA
transfer-queue builder, a different function). Also: the stake sets [$7426]=1 at $92fe, so the
"cont.306 [$7426] barrier broken" was the 3 stakes, not a separate completion event.

THE LEVER = [$742c]: SET at $7ea4/$7eb2 (walk) + $7162/$7bf6/$95da; CLEARED at $7e42/$7e90 (FE-leg) +
$7ca8 (terminator). At the fork it's 1 -> the walk's SET wins over the FE-leg's CLEAR. Which ledger-byte
case the scan ($7e58) takes decides SET-vs-CLEAR, and the read hits the SET case every fork. This is a
[$742c]-timing/cadence question - the DIRECT SIBLING of the parity fix (ONEIRQ5 got the stake to fire
via IRQ6 phase; this needs the convert's IRQ6 phase-1 to arrive with [$742c]==0). NEXT: which ledger[aim]
byte value routes the scan to $7ea4/$7eb2 (SET) vs $7e42/$7e90 (CLEAR), and does SLOTMAP's positive
slot# push it to the SET branch? Read the $7e58 dispatch on ledger[aim] -> [$742c] set/clear. The
convert is a direct IRQ6 - the circularity is gone; the finish is one [$742c] flip.

## cont.308 (2026-07-21) — the convert chain is LINEAR (no loop) and SLOTMAP-independent: it's the [$79ba]/$82b2/[$7968] gate

SLOTMAP-off A/B (control CTL=20): [$742c]=0001 at all 3 forks WITHOUT SLOTMAP too; C0WRITE-9354=0. So
SLOTMAP does NOT push [$742c] to 1 - hypothesis REFUTED. The walk's own SET ($7eb2, 3x) does, running
after the FE-leg CLEAR ($7e90, 3x).

DISASM (the walk dispatch $7e8a): tst $7968; beq $7eb2 -> [$7968]==0 SETS [$742c]=1 ($7eb2); [$7968]!=0
CLEARS it ($7e90). So the [$742c] SET-vs-CLEAR is gated on [$7968] (the exit-enable). Full chain, LINEAR
(the circularity is gone):
  convert $9354  <-  [$742c]==0  <-  [$7968]!=0 (CLEAR $7e90)  <-  $82b2 exit-enable  <-  $822c 7-gate pass  <-  [$79ba]==0
And [$79ba]=1 is set by the STAKE ($931c). So cont.306's [$79ba] and cont.307's [$742c] are the SAME
chain: the stake's [$79ba]=1 blocks $82b2 -> [$7968]=0 -> walk SETs [$742c]=1 -> convert starved.

=> The root gating the floppy convert is the $82b2 exit-enable, gated at $822c on [$79ba]==0 (+ the other
6: 79b6=0 7956=0 7958=0 741c!=0 796a!=0 727e=0). cont.306 measured 3 failing at $822c: 79ba=1 (stake),
7956=5-7 (draining), 741c=0 (stake-cleared). The stake sets [$79ba]=1 AND clears [$741c]=0 - two of the
three - as its own side effects ($931c/$9318). So the parity fix that fires the stake also trips the
exit-enable. NEXT: what restores [$79ba]==0 and [$741c]!=0 for the floppy after the stake, and when does
7956 reach 0 at $822c - i.e. does the $82b2 precondition EVER align in one pass? Read [$79ba]'s clear
($a476, D0=0 case) and whether the 7 gates ever co-hold. This is the finish: one aligned $822c pass ->
$82b2 -> [$7968]=1 -> [$742c] clear -> $9354 convert. Linear chain, no loop, SLOTMAP-independent.
