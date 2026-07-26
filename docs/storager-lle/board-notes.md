# Storager — current understanding (narrative companion to `board.yaml`)

**`board.yaml` is the source of truth.** This file explains what its keys mean and carries the
flow/module reasoning that doesn't fit cleanly in key–value form. Both are a *synthesis* of the
append-only log (`DESIGN.md`) and the commented disassembly
(`siemens/disasm/storager/storager_v260.asm`) as of **cont.407 (2026-07-24)**. Where anything here
disagrees with the log, **the log wins** — this is a distillation, not a new authority.

## What the board is

An on-card **MC68000** disk controller (Interphase 3030-class; Siemens S26361-F415) for the
PC-MX2/MX300, driving 5.25″ QD floppy, ESDI hard disk, and 1/4″ tape. The 68000 talks to exactly one
custom device, the **VGC7219 gate array**.

**The data path (corrected cont.405 from Plamen's board photo):** the board has a **74LS1801F +
74LS1802A ENDEC** that does FM/MFM **data separation only** — it recovers bit clock and data
(RXDAT/RXCLK/SYNC) and detects address marks. There is **no separate SERDES MSI part**; the cont.242
"74LS1811 ENDEC + 74LS1812 SERDES" reading was wrong. The **VGC7219 gate array itself** deserialises,
does the CRC-16/CCITT, and bus-masters the Multibus DMA. The **AM2147-55 (4K×1)** is the gate array's
serial bit buffer — about one sector, which is why the read/transfer is a strict per-sector pipeline.
A local SRAM buffer at `0x4000–0x7FFF` stages sector data and holds the firmware's node/ledger state.

## The doctrine that shapes everything (`board.doctrine`)

Four rules govern the model:

1. **LLE, no shim.** The firmware runs every command end to end; the model provides only gate-array
   *hardware* behaviour. No env-var may change shipped behaviour. (The cont.353 clean rebuild dropped
   all HLE — no flux→host short-circuit, no `s_desc`.)
2. **Stimulus/response only.** The VGC7219 has no intrinsic program; it does exactly what the firmware
   commands, register access by register access. Never build autonomous behaviour.
3. **Detection is capture** (the cont.211 keystone). An address-mark IRQ *always* arrives with the
   just-passed field bytes already in the capture cells, so the model **stages the mark's record
   before raising the IRQ**, through the firmware's published `UIB+$C8..$CE` POSPTR pointers.
4. **DMA holds the CPU (DTACK)** — the cont.405 keystone. A bus-master transfer takes the local bus
   and stalls the 68000 for its duration. The model **must hold the CPU**, or it races ahead and
   desyncs the firmware's per-transfer bookkeeping. Model it as the **level** it is (cont.406) —
   `suspend()` at the kickoff, `resume()` at the transfer end — not as a fixed interval, which other
   code on a five-CPU machine can pre-empt; the duration then falls out of the byte count. Applies to
   reads, writes, format, and the HD path. It looks like a firmware/logic stall but is really a race.

## The interface, in one pass (`address_map`, `registers`)

- **Host DMA address is the `C000` up-counter**, one's-complemented at launch — *not* a latch through
  `C800`/`D800`/`D000` (all three are local). It increments during the DMA; the firmware advances it
  +sector-size per sector.
- `C800` = **write ports into the gate array's field sequencer**, not a file (cont.406): op18 pushes 16
  descriptors through only three offsets, writing cell 0 six times over. Cell 0 doubles as the live SRAM
  chunk pointer at runtime. `D000` = the **local DMA source/dest** word address. `D800` = the **`$7DAC`
  template** word address.
- `E000` = the **16-word channel command block** (`E000..E01E`, nothing above) that op18 builds from a
  per-unit param block in **RAM at `[$7938]`=`$6E84`, i.e. from the UIB** — not a ROM template. The
  window opens on the **`E000` bit11 engagement write** (the per-command arm — *not* E802 bit15).
  `E01E` = the per-record status latch (reading it ACKs the record).
- `E800` bit12 kicks the DMA; bits 13/14 are the direction. `F000` reports drive + timer status; its
  **bits 8/9 are the gate-array op-status** used only by op42 Branch B (seek/status), not the read.

## Interrupts and the alternator (`interrupts`)

IRQ2 is the host doorbell/completion; IRQ4 is channel/DMA-done; **IRQ5/IRQ6 are the per-record
data/ID captures**. The subtle part is the **`$7950` alternator**: every `$29xx` stub does
`bchg #0,$7950` and forks on the *old* bit, and IRQ5 and IRQ6 share the toggler — so **parity counts
across both levels**. The correct per-sector cadence is an **even 4-event cycle** (`VERIFY×2, SETUP,
DONE`, alternator `1,0,1,0`) so parity returns to its start each sector; an odd count net-flips it and
desyncs the routing. The forks: `$298C` (IRQ6) → `$89F2` verify (old 0) or `$92B4` stake (old 1);
`$29C0` (IRQ5) → `$8018` done (old 1) or `$7BA8` setup (old 0).

**Measured cadence (cont.406, opcode taps on AS_OPCODES — instruction fetches don't go through
AS_PROGRAM):** per sector the firmware enters **four** handlers, `verify(1), verify(0), setup(1),
done(0)`, and parity returns to its start. The **pointer ownership is the reverse of the old reading**:
`$7BA8` (setup) *advances* `[$741e]`; `$8018` (done) *writes* `C800[0]=[$741e]`, arming the chunk for the
**next** field (`$899A` re-arms it between the two verifies). So `C800[0]` during field *k* is the arm
written at field *k−1*'s done — which is why the first field of a read has no destination at all.

## The command lifecycle (`flow.command_lifecycle`)

Host writes `GO 0x13` to PIO `0x73F8` with an IOPB pointer → gate array raises IRQ2 and auto-fetches
the **0x18-byte** IOPB into node `$71F0` → intake stamps status `0x81` busy → the command builds and
walks its micro-op ladder → completion stamps `0x80` (or `0x82`+code), transcribed to the host IOPB →
the CPUAP monitor reads it and proceeds. The monitor **never times out** — it spins on `0x81` forever
— so the only host-visible failure modes are a wrong status or a false-early one.

## The read (`read_model`) — the counted, op18-programmed operation

The read is a **counted, finite** gate-array operation: read exactly the IOCB sectors (`[$7abc]`, 8
for the label) then terminate — not free-running. The ladder (`…16,18,42,46,00`) dispatches via
`table[$192+opcode]`: **op18 (`$308C`) loads** the field program (C800 descriptor pushes + the E000
step block, built from the UIB — see below), **op42 (`$6BC2`) waits**, op46 executes.

**Per sector the gate array raises four interrupt-events** (an even, parity-balanced cycle):
IRQ6 (ID address mark) → `$89F2` verify (C/H/R staged at `$7DAC`); IRQ5#1 (data field armed) → `$7BA8`
setup (**advances** `[$741e]`, writes `0xf0` *pending* into the ledger); IRQ5#2 (data field done) →
`$8018` done (stamps `$74C4[slot]+2=0x40` READY, stakes the ledger, and **writes** `C800[0]=[$741e]`
to arm the *next* field — see the measured pointer ownership above). The
model delivers **8×IRQ6 + 16×IRQ5**. Dropping the second IRQ5 (an earlier "1 IRQ5 = captured"
assumption) dead-ended every sector at `$7BA8` setup so `$8018` never ran — that was the long-standing
read stall.

**The ledger** at `$7654` (drain window `$7654+[$7954]`, slots 1..8): `0xc0` is the **idle/resting**
value (all-`c0` *before* capture — not "staked"); staking advances `[$7424]` (READY slot) 0..7 and
`[$7428]` (ledger slot) 1..8, **linearly** — the old alternating-slot symptom is cured by the 2-IRQ5
cadence. **The data buffer is linear**: the firmware assembles a **1024-byte buffer `$4000–$43FF`**,
the chunk pointer advancing +128 bytes per sector. The model stages each sector into the **commanded**
chunk (`C800[0]<<1`), never a self-computed address — computing `$4000 + index·len` puts each sector a
record ahead of the firmware's pointer, so its DMA sources the previous chunk and the last sector
(`HDR1`) is never transferred at all.

**Reads are in NATIVE sectors** (128 B FM / 256 B MFM / 1024 B ESDI-HD), *not* 512-byte blocks
(cont.260, media-verified). `mx2-001.imd` cyl0 head0 (FM 16×128B) holds the label — R7 `VOL1SINIX0`,
R8 `HDR1 NSC Boot`; cyl0 head1 is unformatted (`0xe5`). So the count-8 label read is 8 native sectors
on one track — no track boundary, no head switch.

## The transfer (`transfer`) — and the DTACK breakthrough

The firmware commands each sector's DMA through the channel node (`$7442`) descriptor: `C000` = host
dest (advancing `+0x80`), `D000` = local source (advancing `$4000/$4080/…`), `E800` bit12 kickoff
(setup at `$3cd4`/`$3d08`). The kickoff code **RTS's immediately** (`$3d4c`) — no inline poll — **but
the gate array holds the 68000 off the bus for the DMA duration**. Modelling that hold — as a level,
`suspend()`/`resume()` bounded by the byte count — is what makes the chain run: **without it the drain launches
only 2 of 8 sectors** (the CPU races ahead and re-sets `[$741c]`/`[$7968]` before the completion
processes); **with it all 8 launch**, `D000` walks `$4000..$4300`, and the descriptor queue `[$74b4]`
drains to 0. The IRQ4 handler `$3BFE` clears the busy-lock `[$7a1a]` and re-dispatches the next pending
channel via `$3afe`→`$3cd4`.

## Completion (`completion`) — the remaining frontier

With the DTACK hold the read reaches **8/8 transfers, `[$74b4]=0`, `[$7a64]=1`** — but stays at `0x81`.
The completion is gated, upstream of everything, on **op42 (`$6BC2`) returning 0**, which needs a
status word `(A1)` bit7 and the settle flag `[$7a36]` (`$661a`). Downstream, `node+$26=0x0C` (= host
`0x80`) is set at `$843e` (which also clears `[$72d6]`), gated on `[$7968]=0`; and the terminate chain
`[$7b40]→[$727e]→[$7968]→[$72d6]` only fires when the node enters the **transfer phase** (`node+$20`
bit14, set by `$974c` when `[$7a70]=ffff` at `$9dd4`). At runtime that transfer-phase state is **never
entered** — the DMAs physically complete, but the firmware never marks the node "in transfer phase," so
the terminate chain never fires. That single missing **state transition** is the last knot.

## What op18 actually does — and the micro-sequence hypothesis (cont.406)

op18 does **not** fill a register file. It pushes a **16-descriptor stream** through three `C800` write
ports, each push framed `E800` write-enable (`$2ABD`) → `C800` write → `$6000` strobe → `E800` idle
(`$2A5D`), and each carrying a **one-bit selector** — expressed as *which byte strobe (UDS vs LDS) is
asserted on a dummy read at `$6000`/`$6001`*. `$6000` is zeroed SRAM (the `$073A` boot clear sweeps it
and nothing ever fills it), so the data is meaningless; only the access and its byte lane matter. The
board already uses address-as-payload for the `C000` counter (bits 23–16 on A1–A8), so this is house
idiom, not an anomaly. Then it block-writes the 16-word `E000` field program.

**Speculation, flagged as such:** the shape argues this is a **micro-sequence load**, not a register
fill — writing the same cell six times is a no-op as storage but six pushes into a queue, and the eight
identical `(7e,3f,1)` pushes are eight entries. Read that way the three offsets are three sequencer
**write ports**, the `$6000` read is the **push/advance strobe** with a per-word qualifier bit, and the
E800 pair is the load-enable window. So **op18 = load a program the gate array then runs, op42 = wait
for it, op46 = field-execute** — there *is* a gate-array micro-sequence running for the read before op42
waits, and the model implements none of it. The `E000` words support this: they look like
`{flags, byte-count}` steps (`3f`=63, `29`=41, `09`=9, `3b`=59), the variant paths flip bit9/bit5 per
word, and `023f` tails the block as an idle/terminate step — which maps onto gap, sync, AM, ID(4),
CRC(2), gap, sync, DAM, data(128), CRC.

**Falsifiable prediction:** the **eight** pushes to cell `$3F` equal the **eight** commanded sectors. If
changing the IOCB count changes that push count, cell `$3F` is the per-sector step queue — and the model
can take the sector count *from the gate-array programming* instead of snooping `[$7abc]`, which is
exactly the desnoop that's queued next. It would also explain the dropped first sector: if the loaded
sequence has an initial step before the first data field, hardware's first capture lands one step later
than the model's — precisely the one-record skew measured.

## Open frontier (`open_frontier`)

1. **First field has no chunk** — R=01 is dropped (see above); test the op18 push-count prediction first.
2. **The op18 selector bit is unmodelled** — `$6000` is plain RAM in the model, so the bit is discarded.
3. **The data/control swap-control bit** — control blocks swap bytes, data must not; the model keys on
   field type as a stand-in for a bit that should live in the IOCB/UIB.
4. **op42 return-0 / transfer-phase state** — the sole remaining *completion* gate. Find what `(A1)` points to
   inside op42 and why `(A1)`-bit7 / `[$7a36]` don't assert after the 8 transfers; and what stimulus
   makes the firmware set `[$7a70]=ffff` / enter `node+$20` bit14 (likely the transfer-channel node
   `$7442` becoming `[$799a]`-current, or a status `$9dd4` reads — it checks `[$71bc]` first byte
   `== 0x8b`). Carry the DTACK discipline in; a debugger breakpoint on `$6c8e` (op42's done check) is
   the safest probe. Opcode taps *do* work when installed on **AS_OPCODES** (that's how the cont.406
   handler cadence was measured) — the earlier false negatives came from installing them on AS_PROGRAM,
   where instruction fetches never appear. Don't trust `pcbase()` for counting inside a handler,
   though: `c800_w` saw 144 real op18 pushes (9 identical program loads × 16) but attributed only 16
   of them to the right PC. Prefer a hardware-visible discriminator — the E800 bit7 load-enable
   identifies a push with no PC involved at all.

## Keeping this current

When the log advances, update `board.yaml` first (it's the source of truth), then this note. The
`superseded:` block in the YAML records readings the log overturned — add to it rather than deleting,
so a corrected claim can't creep back in.
