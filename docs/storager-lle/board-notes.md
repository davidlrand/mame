# Storager — current understanding (narrative companion to `board.yaml`)

**`board.yaml` is the source of truth.** This file explains what its keys mean and carries the
flow/module reasoning that doesn't fit cleanly in key–value form. Both are a *synthesis* of the
append-only log (`DESIGN.md`) and the commented disassembly
(`siemens/disasm/storager/storager_v260.asm`) as of **cont.405 (2026-07-24)**. Where anything here
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
   and stalls the 68000 for its duration. The model **must hold the CPU** (`spin_until_time`) during
   every DMA, or the CPU races ahead and desyncs the firmware's per-transfer bookkeeping. This is not
   optional and applies to reads, writes, format, and the HD path. It looks like a firmware/logic
   stall but is really a timing race.

## The interface, in one pass (`address_map`, `registers`)

- **Host DMA address is the `C000` up-counter**, one's-complemented at launch — *not* a latch through
  `C800`/`D800`/`D000` (all three are local). It increments during the DMA; the firmware advances it
  +sector-size per sector.
- `C800` = the **field-boundary offset file** (16 field bit-positions op18 programs; cell 0 doubles as
  the live SRAM chunk pointer). `D000` = the **local DMA source/dest** word address. `D800` = the
  **`$7DAC` template** word address.
- `E000` = the **16-word channel command block** op18 loads from a class ROM template; the window opens
  on the **`E000` bit11 engagement write** (the per-command arm — *not* E802 bit15). `E01E` = the
  per-record status latch (reading it ACKs the record).
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

## The command lifecycle (`flow.command_lifecycle`)

Host writes `GO 0x13` to PIO `0x73F8` with an IOPB pointer → gate array raises IRQ2 and auto-fetches
the **0x18-byte** IOPB into node `$71F0` → intake stamps status `0x81` busy → the command builds and
walks its micro-op ladder → completion stamps `0x80` (or `0x82`+code), transcribed to the host IOPB →
the CPUAP monitor reads it and proceeds. The monitor **never times out** — it spins on `0x81` forever
— so the only host-visible failure modes are a wrong status or a false-early one.

## The read (`read_model`) — the counted, op18-programmed operation

The read is a **counted, finite** gate-array operation: read exactly the IOCB sectors (`[$7abc]`, 8
for the label) then terminate — not free-running. The ladder (`…16,18,42,46,00`) dispatches via
`table[$192+opcode]`: **op18 (`$308C`) arms** the gate array (C800 field bit-positions + the E000
command block from a class ROM template), **op42 (`$6BC2`) waits**, op46 executes.

**Per sector the gate array raises four interrupt-events** (an even, parity-balanced cycle):
IRQ6 (ID address mark) → `$89F2` verify (C/H/R staged at `$7DAC`); IRQ5#1 (data field armed) → `$7BA8`
setup (arms the chunk `C800[0]=[$741e]`, writes `0xf0` *pending* into the ledger); IRQ5#2 (data field
done) → `$8018` done (stamps `$74C4[slot]+2=0x40` READY, stakes the ledger, advances the buffer). The
model delivers **8×IRQ6 + 16×IRQ5**. Dropping the second IRQ5 (an earlier "1 IRQ5 = captured"
assumption) dead-ended every sector at `$7BA8` setup so `$8018` never ran — that was the long-standing
read stall.

**The ledger** at `$7654` (drain window `$7654+[$7954]`, slots 1..8): `0xc0` is the **idle/resting**
value (all-`c0` *before* capture — not "staked"); staking advances `[$7424]` (READY slot) 0..7 and
`[$7428]` (ledger slot) 1..8, **linearly** — the old alternating-slot symptom is cured by the 2-IRQ5
cadence. **The data buffer is linear**: the firmware assembles a **1024-byte buffer `$4000–$43FF`**,
the chunk pointer advancing +128 bytes per sector; the model stages each sector to `$4000 + index·len`.

**Reads are in NATIVE sectors** (128 B FM / 256 B MFM / 1024 B ESDI-HD), *not* 512-byte blocks
(cont.260, media-verified). `mx2-001.imd` cyl0 head0 (FM 16×128B) holds the label — R7 `VOL1SINIX0`,
R8 `HDR1 NSC Boot`; cyl0 head1 is unformatted (`0xe5`). So the count-8 label read is 8 native sectors
on one track — no track boundary, no head switch.

## The transfer (`transfer`) — and the DTACK breakthrough

The firmware commands each sector's DMA through the channel node (`$7442`) descriptor: `C000` = host
dest (advancing `+0x80`), `D000` = local source (advancing `$4000/$4080/…`), `E800` bit12 kickoff
(setup at `$3cd4`/`$3d08`). The kickoff code **RTS's immediately** (`$3d4c`) — no inline poll — **but
the gate array holds the 68000 off the bus for the DMA duration**. Modelling that hold
(`spin_until_time`, ~40 µs) is what makes the chain run: **without it the descriptor drain launches
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

## Open frontier (`open_frontier`)

1. **op42 return-0 / transfer-phase state** — the sole remaining gate. Find what `(A1)` points to
   inside op42 and why `(A1)`-bit7 / `[$7a36]` don't assert after the 8 transfers; and what stimulus
   makes the firmware set `[$7a70]=ffff` / enter `node+$20` bit14 (likely the transfer-channel node
   `$7442` becoming `[$799a]`-current, or a status `$9dd4` reads — it checks `[$71bc]` first byte
   `== 0x8b`). Carry the DTACK discipline in; use a debugger breakpoint on `$6c8e` (op42's done check),
   not opcode taps (they segfaulted / false-negatived here).

## Keeping this current

When the log advances, update `board.yaml` first (it's the source of truth), then this note. The
`superseded:` block in the YAML records readings the log overturned — add to it rather than deleting,
so a corrected claim can't creep back in.
