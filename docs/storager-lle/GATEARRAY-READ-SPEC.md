# Storager Gate-Array Read — Implementation Spec

Derived from the op18→op42→IRQ5/6 disassembly walk (cont.404, 2026-07-24, with Dave).
Commented disasm: `siemens/disasm/storager/storager_v260.asm`. Companion memory:
`storager-op18-gatearray-program`, `storager-op42-completion-reframe`.

## The correct mental model

The read is **not** a free-running stimulus/response. It is a **counted, finite operation**
the firmware *commands* the gate array to perform:

> "Read exactly the N IOCB sectors into these SRAM chunks, then terminate."

The board has the **74LS1801 ENDEC only** (data separation → RXDAT/CLK/SYNC). The **gate array**
(S8526-G + PALs, with the AM2147 4K×1 serial bit-buffer) does deserialize / header-parse / CRC /
DMA. So the gate array is a *programmable engine*, armed once per operation, that runs autonomously
and interrupts at field boundaries.

For an 8-sector FM read the correct signalling is:

    8 × IRQ6 (ID address mark)  +  8 × IRQ5 (data captured)  = 16 interrupts
    (+ possibly one index-crossing interrupt)
    then ONE operation-complete, and STOP.

The current model is wrong in three coupled ways it must fix:
1. **Free-running** — reads whatever passes under the head forever (~16 sectors/rev × ~6.5 revs).
   Must honor the **count** (`[$7956]` = IOCB sector count) and terminate.
2. **3 interrupts/sector** (IRQ6 + data-AM IRQ5 + data-end IRQ5). Must be **2** (IRQ6 + one IRQ5);
   the gate array captures the whole data field after the DAM on its own, so the single IRQ5 IS
   "data captured" and lands on `$8018`.
3. **No operation-complete** — nothing turns "all slots READY" into "op42 returns 0".

## What op18 arms (the program)

`op18 ($308C)` loads two banks from the param block `[$7938]` = node+$76 (built by `op16 $2BD6`
from a class-selected ROM template `$25E/$2EE/$37E/$49E/$52E`; the read uses `$49E`):

- **C800 field-descriptor file** = field **bit-positions** on the track. `op16` accumulates the
  template's field-length list (`$4E2`: 44,6,32,4,…,161) and calls `$3066` per field:
  `cell=(pos>>7)&$1f, value=(pos>>1)&$3f, bit=pos&1`. So C800 says *where* each field (ID AM, ID,
  CRC, gap, data AM, data, CRC) sits, in bits.
- **E000 command block** = 16 words to E000..E01E = the per-field **command program** (template &
  `$fbff`). Meaning of individual command words: still undecoded against the S8526 command set, but
  their *role* is the field-processing sequence.
- **POSPTR** node+$c8..$ce → `$7DAC`+off (C→$7DAD, H→$7DAE, POS→$7DAF); data-field set at node+$78 and
  node+$e4/$e6 → `$7D9D`+off. This tells the firmware where the gate array will stage recovered bytes.

**Slop:** the C800 positions are only where the gate array *starts looking*; the 1801's preamble
search locks the actual AM within a window, so positional drift (other-hardware media) still reads.
The model's pattern-match (`$F57E` IDAM, `$F56F` DAM) already provides this tolerance.

## The engine (per commanded sector, ×N)

Armed with C800 positions + E000 program + count `[$7956]`=N, for each sector the gate array should:

1. **Gap-time** to the ID field's C800 bit-position, then **preamble-search** the ID AM (`$F57E`)
   within a slop window (locks the PLL).
2. **Deserialize** ID (C,H,R,N + CRC) from the bit-buffer; verify ID CRC; **stage** C/H/R/N into the
   `$7DAC` cells (node+$c8..$ce point there).
3. **Raise IRQ6** → firmware `$298C→$89F2` verifies C/H vs `[$7438]/[$7436]` and R vs the aim.
4. **Gap-time** to the data field position, **preamble-search** the data AM (`$F56F`), **deserialize
   128 data bytes** into the `[$741e]` chunk (the DMA), accumulate + verify the data CRC.
5. **Raise IRQ5** → firmware `$29C0→$8018` stamps slot `$74C4[slot]+2=$40` READY, re-programs
   `C800[0]=[$741e]` for the next chunk, re-arms E802 bit11.
6. **Decrement the count.** If sectors remain, advance to the next commanded sector (start+increment)
   and loop; when `[$7956]` hits **0, TERMINATE**: stop capturing and drive the operation-complete.

`[$7950]` alternator (proven): `$8018` re-zeros it each sector so IRQ6 always routes to verify and
IRQ5 always routes to done — one clean IRQ6/IRQ5 pair per sector.

## Termination / operation-complete

The read is **op42 Branch A** (guard timer; node+$12 bit1 clear via template `$49E`). op42 returns 0
only when the drain short-circuits its timer (sets `[$7a36]/[$7a3e]`). The drain runs after all slots
are READY: the firmware launches the SRAM→host transfer, the gate array DMAs it and raises **IRQ4
(channel-done) `$3bfe`**, which drains the READY slots and completes → op42 returns 0 → node phase
`0x0C` = 0x80. **So the model must raise IRQ4 when the (autonomous) transfer completes**, not free-run.

(Branch B commands — seek/status — instead poll F000: done iff `bit5(READY)=1 AND bit8=0`. **F000
bit8/bit9 are the two unmodeled bits**: gate-array op-status, suspected busy/CRC-fault. Not on the
read path, but must be defined for Branch-B commands: model bit8 = "operation busy/fault", set during
the op and cleared on clean completion.)

## Open items to verify at runtime before coding

- **Sector selection**: does the gate array R-match in hardware (delivering only the commanded 8), or
  does it deliver all and the firmware (`$89F2`) discards non-matches? Either way the **count** bound
  is what terminates; a live tap on the per-sector IRQ6/R-value settles it.
- **Exact op-complete signal**: confirm IRQ4 (channel-done) is what satisfies op42's `[$7a36]/[$7a3e]`
  for the read (vs a direct `[$7a3e]` set from the last `$8018`).
- **E000 command-word semantics**: the 16 words' per-field meaning (hunt/compare/capture/CRC/skip)
  against the S8526 command set — needed only if we drive the model *from* the program rather than
  pattern-matching.

## Model-side changes (storager.cpp)

- Replace the continuous `pump`/`deliver_mark` free-run with a **counted capture**: read exactly
  `[$7956]` sectors, one IRQ6 + one IRQ5 each, then stop.
- Drop the second (data-end) IRQ5; the single data IRQ5 carries "captured".
- On count-exhaust, perform the host transfer and raise **IRQ4** as the operation-complete.
- Keep pattern-matched AM detection (provides slop); optionally honor the C800 positions for gap
  timing once the E000 program is decoded.
