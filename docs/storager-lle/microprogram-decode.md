# Storager micro-program — operation decode

**Synthesis doc, as of cont.410 (2026-07-25).** The append-only log (`DESIGN.md`) is the authority;
`board.yaml` is the machine-readable board model. Where this disagrees with the log, the log wins.
Everything below the `## Superseded` line records readings the log overturned — they are kept so a
corrected claim cannot creep back.

## How the micro-sequencer works

Dispatch `$0D54` selects a builder by command (`$92` table, index = cmd−0x70). The builder emits an
**ordered list of op-codes** into the channel node; the walker `$156A` steps the list, dispatching each
op through the jump table at **`$192`** (op N → the **word** at `[$192 + N]`).

The walker (`$15AC`–`$15EC`) is the whole control flow, and the op's **return value in D0** drives it:

| D0 | walker action |
|----|---------------|
| `0` | store at node+`$4`, **advance** the list pointer, continue in the same pass (`$15E6`) |
| `$FE` / `$FD` / `$FF` | leave the pointer put — the op is re-entered on the next pass (**WAIT**) |
| anything else | `$15DA`: store D0 at node+`$18` (**the error**) and set phase `$0C` |

Two ops are special-cased by the walker itself, before dispatch: `0x00` → `$1590` (phase ← `0x0C`,
done) and `0x36` → `$159C` (phase ← `0x0A`, data-phase park).

## The `$192` jump table (dumped from `storager_v260.bin`)

```
00->16c6  02->5e64  04->6208  0c->5f9c  0e->5f74  16->2bd6  18->308c  1a->3182
1c->355c  1e->322e  20->3d4e  22->9018  24->651c  26->6ce4  28->6788  2c->9398
2e->4cc6  30->4fd8  32->4ec2  34->5bec  36->15fe  38->784e  3a->9076  3e->8f74
42->6bc2  44->9602  46->8ac8  48->73fa  4a->6ed2  4c->9ebc  4e->9d6e  50->a08e
52->a2d0  54->a356  56->a392  58->7346  5a->a5f8  5c->50ea  5e->0024
```

## The read's ladder (cmd `0x95`, builder `$5FC0`, param `$8C27`)

`24 28 56 58 [1A] 18 54 4A 42 36 00` — op `1A` is recorded from the cont.348 decode and has **not**
been re-verified since the rebuild; the rest are confirmed by live tracing.

| op | handler | what it actually does | GA traffic |
|----|---------|------------------------|-----------|
| `24` | `$651C` | **unit / drive select + status.** Builds `E804` from the UIB (head ← UIB+`$D4` shifted to bits 8-11 and **one's-complemented**; control byte ← UIB+`$DC`), writes it twice (`$658E`, `$65A0`), pushes the `E802` shadow. Reads `F000` four times and composes the per-unit status byte `[$794e] = ~F000 & table[$22E+unit]`. Aborts with error `$10` (F000 bit0) or `$1B` (F000 bit3). | `F000` ×4, `E802`, **`E804` ×2** |
| `28` | `$6788` | **seek engine.** Drives stepping through `E804` (held in A2) and `E802`. On the cyl-0 label read it short-circuits — the only trace evidence is one `E804=$BF78` write at `$6B20`. | `E804`, `E802` |
| `56` | `$A392` | **channel-descriptor build** at `$7444`: `#$1`, `[$79d8]` (long), **`#$d000`** — the literal that tells the engine which local-address latch to use — then `[$79ce]`. Also parses the IOCB transfer count into `[$7abe]` (`$A42C` 2-byte form, `$A438` 3-byte form, selected by `[$791a]`). | **none** (builds RAM state only) |
| `58` | `$7346` | **count clamp.** `[$7abc] ← [$7abe]`, then bounds it against the track (`$7346`–`$738C`), so the commanded count can be *recomputed*, not taken verbatim. | **none** |
| `18` | `$308C` | **field-program LOAD.** 16 descriptor pushes through the `C800` write ports + the 16-word `E000` step block. See below. | `C800` ×16, `E800` ×32, `E000` ×16 |
| `54` | `$A356` | settle (lea `$66A` param table) | — |
| `4A` | `$6ED2` | arms `[$7b10]=ffff` (`$6ED6`) — the completion-dispatch one-shot | — |
| `42` | `$6BC2` | **the wait.** Returns `$FE` while waiting, so the walker re-enters it every pass (measured ~84 µs apart). DONE=0 needs a status word `(A1)` bit7 + the settle flag `[$7a36]`. | — |
| `36` | `$15FE` | phase-`0x0A` park + the watch-record pump (`$1646`) | — |
| `00` | `$1590` | done — phase `0x0C` (= host status `0x80`) | — |

### op18 in detail (cont.406-408)

Its param block is at `[$7938] = $6E84` — **in RAM, immediately after the UIB at `$6E60`**, so the
field program is *built per-unit from the UIB geometry*, not loaded from a ROM template. Layout: count
word `n=$0F`, 16 triples, then the E000 words at `+$32`.

**The 16 descriptors** are pushed through only three `C800` offsets, the same port written repeatedly —
storage semantics can't represent this, so `C800` is a set of sequencer **write ports**, not a file:

```
off val sel                    (sel = a one-bit selector carried on the UDS/LDS choice
 00  17   1   cell 0 x6         of a dummy read at $6000/$6001 - $6000 is zeroed SRAM and
 00  1c   1                     the data is discarded, so only the access matters)
 00  1d   0
 00  1d   1
 00  3f   0
 00  3f   1
 02  01   1   cell 1 x2
 02  1f   1
 7e  3f   1   cell $3F x8
```

Each push is framed **E800 write-enable (`$2ABD`, bit7) → C800 write → `$6000` strobe → E800 idle
(`$2A5D`)**. E800 **bit7 is the load-enable** and is the model's discriminator between a program push
and an ordinary register write (the per-record chunk arm the ISR issues through cell 0).

**The E000 block** is exactly 16 words, `E000..E01E`, nothing above:

```
0a6d 0829 0033 0a6d | 0a09 x5 | 0809 0829 083b | 023f x4
```

word[0] (`0a6d`) is also stored to `[$7a48]`. The words look like `{flags, byte-count}` steps
(`3f`=63, `29`=41, `09`=9, `3b`=59), the variant paths `ori #$200`/`andi #$ffdf` per word, and `023f`
tails the block as an idle/terminate step. Three variants exist (`$3110` plain / `$3118` / `$3146`,
selected by node+`$20` bit9 and `[$71bc][0]==$99`); the read takes the plain copy.

**op18 is count-invariant** (cont.408, measured): forcing the IOCB count 8→4 leaves the descriptor
stream byte-identical — still 8 pushes to port `$3F` — while records and DMAs drop to 4. The program
describes the **track format**; the count drives only the firmware's own loop. Note 8×128 B FM and the
expected 4×256 B MFM are both exactly the 1024-byte `$4000-$43FF` buffer.

**Load then run**: `0a6d` has bit11 set, so a naive "E000 bit11 = engage" trigger fires on word[0] of
the *load* and delivers records against a half-built program. The program starts on the write that
**completes** the block (`E01E`).

## Ops not in the read ladder, but decoded

| op | handler | what it does |
|----|---------|--------------|
| `3E` | `$8F74` | **the transfer-launch op.** Forks on node+`$20` **bit11**: set → `$8FAE`/`$8FB6` builds the `$7442` staging-channel descriptor and calls the `$3ABC` scheduler; clear → `$8F94`, a local memory fill. `node+$20 = $8C27` for the read (bit11 set, bit9 clear, bit14 clear) — but op`$3E` is **not in the read's ladder** and is dispatched zero times, so the read's transfer comes from the interrupt path instead. |
| `1A` | `$3182` | op18's sibling — the identical push loop but with the dummy-read base at `$4000` (the data buffer) rather than `$6000`. That the two use *different* bases is the main argument that the read is a timing/settling access rather than a decoded strobe. |
| `16` | `$2BD6` | template build |

## The transfer engine the read actually uses (cont.409)

Not a micro-op — a self-clocking IRQ4 chain, with an idle-flag park/kick:

```
launch -> DMA -> IRQ4 -> callback $3F68 -> $3FD0 -> $3FF2 -> $3E3A
       -> queue walk $3EA2 -> node+$20 bit11 -> $3F1E -> $3F36 (bsr $3ABC) -> launch next

park:  $3DE2  bset #0,$7a30        (queue empty -> engine IDLE)
kick:  $833C  bclr #0,$7a30 / beq / bsr $3dbc   (test-and-clear: if it parked, restart)
```

Nothing polls the gate array for transfer completion: the completion signal is **IRQ4** and the "wait"
is the busy lock `[$7a1a]`. The only busy-wait in the path is `$8FF4`, on the op`$3E` path.

**The kick is dead** because `[$7a30]` is written by two different-sized accesses: 68000 bit ops on
memory are always **byte**-sized, so `bset`/`bclr #0` act on the byte at `$7a30` (the **high** byte of
the word), while `$70D4 move.w #$1,$7a30` is a **word** write that zeroes that byte. `$70A0` runs once
per read either way, but takes the `$70D4` branch only when it runs *before* the count drains — which
happens only when the firmware is given real idle time (true rotational record spacing).

## Superseded

- ~~"PIT ctr1 is the DMA transfer counter"~~ → cont.156-158: ctr1 is the seek-settle one-shot; the
  transfer count is `[$7956]`, decremented at `$70A0`.
- ~~"the read never advances past op42/op36 because the data-phase completion event never arrives"~~
  and ~~"the missing step is PUBLISHING the `$1A54` stamp to the host"~~ → cont.353 clean rebuild:
  both were HLE-era artifacts. The read dispatches, arms, captures and transfers.
- ~~"the read stamps 0x82 and never launches its channel"~~ → cont.405-409: it stamps `0x81` and does
  launch; the transfers complete and the payload lands correctly.
- ~~"op18 loads the E000 block from a class-selected ROM template"~~ → cont.406: the param block is in
  RAM at `$6E84`, built per-unit from the UIB.
- ~~"C800 is a 16-entry field-boundary file paired 1:1 with E000"~~ → cont.406: three write ports,
  16 pushes, same port written repeatedly.
- ~~"op18's eight port-`$3F` pushes are the eight commanded sectors"~~ → cont.408: count-invariant.
- ~~"`$70A0` is the per-sector transfer launcher"~~ → cont.409: it fires once per read; the eight
  launches all come through `$3F36`.
