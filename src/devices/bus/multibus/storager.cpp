// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay, Dave Rand

/*
 * Interphase 3030 Storager (Siemens S26361-F415) - MC68000 disk controller for the
 * PC-MX2/MX300, driving 5.25" QD floppy, ESDI hard disk and 1/4" tape.
 *
 *   D8253C-2 x2     programmable interval timers
 *   MC3486P/3487P   quad EIA receiver / driver
 *   VGC7219-0419    custom channel-controller gate array (II-SER--24M)
 *   MCM6164C45 x2   8Kx8 SRAM buffer (0x4000-0x7FFF)
 *   AM2147-55       4Kx1 SRAM
 *   AM27S19APC x2   32x8 bipolar PROM
 *   MC68000P12      50MHz / 4
 *   74LS1801F       FM/MFM ENDEC (data separator only: RXDAT/RXCLK/SYNC, address-mark detect)
 *   74LS1802A       ENDEC companion
 *
 * There is no separate SERDES MSI part on the board: the 74LS1801/1802 recover only bit
 * clock and data (RXDAT/RXCLK/SYNC), and the VGC7219 gate array itself does the deserialise,
 * CRC-16/CCITT and DMA (the AM2147 4Kx1 is its serial bit buffer).  The 68000 talks to exactly
 * one custom device, the VGC7219 gate array, which fronts the ENDEC (not 68000-addressable).
 * This is a low-level model: the firmware runs every command end to end; the model provides
 * only faithful gate-array hardware behaviour (host doorbell/mailbox, the read window and its
 * field-boundary interrupts, the address/parameter latches, the channel DMA and the interrupt
 * levels).  See VGC7219-GATE-ARRAY-SPEC.md.
 */

#include "emu.h"

#include <map>
#include "storager.h"

#include "cpu/m68000/m68000.h"
#include "machine/pit8253.h"
#include "imagedev/floppy.h"
#include "imagedev/harddriv.h"
#include "machine/fdc_pll.h"

#include "formats/imd_dsk.h"

#include <algorithm>
#include <cstring>
#include <iterator>
#include <vector>

#define VERBOSE (0)
#include "logmacro.h"

// The ESDI rigid disks as raw sector images or CHDs.  Derives from the standard hard-disk
// image device, so a flat .img and a .chd both load and are accessed identically.  The
// Storager firmware addresses the medium as a flat byte array; img_read/img_write translate
// byte offsets onto the LBA sector interface (every caller uses 1KB-aligned offsets/lengths).
class storager_hd_image_device : public harddisk_image_device
{
public:
	storager_hd_image_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

	virtual char const *file_extensions() const noexcept override { return "img,chd,hd,hdv,2mg,hdi"; }

	u64 img_length() const { auto const &i = get_info(); return u64(i.cylinders) * i.heads * i.sectors * i.sectorbytes; }
	void img_read(u64 off, void *dst, u32 len);
	void img_write(u64 off, void const *src, u32 len);
};

DECLARE_DEVICE_TYPE(STORAGER_HD_IMAGE, storager_hd_image_device)
DEFINE_DEVICE_TYPE(STORAGER_HD_IMAGE, storager_hd_image_device, "storager_hd", "Storager rigid disk (raw image or CHD)")

storager_hd_image_device::storager_hd_image_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: harddisk_image_device(mconfig, STORAGER_HD_IMAGE, tag, owner, clock)
{
}

void storager_hd_image_device::img_read(u64 off, void *dst, u32 len)
{
	auto const &info = get_info();
	u32 const sb = info.sectorbytes ? info.sectorbytes : 512;
	u8 *const d = static_cast<u8 *>(dst);
	std::vector<u8> sec(sb);
	for (u32 done = 0; done < len; done += sb)
	{
		u32 const chunk = std::min<u32>(sb, len - done);
		if (read(u32((off + done) / sb), sec.data()))
			std::memcpy(d + done, sec.data(), chunk);
		else
			std::memset(d + done, 0, chunk);
	}
}

void storager_hd_image_device::img_write(u64 off, void const *src, u32 len)
{
	auto const &info = get_info();
	u32 const sb = info.sectorbytes ? info.sectorbytes : 512;
	u8 const *const s = static_cast<u8 const *>(src);
	std::vector<u8> sec(sb);
	for (u32 done = 0; done < len; done += sb)
	{
		u32 const chunk = std::min<u32>(sb, len - done);
		if (chunk < sb)
		{
			std::memset(sec.data(), 0, sb);
			read(u32((off + done) / sb), sec.data());
		}
		std::memcpy(sec.data(), s + done, chunk);
		write(u32((off + done) / sb), sec.data());
	}
}

namespace {

// --- Permanent model behaviour (measured; not experiment knobs) -----------------------------
// Physically timed records (one sector period at 300 rpm).  Arm-driven-as-fast-as-rearm was an
// A/B; the timed model is correct.
// One record per address mark; the firmware chooses whether to arm the data phase (no HW sector
// comparator).  The firmware stops re-arming when it has what it wants.
constexpr bool PER_ADDRESS_MARK = true;

// Q3b: the gate array's end-of-transfer marker.
//
// The firmware's record handler forks at $7E8E on [$7968], and the two arms consult the $AA end marker
// DIFFERENTLY:
//   [$7968]==0 -> $7EB2: accept, $7EBE decrements the remaining count, and the marker is tested ONCE,
//                 at $7ED0, one instruction after the count reaches zero.
//   [$7968]!=0 -> $7E90: the marker is tested at $7E9A on EVERY record, with no count precondition.
// They are alternatives, not a sequence: the bootstrap ([$7968] set at $82B2) happens 8.9 ms AFTER the
// remain->0 edge, so nothing can serve $7ED0 - but $7E9A is open from the next record onward.
//
// Trigger: $82B8 bset #0,$7ac2, one instruction after $82B2.  Nothing in the ROM ever reads $7AC2 - it
// is a write-only strobe, i.e. the firmware signalling hardware that it has bootstrapped.
// Address: the FIRST stake's write address (bus-observed) plus the op18 program's own port-$3F push
// count.  Neither is an IOCB or firmware-cell read.
// RESULT (measured): the rule works - $7E9A read $AA at [765f] and took $7ED8, the first time the
// firmware has ever taken a terminate branch.  The +2 offset is right: within a record the firmware
// TESTS S+1 ($7E9A) then STAKES S ($8120/$8128), so the next record's test cell is last stake + 2.
// BUT $7ED8 is not a completion - it only does move.w #$0,$741c and falls to $7F1A.  It does not stop
// the ISR issuing the E802 bit11 capture re-arm, so records keep arriving (116 vs a baseline 16),
// remain runs negative, and a later matched record clears [$7968] again at $7C92.  Left OFF: an
// enabled path that quadruples the record count would poison every other measurement.
// DISPROVED (cont.426).  The marker forces $7ED8, which is the EXCEPTION exit - the same place a
// failed chunk allocation lands ($7EF4 unlk / bra $7ED8).  The NORMAL exit for the count-drain record
// is $7ED6 bne -> $7EE0 -> $32AC allocates -> $7F14 sets [$741c], which $82B2 then consumes on the
// same interrupt to set [$7968].  Measured A/B with the accepted-sector disarm: marker OFF gives
// 8 records / 8 data-done / [$7968]=1; marker ON gives 8 / 8 / [$7968]=0.  The marker's only effect
// is to suppress the bootstrap.  Kept behind the flag as a disproved hypothesis, not a fix.

// TEMP (STRIP): stimulus/response experiment matrix.  Firmware drives; the model only changes how
// the gate array *responds* to programmed arms + disk rotation.  No free-form writes into firmware
// work RAM (UIB/node) except where the GA is known to DMA into a firmware-programmed latch.
//
//   V0 BASELINE     - current behaviour (shims off)
//   V1 COUNTED_STOP - after commanded data-done count, stop delivering marks even if re-armed
//                     (models a finite op18 program: the sequencer terminates)
//   V2 PIT2_OUT     - surface PIT1 ctr2 OUT on F000 bit8 (Branch-B busy line candidate; read is
//                     Branch A so this should be a no-op for op42 - control experiment)
//   V3 STOP+IRQ4    - V1, then one channel-done IRQ4 after the last data field (program-complete
//                     interrupt, no RAM deposit).  Tests whether IRQ4 alone unblocks the drain.

// Q2 hypothesis test (TEMP): $70d4 `move.w #$1,$7a30` zeros the HIGH byte that $6042's
// `bset #0,$7a30` armed for the done-tail kick.  If that clobber is accidental, preserving
// high-byte bit0 through the word write lets $833c actually bsr $3dbc.  false = stock firmware
// behaviour; true = rewrite the $70d4 store to $0101 (low=1 soft flag + high bit0 kick arm).
// This is a temporary CPU-side patch of one store, not a GA SRAM deposit.
// The $4000-$7FFF SRAM's byte order (cont.426).  Three independent sites have the SAME signature -
// the firmware writes a WORD and then tests that value as a BYTE at the identical effective address,
// expecting the word's LOW half:
//   $15A0 move.w d1,($26,a2)=000A  vs  $161C cmpi.b #$a,($26,a1)
//   $70D4 move.w #1,($7a30).w      vs  $833C bclr   #0,($7a30).w
//   $1314 move.w #$81,($2,a0) busy vs  $1A54 move.b #$80,($2,a0) done   (one status field!)
// Under big-endian RAM the byte op reads the HIGH half and gets $00 every time, which is what the
// node+$26 deposit and Q2 exist to paper over.  If byte accesses resolve to the other half, all three
// resolve at once with no patch.  Word accesses are unaffected by construction.
constexpr bool SRAM_BYTE_SWAPPED = true;

// LABEL_BODY_SKIP - EXPERIMENT, NOT JUSTIFIED BY ANY DECODED FIELD.  Do not ship this.
//
// Both CPUAP monitor revisions overlay, on the buffer they hand this controller, a structure whose
// byte 0 is the ANSI/ECMA-13 VOLUME IDENTIFIER (label record byte 4), with the geometry patch at
// +0x30 and a 0x20-dword partition map at +0x44.  Attested independently in rev3 and rev9:
//     signature  'S'@+0 'I'@+1 'X'@+4      (rev3 base+0x70, rev9 base+0x69)
//     geometry   +0x30, stride 0x14, 5 dwords
//     partition  +0x44, 0x20 dwords
//     and ADJSPB is the ONLY instruction between the read returning and the gate, in BOTH.
// So the monitor expects the label record MINUS its 4-byte record identifier ("VOL"+"1").  Nothing
// in the CDB, IOPB or UIB carries that offset, no code between the read and the gate applies it, and
// the sibling Interphase manuals (2180, SMD2190, 4201) describe no labelled-volume read at all.  The
// remover is UNIDENTIFIED.  This flag tests the STRUCTURE, not the gate.
//
// ACCEPTANCE (structural, not "the gate passes"): the parse path must EXECUTE - $fe3696, $fe36a3,
// $fe36ae (partition map) and $fe36d4/$fe36da (geometry patch) all currently have ZERO hits.  If the
// structural account is right they run and the boot advances; if it is wrong they run and the boot
// fails somewhere new, which refutes the account.  Either way it is information.
// ✖ REFUTED cont.450 - left FALSE deliberately.  It DID make the gate pass (fe3696/fe36a3/fe36ae
// all executed for the first time), but the acceptance test was too weak: it asked "does the parse
// path run", not "is what the parse produces sane".  Downstream, the monitor's partition-map copy
// (fe36ae, 0x20 dwords from EXT(0x9)+0x44) then read ANSI reserved bytes as binary geometry and
// requested cylinder 0x3120 / sector 0x20 - ASCII "1   " and a space - which the storager firmware
// correctly rejected in 2 ms with sense 0x15.
//
// And no four-byte variant can fix that: EXT(0x9) is MEASURED to be the read buffer itself
// (r0=0x000FC0DD at fe36a7), so +0x44 is measured from the POINTER, which does not move when the
// record does.  Both source-skip and destination-shift land on record[0x48] - the ECMA-13
// reserved/label-version region ("   1"), ASCII by the standard, not by accident.
//
// So the gate (wants the volume identifier at buffer[0]) and the parse (wants binary geometry at
// buffer+0x44) are mutually unsatisfiable if the buffer receives the raw VOL1 record.  Either the
// buffer is meant to receive a structure assembled elsewhere, or this medium lacks what the monitor
// wants - note it also reports "sasiopen: no label SINIX found" on the hard-disk path.

// SECTOR_ID_REMAP - does the controller renumber sector IDs at all?
//
// FALSE = identity: physical sector IDs are presented as-is, so a 1..8 run lands at buffer slots
// 0..7 and physical sector 7 (the VOL1 record) lands at buffer+0x300.
//
// That is where the ROM actually looks.  The sys-floppy gate has TWO tests, and only the first wants
// a descriptor at buffer byte 0:
//     fe448e: CMPB 0x53, 0x69(R6)          ; 'S' at buffer+0      (Siemens descriptor)
//     fe4493: BEQ  fe449D                  ;   pass -> READ #2
//     fe4495: CMPB 0x53, 0x4(-0x94(FP))    ; 'S' at buffer+0x300+4 (ECMA-13 VOL1 volume identifier)
//     fe449b: BNE  fe44B9                  ;   fail -> skip READ #2
// with the pointers set at fe421d/fe4223: -0x94(FP) = R6+0x369 = buffer+0x300, and
// -0x98(FP) = R6+0x3E9 = buffer+0x380 (the HDR1 record, whose +0x20 READ #2 uses).
//
// So the ROM reads ANSI label sets NATIVELY via the second test: VOL1 expected at physical sector 7,
// HDR1 at physical sector 8, straight through with NO remapping.  A base/sec0 remap that moves VOL1
// to buffer byte 0 defeats BOTH tests at once - byte 0 is not a descriptor, and buffer+0x304 no
// longer holds the volume identifier.  The campaign's original 0x300 "displacement" was never an
// error; it was the ROM's expected layout.

// cont.516: deposit each data field at the chunk the firmware's own claim table ($7696) names for
// that sector, rather than at the chunk C800[0] happens to hold.  Only the FIRST wanted record of a
// run differs - it arrives before its own arm, so C800[0] still holds the previous command's chunk.
// See the block comment at the deposit site for the measurement that motivates it.
// REFUTED cont.516, left false with its evidence: the claim for a record is written ~7us AFTER
// that record's deposit, so the first wanted record has no claim to look up when it lands.  Enabling
// this found only STALE entries from the previous command and redirected r=02/03/04 - which were
// already correct - while never firing for r=01, the broken one.  Measured: STEP2 unchanged at
// A0 A0 A0 A0, console unchanged.  A deposit-time lookup cannot work; the record has to be HELD
// until its claim exists, which is the hold mechanism with a corrected trigger.

// cont.517: hold a data field until the firmware has armed C800[0] for THIS command, not merely
// until C800[0] is in range.  Between commands C800[0] retains the previous command's last chunk,
// so the first wanted record of every run after the first is deposited into a chunk the host DMA
// never reads.  Latched at command start (m_c800_cmd_start); the held field flushes on the next arm.

// cont.518 (Dave): the medium is continuous - a sector arriving before its arm returns next
// revolution, so it never needs rescuing.  Deposit only into a freshly-armed chunk.

// cont.519: flush a held field ON THE CLAIM WRITE.  Measured ordering for cyl1 R=1 (the boot
// header): record t=8.46075, claim written ~8.46082, host DMA reads that chunk t=8.46098.  The
// claim is the only moment the destination exists, and the next C800[0] arm fires BEFORE it
// (8.46077), which is why an arm-triggered flush read a stale entry and resolved R=1 to 4000.

// cont.521 (Dave's argument): the ISR is MEASURED not to compare sector numbers - only head ($7436)
// and cylinder ($7438).  Something must reject non-matching sectors, so it is the gate array, and
// GATEARRAY-READ-SPEC lists "compare" among the E000 command-word operations while leaving sector
// selection open.  The field program op18 pushes carries the physical start sector: measured 0001
// for reads at cyl0 R=1 AND for the LBA-32 read (which the firmware resolves to cyl1 R=1), and 0002
// for the probe on a different unit - density-independent, tracking the sector rather than the block.
// Delivering every record instead drives the firmware's arm pipeline with sectors nobody asked for,
// which is why the first WANTED record arrives with the arm still on the previous command's chunk.
// cont.525: multi-track continuation.  IMPLEMENTED AND MECHANICALLY CORRECT - it walks head 0 -> 1
// at the track boundary (measured: one WALK event at t=8.83280, head=1 cyl=1, 42 blocks left).
// But it does NOT fix the blocker and it changes the completion sense 29 -> 1C (short transfer),
// so it is OFF.  Reason: the failure is at BLOCK 41, which is on the FIRST track (head 0 covers
// blocks 36-47) - it happens BEFORE the boundary is reached, so continuation fires after the fact.
// Do not enable until the first-track failure is understood; it cannot be a multi-track problem.

// cont.494: the chunk-path instruments (claim gate, ownership map, stride table, arm source,
// ID compare, firmware status/decide).  Each answered a specific question this campaign - the
// $7696 table decode, the track-relative sector-ID fix, and the first-arm comparison - and each
// --- Permanent OS-channel / write-path behaviour ------------------------------------------
// Only IDs inside the commanded range produce marks.  Firmware counts marks, not matches;
// post-mark skips stole sector counts while posting success.  Fixed.

// E802 bit0 is a SHARED stimulus: it is the ESDI serial clock AND the floppy STEP line, and the
// SELECTED drive is what responds.  DECODED (cont.565, board.yaml conf V): the E804 low byte is the
// selected unit's UIB+$DC verbatim, bits 5-7 are the drive select, bit4 is motor/write-current, and
// the floppy is select 110 ($c0 base) - established independently because those writes coincide 1:1
// with head steps.  Without this gate the model armed the serial branch on EVERY bit0 edge, so a
// floppy step was mis-latched as a serial ack and stole F000 bit1 from the restore-completion test
// at $6E70 (measured: 61 latches in a boot, and $A118/$A1CE/$A17E executed ZERO times - there was no
// serial transaction at all).  This is the SAFE half of the demux: it only asserts "floppy selected
// -> never serial", and makes no claim that a non-floppy select implies a real serial transfer.
// Deliberately NOT gated on the CPU being inside $A118: no board signal carries a program counter,
// so that would be an HLE shim that fails silently on any other path driving the same line.

// F000 bit1 on the FLOPPY path = "head position not confirmed at track 0", not the settle
// one-shot's OUT.  MEASURED (cont.563-4): the firmware's restore gate at $6D94 tests bit1 and
// treats CLEAR as "already home", zeroing the position cache with no step pulses; bit1 read 0 in
// 104/104 polls, so every post-error seek then ran open-loop from a false zero and $2012 reported
// the wrong track accurately.  A mode-5 one-shot cannot supply that level: OUT is high both idle
// and while counting, dipping low for one CLK at TC, so neither polarity separates "home" from
// "not home".  The one correct sample in the whole run is the tell - f000=$00A0, bit13 CLEAR (at
// track 0) AND bit1 clear - while all 103 failing polls had bit13 SET.  Both firmware consumers
// agree with a position-confidence latch: $6D94 SET -> recalibrate, and the $6E2C pulse loop
// ($6E58 andi #$2) keeps stepping while SET, which terminates exactly when the head reaches
// track 0.  The ESDI handshake use of bit1 is a separate branch and is untouched.
// REQUEST SENSE (0x03): latch fw sense at HOST POST 0x82, serve 4-byte class-0, clean mailbox
// 0x80 so the driver does not inherit the failed command's status.  Classifies $1c/$12; does
// not cause REZERO (measured).  Diagnostic permanent.  Sense bytes 1-3 not fully validated.
// Host 0x0b SEEK -> fw 0x8A (idle-latch wedge / $14E2).
// ESDI serial engine + F000 bit1 frame gate (cont.572).  Measured on the post-restore boot:
//   - rigid unit runs ZERO step pulses; restore completes via [$794c]==0 ($6E6A) with bit1 CLEAR
//   - $A118 then runs (D0=$3000 Request Configuration); $201E is the mid-frame bit1 wait
//   - STEP-TRAIN probes 0; so rigid E802 bit0 edges are serial clocks, not steps (floppy demuxed)
// Frame = 17 beats starting on clock-low to a rigid unit, ending after the rising edge of beat 17.
//   rigid bit1: frame_active ? follows_clock : CLEAR
// That is idle-clear for restore and transfer-ack for $A118/$A1CE.  HD_SETTLE_S retired.
constexpr bool ESDI_SERIAL_ENGINE = true;
// Force-arm the prepared write mark instead of waiting for the firmware's E802 bit11 re-arm.
// LOCATE is the site where the deadlock was MEASURED (WRLOC -> WRDATA arm gap of 1.81s, WRGAP
// pend=6 armed=0, zero E802 writes in the window).  DATA covers the two data-phase IRQ5s, which
// were extrapolated from it, not measured.  Split so the A/B can tell whether the narrow fix is
// enough - forcing marks the firmware did not request is exactly the class of error that the v3
// close-path IRQ4 turned out to be.
// c0 sub-7 status fill (interim C++ until host-op->fw map).  See docs/FIRMWARE-DISPATCH-TABLE.
constexpr bool OS_C0_STATUS_FILL = true;

// --- Experiments (default OFF) ------------------------------------------------------------
// Flux encoder self-test (writes a sector).  Never against archival media.
constexpr bool WRSEC_SELFTEST = false;
// Extra trailing record (refuted for latch clear).

// --- Trace (default OFF except install-oriented ESDI) -------------------------------------
// Per-record probes STRIP before upstream; dma_snoop is functional not TRACE_*.
constexpr bool TRACE_CHUNK_PATH = false;
constexpr bool TRACE_ESDI       = true;   // clamp / rigid-disk ops
constexpr bool TRACE_GAWRITE    = false;
constexpr bool TRACE_ARM        = false;
constexpr bool TRACE_HOSTCMD    = false;
constexpr bool TRACE_SEEKCMP    = false;

// Q3 hypothesis test (TEMP): end-marker after a ledger stake, using only the write address.
// When firmware stakes $c0 ($8120), if the NEXT ledger byte is NOT still a want ($ff), deposit
// $AA there.  That skips mid-window (next=$ff → leave wants visible) and fires when the stake
// lands against $fe/$aa/end - i.e. the filled prefix has no remaining $ff beyond it.  No IOCB
// count, no [$7956] snoop.  v1 (unconditional next=$AA) interleaved $AA in front of $ff and
// cut the read to 4 DMAs / remain stuck at 4 - so the $ff guard is required.  false = off.

class multibus_storager_device
	: public device_t
	, public device_multibus_interface
{
public:
	multibus_storager_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
		: device_t(mconfig, MULTIBUS_STORAGER, tag, owner, clock)
		, device_multibus_interface(mconfig, *this)
		, m_cpu(*this, "cpu")
		, m_pit(*this, "pit%u", 0U)
		, m_floppy(*this, "floppy%u", 0U)
		, m_hd(*this, "hd%u", 0U)
	{
	}

protected:
	// device_t overrides
	virtual const tiny_rom_entry *device_rom_region() const override;
	virtual void device_add_mconfig(machine_config &config) override;
	virtual ioport_constructor device_input_ports() const override;
	virtual void device_start() override;
	virtual void device_reset() override;

private:
	void mem_map(address_map &map);
	void opcodes_map(address_map &map);
	static void floppy_formats(format_registration &fr);

	// host / Multibus interface (spec §5): the PIO window 0x7200-0x73FF maps byte-for-byte to the
	// on-board dual-port RAM at 0x7E00-0x7FFF (+0xC00).  The CPUAP writes the command/IOPB-pointer
	// mailbox there; writing GO (0x13) to the command register raises 68000 IRQ2 (the doorbell).
	u16 host_win_r(offs_t offset);
	void host_win_w(offs_t offset, u16 data, u16 mem_mask);
	void go_submit(u32 dst, u32 dbi);   // shared doorbell GO tail (both command channels)
	// Which firmware commands may arm the gate array's field program.  Reads always; write-class
	// only under the ARM_WRITE_CLASS A/B (see the flag for what that does and does not prove).
	bool cmd_is_read()  const { return m_iopb_cmd == 0x94 || m_iopb_cmd == 0x95; }
	bool cmd_is_write() const { return m_iopb_cmd == 0x96 || m_iopb_cmd == 0x82; }
	bool cmd_armable()  const { return cmd_is_read() || cmd_is_write(); }
	void mailbox_submit(u32 host_iopb, u32 dst);   // deliver via the mailbox, as the hardware does
	u16 bus_data_r(offs_t offset);
	u16 bus_mem_r(offs_t offset, u16 mem_mask);
	void bus_mem_w(offs_t offset, u16 data, u16 mem_mask);

	// gate-array registers (spec §3): E000-E01F channel/ENDEC window, E800-E807 control, F000 status,
	// C000 host-address counter, C800 field-boundary file, D000/D800 local latches.
	u16 ch_r(offs_t offset, u16 mem_mask);
	void ch_w(offs_t offset, u16 data, u16 mem_mask);
	void c000_w(offs_t offset, u16 data, u16 mem_mask);
	u16 c800_r(offs_t offset);
	void c800_w(offs_t offset, u16 data, u16 mem_mask);
	void d000_w(offs_t offset, u16 data, u16 mem_mask);
	void d800_w(offs_t offset, u16 data, u16 mem_mask);

	// PIT outputs
	void timer0_out(int state);   // PIT1 ctr0 = system tick -> IRQ1
	void esdi_serial_beat(u16 e802);   // one beat of the 17-bit ESDI serial frame
	void esdi_command(u16 cmd);        // a complete command word arrived from the controller
	void timer2_out(int state);

	void spin_drives();
	void floppy_swap_cb(floppy_image_device *fdd);

	// The read window: the 74LS1801/1802 ENDEC recovers the FM/MFM cell stream from the drive's real flux
	// (get_next_transition) and the gate array deserialises it, doing address-mark detect and filling the
	// gate array's capture cells + raising IRQ5/IRQ6 per field boundary.
	bool flux_density_fm() const;
	u8   logical_r(u8 phys) const;
	u16  ioreg_r(offs_t offset);
	void ioreg_w(offs_t offset, u16 data, u16 mem_mask);   // physical sector ID -> FIRMWARE sector index (1-based, base UIB[4])
	attotime sector_period() const; // one sector's rotation at 300 rpm
	void start_field_program();     // the loaded op18 program begins running: open the read window
	void advance_read();            // stage + deliver the next record while the window is armed (IRQ6 then IRQ5)
	void deliver_mark();            // arm + held-mark -> one interrupt
	// Deferred completion interrupt for the OS-driver channel (HLE parity).  A real controller
	// takes milliseconds; firing instantly would beat the driver into its wait.
	TIMER_CALLBACK_MEMBER(ioreg_int) { if (m_ioreg_pending > 0) int_w<2>(0); }   // Multibus INT is ACTIVE LOW
	emu_timer *m_ioreg_int_timer = nullptr;
	int  m_ioreg_pending = 0;       // outstanding interrupt-driven completions
	TIMER_CALLBACK_MEMBER(pump_tick);

	// channel DMA: on the E800 bit12 kickoff the gate array bus-masters one field between the local
	// SRAM buffer and host memory via the C000 up-counter, then raises IRQ4 (channel/DMA done).
	void run_channel_dma();
	attotime dma_time(u32 len) const;  // bus-hold / transfer time for a len-byte bus-master DMA
	TIMER_CALLBACK_MEMBER(dma_done);   // transfer end: release the bus hold, raise the channel-done IRQ4
	// Write close-path deferred failsafe (v5): re-check node+2 after firmware's self-complete window.
	TIMER_CALLBACK_MEMBER(wr_close_check);
	void arm_wr_close_failsafe(char const *why);  // schedule re-check; never raises IRQ4 immediately

	required_device<m68000_device> m_cpu;
	required_device_array<pit8253_device, 2> m_pit;
	required_device_array<floppy_connector, 2> m_floppy;
	required_device_array<storager_hd_image_device, 2> m_hd;   // ESDI unit 0 (-hard1) + unit 1 (-hard2)

	bool m_installed = false;
	u16 m_ch[0x1000] = {};       // 0xE000-0xFFFF register backing + E800/E802/E804 shadows

	// gate-array address / parameter latches (spec §3.4)
	u32 m_c000 = 0;              // C000-C7FF: 24-bit Multibus host-address up-counter (stored decomplemented)
	bool m_c000_valid = false;
	u16 m_c800[0x100] = {};      // C800-C9FF: field-boundary offset file (cell 0 = live SRAM chunk word ptr)
	u16 m_d000 = 0;             // D000: local SRAM word-address latch (IOPB work area)
	u16 m_d800 = 0;             // D800: parameter/status template word-address latch

	// DMA + status sampled into F000
	bool m_dma_active = false;   // E800 bit6
	bool m_bus_held = false;     // the gate array holds the 68000 off the local bus for a DMA's duration

	// The loaded field program (op18).  op18 pushes a descriptor stream through the C800 write ports
	// while E800's load-enable is asserted, then block-writes the 16-word E000 step program.  NOTHING
	// RUNS UNTIL THE LOAD COMPLETES: the firmware masks interrupts across the push phase and copies the
	// E000 block in one straight-line loop, so a record delivered mid-load would be captured against a
	// half-built program (which is what dropped the run's first sector).  The sequencer starts on the
	// write that completes the block (E01E), which is also what makes the first record possible at all -
	// the per-record re-engagement writes are issued from the record ISR, so they cannot start the run.
	u16 m_prog[16] = {};         // E000..E01E, the field step program as op18 loads it
	u8  m_desc_port[32] = {};    // the C800 descriptor stack, in push order (a port takes many pushes)
	u16 m_desc_val[32] = {};
	u8  m_desc_n = 0;
	bool m_prog_loaded = false;  // the gate array holds a field program (PERSISTS across commands)
	bool m_prog_loading = false; // an op18 block copy is in progress - word[0] is data, not a command
	u16  m_dma_term = 0;         // transfer terminal, latched from a C800 write while DMA active
	u8   m_term_bit0 = 0;
	u32  m_last_bw = 0;          // last local-bus write address (transfer-pointer snoop -> F000 bit12)
	u32  m_node_base = 0;         // node local address, latched from D000 on a node-class block
	u32  m_uib_base = 0;         // UIB local address, latched from D000 on a UIB-class control block
	int  m_data_done_n = 0;      // data fields delivered this command (arms the completion deposit)
	int  m_accepted_n = 0;        // sectors the firmware has ACCEPTED (ledger $c0 stakes observed)
	int  m_prog_count = 0;        // Q3b: sectors the op18 program commands (its port-$3F push count)
	u32  m_aa_cell = 0;           // Q3b: ledger cell for the end marker (first stake + count)
	bool m_aa_armed = false;      // Q3b: bootstrap strobe seen, deposit owed
	bool m_aa_done = false;       // Q3b: deposited this command
	u32  m_aa_last = 0;           // Q3b: last cell written, so each is deposited once
	int  m_aa_n = 0;              // Q3b: log cap
	bool m_status_armed = false; // all commanded sectors captured - completion byte owed at node+$26
	bool m_timer_out = false;    // PIT1 ctr0 OUT -> F000 bit11
	// ESDI serial command/status interface (spec: $A118 send / $A1CE receive).  E802 bit0 = clock,
	// E802 bit1 = data out (active low), F000 bit1 = the drive's TRANSFER-ACKNOWLEDGE, F000 bit4 = data
	// in.  The drive follows the controller's clock: ack drops when the controller drops the clock and
	// rises when it raises it, so the firmware's two-phase poll ($A17E waits ack low, $A1A2 waits ack
	// high) completes without any timing model - it polls, so latency is absorbed.
	// WRONG AS WRITTEN (cont.565): m_ser_active does NOT mean "a serial transaction has clocked".
	// E802 bit0 is shared with the floppy STEP line and this model arms the serial branch on every
	// edge, so a floppy seek hands F000 bit1 to the ack and destroys the position-confidence meaning
	// the restore completion at $6E70 depends on.  The real discriminator is E804 drive select (who
	// responds to the shared line), and that encoding is not yet decoded - see board.yaml.  Needs one
	// ESDI $A118 window sampled together with sel_drive before this can be gated correctly.
	bool m_ser_clk = true;       // last E802 bit0 seen
	bool m_ser_active = false;   // a serial transaction has driven the clock this command
	// ESDI SERIAL ENGINE (cont.570).  Decoded from the firmware's own halves, not from a textbook:
	//   $A118 send    D2=$10 + dbra = 17 iterations; 1-16 shift D0 out MSB first, the 17th shifts
	//                 D1 (a one-counter seeded at 1), so the frame always carries ODD parity.
	//                 A data 1 CLEARS E802 bit1 ($a154) and a 0 SETS it - the line is INVERTED.
	//   $A1CE recv    same 17 beats; samples F000 bit4 with `btst #4` and treats CLEAR as a 1
	//                 (also inverted), then `lsr.w #1,D5 / bcs` REQUIRES odd parity or returns $201e.
	// The drive is the responder: it acknowledges every beat on F000 bit1 (already modelled - that
	// is why send-only commands complete) and drives F000 bit4 for the two commands that read back.
	u16  m_esdi_shift = 0;       // command word being clocked IN from E802 bit1
	u16  m_esdi_resp = 0;        // response word being clocked OUT on F000 bit4
	int  m_esdi_beat = 0;        // 0..16 within the 17-beat frame (shift-register index)
	int  m_esdi_frame_falls = 0; // falling edges seen in the current frame (1..17)
	bool m_esdi_frame_active = false; // true while bit1 must follow the serial clock
	bool m_esdi_frame_closing = false; // rises of beat 17 seen; end on next non-rise e802 write
	bool m_esdi_resp_phase = false;  // false = receiving a command, true = returning a response
	bool m_esdi_out_bit = false;     // the bit currently presented on F000 bit4
	u16  m_esdi_last_cmd = 0;
	u16  m_esdi_cyl = 0;         // drive position as the serial interface last commanded it
	u8   m_esdi_head = 0;
	bool m_stepped_since_arm = false;  // head moved after the command armed -> re-decode on settle
	bool m_settle_out = true;    // PIT0 ctr1 mode-5 one-shot OUT; F000 bit1 seek/settle busy = !OUT
	bool m_pit2_out = false;     // PIT1 ctr2 OUT (recorded, not surfaced on F000)
	u32 m_gaw_f000_n = 0;   // TRACE_GAWRITE (STRIP): F000 polls during a write-class command
	bool m_r0_busy = false;      // R0 status bit1: latched at host GO, cleared at the firmware's DONE stamp
	bool m_r0_doneint = false;   // R0 status bit2: OPER-DONE-INT, set at DONE, cleared by CLR-INT
	bool m_gate0 = false;        // E800 bit9 -> PIT1 ctr0 gate
	bool m_e800_bit12_prev = false;   // bit12 kickoff edge detect
	bool m_host_int_prev = false;     // E802 bit7 host-completion interrupt level
	bool m_floppy_loaded = false;

	// E804 drive/head select, latched in the gate array (spec: op24 $6576-$65A0 builds it from the UIB -
	// head from UIB+$D4 shifted into bits 8-11 and COMPLEMENTED, drive/control byte from UIB+$DC).  The
	// gate array holds this until the firmware changes it; the read path uses the latched head to pick
	// the side, rather than reading the firmware's own copy out of its work area.
	u8 m_sel_head = 0;           // 0-15, decoded from E804 bits 8-11
	// cont.536: the OS DRIVER's command channel, Multibus I/O 0x0800-0x0807.  Distinct from the boot
	// monitor's doorbell+IOCB at 0x7200-0x73ff: the monitor loads the kernel through that, then the
	// loaded SINIX driver talks HERE.  Dropped in the LLE rebuild, which is why the kernel's writes
	// (measured: unmapped bus I/O 0x0804/0x0806 from pc 008928) vanished and its poll read garbage -
	// the recorded "post-Boot: stall".  Register file: cmd byte at [0] written LAST, bit7 read back
	// = controller DONE.  Pointer lanes per the kernel's sacmd @0x9504:
	//     dev-blk = regs [4],[5],[7]   req-blk = regs [2],[3],[6]     (reg[6] is NOT dev-blk's byte 3)
	u8   m_ioreg[8] = {};
	u32  m_verify_n = 0, m_verify_bad = 0, m_verify_prints = 0;   // end-to-end delivery check
	u32  m_step_writes = 0, m_step_edges = 0, m_step_moves = 0, m_step_prints = 0;   // TEMP step census
	u8   m_last_sense = 0;          // firmware sense byte from the last 0x82 completion (node+3)
	u8   m_last_sense_op = 0;       // firmware opcode that failed, for the sense reply
	bool m_last_sense_valid = false;
	double m_ser_last_t = 0.0;      // TEMP cont.565: time of the last serial clock edge
	u32  m_mux_prints = 0;          // TEMP cont.565: bit1-ownership probe cap
	bool m_step_bit0_prev = false, m_step_lvl_prev = false;
	bool m_ioreg_done = false;
	// OS-channel command submitted to the firmware via mailbox_submit, awaiting HOST POST.
	// Until that posts, reg[0] bit7 must stay clear: the HLE set DONE in the same call because
	// its transfer was synchronous; the LLE path is asynchronous and must not.
	bool m_os_fw_pending = false;
	u32  m_id_cmp_gen = 0;          // bumps per OS/boot read so ID-CMP logs reset per command
	int  m_id_cmp_n = 0;
	u8 m_sel_drive = 0;          // E804 low byte: drive select + motor/write-current/precomp controls

	// cont.525: MULTI-TRACK CONTINUATION.  The IOCB is LINEAR-addressed (`cyl=0 head=0 start=36
	// count=53`), so the CHS walk is the CONTROLLER's job, and measurement says the firmware does not
	// do it: at a track boundary it re-arms (PROG n=0) and waits, never updating position (the
	// $6ab8 block runs 4x per run, never at a boundary), never changing head, never seeking.  So the
	// gate array walks head-then-cylinder itself.  These track the walk independently of the
	// firmware-selected head, which stays put across the whole command.
	bool m_walk_pending = false;   // track-boundary head switch owed, applied at the next record
	u8  m_walk_head = 0;         // head the GA is currently reading during a linear walk
	int m_blocks_left = 0;       // blocks still owed on this command (from the NODE count, not [$7abc])
	// Linear-addressed commands name a cylinder via the IOPB address.  Delivery must not start
	// until the drive is there: measured psec=240 opened ID-CMP on C=5 (previous track) with
	// want still 5, then sought to 7 with [$7438] lagging, and raised 0x2012.  0xffff = no gate
	// (sequential boot reuses retention and is already on-cylinder).
	u16 m_cmd_cyl = 0xffff;
	// cont.524: STORED AND NEVER READ - and it is the byte the firmware changes at a TRACK BOUNDARY.
	// Measured on the multi-track kernel read (blocks 36-88, four tracks): steady state E804 = FFC8 /
	// FF88 / FF0B / FF0C; at the boundary, immediately after the track's last record and alongside the
	// PROG re-arm, the firmware writes **FFD8** - low-byte bit 4.  Head bits (8-11, complemented) are
	// UNCHANGED, so this is not a head change.  The model latches this byte and acts on none of it.
	// DO NOT implement model-side auto-advance to the next track: the firmware is signalling here and
	// the model is discarding the signal, so auto-advance would paper over a dropped register.
	// NEXT (static, free): op24's E804 builder at $6576-$65A0 composes this from UIB+$DC - read what
	// bit 4 of that field means before honouring it.

	// host doorbell / IOPB
	u32 m_tbl_wr_n = 0, m_tbl_clr_n = 0, m_tbl_early_n = 0;
	u32 m_cg_n = 0; u16 m_cg_last = 0xffff;
	u32 m_idc_n = 0, m_idc_cmp = 0;
	u32 m_own_n = 0, m_own_idx = 0;
	u32 m_as_n = 0;
	u32 m_ctf_n = 0, m_ctf_early = 0; bool m_ctf_ctl = false;
	u32 m_f000_n = 0; double m_f000_last_t = 0;
	std::map<u32, u32> m_f000_hist;
	u32 m_fk_n = 0;
	std::map<u32, u32> m_v138_hist; double m_v138_last = 0;
	u32 m_sweep_n = 0, m_n26_all = 0; double m_n26_ctl = 0;
	u32 m_ds_n = 0; double m_ds_last = 0; std::map<u32, u32> m_ds_hist;
	bool m_tbl_ctl_done = false;          // uncapped count of stride-table writes (cont.491)
	u8  m_last_r = 0, m_last_presented = 0;   // identity of the record being decided (cont.460)
	u32 m_data_chunks = 0;       // data chunks delivered in the current command (LABEL_BODY_SKIP)
	u8  m_iopb_cmd = 0;          // command byte from the auto-fetched IOPB (drives the model's channel)
	std::unique_ptr<u16[]> m_lram;   // SRAM_BYTE_SWAPPED backing store
	u16 lram_r(offs_t offset, u16 mem_mask);
	void lram_w(offs_t offset, u16 data, u16 mem_mask);
	void flush_held_on_claim();
	u32 m_iopb_addr = 0;         // host IOPB base (the auto-fetch source)
	u8  m_mb_in[8] = {};         // inbound host register-file latch (pio 73F4-73FB)

	bool m_read_window = false;     // E000 armed for a read (E802 bit15 / E000 bit11 engagement)

	// arm/latch handshake (the gate array's record-status latch the firmware clears by reading E01E):
	// the disk spins continuously (m_read_window gates the pump), but the gate array only latches a
	// record - and interrupts the firmware - while capture is ARMED (E802 bit15) and the previous
	// record has been ACKed (E01E read cleared the latch).  One record per arm/ack cycle.  When the
	// firmware stops re-arming, records stop and it converges to completion.
	// the firmware's active gate-array command (E000 code): what record the gate array is to deliver next
	enum { CMD_IDLE = 0, CMD_IDHUNT, CMD_DATA };
	int m_cmd = CMD_IDLE;

	// event-driven interrupt delivery (cont.382): the read is an interleaved IRQ4/5/6 handshake and only
	// ONE interrupt is live at a time.  Each interrupt is gated by a firmware ARM (an E802/E000 bit-change)
	// PRIOR to it: the flux recovers a record and HOLDS at the mark (m_mark_pending = the IRQ level); the
	// firmware's arm bit-change then delivers exactly that one interrupt.  arm + mark -> one IRQ; the flux
	// does not advance to the next mark until this one is delivered and serviced.
	int m_mark_pending = 0;         // 0 = none, 5 = a data record, 6 = an ID record held awaiting the arm

	bool m_armed = false;           // capture-arm, primed on the E802 bit15 rising edge, consumed at the
	                                // record end (a persistent one-shot: the CPU arms once per record and
	                                // the mark is delivered whenever the pump next reaches it - the 200us
	                                // pump cannot sample an instantaneous bit15 the firmware pulses in us).
	bool m_e000b11_prev = false;    // E000 bit11 (read-window open) level, for rising-edge detect
	bool m_bit11_prev = false;      // E802 bit11 (per-record re-arm) level, for rising-edge detect
	// cont.394 (pass 2): window-close terminator.  m_window_seen set once the last wanted ledger position
	// is captured; at the next index crossing the gate array presents the $fe terminator + arms the walk.
	bool m_idx_prev = false;        // floppy index level, for rising-edge detect
	bool m_window_seen = false;     // the wanted sector window has been fully captured
	bool m_term_fired = false;      // the window-close terminator has been presented (one-shot per read)
	bool m_rec_latch = false;       // a data record is latched awaiting the firmware's E01E ack

	emu_timer *m_pump = nullptr;    // gate-array field-boundary mark clock
	emu_timer *m_dma_done = nullptr;   // async channel-done IRQ4 (real DMA only)
	emu_timer *m_wr_close = nullptr;  // deferred write-close failsafe re-check

	// The gate array captures the whole track as it rotates (detection-is-capture) into m_track, then the
	// the engine (advance_read) delivers a record per address mark, one IRQ6 + one IRQ5 each, for as long
	// as the firmware keeps re-arming E802 bit11 - it does NOT count down a commanded run.
	struct captured_sector { u8 c = 0, h = 0, r = 0, nn = 0, dam = 0; u16 len = 0; u8 data[1200] = {}; };
	captured_sector m_track[32];
	int  m_track_n = 0;             // sectors recovered in the last full-track capture
	// WRSEC_SELFTEST (STRIP): phase 0 = write the pattern, 1 = verify on the next capture, 2 = done.
	int  m_wrsec_phase = 0;
	u8   m_wrsec_r = 0;
	u16  m_wrsec_len = 0;
	u8   m_wrsec_pat[1200] = {};
	void capture_track();          // synchronous full-revolution flux decode into m_track
	void stage_record(captured_sector const &s);   // stage the record the firmware verifies (read AND write)
	bool write_sector(u8 want_r, const u8 *src, u32 len);   // locate by ID, overwrite data field in place
	bool m_read_active = false;     // a read window is open (records delivered until the firmware stops re-arming)
	// WRITE FIRST-ARM (cont.562).  The write's own window.  Deliberately NOT m_read_active: a write
	// locates then commits, a read captures then delivers, and conflating them would run
	// capture-and-deliver logic on a write - worse than doing nothing honestly.
	bool m_write_active = false;
	// WRITE COMPLETION:
	//   v1–v3: various immediate/deferred IRQ4 close attempts
	//   v4: skip IRQ4 if node+2==80 or [$7986]!=0 — stopped write#2 stomp; one write never posts
	//   v5: deferred re-check then raise IRQ4 if still 81 — MEASURED counterproductive (inst6):
	//       $4626 derail is one-to-one with that raise while node+2=81; healthy closes never hit it
	//   v6 (this): deferred re-check is OBSERVE-ONLY.  Cancel log if node+2==80; if still 81,
	//       log the stall (with [$7986]) and do NOT raise IRQ4.  The open defect is the one write
	//       that never self-completes with live freelist [$7986]=733c — not another IRQ4 variant.
	// m_wr_final_pending = count met, one mark cycle still owed before channel-done.
	// m_wr_complete      = model closed the write window; ignore further $0A6D for this command.
	bool m_wr_final_pending = false;
	bool m_wr_complete = false;
	// Post-STALL: sample the hang block's count word across successive IRQ1 raises (inst8 cut).
	// cnt=ffff at STALL is either wrap-from-0 (subq of 0) or a live huge countdown; only a
	// multi-tick sample separates them.  0 = inactive.
	u16  m_wr_stall_watch_blk = 0;
	int  m_wr_stall_watch_left = 0;
	u32  m_wr_host_buf = 0;         // host source address for this write (firmware IOPB [0x0d-0x0f])
	u32  m_wr_psec = 0;             // commanded block address, for deriving the wanted sector range
	int  m_wr_done = 0;             // sectors COMMITTED this command
	int  m_wr_walked = 0;           // sectors presented - bounds the walk at two revolutions
	int  m_sec_count = 0;           // commanded sectors this operation ([$7abc])
	int  m_am_presented = 0;   // EXPERIMENT cont.451: marks presented to the [$7a0c] AM-count
	u32  m_held_len = 0;            // first field of a run, awaiting a chunk to land in
	u8   m_held_data[1024] = {};
	u32  m_cur_last_chunk = 0;      // last chunk deposited into during THIS command
	u32  m_prev_last_chunk = 0;     // ...and during the previous one
	u16  m_held_r = 0;              // presented sector of the field currently held
	u16  m_want_r = 0;              // physical sector the gate array is hunting for (0 = any)
	u32  m_first_chunk = 0;         // chunk the FIRST record of this command was deposited into
	u32  m_host_dma_n = 0;          // host data transfers issued so far this command
	int  m_cmd_records = 0;         // data fields delivered THIS command (bounds the run)
	int  m_sec_index = 0;           // sector currently being delivered
	int  m_sec_phase = 0;           // 0 = ID (IRQ6) next, 1 = data (IRQ5) next
	// The commanded host extent, for the out-of-extent DMA guard.  The firmware does almost no
	// validation of what raised a record interrupt, so a spurious presentation makes it transfer an
	// extra sector into host memory PAST the requested buffer - silent corruption that previously
	// showed up only as out-of-order delivery much later.  Anything targeting outside this window is
	// a modelling fault by definition and must announce itself at the moment it happens.
	u32  m_cmd_buf = 0;             // host buffer base of the command in flight
	u32  m_cmd_bytes = 0;           // its length in bytes (nsec * sector size)
	bool m_bound_logged = false;    // one bound report per command
	bool m_extra_done = false;      // the one trailing record has already been offered this command
	bool m_extra_active = false;    // that record is in flight; restore the hunt when it lands
	int  m_extra_sec_index = 0;     // hunt state frozen across the trailing record
	u16  m_extra_want_r = 0;
	int  m_extra_blocks_left = 0;
	u32  m_extra_presented = 0;     // record cycles completed since the trailing record was offered
	attotime m_extra_deadline = attotime::never;
	attotime m_next_rec = attotime::never;   // when the next field has physically passed the head
};

// ---------------------------------------------------------------------------
// read path (74LS1801/1802 ENDEC data separation + gate-array deserialise)
// ---------------------------------------------------------------------------

// FM vs MFM cell rate.  The firmware programs the ENDEC format via E800 bits10-11 (latched from
// UIB+$11 by op 0x16, spec §3.2); the boot FM label read was observed with bit10 cleared (spec
// §3.1), so bit10 clear = FM (4us/cell, 128B), set = MFM (2us/cell, 256B).
// Density comes from the op18 FIELD PROGRAM, not from E800 bit10.  The firmware demonstrably never
// sets E800 bit10 for this purpose (measured: set once during the boot self-test at t=0.20, cleared at
// t=0.51, never again), while cont.428 established that the program describes the TRACK FORMAT - which
// is also why it is count-invariant.  Two distinct programs are observed, separating perfectly:
//   0a6d 0829 0033 0a6d 0a09 ... 023f   bit7 clear in 16/16 words   (300k FM,  16x128)
//   02ad 0aad 08a9 00b3 06ad ... 0cbb   bit7 set   in 16/16 words   (300k MFM, 16x256)
//
// TWO CAVEATS, both deliberate and both recorded rather than papered over:
//  1. UNDERDETERMINED.  0a6d ^ 02ad = $08C0, so bits 11, 7 and 6 all separate the two programs and
//     with only two samples each would test equally clean.  Bit 7 is chosen PROVISIONALLY; the data
//     does not distinguish it from 11 or 6.
//  2. The MFM sample's provenance is weak - $02ad is only ever observed at cylinder 69, i.e. after a
//     seek has already gone wrong.  It is the only second program seen and it separates cleanly, but
//     "$02ad is the MFM program" is inferred from a post-failure state.
//
// NOT taken from UIB+$12 bit1, which is the firmware's own decoded density flag ($8AE4/$7366/$9456/
// $6536/$7C5C, all consistent) and would be a one-line fix: that is a firmware-RAM snoop, the shape
// the LLE mandate rules out and that this campaign has spent months removing.  The program is what the
// gate array is actually TOLD, so it is the correct source even while its semantics are undecoded.
// UIB[4] is the STARTING SECTOR ID: the physical ID that logical sector 0 lives at.  Measured from
// the host-supplied UIB (fetched per operation from $0FE948):
//     FM  : 02 10 80 00 07 ...   heads=2 spt=16 size=$0080(128) sec0=$07
//     MFM : 02 10 00 01 0d ...   heads=2 spt=16 size=$0100(256) sec0=$0D(13)
// So logical 1 is physical 7 on FM - which is exactly where VOL1SINIX0 lives on this medium (the IMD
// puts it at cyl0/head0/R=7).  Without the mapping, physical 7 lands at buffer position 6 and
// 6 x 128 = $300 - precisely the measured displacement of the label, host $0FC0DD -> $0FC3DD.
// The GA reports the recovered ID; the mapping to the logical numbering the firmware asks for comes
// from the UIB, a host-supplied control block the gate array DMAs in itself - not a firmware snoop.
// Corroboration for the field map: UIB[1] = sectors/track is read by the FIRMWARE at $7362
// (move.b ($1,A6),D2) for exactly that purpose, so the ROM and the HLE agree on the layout.
u8 multibus_storager_device::logical_r(u8 phys) const
{
	// The CPUAP addresses the medium by LOGICAL BLOCK, not by physical sector ID: it issues SASI
	// READ(6) CDBs (built at CPUAP $fe3b88-$fe3bac - opcode, LUN, 21-bit LBA, transfer length),
	// converts the LBA to cyl/head/sector itself against its own geometry table, and hands us a
	// ZERO-BASED sector index.  Measured: LBA 0 arrives as head=0 cyl=0 sector=0, on a medium whose
	// sectors are physically numbered 1..16.
	//
	// The one thing the CPUAP cannot know is which physical sector logical block 0 begins at - that
	// is a property of how the medium was formatted, and it is what the host-supplied UIB exists to
	// tell the controller.  UIB[4] carries it (measured: 07 on the FM cylinder-0 track, 0d on the
	// MFM tracks).  So the controller's job is:
	//
	//     physical_ID = ((UIB[4] - 1 + index) mod spt) + 1
	//
	// and this function is its inverse - given a captured physical sector, which index is it - used
	// to present the R value the firmware matches its request against:
	//
	//     index = (physical_ID - UIB[4]) mod spt
	//
	// This is NOT a renumbering of the medium and it moves no data; it is the controller answering
	// "which physical sector does the volume start at".
	//
	// NAMING - two different quantities, do not conflate them:
	//   CDB LBA              the host's logical BLOCK number, 0-based, in the SASI READ(6) the CPUAP
	//                        issues (CDB[1..3]).  The CPUAP converts it to cyl/head/sector itself.
	//   firmware sector index what THIS function returns and what the ID hunt matches against.
	//                        1-BASED (see below).  It is NOT the CDB LBA and never equals it.
	//
	// The index the FIRMWARE matches against is 1-BASED, even though the CPUAP's CDB LBA and the
	// cyl/head/sector it derives from it are 0-based (measured: LBA 0 -> IOPB sector byte = 0).  So
	// the CPUAP's zero-based sector field is NOT what reaches this comparison - something between it
	// and the ID hunt re-bases it, and where that happens is not yet established.  Measured directly:
	// returning a 0-based index puts HDR1 (R=08) at host buffer+0 instead of VOL1 (R=07), i.e. every
	// record lands one slot early.  Hence the +1 - it is empirical, not cosmetic; do not remove it
	// without re-running the LBA 0 len 8 read and checking VOL1 lands at buffer+0.
	// cont.457: the CPUAP addresses the volume LINEARLY and resolves the cylinder itself.  Measured:
	//   working cyl-0 read : REQ cyl=0 head=0 start=0  count=8   (drive at cyl 0)
	//   failing cyl-1 read : REQ cyl=0 head=0 start=32 count=4   (SEEK CHECK: firmware wants cyl=1)
	// The IOCB cylinder field stays 0 because fe3833 zeroes it deliberately for this slot and carries
	// the whole block number in the sector field.  So the hunt compares a LINEAR block index while the
	// model was presenting a TRACK-RELATIVE R - two different spaces, which is why 32..35 never
	// matched 1..16, and why cylinder 0 worked only by coincidence (there the spaces agree when the
	// run starts at physical 1).
	//
	//     presented_index = (cyl * heads + head) * spt + (R - 1)
	//
	// cyl/head come from the DRIVE (get_cyl(), and the side-select the model itself drove), not from a
	// tracked variable, so the index cannot disagree with the physical head position by construction -
	// the cylinder-68 desync therefore cannot corrupt recognition even while it stays unexplained.
	//
	// LIMIT, documented not hidden: the presented value is written into a one-byte ID field, so this
	// truncates past index 255 (cylinder 8 at 2 heads x 16 sectors).  The request itself may be 16-bit
	// (node+8/+9 read as 00 20 for block 32), so a wider presented index is needed before anything
	// beyond cylinder 7 can be recognised.  Sufficient for the label + boot-file path; NOT sufficient
	// for a full install.
	// SECTOR_ID_LINEAR - REFUTED, see above - body removed; the code is preserved verbatim in commit bebdc9428d9
	// (pre-cleanup checkpoint).  Kept as a comment so the verdict survives without
	// dead machinery that reads as live code.
	if (true)    // SECTOR_ID_REMAP - REFUTED, see above
		return phys;
	if (m_uib_base < 0x4000 || m_uib_base >= 0x8000)
		return phys;
	address_space &cs = m_cpu->space(AS_PROGRAM);
	u8 const spt  = cs.read_byte((m_uib_base + 1) & 0xffff);
	// UIB[4], NOT UIB[5] - and that choice is ASSUMED, not measured.  Both floppy UIBs carry the
	// same value in +4 and +5 (07/07 FM, 0d/0d MFM), so no floppy observation can distinguish them.
	// The HARD-DISK UIB proves they are independent fields: it reads +4=0d, +5=13.  If this mapping
	// is ever wrong, the HD is the discriminator - do not re-confirm it against a floppy.
	u8 const base = cs.read_byte((m_uib_base + 4) & 0xffff);   // physical sector the volume starts at
	if (spt == 0 || base < 1 || base > spt || phys < 1 || phys > spt)
		return phys;
	return u8(((phys - base + spt) % spt) + 1);
}

bool multibus_storager_device::flux_density_fm() const
{
	// cont.443: density is UIB+$12 BIT 2, not the program's bit7 and not UIB+$12 bit1.
	// Measured against the media, which is mixed (cyl 0 FM 16x128, cyl 1+ MFM 16x256):
	//   cyl  0  UIB+12=$40  bit1=0 bit2=0  media FM
	//   cyl  1  UIB+12=$44  bit1=0 bit2=1  media MFM
	//   cyl 69  UIB+12=$37  bit1=1 bit2=1  media MFM
	//   cyl 83  UIB+12=$2f  bit1=1 bit2=1  media MFM
	// bit2 tracks the media exactly; bit1 does not (it is clear for the MFM cylinder-1 read).
	// The earlier reading - program bit7 == UIB+$12 bit1 == density - correlated two signals against
	// each other and never against the media; they agreed while both were wrong at cylinder 1.
	// The UIB is a HOST-SUPPLIED control block the gate array itself DMAs in (measured: fetched from
	// host $0FE948 per operation), so reading it is not a firmware-RAM snoop - these bytes pass
	// through the gate array's own channel.
	address_space &cs = m_cpu->space(AS_PROGRAM);
	if (m_uib_base >= 0x4000 && m_uib_base < 0x8000)
		return !BIT(cs.read_byte((m_uib_base + 0x12) & 0xffff), 2);
	return !BIT(m_ch[(0xe800 - 0xe000) / 2], 10);   // no UIB yet: fall back
}

// Detection-is-capture: sweep the whole track's flux once (the gate array is continuously reading the
// rotating disk) and decode every sector's ID + data field into m_track, in physical order.  The flux is
// the track's angular data - readable ahead of machine time - so this is synchronous.
// ---------------------------------------------------------------------------------------------
// FLUX SECTOR WRITE.  The mirror of capture_track(): sweep the track exactly as the read does,
// and when the TARGET sector's data address mark passes, switch the PLL to writing and overwrite
// the data field and its CRC in place.
//
// WHY IN-PLACE, and not "regenerate the sector".  Rebuilding a sector means emitting gaps of a
// computed length, and a wrong gap runs into the NEXT sector's ID and destroys it.  This medium is
// not IBM-standard (FM 16x128 on cyl 0, MFM 16x256 above), so those lengths are not known.  By
// starting the write immediately after the DAM the ID, both address marks and every gap are left
// physically untouched - only the bytes that are supposed to change do.
//
// Encoding conventions are taken from wd_fdc (live_write_mfm / live_write_fm / write_one_bit),
// not reinvented: MFM sets the clock bit only when neither this data bit nor the previous one is
// set; FM presents every clock cell.  Raw words shift out MSB-first, one cell per pll bit.  CRC is
// CCITT-16 (poly 0x1021, init 0xffff) over the address-mark bytes and the data.
//
// Returns true only if the sector was located AND committed.
static u16 crc_ccitt_byte(u16 crc, u8 b)
{
	crc ^= u16(b) << 8;
	for (int i = 0; i < 8; i++)
		crc = (crc & 0x8000) ? u16((crc << 1) ^ 0x1021) : u16(crc << 1);
	return crc;
}

static u16 mfm_raw_byte(u8 b, bool &context)
{
	u16 raw = 0;
	for (int i = 0; i < 8; i++)
	{
		bool const bit = BIT(b, 7 - i);
		if (!(bit || context)) raw |= 0x8000 >> (2 * i);
		if (bit)               raw |= 0x4000 >> (2 * i);
		context = bit;
	}
	return raw;
}

static u16 fm_raw_byte(u8 b)
{
	u16 raw = 0xaaaa;                       // every clock cell present
	for (int i = 0; i < 8; i++)
		if (BIT(b, 7 - i)) raw |= 0x4000 >> (2 * i);
	return raw;
}

// want_r is the PHYSICAL sector ID as it appears in the ID field on the medium (`R`), which on
// this medium is track-relative and 1-based - see storager-sector-id-track-relative.  Callers must
// NOT pass a logical/linear block number; converting is the caller's job.
bool multibus_storager_device::write_sector(u8 want_r, const u8 *src, u32 len)
{
	floppy_image_device *const fdd = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
	if (!fdd || !fdd->exists())
	{
		logerror("WRSEC r=%02x REFUSED: no medium\n", want_r);
		return false;
	}
	if (fdd->wpt_r())                       // wpt_r() true == protected (upd765: `wpt_r() ? ST3_WP : 0`)
	{
		logerror("WRSEC r=%02x REFUSED: medium is WRITE PROTECTED\n", want_r);
		return false;
	}
	fdd->mon_w(0);
	fdd->ss_w(m_walk_head & 1);
	bool const fm = flux_density_fm();
	fdc_pll_t pll;
	pll.set_clock(attotime::from_nsec(fm ? 4000 : 2000));
	attotime tm = machine().time();
	pll.read_reset(tm);
	attotime const end = tm + attotime::from_msec(210);   // > one revolution at 300 rpm

	// ---- LOCATE: sweep to the target sector's data address mark ----------------------------
	u32 shift = 0;
	int state = 0, cells = 0, nb = 0, want = 0, n = 0;
	u8 dam = 0, id[4] = {};
	bool have_id = false, found = false;
	u8 buf[1200];
	while (!found)
	{
		int const bit = pll.get_next_bit(tm, fdd, end);
		if (bit < 0)
			break;                          // ran out of track
		shift = (shift << 1) | unsigned(bit);
		if (state == 0)
		{
			if (fm && (shift & 0xffff) == 0xf57e) { state = 2; cells = 0; nb = 0; want = 4; }
			else if (fm && ((shift & 0xffff) == 0xf56f || (shift & 0xffff) == 0xf56a))
			{
				state = 3; cells = 0; nb = 0; want = 128 << n;
				dam = ((shift & 0xffff) == 0xf56f) ? 0xfb : 0xf8;
				// Stop HERE - tm is exactly one cell past the DAM.  Testing for the target after
				// the byte-decode step below consumes the first DATA byte first and starts the
				// write one byte late; the self-test caught exactly that (differs=128, firstbad=0).
				if (have_id && id[2] == want_r) { found = true; break; }
			}
			else if (!fm && (shift & 0xffff) == 0x4489) { state = 1; cells = 0; }
			continue;
		}
		if (++cells % 16)
			continue;
		u8 b = 0;
		for (int k = 7; k >= 0; k--)
			b = u8((b << 1) | ((shift >> (2 * k)) & 1));
		if (state == 1)
		{
			if (b == 0xa1) continue;
			if (b == 0xfe) { state = 2; nb = 0; want = 4; continue; }
			if (b == 0xfb || b == 0xf8)
			{
				state = 3; nb = 0; want = 128 << n; dam = b;
				if (have_id && id[2] == want_r) { found = true; break; }   // tm is just past the DAM
				continue;
			}
			state = 0; continue;
		}
		if (nb < int(sizeof(buf))) buf[nb] = b;
		nb++;
		int const need = (state == 3) ? want + 2 : want;
		if (nb < need) continue;
		if (state == 2) { id[0]=buf[0]; id[1]=buf[1]; id[2]=buf[2]; id[3]=buf[3]; n = buf[3] & 7; have_id = true; }
		else            { have_id = false; }
		state = 0; nb = 0;
	}
	if (!found)
	{
		logerror("WRSEC r=%02x NOT FOUND on cyl=%d head=%d (%s) - nothing written\n",
			want_r, fdd->get_cyl(), m_walk_head & 1, fm ? "FM" : "MFM");
		return false;
	}

	// ---- COMMIT: overwrite the data field + CRC, leaving marks and gaps alone ---------------
	// The CRC covers the address-mark bytes the medium already carries: MFM A1,A1,A1,DAM; FM DAM.
	u16 crc = 0xffff;
	if (!fm) { crc = crc_ccitt_byte(crc, 0xa1); crc = crc_ccitt_byte(crc, 0xa1); crc = crc_ccitt_byte(crc, 0xa1); }
	crc = crc_ccitt_byte(crc, dam);
	bool context = BIT(dam, 0);             // MFM clock of the first data byte follows the DAM's last bit
	u32 const nbytes = std::min<u32>(len, u32(want));
	pll.start_writing(tm);
	// COMMIT PER BYTE.  fdc_pll_t::write_buffer holds only 32 transitions; wd_fdc calls
	// pll_commit() inside its live loop for exactly this reason.  Emitting a whole 128/256-byte
	// field with a single commit at the end overruns the buffer and everything past the first two
	// bytes is corrupt - measured: bytes 0-1 correct, 2..127 garbage, which is 32 cells in.
	auto emit = [&](u8 byte)
	{
		u16 raw = fm ? fm_raw_byte(byte) : mfm_raw_byte(byte, context);
		for (int k = 0; k < 16; k++)
		{
			pll.write_next_bit((raw & 0x8000) != 0, tm, fdd, end);
			raw = u16(raw << 1);
		}
		pll.commit(fdd, tm);
	};
	for (u32 k = 0; k < nbytes; k++) { emit(src[k]); crc = crc_ccitt_byte(crc, src[k]); }
	for (u32 k = nbytes; k < u32(want); k++) { emit(0x00); crc = crc_ccitt_byte(crc, 0x00); }
	emit(u8(crc >> 8));
	emit(u8(crc));
	pll.commit(fdd, tm);
	pll.stop_writing(fdd, tm);
	logerror("WRSEC r=%02x cyl=%d head=%d %s len=%u (src %u) crc=%04x COMMITTED t=%.5f\n",
		want_r, fdd->get_cyl(), m_walk_head & 1, fm ? "FM" : "MFM", want, len, crc,
		machine().time().as_double());
	return true;
}

void multibus_storager_device::capture_track()
{
	m_track_n = 0;
	floppy_image_device *const fdd = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
	if (!fdd || !fdd->exists())
		return;
	// The side comes from the head the firmware SELECTED through E804, which the gate array latches -
	// not from the firmware's own work area.  The gate array has no visibility into [$7436]; it knows
	// only what it was commanded.
	fdd->mon_w(0);
	fdd->ss_w(m_walk_head & 1);   // cont.525: the GA's walk head, not the latched select
	bool const fm = flux_density_fm();
	fdc_pll_t pll;
	pll.set_clock(attotime::from_nsec(fm ? 4000 : 2000));
	attotime tm = machine().time();
	pll.read_reset(tm);
	attotime const end = tm + attotime::from_msec(210);   // >1 revolution at 300 rpm (200ms)
	u32 shift = 0;
	int state = 0, cells = 0, nb = 0, want = 0, n = 0;
	u8 dam = 0, id[4] = {};
	// A data field is only a sector if its OWN ID field preceded it.  The sweep starts at an arbitrary
	// rotational position, so it can begin mid-sector and meet a data field whose ID already passed the
	// head - storing that yields a phantom sector carrying the STALE id[] (r=00) and the stale size
	// code (128 instead of 256).  Measured on cyl 69 (a 16x256 MFM track): 17 sectors, first r=00
	// len=128.  This was invisible while only FM was exercised. (cont.440)
	bool have_id = false;
	u8 buf[1200];
	while (m_track_n < 32)
	{
		int const bit = pll.get_next_bit(tm, fdd, end);
		if (bit < 0)
			break;
		shift = (shift << 1) | unsigned(bit);
		if (state == 0)   // HUNT
		{
			if (fm && (shift & 0xffff) == 0xf57e) { state = 2; cells = 0; nb = 0; want = 4; }
			else if (fm && ((shift & 0xffff) == 0xf56f || (shift & 0xffff) == 0xf56a)) { state = 3; cells = 0; nb = 0; want = 128 << n; dam = ((shift & 0xffff) == 0xf56f) ? 0xfb : 0xf8; }
			else if (!fm && (shift & 0xffff) == 0x4489) { state = 1; cells = 0; }
			continue;
		}
		if (++cells % 16)
			continue;
		u8 b = 0;
		for (int k = 7; k >= 0; k--)
			b = u8((b << 1) | ((shift >> (2 * k)) & 1));
		if (state == 1)   // MFM READ_MARK
		{
			if (b == 0xa1) continue;
			if (b == 0xfe) { state = 2; nb = 0; want = 4; continue; }
			if (b == 0xfb || b == 0xf8) { state = 3; nb = 0; want = 128 << n; dam = b; continue; }
			state = 0; continue;
		}
		if (nb < int(sizeof(buf)))
			buf[nb] = b;
		nb++;
		int const need = (state == 3) ? want + 2 : want;
		if (nb < need)
			continue;
		if (state == 2)   // ID field recovered
		{
			id[0] = buf[0]; id[1] = buf[1]; id[2] = buf[2]; id[3] = buf[3];
			n = buf[3] & 7;
			have_id = true;
		}
		else if (have_id)   // data field recovered, and its ID preceded it -> store this sector
		{
			have_id = false;                    // this ID belongs to exactly one data field
			// cont.531: the sweep window is 210ms against a 200ms revolution - deliberately over-scanned
			// so a full turn is guaranteed - and that 10ms overlap RE-READS the starting sector when the
			// rotational phase lines up.  Measured on cyl 3 head 0: six captures of the same track give
			// 16 sectors with first != last, one gives 17 with `first: r=03 ... last: r=03` - the same
			// ID at both ends.  m_track_n then feeds the hunt wrap (`m_want_r % m_track_n + 1`), which
			// produced the out-of-range `PROG start sector = 17`.  Stop at the wrap: an ID already in
			// the set means the head has come round again.  (Sibling of the cont.440 have_id guard,
			// which fixed the mid-sector START; this fixes the mid-sector END.)
			bool dup = false;
			for (int q = 0; q < m_track_n; q++)
				if (m_track[q].r == id[2]) { dup = true; break; }
			if (dup)
				break;
			captured_sector &s = m_track[m_track_n++];
			s.c = id[0]; s.h = id[1]; s.r = id[2]; s.nn = id[3]; s.dam = dam;
			s.len = u16(want);
			for (int k = 0; k < want && k < int(sizeof(s.data)); k++)
				s.data[k] = buf[k];
		}
		state = 0;
	}
	// TEMP cont.439: the MFM path is exercised for the first time (media is mixed - cyl 0 is 300k FM
	// 16x128, cyl 1+ are 300k MFM 16x256).  Report what the gate array actually decoded.
	// TEMP cont.440: content check - dump sector R=1's first 8 bytes so the decode can be compared
	// against the media (cyl 69 head 0 R=1 is 000000907c4e18a6 in the IMD).
	for (int i = 0; i < m_track_n; i++)
		if (m_track[i].r == 1)
		{
			logerror("  R=1 first8: %02x%02x%02x%02x%02x%02x%02x%02x  (len=%d)\n",
				m_track[i].data[0], m_track[i].data[1], m_track[i].data[2], m_track[i].data[3],
				m_track[i].data[4], m_track[i].data[5], m_track[i].data[6], m_track[i].data[7],
				m_track[i].len);
			break;
		}
	// TEMP cont.442: is the head where the operation TARGETS?  [$7438] is the firmware's expected
	// cylinder and [$7436] its expected head - the values the record verify compares against
	// ($7C52-$7C66 and $7C40-$7C46).  Compare them against the drive's actual position.
	{
		address_space &cs4 = m_cpu->space(AS_PROGRAM);
		floppy_image_device *const f4 = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
		int const act = f4 ? f4->get_cyl() : -1;
		int const wantc = cs4.read_word(0x7438) & 0xff;
		int const wanth = cs4.read_word(0x7436) & 0xff;
		logerror("  SEEK CHECK: firmware wants cyl=%d head=%d | drive at cyl=%d head=%d | %s\n",
			wantc, wanth, act, m_sel_head,
			(act == wantc) ? "MATCH" : "*** MISMATCH - head not where the operation targets ***");
	}
	// RETIRED (cont.560): the three outcomes below were decided by a dump that only ever looked for
	// $FF, so it reported "(none)" unconditionally and outcome three - "the firmware never built a
	// want-list" - was a FALSE EMPTY, not a measurement.  The ledger is populated: $AA end sentinel,
	// $FE/$FF special, $C0 fill.  What the ledger actually drives is the PRE-ARM fork at $7E9A/$7ED0:
	// the first special byte at or after R+1 is $AA -> $7ED8 clears [$741c] and no chunk is armed;
	// anything else -> $7EE0 pops a chunk ($32AC), installs it at [$741e] and $7F14 sets [$741c]=1.
	// That is an allocation for the NEXT record, not a verdict on the last one.  Kept only as a
	// record of how the wrong instrument produced a confident wrong conclusion.
	// cont.459: dump the WANT-LIST LEDGER ($7654..$76BF) before any record is delivered.  Acceptance
	// is a ledger lookup ($ff = wanted at that position), NOT an arithmetic compare - which is why an
	// uncapped grep found only head/cylinder compares and no sector compare at all.  Three outcomes:
	//   wants at positions 1..4    -> identity is right; the linear index DOUBLE-COUNTS the cylinder
	//                                 (the firmware already applies its own base), revert it
	//   wants at positions 33..36  -> linear is right; the defect is the off-by-one arm window
	//   NO wants marked at all     -> the firmware never built a want-list; BOTH index schemes are
	//                                 untested and the defect is upstream of the presented identity
	// cont.463: STATE IN FORCE AT THE POINT OF USE, not an event somewhere upstream.  "Did a program
	// load or $023F wipe happen between READ #1 and READ #3" is an event probe with a hidden third
	// outcome - a load occurs and loads the WRONG thing.  Snapshotting the state when the read arms
	// distinguishes three: no reload (retention defect), reload carrying FM parameters (a different
	// defect in the same family), or correct MFM state (retention exonerated).  Event probes miss by
	// sampling window or by firing upstream; a state read at the moment of use has no window to miss.
	{
		address_space &sc = m_cpu->space(AS_PROGRAM);
		u8 const u_heads = (m_uib_base >= 0x4000 && m_uib_base < 0x8000) ? sc.read_byte(m_uib_base + 0) : 0xff;
		u8 const u_spt   = (m_uib_base >= 0x4000 && m_uib_base < 0x8000) ? sc.read_byte(m_uib_base + 1) : 0xff;
		u16 const u_bps  = (m_uib_base >= 0x4000 && m_uib_base < 0x8000)
			? u16((sc.read_byte(m_uib_base + 2) << 8) | sc.read_byte(m_uib_base + 3)) : 0xffff;
		u8 const u_sec0  = (m_uib_base >= 0x4000 && m_uib_base < 0x8000) ? sc.read_byte(m_uib_base + 4) : 0xff;
		u8 const u_d12   = (m_uib_base >= 0x4000 && m_uib_base < 0x8000) ? sc.read_byte(m_uib_base + 0x12) : 0xff;
		logerror("STATE@ARM: prog_loaded=%d loading=%d id[$793c]=%04x prev[$793e]=%04x"
			"  | UIB@%04x heads=%u spt=%u bps=%u sec0=%02x +12=%02x  | model density=%s\n",
			m_prog_loaded ? 1 : 0, m_prog_loading ? 1 : 0,
			sc.read_word(0x793c), sc.read_word(0x793e),
			m_uib_base, u_heads, u_spt, u_bps, u_sec0, u_d12,
			flux_density_fm() ? "FM" : "MFM");
	}
	{
		// THE LEDGER HAS FOUR CODEPOINTS, NOT ONE.  The scan the firmware actually runs is $7E6C:
		//     addq.w #1,D0
		//     cmpi.b #$AA,(A0,D0.w) beq $7E8A     end-of-track sentinel
		//     cmpi.b #$FF,(A0,D0.w) beq $7E86     special
		//     cmpi.b #$FE,(A0,D0.w) bne $7E6C     special; anything else is SKIPPED
		// so $AA / $FF / $FE are all live and everything else ($C0 fill from the $09F8 init) is
		// stepped over.  Reporting only $FF made this dump answer "(none)" on every command of
		// every run while the ledger was populated with $FE and $AA - a FALSE EMPTY, which is what
		// the retired "the firmware never built a want-list" conclusion rested on.  $FF is a real
		// codepoint ($7D2C, $178E); the dump was incomplete, not aimed at a fiction.
		address_space &lc = m_cpu->space(AS_PROGRAM);
		char w[400]; w[0] = 0; int wn = 0; int pos = 0;
		for (u32 k = 0x7654; k <= 0x76bf; k++)
		{
			u8 const v = lc.read_byte(k);
			if ((v == 0xaa || v == 0xff || v == 0xfe) && wn < 28)
			{
				pos += sprintf(w + pos, "%u:%02x ", k - 0x7654, v);
				wn++;
			}
		}
		logerror("LEDGER $7654..$76BF: %d special cell(s) [pos:byte]: %s\n", wn, wn ? w : "(none)");
	}
	if (WRSEC_SELFTEST && m_wrsec_phase == 0 && m_track_n > 0 && !flux_density_fm())
	{
		m_wrsec_r = m_track[0].r;
		m_wrsec_len = m_track[0].len;
		for (u32 k = 0; k < m_wrsec_len && k < sizeof(m_wrsec_pat); k++)
			m_wrsec_pat[k] = u8(0x5a ^ (k * 7));
		logerror("WRSEC-TEST phase1: writing pattern into r=%02x len=%u\n", m_wrsec_r, m_wrsec_len);
		m_wrsec_phase = write_sector(m_wrsec_r, m_wrsec_pat, m_wrsec_len) ? 1 : 2;
	}
	else if (WRSEC_SELFTEST && m_wrsec_phase == 1)
	{
		m_wrsec_phase = 2;
		int idx = -1;
		for (int i = 0; i < m_track_n; i++)
			if (m_track[i].r == m_wrsec_r) { idx = i; break; }
		if (idx < 0)
			logerror("WRSEC-TEST phase2: *** FAIL *** r=%02x no longer decodes - the write destroyed the sector\n", m_wrsec_r);
		else
		{
			int diff = 0, first = -1;
			for (u32 k = 0; k < m_wrsec_len; k++)
				if (m_track[idx].data[k] != m_wrsec_pat[k]) { if (first < 0) first = int(k); diff++; }
			{
				std::string ex, ac;
				for (int k = 0; k < 12; k++)
				{
					ex += util::string_format(" %02x", m_wrsec_pat[k]);
					ac += util::string_format(" %02x", m_track[idx].data[k]);
				}
				logerror("WRSEC-TEST   expected:%s\n             actual  :%s\n", ex.c_str(), ac.c_str());
			}
			logerror("WRSEC-TEST phase2: r=%02x len=%u/%u differs=%d firstbad=%d  sectors_on_track=%d -> %s\n",
				m_wrsec_r, m_track[idx].len, m_wrsec_len, diff, first, m_track_n,
				(diff == 0 && m_track[idx].len == m_wrsec_len) ? "*** PASS ***" : "*** FAIL ***");
		}
	}
	logerror("capture_track: cyl=%d head=%d density=%s  sectors=%d  first: r=%02x len=%d  last: r=%02x len=%d\n",
		m_floppy[0] && m_floppy[0]->get_device() ? m_floppy[0]->get_device()->get_cyl() : -1,
		m_walk_head & 1, flux_density_fm() ? "FM" : "MFM", m_track_n,   // the head ACTUALLY decoded
		                                                                  // (ss_w uses m_walk_head;
		                                                                  // m_sel_head reads 0 all
		                                                                  // command and hid the switch)
		m_track_n ? m_track[0].r : 0, m_track_n ? m_track[0].len : 0,
		m_track_n ? m_track[m_track_n - 1].r : 0, m_track_n ? m_track[m_track_n - 1].len : 0);
}



// op18's load has completed and the gate array's sequencer begins running the field program: open the
// read window, decode the track, and arm record delivery.  This is the point
// the program starts - not the E000 word[0] write that merely begins loading it.

// 300 rpm = 200ms/revolution, divided by the sectors on the track (FM 16x128B -> 12.5ms).
attotime multibus_storager_device::sector_period() const
{
	return attotime::from_msec(200) / std::max(1, m_track_n);
}

void multibus_storager_device::start_field_program()
{
	if (!cmd_armable())
		return;
	// Post-completion $0A6D re-arms must not re-open a finished write (endless transfer).  The
	// firmware has no write-side count/done path; only the model stops answering.
	if (cmd_is_write() && m_wr_complete)
	{
		logerror("WRREARM ignored (write already complete) cmd=%02x t=%.5f\n",
			m_iopb_cmd, machine().time().as_double());
		return;
	}
	// Final pre-IRQ4 cycle: firmware re-issues $0A6D; ensure we are armed so the mark pump
	// actually runs (v1 failed here — pending set, never reached phase 3).
	if (cmd_is_write() && m_write_active && m_wr_final_pending)
	{
		m_armed = true;
		m_read_window = true;
		m_next_rec = machine().time();
		m_pump->adjust(attotime::from_usec(100));
		logerror("WRFINAL re-arm engage cmd=%02x done=%d/%d t=%.5f\n",
			m_iopb_cmd, m_wr_done, m_sec_count, machine().time().as_double());
		return;
	}
	// The commanded sector count is carried by the program the firmware just loaded - one port-$3F
	// push per sector - so the gate array has it without reading the IOCB or any firmware cell.
	{
		int p3f = 0;
		for (int i = 0; i < m_desc_n; i++)
			if (m_desc_port[i] == 0x3f)
				p3f++;
		m_prog_count = p3f;
		// WHERE DOES A WRITE ENCODE ITS COUNT?  ANSWERED: exactly where a read does.  The count is
		// pushes of C800 port $3F (captured in c800_w, framed by E800 bit7), and a write pushes
		// SIXTEEN of them - same as a read.  The count was never the problem.
		//
		// Do not re-derive "the write has no count" from a `PROG n=0` line: that logerror lives
		// inside the m_iopb_cmd == 0x95 block below and CANNOT fire for a write.  Any such line is
		// a later READ arming against the write program still resident in E000 (the gate array
		// retains its program across commands), and its `n` is m_desc_n, not a sector count.
		//
		// What the stream does show: a write pushes TWO descriptor blocks (32 pushes = 2 x 16)
		// where a read pushes one, and uses different port numbers.  With the E000 bit8 finding
		// (bit8 = write gate; slot 1 `0829` identical in both programs and the only active write
		// word with bit8 clear) that is two independent lines of evidence for LOCATE-THEN-WRITE:
		// read the ID to find the sector, then write its data field.
		if (TRACE_GAWRITE && cmd_is_write())
		{
			std::string ps;
			for (int i = 0; i < m_desc_n; i++)
				ps += util::string_format(" %02x:%04x", m_desc_port[i], m_desc_val[i]);
			if (ps.empty()) ps = " (none - no descriptors pushed)";
			logerror("GAWDESC cmd=%02x m_desc_n=%u port3f=%d |%s t=%.5f\n",
				m_iopb_cmd, m_desc_n, p3f, ps.c_str(), machine().time().as_double());
		}
	}
	m_read_window = true;
	m_armed = true;
	// ---- WRITE FIRST-ARM ------------------------------------------------------------------
	// The fork.  A read captures the track and delivers records; a write must LOCATE the target
	// sector and then COMMIT data to it.  What is genuinely shared is the locate step: the write's
	// descriptor slot 1 is byte-identical to the read's, and E000 slot 1 (`0829`) is the only
	// active write word with bit8 (write-gate) CLEAR - i.e. the firmware reads the ID first.  So
	// the ID phase reuses the read's mark delivery; nothing beyond it is assumed.
	if (!m_write_active && cmd_is_write())
	{
		// SEED THE WALK SURFACE FROM THE SELECT LATCH - WRITE COMMAND ENTRY ONLY.
		// MEASURED (cont.566): on every failing 2a write the staged ID head followed m_walk_head
		// while the firmware's wanted head ([$7436], copied from [$7946]) followed the SELECT latch,
		// and the two disagreed - #5 sel=1/walk=0 staged h=00 against want 01, #7 sel=0/walk=1 staged
		// h=01 against want 00.  On every healthy write sel == walk.  So capture_track() swept the
		// wrong surface and the ID it staged was internally consistent with the track we captured -
		// we captured the wrong one.  The verify at $98F6 then fails and op $48 returns $202A.
		// m_walk_head is the gate array's OWN position across a multi-track walk and is SUPPOSED to
		// diverge from the select latch mid-command (the $1949 deferred flip crosses track
		// boundaries), so this seeding belongs at COMMAND ENTRY and nowhere else.  cont.526 measured
		// what happens if it is done in the program re-arm block instead: the re-arm re-seeded
		// m_walk_head 14us after a legitimate crossing had advanced it (WALK -> head=1 at t=8.83315,
		// re-seeded head=0 at t=8.83329) and threw the crossing away.  Walk state belongs to the
		// COMMAND, not to a program re-arm.  Any pending boundary flip is prior-command residue here
		// and would flip the surface out from under this write.
		m_walk_head = m_sel_head & 1;
		m_walk_pending = false;
		capture_track();                     // the ID layout we must match against
		m_wr_done = 0;
		m_wr_walked = 0;
		m_wr_final_pending = false;
		m_wr_complete = false;
		m_wr_stall_watch_blk = 0;
		m_wr_stall_watch_left = 0;
		if (m_wr_close)
			m_wr_close->adjust(attotime::never);   // cancel prior command's failsafe
		address_space &hs = m_bus->space(AS_PROGRAM);
		u32 const iopb = m_iopb_addr;
		m_wr_host_buf = (u32(hs.read_byte((iopb + 0x0d) & 0xffffff)) << 16)
		              | (u32(hs.read_byte((iopb + 0x0e) & 0xffffff)) << 8)
		              |  u32(hs.read_byte((iopb + 0x0f) & 0xffffff));
		m_wr_psec = (u32(hs.read_byte((iopb + 0x06) & 0xffffff)) << 24)
		          | (u32(hs.read_byte((iopb + 0x07) & 0xffffff)) << 16)
		          | (u32(hs.read_byte((iopb + 0x08) & 0xffffff)) << 8)
		          |  u32(hs.read_byte((iopb + 0x09) & 0xffffff));
		int n = m_cpu->space(AS_PROGRAM).read_word(0x7abc) & 0xff;
		if (n < 1 || n > m_track_n) n = m_track_n;
		m_sec_count = n;
		double const rev = sector_period().as_double() * std::max(1, m_track_n);
		double const phase = rev > 0.0 ? std::fmod(machine().time().as_double(), rev) / rev : 0.0;
		m_sec_index = int(phase * std::max(1, m_track_n)) % std::max(1, m_track_n);
		m_sec_phase = 0;
		m_write_active = true;
		m_armed = true;
		m_read_window = true;
		// STAGE BEFORE ANY MARK CAN BE TAKEN.  Staging only inside the locate phase is too late if
		// an interrupt is already pending when the write arms: the ISR then reads $7D9E before we
		// have written it and the $A1 sync check sees 00.  MEASURED: 2 of 5 checks read
		// `d2 low byte = 00` with a3=$7DA1 (i.e. the right address, empty), 3 read A1 and passed.
		// The content and the address were never wrong - only the ordering.
		if (m_track_n > 0)
			stage_record(m_track[m_sec_index % m_track_n]);
		logerror("WRARM cmd=%02x first-arm: track_n=%d sec_count=%d host_buf=%06x cyl=%d %s t=%.5f\n",
			m_iopb_cmd, m_track_n, m_sec_count, m_wr_host_buf,
			m_floppy[0] && m_floppy[0]->get_device() ? m_floppy[0]->get_device()->get_cyl() : -1,
			flux_density_fm() ? "FM" : "MFM", machine().time().as_double());
	}
	bool const first_arm = !m_read_active;
	if (!m_read_active && m_iopb_cmd == 0x95)
	{
		// ARM AT GO, DECODE AT SEEK-COMPLETE.  A floppy cannot read while seeking - the heads are
		// moving and then need their settle time - so the gate array arms on the command but must
		// not begin decoding until the drive is on-cylinder and settled.  The model conflated the
		// two and captured synchronously at engage, which was harmless only while every read
		// found the head already in position (the whole boot).  The first read that genuinely had
		// to seek captured the OLD cylinder before the head moved, and the next one captured
		// cyl 0 with the MFM decoder - cyl 0 is FM, so zero sectors decoded, the armed chunk
		// never filled and the firmware raised its $7B1C watchdog (0x201C).
		//
		// The settle one-shot is already modelled: every STEP retriggers PIT0 ctr1 (mode 5) and
		// F000 bit1 reports busy while it runs.  Defer the capture until its OUT rises; arming
		// (m_armed / m_read_window, set above) is untouched, so cont.429's retained-program
		// engage-at-command-boundary still holds.
		capture_track();
		// per-COMMAND state.  NOTE: start_field_program is re-entered on EVERY record (the ISR
		// re-issues the program's first command, E000 <= $0a6d, bit11 set), so anything that must
		// happen once per read belongs in this block, never above it.
		m_aa_cell = 0; m_aa_armed = false; m_aa_done = false;
		m_accepted_n = 0; m_aa_last = 0; m_aa_n = 0;
		int n = m_cpu->space(AS_PROGRAM).read_word(0x7abc) & 0xff;   // commanded sectors this track [$7abc]
		logerror("7ABC-READ n=%d m_track_n=%d t=%.5f\n", n, m_track_n, machine().time().as_double());
		if (n < 1 || n > m_track_n) n = m_track_n;
		m_sec_count = n;
		// cont.526: the walk is seeded at HOST GO, NOT here.  This block is re-entered whenever
		// m_read_active drops - including the PROG n=0 re-arm that follows a track boundary - and
		// seeding here reset m_walk_head to the firmware-selected head 14us after the walk advanced
		// it, throwing the crossing away (measured: WALK -> head=1 at t=8.83315, re-seeded head=0 at
		// t=8.83329).  Walk state belongs to the COMMAND, not to a program re-arm.
		// Where is the head NOW?  The disk has been turning since power-on, so the first address mark
		// the gate array meets is whichever sector happens to be passing - essentially never sector 1.
		// This matters to the FIRMWARE, not just to realism.  The drain marks its want-list ($ff at the
		// commanded positions $7654+base..) and the per-record handler ACCEPTS those ($7C7A -> $7E0A,
		// and $7CA4 CLEARS [$741c]) while REJECTING everything else ($7D34 -> $7D4A -> $7D5C, which
		// SETS [$741c]).  [$741c] is the sole bootstrap for [$7968] (via $82B2's gate), and [$7968] is
		// what flips [$742C] at $7E8E so $8128 stakes a drainable index instead of $8120 writing c0.
		// Starting every read at sector 1 means the firmware only ever sees wanted records, never
		// rejects one, and can never bootstrap - the ledger stays undrainable and the drain finds D3=0.
		double const rev = sector_period().as_double() * std::max(1, m_track_n);
		double const phase = rev > 0.0 ? std::fmod(machine().time().as_double(), rev) / rev : 0.0;
		m_sec_index = int(phase * m_track_n) % std::max(1, m_track_n);
		m_sec_phase = 0;
		m_read_active = true;
		// cont.520: dump the field program the firmware just pushed.  The spec lists 'compare' among
		// the E000 command-word operations and leaves sector selection open; the ISR is measured NOT
		// to compare sector numbers, so the wanted sector must be IN this program.  Find it.
		{
			char pb[32*12+1]; pb[0]=0; int pp=0;
			for (int i = 0; i < m_desc_n && pp < int(sizeof(pb))-12; i++)
				pp += sprintf(pb+pp, "%02x:%04x ", m_desc_port[i], m_desc_val[i]);
			// cont.521: the gate array RETAINS its field program AND its sector-hunt state.  A
			// program-reusing command pushes no descriptors (m_desc_n == 0) and NO port supplies a
			// start sector - measured at the mirror addresses (0xFFxxxx; short-absolute sign-extends):
			// every port such a command writes carries values IDENTICAL to a working read
			// (E000 {0A6D,022F,023F}, D800 3ED6, C800+03E 0000) and C800[0] is the chunk ladder, not
			// a sector.  By elimination the hunt state is retained - and retention is CORRECT: the
			// previous run ended at sector 4 and this command starts at block 36 = sector 5, i.e.
			// exactly where the run left off.  Zeroing here disabled the match entirely, so every
			// arriving sector was accepted regardless of its ID.
			if (m_desc_n)
			{
				m_want_r = 0;
				for (int i = 2; i < m_desc_n; i++)
					if (m_desc_val[i-2] >= 0x3e && m_desc_val[i-1] >= 0x3e && m_desc_val[i] > 0 && m_desc_val[i] < 0x3e)
					{ m_want_r = m_desc_val[i]; break; }
			}
		logerror("PROG n=%u %s| e000: %04x %04x %04x %04x t=%.5f\n",
				m_desc_n, pb, m_ch[0], m_ch[1], m_ch[2], m_ch[3], machine().time().as_double());
		logerror("PROG start sector = %u%s\n", m_want_r,
				m_desc_n ? "" : "  (RETAINED - program reused)");
		}
		// cont.517: carry the previous command's LAST chunk across the boundary.  C800[0] is armed only
		// in response to a data-done, so at the first record of a run it still points at wherever the
		// previous command finished - valid, in range, and not ours.  Latching C800[0] itself does not
		// work: measured, it holds a transient 0x0028 at this point, not a chunk.
		m_prev_last_chunk = m_cur_last_chunk;
		m_cur_last_chunk = 0;
		m_first_chunk = 0;
		m_host_dma_n = 0;
		m_cmd_records = 0;
	m_bound_logged = false;
	m_extra_done = false;
	m_extra_active = false;
	}
	// The gate array holds the whole track in its bit buffer (capture_track - detection-is-capture, the
	// AM2147 + 1801 preamble search absorb positional slop), so the FIRST record of a read is available
	// at once rather than a sector-period later.  Do not deliver from here - that would land inside
	// op18's own block-copy loop before it returns; arm the time and let the pump release it.
	//
	// ONLY on the first arm.  Every per-record ISR re-issues the program's first command (E000 <= $0a6d,
	// bit11 set), which re-enters this routine; resetting the deadline there would destroy the
	// rotational spacing and free-run the phase machine (measured: 200us per phase instead of
	// 1.875/8.75ms, a 64us IRQ6/IRQ5 runaway with no data-done).
	if (first_arm)
		m_next_rec = machine().time();
	m_pump->adjust(attotime::from_usec(100));
}

// Deliver the held mark's interrupt IFF the firmware has armed (a bit-change PRIOR to the interrupt).
// arm + held-mark -> exactly one IRQ; consumes both, so only one interrupt is ever live at a time.
// Stage the record the firmware will verify.  Extracted from deliver_mark() so the WRITE
// path can stage identically: the firmware polls this area and checks the MFM sync byte
// ($98BE `cmpi.b #$A1`), and destination is m_d800<<1 - $7DAC for a read, $7D9E for a write
// (the firmware points D800 itself at $97C8 from mode-word bit14).
void multibus_storager_device::stage_record(captured_sector const &s)
{
	address_space &cs = m_cpu->space(AS_PROGRAM);
	// STAGE-vs-ISR ADDRESS CHECK.  The ISR reads from wherever the firmware pointed D800; we write
	// to m_d800<<1 at stage time.  If those differ on the FAIL cases the bug is a D800 re-point
	// between stage and ISR (the retry path at $97A6 re-arms staging every attempt); if they agree
	// and the bytes are A1 yet the ISR still reads 00, something clears the buffer in between.
	// One run decides which - do NOT add a third staging site before this table exists.
	if (TRACE_GAWRITE && cmd_is_write())
		logerror("STAGE dst=%04x (d800=%04x) sector r=%02x %s t=%.5f\n",
			u32(m_d800) << 1, m_d800, s.r, flux_density_fm() ? "FM" : "MFM",
			machine().time().as_double());
	// cont.569 (TEMP): WHICH BUFFER DID WE STAGE INTO?  The 1.81s mid-command hang on the failing
	// psec=2092 attempt showed `staged@7dac` on the FIRST record while every healthy record stages
	// at `7d9e`.  $7DAC is the READ base (d800=3ed6); $7D9E is the write base (d800=3ecf), which
	// the firmware only selects later at $97C8.  So a first record staged before that re-point
	// lands in the read buffer and the firmware waits on a write buffer we never filled.  Log the
	// buffer identity on every write stage so the fail/heal cases are separable in one run.
	if (cmd_is_write())
	{
		u32 const d = u32(m_d800) << 1;
		logerror("STAGEBUF dst=%04x d800=%04x %s r=%02x done=%d/%d pc=%06x t=%.5f\n",
			d, m_d800,
			(d == 0x7dac) ? "READ-BASE(wrong for write)" : (d == 0x7d9e) ? "write-base" : "other",
			s.r, m_wr_done, m_sec_count, m_cpu->pcbase() & 0xffffff,
			machine().time().as_double());
	}
		bool const fm = flux_density_fm();

	u32 dst = u32(m_d800) << 1;
	if (dst < 0x4000 || dst + (fm ? 5 : 8) > 0x8000)
		dst = 0x7dac;                       // not yet latched (first record of a command)
	int k = 0;
	if (fm)
	{
		// FM record, five bytes.  The firmware's POSPTRs for this layout are C=+1 H=+2 R=+3
		// (measured from $2CF2's density-driven init), and cylinder 0 reads correctly with it.
		cs.write_byte((dst + k++) & 0xffff, 0xfe);       // +0  ID address mark (no A1 sync in FM)
		cs.write_byte((dst + k++) & 0xffff, s.c);        // +1  C
		cs.write_byte((dst + k++) & 0xffff, s.h);        // +2  H
		cs.write_byte((dst + k++) & 0xffff, logical_r(s.r));   // +3  R (firmware sector index, 1-based, base UIB[4])
		cs.write_byte((dst + k++) & 0xffff, s.nn);       // +4  N
	}
	else
	{
		// MFM record, TEN bytes - specified by two independent ROM sites (cont.448):
		//   the verify at $8A1A-$8A2C: bytes 0-2 OR'd == $A1, byte 3 == $FE, byte 4 == $FF
		//   the hardcoded POSPTRs at $2DB4: C=+5, H=+7, R=+8 - which only fit this layout
		// The model previously wrote the EIGHT-byte A1 A1 A1 FE C H R N, so byte 4 held the
		// cylinder ($01 on cyl 1) and $8A2C rejected every MFM record.
		// Cylinder is TWO bytes, HIGH at +5 and LOW at +6: $7C56 move.b (A0)+,D0 / $7C62
		// asl.w #8,D0 / $7C64 or.b (A0),D0 shifts the FIRST byte up.  Cylinder 1 is 00 01.
		// The media's ID field carries a single C byte, so the high half is always zero here.
		// OPEN: what the $FF at +4 IS.  It is not a cylinder high byte (that is +5, and would be
		// $00).  Most likely gate-array-supplied status - a validity/no-defect flag - in which
		// case writing it unconditionally encodes "always good" and a bad record should carry
		// something else.  Written as a constant only because its semantics are undecoded.
		// REVERTED cont.449: the ten-byte layout is refuted by measurement.  It took the MFM
		// C800 arms from 4 to 0, and the LIVE POSPTRs contradict it: at the MFM program load
		// $2CF2 writes C=$7DB0 H=$7DB1 R=$7DB2, i.e. +4/+5/+6, which fits THIS eight-byte record.
		// The $2DB4 hardcoded set (C=+5 H=+7 R=+8) that motivated the ten-byte reading is a
		// DIFFERENT initialisation and is not the one in use.
		// RESOLVED cont.487: $8A2C is a DISCRIMINATOR, not a layout assertion.  Its non-match path
		// is the NORMAL continuation, not an error:
		//   $8A20 cmpi.b #$a1,D0 (3 sync bytes OR'd) / bne $8A38
		//   $8A26 cmpi.b #$fe,(A0)+ (+3)             / bne $8A38
		//   $8A2C cmpi.b #$ff,(A0)  (+4)             / bne $8A38
		//   match   -> subq.w #1,$7A0C   (count this record type)
		//   NO match-> clr.w $7950 ; subq.w #1,$79A4 ; bne $8AB8   (carry on)
		// So the routine COUNTS records bearing A1/FE/FF, and an ordinary ID record failing that
		// test is the expected case - $FF at +4 is a sentinel marking a special record, the same
		// $ff-as-marker idiom as the $7654 want-list.  It therefore does NOT contradict $2CF2's
		// POSPTRs (C=+4 H=+5 R=+6), which name the fields of a normal ID record.  The two
		// describe different things and both hold.
		for (int p = 0; p < 3; p++)
			cs.write_byte((dst + k++) & 0xffff, 0xa1);   // +0..2  sync preamble
		cs.write_byte((dst + k++) & 0xffff, 0xfe);       // +3  ID address mark
		// REVERTED cont.451: presenting the AM-count signature ($FF at +4) for the first three
		// marks and the ID layout thereafter took MFM C800 arms 4 -> 0 - the same failure as the
		// ten-byte layout, and for the same reason: inserting a byte at +4 shifts C/H/R by one
		// and breaks the compare path, which satisfying the count does not compensate for.
		// So the signature and the ID field are NOT phase-separable by a simple mark counter.
		// They genuinely conflict at +4 on one buffer, and the resolution is not in this routine.
		cs.write_byte((dst + k++) & 0xffff, s.c);        // +4  C
		cs.write_byte((dst + k++) & 0xffff, s.h);        // +5  H
		cs.write_byte((dst + k++) & 0xffff, logical_r(s.r));   // +6  R (firmware sector index, 1-based, base UIB[4])
		cs.write_byte((dst + k++) & 0xffff, s.nn);       // +7  N
		// 0x2012 operands: $7C52 reads [[UIB+$ca]] as C (1 or 2 bytes if UIB+$12 bit1),
		// compares to [$7438].  Log both sides for the first few ID marks of a command.
		if (m_id_cmp_n < 4)
		{
			m_id_cmp_n++;
			u16 const ub = (m_uib_base >= 0x4000 && m_uib_base < 0x8000) ? m_uib_base
				: cs.read_word(0x799a);
			u8 const u12 = (ub >= 0x4000 && ub < 0x7f00) ? cs.read_byte((ub + 0x12) & 0xffff) : 0xff;
			u16 const want = cs.read_word(0x7438);
			// What $7C52-$7C66 will form from the staged field (C at +4, next at +5 = H)
			u16 const capt1 = s.c;
			u16 const capt2 = u16((u16(s.c) << 8) | s.h);   // bit1 path: asl #8 | next
			u16 const capt = BIT(u12, 1) ? capt2 : capt1;
			logerror("ID-CMP #%d staged C=%u H=%u R=%u | want[$7438]=%04x [$7436]=%04x | UIB+$12=%02x bit1=%d -> capt %s=%04x %s t=%.5f\n",
				m_id_cmp_n, s.c, s.h, logical_r(s.r), want, cs.read_word(0x7436),
				u12, BIT(u12, 1),
				BIT(u12, 1) ? "C16" : "C8", capt,
				capt == want ? "MATCH" : "*** 0x2012 ***",
				machine().time().as_double());
		}
	}
}

void multibus_storager_device::deliver_mark()
{
	if (!m_armed || !m_mark_pending)
		return;
	// Do NOT deliver into a masked CPU: the firmware arms (bit11) from inside its masked ISR chain, so an
	// immediate deliver would land masked and be lost/merged.  Hold the mark; it is delivered from the pump
	// tick (or a later arm) once the CPU IPL drops below this level - i.e. the firmware is ready to service.
	if (((m_cpu->state_int(M68K_SR) >> 8) & 7) >= (unsigned)m_mark_pending)
		return;
	int const level = (m_mark_pending == 6) ? M68K_IRQ_6 : M68K_IRQ_5;
	m_armed = false;
	m_mark_pending = 0;
	m_cpu->set_input_line(level, HOLD_LINE);
}

// The gate array, armed by op18 with the field program + the commanded sector count, reads EXACTLY the
// RETIRED framing (cont.428): this was described as a counted operation that terminates after the
// commanded sectors.  It is not - op18 is count-invariant, its program describes the TRACK FORMAT, and
// the firmware stops the records by ceasing to re-arm E802 bit11.  The
// whole track is decoded up front (capture_track - detection-is-capture, the AM2147 bit-buffer + the
// 1801 preamble search absorb positional slop); this delivers the wanted run to the firmware lock-step
// with its per-record arm.  Per sector: IRQ6 (ID address mark, C/H/R/N staged in $7DAC where the
// node+$c8..$ce POSPTR verify reads it) then TWO IRQ5 - the data field armed, then the data field done
// (the firmware's $7BA8 setup and $8018 done handlers) = 8xIRQ6 + 16xIRQ5 for an 8-sector read.  Only
// one IRQ5 dead-ends every sector at $7BA8 so $8018 never runs.  When the count exhausts,
// capture stops; the firmware then launches the SRAM->host transfer whose channel-done IRQ4
// (run_channel_dma) is the operation-complete op42 waits on.
void multibus_storager_device::advance_read()
{
	// WHY DOES THE WRITE WALK STOP?  Commits rose from 1 to 1 after the count fix, so termination
	// was not the limiter - the cycle simply stops being driven.  Log every early return while a
	// write is in flight, with each gate separately, rather than guessing which one closed.
	if (TRACE_GAWRITE && cmd_is_write() && (m_mark_pending || (!m_read_active && !m_write_active)))
	{
		static int n = 0;
		if (++n <= 40)
			logerror("WRSTOP pend=%d armed=%d IPL=%d [7302]=%04x [7950]=%04x phase=%d done=%d t=%.5f\n",
				m_mark_pending, m_armed ? 1 : 0,
				int((m_cpu->state_int(M68K_SR) >> 8) & 7),
				m_cpu->space(AS_PROGRAM).read_word(0x7302),
				m_cpu->space(AS_PROGRAM).read_word(0x7950),
				m_sec_phase, m_wr_done, machine().time().as_double());
	}
	if (m_mark_pending || (!m_read_active && !m_write_active))
		return;                             // a record is held awaiting the arm, or the op is complete
	if (m_write_active)
	{
		// WRITE LOCATE PHASE.  Deliver the ID mark (IRQ6) exactly as a read does - that step is
		// measured-identical - and then STOP.  The write's data-phase cadence is NOT known: the
		// E000 write program's tail (090a 090b 091b) has no read counterpart and is undecoded, so
		// assuming the read's IRQ5/IRQ5 pair here would be invention.  What the firmware does on
		// receiving this IRQ6 is the measurement that decides the next step.
		if (m_sec_phase == 0 && m_track_n > 0)
		{
			// ONLY A MATCHING ID MAY PRODUCE A MARK.  MEASURED (cont.563, the run that finally
			// read the completion instead of hunting for a stall): a record outside the commanded
			// range was still given the full locate+data mark cadence and was declined only
			// afterwards, at the commit.  The firmware cannot see that decline - it counts the
			// marks it is handed.  Three skipped records consumed three of the command's eight
			// counts, so it posted COMPLETION (0x80) having stored 5 of 8 sectors, and the model's
			// own log agreed with it: `WORK=8 = COMMIT=5 + SKIP=3`.  That identity was read for
			// several runs as a consistency check passing; it was the defect.
			// The gate array signals on an ID MATCH, so a non-matching record must generate no
			// stimulus whatsoever - walk past it here, before staging, silently.
			{
				address_space &cs5 = m_cpu->space(AS_PROGRAM);
				u32 const spt5 = m_uib_base ? cs5.read_byte((m_uib_base + 1) & 0xffff) : 16;
				u32 const first5 = spt5 ? (m_wr_psec % spt5) + 1 : 1;
				u32 const last5  = first5 + m_sec_count - 1;
				for (int guard = 0; guard <= m_track_n * 2; guard++)
				{
					captured_sector const &c = m_track[m_sec_index % m_track_n];
					if (c.r >= first5 && c.r <= last5)
						break;
					if (TRACE_GAWRITE)
						logerror("WRPASS r=%02x not in %u..%u - no mark issued, walked=%d t=%.5f\n",
							c.r, first5, last5, m_wr_walked, machine().time().as_double());
					m_sec_index = (m_sec_index + 1) % std::max(1, m_track_n);
					m_wr_walked++;
				}
			}
			captured_sector const &s = m_track[m_sec_index % m_track_n];
			// STAGE THE RECORD, exactly as the read path does.  MEASURED (debugger, cont.562): the
			// firmware polls the staging area and verifies the MFM sync byte -
			//     $98B2: move.b (A3)+,D2 / ori #$80 / eor / move.b (A3)+,D1 / eor
			//     $98BE: cmpi.b #$A1,D2 / bne -> error
			// A read satisfies this because the gate array stages `A1 A1 A1 FE C H R N` for it
			// ([$7DAC] read back as A1A1).  A write stages NOTHING, so [$7D9E] read 0000 on all 180
			// polls, the compare failed, and after 5 retries the firmware posted $2029 ($979E).
			// The destination needs no special casing: it is `m_d800 << 1`, and $97C8 has already
			// pointed D800 at $7D9E for a write (mode word bit14) where a read gets $7DAC.
			stage_record(s);
			logerror("WRLOC ID r=%02x done=%d/%d staged@%04x V6[7300]=%04x V5[7302]=%04x [7304]=%04x [7950]=%04x t=%.5f\n",
				s.r, m_wr_done, m_sec_count, u32(m_d800) << 1,
				m_cpu->space(AS_PROGRAM).read_word(0x7300),
				m_cpu->space(AS_PROGRAM).read_word(0x7302),
				m_cpu->space(AS_PROGRAM).read_word(0x7304),
				m_cpu->space(AS_PROGRAM).read_word(0x7950),
				machine().time().as_double());
			m_mark_pending = 6;
			m_sec_phase = 1;
			// cont.569 DEADLOCK (measured inst11 WRGAP): after first sector, WRLOC sets pend=6
			// with armed=0 and then waits for E802 bit11.  During the 1.81s gap there are ZERO
			// E802 writes; firmware spins at $15xx/$22xx IPL=0 waiting for the mark.  Each side
			// waits for the other.  The read path gets a bit11 re-arm from the prior field's ISR
			// before the next ID; the write's first post-commit ID does not.  Deliver the prepared
			// mark - same as m_wr_final_pending force-arm, for every write locate.
			m_armed = true;
			m_next_rec = machine().time() + sector_period() * 15 / 100;
			return;
		}
		// MEASURED after the locate IRQ6: the firmware patches the E000 program
		// (0a6d/0a29/0b3b/032e into slots 0-3) and sets E802 bit15 - the same capture-arm edge the
		// read path uses.  So it IS waiting on a next-field interrupt.  Deliver the data-phase
		// mark as the read does and observe whether it then kicks a channel DMA (E800 bit12),
		// which is what would tell us the direction and where the data comes from.
		if (m_sec_phase == 1)
		{
			logerror("WRDATA arm mark (IRQ5) done=%d/%d t=%.5f\n",
				m_wr_done, m_sec_count, machine().time().as_double());
			// ONE ARM PER SECTOR.  MEASURED timeline: the firmware raises E802 bit11 once per
			// sector (edge -> armed=1), we consume it with the ID/data mark, and bit11 then stays
			// LOW - it never rises again for that sector.  The read's second IRQ5 ("data field
			// done") therefore has no arm behind it on a write: it sits pending forever and stalls
			// the walk, which is what produced 1 commit + ~5 marks against 101 $201C expiries.
			// So a write is IRQ6 (locate) + ONE IRQ5, then straight on to the next sector.
			// TWO IRQ5-CLASS EVENTS PER SECTOR.  $7302 holds $29EA, which IS the IRQ5 handler, and
			// it is an ALTERNATOR on $7950 bit0:
			//     bit was 0 -> BRA $8552   (re-arm: pulses E802 bit11)
			//     bit was 1 -> BNE $873E   (the work leg)
			// So the pair must BOTH run per sector.  Cutting to a single locate IRQ6 removed the
			// IRQ5 altogether - measured, the alternator then never executed at all (0 entries,
			// 0 $8552, 0 $873E), which is why no re-arm was ever produced.
			// The read uses the same table at $29C0 with its own pair ($7BA8 setup / $8018 done).
			m_mark_pending = 5;
			m_sec_phase = 2;
			// Second-sector data arm: first IRQ5 of a sector is usually re-armed by the ID ISR's
			// bit11 pulse; when that pulse is late/missing the same deadlock as WRLOC applies.
			// Force-arm the prepared mark (D800 same-value re-arm is a separate measured path).
			m_next_rec = machine().time() + sector_period() * 70 / 100;
			return;
		}
		if (m_sec_phase == 2)
		{
			// SECOND IRQ5, BEFORE THE RECORD RESTARTS.  $88D0 `clr.w $7950` resets the alternator
			// parity at every record start and then arms the ID hunt, so $29EA only alternates
			// WITHIN a record.  The read gets two entries between clears (setup $7BA8, done
			// $8018); our write was getting one, so parity was 0 on every entry and all three hits
			// took the re-arm leg ($8552) with the work leg ($873E) never running.
			// The first IRQ5 (parity 0) runs $8552, which issues the E802 bit11 re-arm - so the
			// arm for THIS second delivery is produced by the first one.
			// Final-cycle probe (inst4 write#1 vs #2): firmware posts 8080 @ $1a54 only when it
			// still has a live soft-timer freelist and the write alternator.  Snapshot the cells
			// that arm that path so a same-run compare does not need a debugger for [$7986].
			{
				address_space &cs = m_cpu->space(AS_PROGRAM);
				logerror("WRDATA2 second IRQ5 done=%d/%d final=%d [7302]=%04x [7950]=%04x "
					"[$7986]=%04x node+2=%02x t=%.5f\n",
					m_wr_done, m_sec_count, m_wr_final_pending ? 1 : 0,
					cs.read_word(0x7302), cs.read_word(0x7950), cs.read_word(0x7986),
					m_node_base ? cs.read_byte((m_node_base + 2) & 0xffff) : 0xff,
					machine().time().as_double());
			}
			m_mark_pending = 5;
			m_sec_phase = 3;
			// Second IRQ5: normally armed by first IRQ5's $8552 bit11 pulse.  Force so a missed
			// pulse cannot strand the work leg (and the final cycle that posts $1a54).
			m_next_rec = machine().time() + sector_period() * 15 / 100;
			return;
		}
		// MEASURED after the data-arm IRQ5: the firmware loops e802 22d3 -> bad3 (pc 009890/009812),
		// i.e. it re-arms and waits for a further mark - the same shape as the read's second IRQ5
		// ("data field done").  Deliver it and observe whether a DATA DMA then appears.
		if (m_sec_phase == 3)
		{
			// THE GATE ARRAY IS THE MOVER.  On a read the firmware commands and the gate array
			// bus-masters the bytes to the host; a write is the same engine in the other
			// direction, so the gate array sources from host memory and commits to the medium.
			// That is why no firmware-issued data DMA is observed: it is not the firmware's job.
			// Host source is the IOPB's own buffer pointer [0x0d-0x0f], captured at first-arm.
			// ONLY THE COMMANDED SECTORS.  The walk presents whatever ID rotates past; committing
			// on every one scribbles arbitrary sectors (measured: wrote r=0e when the command
			// wanted sectors 1-8).  The gate array knows the command, so derive the wanted range
			// from the IOPB the same way the boot ROM does - sector IDs here are track-relative
			// and 1-based - and map the host offset by the sector's position in the range, never
			// by an arrival counter.
			if (m_track_n > 0)
			{
				captured_sector const &tgt = m_track[m_sec_index % m_track_n];
				address_space &cs4 = m_cpu->space(AS_PROGRAM);
				u32 const spt = m_uib_base ? cs4.read_byte((m_uib_base + 1) & 0xffff) : 16;
				u32 const first = spt ? (m_wr_psec % spt) + 1 : 1;
				u32 const last  = first + m_sec_count - 1;
				if (tgt.r >= first && tgt.r <= last)
				{
					// Post-IRQ4 final cycle: marks only - do not re-commit (count already met).
					if (m_wr_final_pending || (m_wr_done >= m_sec_count && m_sec_count > 0))
					{
						logerror("WRFINAL no-commit r=%02x (done=%d/%d final_pend=%d) t=%.5f\n",
							tgt.r, m_wr_done, m_sec_count, m_wr_final_pending ? 1 : 0,
							machine().time().as_double());
					}
					else
					{
						u32 const len = tgt.len ? tgt.len : 256;
						u32 const idx = tgt.r - first;
						u32 const off = m_wr_host_buf + idx * len;
						std::vector<u8> src(len, 0);
						address_space &hs = m_bus->space(AS_PROGRAM);
						for (u32 k = 0; k < len; k++)
							src[k] = hs.read_byte((off + k) & 0xffffff);
						bool const ok = write_sector(tgt.r, src.data(), len);
						if (ok) m_wr_done++;          // COMMITS, not walked sectors - see below
						logerror("WRCOMMIT r=%02x (want %u..%u idx=%u) from host %06x len=%u -> %s "
							"done=%d/%d t=%.5f\n",
							tgt.r, first, last, idx, off, len, ok ? "OK" : "FAILED",
							m_wr_done, m_sec_count, machine().time().as_double());
					}
				}
				else
					// UNREACHABLE with WR_SKIP_SILENT: phase 0 walks past non-matching records
					// before any mark is issued.  If this fires, a record was marked and then
					// declined, which silently shortens the transfer - so say so loudly.
					logerror("*** WRSKIP AFTER MARK (steals a count) *** r=%02x not in commanded range %u..%u done=%d/%d walked=%d/%d t=%.5f\n",
						tgt.r, first, last, m_wr_done, m_sec_count, m_wr_walked, m_track_n * 2,
						machine().time().as_double());
			}
			// Historical string said "no second IRQ5"; that was wrong once WRDATA2 was restored.
			// This line is end-of-record only (phase 3 finished).  A zero-done walk with
			// WRCOMMIT ... FAILED is media/protect/locate, not a missing mark.
			logerror("WRNEXT sector done=%d/%d walked=%d t=%.5f\n",
				m_wr_done, m_sec_count, m_wr_walked, machine().time().as_double());
			m_sec_phase = 0;
			// DO NOT count walked sectors.  The walk starts wherever the head happens to be, so it
			// meets out-of-range sectors first and skips them; counting those toward m_sec_count
			// retired the command after N SECTORS PASSED rather than N WRITTEN.  Measured: only 1
			// commit and ~5 marks delivered per run against 101 $201C watchdog expiries - the
			// firmware was still waiting for sectors we had already stopped presenting.
			// m_wr_done is now incremented only on a successful commit (above).
			m_sec_index = (m_sec_index + 1) % std::max(1, m_track_n);
			m_wr_walked++;

			// Close path v6: mark cycle, then observe-only deferred re-check (never IRQ4 from here).
			//   1) count met → m_wr_final_pending, one more mark cycle
			//   2) that cycle's phase-3 → m_wr_complete + arm observe (~750us)
			// Seven healthy closes: fw posts 80 within 90–130us, often before WRDONE.  The spare-
			// track 4-sector write never posts and holds [$7986] live (733c) — model must deliver
			// the event that path waits on, not invent a channel-done.
			if (m_wr_final_pending)
			{
				m_status_armed = true;
				m_wr_complete = true;
				m_wr_final_pending = false;
				m_write_active = false;
				arm_wr_close_failsafe("final-cycle");
			}
			else if (m_wr_done >= m_sec_count && m_sec_count > 0)
			{
				m_wr_final_pending = true;
				m_armed = true;
				m_sec_phase = 0;
				{
					address_space &cs = m_cpu->space(AS_PROGRAM);
					logerror("WRFINAL pending %d/%d walked=%d - one mark cycle then deferred close "
						"[$7986]=%04x [7302]=%04x t=%.5f\n",
						m_wr_done, m_sec_count, m_wr_walked,
						cs.read_word(0x7986), cs.read_word(0x7302),
						machine().time().as_double());
				}
				// Do not wait solely on firmware bit11; schedule the final locate promptly.
				m_next_rec = machine().time() + sector_period() * 15 / 100;
				m_pump->adjust(attotime::from_usec(100));
				return;
			}
			else if (m_wr_walked > m_track_n * 2)
			{
				m_status_armed = true;
				m_wr_complete = true;
				m_write_active = false;
				logerror("WRWALKEND done=%d/%d walked=%d/%d reason=WALK-BOUND(model) t=%.5f\n",
					m_wr_done, m_sec_count, m_wr_walked, m_track_n * 2,
					machine().time().as_double());
				arm_wr_close_failsafe("walk-bound");
				logerror("WRDONE %d/%d (walk-bound force) t=%.5f\n",
					m_wr_done, m_sec_count, machine().time().as_double());
			}
			m_next_rec = machine().time() + sector_period() * 15 / 100;
			return;
		}
		logerror("WRLOC phase=%d unexpected - holding t=%.5f\n",
			m_sec_phase, machine().time().as_double());
		m_write_active = false;
		return;
	}
	// Hold marks until the drive reaches the command's STARTING cylinder, then release so a
	// multi-track walk (LOADER 53 blocks, superblock 32) can cross heads/cylinders.  Gating for
	// the whole command on m_cmd_cyl starved the LOADER at the cyl1→2 boundary (0x201C).
	// Presenting IDs from the pre-seek capture while [$7438] still held the previous command's
	// target is what produced 0x2012 on psec=240 (capt C=7 vs want 5).  Gate on IOPB address
	// geometry, not [$7438]: $6AD0 updates the verify target only AFTER the seek pulse loop.
	if (m_cmd_cyl != 0xffff)
	{
		floppy_image_device *const fgate = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
		if (!fgate || fgate->get_cyl() != int(m_cmd_cyl))
		{
			m_next_rec = machine().time() + sector_period();
			return;
		}
		logerror("CYL GATE open: drive on cmd_cyl=%u, releasing for multi-track t=%.5f\n",
			m_cmd_cyl, machine().time().as_double());
		m_cmd_cyl = 0xffff;
	}
	// V1/V3: a finite field program stops raising marks once the commanded count is done, even if
	// the firmware keeps re-arming (measured: it does).  That is a GA response to the loaded program
	// + count, not a firmware-RAM poke.
	// The count is the gate array's OWN: its op18 program's port-$3F push count.  It must not be the
	// firmware's stake tally - the gate array cannot see how many sectors the firmware chose to stake,
	// and snooping that is the shape the LLE mandate rules out.  (cont.428: m_accepted_n also
	// over-reported badly - the drain's $6FF0 consume-mark writes the same $c0 the tap counted, 34
	// against 14 real stakes - so the disarm fired at ~40% of the commanded sectors, stopping the marks
	// while a chunk was armed with [$741c]=1, which is exactly the state $7B1C's 1.305s watchdog exists
	// to catch: over-count -> early disarm -> armed chunk never fills -> $201C -> host 0x82/$1C.)
	// NO self-disarm.  Every counted stop needed a number the gate array cannot legitimately have:
	// a stake tally is a firmware-RAM snoop, a raw delivered count starves the read (delivered includes
	// records the firmware REJECTED at $7DA2 -> $88AC), and op18 is count-invariant anyway - its program
	// describes the TRACK FORMAT, not the transfer, so there is nothing to count down.  The gate array
	// raises marks as the disk turns; the firmware stops them by ceasing to re-arm E802 bit11.
	// The three counted variants and their measurements are archival - see board.yaml counted_stop_open
	// and the cont.428 commit.  Do not reintroduce one without reading those first.
	if (PER_ADDRESS_MARK)
	{
		// The disk keeps turning and every address mark that passes the head raises a record.  The gate
		// array cannot know which sector is wanted - it has no header comparator; the firmware compares
		// the captured field itself ($89F2) and simply stops re-arming when it is done.  So wrap, and
		// let the arm/hold handshake be what ends the run.
		if (m_sec_index >= m_track_n)
			m_sec_index = 0;
	}
	else if (m_data_done_n >= m_sec_count && m_sec_count > 0)
	{
		m_read_active = false;              // window closed
		return;
	}
	// cont.523: an N-sector IOCB reads N sectors.  The firmware arms once MORE after the last one
	// (it always arms for the next record), and honouring that arm delivered a 5th sector for a
	// 4-sector command - which then advanced the retained sector hunt past where the following
	// command resumes.  m_data_done_n cannot bound this: it is never reset per command despite its
	// declaration, and the count-based close at PER_ADDRESS_MARK's else branch is dead code.
	// cont.523: BOUND.  With the count LATCHED on the firmware's write (see the cnt7abc tap) the
	// bound fires exactly right for the per-track remainder:
	//     recs=8 sec_count=8 | recs=4 sec_count=4 | recs=12 sec_count=12
	// Multi-track continuation is the firmware's job via PROG re-arm (which resets m_cmd_records
	// in start_field_program).  A model-side "continue while blocks_left > 0" was tried and
	// REGRESSED the LOADER load (free-running marks, done_n into the hundreds, host address stuck).
	// Do not re-open without a working-path control that still reaches Boot: sa(22,0)sinix.
	if (m_sec_count > 0 && m_cmd_records >= m_sec_count)
	{
		// THE TRAILING RECORD MUST ACTUALLY BE PRESENTED.  The first cut restored the hunt on the
		// next pump entry, which arrives long before PHYSICAL_TIMING lets the field come round - so
		// EXTRA -> EXTRA done took 18us against a ~12.8ms sector period and nothing was ever
		// delivered.  Nor can m_host_dma_n be the detector: the record count increments at
		// data-done but its DMA lands ~240us later, so there is ALWAYS one transfer outstanding
		// at the bound and the previous record's own write reads as the trailing one.  Count
		// completed record CYCLES after the offer, with a two-revolution deadline because the
		// hunt may have to wait for R+1 to come round.
		if (m_extra_active)
		{
			bool const landed = (m_extra_presented != 0);
			if (landed || machine().time() >= m_extra_deadline)
			{
				m_extra_active = false;
				m_sec_index = m_extra_sec_index;
				m_want_r = m_extra_want_r;
				m_blocks_left = m_extra_blocks_left;
				logerror("EXTRA done (%s): hunt restored index=%d want_r=%u | [$741c]=%04x "
					"[$7968]=%04x recs=%d/%d t=%.5f\n",
					landed ? "delivered" : "TIMED OUT - never presented",
					m_sec_index, m_want_r,
					m_lram[(0x741c - 0x4000) >> 1], m_lram[(0x7968 - 0x4000) >> 1],
					m_cmd_records, m_sec_count, machine().time().as_double());
				m_read_active = false;
				return;
			}
			// still waiting for the field to come round - keep presenting
		}
		else
		{
		if (!m_bound_logged)
			{
				m_bound_logged = true;
				address_space &bc = m_cpu->space(AS_PROGRAM);
				logerror("BOUND recs=%d/%d blocks_left=%d armed=%d next r=%02x | [$741c]=%04x [$7968]=%04x "
					"[$7a36]=%04x | dma_n=%u buf=%06x+%u t=%.5f\n",
					m_cmd_records, m_sec_count, m_blocks_left, m_armed ? 1 : 0,
					(m_sec_index >= 0 && m_sec_index < m_track_n) ? m_track[m_sec_index].r : 0xff,
					bc.read_word(0x741c), bc.read_word(0x7968), bc.read_word(0x7a36),
					m_host_dma_n, m_cmd_buf, m_cmd_bytes, machine().time().as_double());
			}
			m_read_active = false;
			return;
		}
	}
	if (machine().time() < m_next_rec)
		return;                             // this field has not passed the head yet
	captured_sector const &s = m_track[m_sec_index];   // physical order = ascending R for the boot read
	// cont.521: the gate array HUNTS for the programmed sector.  A record that is not the one the
	// firmware asked for must not be presented at all - presenting it drives the arm pipeline with
	// sectors nobody requested, so the first WANTED record arrives with the arm still pointing at
	// the previous command's chunk.  The disk keeps turning; skip to the next field and let the
	// wanted one come round.
	if (m_want_r && logical_r(s.r) != m_want_r)
	{
		m_sec_index++;
		if (m_sec_index >= m_track_n) m_sec_index = 0;
		m_sec_phase = 0;
		m_next_rec = machine().time() + sector_period();
		return;
	}
	address_space &cs = m_cpu->space(AS_PROGRAM);
	// The gate array runs op18's field sequence autonomously.  Per sector it raises the ID address mark
	// (IRQ6 -> $298C -> $89F2 verify, C/H/R staged at $7DAC) then two data-field events (IRQ5 -> $29C0):
	// #1 -> $7BA8 setup (arms the chunk, leaves alternator bit0=1), #2 -> $8018 done (stamps $74C4+2=0x40
	// READY, stakes the ledger).  Measured routing per sector = VERIFY x2 / SETUP / DONE (alternator 1,0,1,0),
	// an even parity cycle; the second verify arrives via the IRQ6 HOLD_LINE re-fire (making it explicit here
	// via a second pump-delivered IRQ6 mis-times the transfer cadence and regresses to zero transfers).
	if (m_sec_phase == 0)
	{
		// Perform any deferred track-boundary head switch HERE - the previous sector has
		// finished with the old capture, so replacing it is now safe.
		if (m_walk_pending)
		{
			m_walk_pending = false;
			m_walk_head ^= 1;
			capture_track();
			logerror("WALK (deferred) -> head=%d t=%.5f\n", m_walk_head, machine().time().as_double());
		}
		// Deposit the RAW RECOVERED ID FIELD at the address the firmware COMMANDED through the D800
		// latch - the gate array writes where it was told, it does not know about $7DAC.  The layout is
		// exactly what came off the surface, so it is density-dependent, and the firmware's readers
		// expect precisely that: $9884 (the real sector-select compare) combines bytes 0-2 to $A1,
		// requires $FE at byte 3 and takes the cylinder from byte 4(+5), gated on UIB+$12 bit2; the FM
		// reader $7C7A instead expects $FE at byte 0, because FM HAS NO $A1 SYNC BYTES.
		stage_record(s);
		m_mark_pending = 6;
		m_sec_phase = 1;
		m_next_rec = machine().time() + sector_period() * 15 / 100;   // ID -> gap -> data field
	}
	else if (m_sec_phase == 1)
	{
		m_mark_pending = 5;                     // data field armed (IRQ5 #1 -> $7BA8 setup)
		m_sec_phase = 2;
		m_next_rec = machine().time() + sector_period() * 70 / 100;   // the data field passing
	}
	else
	{
		// The setup handler this record's arm ran ($7BA8) has just programmed the destination chunk into
		// C800[0] - the live SRAM chunk word pointer, = [$741e].  The gate array deserialises the data field
		// into exactly the chunk it was COMMANDED with, so the model must use that address rather than one of
		// its own: staging at a self-computed 0x4000 + index*len puts each sector a record ahead of the
		// firmware's pointer, so its per-sector DMA sources the previous chunk, the first chunk is sent twice
		// and the last sector is never transferred at all.
		u32 const dst = u32(m_c800[0]) << 1;
		// TEMP (STRIP): correlate the sector the FIRMWARE accepted ([$7428], written only at $992E in
		// the ID-match routine) against the chunk it is actually landing in.  Both are model-side reads
		// at a point we control - no prefetch semantics involved.
		// The FIRST data field of a run is structurally discarded: the firmware arms C800[0] only in
		// RESPONSE to the data-done record ($8018), so field 0 can have no destination, and withholding
		// its record to wait for one deadlocks (measured: 9293 attempts, never armed).  The field is
		// genuinely lost - it arms the pipeline - and the wanted data must be caught on a later pass.
		// TEMP cont.437: does the firmware choose the chunk by the recovered sector IDENTITY or by
		// arrival order?  Log R against the chunk it commanded; the host address is logged at the
		// channel DMA, so R -> chunk -> host can be matched end to end.
		// cont.457: log the REQUESTED range beside the presented identity.  Without it a total
		// non-match reads as "matched nothing"; with it, "requested 32..35, presented 0..15" is
		// visible at a glance - two index SPACES, not a bad comparison.  This is the line that
		// would have caught the linear-vs-track-relative mismatch on the first run, not the third.
		{
			address_space &nc = m_cpu->space(AS_PROGRAM);
			u32 const nd = m_node_base ? m_node_base : 0x71f0;
			u8 const req_start = nc.read_byte((nd + 0x09) & 0xffff);
			u16 const req_cnt  = (u16(nc.read_byte((nd + 0x0a) & 0xffff)) << 8)
			                   | nc.read_byte((nd + 0x0b) & 0xffff);
			u8 const req_head  = nc.read_byte((nd + 0x05) & 0xffff);
			u16 const req_cyl  = (u16(nc.read_byte((nd + 0x06) & 0xffff)) << 8)
			                   | nc.read_byte((nd + 0x07) & 0xffff);
			m_last_r = s.r; m_last_presented = logical_r(s.r);
			logerror("FIELD r=%02x c=%02x h=%02x -> chunk %04x  (sec_index=%d, done_n=%d) t=%.5f"
				"  | REQ cyl=%u head=%u start=%u count=%u  [presented R=%u]"
				"  | c800=%04x prevlast=%04x stale=%d\n",
				s.r, s.c, s.h, dst, m_sec_index, m_data_done_n, machine().time().as_double(),
				req_cyl, req_head, req_start, req_cnt, logical_r(s.r),
				m_c800[0], m_prev_last_chunk, int(u32(m_c800[0])<<1 == m_prev_last_chunk));
		}
		// cont.516: DEPOSIT BY CLAIM, not by the published pointer.
		// C800[0] is armed only in RESPONSE to a data-done, so the FIRST wanted record of a run
		// arrives before its own arm and is deposited into whatever chunk the PREVIOUS command
		// left published.  The firmware's own claim table ($7696, 6-byte entries: +0 chunk BYTE
		// address, +2 claimed sector index) is what the host DMA sources from, so a record whose
		// claim already exists belongs at the CLAIMED chunk.
		// MEASURED (mx2-001.imd, cyl1 h0): R=1 carries the boot header 0b 01 00 00; without this
		// it lands in chunk 4580 while its claim says 4b00, and the CPUAP reads A0 filler at
		// buffer+0 - the "sector 1 lost" defect.  Records 2..n are unaffected: by then the
		// allocator has run and C800[0] already agrees with the claim.
		// The firmware divides the table's +0 by two into [$741e] ($007AFC asr.w #1), so +0 is a
		// byte address and C800[0] is a word address - hence the <<1 above and none here.
		u32 dep = dst;
		// DEPOSIT_BY_CLAIM - REFUTED - body removed; the code is preserved verbatim in commit bebdc9428d9
		// (pre-cleanup checkpoint).  Kept as a comment so the verdict survives without
		// dead machinery that reads as live code.
		// cont.517: a chunk is only OURS once the firmware has armed C800[0] for THIS command.  Until
		// then it still holds the previous command's last chunk - valid, in range, and wrong.  Holding
		// on "out of range" alone works on the FM read (C800[0] genuinely unset) and fails on the MFM
		// read (stale 4580 passes the range test), which is why sector 1 - the boot header - is lost.
		// The firmware arms for the NEXT record, so the first wanted record of a run has no arm of
		// its own: the arm does not advance until AFTER it arrives.  Detect that directly - the
		// chunk is the same one the previous record used - and hold rather than overwrite it.
		// cont.518: THE DISK NEVER STOPS TURNING.  A wanted sector arriving before its arm is not
		// lost - it comes round again next revolution.  The firmware arms C800[0] only in RESPONSE
		// to a data-done, so the first record of a run necessarily precedes its own arm; depositing
		// it anyway puts it in the PREVIOUS command's chunk, which the host DMA never reads
		// (measured: cyl1 R=1, the boot header, into 4580 while host position 0 sources 4b00).
		// So deposit ONLY into a chunk armed since the last deposit; otherwise let the record pass
		// - the firmware still sees it and arms from it - and take the sector on the next pass.
		// A skipped record is NOT a completed sector and must not count toward the run.
		bool const fresh_arm = true;   // FLUSH_ON_CLAIM - REFUTED
		bool deposited = false;
		// DEPOSIT_BY_CLAIM - REFUTED.  Body removed; preserved verbatim in commit bebdc9428d9
		// (pre-cleanup checkpoint).  It was the `if` arm of this if/else, so the surviving arm
		// below is now the unconditional path.
		if (fresh_arm && dep >= 0x4000 && dep + s.len <= 0x8000)
		{
			for (int k = 0; k < s.len; k++)
				cs.write_byte((dep + k) & 0xffff, s.data[k]);
			m_cur_last_chunk = dep;
			if (!m_first_chunk) m_first_chunk = dep;
			deposited = true;
		}
		else
		{
			// The FIRST field of a run has no destination yet: the firmware arms C800[0] in RESPONSE
			// to a data-done, so field 1 arrives before any arm.  Discarding it loses sector 1 - the
			// firmware has already armed a chunk for it and ships that chunk empty, so host position 0
			// receives stale content while sectors 2..8 land correctly at positions 1..7.
			// Do not withhold the RECORD (that deadlocks - the arm never comes); hold the DATA and
			// flush it as soon as a chunk is armed. (cont.437)
			m_held_len = s.len;
			m_held_r = logical_r(s.r);
			std::copy_n(s.data, s.len, m_held_data);
			// The claim may ALREADY be in the table when the record arrives (it is not always
			// written afterwards), so try immediately as well as on a later claim write.
			// FLUSH_ON_CLAIM - REFUTED (called flush_held_on_claim() here); see bebdc9428d9
			logerror("FIELD r=%02x held - no chunk armed yet t=%.5f\n",
				s.r, machine().time().as_double());
		}
		m_mark_pending = 5;                     // data field captured (IRQ5 #2 -> $8018 done)
		m_sec_phase = 0;
		// cont.531: count DELIVERED sectors, not presented records.  Counting presentations lets a
		// command whose records never reach a chunk still reach its count and post 0x80 - a hollow
		// completion, which is a worse diagnostic state than the 0x82 it replaced because it reports
		// success for a transfer that moved nothing.
		if (deposited)
			m_cmd_records++;
		if (m_extra_active)
			m_extra_presented++;   // a record cycle completed AFTER the offer
		if (m_blocks_left > 0) m_blocks_left--;   // cont.525: one block of the linear run
		m_sec_index++;                          // one sector completed after its data-done record
		if (m_want_r)        // hunt the next sector in the run
		{
			m_want_r = u16(m_want_r % std::max(1, m_track_n) + 1);
			// cont.525: wrapping to sector 1 IS the track boundary.  The firmware does not seek here
			// (measured), so the gate array walks: head 0 -> head 1 -> step to the next cylinder.
			if (m_want_r == 1 && m_blocks_left > 0)
			{
				floppy_image_device *const fw = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
				if (fw)
				{
					// cont.532: the model must supply ONLY the head.  The FIRMWARE steps the drive
					// itself - measured: dozens of `FWSTEP dir=0` pulses through the read - so the
					// model's own step was ADDITIVE, and the sum produced the impossible trail
					// cyl1h0 -> cyl1h1 -> cyl2h1 (a cylinder advance with the head still 1, which
					// this branch alone cannot generate).  The head is the part the firmware never
					// moves: m_sel_head reads 0 for the whole command.
					// DEFER the switch: the last sector of the track may still be in flight.
					// Toggling the head and re-capturing HERE replaces m_track[] while sector 16's
					// data is still being copied out of it, so that sector is delivered from the
					// OTHER SURFACE.  Measured end-to-end against the IMD: 98/101 sectors correct,
					// and all 3 failures were r=16 carrying the other head's bytes - cyl1 h0 and h1
					// had exactly each other's checksums.  That is also the source of the
					// non-determinism the loader saw (superblock rejected, then accepted).
					// Mark it pending and perform it when the NEXT record is staged.
					m_walk_pending = true;
					// NOTE: writing the cached head at UIB+$d0 from here REGRESSED the boot
					// (88 -> 28 deliveries) and is reverted.  The firmware compares that field
					// with `cmp.b ($d0,A4),D1` at $21E8 - it is a BYTE - and a word write also
					// clobbers +$d1.  Re-derive the exact field width and meaning before trying
					// again; the read at cont.562 showed the WORD at +$d0 as 0001 while the head
					// was 1, so the head may live in +$d1 rather than +$d0.
					capture_track();
					// REFUTED, do not reinstate: `m_sec_count += min(m_blocks_left, m_track_n)` here
					// dropped deliveries 88 -> 48.  The BOOT's reads cross this same boundary and
					// their completion depends on the count as it stands, so extending it here
					// breaks the working path.  The two-track bound has to come from somewhere that
					// does not perturb a single-track run - probably the exhaust test itself
					// (gating on m_blocks_left) rather than the count.
					logerror("WALK -> head=%d cyl=%d blocks_left=%d sec_count=%d t=%.5f\n", m_walk_head,
						fw->get_cyl(), m_blocks_left, m_sec_count, machine().time().as_double());
				}
			}
		}
		m_next_rec = machine().time() + sector_period() * 15 / 100;   // trailing gap -> next ID
		// WAIT_FOR_FRESH_ARM - REFUTED (decremented m_data_done_n here); see bebdc9428d9
		// TESTED AND REFUTED as the track-2 terminator: gating this on `&& m_blocks_left <= 0`
		// changed NOTHING (88 deliveries either way).  The count exhaust is NOT what ends the
		// second track's run - look elsewhere.
		if (++m_data_done_n == m_sec_count)
		{
			// COUNT EXHAUST (model observation only).  Do NOT write firmware RAM here.
			// A prior experiment OR'd UIB+$12 bit7 (op42 guard bypass at $6C32) and deposited a
			// phase byte at node+$26 from the pump.  Both are RETIRED (cont.426): with the SRAM byte
			// order corrected the firmware's own word writes satisfy those gates, and the read
			// completes and posts 0x80 with neither deposit.  The old note here - "even when op42
			// times out the host still never sees 0x80" - was measured through the inverted byte
			// mapping and is FALSE; op42 completes in 0.5ms and the host sees 0x80.
			m_status_armed = true;
			logerror("GA count-exhaust n=%d t=%.4f\n",
				m_data_done_n, machine().time().as_double());

		}
	}
	deliver_mark();
}

// The mark clock: stage/deliver the next record while the window is armed, then
// reschedule.  advance_read() holds one record at a time and stops after the commanded count; deliver_mark
// releases it once the firmware has armed and the CPU is unmasked.
TIMER_CALLBACK_MEMBER(multibus_storager_device::pump_tick)
{
	// cont.569: PC sample inside the post-WRLOC wait.  phase==1 + mark_pending IRQ6 + !armed means
	// the model has staged the next ID and is holding IRQ6 until a re-arm (bit11 or D800 same-
	// value).  Cap samples so a healthy run stays quiet; the 1.81s gap produces many ticks.
	{
		static int s_gap_n = 0;
		static double s_gap_t0 = -1.0;
		bool const in_gap = (m_write_active && m_sec_phase == 1 && m_mark_pending == 6 && !m_armed);
		if (in_gap)
		{
			double const t = machine().time().as_double();
			if (s_gap_n == 0)
				s_gap_t0 = t;
			// ~every 50ms (pump is 200us) → every 250th tick, plus the first few
			if (s_gap_n < 6 || (s_gap_n % 250) == 0)
			{
				address_space &cs = m_cpu->space(AS_PROGRAM);
				logerror("WRGAP wait IRQ6 armed=0 phase=1 done=%d/%d pc=%06x IPL=%d "
					"[7302]=%04x [7950]=%04x [$7986]=%04x d800=%04x dt=%.5f t=%.5f\n",
					m_wr_done, m_sec_count, m_cpu->pcbase() & 0xffffff,
					int((m_cpu->state_int(M68K_SR) >> 8) & 7),
					cs.read_word(0x7302), cs.read_word(0x7950), cs.read_word(0x7986),
					m_d800, t - s_gap_t0, t);
			}
			s_gap_n++;
		}
		else if (s_gap_n > 0)
		{
			logerror("WRGAP end after %d samples dt=%.5f done=%d/%d phase=%d pend=%d armed=%d t=%.5f\n",
				s_gap_n, machine().time().as_double() - s_gap_t0,
				m_wr_done, m_sec_count, m_sec_phase, m_mark_pending, m_armed ? 1 : 0,
				machine().time().as_double());
			s_gap_n = 0;
			s_gap_t0 = -1.0;
		}
	}
	deliver_mark();
	if (m_read_window)
		advance_read();
	// Q3b: deposit the end marker once the firmware has signalled it bootstrapped.  Done HERE, from the
	// device's own context: currently_executing() is not the local CPU, so the model's $4000-$7FFF snoop
	// taps ignore it and m_term_bit0 / m_last_bw stay clean.  From the next record onward the firmware
	// tests it at $7E9A (the [$7968]!=0 arm) and terminates via $7ED8.
	if (false && m_aa_armed && m_aa_cell >= 0x7654 && m_aa_cell <= 0x76bf)
	{
		address_space &cs2 = m_cpu->space(AS_PROGRAM);
		if (!m_aa_done)
		{
			cs2.write_byte(m_aa_cell & 0xffff, 0xaa);
			m_aa_done = true;
			m_aa_armed = false;
			logerror("Q3D %10.1fus $AA -> [%04x] (bus-master) [7968]=%04x remain=%04x\n",
				machine().time().as_double() * 1e6, m_aa_cell,
				cs2.read_word(0x7968), cs2.read_word(0x7956));
		}
	}

	// Intentionally NO firmware-RAM completion deposit here (see count-exhaust note above).
	// Log once when count has exhausted and the walker has parked, so we can see the window
	// without inventing the byte the watch pump supposedly needs.
	// The phase byte is not a completion signal - it is the gate array's "service me" handshake.  The
	// walker parks by writing the phase as a WORD ($15A0), which leaves $00 in the HIGH half, and the
	// watch pump gates on a BYTE read of exactly that half ($162A).  Supply it at EVERY park while a
	// capture is live, not once at the end: the pump is what dispatches $7964, whose $32AC query sets
	// [$7968] ("a record is pending"), and [$7968] is what makes op4A DEFER its drain ($6EE2) instead
	// of running it inline against an empty ledger.  The deferred drain then runs from the ISR once
	// records exist, finds [$74b4] non-empty, and launches at $70F4 - no kick involved.
	if (!SRAM_BYTE_SWAPPED && (m_status_armed || m_read_active))
	{
		u32 const node = m_node_base;
		if (node >= 0x4000 && node + 0x26 < 0x8000)
		{
			address_space &cs = m_cpu->space(AS_PROGRAM);
			if (cs.read_word((node + 0x26) & 0xffff) == 0x000a)
			{
				// The walker parks by writing the phase as a WORD ($15A0 move.w #$a,($26,A2)), which
				// leaves $00 in the HIGH half - and the watch pump gates on a BYTE read of exactly that
				// half ($162A cmpi.b #$a,($26,A1)).  No firmware path can set it, so the gate array
				// must, after the park (the park would otherwise overwrite it).  Address derived from
				// the D000 latch; written from device context, so it is a bus-master cycle the model's
				// own $4000-$7FFF snoop ignores.
				cs.write_byte((node + 0x26) & 0xffff, 0x0a);
				bool const final = m_status_armed;
				m_status_armed = false;
				logerror("GA phase byte -> node+26 (%04x) node=%04x %s t=%.5f\n",
					(node + 0x26) & 0xffff, node, final ? "(count exhausted)" : "(capture live)",
					machine().time().as_double());
			}
		}
	}
	m_pump->adjust(attotime::from_usec(200));
}

// ---------------------------------------------------------------------------
// channel DMA (E800 bit12 kickoff)
// ---------------------------------------------------------------------------

// One host DMA between the local SRAM buffer and host memory via the C000 up-counter (spec §3.4/§6.4).
// Read (0x94/0x95): SRAM chunk -> host; write: host -> SRAM.  Completion raises IRQ4.
//
// FRONTIER: the transfer count register and multi-field streaming; the first cut moves one field
// (sector) and keys the source on the same SRAM chunk the flux capture staged into.
void multibus_storager_device::run_channel_dma()
{
	if (!m_c000_valid)
		return;
	address_space &cs = m_cpu->space(AS_PROGRAM);
	address_space &bs = m_bus->space(AS_PROGRAM);
	u16 const e800 = m_ch[(0xe800 - 0xe000) / 2];

	// The gate array's bus-master engine moves a block between the D000-addressed local work area and
	// the C000 host address, direction from E800 bits 14/13 (spec §3.2):
	//   bit14 set               = host -> local (the firmware reads the IOPB/UIB from host memory).
	//   bit13 set (bit14 clear) = local -> host (the firmware writes its node - carrying the STATUS/ERROR
	//                             bytes at +2/+3 - back to the host IOPB; this is the 2180 completion
	//                             handshake the CPUAP polls, spec §5.1).
	// The 0x87 INIT reads the per-unit UIB (host 0FE948 -> $6E60, the 0x20-byte geometry carrying the
	// FM/MFM density at UIB+$12); the IOPB node ($71F0) is 0x18 bytes.
	u32 const ld = u32(m_d000) << 1;
	bool const to_local = BIT(e800, 14);
	// cont.535: bit 13 REMOVED from the direction test.  Measured, two kicked transfers:
	//     working host xfer  e800=3a5d  0011 1010 0101 1101
	//     kernel read        e800=1a0d  0001 1010 0000 1101   -> bit14 CLEAR in both
	// Bit 14 alone is consistent with direction; requiring bit 13 as well is what rejected the
	// kernel transfers as "neither direction", so nothing shipped.  Bit 13 is the CONTROL-BLOCK TYPE
	// (node 0x18 vs UIB 0x20, used at the length line) and has no business qualifying direction -
	// the model was reading one bit two ways.
	bool const to_host  = !BIT(e800, 14);
	// A read data transfer sources the captured SRAM chunk ($8018 sets C800[0]=[$741e] = the chunk word
	// pointer), distinguishing it from the IOPB/UIB control blocks (which source the node work area).
	// Length comes from the sector ACTUALLY IN FLIGHT, not from index 0 of the last capture.  The old
	// form (m_track[0].len) was right only by coincidence - a track is re-captured per operation and
	// every sector on it shares a size - and its comment baked in "one FM sector".  It breaks on a
	// mixed-size track, and on a stale capture it would move an FM-sized 128 bytes for a 256-byte MFM
	// sector.  Measured cont.447: FM 128 / MFM 256 with track[0] and track[m_sec_index] agreeing, so
	// this is a latent fragility rather than the cause of the MFM drop pattern - fixed on its own
	// terms.  (The firmware's commanded size lives at [$7996] - $0080 FM / $0100 MFM - but that is
	// firmware RAM; the gate array's own decode is the correct source.)
	// cont.530: `to_host` REMOVED from the predicate.  It made a LOCAL data-chunk transfer classify
	// as a control block, and line 1439 then latched the chunk address as m_uib_base (measured:
	// uib=4300, which is the chunk r=05 landed in).  Density then came from a chunk buffer, the model
	// wrote the FM header layout on an MFM track, and the firmware read a stale sector ID forever.
	// Enumerated before changing: 11 consumers, 8 re-test the direction themselves so they are
	// unaffected; only 1418 (length), 1424 (a TEMP diagnostic) and 1439 (the bug) are bare.
	// 1503 is safe by construction - for to_host, old `to_host && X` == new `X`.
	// 1536 (`to_local && !is_data && ld == m_uib_base`) is the one to watch: a genuine to_local UIB
	// fetch could now classify as data.  VERIFIED by measurement that it still fires.
	// WHERE DOES A WRITE'S HOST->LOCAL DATA LAND?  The destination is the D000-latched local word
	// address; log it for write-class commands so write_sector()'s source is measured, not guessed.
	if (TRACE_GAWRITE && cmd_is_write())
		logerror("WRDMA e800=%04x %s ld=%04x c000=%06x d000=%04x sec_idx=%d track_n=%d t=%.5f\n",
			e800, BIT(e800, 14) ? "host->LOCAL" : "local->host", ld, m_c000, m_d000,
			m_sec_index, m_track_n, machine().time().as_double());
	bool const is_data = m_sec_count > 0 && ld >= 0x4000 && ld < 0x7000;   // the linear data buffer
	u32 data_len = m_track[0].len;
	if (m_sec_index >= 0 && m_sec_index < m_track_n)
		data_len = m_track[m_sec_index].len;
	u32 const len = is_data ? data_len : (BIT(e800, 13) ? 0x18 : 0x20);   // node = 0x18, UIB = 0x20
	// TEMP cont.447: is the transfer length coming from the wrong object?  m_track[0].len is sector
	// INDEX 0 of the LAST capture - not the sector being moved (m_sec_index), and not any count the
	// firmware programmed.  Compare against the sector actually in flight and the firmware's own
	// commanded sizes ([$7460] -> PIT0 counter 1 at $4504-$4518, and [$7996], the chunk-table stride
	// at $0ADC).
	if (is_data)
	{
		address_space &cs5 = m_cpu->space(AS_PROGRAM);
		int const si = (m_sec_index >= 0 && m_sec_index < m_track_n) ? m_sec_index : -1;
		logerror("XFERLEN len=%u | track[0].len=%u | track[%d].len=%s | [$7460]=%04x [$7996]=%04x\n",
			len, m_track[0].len, m_sec_index,
			(si >= 0) ? std::to_string(m_track[si].len).c_str() : "n/a",
			cs5.read_word(0x7460), cs5.read_word(0x7996));
	}
	// D000 is the GENERIC local-DMA address latch - it carries the node for one control block and the
	// UIB for the next (both loaded at $3D42).  The class is what distinguishes them, so latch the UIB
	// base here: it is where the operation-complete bit (UIB+$12 bit7) has to be deposited.
	// D000 is the GENERIC local-DMA latch - it carries the node for one control block and the UIB for
	// the next, and once the transfer is launched it carries data addresses too.  Latch each base by
	// its class (E800 bit13: set = node/0x18, clear = UIB/0x20) so later traffic cannot move them.
	if (!is_data && ld >= 0x4000 && ld < 0x8000)
	{
		if (BIT(e800, 13)) m_node_base = ld;
		else               m_uib_base  = ld;
	}
	// cont.522: the firmware's allocation index carries over between commands (entries are claimed
	// and never released - that release is the teardown that never runs on the read path), so the
	// scatter/gather entry for position 0 is whatever the previous command left off at (measured:
	// entry 11 = 4b00) while C800[0] still published the PREVIOUS stride's entry 11 (4580).  The
	// first record therefore lands one entry out.  Serve position 0 from where it actually went.
	if (to_host && is_data && m_host_dma_n == 0 && m_first_chunk && ld != m_first_chunk
		&& m_first_chunk + len <= 0x8000 && ld >= 0x4000 && ld + len <= 0x8000)
	{
		address_space &fc = m_cpu->space(AS_PROGRAM);
		for (u32 k = 0; k < len; k++)
			fc.write_byte((ld + k) & 0xffff, fc.read_byte((m_first_chunk + k) & 0xffff));
		logerror("POS0 FIXUP: served from chunk %04x into %04x t=%.5f\n",
			m_first_chunk, ld, machine().time().as_double());
	}
	if (to_host && is_data) m_host_dma_n++;
	// cont.534: the lower bound was `m_c000 >= 0x010000` - the STORAGER's own map, where the Multibus
	// window starts at 0x010000 because 0x0000-0xffff is its local ROM/RAM/IO.  But m_c000 is a HOST
	// address on the Multibus, and the CPUAP installs its RAM at 0x000000-0x0fffff.  So every transfer
	// to the low 64K of host memory was silently dropped.  The label/boot reads target 0x0FC0DD and
	// worked; the KERNEL LOAD targets 0x8000-0xB898 (console: text_addr=x8000 dat_addr=xA898) and was
	// rejected at this line - measured c000=00b300/00b400 with c000v=1.  That is why the kernel read
	// has never shipped a block, at any point, independently of every other change today.
	// Validity is tracked separately (m_c000_valid, printed as c000v), so use it instead of a magic
	// lower bound that conflated two address spaces.
	if ((to_local || to_host) && ld >= 0x4000 && ld + len <= 0x8000 && m_c000_valid && m_c000 < 0xff0000)
	{
		// Control blocks (local->host) byte-swap each 16-bit word: the 68000 holds the status as its BE low
		// byte (node+3) and the CPUAP polls IOPB+2, so the swap is what lands node+3 at host+2 (verified -
		// it is how the host ever sees 0x81).  DATA fields do NOT swap: the gate array presents the recovered
		// byte stream to the bus, and the host data buffer is byte-addressed (the label read targets the ODD
		// address 0x0FC0DD, which no word-boundary swap could serve).  Swapping data delivers the label as
		// "OV1LISIN0X" instead of "VOL1SINIX0".  The real board almost certainly selects this per transfer
		// from a swap-control bit in the IOCB/UIB; that bit is not decoded yet, so key it on the field type.
		// Under SRAM_BYTE_SWAPPED the local read_byte already returns the other half, so the
		// compensation inverts - otherwise the host's view double-corrects back to raw big-endian.
		// NO byte swap, either direction.  The SRAM byte-order correction (cont.426) subsumes the
		// compensation that used to live here: control blocks were already at 0 after it and verified
		// working (the 0x80 completion reaches host+3 and the host accepts commands), and cont.438
		// showed data needs 0 too - it was arriving transposed ("OV1LISIN0X" for "VOL1SINIX0").
		// Note the old expression was also wrong in form: (m_c000 + (k ^ 1)) pairs bytes across the
		// base rather than within host words, and $0FC0DD is ODD - so it transposed across the odd
		// boundary.  Removed rather than corrected, since the right value is now zero.
		// TEMP cont.436: is the DATA payload right after the cont.426 swap inversion?  The
		// firmware says "no sys-floppy", so dump what the host actually receives.
		if (to_host && is_data && len >= 16)
		{
			char t[17]; t[16] = 0;
			for (int k = 0; k < 16; k++)
			{
				u8 const c = cs.read_byte((ld + k) & 0xffff);
				t[k] = (c >= 0x20 && c < 0x7f) ? char(c) : '.';
			}
			// cont.466: timestamped so the DELIVERY can be ordered against the 0x80 completion post.
			// The CPUAP has NO poll between READ #2 and the magic test - `CXP 0x2B` blocks only because
			// of the poll INSIDE it (fe3e90 spinning on IOPB+2).  So the entire ordering contract is:
			// status must not go 0x80 until the last byte has landed.  If it does, the poll exits early
			// by construction, everything downstream reads stale memory, and NO error is raised
			// anywhere - which is exactly the observed console.
			// cont.492 acceptance test: ASCII alone cannot tell a CORRECT sector from a WRONG one
			// when the payload is binary (every MFM delivery printed as "................").  Add the
			// raw bytes so the delivery can be matched against the actual sector on the medium.
			char hx[16 * 3 + 1]; hx[0] = 0;
			for (int k = 0; k < 16; k++)
				sprintf(hx + k * 3, "%02x ", cs.read_byte((ld + k) & 0xffff));
			logerror("DATA->host %06x <- chunk %04x len=%d  first16: \"%s\"  hex: %s t=%.5f\n",
				m_c000, ld, len, t, hx, machine().time().as_double());
		}
		// OUT-OF-EXTENT GUARD (keep this - it is not a probe).  The firmware barely validates what
		// raised a record interrupt, so any spurious presentation the model makes turns into a real
		// bus-master write into host memory beyond the commanded buffer.  That is silent corruption
		// whose symptom surfaces much later and somewhere else, which is how the earlier
		// out-of-order-delivery episode stayed expensive for so long.  Announce it at the instant it
		// happens, with the transfer that did it.
		if (is_data && to_host && m_cmd_bytes
			&& (m_c000 < m_cmd_buf || m_c000 + len > m_cmd_buf + m_cmd_bytes))
			logerror("*** DMA OUT OF EXTENT: host %06x+%u outside commanded %06x..%06x "
				"(xfer #%u, recs=%d/%d) t=%.5f ***\n",
				m_c000, len, m_cmd_buf, m_cmd_buf + m_cmd_bytes,
				m_host_dma_n, m_cmd_records, m_sec_count, machine().time().as_double());
		// TEMP: the host's completion verdict is node+3, which the swap lands at host+2.
		if (to_host && !is_data && len > 3)
		{
			u8 const stv = cs.read_byte((ld + 3) & 0xffff);
			u8 const st2 = cs.read_byte((ld + 2) & 0xffff);
			// The swap sends node+2 -> host+3 and node+3 -> host+2.  The firmware's two stamps differ in
			// operand size: $1314 move.w #$81,($2,A0) puts BUSY in node+3; $1A54 move.b #$80,($2,A0) puts
			// the COMPLETION verdict in node+2, with node+3 then taking D5 (the clamped retry byte, $1A78).
			// cont.450: this message USED to print "-> host+2=%x host+3=%x" with the arguments
			// swapped, describing a byte swap that cont.438 REMOVED ("NO byte swap, either
			// direction").  The transfer is a straight copy, so host+2 == node+2.  The stale label
			// cost hours: it made the 0x80 verdict look like it landed at host+3 while the monitor
			// tests host+2, inventing a plumbing bug that did not exist.  Measured truth:
			// iopb+2=80 on success, iopb+2=82 / iopb+3=<sense> on error.
			// SENSE LATCH.  The driver asks REQUEST SENSE (0x03) once per failure - measured 103 times
			// against 103 errors - and the model answers with nothing, so it cannot classify the
			// failure or choose a recovery.  Keep the firmware's own verdict here; serving it is a
			// separate, flag-gated step.
			if (st2 == 0x82)
			{
				m_last_sense = stv;
				m_last_sense_op = m_iopb_cmd;
				m_last_sense_valid = true;
			}
			else if (st2 == 0x80)
				m_last_sense_valid = false;
			logerror("HOST POST: node+2=%02x node+3=%02x -> host+2=%02x host+3=%02x (straight copy) %s t=%.5f\n",
				st2, stv, st2, stv,
				st2 == 0x80 ? "*** COMPLETION (0x80) ***"
					: (st2 == 0x82 ? "*** ERROR (0x82) - sense in +3 ***" : (stv == 0x81 ? "busy" : "other")),
				machine().time().as_double());

			// cont.449: THE DONE EDGE ARRIVES HERE, on the control-block DMA - not through
			// bus_mem_w, which is where it was being watched.  R0 (PIO 0x73F8) is what the monitor
			// reads for its verdict on every operation ($fe3f67 MOVXBD (R5) with R5 = 0xF073F8,
			// printed as "READ -compl-stat: %x"), and spec §5 has bit1 BUSY clearing and bit2
			// OPER-DONE-INT setting at DONE.  Watching the wrong path meant BUSY never cleared, so
			// the monitor read 0x42 (floppy-present | BUSY) after every read and judged them all
			// failed regardless of the data delivered.  The tell was in every log for weeks: "HOST
			// POST ... *** COMPLETION (0x80) ***" appeared on every command while "HOST IOPB+2
			// stamp" - the bus_mem_w detector - never appeared once.
			if (st2 & 0x80) { m_r0_busy = false; m_r0_doneint = true; }

			// OS-driver channel (0x0800): DONE is polled as reg[0] bit7.  Raise it only when the
			// firmware has finished - status 0x80 (ok) or 0x82 (error) - so the loader cannot read
			// the host buffer before the data DMA lands.  Busy (0x81) is not a completion.
			if (m_os_fw_pending && (st2 == 0x80 || st2 == 0x82))
			{
				m_os_fw_pending = false;
				m_ioreg_done = true;
				m_ioreg[1] = (st2 == 0x80) ? 0x00 : (st2 & 0x03);
				// RAISE THE COMPLETION INTERRUPT ON EVERY COMMAND, not only when cmd bit0 is set.
				// The SINIX kernel registers this channel as `sa 0 ... vector 3 ipl 5` and is
				// INTERRUPT-DRIVEN: measured during the hang it reads reg0 exactly ONCE, sees
				// done=1, and then never polls again - it is blocked in its ISR wait.  Gating INT2
				// on bit0 (the HLE's rule) means no interrupt ever fires, since every kernel command
				// is 0x00/0x02 with bit0 clear, so the driver waits forever after a SUCCESSFUL read.
				if (m_ioreg_int_timer)
				{
					if (m_ioreg_pending == 0) m_ioreg_pending = 1;
					// 500us, NOT 20us: the HLE defers by 500us - "deferred so it can't beat the driver
					// into its wait".  Firing early LOSES the interrupt (the driver has not reached its
					// wait yet) and it then blocks forever.  Being a RACE, it presents as non-determinism:
					// identical code, sometimes past the j/n prompt, sometimes locked before disk select.
					m_ioreg_int_timer->adjust(attotime::from_usec(500));
				}
				logerror("IOREG DONE from HOST POST st=%02x t=%.5f\n", st2, machine().time().as_double());
			}
		}
		// TEMP cont.443: the UIB fetch.  It is re-fetched per operation here and carries FM
		// (bit1 clear at +$12) for a cylinder-1 read that is MFM by construction.  Log the SOURCE
		// address and what the host actually holds, to test whether the right block is fetched.
		if (to_local && !is_data && ld == m_uib_base && len >= 0x14)
		{
			// cont.447: was hardcoded to 0x14 - the UIB is 0x20 bytes, so +0x14..+0x1F were
			// never dumped.  Dump the WHOLE fetch; a truncated dump reads as a complete one.
			u32 const n = std::min<u32>(len, 0x40);
			char h[0x40 * 3 + 1]; h[0] = 0;
			for (u32 k = 0; k < n; k++)
				sprintf(h + k * 3, "%02x ", bs.read_byte((m_c000 + k) & 0xffffff));
			logerror("UIB FETCH host %06x -> local %04x len=%d: %s\n", m_c000, ld, len, h);
		}
		// LABEL_BODY_SKIP: drop the 4-byte ANSI record identifier from the FIRST chunk of a
		// cylinder-0/head-0 FM read and shift the whole transfer down by 4, so the host sees the
		// label BODY contiguously (volume identifier at buffer[0], +0x30/+0x44 still on their
		// fields, and the run continuing past VOL1 into the HDR1 sector).  dest = m_c000 - 4 + k
		// uniformly; the first chunk starts at k=4, which lands exactly on m_c000, so the next
		// chunk continues at +0x7C.  See the flag comment: this is an experiment.
		bool const body_skip = false && to_host && is_data && m_iopb_cmd == 0x95
			&& m_floppy[0] && m_floppy[0]->get_device()
			&& m_floppy[0]->get_device()->get_cyl() == 0 && (m_sel_head & 1) == 0
			&& flux_density_fm();
		u32 const bias = body_skip ? 4 : 0;
		u32 const k0 = (body_skip && m_data_chunks == 0) ? 4 : 0;
		for (u32 k = k0; k < len; k++)
		{
			if (to_local) cs.write_byte((ld + k) & 0xffff, bs.read_byte((m_c000 + k) & 0xffffff));
			else          bs.write_byte((m_c000 + k - bias) & 0xffffff, cs.read_byte((ld + k) & 0xffff));
		}
		if (to_host && is_data) m_data_chunks++;
		// cont.438: read the HOST side AFTER the copy.  The cont.436 "payload byte-correct" check
		// read cs.read_byte(ld+k) - the storager's OWN RAM - and BEFORE the copy, so it never
		// tested the swap.  With swap=1 on data and an ODD host base ($0FC0DD), byte k lands at
		// m_c000 + (k^1), pairing bytes across the odd boundary.
		if (to_host && is_data && len >= 16)
		{
			char hst[17]; hst[16] = 0;
			for (int k = 0; k < 16; k++)
			{
				u8 const c = bs.read_byte((m_c000 + k) & 0xffffff);
				hst[k] = (c >= 0x20 && c < 0x7f) ? char(c) : '.';
			}
			logerror("HOSTSIDE %06x  first16: \"%s\"\n", m_c000, hst);
		}
		// END-TO-END DELIVERY VALIDATION.  A CRC-validated floppy read is DETERMINISTIC, so any
		// run-to-run variation in what the host sees is a DELIVERY fault, not a media or
		// filesystem question.  (Observed: the loader rejects the superblock on attempt 1 and
		// ACCEPTS it on attempt 2 - same sectors, same medium.)  Compare every byte the model
		// placed in host memory against the bytes it decoded off the surface; they must be
		// identical.  Reports the first mismatching offset and the running totals so a capped
		// print can never be read as a clean result.
		if (to_host && is_data)
		{
			u32 bad = 0; int first_bad = -1; u8 exp0 = 0, got0 = 0;
			for (u32 k = 0; k < len; k++)
			{
				u8 const exp = cs.read_byte((ld + k) & 0xffff);
				u8 const got = bs.read_byte((m_c000 + k - bias) & 0xffffff);
				if (exp != got) { if (first_bad < 0) { first_bad = int(k); exp0 = exp; got0 = got; } bad++; }
			}
			// FIRST LEG: log each delivered sector's IDENTITY and CHECKSUM so it can be compared
			// against the IMD outside the model - that closes image -> flux -> SRAM -> host.
			{
				u32 sum = 0;
				for (u32 k = 0; k < len; k++) sum = (sum + bs.read_byte((m_c000 + k - bias) & 0xffffff)) & 0xffffff;
				floppy_image_device *const fv = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
				// Dave: "it is not delivering to the correct CPUAP address."  We WRITE through the
				// BUS space; the loader READS through the CPUAP's own space.  If those do not resolve
				// to the same memory the bytes are correct and simply not where the loader looks.
				// Read the same address back from the CPUAP side and compare.
				u32 csum = 0; bool cpuap_ok = false;
				if (m_bus)
				{
					device_t *const hd = m_bus->subdevice("^slot6:cpuap:cpu");
					cpu_device *const hc = dynamic_cast<cpu_device *>(hd ? hd : machine().device<device_t>(":slot6:cpuap:cpu"));
					if (hc)
					{
						address_space &hs = hc->space(AS_PROGRAM);
						for (u32 k = 0; k < len; k++) csum = (csum + hs.read_byte((m_c000 + k - bias) & 0xffffff)) & 0xffffff;
						cpuap_ok = true;
					}
				}
				logerror("DELIV cyl=%d head=%d r=%02x len=%u host=%06x sum=%06x cpuap=%s%06x t=%.5f\n",
					fv ? fv->get_cyl() : -1, m_walk_head & 1, m_last_r, len, m_c000, sum,
					cpuap_ok ? "" : "N/A:", csum,
					machine().time().as_double());
			}
			m_verify_n++;
			if (bad) m_verify_bad++;
			if (bad && m_verify_prints < 24)
			{
				m_verify_prints++;
				logerror("VERIFY MISMATCH host=%06x len=%u bad=%u first@+%d exp=%02x got=%02x | sectors=%u bad=%u t=%.5f\n",
					m_c000, len, bad, first_bad, exp0, got0, m_verify_n, m_verify_bad, machine().time().as_double());
			}
		}
	}
	// The gate array bus-masters the transfer: it takes the local bus for the transfer's duration and the
	// 68000 stalls on DTACK, so the firmware cannot start the next field's work until the DMA completes.
	// Model that as the level it is - hold the CPU off the bus at the kickoff, release it when the transfer
	// ends - rather than as a fixed timeout, so the hold is bounded by this transfer alone (nothing else on
	// the machine can pre-empt it) and its length falls out of the byte count.  Without the hold the CPU
	// races ahead of the instant DMA and the read's descriptor drain launches only 2 of 8 sectors; with it
	// all 8 launch and the descriptor queue drains.
	m_cpu->suspend(SUSPEND_REASON_HALT, true);
	m_bus_held = true;
	m_dma_done->adjust(dma_time(len));
}

// Transfer time for a bus-master DMA of len bytes.  The gate array moves a 16-bit word per Multibus
// cycle; ~600ns/word (rev A handshake at 10MHz) puts a 128-byte FM sector at ~38us.
attotime multibus_storager_device::dma_time(u32 len) const
{
	return attotime::from_nsec(600) * ((len + 1) / 2);
}

// End of a real bus-master DMA: release the local-bus hold and raise channel-done IRQ4.
// Write-close failsafe does NOT use this timer (see arm_wr_close_failsafe / wr_close_check).
TIMER_CALLBACK_MEMBER(multibus_storager_device::dma_done)
{
	if (m_bus_held)
	{
		m_cpu->resume(SUSPEND_REASON_HALT);
		m_bus_held = false;
	}
	m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE);
}

// Observe window only (v6).  ≈5× measured fw self-complete (90–130us).  Do NOT raise IRQ4 on
// timeout — inst6: that raise while node+2=81 is one-to-one with the $4626 derail.
static constexpr int WR_CLOSE_OBSERVE_US = 750;

void multibus_storager_device::arm_wr_close_failsafe(char const *why)
{
	address_space &cs = m_cpu->space(AS_PROGRAM);
	u8 const node2 = m_node_base ? cs.read_byte((m_node_base + 2) & 0xffff) : 0xff;
	// Already done at the arming instant — no need to wait (healthy closes often land 80 first).
	if (node2 == 0x80)
	{
		logerror("WRDONE %d/%d walked=%d - fw already posted (node+2=80) at close (%s); "
			"[7302]=%04x [$7986]=%04x t=%.5f\n",
			m_wr_done, m_sec_count, m_wr_walked, why,
			cs.read_word(0x7302), cs.read_word(0x7986),
			machine().time().as_double());
		return;
	}
	logerror("WRDONE %d/%d walked=%d - arm close observe %dus (%s) "
		"[7302]=%04x [$7986]=%04x node+2=%02x t=%.5f\n",
		m_wr_done, m_sec_count, m_wr_walked, WR_CLOSE_OBSERVE_US, why,
		cs.read_word(0x7302), cs.read_word(0x7986), node2,
		machine().time().as_double());
	if (m_wr_close)
		m_wr_close->adjust(attotime::from_usec(WR_CLOSE_OBSERVE_US));
}

TIMER_CALLBACK_MEMBER(multibus_storager_device::wr_close_check)
{
	address_space &cs = m_cpu->space(AS_PROGRAM);
	u8 const node2 = m_node_base ? cs.read_byte((m_node_base + 2) & 0xffff) : 0xff;
	u16 const freelist = cs.read_word(0x7986);
	if (node2 == 0x80)
	{
		logerror("WRCLOSE observe: node+2=80 (fw self-completed) done=%d/%d "
			"[$7986]=%04x t=%.5f\n",
			m_wr_done, m_sec_count, freelist, machine().time().as_double());
		return;
	}
	// Never raise IRQ4 here.  Live [$7986] is the hang fingerprint (healthy final clears it to 0;
	// the spare-track 4-sector write leaves it non-zero).  Dump the ACTIVE queue head [$736c]
	// separately: IRQ1 walks $736c, not $7986.  $7986 is the per-context cell; $2B80 is expire
	// only (count hit 0), not the tick itself.
	u16 const qhead = cs.read_word(0x736c);
	u16 blk_cnt = 0xffff, blk_code = 0xffff, blk_next = 0xffff;
	bool on_queue = false;
	if (freelist >= 0x4000 && freelist < 0x7f00)
	{
		blk_cnt  = cs.read_word(freelist);
		blk_code = cs.read_word((freelist + 2) & 0xffff);
		blk_next = cs.read_word((freelist + 8) & 0xffff);
		for (u16 p = qhead, guard = 0; p >= 0x4000 && p < 0x7f00 && guard < 32; guard++)
		{
			if (p == freelist) { on_queue = true; break; }
			u16 const n = cs.read_word((p + 8) & 0xffff);
			if (n == p || n == 0) break;
			p = n;
		}
	}
	logerror("WRCLOSE STALL (no IRQ4): node+2=%02x still busy after %dus done=%d/%d "
		"[7302]=%04x [7950]=%04x [$7986]=%04x [$736c]=%04x "
		"blk{cnt=%04x code=%04x next=%04x on_q=%d} pc=%06x t=%.5f\n",
		node2, WR_CLOSE_OBSERVE_US, m_wr_done, m_sec_count,
		cs.read_word(0x7302), cs.read_word(0x7950), freelist, qhead,
		blk_cnt, blk_code, blk_next, on_queue ? 1 : 0,
		m_cpu->pcbase() & 0xffffff, machine().time().as_double());
	// Discriminator (cheap): if on_q and the block is in RAM, sample cnt across the next few
	// IRQ1 raises.  ffff→fffe→fffd = live countdown (wrap or huge arm).  stuck ffff = not
	// serviced despite on_q=1.  Do not conclude from a single STALL snapshot alone.
	if (on_queue && freelist >= 0x4000 && freelist < 0x7f00)
	{
		m_wr_stall_watch_blk = freelist;
		m_wr_stall_watch_left = 8;
		logerror("WRSTALL WATCH arm blk=%04x cnt=%04x for %d IRQ1 samples t=%.5f\n",
			m_wr_stall_watch_blk, blk_cnt, m_wr_stall_watch_left,
			machine().time().as_double());
	}
}

// ---------------------------------------------------------------------------
// gate-array registers
// ---------------------------------------------------------------------------

// TEMP (STRIP): one ordered, timestamped trace of every gate-array access during the read, so the
// op18 -> op42 transition and the IRQ5/IRQ6 cadence can be read against each other on one clock.

u16 multibus_storager_device::ch_r(offs_t offset, u16 mem_mask)
{
	u32 const a = 0xe000 + offset * 2;
	u16 d = m_ch[offset];

	// E000: the gate array's command/status register (the firmware reads it only a handful of times, as
	// status - it does NOT byte-assemble the bit stream; the gate array deserializes each record into SRAM
	// itself and signals via the IRQ6/IRQ5 marks).  Return the register echo.
	if (a == 0xe000)
	{
		return m_ch[offset];
	}

	// E01E: the gate array's record-status latch.  Reading it is the firmware's ACK - it CLEARS the latch
	// so the gate array can deliver the next record.  The read value itself is discarded by the firmware
	// (the CRC/status verdict rides the collected status word, not this port), so the strobe is the point.
	if (a == 0xe01e)
	{
		d |= 0x0010;
		m_rec_latch = false;
	}

	// F000: drive + auxiliary status (spec §3.3).  Decoded from op24 ($651C), the unit-select/status
	// micro-op, which is the only reader before a transfer starts:
	//   bit0  FAULT      $663C btst #0 -> if SET the op aborts with error $10
	//   bit2  BATCH RDY  $846a btst #2 -> SOLE ROM consumer; SET required to enter $8480 and
	//                    install the {#$32,#$201C} soft-timer at $84DA.  See the bit2 transfer-gate note below.
	//   bit3  FAULT      $65AE btst #3 -> if SET the op aborts with error $1B
	//   bit4  INDEX      $6746 btst #4
	//   bit5  UNIT READY $66D6 masks F000 and compares == $20, i.e. ready with no other status bits
	//   $6772 composes the per-unit status byte: [$794e] = ~F000 & table[$22E + unit], so a CLEAR F000
	//   bit becomes a SET fault bit.  Table @$22E = 00 06 00 27 00 3f 00 2d ef ff cf ef 2f ff 0f ef,
	//   i.e. a different mask per device class (floppy / ESDI / tape).
	// Both fault bits must read back CLEAR and bit5 SET for a unit to be usable - which is what lets
	// the firmware decide a unit is present before it will boot from it.
	if (a == 0xf000)
	{
		// cont.498: WHO reads F000, as a PC HISTOGRAM.  The claim under test is an ABSENCE - that
		// 008446 (`btst #$2` -> the deferred-queue reload at 00847e) never proceeds - and every
		// instrument failure in this campaign has produced a false negative.  A histogram cannot:
		// the other readers ARE the control, in the same stream.  pcbase runs +0x18 ahead here
		// (measured on four sites), so 008446 reports as 00845e.  Count uncapped, print capped,
		// last timestamp reported with the count.
		if (TRACE_CHUNK_PATH)
		{
			// A PRINT CAP truncates by TIME (3000 lines were all consumed before t=0.23, leaving the
			// entire read window unsampled).  Histogram in-device instead: count uncapped, emit the
			// whole table once per emulated second.  Coverage complete, volume bounded.
			m_f000_n++;
			double const ft = machine().time().as_double();
			m_f000_hist[m_cpu->pcbase()]++;
			if (ft - m_f000_last_t >= 1.0)
			{
				m_f000_last_t = ft;
				std::string h;
				for (auto const &kv : m_f000_hist)
					h += util::string_format("%06x:%u ", kv.first, kv.second);
				logerror("F000HIST t=%.3f total=%u | %s\n", ft, m_f000_n, h);
			}
		}
		if (!m_floppy_loaded)
			spin_drives();
		floppy_image_device *const fdd = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
		bool const present = fdd && fdd->exists();
		bool const trk0 = present && !fdd->trk00_r();   // trk00_r active-low
		// wpt_r() is NOT active-low.  MAME sets m_wpt = is_readonly() || !write_supported and every
		// consumer tests it directly (upd765: `wpt_r() ? ST3_WP : 0`; wd_fdc: `if (floppy->wpt_r())`).
		// trk00_r() above IS active-low; this comment had copied that convention.
		// MEASURED on the boot floppy (mx2-001.imd, mode -r--r--r-- since 2022):
		//     wpt_r()=1  is_readonly=1  -> old code gave wprot=0, i.e. the model told the firmware
		//     a WRITE-PROTECTED medium was WRITABLE.
		bool const wprot = present && fdd->wpt_r();
		bool const index = present && fdd->idx_r();

		// bit12 DMA address-match (the transfer pointer reached the latched terminal)
		u32 const term = 0x4000 | m_dma_term | m_term_bit0;
		d = (d & ~0x1000) | ((m_dma_active && m_last_bw == term) ? 0x1000 : 0);
		d = (d & ~0x0800) | (m_timer_out ? 0x0800 : 0);   // bit11 = PIT timer OUT
		// Drive READY / door-closed demux by E804 select (same split as bit0 clock/STEP).
		// Floppy: medium present.  Rigid: hard-disk image present.
		// MEASURED (cont.573): after serial RECAL ($1000) the rigid path at $6d4e waits for
		//   F000 & ($7a0a ? $220 : $120) == $20
		// i.e. bit5 SET and bit8/9 CLEAR.  Bit5 was floppy-only, so HD-only boot timed out $30
		// in the CONTROL+RECAL loop forever.  Bit8 is already held clear below.
		bool const f000_floppy_sel = ((m_sel_drive & 0xe0) == 0xc0);
		bool const rigid_ready = (m_hd[0] && m_hd[0]->exists()) || (m_hd[1] && m_hd[1]->exists());
		bool const unit_ready = f000_floppy_sel ? present : rigid_ready;
		d = (d & ~0x0080) | (unit_ready ? 0x0080 : 0);   // bit7  = drive ready / door-closed
		d = (d & ~0x0020) | (unit_ready ? 0x0020 : 0);   // bit5  = drive READY (fw $6d62/$66D6)
		d = (d & ~0x2000) | (trk0 ? 0 : 0x2000);          // bit13 = track 0, active-low (fw 0x5de4 btst #$d)
		d = (d & ~0x0400) | (wprot ? 0x0400 : 0);         // bit10 = write-protect
		d = (d & ~0x0010) | (index ? 0x0010 : 0);         // bit4  = index pulse
		// ...except on an ESDI unit mid-response, where bit4 is the drive's CONFIG/STATUS DATA line.
		// Demux by drive select, the same hardware-faithful routing as the E802 bit0 clock/STEP
		// split - an ESDI drive has no index line here and a floppy has no serial response.
		// INVERTED: $A1CE treats `btst #4` CLEAR as a received 1.
		if (ESDI_SERIAL_ENGINE && m_esdi_resp_phase && !f000_floppy_sel)
			d = (d & ~0x0010) | (m_esdi_out_bit ? 0 : 0x0010);
		if (!f000_floppy_sel)
		{
			// Rigid F000 bit1 (cont.572): transfer-ack ONLY inside a 17-beat serial frame; CLEAR at
			// idle.  Measured: restore never steps a rigid unit; $6E58 needs bit1 CLEAR to finish,
			// while $A118 needs bit1 to follow the clock mid-frame.  "Always follow clock" breaks
			// restore (idle clock high → bit1 stuck SET → $30).  Settle-after-edge breaks $A118
			// (second half of each beat never sees bit1 SET → $201E).
			if (m_esdi_frame_active)
				d = (d & ~0x0002) | (m_ser_clk ? 0x0002 : 0);
			else
				d &= ~0x0002;
		}
		else
			d = (d & ~0x0002) | (trk0 ? 0 : 0x0002);   // floppy: position not confirmed home
		// Always strip backing-store residue; optionally drive from m_prog_loaded (v2 — DMA term
		// retired after measured falsification at $846a with prog=1 dma=1).
		// bit2 = TRANSFER GATE, and a healthy channel holds it asserted.  ESTABLISHED (cont.571)
		// from the only two consumers in the ROM, both of which treat CLEAR as a fault and neither
		// of which has a path that wants it clear:
		//     $846E  btst #2 -> clear -> sense $1A
		//     $8ACC  andi #4 -> clear -> $8F1E -> sense $1A  (alongside faults $27/$28)
		// So it is not a batch-ready strobe.  It was previously driven from m_prog_loaded, which
		// was a guess: set often enough to carry an install, clear often enough elsewhere to
		// produce 1736 x $201A in one 220s run.  Same treatment as the drive-fault bits below - a
		// healthy modelled unit reports no fault.
		d |= 0x0004;
		if (TRACE_GAWRITE && cmd_is_write())
			logerror("F000B2 d=%04x b2=%d prog=%d dma=%d t=%.5f\n",
				d, BIT(d, 2), m_prog_loaded ? 1 : 0, m_dma_active ? 1 : 0,
				machine().time().as_double());
		// V2: surface PIT1 ctr2 OUT on bit8 (suspected op42 Branch-B busy/fault).  Baseline leaves
		// bit8 clear.  Read path is Branch A so a change here should not move op42 by itself.
		d &= ~0x0100;                                    // bit8 unmodeled; keep clear
		d &= ~0x0200;                                    // bit9 unmodeled; keep clear
		d &= ~0x0009;                                    // bits 0/3 = drive faults (errors $10 / $1B); a healthy unit reports none
		// WHAT DOES THE FIRMWARE WAIT ON AFTER THE WRITE SETUP?  Logged on the COMPOSED value -
		// an earlier version of this tap sat before the composition above and reported 0x0000 for
		// every read, which is the raw backing store and says nothing about what the firmware saw.
		if (TRACE_GAWRITE && (m_iopb_cmd == 0x96 || m_iopb_cmd == 0x82))
		{
			m_gaw_f000_n++;
			if (m_gaw_f000_n <= 400 || (m_gaw_f000_n % 5000) == 0)
				logerror("GAWF000 #%u -> %04x  b2batch=%d b5rdy=%d b7rdy=%d b10wp=%d b4idx=%d b13trk0=%d "
					"prog=%d dma=%d pc=%06x t=%.5f\n",
					m_gaw_f000_n, d, BIT(d,2), BIT(d,5), BIT(d,7), BIT(d,10),
					BIT(d,4), BIT(d,13), m_prog_loaded ? 1 : 0, m_dma_active ? 1 : 0,
					m_cpu->pcbase() & 0xffffff, machine().time().as_double());
		}
	}
	return d;
}

void multibus_storager_device::ch_w(offs_t offset, u16 data, u16 mem_mask)
{
	// TRACE_GAWRITE (STRIP): capture the gate-array program the firmware issues for a WRITE.
	// We have never observed one - every flux operation this model has ever serviced was a read -
	// so the write path would otherwise be built against a guess.  Logs every gate-array register
	// write while a write-class firmware command is current, which is a small, bounded set.
	if (TRACE_GAWRITE && (m_iopb_cmd == 0x96 || m_iopb_cmd == 0x82))
	{
		u32 const a = 0xe000 + (offset * 2);
		logerror("GAW cmd=%02x reg=%04x <= %04x mask=%04x pc=%06x t=%.5f\n",
			m_iopb_cmd, a, data, mem_mask, m_cpu->pcbase() & 0xffffff,
			machine().time().as_double());
	}
	u32 const a = 0xe000 + offset * 2;
	COMBINE_DATA(&m_ch[offset]);

	// E000-E01E doubles as the field step program's 16 words.  op18 block-writes them in ascending
	// order; the write that completes the block (E01E) is the load-complete edge on which the gate
	// array's sequencer starts running the program.
	if (offset < std::size(m_prog))
	{
		m_prog[offset] = m_ch[offset];
		// m_prog_loaded means "the gate array HOLDS a program" and now persists across commands, so it
		// can no longer double as "we are not mid-load".  Track the block copy separately: the record
		// ISR's per-record cycle (022f -> 023f -> 0a6d) only ever writes offset 0, so a write to any
		// offset >= 1 is the block copy and nothing else. (cont.429)
		if (offset >= 1)
			m_prog_loading = true;
		// Trigger only when a real block copy just finished.  Without this, any repeat of the
		// load-complete edge re-enters start_field_program() mid-read, which resets per-command state
		// (model trap #1).  m_prog_loading is set only by a write to offset >= 1, which only the block
		// copy does - so this is the block-copy completion edge and nothing else.
		if (offset == std::size(m_prog) - 1 && m_prog_loading)
		{
			m_prog_loading = false;
			// A block whose every word is $023F is a WIPE, not a load.  $023F is the idle/terminate
			// step that tails op18's own program, and $5E64 ($5E98 lea $e000,a1 / $5EA0 move.w
			// #$23f,(a1)+ / dbra, sixteen times) uses exactly that to CLEAR the field program - it runs
			// on every read, emitting through the inherited A6 and wiping E000 as it goes.  Treating
			// that as a load-complete edge marked a program loaded and opened the read window on the
			// very sequence that clears it, so the model and the firmware disagreed about what the
			// gate array was running. (cont.434)
			bool wipe = true;
			for (u16 w : m_prog)
				if (w != 0x023f) { wipe = false; break; }
			if (wipe)
			{
				if (m_prog_loaded)
					logerror("GA field program WIPED (16x $023f) during cmd=%02x - was loaded, now cleared t=%.5f\n",
						m_iopb_cmd, machine().time().as_double());
				m_prog_loaded = false;
				m_read_window = false;   // nothing left to run
				return;
			}
			m_prog_loaded = true;      // a fresh load REPLACES a retained one
			// TEMP cont.439: is DENSITY encoded in the field program?  The firmware never commands
			// E800 bit10 (the model's density source) but does set node+$12 bit1 for MFM, and cont.428
			// established this program describes the TRACK FORMAT.  Dump it per load and compare the
			// FM (cyl 0) and MFM (cyl 1) programs.
			{
				char h[16 * 5 + 1]; h[0] = 0;
				for (int k = 0; k < 16; k++)
					sprintf(h + k * 5, "%04x ", m_prog[k]);
				// Correlate against the CYLINDER, whose density is known independently from the media
				// (IMD: cyl 0 = 300k FM 16x128, cyl 1+ = 300k MFM 16x256).  bit7 of every program word
				// is the candidate discriminator; print its tally so the labelling is measured, not
				// assumed.
				int b7set = 0;
				for (int k = 0; k < 16; k++)
					if (BIT(m_prog[k], 7)) b7set++;
				floppy_image_device *const fdd0 = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
				address_space &cs3 = m_cpu->space(AS_PROGRAM);
				u32 const uib = m_uib_base, nd2 = m_node_base;
				logerror("  at load: node+12=%02x(bit1=%d)  UIB+12=%02x(bit1=%d)  E800=%04x(bit10=%d)\n",
					cs3.read_byte((nd2 + 0x12) & 0xffff), BIT(cs3.read_byte((nd2 + 0x12) & 0xffff), 1),
					cs3.read_byte((uib + 0x12) & 0xffff), BIT(cs3.read_byte((uib + 0x12) & 0xffff), 1),
					m_ch[(0xe800 - 0xe000) / 2], BIT(m_ch[(0xe800 - 0xe000) / 2], 10));
				logerror("PROGRAM cyl=%d (media says %s)  bit7 set in %d/16 words  cmd=%02x: %s\n",
					fdd0 ? fdd0->get_cyl() : -1,
					(fdd0 && fdd0->get_cyl() == 0) ? "FM" : "MFM",
					b7set, m_iopb_cmd, h);
			}
			start_field_program();
		}
	}

	if (a == 0xe000)
	{
		// Command decode (the gate array is stimulus/response: the firmware COMMANDS what to hunt for via
		// E000 codes, and the gate array delivers the matching record's interrupt to the phase's installed
		// soft-vector handler, spec cont.370).  0x22F/0x23F = hunt the ID address mark (-> $9884 compare);
		// 0x2AF/0x2FF = read the data field.  The low byte's arm nibble selects; the mark is delivered only
		// while the corresponding command is armed, so interrupts arrive in the firmware's phase order.
		// 0x0A6D = op18 word[0] re-issue (per-record re-arm third word @ $8936).  board.yaml noted the
		// model decoded 022f/023f but not 0a6d as a *command*; bit11 below still opens the window.
		// Name it here so ID-hunt state matches the locate head of the program.
		u16 const code = data & 0x07ff;   // mask off bit11 (window) - the code may ride the same write
		if ((code == 0x22f || code == 0x23f || code == 0x26d) && cmd_armable()) m_cmd = CMD_IDHUNT;
		else if ((code == 0x2af || code == 0x2ff) && (m_iopb_cmd == 0x94 || m_iopb_cmd == 0x95)) m_cmd = CMD_DATA;

		// The firmware's channel-engagement write (E000 bit11) opens the read window.  op18 has already
		// programmed the gate array with the field layout + the commanded sector count; the gate array runs
		// the operation autonomously.  On the rising edge we capture the whole track (detection-is-capture)
		// and arm record delivery (advance_read).  Each per-record re-arm
		// (E802 bit11) releases the next mark.
		// Once the program is loaded, an E000 bit11 write is the firmware's per-record RE-engagement
		// (the 022f -> 023f -> 0a6d cycle each record ISR issues, whose third word re-issues the
		// program's first command).  Before the load completes the identical bit pattern is merely
		// word[0] of the block being copied in - not a command - so it must not start anything.
		bool const win = BIT(data, 11) && m_prog_loaded && !m_prog_loading && cmd_armable();
		// Which term blocks the write window?  Log every bit11 write under a write-class command
		// with each conjunct separately - guessing which one is false has already cost one round.
		if (TRACE_GAWRITE && cmd_is_write() && BIT(data, 11))
			logerror("GAWWIN e000<=%04x b11=1 loaded=%d loading=%d armable=%d prev=%d -> win=%d "
				"pc=%06x t=%.5f\n", data, m_prog_loaded, m_prog_loading, cmd_armable(),
				m_e000b11_prev, win, m_cpu->pcbase() & 0xffffff, machine().time().as_double());
		if (win && !m_e000b11_prev)
			start_field_program();
		m_e000b11_prev = win;
	}
	else if (a == 0xe800)
	{
		m_dma_active = BIT(data, 6);
		m_gate0 = BIT(data, 9);
		m_pit[1]->write_gate0(BIT(data, 9));
		// kickoff = the RISING edge of bit12 (the disk op does ori #$1000; the IRQ4 handler clears it).
		bool const kick = BIT(data, 12) && !m_e800_bit12_prev;
		m_e800_bit12_prev = BIT(data, 12);
		if (kick)
		{
			// TEMP (STRIP): every bit12 rising edge, including those that do not transfer
			// (missing C000, wrong direction, etc.) - settles the "0 vs 8/8 DMA" measurement gap.
			logerror("E800b12 %10.1fus rise e800=%04x d000=%04x c000=%06x c000v=%d pc=%06x\n",
				machine().time().as_double() * 1e6, data, m_d000, m_c000, m_c000_valid ? 1 : 0,
				m_cpu->pcbase());
			run_channel_dma();
		}
	}
	else if (a == 0xe804)
	{
		// Drive / head select.  op24 writes it twice (the second write merges the head nibble back over
		// the control byte); op28's seek engine drives stepping through it.  Bits 8-11 carry the head
		// ONE'S-COMPLEMENTED, so head 0 presents as $F (the observed $BF78 = head 0, the cyl0 label).
		// TEMP cont.565 (STRIP): the E804 low byte is copied verbatim from the SELECTED UNIT'S
		// UIB at +$DC ($6576 reloads D3 from ($dc,A1), $6588 moves it into the low byte).  UIB+$12
		// never reaches E804 - it only gates E802 bit2 at $655C.  Print both so "select byte ==
		// active unit's +$DC" is confirmed rather than inferred, which is what lets the model tell
		// which device the shared E802 bit0 pulse is addressed to.
		if (m_uib_base && m_mux_prints < 24)
		{
			address_space &us = m_cpu->space(AS_PROGRAM);
			m_mux_prints++;
			logerror("E804W data=%04x low=%02x | uib=%04x UIB+$dc=%04x | cmd=%02x t=%.5f\n",
				data, u8(data & 0xff), m_uib_base,
				m_uib_base ? us.read_word((m_uib_base + 0xdc) & 0xffff) : 0xffff,
				m_iopb_cmd, machine().time().as_double());
		}
		m_sel_head = u8(~(data >> 8) & 0x0f);
		m_sel_drive = u8(data & 0xff);
	}
	else if (a == 0xe802)
	{
		// bit0 = ESDI serial clock (rigid) or floppy STEP (select $c0).  Demux by E804 select.
		if (ACCESSING_BITS_0_7)
		{
			bool const clk = BIT(data, 0);
			bool const floppy_sel = ((m_sel_drive & 0xe0) == 0xc0);
			bool const rigid = !floppy_sel;
			bool const fall = rigid && m_ser_clk && !clk;
			bool const rise = rigid && !m_ser_clk && clk;
			if (fall)
			{
				// New frame, or continue.  If the previous frame already had 17 falls, close it
				// first (covers A118→A1CE with no intervening non-edge write).
				if (m_esdi_frame_active && m_esdi_frame_falls >= 17)
				{
					if (TRACE_ESDI)
						logerror("ESDI frame end (next fall) falls=%d t=%.5f\n",
							m_esdi_frame_falls, machine().time().as_double());
					m_esdi_frame_active = false;
					m_esdi_frame_falls = 0;
					m_esdi_frame_closing = false;
				}
				if (!m_esdi_frame_active)
				{
					m_esdi_frame_active = true;
					m_esdi_frame_falls = 0;
					m_esdi_frame_closing = false;
				}
				m_ser_last_t = machine().time().as_double();
				m_ser_active = true;
				if (ESDI_SERIAL_ENGINE)
					esdi_serial_beat(data);
				m_esdi_frame_falls++;
				if (TRACE_ESDI && m_esdi_frame_falls <= 3)
					logerror("ESDI frame fall #%d beat=%d resp=%d sel=%02x e802=%04x t=%.5f\n",
						m_esdi_frame_falls, m_esdi_beat, m_esdi_resp_phase ? 1 : 0,
						m_sel_drive, data, machine().time().as_double());
			}
			else if (rise)
			{
				// Do NOT end the frame here: $A118 still samples bit1 SET after this rising edge.
				// Mark closing; the epilogue e802 rewrite (clock already high) ends the frame.
				if (m_esdi_frame_active && m_esdi_frame_falls >= 17)
					m_esdi_frame_closing = true;
			}
			else if (!rigid && m_ser_clk && !clk)
			{
				// Floppy STEP — do not arm the ESDI frame or shift register.
				m_ser_last_t = machine().time().as_double();
			}
			m_ser_clk = clk;
			// Epilogue / level rewrite after beat 17 high: drop frame so idle bit1 is CLEAR.
			if (rigid && m_esdi_frame_closing && m_esdi_frame_falls >= 17 && clk && !rise)
			{
				if (TRACE_ESDI)
					logerror("ESDI frame end (epilogue) falls=%d t=%.5f\n",
						m_esdi_frame_falls, machine().time().as_double());
				m_esdi_frame_active = false;
				m_esdi_frame_falls = 0;
				m_esdi_frame_closing = false;
			}
		}
		// bit6 = the IRQ2 ack the doorbell handler toggles LOW on entry.
		if (!BIT(data, 6))
			m_cpu->set_input_line(M68K_IRQ_2, CLEAR_LINE);
		// bit11 = per-record capture re-arm (the `ori #$800` each read ISR issues, $7BA8/$8018).  Its
		// RISING edge primes the gate array to latch the NEXT record; the arm persists (a one-shot) until
		// a record consumes it, so the decoupled 200us pump can deliver the mark whenever it reaches it.
		// When the firmware stops re-arming (read complete), records stop and it converges to completion.
		bool const b11 = BIT(data, 11) && cmd_armable();
		// ARM TIMELINE (TRACE_ARM, STRIP).  Ground truth on the WORKING READ path: does the SETUP
		// leg's E802 bit11 pulse produce m_armed=1, and does the DONE mark then consume it?
		// $7BA8 (read SETUP) and $8552 (write re-arm) both do the same clear->set pulse, so if the
		// read's second entry is armed by its own first leg, the write's $8552 should do the same.
		// Log the LEVEL and the EDGE separately - "armed became true" alone cannot distinguish a
		// missing pulse from a missed rising edge (m_bit11_prev stuck high).
		if (TRACE_ARM)
		{
			static int n = 0;
			if (++n <= 6000)
				logerror("ARM| e802<=%04x lvl=%d prev=%d edge=%d armed=%d pend=%d pc=%06x t=%.5f\n",
					data, BIT(data, 11), m_bit11_prev ? 1 : 0,
					(b11 && !m_bit11_prev) ? 1 : 0, m_armed ? 1 : 0, m_mark_pending,
					m_cpu->pcbase() & 0xffffff, machine().time().as_double());
		}
		// cont.569 write gap probe: after WRLOC the next mark needs a bit11 rising edge (or the
		// D800 same-value re-arm).  Log every E802 write while a write is live so the 1.81s
		// window shows A (no writes) vs B (writes, no edge) vs C (edges that don't arm).
		if (cmd_is_write() && m_write_active)
		{
			static int n = 0;
			if (++n <= 200)
				logerror("E802W data=%04x b11=%d prev=%d edge=%d armed=%d pend=%d phase=%d "
					"done=%d/%d pc=%06x t=%.5f\n",
					data, BIT(data, 11), m_bit11_prev ? 1 : 0,
					(b11 && !m_bit11_prev) ? 1 : 0, m_armed ? 1 : 0, m_mark_pending,
					m_sec_phase, m_wr_done, m_sec_count, m_cpu->pcbase() & 0xffffff,
					machine().time().as_double());
		}
		if (b11 && !m_bit11_prev)
		{
			m_armed = true;          // the ISR's re-arm bit-change: stage + deliver the next record
			deliver_mark();
			advance_read();
		}
		m_bit11_prev = b11;
		// bit7 = host-completion interrupt level to the CPUAP (rising edge -> Multibus INT).
		bool const host_int = BIT(data, 7);
		if (host_int && !m_host_int_prev)
			int_w<2>(0);
		m_host_int_prev = host_int;

		// bit0 = floppy STEP or ESDI serial clock.  Demux by E804 select: a rigid unit never steps
		// (cont.572), and serial clocks must not move the floppy or fire TRK0 position zeroing
		// (cont.573 STEPEDGE leak during $A118).  Direction is bit1, not bit13 (cont.558).
		// MAME steps on the 1->0 edge; dir_w=1 -> toward track 0.  Each STEP retriggers the PIT ctr1
		// mode-5 settle one-shot on its gate (F000 bit1 busy until it times out) - floppy only.
		if ((m_sel_drive & 0xe0) == 0xc0)
		if (floppy_image_device *const fdd = m_floppy[0] ? m_floppy[0]->get_device() : nullptr)
		{
			// TEMP cont.552 (STRIP): step-pulse census.  One firmware pulse must move exactly one
			// cylinder.  Print is capped, COUNTING never is, and the running totals go out with
			// every line so a capped tail cannot be read as a complete record.
			int const cyl_before = fdd->get_cyl();
			// STEP DIRECTION IS BIT 1, NOT BIT 13 (cont.558).  Measured by diffing the E802 word
			// between seeks whose required direction is known:
			//   boot 0->1, 1->2   (inward)          w=22d0   bit1=0
			//   pos 15 -> target 5 (needs OUTWARD)  w=22d2   bit1=1
			//   pos -1 -> target 5 (needs inward)   w=22d0   bit1=0
			// bit13 is 1 in ALL of them, so keying on it stepped inward always - which is why the
			// head ran away from every target below its position.  The firmware's own seek loop
			// tests this bit ($6A00 `btst #1,D6`) before choosing its pulse path.
			// MAME: dir_w(1) decrements toward track 0, dir_w(0) increments.
			fdd->dir_w(BIT(data, 1) ? 1 : 0);
			// A flat divide-by-two here is REFUTED by measurement (cont.555): it dropped boot
			// deliveries 69 -> 12 and sent the head to cyl 43.  The firmware DOES double its pulse
			// count at $6A60 (`lsl.w #1,D6`), but halving every pulse starves the boot's
			// single-cylinder seeks, after which the firmware recalibrates in a fast pulse storm
			// (5us spacing vs the seek loop's 5.25ms) - a CONSEQUENCE of the break, not evidence
			// of a pre-existing second train.  Measure pulses-per-seek against the intended
			// cylinder delta, on the WORKING boot seeks, before touching this again.
			fdd->stp_w(BIT(data, 0));
			int const cyl_after = fdd->get_cyl();
			// TRACK-0 RESET IS THE GATE ARRAY'S JOB.  The TRK0 sensor is wired to the gate array,
			// not to the CPU, so the gate array zeroes the head position on the TRK0 EDGE - it owns
			// the sensor, and it already knows where the position lives because it fetched the UIB
			// (base in [$799a], position at +$d6).  This is hardware behaviour, not a poke into
			// firmware state.
			//
			// Without it a recalibrate cannot converge: the firmware writes `pos = 15` as a
			// position-unknown sentinel and seeks outward expecting the hardware to stop it and
			// re-zero at track 0.  The head reached track 0 correctly once the direction bit was
			// fixed (cont.558) but the position stayed at 5, leaving it five cylinders adrift with
			// nothing to correct it.  (The firmware's own $5DE0 TRK00 handler dereferences $72e2,
			// which is null on this path - a fallback the read never reaches.)
			if (cyl_after != cyl_before) m_stepped_since_arm = true;
			if (cyl_after == 0 && cyl_before != 0)
			{
				address_space &ct = m_cpu->space(AS_PROGRAM);
				u16 const ub = ct.read_word(0x799a);
				if (ub >= 0x4000 && ub < 0x7f00)
				{
					ct.write_word((ub + 0xd6) & 0xffff, 0);
					logerror("TRK0 EDGE: gate array zeroes head position at %04x (uib %04x) t=%.5f\n",
						(ub + 0xd6) & 0xffff, ub, machine().time().as_double());
				}
			}
			m_step_writes++;
			if (cyl_after != cyl_before) m_step_moves++;
			if (BIT(data, 0) && !m_step_bit0_prev) m_step_edges++;
			m_step_bit0_prev = BIT(data, 0);
			// Print EVERY move from t=0, not just the failing seek: the boot's two successful moves
			// are the control for direction polarity, and gating on t>11.4 hid exactly the arm that
			// works.  Only actual movement is printed, so the cap cannot be consumed by idle writes.
			// cont.556: log every bit0 TRANSITION, moved or not, so the write pattern per cylinder
			// is visible on the boot's known-good single-cylinder seeks.  If one cylinder costs a
			// complete low->high cycle (two writes), the $6A60 doubling is counting write halves
			// and no divider is needed anywhere.
			bool const lvl = BIT(data, 0);
			bool const changed = lvl != m_step_lvl_prev;
			m_step_lvl_prev = lvl;
			if (m_step_prints < 40 && changed)
			{
				m_step_prints++;
				// The firmware's believed head position is [[$72e2]]: $72e2 holds a POINTER to the
				// position word, which the TRK00 handler at $5DE0 zeroes (movea.w $72e2.w,A0 /
				// move.w #0,(A0)) and several recalibrate sites clear the same way.  Print it on the
				// SAME LINE as the drive's real cylinder - two taps agreeing about position would
				// only be two derived signals, and the whole question is whether these two DISAGREE.
				address_space &cq = m_cpu->space(AS_PROGRAM);
				// The seek is OPEN-LOOP and the position lives in the UIB, not $72e2:
				//   006a56 move.w D1,D6 / 006a58 sub.w ($d6,A6),D6 / 006a5e neg.w D6  -> |target-current|
				//   006a62 move.w D1,($d6,A6)                                          -> current := target
				// A6 is the UIB base (local 0x6e60), so the head position is UIB+$d6 = 0x6f36 and the
				// firmware simply ASSUMES it arrived.  Print it beside the drive's real cylinder.
				// A6 is NOT the UIB local base: the routines load it from [$799a]/[$799c] (see the
				// error path at $16D2/$16EA).  Read both candidate bases and the +$d6 word under
				// each, on the same line, rather than assuming which one the seek uses.
				u16 const b9a = cq.read_word(0x799a), b9c = cq.read_word(0x799c);
				auto rd = [&](u16 b) { return (b >= 0x4000 && b < 0x7f00) ? int(s16(cq.read_word((b + 0xd6) & 0xffff))) : -999; };
				int const fwpos = rd(b9c);
				logerror("STEPEDGE bit0=%d | drive %d->%d %s | [$799a]=%04x pos=%d  [$799c]=%04x pos=%d | target=%d h=%d | moves=%u t=%.5f\n",
					lvl, cyl_before, cyl_after,
					cyl_after != cyl_before ? "MOVED" : "-    ", b9a, rd(b9a), b9c, fwpos,
					cq.read_word(0x7438) & 0xff, cq.read_word(0x7436) & 0xff,
					m_step_moves, machine().time().as_double());
			}

			m_pit[0]->write_gate1(BIT(data, 0));
		}
	}
}

// C000-C7FF: the Multibus host-address up-counter (spec §3.4).  One write loads the one's-complemented
// host address - bits 23-16 on address lines A1-A8, bits 15-0 on the data bus.  Stored decomplemented.
void multibus_storager_device::c000_w(offs_t offset, u16 data, u16 mem_mask)
{
	m_c000 = ~((u32(offset & 0xff) << 16) | data) & 0xffffff;
	m_c000_valid = true;
}

// C800-C9FF: the field-boundary offset file (spec §3.4).  Cell 0 doubles as the live SRAM chunk
// pointer in the per-record capture cycle.  A write while DMA is active latches the transfer terminal.
// The OS driver's command channel (see m_ioreg).  OBSERVE ONLY for now: latch the registers and
// report what the driver sends.  No dispatch until the real command stream has been seen - writing
// a decoder against the HLE's guesses would be building on prior art rather than measurement.
u16 multibus_storager_device::ioreg_r(offs_t offset)
{
	// TEMP: what does the kernel POLL while hung?  The machine goes quiet after a successful
	// completion (IOREG DONE st=80) and issues no further commands, so whatever it is waiting
	// on is read through this register file.  Cap the printing, never the counting.
	{
		static u32 late = 0;
		double const t = machine().time().as_double();
		if (t > 26.0 && late++ < 30)
			logerror("IOREG RD reg%u -> (regs %02x %02x %02x %02x %02x %02x %02x %02x) done=%d t=%.5f\n",
				offset * 2, m_ioreg[0], m_ioreg[1], m_ioreg[2], m_ioreg[3], m_ioreg[4], m_ioreg[5],
				m_ioreg[6], m_ioreg[7], m_ioreg_done ? 1 : 0, t);
	}
	u16 v = u16(m_ioreg[offset * 2]) | (u16(m_ioreg[offset * 2 + 1]) << 8);
	if (offset == 0 && m_ioreg_done)
	{
		v |= 0x80;                      // cmd byte bit7 = controller DONE, which the driver polls
		// Acknowledge ONE completion when the driver reads the command byte; keep INT2 asserted
		// while others remain outstanding so the level-triggered ICU re-enters the handler.
		if (!machine().side_effects_disabled() && m_ioreg_pending > 0)
		{
			int_w<2>(1);
			if (--m_ioreg_pending > 0)
				m_ioreg_int_timer->adjust(attotime::from_usec(20));   // re-edge for the next one
		}
		// reg[1] is the CHANNEL STATUS, read back as (status & 3) with 0 = OK.  It used to be
		// hard-coded to 0 (a cont.536 guess made to stop an infinite retry loop, never
		// validated), so the driver was told every command succeeded even when the firmware had
		// posted an error.  Report the firmware's own verdict: it DMAs the status into the host
		// IOPB at +2 (0x80 = complete, 0x82 = error with the code at +3).
		u8 const st = m_bus->space(AS_PROGRAM).read_byte((0x0fe780 + 2) & 0xffffff);
		v = (v & 0x00ff) | (u16(st == 0x80 ? 0x00 : (st & 0x03)) << 8);
	}
	return v;
}

void multibus_storager_device::ioreg_w(offs_t offset, u16 data, u16 mem_mask)
{
	if (ACCESSING_BITS_0_7)  m_ioreg[offset * 2]     = data & 0xff;
	if (ACCESSING_BITS_8_15) m_ioreg[offset * 2 + 1] = data >> 8;
	if (offset == 0 && ACCESSING_BITS_0_7)
	{
		m_ioreg_done = false;           // new command: DONE drops until it completes
		// RELEASE ANY STALE COMPLETION INTERRUPT.  The acknowledge path in ioreg_r is gated on
		// m_ioreg_done, so if the driver issues its NEXT command before reading the status of
		// the previous one, that interrupt can never be acked: INT2 stays asserted and
		// m_ioreg_pending stays non-zero.  The next completion then finds pending already > 0,
		// its `if (pending == 0) pending = 1` is a no-op, and the timer asserts an ALREADY
		// ASSERTED line - no edge, so the driver is never woken and blocks forever.
		// Whether it happens depends on the driver's read-vs-issue ordering, which is why it
		// presents as non-determinism.  Clearing here guarantees every completion produces a
		// clean edge - the same release the HLE performs at device_reset.
		if (m_ioreg_pending > 0)
		{
			int_w<2>(1);                // deassert (Multibus INT is ACTIVE LOW)
			m_ioreg_pending = 0;
		}
		if (m_ioreg_int_timer) m_ioreg_int_timer->adjust(attotime::never);
		u32 const dev = u32(m_ioreg[4]) | (u32(m_ioreg[5]) << 8) | (u32(m_ioreg[7]) << 16);
		u32 const req = u32(m_ioreg[2]) | (u32(m_ioreg[3]) << 8) | (u32(m_ioreg[6]) << 16);
		address_space &bs = m_bus->space(AS_PROGRAM);
		if (TRACE_HOSTCMD)
		{
			std::string db, rb;
			for (u32 k = 0; k < 0x18; k++) db += util::string_format(" %02x", bs.read_byte((dev + k) & 0xffffff));
			if (req) for (u32 k = 0; k < 0x18; k++) rb += util::string_format(" %02x", bs.read_byte((req + k) & 0xffffff));
			logerror("IOREG CMD=%02x dev=%06x req=%06x regs=%02x %02x %02x %02x %02x %02x %02x %02x |%s t=%.5f\n",
				data & 0xff, dev, req, m_ioreg[0], m_ioreg[1], m_ioreg[2], m_ioreg[3],
				m_ioreg[4], m_ioreg[5], m_ioreg[6], m_ioreg[7], db.c_str(), machine().time().as_double());
			if (req) logerror("IOREG   req-blk@%06x:%s\n", req, rb.c_str());
		}
		// Decode per the HLE (git show 1a6d687d896:.../storager.cpp - ioreg_w @5387, floppy @5686),
		// which measured this channel against real SINIX.  The command fields live in the DEVICE
		// block, not the request block:
		//   dev[0]   op.  THIS IS THE SASI / SCSI-1 GROUP 0 COMMAND SET, not the Interphase command
		//            space: 00 TEST UNIT READY, 01 REZERO, 03 REQUEST SENSE, 04 FORMAT UNIT,
		//            08 READ(6), 0a WRITE(6), 0b SEEK(6), c0/c2 in the vendor-unique range C0-FF.
		//            Every observed op lands on its standard code - structural, not an analogy.
		//            0x0b is therefore SEEK, NOT a data op: the install-proven HLE records the
		//            same and warns that reading it as "write-with-verify" makes a phantom
		//            transfer that overwrites freshly written sectors with stale request-buffer
		//            bytes.  Consistent with measurement here - dropping all 36 0x0b dispatches
		//            changed nothing.
		//            0x0a is WRITE(6) by the same structural reading, but the DIRECTION QUESTION
		//            IS OPEN by standing instruction (Dave: "there are no floppy writes, and we
		//            have not got to the ESDI writes") - and five 0x0a with unit=2 and live counts
		//            were measured anyway.  If the SASI reading is right, those writes are real
		//            and should not be happening, which makes them a SYMPTOM to trace upstream
		//            rather than an opcode to implement.  Do not route 0x0a until that is settled.
		//            Host op != firmware op: the board translates (host 0x08 -> firmware 0x95).
		//            See docs/storager-lle/IOCB-STRUCTURE.md and FIRMWARE-DISPATCH-TABLE.md.
		//   dev[1]   unit<<5 | position-high;  unit 2/3 = floppy, 0/1 = ESDI HD.  Named by the
		//            SMD 2180 as "UNIT/CYLHI SELECT - identifies one of four units and the most
		//            significant bits of the cylinder number".
		//   dev[2-3] position, BIG-ENDIAN, absolute in CURRENT-SIZE (256-byte) sectors
		//   dev[4]   sector count.  Do NOT read [4-7] as a 32-bit count - it pulls in dev[5].
		//   dev[5]   PER-OP, not a constant.  MEASURED over a full run:
		//              op 08 -> 00 (73x)    op 0a -> 00 (5x)     op 0b -> c0 (36x)
		//              op c0 -> 07 ( 4x)    op c2/00/01 -> c0
		//            The old comment here claimed dev[5] is always 0xC0 (a Multibus burst-size
		//            parameter).  That is REFUTED: it is 00 on the read op it mattered most for.
		//            On c0 it is a SUBCODE - 07 = read device/drive status - which is exactly what
		//            the install-proven HLE recorded ("the kernel's sacmd hardcodes subcode 7 for
		//            op 0xc0"), independently corroborated here from the opposite side.
		//   buffer = the request pointer (regs [2],[3],[6])
		//
		// The model decodes only the HOST INTERFACE - that much is board-level - then hands the
		// command to the FIRMWARE in its own IOPB form, through the identical path the monitor's
		// doorbell uses.  The firmware drives the gate array and the flux read; nothing here touches
		// the medium.  Pasting the HLE's C++ transfer back would restore exactly the shim the LLE
		// mandate exists to remove.
		if (dev)
		{
			u8 const op    = bs.read_byte(dev & 0xffffff);
			u8 const unit  = bs.read_byte((dev + 1) & 0xffffff) >> 5;
			u32 const psec = (u32(bs.read_byte((dev + 2) & 0xffffff)) << 8) | bs.read_byte((dev + 3) & 0xffffff);
			u32 const nsec = bs.read_byte((dev + 4) & 0xffffff);
			if (TRACE_HOSTCMD)
			{
				// The firmware head position is UIB+$d6, UIB base = [$799a] (measured 0x6e60), so the
				// word is 0x6f36.  The seek at $6A56 computes |target - THIS| and doubles it into a
				// transition count, so a stale value here sends the head exactly that far wrong.
				// Print it at every command submit to see what the INIT does to it.
				address_space &cp = m_cpu->space(AS_PROGRAM);
				u16 const ub = cp.read_word(0x799a);
				floppy_image_device *const fp = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
				// $72d6 and $72e2 are set by the SAME straight-line builder ($37ae and $3836, no
				// branch or rts between them).  If one is non-zero and the other is not, the fault
				// is in how the model READS them, not in whether the firmware ran.
				// The 0x8A seek gate at $699E reads its command byte through [$71bc], NOT [$7b20]
				// which go_submit stamps.  Log both pointers and the byte each one selects.
				u16 const p1bc = cp.read_word(0x71bc), p7b20 = cp.read_word(0x7b20);
				logerror("IOREG   POS uib=%04x fw-pos=%d | cyl=%d | [$71bc]=%04x->cmd %02x  [$7b20]=%04x->cmd %02x  [$7b24]=%04x t=%.5f\n",
					ub, (ub >= 0x4000 && ub < 0x7f00) ? int(s16(cp.read_word((ub + 0xd6) & 0xffff))) : -999,
					fp ? fp->get_cyl() : -1,
					p1bc, (p1bc >= 0x4000 && p1bc < 0x7f00) ? cp.read_byte(p1bc) : 0xff,
					p7b20, (p7b20 >= 0x4000 && p7b20 < 0x7f00) ? cp.read_byte(p7b20) : 0xff,
					cp.read_word(0x7b24), machine().time().as_double());
				// The IMPLIED SEEK decision ($21DC): the firmware caches its last-accessed head and
				// cylinder in UIB+$d0 / UIB+$d2 and sets [$7b24] only when the REQUEST differs.
				// The manuals confirm implied seeks are how these controllers position
				// (Panther UG: "Overlapped and implied seeks are supported").
				if (ub >= 0x4000 && ub < 0x7f00)
					// $7978 gates the verify-target update at $6AB8 (`cmpi.b #$0,$7978.w`): it is WRITTEN
				// as a WORD (`move.w #$1,$7978` at $220E) and TESTED as a BYTE.  Log BOTH halves and
				// the word, so a swap shows as the 1 sitting in the half the test does not read.
				// (A Lua RAM tap here is silent - bypassed by direct access, cont.557 - so this is
				// measured model-side, which is known to reflect live values.)
				// UIB+$14 bit5 selects between the TWO seek loops ($69E4 `btst #5,D6`):
				//   bit5 CLEAR -> $6A56 path: pos := D1, and D1 then stamps the verify target
				//   bit5 SET   -> $69EC path: pos := D7, leaving D1 STALE for the $6AC0 update
				// The failing psec=240 read shows pos=7 with target=5, the signature of the D7 path.
				if (ub >= 0x4000 && ub < 0x7f00)
					logerror("IOREG   UIB+$14=%02x bit5=%d -> seek path %s\n",
						cp.read_byte((ub + 0x14) & 0xffff), BIT(cp.read_byte((ub + 0x14) & 0xffff), 5),
						BIT(cp.read_byte((ub + 0x14) & 0xffff), 5) ? "A (D7, D1 stale)" : "B (D1)");
				logerror("IOREG   GATE $7978 word=%04x  byte[7978]=%02x byte[7979]=%02x  ($7b24=%04x)\n",
					cp.read_word(0x7978), cp.read_byte(0x7978), cp.read_byte(0x7979), cp.read_word(0x7b24));
				logerror("IOREG   CACHE UIB+$d0(head)=%04x UIB+$d2(cyl)=%04x UIB+$d6(pos)=%04x t=%.5f\n",
						cp.read_word((ub + 0xd0) & 0xffff), cp.read_word((ub + 0xd2) & 0xffff),
						cp.read_word((ub + 0xd6) & 0xffff), machine().time().as_double());
				// cont. handoff: sample verify targets AT SUBMIT, not mid-seek (STEPEDGE reads
				// [$7438] inside the pulse loop at $6A70, BEFORE $6AD0 writes the new target).
				u8 const u12 = (ub >= 0x4000 && ub < 0x7f00) ? cp.read_byte((ub + 0x12) & 0xffff) : 0xff;
				logerror("IOREG   TARGET@SUBMIT [$7438]=%04x [$7436]=%04x UIB+$12=%02x bit1=%d t=%.5f\n",
					cp.read_word(0x7438), cp.read_word(0x7436), u12, BIT(u12, 1),
					machine().time().as_double());
			}
			// THE DRIVER'S OWN PC.  The CPUAP is the bus master executing this register write, so
			// the currently-executing device IS the SINIX driver - its pcbase is the call site that
			// issued the opcode.  That is the entry point for decoding what each host op MEANS,
			// which must come from the driver and not from the HLE's inherited names.
			if (TRACE_HOSTCMD)
			{
				device_execute_interface *const ex = machine().scheduler().currently_executing();
				device_state_interface *st = nullptr;
				u32 wpc = 0;
				if (ex) ex->device().interface(st);
				if (st) wpc = st->pcbase();
				logerror("IOREG   ISSUER op=%02x from %s pc=%06x t=%.5f\n",
					op, ex ? ex->device().tag() : "(none)", wpc, machine().time().as_double());
			}
			logerror("IOREG   op=%02x unit=%u psec=%u nsec=%u buf=%06x t=%.5f\n",
				op, unit, psec, nsec, req, machine().time().as_double());
			// c0 sub-7 status reply (interim shim - see OS_C0_STATUS_FILL).
			if (OS_C0_STATUS_FILL && op == 0xc0 && req)
			{
				u8 const sub = bs.read_byte((dev + 5) & 0xffffff);
				std::string was;
				for (u32 k = 0; k < 8; k++)
					was += util::string_format(" %02x", bs.read_byte((req + k) & 0xffffff));
				for (u32 k = 1; k < 16; k++) bs.write_byte((req + k) & 0xffffff, 0x00);
				bs.write_byte(req & 0xffffff, 0xc1);   // Unit Ready | Present | Drive Ready
				logerror("IOREG C0 STATUS unit=%u sub=%02x reply@%06x was:%s -> c1 t=%.5f\n",
					unit, sub, req, was.c_str(), machine().time().as_double());
			}
			if (op == 0x03 && req)
			{
				// Allocation length is 0 (measured cdb: 03 40 00 00 ...), which for class-0 sense
				// means the 4-byte default.  AV stays CLEAR: we have no verified failing block to
				// report, and asserting it would claim LOGBN is meaningful when it is not.
				u8 const b0 = m_last_sense_valid ? m_last_sense : 0x00;
				u8 const b1 = u8((unit & 7) << 5);
				bs.write_byte((req + 0) & 0xffffff, b0);
				bs.write_byte((req + 1) & 0xffffff, b1);
				bs.write_byte((req + 2) & 0xffffff, 0x00);
				bs.write_byte((req + 3) & 0xffffff, 0x00);
				// THE SHARED MAILBOX STILL HOLDS THE LAST FAILURE.  MEASURED: at the 0x03 the host
				// IOPB at 0x0fe780 still reads +2=82 +3=1c from the write that just failed, because
				// only firmware-backed completions (HOST POST) refresh it - a synchronous command
				// inherits the previous verdict.  The driver reads that, treats the sense fetch as
				// failed, and never loads the buffer (4 reads against 103 commands).  Post a clean
				// completion for THIS command so the reply is collectable.
				bs.write_byte(0x0fe782, 0x80);
				bs.write_byte(0x0fe783, 0x00);
				logerror("IOREG SENSE served unit=%u -> %02x %02x 00 00 (fwop %02x, ETYPE=%u ECODE=%X) t=%.5f\n",
					unit, b0, b1, m_last_sense_op, (b0 >> 4) & 7, b0 & 0xf,
					machine().time().as_double());
			}
			// ---- ESDI / rigid-disk channel (unit 0/1) -------------------------------------
			// Geometry and decode differ from the floppy and are taken from the HLE, which drove a
			// full multi-user install (see docs/storager-lle/GATEARRAY-READ-SPEC.md):
			//   position is 21 BITS - the top 5 live in the unit byte.  Dropping them wraps every
			//   track past 64 MB and sends the format's verify pass into an endless spare-assignment
			//   loop.  Sector size is 1024 (the c2 SET-CONFIG declares it), not the floppy's 256.
			//   dev[5] = 0xC0 is the Multibus burst-size parameter and is NOT part of the count.
			//   count 0 means 4, NOT 256 - a 256 reading sprays 256 KB of kernel memory onto the
			//   platter (the HLE confirmed this from scattered superblock images on the disk).
			if (unit < 2 && (op == 0x08 || op == 0x0a || op == 0x0b || op == 0xc0 || op == 0xc2))
			{
				u32 const hpsec = (u32(bs.read_byte((dev + 1) & 0xffffff) & 0x1f) << 16)
				                | (u32(bs.read_byte((dev + 2) & 0xffffff)) << 8)
				                |  u32(bs.read_byte((dev + 3) & 0xffffff));
				u32 hnsec;
				if (op == 0x0b)   // 0x0b (direction undecided): dev[4-7] LE is a BYTE count
				{
					u32 const bytes = u32(bs.read_byte((dev + 4) & 0xffffff))
					                | (u32(bs.read_byte((dev + 5) & 0xffffff)) << 8)
					                | (u32(bs.read_byte((dev + 6) & 0xffffff)) << 16)
					                | (u32(bs.read_byte((dev + 7) & 0xffffff)) << 24);
					hnsec = (bytes + 1023) / 1024;
				}
				else
					hnsec = bs.read_byte((dev + 4) & 0xffffff);
				// SILENT CAPS ARE THE PRIME SUSPECT FOR "the write landed but the install stalls".
				// A >64 clamp means we transfer 64 sectors, report SUCCESS, and the driver believes
				// everything it asked for is on the platter.  The disk then looks plausibly written
				// while being short - which is exactly the shape of a mkfs that finishes its pass
				// and then blocks.  Never truncate quietly: say what was dropped.
				u32 const nsec_raw = hnsec;
				bool const is_xfer = (op == 0x08 || op == 0x0a);   // c0/c2/0b move no sectors
				if (hnsec == 0) hnsec = 4;
				if (hnsec > 64)
				{
					logerror("ESDI  *** COUNT CLAMPED *** op=%02x unit=%u psec=%u asked=%u -> %u "
						"(%u sectors / %u KB DROPPED, completion still reports success) t=%.5f\n",
						op, unit, hpsec, nsec_raw, 64u, nsec_raw - 64, nsec_raw - 64,
						machine().time().as_double());
					hnsec = 64;
				}
				if (TRACE_ESDI)
					logerror("ESDIOP op=%02x unit=%u psec=%u nsec_raw=%u nsec_used=%s req=%06x%s t=%.5f\n",
						op, unit, hpsec, nsec_raw,
						is_xfer ? util::string_format("%u", hnsec).c_str() : "-",
						req, (is_xfer && nsec_raw == 0) ? "  [count0->4]" : "",
						machine().time().as_double());
				logerror("ESDI op=%02x unit=%u psec=%u (byte %llu) nsec=%u buf=%06x t=%.5f\n",
					op, unit, hpsec, (unsigned long long)(u64(hpsec) * 1024), hnsec, req,
					machine().time().as_double());
				// DATA PHASE.  The gate array is the DMA engine: the firmware issues the ESDI
				// command, the gate array moves the bytes.  On the floppy that means decoding flux;
				// on the rigid disk the image IS the drive surface, so the transfer is a flat
				// sector-array copy at psec*1024.  This is NOT the HLE shim - that bypassed the
				// firmware's command processing entirely; here the host interface is decoded and the
				// data phase serviced, which is the gate array's own role.
				//
				// Required by the installer's DISK-TYPE AUTO-DETECT: it writes 48 sectors at psec 0
				// (op 0x0b) and reads 1 back (op 0x08).  With no data moved the read-back is garbage
				// and detection fails - measured, "Es war dem System leider nicht moeglich, den
				// Plattentyp automatisch zu erkennen."
				//
				// FORMAT REMAINS SIGNAL-ONLY (standing instruction): there is no format opcode, no
				// track structure / spare table / defect map is laid down, and the 0x1a pattern is not
				// special-cased.  Pattern writes simply land in the flat image, which keeps the
				// installer's own read-back verify honest.
				if (unit < 2 && m_hd[unit] && m_hd[unit]->exists())
				{
					std::vector<u8> sec(size_t(hnsec) * 1024, 0);
					if (op == 0x08)
					{
						m_hd[unit]->img_read(u64(hpsec) * 1024, sec.data(), u32(sec.size()));
						for (size_t k = 0; k < sec.size(); k++)
							bs.write_byte((req + u32(k)) & 0xffffff, sec[k]);
					}
					else if (op == 0x0a)
					{
						// 0x0a is SASI WRITE(6) and is the ONLY op that puts data on this platter.
						// Writing here is expected - the installer's format is meant to overwrite.
						for (size_t k = 0; k < sec.size(); k++)
							sec[k] = bs.read_byte((req + u32(k)) & 0xffffff);
						m_hd[unit]->img_write(u64(hpsec) * 1024, sec.data(), u32(sec.size()));
					}
					// 0x0b USED TO WRITE HERE.  It is SASI SEEK(6) and transfers nothing, so that
					// copied whatever stale bytes were in the request buffer onto the platter at the
					// seek target - and with `hnsec == 0 -> 4` a count-0 seek scribbled 4 KB.
					// Measured: `ESDI op=0b unit=0 psec=0 nsec=48` put 48 KB of stale buffer at LBA 0.
					// This is exactly the failure the install-proven HLE left a warning about: "the
					// mkfs-era count-0 SEEKs were long misread as write-with-verify data ops; the
					// real data always arrives in the accompanying per-buffer 0x0A writes (a phantom
					// transfer here overwrites freshly-written sectors with stale request-buffer
					// bytes)".  A format is precisely the workload that seeks between writes, so the
					// damage landed behind the head on already-formatted ground.
					logerror("ESDI   %s %u sec @%llu first8: %02x %02x %02x %02x %02x %02x %02x %02x\n",
						(op == 0x08) ? "read " : (op == 0x0a) ? "write" : (op == 0x0b) ? "seek " : "ctl  ", hnsec, (unsigned long long)(u64(hpsec) * 1024),
						sec[0], sec[1], sec[2], sec[3], sec[4], sec[5], sec[6], sec[7]);
				}
				m_ioreg[1] = 0x00;
				m_ioreg_done = true;
				if (m_ioreg_int_timer)
				{
					if (m_ioreg_pending == 0) m_ioreg_pending = 1;
					// 500us, NOT 20us: the HLE defers by 500us - "deferred so it can't beat the driver
					// into its wait".  Firing early LOSES the interrupt (the driver has not reached its
					// wait yet) and it then blocks forever.  Being a RACE, it presents as non-determinism:
					// identical code, sometimes past the j/n prompt, sometimes locked before disk select.
					m_ioreg_int_timer->adjust(attotime::from_usec(500));
				}
				return;
			}
			// ---- host 0x0b SEEK -> firmware 0x8A SEEK (OS_ROUTE_SEEK) ---------------------
			// The best-evidenced mapping we have, from two independent identifications:
			//   * host 0x0b is SASI SEEK(6) - the whole OS op set is SASI Group 0, and the
			//     install-proven HLE names 0x0b SEEK and warns that reading it as a data op makes
			//     a phantom transfer over freshly written sectors.  Measured: nsec is 0 on all 36.
			//   * firmware 0x8A is SEEK - $92 table entry `8a: 5f9c0066`, and this file's own
			//     seek-gate comments already name 0x8A ("the 0x8A seek gate at $699E").
			// A seek moves the head and transfers nothing, so unlike the 0x96 write experiment it
			// needs no write engine and cannot touch the medium.
			//
			// ADDRESSING IS CHS HERE, NOT LINEAR.  The table's mode word for 8a is 0x0066 - bit0
			// CLEAR - where 95/96 carry 0x8c27/0xc827 with bit0 SET.  MODEFORK reads UIB+$20 bit0
			// (the firmware stages it from the table, not from IOPB[1]), so a seek wants a real
			// cylinder/head, built the way the boot ROM builds one at FE3C20-FE3C67: divide the
			// address by the per-cylinder size for CYL, then MOD/DIV for HEAD.  Sector IDs are
			// track-relative and 1-based on this medium.
			if (op == 0x0b && unit >= 2)
			{
				address_space &cs3 = m_cpu->space(AS_PROGRAM);
				u32 const spt = m_uib_base ? cs3.read_byte((m_uib_base + 1) & 0xffff) : 16;
				u32 const hds = m_uib_base ? cs3.read_byte(m_uib_base & 0xffff) : 2;
				if (spt && hds)
				{
					u32 const cyl = psec / (spt * hds);
					u32 const hd  = (psec / spt) % hds;
					u32 const sec = (psec % spt) + 1;
					u8 iopb[0x18] = {};
					iopb[0x00] = 0x8a;
					iopb[0x04] = unit;
					iopb[0x05] = u8(hd);
					iopb[0x06] = u8(cyl >> 8);  iopb[0x07] = u8(cyl);
					iopb[0x08] = u8(sec >> 8);  iopb[0x09] = u8(sec);
					iopb[0x0c] = 0x14;          // DMA COUNT, the value the boot ROM writes
					constexpr u32 HOST_IOPB = 0x0fe780;
					for (u32 k = 0x10; k < 0x18; k++)
						iopb[k] = bs.read_byte((HOST_IOPB + k) & 0xffffff);
					for (u32 k = 0; k < 0x18; k++)
						bs.write_byte((HOST_IOPB + k) & 0xffffff, iopb[k]);
					u32 dst = cs3.read_word(0x7a06);
					if (dst < 0x4000 || dst >= 0x7e00) dst = 0x71f0;
					for (u32 k = 0; k < 0x18; k++)
						cs3.write_byte((dst + k) & 0xffff, iopb[k]);
					m_iopb_cmd = 0x8a;
					logerror("IOREG   -> firmware SEEK cmd=8a unit=%u psec=%u -> cyl=%u hd=%u sec=%u "
						"(spt=%u hds=%u) dst=%04x t=%.5f\n",
						unit, psec, cyl, hd, sec, spt, hds, dst, machine().time().as_double());
					m_os_fw_pending = true;
					m_ioreg_done = false;
					mailbox_submit(HOST_IOPB, dst);
				}
				else
				{
					// NEVER SKIP SILENTLY.  With no geometry there is no CHS to build, so the seek
					// falls back to the old sync-ACK - i.e. the wedge is armed again for that
					// command.  A quiet fallthrough here would look exactly like "the fix works"
					// right up until it doesn't, which is the false-empty failure this campaign
					// keeps paying for.  Expected to be silent; if it ever fires, that is the bug.
					logerror("IOREG   SEEK NOT ROUTED: no geometry (spt=%u hds=%u uib=%04x) - "
						"falling back to sync-ACK, latch clearer NOT restored for this command "
						"t=%.5f\n", spt, hds, m_uib_base, machine().time().as_double());
				}
			}
			else if ((op == 0x08 || op == 0x0a) && unit >= 2 && nsec)
			{
				// The firmware's own read IOPB (STEP 134 layout - the one the boot read uses):
				// [0] cmd 0x95, [1] bit0 = ADDRESSED, [4] unit, [6-9] BE32 address in current-size
				// sectors, [0x0a-0x0b] BE16 count, [0x0d-0x0f] host buffer.
				//
				// WRITE (host 0x0a) TAKES THE SAME ENVELOPE WITH THE MIRROR COMMAND.  The ROM's own
				// dispatch table at $92 pairs the ladders exactly:
				//     read  81 -> 5fc0 8c26      write  82 -> 6102 c826
				//           95 -> 5fc0 8c27             96 -> 6102 c827
				// so 0x96 is the structural counterpart of the 0x95 this path already drives
				// successfully.  Derived from the board's table, not from a manual analogy.
				u8 iopb[0x18] = {};
				u8 const fwcmd = (op == 0x08) ? 0x95 : 0x96;
				iopb[0x00] = fwcmd;
				// The IOPB must LIVE IN HOST MEMORY: the firmware's completion is a bus-master DMA back
				// into it (HOST POST: node+2=81 busy -> node+2=80 done), so an IOPB that exists only in the
				// node work area has nowhere to post and the command never completes.  That, not any field
				// value, is why the read engaged and then stalled - all four cont.547 A/B arms came back
				// byte-identical because the defect was in the ENVELOPE, not in the request.
				//
				// [1] = 0x00: every DELIVERING boot read is SEQUENTIAL.  The addressed (bit0) form belongs to
				// the loader's own file read (STEP 134) - a different command on a different channel.
				//
				// This channel's driver builds a device block + request block, NOT a monitor IOPB, so there
				// is no host IOPB to point at and the model has to place one.  PROVISIONAL: the monitor's own
				// slot, idle once the loaded driver has taken over.  Bytes 0x10-0x17 are two LE32 host
				// pointers present in every working IOCB - preserve what the monitor left there rather than
				// zeroing them, and log the slot before overwriting so any reuse by the kernel shows up as
				// changed bytes instead of silent corruption.
				// SEQUENCING PROBE (cont.549, STRIP once answered).  Addressed mode is relative to the base
				// latched at the last INIT, and this channel has issued none - the last INIT was the
				// monitor's, aimed at the loader's file.  So an INIT must precede the read.  The firmware
				// takes one command at a time, and the driver already retries every 133 ms, so its OWN
				// retries sequence the pair: first request submits the INIT, the next submits the read.
				// If INIT is the missing piece the second retry delivers; a queue can be built properly
				// afterwards.  The 87 IOPB is the boot's, verbatim from the log - [0x0d-0x0f] is the UIB
				// pointer, the same field the read uses for its host buffer.
				// NO SYNTHESIZED INIT.  An INIT here is actively HARMFUL: it invalidates the
				// firmware's cached head/cylinder (UIB+$d0/$d2) to the position-unknown sentinel,
				// forcing a recalibrate to track 0 instead of the short positioning seek the request
				// actually needs.  Measured either side of a synthesized 87:
				//   before: UIB+$d0=0001 UIB+$d2=0002  (== where the drive is)
				//   after:  UIB+$d0=ffff UIB+$d2=ffff  (unknown)
				// The boot's own INIT already latched the base, and these controllers position by
				// IMPLIED SEEK - the Interphase 4201 Panther user's guide states "Overlapped and
				// implied seeks are supported", and $21DC compares the requested cylinder/head against
				// the UIB cache, setting [$7b24] when they differ, which is what enters the seek at
				// $69C8.  Leave the cache alone and the read positions itself: measured 2->3->4->5 with
				// SEEK CHECK MATCH at cyl 5, and deliveries 69 -> 88.
				constexpr u32 HOST_IOPB = 0x0fe780;
				iopb[0x01] = 0x01;   // ADDRESSED - the base is now latched by the INIT above
				iopb[0x04] = unit;
				// The address is the request's own, unmodified.  A `STORAGER_PARTBASE` env override lived
				// here to test a suspected 128-block partition origin; REMOVED - its premise was
				// retracted (the bytes read as a superblock at block 288 were ASCII, 0x2e = '.').
				// No env gate belongs in the read path.
				u32 const eff = psec;
				iopb[0x06] = u8(eff >> 24); iopb[0x07] = u8(eff >> 16);
				iopb[0x08] = u8(eff >> 8);   iopb[0x09] = u8(eff);
				iopb[0x0a] = u8(nsec >> 8);  iopb[0x0b] = u8(nsec);
				// THE DATA BUFFER IS i_ma AT iob+0xAA.  HLE STEP 142/143 field map (NS32k sys.c):
				//     +0x9A i_boff   +0xA6 i_bn   +0xAA i_ma (BUFFER)   +0xAE i_cc (bytes)
				// Measured on the superblock read: i_bn=16, psec=160, i_ma=0xBD8C, i_cc=0x2000.
				// i_bn is the partition-relative 1K block (base 24 + 16 = FS block 40 = psec 160);
				// it is NOT equal to psec.  The previous validation required i_bn == psec, so it
				// NEVER adopted i_ma and always fell back to `req` - which is how the loader saw
				// correct media bytes at the wrong host address.
				//
				// Adopt i_ma when it looks like a host buffer and i_cc matches the sector count
				// (or is zero / unset).  Otherwise keep reg[2,3,6] (= req), which the HLE treats
				// as i_ma when the driver wired them that way.
				auto le32 = [&](u32 a) {
					return u32(bs.read_byte(a & 0xffffff)) | (u32(bs.read_byte((a+1) & 0xffffff)) << 8)
					     | (u32(bs.read_byte((a+2) & 0xffffff)) << 16) | (u32(bs.read_byte((a+3) & 0xffffff)) << 24);
				};
				u32 const i_bn = le32(dev + 0xa6), i_ma = le32(dev + 0xaa), i_cc = le32(dev + 0xae);
				u32 const want_bytes = nsec * 256;
				bool const ma_plausible = i_ma != 0 && i_ma < 0x100000;
				bool const cc_ok = (i_cc == 0) || (i_cc == want_bytes);
				bool const map_ok = ma_plausible && cc_ok;
				u32 const dstbuf = map_ok ? i_ma : req;
				logerror("IOB MAP i_bn=%u (psec=%u) i_ma=%06x i_cc=%u (cnt bytes=%u) -> %s buffer %06x\n",
					i_bn, psec, i_ma, i_cc, want_bytes,
					map_ok ? "using i_ma" : "keeping req", dstbuf);
				// The commanded host extent, for the out-of-extent DMA guard below.
				m_cmd_buf = dstbuf;
				m_cmd_bytes = want_bytes;
				m_bound_logged = false;
				m_extra_done = false;
				m_extra_active = false;
				iopb[0x0d] = u8(dstbuf >> 16);  iopb[0x0e] = u8(dstbuf >> 8); iopb[0x0f] = u8(dstbuf);
				std::string prev;
				for (u32 k = 0; k < 0x18; k++)
					prev += util::string_format(" %02x", bs.read_byte((HOST_IOPB + k) & 0xffffff));
				for (u32 k = 0x10; k < 0x18; k++)
					iopb[k] = bs.read_byte((HOST_IOPB + k) & 0xffffff);
				for (u32 k = 0; k < 0x18; k++)
					bs.write_byte((HOST_IOPB + k) & 0xffffff, iopb[k]);
				address_space &cs2 = m_cpu->space(AS_PROGRAM);
				u32 dst = cs2.read_word(0x7a06);
				if (dst < 0x4000 || dst >= 0x7e00)
					dst = 0x71f0;
				for (u32 k = 0; k < 0x18; k++)
					cs2.write_byte((dst + k) & 0xffff, iopb[k]);
				m_iopb_cmd = fwcmd;
				logerror("IOREG   host IOPB@%06x was:%s\n", HOST_IOPB, prev.c_str());
				logerror("IOREG   -> firmware IOPB cmd=%02x (host op %02x) unit=%u addr=%u cnt=%u buf=%06x dst=%04x opts=%02x t=%.5f\n",
					fwcmd, op, unit, psec, nsec, dstbuf, dst, iopb[0x01], machine().time().as_double());
				// Firmware path is asynchronous.  Leave DONE clear; HOST POST raises it.
				m_os_fw_pending = true;
				m_ioreg_done = false;
				m_id_cmp_gen++;
				m_id_cmp_n = 0;
				// WHICH SIDE OWNS THE +1?  The command wants cyl 39 (psec 1264) while [$7438] and the
				// UIB cache both read 40, so no seek is issued and the wrong track is captured.
				// Log the address bytes we hand the firmware AND [$7438] on BOTH sides of the
				// submit: if [$7438] is UNCHANGED the firmware never recomputed it and we are
				// relying on a stale target; if it CHANGES to 40 then our address produces 40 and
				// the addressing base is wrong.  These need opposite fixes.
				{
					u32 const spt2 = m_uib_base ? cs2.read_byte((m_uib_base + 1) & 0xffff) : 16;
					u32 const hds2 = m_uib_base ? cs2.read_byte(m_uib_base & 0xffff) : 2;
					logerror("IOPBCYL BEFORE: iopb[6-9]=%02x %02x %02x %02x (addr=%u) -> cyl %u h %u s %u | "
						"[$7438]=%04x [$7436]=%04x UIB+$d2=%04x\n",
						iopb[6], iopb[7], iopb[8], iopb[9], psec,
						(spt2 && hds2) ? psec / (spt2 * hds2) : 0,
						(spt2 && hds2) ? (psec / spt2) % hds2 : 0,
						spt2 ? (psec % spt2) + 1 : 0,
						cs2.read_word(0x7438), cs2.read_word(0x7436),
						m_uib_base ? cs2.read_word((m_uib_base + 0xd2) & 0xffff) : 0xffff);
				}
				mailbox_submit(HOST_IOPB, dst);
				{
					logerror("IOPBCYL AFTER : [$7438]=%04x [$7436]=%04x UIB+$d2=%04x UIB+$d6=%04x\n",
						cs2.read_word(0x7438), cs2.read_word(0x7436),
						m_uib_base ? cs2.read_word((m_uib_base + 0xd2) & 0xffff) : 0xffff,
						m_uib_base ? cs2.read_word((m_uib_base + 0xd6) & 0xffff) : 0xffff);
				}
			}
		}
		// Synchronous/no-firmware commands complete here.  Firmware-backed floppy reads leave
		// DONE clear until HOST POST (see m_os_fw_pending above).  Setting DONE immediately was
		// correct for the HLE's in-line transfer and catastrophic for the LLE: the loader polled
		// bit7, found it set, and validated a still-empty superblock buffer.
		if (!m_os_fw_pending)
		{
			m_ioreg[1] = 0x00;
			m_ioreg_done = true;
			// cmd bit0 = INTERRUPT-DRIVEN: count the completion and arm the deferred INT2
			// (Multibus INT2 -> CPUAP ICU IR3 = the kernel's sa handler).  Deferred 500us so it
			// cannot arrive before the driver reaches its wait.  Polled commands (bit0 = 0, which
			// is all the boot loader issues) complete via the DONE bit alone - unchanged.
			if (data & 1)
			{
				m_ioreg_pending++;
				m_ioreg_int_timer->adjust(attotime::from_usec(500));
				logerror("IOREG INT armed (pending=%d) t=%.5f\n", m_ioreg_pending, machine().time().as_double());
			}
		}
		else if (data & 1)
		{
			// Interrupt-driven OS command: arm INT2 when HOST POST completes (below), not now.
			// Count it so the pending depth is right when the post fires.
			m_ioreg_pending++;
			logerror("IOREG INT deferred until HOST POST (pending=%d) t=%.5f\n",
				m_ioreg_pending, machine().time().as_double());
		}
	}
}

u16 multibus_storager_device::c800_r(offs_t offset)
{
	u16 const d = m_c800[offset & 0xff];
	return d;
}

void multibus_storager_device::c800_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_c800[offset & 0xff]);

	// A held first field flushes as soon as the firmware arms a chunk: the arm is what gives it a
	// destination, and until then it has none. (cont.437)
	if ((offset & 0xff) == 0 && m_held_len)
	{
		address_space &cs = m_cpu->space(AS_PROGRAM);
		u32 dst = u32(m_c800[0]) << 1;
		// cont.517: the arm we are seeing NOW serves the NEXT record, not the held one.  By this point
		// the firmware has written the held record's claim ($7696 table: +0 chunk byte address,
		// +2 claimed sector index), which is what the host DMA sources from - so flush THERE.  At
		// deposit time the claim did not exist yet (measured: written ~7us after the deposit), which
		// is why a deposit-time lookup cannot work and the hold is what makes this reachable.
		// HOLD_UNTIL_ARMED - REFUTED - body removed; the code is preserved verbatim in commit bebdc9428d9
		// (pre-cleanup checkpoint).  Kept as a comment so the verdict survives without
		// dead machinery that reads as live code.
		if (dst >= 0x4000 && dst + m_held_len <= 0x8000)
		{
			for (u32 k = 0; k < m_held_len; k++)
				cs.write_byte((dst + k) & 0xffff, m_held_data[k]);
			logerror("held field (R=%u) flushed into chunk %04x t=%.5f\n",
				m_held_r, dst, machine().time().as_double());
			m_held_len = 0;
		}
	}

	// C800 is a set of write PORTS into the field sequencer, not a 256-cell file: op18 pushes a
	// descriptor stream through three offsets, writing the SAME port repeatedly (cell 0 six times over,
	// cell $3F eight times) - as storage all but the last would be lost, so each write is a push.  A
	// push is framed by E800's load-enable (the ori #$80 op18 issues before every descriptor); a C800
	// write WITHOUT it is an ordinary register access - the per-record chunk arm the record ISR issues
	// through cell 0.  Recorded in push order; what the descriptors program is not yet decoded.
	if (BIT(m_ch[(0xe800 - 0xe000) / 2], 7) && m_desc_n < std::size(m_desc_port))
	{
		m_desc_port[m_desc_n] = u8(offset & 0xff);
		m_desc_val[m_desc_n] = data;
		m_desc_n++;
	}
	// cont.491: the table AT THE MOMENT OF THE LOOKUP - cell 0 is published one instruction after
	// the fetch, so this is state-at-point-of-use.  Read as WORDS: the firmware fetches with
	// move.w (A0,D0.w),D0, which does NOT go through the byte-swapped mapping.
	if (TRACE_CHUNK_PATH && (offset & 0xff) == 0)
	{
		address_space &tc = m_cpu->space(AS_PROGRAM);
		// cont.496: was hard-coded to 7 and SILENTLY TRUNCATED the table - 12 distinct chunks are
		// armed across a run, so entries exist at least to index 11.  Dump 16 and let the contents
		// show where it ends; never assume a length for a structure you are trying to identify.
		char tb[16 * 20 + 1]; tb[0] = 0; int tp = 0;
		for (int e = 0; e < 16; e++)
			tp += sprintf(tb + tp, "[%d]%04x,%04x,%04x ", e,
				tc.read_word(0x7696 + e * 6), tc.read_word(0x7698 + e * 6),
				tc.read_word(0x769a + e * 6));
		floppy_image_device *const tf = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
		logerror("TBL@USE c800[0]<=%04x (dst=%04x) cyl=%d %s t=%.5f | %s\n",
			data, data * 2, tf ? tf->get_cyl() : -1, flux_density_fm() ? "FM" : "MFM",
			machine().time().as_double(), tb);
	}
	if (TRACE_CHUNK_PATH && m_cpu->pcbase() == 0x30e0)   // TEMP (STRIP): op18's triple loop
		logerror("TEMPC800 cell[%02x] <= %04x  A2=%06x D1=%02x D0=%02x D5=%02x [7938]=%04x\n",
			offset & 0xff, data, u32(m_cpu->state_int(M68K_A2)),
			u32(m_cpu->state_int(M68K_D1)) & 0xff, u32(m_cpu->state_int(M68K_D0)) & 0xff,
			u32(m_cpu->state_int(M68K_D5)) & 0xff, m_cpu->space(AS_PROGRAM).read_word(0x7938));
	else if (TRACE_CHUNK_PATH)
		logerror("TEMPC800 cell[%02x] <= %04x pc=%06x\n", offset & 0xff, data, m_cpu->pcbase());   // TEMP (STRIP)
	if (m_dma_active)
		m_dma_term = (u16((offset & 0x7f) * 2) << 6) | ((data & 0x3f) << 1);
}

// D000: local SRAM word-address latch (the IOPB work area / capture area word address).  Write-only.
void multibus_storager_device::d000_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_d000);
}

// D800: parameter/status template word-address latch ($7DAC>>1 on every traced write).  Local, not
// host-high - host addresses ride the C000 counter (spec §3.4).  Write-only.
void multibus_storager_device::d800_w(offs_t offset, u16 data, u16 mem_mask)
{
	u16 const was = m_d800;
	COMBINE_DATA(&m_d800);
	// RE-STAGE WHEN THE FIRMWARE MOVES THE STAGING POINTER (write path only).
	// MEASURED: at write first-arm D800 still holds the READ base (3ed6 -> $7DAC); the firmware
	// only re-points it to the write base (3ecf -> $7D9E) later, at $97C8 in its arm/retry path.
	// Anything staged before that lands in the read buffer while the ISR reads the write buffer
	// and finds 00 - which is exactly the 2-of-5 `$A1` sync failures.  Staging EARLIER cannot fix
	// it (that was measured as a no-op); the record has to be re-staged at the address the
	// firmware has just chosen.
	// Read path deliberately untouched: it stages through deliver_mark() as it always has, and
	// this fires only while a write command is current.
	// cont.569: D800 on the write path is dual-use.
	//   (1) value change 3ed6→3ecf: select write staging base ($7DAC→$7D9E); re-stage.
	//   (2) same-value store at $97C8: measured end of the 1.81s post-WRLOC gap (attempt 1
	//       WRLOC r=0e → silence → D800SEL 3ecf→3ecf restage=0 → WRDATA 4us later).  The
	//       store IS the re-arm signal; m_d800!=was discarded it and left IRQ6 pending with
	//       armed=0 until a soft-timer path re-entered $97C8.  Staging was already correct
	//       (7d9e) — not a buffer-identity bug.
	if (cmd_is_write())
	{
		bool const value_changed = (m_d800 != was);
		bool const will_restage = (m_write_active && value_changed && m_track_n > 0);
		bool const same_rearm = (m_write_active && !value_changed && m_mark_pending != 0);
		logerror("D800SEL %04x -> %04x (dst %04x -> %04x) write_active=%d restage=%d "
			"same_rearm=%d pend=%d phase=%d armed=%d done=%d/%d pc=%06x t=%.5f\n",
			was, m_d800, u32(was) << 1, u32(m_d800) << 1, m_write_active ? 1 : 0,
			will_restage ? 1 : 0, same_rearm ? 1 : 0, m_mark_pending, m_sec_phase,
			m_armed ? 1 : 0, m_wr_done, m_sec_count,
			m_cpu->pcbase() & 0xffffff, machine().time().as_double());
		if (will_restage)
			stage_record(m_track[m_sec_index % m_track_n]);
		if (same_rearm)
		{
			m_armed = true;
			deliver_mark();
			advance_read();
		}
	}
}

// ---------------------------------------------------------------------------
// host / Multibus interface
// ---------------------------------------------------------------------------

// The doorbell GO tail, shared by BOTH command channels.  Everything from the node-pointer
// stamp to the IRQ2 assert is gate-array behaviour and is identical whichever host window
// submitted the command, so the 0x0800 OS-driver channel runs the SAME path the monitor's
// doorbell does - the firmware then drives the read, which is the whole point.
// Deliver a synthesized command the way the HARDWARE delivers one.  The firmware's dispatcher
// (@0x24c6) edge-detects on the MAILBOX BYTE against its own state variable - it does not scan the
// node work area - so a command stuffed straight into the node is invisible to it.  That is why an
// INIT submitted without this produced HOST GO and then absolutely nothing, where the boot's own 87
// fetches a UIB and posts completion.  Write the ch1 registers into fw RAM exactly as the doorbell
// window does (R0 = GO, R1-R3 = the IOPB pointer), mark the channel busy, then run the shared tail.
void multibus_storager_device::mailbox_submit(u32 host_iopb, u32 dst)
{
	address_space &cs = m_cpu->space(AS_PROGRAM);
	cs.write_byte(0x7ff8, 0x13);
	cs.write_byte(0x7ffa, u8(host_iopb >> 16));
	cs.write_byte(0x7ffc, u8(host_iopb >> 8));
	cs.write_byte(0x7ffe, u8(host_iopb));
	m_mb_in[0] = 0x13;
	m_mb_in[1] = u8(host_iopb >> 16);
	m_mb_in[2] = u8(host_iopb >> 8);
	m_mb_in[3] = u8(host_iopb);
	m_r0_busy = true;
	go_submit(dst, host_iopb);
}

void multibus_storager_device::go_submit(u32 dst, u32 dbi)
{
	address_space &cs = m_cpu->space(AS_PROGRAM);
	address_space &bs = m_bus->space(AS_PROGRAM);
	cs.write_word(0x7b20, dst);
	m_iopb_cmd = cs.read_byte(dst);
	// TEMP cont.432: the IOPB->node copy is a model<->SRAM byte boundary the rule never enumerated.
	// cs.write_byte goes through the swapped mapping, bs.read_byte does not, so node WORD reads see
	// swapped halves while node BYTE reads round-trip correctly.  Check the $5FC0 guard's inputs
	// (node+$a/$b, both BYTE reads) against the host's own bytes - the IOCB is known good from the
	// HLE, so a mismatch is ours.
	{
		// TEMP cont.437: the whole IOCB, so the REQUESTED starting sector is visible.  The label
		// lives in sector R=7 on this media; if the request starts at 7 the label belongs at host
		// position 0, and delivering a run from r=01 is the defect.
		char h[3 * 0x18 + 1]; h[0] = 0;
		for (u32 k = 0; k < 0x18; k++)
			sprintf(h + k * 3, "%02x ", cs.read_byte((dst + k) & 0xffff));
		logerror("IOCB cmd=%02x: %s\n", m_iopb_cmd, h);
	}
	logerror("IOPB->node cmd=%02x  node+a=%02x node+b=%02x  host+a=%02x host+b=%02x  guard=%s"
		"  node+20 word=%04x (bit14=%d)\n",
		m_iopb_cmd,
		cs.read_byte((dst + 0x0a) & 0xffff), cs.read_byte((dst + 0x0b) & 0xffff),
		bs.read_byte((dbi + 0x0a) & 0xffffff), bs.read_byte((dbi + 0x0b) & 0xffffff),
		((cs.read_byte((dst + 0x0a) & 0xffff) | cs.read_byte((dst + 0x0b) & 0xffff)) == 0)
			? "*** BOTH ZERO - $5FC0 WOULD BAIL ***" : "passes",
		cs.read_word((dst + 0x20) & 0xffff),
		BIT(cs.read_word((dst + 0x20) & 0xffff), 14));
	logerror("HOST GO: cmd=%02x iopb=%06x t=%.5f\n", m_iopb_cmd, dbi, machine().time().as_double());
	// cont.526: seed the linear walk ONCE PER COMMAND.  Total from the NODE count ([$7abc] is the
	// per-track remainder and cannot bound a multi-track run); head from the firmware's select.
	if (m_iopb_cmd == 0x95)
	{
		address_space &wn = m_cpu->space(AS_PROGRAM);
		u32 const wb = m_node_base ? m_node_base : 0x71f0;
		m_blocks_left = (int(wn.read_byte((wb + 0x0a) & 0xffff)) << 8)
		              | int(wn.read_byte((wb + 0x0b) & 0xffff));
		m_walk_head = m_sel_head & 1;
		m_cmd_cyl = 0xffff;   // sequential: no cylinder gate (already on-cylinder by retention)
		// AN ADDRESSED READ STARTS AT THE SECTOR IT ASKED FOR.  A controller reads in natural
		// sector order: seek, select the head, then wait for the REQUESTED sector to come round
		// and read ascending from there - it does not start at whatever is under the head.  The
		// hunt start was being RETAINED from the previous run, which is correct for the boot's
		// SEQUENTIAL reads (they genuinely continue where the last one stopped) but wrong for an
		// addressed read, which names its own position.  Measured: a read of psec 160 (= cyl5 h0
		// sector 1) inherited `PROG start sector = 13`, delivered r=13..16 and ran on into head 1,
		// so the firmware - which files each record by its sector ID - wrote track 2 over track 1.
		// IOPB [1] bit0 = ADDRESSED; [6-9] = BE32 position in current-size sectors.
		if (BIT(wn.read_byte((wb + 0x01) & 0xffff), 0))
		{
			u32 const addr = (u32(wn.read_byte((wb + 0x06) & 0xffff)) << 24)
			               | (u32(wn.read_byte((wb + 0x07) & 0xffff)) << 16)
			               | (u32(wn.read_byte((wb + 0x08) & 0xffff)) << 8)
			               |  u32(wn.read_byte((wb + 0x09) & 0xffff));
			u8 const spt = m_uib_base ? wn.read_byte((m_uib_base + 1) & 0xffff) : 0;
			if (spt)
			{
				m_want_r = u16(addr % spt) + 1;
				// The HEAD comes from the request too.  Seeding the walk from m_sel_head uses the
				// stale E804 select - measured `WALK seed head=1` for a request whose position
				// (psec 160) is cyl 5 HEAD 0 - so the run started on the wrong surface before it
				// had read a single sector.  Same defect as the retained hunt start, same fix:
				// take it from the address.  heads = UIB[0].
				u8 const heads = wn.read_byte(m_uib_base & 0xffff);
				if (heads)
				{
					m_walk_head = u8((addr / spt) % heads);
					m_cmd_cyl = u16((addr / spt) / heads);
				}
				logerror("HUNT start = sector %u cyl=%u head=%u (addressed, addr=%u spt=%u) t=%.5f\n",
					m_want_r, m_cmd_cyl, m_walk_head, addr, spt, machine().time().as_double());
			}
		}
		logerror("WALK seed blocks=%d head=%d cmd_cyl=%u t=%.5f\n", m_blocks_left, m_walk_head,
			m_cmd_cyl, machine().time().as_double());
	}
	m_iopb_addr = dbi;
	m_window_seen = false; m_term_fired = false; m_idx_prev = false; m_read_active = false; m_data_chunks = 0;   // per-command reset
	m_write_active = false; m_wr_done = 0; m_wr_walked = 0;
	m_wr_final_pending = false; m_wr_complete = false;
	m_wr_stall_watch_blk = 0; m_wr_stall_watch_left = 0;
	if (m_wr_close)
		m_wr_close->adjust(attotime::never);
	// The gate array RETAINS its field program across command boundaries - the firmware says so
	// explicitly.  $5FF6 cmpi.w #$1,$793e / beq $6028 makes the builder emit NEITHER op1A NOR op18
	// when the program already in the gate array is the one this command needs; op18 publishes the
	// identity itself at $3094 (move.w $793c,$793e), and [$793c] carries 1/2/3/4 from seven
	// builders with a proper invalidate path ($0E48/$0EC2/$3142/$317C write #$ffff, $2756 clears).
	// Clearing m_prog_loaded per command therefore threw away a program the firmware was relying on
	// us to keep: the SECOND read emitted no op18, so no C800/E000 burst ever arrived, nothing was
	// armed, and the read produced one record's worth of activity and then silence. (cont.429)
	m_desc_n = 0;   // the descriptor capture buffer is per-load, but the LOADED program persists
	// ...and so is the LOAD-IN-PROGRESS flag, which unlike m_prog_loaded must NOT survive a command
	// boundary: a block copy into E000-E01E cannot physically be in flight across a HOST GO.  It is
	// set by any write to offset >= 1 and cleared only by the write that completes the block, so an
	// incomplete copy latches it true for good.  MEASURED (cont.567) as the sole cause of the write
	// `1c` class: on every program-reusing write the firmware DOES issue the per-record engagement
	// (E000 <= $0a6d, bit11 set, on a clean rising edge) and the model rejected all three, with
	// loaded=1 armable=1 prev=0 and `loading=1` the only false conjunct.  Nothing then armed, no
	// mark was ever presented, the command took zero IRQ5/IRQ6, and $9602's ID poll ran its D2
	// countdown out to $983E ($201c).  This also retires the belief that a reusing write writes no
	// E000 at all - it does; the model was not listening.
	m_prog_loading = false;
	// The firmware carries the STATUS/ERROR bytes back to the host IOPB itself, via its node->host
	// bus-master DMA (run_channel_dma, E800 bit13); the model transcribes nothing here.
	m_held_len = 0;   // no field carries across a command boundary
	m_am_presented = 0;
	m_ser_active = false; m_ser_clk = true;   // the serial ack does not carry across a command boundary
	m_esdi_frame_active = false;
	m_esdi_frame_falls = 0;
	m_esdi_frame_closing = false;
	m_esdi_beat = 0;
	m_esdi_resp_phase = false;
	m_esdi_shift = 0;
	// Engagement must be decided HERE for a retained program.  With op18 skipped, nothing in the
	// ladder (24 28 56 58 54 4A 42 36 00) writes E000, so the bit11 test below never evaluates and
	// start_field_program() would never re-trigger.  Both inputs are the gate array's own: it knows
	// whether it still holds a program, and it has the command byte it just fetched.
	m_read_window = false;
	// READ ONLY, and cmd 0x96 must NOT be added here.  A program-reusing WRITE reaches the same
	// dead end by the same route (no op18 -> no E000 write -> the bit11 test never runs -> nothing
	// ever arms, so $9602's ID poll spins its D2 countdown to exhaustion and $983E returns $201c),
	// so extending this test to 0x96 is the obvious fix and it is REFUTED.  Measured A/B on the
	// reuse baseline (cont.567): it does not rescue a single reuse write and it BREAKS the loading
	// write that passed - identical mark counts in both arms (L5=18, L6=9, [$7a68] set, 8 commits,
	// no $983E hit) yet 82/1c instead of 80/00, after which the installer aborts.  Cause is the
	// cont.523 ordering hazard: engaging at command entry runs the write first-arm (capture_track,
	// stage_record, m_sec_count from [$7abc]) BEFORE the firmware has written its per-command
	// values, and the real op18 load then re-enters this function with m_write_active already true
	// and skips the genuine setup.  A write must engage LAZILY, once the firmware has demonstrably
	// set up the operation - not at the command boundary, where the class cannot yet be known.
	if (m_prog_loaded && m_iopb_cmd == 0x95)
	{
		// Engage NOW.  E802 bit11 cannot be the trigger: it is issued by the read ISR, which only
		// runs after a record has landed, so on a retained program nothing would ever arm the first
		// one.  op18 armed it directly when it loaded; a retained program must arm it here.
		logerror("GA retains field program across command boundary -> engaging for cmd=%02x t=%.5f\n",
			m_iopb_cmd, machine().time().as_double());
		start_field_program();
	}
	m_cmd = CMD_IDLE;
	m_armed = false;
	m_e000b11_prev = false;
	m_bit11_prev = false;
	m_rec_latch = false;
	m_cpu->set_input_line(M68K_IRQ_2, ASSERT_LINE);
}

u16 multibus_storager_device::host_win_r(offs_t offset)
{
	u32 const pio = 0x7200 + offset * 2;
	u32 const fa = 0x7e00 + offset * 2;
	address_space &cs = m_cpu->space(AS_PROGRAM);
	u16 v = cs.read_byte(fa) | (u16(cs.read_byte(fa + 1)) << 8);
	// R0 status register (ch1 command/status @pio 0x73F8): the gate array presents the board state the
	// CPUAP polls before each GO - bit0 Idle / bit1 Busy / bit2 OPER-DONE-INT / bits4-7 unit ready
	// (spec §5).  A generated register, not the dual-port mailbox echo.
	if (pio == 0x73f8)
	{
		u8 r0 = m_r0_busy ? 0x02 : 0x01;
		if (m_r0_doneint) r0 |= 0x04;
		if (m_hd[0] && m_hd[0]->exists()) r0 |= 0x10;
		if (m_hd[1] && m_hd[1]->exists()) r0 |= 0x20;
		if (m_floppy[0] && m_floppy[0]->get_device() && m_floppy[0]->get_device()->exists()) r0 |= 0x40;
		if (m_floppy[1] && m_floppy[1]->get_device() && m_floppy[1]->get_device()->exists()) r0 |= 0x80;
		v = (v & 0xff00) | r0;
	}
	return v;
}

void multibus_storager_device::host_win_w(offs_t offset, u16 data, u16 mem_mask)
{
	u32 const pio = 0x7200 + offset * 2;
	address_space &cs = m_cpu->space(AS_PROGRAM);

	// The host I/O register file maps register-per-WORD into fw RAM (the dual-port spreads each 8-bit
	// host register into the low byte of its own 16-bit word: ch0 R0-R3 @73F4-73F7 -> $7FF0/2/4/6,
	// ch1 R0-R3 @73F8-73FB -> $7FF8/A/C/E), latched inbound (spec cont.202/209).  Other window writes
	// map byte-for-byte to the dual-port RAM at +0xC00.
	if (pio >= 0x73f4 && pio <= 0x73fb)
	{
		if (ACCESSING_BITS_0_7)  { cs.write_byte(0x7ff0 + (pio - 0x73f4) * 2, data & 0xff); m_mb_in[pio - 0x73f4] = data & 0xff; }
		if (ACCESSING_BITS_8_15) { cs.write_byte(0x7ff0 + (pio + 1 - 0x73f4) * 2, data >> 8); if (pio + 1 <= 0x73fb) m_mb_in[pio + 1 - 0x73f4] = data >> 8; }
	}
	else
	{
		u32 const fa = 0x7e00 + offset * 2;
		if (ACCESSING_BITS_0_7)  cs.write_byte(fa,     data & 0xff);
		if (ACCESSING_BITS_8_15) cs.write_byte(fa + 1, data >> 8);
	}

	// R0 command register lifecycle (pio 0x73F8 write): bit0 = GO (-> BUSY), bit1 = CLR-INT.
	if (pio == 0x73f8 && ACCESSING_BITS_0_7)
	{
		u8 const r0cmd = data & 0xff;
		if (r0cmd & 0x02) m_r0_doneint = false;
		if (r0cmd & 0x01) m_r0_busy = true;
	}

	// Doorbell: the CPUAP writes GO (0x13) to the ch1 command register (pio 0x73F8) last.  The gate
	// array auto-fetches the 0x18-byte IOPB from the mailbox pointer into the firmware's node work
	// area and raises IRQ2.  (0x18, not 0x1C: the node's +0x18.. is firmware-owned error state.)
	if (pio == 0x73f8 && ACCESSING_BITS_0_7 && (data & 0xff) == 0x13)
	{
		u32 const dbi = (u32(cs.read_byte(0x7ffa)) << 16) | (u32(cs.read_byte(0x7ffc)) << 8) | cs.read_byte(0x7ffe);
		u32 dst = cs.read_word(0x7a06);
		if (dst < 0x4000 || dst >= 0x7e00)
			dst = 0x71f0;
		address_space &bs = m_bus->space(AS_PROGRAM);
		// FIFTH model<->SRAM byte boundary (cont.432), made explicit: cs.write_byte goes through the
		// swapped local mapping, bs.read_byte does not.  Node BYTE reads therefore round-trip
		// correctly - which is why the command byte and $5FC0's node+$a/$b guard inputs are faithful -
		// but any host-supplied field the firmware reads as a WORD would see swapped halves.  Swept
		// against the disassembly: zero such reads exist (0 word reads at displacement <$18 off any
		// register loaded from $71bc/$7b20/$7a06, 39 load sites), consistent with a byte-oriented
		// IOPB.  That sweep does not follow node pointers passed across calls, so if a word read of a
		// host field is ever found, THIS is the copy that has to swap.
		for (u32 k = 0; k < 0x18; k++)
			cs.write_byte((dst + k) & 0xffff, bs.read_byte((dbi + k) & 0xffffff));
		// Capture the fetched-IOPB pointer so the firmware knows where its work IOPB is: [$7B20] is the
		// "fetched-IOPB ptr" the intake reads (A2 = [$7B20]; ($2,A2) = 0x81 busy) and the completion
		// stamps.  Without it the accept/status stamps land on garbage (addr 0).
		go_submit(dst, dbi);
	}
}

// Multibus I/O 0x0000 data aperture (host reads kernel blocks through 0xF00000 = bus I/O 0x0000).
u16 multibus_storager_device::bus_data_r(offs_t offset)
{
	return 0;
}

// Multibus memory window: the Storager (a bus master) dereferences CPUAP RAM pointers (IOPB, data
// buffers) directly as 68000 addresses.  68000 is big-endian: high byte at the lower Multibus byte.
// The firmware stamps host IOPB status through this window itself (spec §5.1); the model transcribes
// nothing.
u16 multibus_storager_device::bus_mem_r(offs_t offset, u16 mem_mask)
{
	u32 const addr = 0x010000 + offset * 2;
	address_space &bs = m_bus->space(AS_PROGRAM);
	u16 v = 0;
	if (ACCESSING_BITS_8_15) v |= u16(bs.read_byte(addr)) << 8;
	if (ACCESSING_BITS_0_7)  v |= bs.read_byte(addr + 1);
	return v;
}

void multibus_storager_device::bus_mem_w(offs_t offset, u16 data, u16 mem_mask)
{
	u32 const addr = 0x010000 + offset * 2;
	address_space &bs = m_bus->space(AS_PROGRAM);
	if (ACCESSING_BITS_8_15) bs.write_byte(addr,     data >> 8);
	if (ACCESSING_BITS_0_7)  bs.write_byte(addr + 1, data & 0xff);
	// The firmware stamps host IOPB status (byte +2, bit7 set = 0x80 OK / 0x82 error) through this
	// bus-master window (spec §5.1).  Observe that DONE edge to drive the R0 status register: clear
	// BUSY, raise OPER-DONE-INT.
	if (m_iopb_addr)
	{
		u32 const st = (m_iopb_addr + 2) & 0xffffff;
		u8 stv = 0;
		bool hit = false;
		if (ACCESSING_BITS_8_15 && addr == st)     { stv = data >> 8;   hit = true; }
		if (ACCESSING_BITS_0_7  && addr + 1 == st) { stv = data & 0xff; hit = true; }
		if (hit)
			logerror("HOST IOPB+2 stamp <= %02x  (%s) iopb=%06x t=%.5f\n", stv,
				stv == 0x80 ? "*** 0x80 GOOD ***" : (stv == 0x82 ? "0x82 ERROR" : "busy/other"),
				m_iopb_addr, machine().time().as_double());
		if (hit && (stv & 0x80)) { m_r0_busy = false; m_r0_doneint = true; }
	}
}

// ---------------------------------------------------------------------------
// PIT outputs
// ---------------------------------------------------------------------------

// PIT1 ctr0 OUT = the system tick -> 68000 IRQ1 (handler $2B58 walks the $736c software-timeout
// queue and re-pets the counter via the E800 bit9 gate).  count 0xFF00 (mode 3, MSB-only) ≈ 26ms
// at /4 (board.yaml + cont.159: authentic, not a ×256 defect).
//
// NOTE on debugger bps: $2B80 is NOT the tick entry — it is the EXPIRE path inside $2B58
// (subq count hits 0).  Entry is $2B58; per-tick decrement is $2B7A.  Sparse $2B80 hits mean few
// timers ran to completion (most are cancelled via $2ABA), not that IRQ1 is starved.
// ---------------------------------------------------------------------------------------------
// ESDI serial command/status interface.  One beat per E802 bit0 falling edge while an ESDI unit
// is selected.  Frame = 17 beats: 16 data MSB-first plus one odd-parity bit, both directions,
// both data lines INVERTED.  See the m_esdi_* declarations for the ROM evidence ($A118/$A1CE).
void multibus_storager_device::esdi_serial_beat(u16 e802)
{
	if (m_esdi_resp_phase)
	{
		if (m_esdi_beat < 16)
		{
			m_esdi_out_bit = BIT(m_esdi_resp, 15 - m_esdi_beat);
		}
		else
		{
			// 17th beat: make the whole frame carry an ODD number of ones, which is what
			// $A1CE's `lsr.w #1,D5 / bcs` demands - an even frame returns $201e.
			int ones = 0;
			for (int b = 0; b < 16; b++) if (BIT(m_esdi_resp, b)) ones++;
			m_esdi_out_bit = ((ones & 1) == 0);
		}
		if (++m_esdi_beat >= 17)
		{
			m_esdi_beat = 0;
			m_esdi_resp_phase = false;
			m_esdi_out_bit = false;
		}
		return;
	}
	// Controller -> drive.  A data 1 CLEARS E802 bit1 ($a154 andi #$fffd), a 0 sets it ($a15e).
	if (m_esdi_beat < 16)
		m_esdi_shift = u16((m_esdi_shift << 1) | (BIT(e802, 1) ? 0 : 1));
	// beat 16 is the controller's parity bit; the drive checks it, and a real one would fault on a
	// mismatch.  Not enforced until we have seen a good frame - a false parity fault here would
	// look exactly like the all-ones response this engine exists to replace.
	if (++m_esdi_beat >= 17)
	{
		m_esdi_beat = 0;
		esdi_command(m_esdi_shift);
		m_esdi_shift = 0;
	}
}

void multibus_storager_device::esdi_command(u16 cmd)
{
	m_esdi_last_cmd = cmd;
	u16 const op = cmd >> 12;
	char const *name = "?";
	bool respond = false;
	switch (op)
	{
	case 0x0:  // SEEK - cylinder in bits 11..0
		name = "SEEK";
		m_esdi_cyl = cmd & 0x0fff;
		break;
	case 0x1:  // RECALIBRATE
		name = "RECAL";
		m_esdi_cyl = 0;
		break;
	case 0x2:  // REQUEST STATUS -> 16-bit general status
		name = "REQ-STATUS";
		// bit4 SET is a fault: $6704 `btst #4,D1` maps it straight to sense $4B, which is the
		// error the all-ones stub produced.  A healthy drive reports no faults.
		m_esdi_resp = 0x0000;
		respond = true;
		break;
	case 0x3:  // REQUEST CONFIGURATION
		// $a686 stores the response big-endian into NODE([$71bc])+$0e/$0f (NOT the UIB — A0 is
		// the node pointer latched at accept).  Post-store $a690 waits for F000 bit8 (or bit9
		// when [$7a0a]!=0) CLEAR.  UIB+$0e is a separate host-template field ($743a gate).
		// Encoding of the 16-bit product word is still open; 0 leaves NODE+$e/$f zeroed.
		name = "REQ-CONFIG";
		m_esdi_resp = 0x0000;   // PROVISIONAL: NODE+$e/$f consumers ($12ae not.w of +$f, …)
		respond = true;
		break;
	case 0x4: name = "SELECT-HEAD"; m_esdi_head = cmd & 0x000f; break;
	case 0x5: name = "CONTROL"; break;
	default:  name = "UNDECODED"; break;
	}
	if (TRACE_ESDI)
		logerror("ESDI cmd=%04x %-11s cyl=%u head=%u%s resp=%04x t=%.5f\n",
			cmd, name, m_esdi_cyl, m_esdi_head,
			respond ? " -> RESPOND" : "", respond ? m_esdi_resp : 0,
			machine().time().as_double());
	if (respond)
	{
		m_esdi_resp_phase = true;
		m_esdi_beat = 0;
		m_esdi_out_bit = BIT(m_esdi_resp, 15);
	}
}

void multibus_storager_device::timer0_out(int state)
{
	bool const rising = state && !m_timer_out;
	m_timer_out = bool(state);          // F000 bit11
	if (rising && m_gate0)
	{
		// Cadence probe (inst7 soft-timer cut): how often does the model actually raise IRQ1?
		// Expected ≈ 1/26ms.  A live freelist with count #$32 should expire in ~1.3s if these fire.
		static int s_irq1_n = 0;
		static double s_irq1_t0 = -1.0;
		double const t = machine().time().as_double();
		if (s_irq1_t0 < 0.0)
			s_irq1_t0 = t;
		s_irq1_n++;
		if (s_irq1_n <= 5 || (s_irq1_n % 500) == 0)
			logerror("IRQ1 raise #%d gate0=1 dt=%.5f (from first) t=%.5f\n",
				s_irq1_n, t - s_irq1_t0, t);
		// Sample BEFORE the handler runs: successive samples must drop by 1 if $2B7A services the block.
		if (m_wr_stall_watch_left > 0 && m_wr_stall_watch_blk >= 0x4000 && m_wr_stall_watch_blk < 0x7f00)
		{
			address_space &cs = m_cpu->space(AS_PROGRAM);
			u16 const cnt = cs.read_word(m_wr_stall_watch_blk);
			u16 const qh  = cs.read_word(0x736c);
			logerror("WRSTALL WATCH irq1#%d blk=%04x cnt=%04x [$736c]=%04x left=%d t=%.5f\n",
				s_irq1_n, m_wr_stall_watch_blk, cnt, qh, m_wr_stall_watch_left, t);
			m_wr_stall_watch_left--;
			if (m_wr_stall_watch_left == 0)
				logerror("WRSTALL WATCH done blk=%04x (ffff->fffe->… = live; stuck = not serviced) t=%.5f\n",
					m_wr_stall_watch_blk, t);
		}
		m_cpu->set_input_line(M68K_IRQ_1, HOLD_LINE);
	}
	else if (rising && !m_gate0)
	{
		static int s_nongate = 0;
		if (++s_nongate <= 8)
			logerror("IRQ1 SUPPRESSED (OUT rise, gate0=0) t=%.5f\n", machine().time().as_double());
	}
}

void multibus_storager_device::timer2_out(int state)
{
	// PIT1 ctr2 is programmed by the firmware (op28/$684C, op42/$6CC2 control $9A).  Capture OUT
	// recorded but not surfaced on F000 bit8 (see cont.428).
	bool const rising = state && !m_pit2_out;
	m_pit2_out = bool(state);
	if (rising)
		logerror("PIT2 OUT rise t=%.4f\n", machine().time().as_double());
}

// ---------------------------------------------------------------------------
// media
// ---------------------------------------------------------------------------

void multibus_storager_device::spin_drives()
{
	m_floppy_loaded = true;
	// Spin the drives so INDEX pulses (5.25"): the firmware polls F000 for INDEX/READY/TRACK0, so the
	// media must present as a real spinning floppy.  Done at first F000 access, after every device_reset
	// (the floppy's own reset runs after the storager's and would re-raise m_mon).
	for (int i = 0; i < 2; i++)
		if (floppy_image_device *const fdd = m_floppy[i]->get_device())
			fdd->mon_w(0);
}

void multibus_storager_device::floppy_swap_cb(floppy_image_device *)
{
	m_floppy_loaded = false;
}

// ---------------------------------------------------------------------------
// device
// ---------------------------------------------------------------------------

void multibus_storager_device::device_start()
{
	m_lram = std::make_unique<u16[]>(0x2000);
	save_pointer(NAME(m_lram), 0x2000);
	save_item(NAME(m_ch));
	save_item(NAME(m_c000));
	save_item(NAME(m_c000_valid));
	save_item(NAME(m_c800));
	save_item(NAME(m_d000));
	save_item(NAME(m_d800));
	m_ioreg_int_timer = timer_alloc(FUNC(multibus_storager_device::ioreg_int), this);
	m_pump = timer_alloc(FUNC(multibus_storager_device::pump_tick), this);
	m_dma_done = timer_alloc(FUNC(multibus_storager_device::dma_done), this);
	m_wr_close = timer_alloc(FUNC(multibus_storager_device::wr_close_check), this);
}

void multibus_storager_device::device_reset()
{
	if (m_ioreg_pending > 0) int_w<2>(1);   // release INT2 across reset
	m_ioreg_pending = 0;
	m_os_fw_pending = false;
	m_ioreg_done = false;
	if (m_ioreg_int_timer) m_ioreg_int_timer->adjust(attotime::never);
	if (!m_installed)
	{
		// Multibus PIO window 0x7200-0x73FF -> on-board dual-port RAM 0x7E00-0x7FFF (+0xC00).
		m_bus->space(AS_IO).install_readwrite_handler(0x7200, 0x73ff,
			read16sm_delegate(*this, FUNC(multibus_storager_device::host_win_r)),
			write16s_delegate(*this, FUNC(multibus_storager_device::host_win_w)));
		// data aperture the host reads kernel blocks through (0xF00000 = bus I/O 0x0000).
		m_bus->space(AS_IO).install_readwrite_handler(0x0800, 0x0807,
			read16sm_delegate(*this, FUNC(multibus_storager_device::ioreg_r)),
			write16s_delegate(*this, FUNC(multibus_storager_device::ioreg_w)));
		m_bus->space(AS_IO).install_read_handler(0x0000, 0x00ff,
			read16sm_delegate(*this, FUNC(multibus_storager_device::bus_data_r)));
		// Snoop the local-bus buffer (0x4000-0x7FFF): the gate array's address comparator watches the
		// bus so it can answer the F000 bit12 address-match test - it latches the byte-strobe of a read
		// and the address of a write.
		//
		// The snoop must fire only on a REAL BUS CYCLE.  A MAME tap is called for every access through
		// the address space, and that includes debugger and Lua reads, which are not bus cycles at all
		// (the Lua accessors call read_byte() with no side-effect guard).  Honouring those makes the
		// device corruptible by observing it: a probe polling firmware RAM each tick clobbers
		// m_term_bit0, the firmware's $9C00 address-comparator self-test then reads a wrong bit12, and
		// the board exits to $367A self-test idle and never services another command - which looked
		// exactly like a boot hang and cost a session to run down.  Gate on the storager's own CPU
		// actually executing, so inspecting the model cannot change it.
		address_space &cs = m_cpu->space(AS_PROGRAM);
		auto const on_local_bus = [this]()
		{
			device_execute_interface *const exec = machine().scheduler().currently_executing();
			return exec && (&exec->device() == m_cpu.target());
		};
		cs.install_read_tap(0x4000, 0x7fff, "dma_snoop_r",
			[this, on_local_bus](offs_t, u16 &, u16 mem_mask)
			{ if (m_dma_active && on_local_bus()) m_term_bit0 = (mem_mask == 0x00ff) ? 1 : 0; });
		cs.install_write_tap(0x4000, 0x7fff, "dma_snoop_w",
			[this, on_local_bus](offs_t offset, u16 &, u16 mem_mask)
			{ if (m_dma_active && on_local_bus()) m_last_bw = offset + ((mem_mask == 0x00ff) ? 1 : 0); });
		// TEMP cont.560 (STRIP): the two implied-seek compares, read off the firmware's own registers.
		//   $215E  cmp.w ($d2,A3),D2   request cyl in D2, head in D5, IOPB at A6, UIB at A3
		//   $21DC  cmp.w ($d2,A4),D1   request cyl in D1, IOPB at A0, UIB at A4
		// Both derive the request at ($7,An): a 24-bit big-endian block address divided by UIB+1
		// (sectors/track) and then UIB+0 (heads).  The IOPB bytes are printed alongside so a stale
		// request (firmware still holding the previous command's address) is distinguishable from a
		// fresh request that computes the same cylinder.
		//
		// AN OPCODE-SPACE TAP CANNOT SEE THIS.  The 68000 fetches through a memory_access cache
		// handle, and caches bypass taps entirely - taps at $215E/$21DC fired zero times.  The
		// compare's OPERAND read (UIB+$d2) is an ordinary data cycle in AS_PROGRAM, so tap that
		// and identify the site from pcbase().  Same instruction, a path that is actually observable.
		//
		// READ m_lram DIRECTLY, never through the address space: this runs INSIDE a 0x4000-0x7fff
		// read tap, so a read_byte() here would re-enter the dma_snoop tap and clobber m_term_bit0 -
		// the observe-the-device-and-break-it class that already cost a session.
		if (TRACE_SEEKCMP)
		{
			auto const lb = [this](u32 a) -> u32 {
				if (a < 0x4000 || a > 0x7fff) return 0xff;
				u16 const v = m_lram[(a - 0x4000) >> 1];
				return (a & 1) ? ((v >> 8) & 0xff) : (v & 0xff);
			};
			auto const lw = [this](u32 a) -> u32 {
				if (a < 0x4000 || a > 0x7fff) return 0xffff;
				return m_lram[(a - 0x4000) >> 1];
			};
			auto const dump = [this, lb, lw](char site, u32 iopb, u32 uib, u32 reqcyl)
			{
				u32 const blk = (lb(iopb + 7) << 16) | (lb(iopb + 8) << 8) | lb(iopb + 9);
				u32 const cache = lw(uib + 0xd2);
				logerror("SEEKCMP %c iopb=%04x cmd=%02x opt=%02x blk=%u | req cyl=%u | cache UIB@%04x "
					"+$d0(head)=%04x +$d2(cyl)=%04x +$d6(pos)=%04x | spt=%u hds=%u | %s t=%.5f\n",
					site, iopb, lb(iopb), lb(iopb + 1), blk, reqcyl, uib,
					lw(uib + 0xd0), cache, lw(uib + 0xd6), lb(uib + 1), lb(uib),
					(reqcyl == cache) ? "MATCH -> no seek" : "DIFFER -> seek",
					machine().time().as_double());
			};
			// A SILENT ZERO IS A DEAD TAP UNTIL PROVEN OTHERWISE.  The first two attempts printed
			// nothing, which is equally consistent with "the compare never runs" and "the probe never
			// runs".  Carry a hit counter and a census of every pcbase seen inside $2000-$23FF, so a
			// zero at $215E/$21DC comes with evidence that the tap itself was live and that the
			// firmware was executing in that region at all.
			cs.install_read_tap(0x4000, 0x7fff, "seekcmp",
				[this, dump, lb, lw, on_local_bus, hits = u64(0), seen = std::map<u32, u32>(),
				 nextrep = 0.0, a2_last = u32(0xffffffff), a2_next = 0.0, a6_next = 0.0, vcmp_n = 0]
				(offs_t, u16 &, u16) mutable
				{
					if (!on_local_bus()) return;
					hits++;
					u32 const pc = m_cpu->pcbase() & 0xffffff;
					// Bucket the WHOLE address space by $100 and CLEAR each report, so the numbers
					// are per-interval deltas.  A cumulative map over one region answered "is the
					// dispatcher spinning" but could not say what stopped running when it started.
					seen[pc & 0xffff00]++;
					double const t = machine().time().as_double();
					if (t >= nextrep)
					{
						nextrep = t + 2.0;
						std::vector<std::pair<u32, u32>> v(seen.begin(), seen.end());
						std::sort(v.begin(), v.end(),
							[](auto const &a, auto const &b) { return a.second > b.second; });
						std::string s;
						for (unsigned i = 0; i < v.size() && i < 12; i++)
							s += util::string_format(" %04x:%u", v[i].first, v[i].second);
						logerror("SEEKCMP CENSUS t=%.3f hits=%llu | pcbase/$100 this interval:%s\n",
							t, (unsigned long long)hits, s.empty() ? " (none)" : s.c_str());
						seen.clear();
					}
					if (pc == 0x215e)
						dump('A', m_cpu->state_int(M68K_A6) & 0xffff, m_cpu->state_int(M68K_A3) & 0xffff,
							m_cpu->state_int(M68K_D2) & 0xffff);
					else if (pc == 0x21dc)
						dump('B', m_cpu->state_int(M68K_A0) & 0xffff, m_cpu->state_int(M68K_A4) & 0xffff,
							m_cpu->state_int(M68K_D1) & 0xffff);
					// $0F94 IS THE ADDRESSING-MODE FORK, and it does not read IOPB[1].
					//   000f94 move.w ($20,A6),D0 / andi.w #1,D0 / bne $FD4
					//     bit0 CLEAR -> PHYSICAL: head := iopb[5], cyl := iopb[6..7] BE16,
					//                             sector := iopb[8..9]
					//     bit0 SET   -> ADDRESSED: iopb[7..9] / UIB+1 (spt) / UIB+0 (heads)
					//   Either way the results land in [$7946] (head) and [$7948] (cylinder).
					else if (pc == 0x0f94)
					{
						u32 const uib = m_cpu->state_int(M68K_A6) & 0xffff;
						u32 const iop = m_cpu->state_int(M68K_A0) & 0xffff;
						std::string b;
						for (u32 k = 0; k < 10; k++)
							b += util::string_format(" %02x", lb(iop + k));
						logerror("MODEFORK $0F94 uib=%04x +$20=%04x bit0=%u -> %s | iopb@%04x:%s"
							" | spt=%u hds=%u t=%.5f\n",
							uib, lw(uib + 0x20), lw(uib + 0x20) & 1,
							(lw(uib + 0x20) & 1) ? "ADDRESSED" : "PHYSICAL",
							iop, b.c_str(), lb(uib + 1), lb(uib), machine().time().as_double());
					}
					// $67F6 IS THE SEEK GATE THAT ACTUALLY RUNS.  0067ba loads the requested cylinder
					// from [$7948] and 0067d4 the head from [$7946]; 0067f2/0067f6 compare them against
					// the UIB cache and `beq $683E' skips the seek outright when both match.  This is
					// the compare the whole off-by-one question turns on - $21DC never executes.
					// $67A2 IS THE EARLY BAIL, and it is a WAIT, not an error.  $6788 -> $6792 loads
					// UIB+$12; for the floppy (bits0-1 == 0) it falls to `tst.w $7a36 / bne $67BA'.
					// With [$7a36] clear and $14 bit4 clear it returns D0 = $FE, which the walker
					// reads as "in progress, call me again" ($63BC cmpi #$FE -> $63B4 loop) - so the
					// board spins in the dispatcher forever instead of seeking.  [$7a36] is set in
					// exactly one place ($661A), either immediately when UIB+$14's low nibble is zero
					// or after a queued delay of nibble*10 ticks via $29F8.  Rate-limited: this site
					// executes ~38k times a second once the wait starts.
					else if (pc == 0x67a2)
					{
						u32 const v = lw(0x7a36);
						double const now = machine().time().as_double();
						if (v != a2_last || now >= a2_next)
						{
							a2_last = v;
							a2_next = now + 1.0;
							address_space &ls = m_cpu->space(AS_PROGRAM);   // 0x14 is outside the snoop range
							u32 const uib = lw(0x799a);
							logerror("SETTLE $67A2 [$7a36]=%04x [$14]=%02x bit4=%u | uib=%04x +$12=%02x "
								"+$14=%02x | %s t=%.5f\n",
								v, ls.read_byte(0x14), BIT(ls.read_byte(0x14), 4), uib,
								lb(uib + 0x12), lb(uib + 0x14),
								(v || BIT(ls.read_byte(0x14), 4)) ? "-> $67BA seek" : "*** RETURNS $FE - WAIT ***",
								now);
						}
					}
					// $65C6 is the motor-on poller's own gate (`tst.w $7a34 / beq $6620').  Reaching
					// it at all is the fact that matters: if the poller never runs, no amount of
					// flag state explains the wedge; if it runs and still does not queue the timer,
					// the three words say which of the three gates stopped it.
					// $651C is micro-op 24 (select + head-load + seek-start) - the ONLY route to the
					// motor-on poller, called once per command from $0F56/$636E/$64E6/$A648.  It has
					// two early bails before the poller: `tst.w $7968 / bne $6784' at entry, and
					// `btst #$f,[$7944] / bne $664A'.  Logging entry AND those two words separates
					// "op24 never ran" from "op24 ran and bailed", which the page census cannot.
					else if (pc == 0x651c)
						logerror("OP24 $651C entry [$7968]=%04x -> %s t=%.5f\n",
							lw(0x7968), lw(0x7968) ? "*** BAILS to $6784 ***" : "continues",
							machine().time().as_double());
					else if (pc == 0x652e)
						logerror("OP24 $652E [$7944]=%04x bit15=%u bit4=%u | uib+$12=%02x t=%.5f\n",
							lw(0x7944), BIT(lw(0x7944), 15), BIT(lw(0x7944), 4),
							lb(lw(0x799a) + 0x12), machine().time().as_double());
					else if (pc == 0x65c6)
					{
						double const now = machine().time().as_double();
						if (now >= a6_next)
						{
							a6_next = now + 1.0;
							logerror("MOTORPOLL $65C6 [$7a34]=%04x [$7a36]=%04x [$7a38]=%04x | %s t=%.5f\n",
								lw(0x7a34), lw(0x7a36), lw(0x7a38),
								!lw(0x7a34)  ? "no request -> $6620"
								: lw(0x7a36) ? "already settled -> $6620"
								: lw(0x7a38) ? "*** TIMER HANDLE STILL SET - will not re-queue ***"
								             : "queues the spin-up timer", now);
						}
					}
					// $14E2 ENTRY AND ITS CALLER.  Four callers ($0DC6, $0E4A, $193E, $6806); only
					// $6806 is blocked in the wedged state, so which one performs the 130us clear on
					// a WORKING command has been inferred, never measured.  At the entry tst the bsr
					// return address is still on top of the stack, so read it directly.
					// THE PRE-ARM DECISION.  $7EC4/$7ECA/$7ED0 are three gates; any one of them sends
					// the path to $7ED8 (clear [$741c], no chunk armed) instead of $7EE0 (pop a chunk,
					// arm it, set [$741c]).  $7ED0 tests for an $AA END MARKER at (A0,D0.w).  All three
					// inputs are live at $7EC4, so one tap reports the whole decision.
					// $7E9A is the SAME $AA test on the [$7968]!=0 arm.  Log D0 at both sites: $7E6C
					// scans FORWARD past any byte that is not $AA/$FF/$FE, so the deciding cell is the
					// first special one at or after R+1 - which is only R+1 itself when that cell is
					// already special.  Reporting the address alone let "marker@765d" be read as
					// "sector 9" when it may be a later cell the scan walked to.
					else if (pc == 0x7e9a)
					{
						u32 const a0 = m_cpu->state_int(M68K_A0) & 0xffff;
						s32 const d0 = s16(m_cpu->state_int(M68K_D0) & 0xffff);
						u32 const mk = (a0 + d0) & 0xffff;
						logerror("PREARM $7E9A D0=%d cell=%04x(pos %d)=%02x -> %s | recs=%d/%d t=%.5f\n",
							d0, mk, int(mk) - 0x7654, lb(mk),
							(lb(mk) == 0xaa) ? "$7ED8 clear" : "$7EE0 allocate + arm",
							m_cmd_records, m_sec_count, machine().time().as_double());
					}
					else if (pc == 0x7ec4)
					{
						u32 const a0 = m_cpu->state_int(M68K_A0) & 0xffff;
						s32 const d0 = s16(m_cpu->state_int(M68K_D0) & 0xffff);
						u32 const mk = (a0 + d0) & 0xffff;
						u32 const g1 = lw(0x796a);
						u32 const g2 = (lw(0x7958) << 16) | lw(0x795a);
						u32 const mv = lb(mk);
						logerror("PREARM $7EC4 [$796a]=%04x [$7958]=%08x | D0=%d cell=%04x(pos %d)=%02x %s | -> %s "
							"| recs=%d/%d t=%.5f\n",
							g1, g2, d0, mk, int(mk) - 0x7654, mv, (mv == 0xaa) ? "IS $AA" : "not $AA",
							(!g1 || g2 || mv == 0xaa) ? "$7ED8 clear (no chunk armed)"
							                          : "$7EE0 allocate + arm -> [$741c]=1",
							m_cmd_records, m_sec_count, machine().time().as_double());
					}
					// THE VERIFY COMPARE ITSELF.  $7C66 cmp.w $7438,D0 decides the whole record-service
					// fork: equal -> $7CE6 normal; unequal -> $7C6C, error $2012, and (if the latch is
					// already set) the $7C92 self-heal that arms $7B2E and lets the NEXT 0x95 tear down
					// at intake despite the $0DB0 skip.  $7C92 has never fired in any run, so the
					// question is whether this compare is ALWAYS equal on the OS path or merely usually.
					else if (pc == 0x7c66)
					{
						u32 const d0 = m_cpu->state_int(M68K_D0) & 0xffff;
						u32 const tgt = lw(0x7438);
						if (d0 != tgt || vcmp_n < 6)
						{
							vcmp_n++;
							logerror("VERIFY $7C66 D0=%04x vs [$7438]=%04x -> %s | staged $7dac=%02x "
								"$7daf=%02x | [$7968]=%04x recs=%d/%d t=%.5f\n",
								d0, tgt, (d0 == tgt) ? "EQUAL -> $7CE6" : "*** UNEQUAL -> $7C6C/$2012 ***",
								lb(0x7dac), lb(0x7daf), lw(0x7968),
								m_cmd_records, m_sec_count, machine().time().as_double());
						}
					}
					else if (pc == 0x14e2)
					{
						// M68K_SP, not M68K_A7: the header says SP fetches the CURRENT stack pointer
						// (USP/ISP/MSP).  Reading A7 returned an address outside SRAM and the caller
						// readback came back ffffffff - a dead probe, not a missing caller.
						u32 const sp = m_cpu->state_int(M68K_SP) & 0xffff;
						u32 const ret = (lw(sp) << 16) | lw(sp + 2);
						logerror("TEARDOWN $14E2 [$7968]=%04x -> %s | called from %06x (sp=%04x) t=%.5f\n",
							lw(0x7968), lw(0x7968) ? "runs, will clear at $155C" : "early exit $14E6",
							ret, sp, machine().time().as_double());
					}
					else if (pc == 0x67f6)
					{
						u32 const uib = m_cpu->state_int(M68K_A6) & 0xffff;
						u32 const rc = lw(0x7948), rh = lw(0x7946);
						logerror("SEEKGATE $67F6 uib=%04x | want cyl[$7948]=%u head[$7946]=%u | "
							"cache +$d0(head)=%u +$d2(cyl)=%u +$d6(pos)=%u | %s t=%.5f\n",
							uib, rc, rh, lw(uib + 0xd0), lw(uib + 0xd2), lw(uib + 0xd6),
							(rc == lw(uib + 0xd2)) ? "EQUAL -> skip seek ($683E)" : "DIFFER -> seek",
							machine().time().as_double());
					}
				});
			// The requested cylinder/head themselves, every time they change, with the writing PC.
			// MODEFORK says the request was PARSED; this says what it was parsed INTO, and $67F6
			// says whether the seek engine ever looked at it.  Three separate facts - conflating
			// any two of them is what made [$7438] look like the answer for a whole session.
			// [$7a36] is the settle/ready flag $67A2 waits on.  One setter ($661A, either immediately
			// when UIB+$14's low nibble is zero or after a queued nibble*10 tick delay through $29F8)
			// and three clearers ($0B74, $1B48, $6DF8).  Log every transition with its PC: which
			// clearer ran last, and whether the $29F8 timer ever came back, are different faults.
			// Widened to the whole motor/settle triple.  $7a34 = "a command wants the motor",
			// $7a36 = settled, $7a38 = the outstanding timer handle.  $0B74 (motor off) cancels the
			// timer through $2ABA and clears $7a36; $65C0 (motor on) re-queues it, but ONLY if
			// $7a38 is zero ($65DE tst / bne $6620).  A cancel that leaves $7a38 set therefore
			// wedges the spin-up permanently - which is what a 2-second idle would expose.
			// [$7968] is the gate op24 bails on.  Set in exactly two places ($79D6, $82B2, both in
			// the record-service region) and cleared in seven.  A stale 1 left by the previous
			// command's completion stops the next command's drive select outright.
			cs.install_write_tap(0x7968, 0x7969, "busyflag",
				[this, on_local_bus](offs_t, u16 &data, u16 mem_mask)
				{
					if (!on_local_bus()) return;
					logerror("BUSYW [$7968] <= %04x mask=%04x pc=%06x t=%.5f\n",
						data & 0xffff, mem_mask & 0xffff,
						m_cpu->pcbase() & 0xffffff, machine().time().as_double());
				});
			cs.install_write_tap(0x7a34, 0x7a3b, "settleflag",
				[this, on_local_bus](offs_t offset, u16 &data, u16 mem_mask)
				{
					if (!on_local_bus()) return;
					logerror("SETTLEW [%04x] <= %04x mask=%04x pc=%06x t=%.5f\n",
						offset, data & 0xffff, mem_mask & 0xffff,
						m_cpu->pcbase() & 0xffffff, machine().time().as_double());
				});
			cs.install_write_tap(0x7946, 0x7949, "reqpos",
				[this, on_local_bus](offs_t offset, u16 &data, u16 mem_mask)
				{
					if (!on_local_bus()) return;
					logerror("REQPOS [%04x] <= %04x mask=%04x pc=%06x t=%.5f\n",
						offset, data & 0xffff, mem_mask & 0xffff,
						m_cpu->pcbase() & 0xffffff, machine().time().as_double());
				});
		}
		// Count the sectors the firmware ACCEPTS.  The gate array has no header comparator - it raises
		// every address mark and the firmware decides ($7C7A/$7D34).  A rejected record is torn down at
		// $7DA2 -> $88AC, which clears the capture re-arm (E802 bit11) and the $7950 alternator; an
		// accepted one is STAKED $c0 into the ledger.  So delivered-field count != sectors read, and the
		// counted disarm must follow the stakes.  Value only - no SRAM read, so the snoop above is not
		// disturbed.
		// Q2 DIAGNOSTIC, not a fix.  $70D4 `move.w #$1,$7a30' genuinely zeroes the high byte that
		// $6042's `bset #0,$7a30' armed - a word write does that on real hardware, so this is firmware
		// behaviour, not a modelling error.  Wired here only to answer one question: if the kick
		// survives to the first ISR after the descriptor queue goes non-empty, does $8344 -> $3DBC ->
		// (queue non-empty) -> $3E50 -> $4062 -> $4122 -> $414C launch, and does the rest of the chain
		// then complete?  The $3E08 SR fork is only on the EMPTY path, so an interrupt-context call
		// launches just as a mainline one would.
		// cont.460: the ACCEPT/REJECT decision itself.  $7CA4 CLEARS [$741c] on accept
		// ($7C7A -> $7E0A); $7D5C SETS it on reject ($7D34 -> $7D4A -> $7D5C).  A DATA tap, so no
		// prefetch shadow.  Logged WITH the presented identity so "which records armed" and "which
		// path fired" are ONE table - if accept ever fires on an identity outside the inferred
		// window, that kills the window model outright instead of quietly surviving.
		// cont.469 timestamp (1): the FIRMWARE's own status write at node+2 ($1314 move.w #$81,($2,A0)
		// and $1A54 move.b #$80,($2,A0)).  Timestamp (2) is the HOST POST logerror (control-block DMA
		// to host IOPB+2), timestamp (3) is the poll.  Two points would hide WHICH of (1) and (2) is
		// late, so all three are captured against the GO.
		if (TRACE_CHUNK_PATH)
			cs.install_write_tap(0x71f2, 0x71f3, "fw_status",
			[this](offs_t, u16 &data, u16 mem_mask)
			{
				logerror("FWSTAT node+2 <= %04x mask=%04x  t=%.5f\n",
					data & 0xffff, mem_mask & 0xffff, machine().time().as_double());
			});
		if (TRACE_CHUNK_PATH)
			cs.install_write_tap(0x741c, 0x741d, "accept_reject",
			[this](offs_t, u16 &data, u16 mem_mask)
			{
				floppy_image_device *const fd = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
				// THE WRITER PC, NOT THE VALUE.  Six sites write #$1 to [$741c] - $79F6, $7B06,
				// $7D5C, $7F14, $84AA, $8664 - and only $7D5C is the record-reject path.  Labelling
				// a non-zero write "REJECT" conflated all six and mis-aimed the whole end-of-command
				// story.  $8282 only cares about the value, but WE need the site to know what left
				// it set.
				logerror("DECIDE %-6s [741c]<=%04x pc=%06x  r=%02x presented=%u  cyl=%d head=%u\n",
					(data & 0xffff) ? "set" : "clear", data & 0xffff, m_cpu->pcbase() & 0xffffff,
					m_last_r, m_last_presented, fd ? fd->get_cyl() : -1, m_sel_head & 1);
			});
		// cont.491: FRESH UNFILTERED tap on the stride table.  Do NOT reuse "accept_count" below -
		// it carries three filters from a retired question, EACH of which would silence this one:
		// `if (!m_read_active) return`, `if (mem_mask == 0xffff) return` (discards exactly the
		// move.w Dn,(An) form a table builder uses), and `if (v != 0xc0) return` (a table holding
		// 0x80/0x100 emits nothing).  Count uncapped, printing capped.
		// cont.492: [$7426] is the CLAIM GATE.  The record ISR at 008072 does tst.w [$7426] /
		// beq $80f6, and only the taken branch reaches the chunk-claim at 008114 that writes the
		// sector ID into the $7696 table's +2 field.  Written ONLY with literal 0 or 1 (11 refs,
		// all direct - grep is sound here), so this is a firmware flag, not a captured hw value.
		// No time gate and no value filter: every write, with the cylinder in force.
		// cont.492: the SECTOR-ID COMPARE.  007e40 is `cmp.w $7428.w,D0` / `bne $7d32`; D0 holds the
		// ID presented off the medium, [$7428] the ID the firmware wants.  A mismatch jumps to the
		// set-[$7426]=1 path and the chunk is never claimed.  Log BOTH sides at the compare itself.
		// pcbase runs a consistent +0x18 ahead here (verified on four independent sites), so the
		// compare reports as 007e58.  No value filter, no time gate; every read is counted.
		// cont.492: $7654 is a per-sector-ID OWNERSHIP MAP, indexed at 007d10 by the ID presented
		// off the medium (`move.b (A0,D0.w),D1`, A0=$7654, D0=the presented ID).  So the read
		// ADDRESS minus $7654 IS the presented sector ID - this reads the ID stream straight off
		// the firmware's own index, with no dependence on my model's idea of what it presented.
		// Map is only $42 bytes ($7654..$7695) and butts directly against the $7696 stride table.
		// cont.523: LATCH the commanded count ON THE WRITE, rather than sampling [$7abc] at a moment
		// of the model's choosing.  Measured: on a program-reusing command the model's per-command
		// init runs 108us BEFORE the firmware writes the count, so it caught the previous command's
		// value and truncated the read.  The firmware writes it correctly every time (8 / 4 / 4, and
		// 0x35=53 then 0x0c=12 for the multi-track one) - only the model's sampling point was wrong.
		// Latching on the write removes the ordering dependency entirely: there is nothing to be
		// early or late relative to.  Deferring to "after the program push" would NOT fix it - the
		// read is already inside start_field_program(), which the reusing command does enter (that
		// is why PROG n=0 prints); it simply enters it before the write.
		// WHO SETS THE WRITE'S ERROR?  The sense byte reaches the host as node+3 via HOST POST.
		// Tap the node status/error word and log the firmware PC that writes it, so 0x82/sense 0x29
		// gets a call site instead of a guess.  Node is $71F0 in every observed command.
		// WHICH $2029 SITE FIRES?  Two in the ROM: $979E (retry budget exhausted - `move.w #$5,D5`
		// then subq/bne) and $99FE (a $7a0e / D2 test).  Tap the instruction fetch at each.
		// $2029 is deposited THROUGH A POINTER: `movea.w $71be.w,A0 / move.w #$2029,(A0)` at both
		// $7E02 (-> $7C92, the position-verify failure path) and $8AB2.  A data write, so unlike an
		// instruction fetch this IS visible - the Harvard split hid the fetch taps below.
		cs.install_write_tap(0x71be, 0x71bf, "errptr",
			[this, on_local_bus](offs_t, u16 &data, u16 mem_mask)
			{
				if (!on_local_bus()) return;
				logerror("ERRPTR [$71be] <= %04x pc=%06x cmd=%02x wr_done=%d phase=%d t=%.5f\n",
					data & 0xffff, m_cpu->pcbase() & 0xffffff, m_iopb_cmd, m_wr_done,
					m_sec_phase, machine().time().as_double());
			});
		cs.install_read_tap(0x979e, 0x979f, "err979e",
			[this](offs_t, u16 &, u16)
			{ logerror("ERR979E (retry exhausted) cmd=%02x wr_done=%d phase=%d t=%.5f\n",
				m_iopb_cmd, m_wr_done, m_sec_phase, machine().time().as_double()); });
		cs.install_read_tap(0x99fe, 0x99ff, "err99fe",
			[this](offs_t, u16 &, u16)
			{ logerror("ERR99FE ($7a0e/D2 test) cmd=%02x wr_done=%d phase=%d t=%.5f\n",
				m_iopb_cmd, m_wr_done, m_sec_phase, machine().time().as_double()); });
		cs.install_write_tap(0x71f0, 0x71f3, "nodestat",
			[this, on_local_bus](offs_t offset, u16 &data, u16 mem_mask)
			{
				if (!on_local_bus()) return;
				u32 const wpc = m_cpu->pcbase() & 0xffffff;
				logerror("NODESTAT [%04x] <= %04x mask=%04x pc=%06x cmd=%02x wr_done=%d phase=%d t=%.5f\n",
					unsigned(offset), data & 0xffff, mem_mask & 0xffff,
					wpc, m_iopb_cmd, m_wr_done, m_sec_phase, machine().time().as_double());
				// WHERE DID THE ERROR CODE COME FROM?  $184E is the ROM's generic 0x82 stamp, so the
				// sense byte is supplied by a CALLER.  Dump the stack so the call chain names it.
				if (wpc == 0x184a || wpc == 0x184e)
				{
					address_space &ls = m_cpu->space(AS_PROGRAM);
					u32 const sp = m_cpu->state_int(M68K_SP) & 0xffffff;
					std::string st;
					for (int k = 0; k < 8; k++)
						st += util::string_format(" %06x", ls.read_dword((sp + k * 4) & 0xffffff) & 0xffffff);
					logerror("NODESTAT   stack@%06x:%s  D0=%08x D1=%08x\n", sp, st.c_str(),
						m_cpu->state_int(M68K_D0), m_cpu->state_int(M68K_D1));
				}
			});
		cs.install_write_tap(0x7abc, 0x7abd, "cnt7abc",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				int const v = data & 0xff;
				bool const use = m_read_active && v >= 1 && v <= m_track_n;
				if (use) m_sec_count = v;
				logerror("7ABC-WRITE <= %04x mask=%04x%s t=%.5f\n", data, mem_mask,
					use ? "  -> LATCHED m_sec_count" : "", machine().time().as_double());
			});
		// Soft-timer count arm (inst8 wrap finding): block addresses move (7318/733c/7360…) so
		// catch the WRITE of the count word, not a fixed address.  IRQ1 walk is proven correct;
		// cnt=ffff after first subq means the arm was 0000 (wrap) or ffff (huge).  Log every
		// full-word store of 0000/ffff in the freelist/block pool, plus #$32 (the normal arm) so
		// the healthy path is the same-run control.  PC names $89E8 (reload #$32/#$201C) vs
		// $29F8 (register from stack) vs anything else.
		cs.install_write_tap(0x7300, 0x74ff, "tmrcnt",
			[this, on_local_bus](offs_t offset, u16 &data, u16 mem_mask)
			{
				if (!on_local_bus()) return;
				if (mem_mask != 0xffff) return;          // count is always a full word
				u16 const v = data & 0xffff;
				if (v != 0x0000 && v != 0xffff && v != 0x0032)
					return;
				// Prefer even addresses (block+0 = count); odd-byte halves of other fields are noise.
				if (offset & 1) return;
				address_space &ls = m_cpu->space(AS_PROGRAM);
				u32 const pc = m_cpu->pcbase() & 0xffffff;
				u16 const cell = ls.read_word(0x7986);
				u16 const qh   = ls.read_word(0x736c);
				// code field sits at block+2 when this is a real timer block
				u16 const code = (offset + 2 <= 0x74ff) ? ls.read_word((offset + 2) & 0xffff) : 0xffff;
				char const *tag = (v == 0x0000) ? "ZERO" : (v == 0xffff) ? "FFFF" : "n32 ";
				logerror("TMRCNT %s [%04x]<=%04x pc=%06x code@+2=%04x [$7986]=%04x [$736c]=%04x "
					"cmd=%02x t=%.5f\n",
					tag, unsigned(offset), v, pc, code, cell, qh, m_iopb_cmd,
					machine().time().as_double());
			});
		if (TRACE_CHUNK_PATH)
			cs.install_read_tap(0x7654, 0x7695, "ownmap",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				m_own_n++;
				floppy_image_device *const of = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
				// pc 007d28 == the 007d10 lookup (+0x18 pcbase skew, measured not assumed): it is the
				// ONLY read that indexes this map by the presented ID.  Filter is data-derived from a
				// PC histogram of an unfiltered run; total read count carried inline as the control.
				if (m_cpu->pcbase() != 0x7d28) return;
				m_own_idx++;
				if (m_own_idx <= 400)
					logerror("OWNMAP id=%02x -> %04x mask=%04x pc=%06x cyl=%d %s t=%.5f\n",
						(offset & 0xffff) - 0x7654, data & 0xffff, mem_mask & 0xffff,
						m_cpu->pcbase(), of ? of->get_cyl() : -1,
						flux_density_fm() ? "FM" : "MFM", machine().time().as_double());
				if ((m_own_idx % 25) == 1)
					logerror("OWNMAP-CONTROL: total map reads=%u, indexed-by-ID reads=%u\n",
						m_own_n, m_own_idx);
			});
		if (TRACE_CHUNK_PATH)
			cs.install_read_tap(0x7428, 0x7429, "id_compare",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				m_idc_n++;
				m_idc_cmp++;
				floppy_image_device *const df = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
				u16 const d0 = u16(m_cpu->state_int(M68K_D0));
				if (m_idc_cmp <= 400)
					logerror("IDCMP pc=%06x D0=%04x mem=%04x %s cyl=%d head=%d t=%.5f (reads=%u)\n",
						m_cpu->pcbase(), d0, data & 0xffff,
						(d0 == (data & 0xffff)) ? "MATCH" : "miss",
						df ? df->get_cyl() : -1, m_sel_head & 1,
						machine().time().as_double(), m_idc_n);
			});
		// cont.494: [$741e] is the ARM SOURCE - 008972 `move.w (A1),$c800` with A1=$741e publishes it
		// into C800[0] for every record.  Five writers, ALL `move.w D0,$741e` after the $7696 lookup;
		// none clears it.  It is 0 entering the FM read only because that is the first read after
		// power-on, so the model's hold rescued sector 1 by accident.  Which writer fires at RUN START
		// - and whether it fires at all on MFM - is the question.  No filters; every write printed.
		if (TRACE_CHUNK_PATH)
			cs.install_write_tap(0x741e, 0x741f, "arm_source",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				m_as_n++;
				floppy_image_device *const af = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
				if (m_as_n <= 200)
					logerror("ARMSRC [741e] <= %04x (dst=%04x) pc=%06x cyl=%d %s t=%.5f (n=%u)\n",
						data & 0xffff, (data & 0xffff) << 1, m_cpu->pcbase(),
						af ? af->get_cyl() : -1, flux_density_fm() ? "FM" : "MFM",
						machine().time().as_double(), m_as_n);
			});
		if (TRACE_CHUNK_PATH)
			cs.install_write_tap(0x7426, 0x7427, "claim_gate",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				m_cg_n++;
				floppy_image_device *const cf = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
				if (m_cg_n <= 60 || (data & 0xffff) != m_cg_last)
					logerror("CLAIMGATE [7426] <= %04x pc=%06x t=%.5f cyl=%d %s (n=%u)\n",
						data & 0xffff, m_cpu->pcbase(), machine().time().as_double(),
						cf ? cf->get_cyl() : -1, flux_density_fm() ? "FM" : "MFM", m_cg_n);
				m_cg_last = data & 0xffff;
			});
		// cont.506: the OP-LIST CURSOR.  001560 `movea.w $721a.w,A0` / 001564 `movea.w (A0),A1`:
		// A0 = address of this node's cursor slot, A1 = the cursor.  001582 tests (A1): 0 -> state
		// 0x0c, 0x36 -> state 0x0a, else dispatch via the $192 table.  Cursor is advanced+stored at
		// 00159a/00159e ONLY on the 0 and 0x36 arms.  Every observed state write is 0x0c, so the
		// cursor is at the terminator whenever tested.  Two shapes to separate (Dave, cont.506):
		//   (a) the test runs only AFTER the walk completes -> terminator by construction (ordering)
		//   (b) the saved position is one past the op it should report -> 0x36 skipped (off-by-one)
		// Log the store with BOTH neighbours: the op at the old cursor and at the new one.
		// Range ends ODD - an even end throws and silently installs NOTHING (cont.505).
		if (TRACE_CHUNK_PATH)
			cs.install_write_tap(0x7200, 0x7301, "opcursor",
				[this](offs_t offset, u16 &data, u16 mem_mask)
				{
					u32 const pc = m_cpu->pcbase();
					if (pc < 0x1596 || pc > 0x15c0) return;      // the 00159e store window
					address_space &os = m_cpu->space(AS_PROGRAM);
					u32 const newc = data & 0xffff;
					u32 const oldc = (newc - 2) & 0xffff;
					logerror("CURSOR [%04x] <= %04x  op@old(%04x)=%04x op@new(%04x)=%04x  "
						"D0=%04x  [$721a]=%04x pc=%06x t=%.5f\n",
						offset & 0xffff, newc, oldc, os.read_word(oldc), newc, os.read_word(newc),
						u16(m_cpu->state_int(M68K_D0)), os.read_word(0x721a), pc,
						machine().time().as_double());
				});
		// cont.504: ALL FOUR NODES, not just the active one.  001f88 names them:
		// `lea $71f0,A0 / lea $71c6,A1 / lea $7224,A2 / lea $7250,A3`.  The previous tap watched only
		// [$71bc] (=0x71f0) and concluded "no working case in this run" - a SCOPE artifact, the same
		// class as the extent errors.  The FM label read completes, so its node should walk the full
		// ladder and give a working case beside the failing one.
		// State 0x0a is set ONLY by op 0x36 (the WAIT op) at 001592; op 36 is in the observed ladder.
		// RAM-SWEEP EXCLUSION IS NOW DEFAULT, not per-probe: 009ce6/009d04/009d10/009d46/00073a have
		// faked three results today (print-cap exhaustion, descriptor reads, a bogus working case).
		if (TRACE_CHUNK_PATH)
			cs.install_write_tap(0x71c0, 0x7279, "node26_all",
				[this](offs_t offset, u16 &data, u16 mem_mask)
				{
					u32 const pc = m_cpu->pcbase();
					if (pc == 0x9ce6 || pc == 0x9d04 || pc == 0x9d10 || pc == 0x9d46 || pc == 0x73a)
						{ m_sweep_n++; return; }        // counted, never reported as data
					u32 const a = offset & 0xffff;
					// INLINE CONTROL: a silent zero here must be distinguishable from a dead tap.
					m_n26_all++;
					double const ct = machine().time().as_double();
					if (ct - m_n26_ctl >= 5.0)
					{
						m_n26_ctl = ct;
						logerror("N26-CONTROL t=%.3f: writes seen in 71c0-7280 = %u (sweep excluded %u)\n",
							ct, m_n26_all, m_sweep_n);
					}
					static constexpr u32 NODES[4] = { 0x71c6, 0x71f0, 0x7224, 0x7250 };
					for (int n = 0; n < 4; n++)
						if (a == ((NODES[n] + 0x26) & 0xfffe))
							logerror("N26 node%d(%04x)+26 <= %04x pc=%06x t=%.5f%s  [sweep excluded=%u]\n",
								n, NODES[n], data & 0xffff, pc, machine().time().as_double(),
								((data & 0xff) == 0x0a || ((data >> 8) & 0xff) == 0x0a)
									? "  <<< STATE 0x0A - op36 WAIT reached" : "",
								m_sweep_n);
				});
		// cont.502: does the vec-138 routine at 0x1560 EXECUTE?  0015e2 `cmpi.w #$1,$71b2.w` and
		// 0015ea `tst.w $71b6.w` are inside it.  A pc histogram on those reads separates "vector not
		// taken" from "taken but never reaches the scanner" - the two readings a null DESCRSCAN
		// cannot tell apart.  Other readers of the same words are the control.
		if (TRACE_CHUNK_PATH)
			cs.install_read_tap(0x71b2, 0x71b7, "vec138_entry",
				[this](offs_t offset, u16 &, u16)
				{
					m_v138_hist[(u32(offset & 0xffff) << 24) | (m_cpu->pcbase() & 0xffffff)]++;
					double const vt = machine().time().as_double();
					if (vt - m_v138_last >= 4.0)
					{
						m_v138_last = vt;
						std::string h;
						for (auto const &kv : m_v138_hist)
							h += util::string_format("[%04x]@%06x:%u ",
								(kv.first >> 24) | 0x7100, kv.first & 0xffffff, kv.second);
						logerror("VEC138 t=%.3f | %s\n", vt, h);
					}
				});
		// cont.500: DOES THE DESCRIPTOR SCANNER RUN, and what does it visit?  The scanner at
		// 0015f4-001656 walks a table of descriptor POINTERS via `movea.w -(A0),A0` and compares
		// [+2] against [+4].  Reading that arithmetic off the listing is the step that has been
		// wrong twice in this campaign, so measure it: histogram every read of the pointer table
		// AND of the descriptor block, by pc.  Other readers in the same stream are the control.
		if (TRACE_CHUNK_PATH)
			cs.install_read_tap(0x727c, 0x72ef, "descr_scan",
				[this](offs_t offset, u16 &data, u16 mem_mask)
				{
					m_ds_n++;
					double const dt = machine().time().as_double();
					m_ds_hist[(u32(offset & 0xffff) << 24) | (m_cpu->pcbase() & 0xffffff)]++;
					if (dt - m_ds_last >= 2.0)
					{
						m_ds_last = dt;
						std::string h;
						for (auto const &kv : m_ds_hist)
							h += util::string_format("[%04x]@%06x:%u ",
								kv.first >> 24 | 0x7200, kv.first & 0xffffff, kv.second);
						logerror("DESCRSCAN t=%.3f total=%u | %s\n", dt, m_ds_n, h);
					}
				});
		// cont.499: THE FORK inside handler $794c (the READ path's [$72d6] handler).  Its two arms
		// write the SAME variable with different values, so each arm is the other's control - an
		// absence test that cannot return a false negative:
		//     0079b2  clr.w  $7968     skip arm  -> bra $7a34, no reload
		//     0079be  move.w #$1,$7968 reload arm -> falls into 0079d6 `move.w D0,$741e`
		// If only value 0 ever appears, the read path takes the skip and $741e is never reloaded -
		// which is the 15 ms gap between the table rewrite and the first deposit.
		if (TRACE_CHUNK_PATH)
			cs.install_write_tap(0x7968, 0x7969, "fork_7968",
				[this](offs_t, u16 &data, u16 mem_mask)
				{
					m_fk_n++;
					floppy_image_device *const kf = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
					if (m_fk_n <= 120)
						logerror("FORK [7968] <= %04x %s pc=%06x cyl=%d %s t=%.5f (n=%u)\n",
							data & 0xffff, (data & 0xffff) ? "RELOAD-ARM" : "skip-arm  ",
							m_cpu->pcbase(), kf ? kf->get_cyl() : -1,
							flux_density_fm() ? "FM" : "MFM", machine().time().as_double(), m_fk_n);
				});
		// cont.497: THE FULL TABLE, range derived not inherited: $7696 + 16*6 = $76F6.  Decodes each
		// write into entry/field so the REWRITE (+0, the stride era) and the CLAIM (+2, the sector ID)
		// are distinguishable in one stream, timestamped against HOST GO and the $741e load.  This is
		// the capture that decides whether the claim entry for a record exists at DEPOSIT time.
		if (TRACE_CHUNK_PATH)
			cs.install_write_tap(0x7696, 0x76f5, "chunk_tbl_full",
				[this](offs_t offset, u16 &data, u16 mem_mask)
				{
					u32 const a = offset & 0xffff;
					u32 const pc = m_cpu->pcbase();
					double const t = machine().time().as_double();
					if (t < 5.0) { m_ctf_early++; return; }        // boot RAM sweep - counted, not printed
					if (!m_ctf_ctl) { m_ctf_ctl = true;
						logerror("CHUNKTBL-CONTROL: pre-t5 writes=%u - tap live over $7696-$76F5\n", m_ctf_early); }
					u32 const e = (a - 0x7696) / 6, f = (a - 0x7696) % 6;
					char const *fn = (f == 0) ? "+0 CHUNKADDR(stride era)"
					               : (f == 2) ? "+2 CLAIM(sector id)  " : "+4 CYL               ";
					floppy_image_device *const cf = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
					if (++m_ctf_n <= 600)
						logerror("CHUNKTBL e[%02u] %s <= %04x mask=%04x pc=%06x cyl=%d %s t=%.5f\n",
							e, fn, data & 0xffff, mem_mask & 0xffff, pc,
							cf ? cf->get_cyl() : -1, flux_density_fm() ? "FM" : "MFM", t);
				});
		// cont.496 RANGE DEFECT: 0x76bf covers only table entries 0-6.  The table has >=16 entries
		// ($7696 + 16*6 = $76F6), and the sector-1 claim lives in entry 11 at $76DA - OUTSIDE this
		// range.  That is why "sector 1 is never claimed" was recorded: the instrument could not see
		// it.  Range inherited from the accept_count tap.  Widen to $76F6 before reusing.
		if (TRACE_CHUNK_PATH)
			cs.install_write_tap(0x7654, 0x76bf, "stride_tbl",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				m_tbl_wr_n++;
				u32 const wpc = m_cpu->pcbase();
				// pc 073a is the power-on clear loop: it writes the whole region with zeros and would
				// eat every print slot.  COUNTED (m_tbl_clr_n), never printed - cap the printing only.
				if (wpc == 0x73a) { m_tbl_clr_n++; return; }
				// 009ce8/009d0c/009d1e are the power-on RAM sweep (dbra #$1dff) - it crosses this
				// range incidentally and fired 300+ times before t=0.21, eating every print slot.
				// Gate on the READ WINDOW, and print the excluded count inline as the control so a
				// silent zero here cannot be mistaken for "nothing writes the table".
				double const wt = machine().time().as_double();
				if (wt < 5.0) { m_tbl_early_n++; return; }
				if (!m_tbl_ctl_done) { m_tbl_ctl_done = true;
					logerror("TBLWR-CONTROL: pre-t5 writes=%u (clear-loop=%u) - tap is live\n",
						m_tbl_early_n, m_tbl_clr_n); }
				bool const isid = ((offset & 0xffff) >= 0x7696)
					&& (((offset & 0xffff) - 0x7698) % 6) == 0;   // the +2 sector-ID field
				if (m_tbl_wr_n - m_tbl_clr_n - m_tbl_early_n <= 400)
					logerror("TBLWR%s [%04x] <= %04x mask=%04x pc=%06x t=%.5f (n=%u clr=%u)\n",
						isid ? "-ID" : "   ", offset & 0xffff, data & 0xffff, mem_mask & 0xffff,
						wpc, machine().time().as_double(), m_tbl_wr_n, m_tbl_clr_n);
			});
		if (TRACE_CHUNK_PATH)
			cs.install_write_tap(0x7654, 0x76bf, "accept_count",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				if (!m_read_active) return;
				// The stakes are BYTE writes ($8120 move.b #$c0,(0,a1,d1.w) / $8128 move.b d0,...),
				// and for a byte access mem_mask identifies the logical lane, so the extraction below
				// holds under SRAM_BYTE_SWAPPED.  A WORD write is a different matter - taking the high
				// half is the wrong half once byte accesses resolve to the other one - and no stake is
				// ever a word write, so ignore them rather than guess. (cont.427: this is a fourth
				// model<->SRAM byte boundary, missed when THE_RULE enumerated three.)
				if (mem_mask == 0xffff)
					return;
				u8 const v = (mem_mask == 0x00ff) ? u8(data) : u8(data >> 8);
				if (v != 0xc0)
					return;
				m_accepted_n++;
				// End-of-transfer marker.  The firmware's terminate fork tests ONE cell, one
				// instruction after the last accepted sector takes the remaining count to zero
				// ($7EBE -> $7EC2 not taken -> $7ED0 cmpi.b #$aa,(A0,D0.w) -> $7ED8, stop re-arming).
				// Within a record it TESTS S+1 then STAKES S, so the cell that test will read is
				// (this stake's address + 2) - and the PENULTIMATE stake is the last chance to write
				// it, 4.6 ms ahead of the edge.  Both inputs are the gate array's own: the write
				// address it just saw on the bus, and the count from its op18 program's port-$3F
				// pushes.  Record only; the pump deposits it as a bus-master cycle.
				int const n = m_prog_count > 0 ? m_prog_count : m_sec_count;
				if (false && n > 1 && m_accepted_n == n - 1 && !m_aa_done)
				{
					u32 const addr = (mem_mask == 0x00ff) ? (offset + 1) : offset;
					u32 const cell = addr + 2;
					if (cell >= 0x7654 && cell <= 0x76bf)
					{
						m_aa_cell = cell;
						m_aa_armed = true;
					}
				}
			});
		// (Diagnostic taps removed.  The Lua instruments under docs/storager-lle carry the same
		//  coverage without compiling anything into the device: timeline.lua for the completion
		//  chain, gatrace.lua for every gate-array access, blocks.lua for the control blocks.)
		m_installed = true;
	}
	m_cpu->set_input_line(M68K_IRQ_2, CLEAR_LINE);
	if (m_bus_held)          // a reset mid-DMA drops the gate array's bus hold
	{
		m_cpu->resume(SUSPEND_REASON_HALT);
		m_bus_held = false;
	}
	m_dma_active = false;
	m_e800_bit12_prev = false;
	m_host_int_prev = false;
	m_read_window = false;
	m_cmd = CMD_IDLE;
	m_armed = false;
	m_e000b11_prev = false;
	m_bit11_prev = false;
	m_rec_latch = false;
	m_mark_pending = 0;
	m_sel_head = 0;
	m_sel_drive = 0;
	m_prog_loaded = false;
	m_desc_n = 0;
	m_pit2_out = false;
	m_data_done_n = 0;
	m_status_armed = false;
	m_read_active = false;
	spin_drives();
	logerror("GA model: SRAM byte-swapped, no self-disarm, field program retained across commands\n");
}

void multibus_storager_device::floppy_formats(format_registration &fr)
{
	fr.add_mfm_containers();
	fr.add(FLOPPY_IMD_FORMAT);
}

static void storager_floppies(device_slot_interface &device)
{
	device.option_add("525qd", FLOPPY_525_QD);   // 5.25" 80-track DS (SINIX media)
}

void multibus_storager_device::device_add_mconfig(machine_config &config)
{
	// Board oscillators (storagerii.webp): a 32/10 MHz can and a 50 MHz can.  The CPU runs from the
	// 10 MHz output - the Storager II carries an MC68000-10 and the v180/SGI revisions an MC68000L10,
	// both 10 MHz parts, so the earlier 50/4 = 12.5 MHz would have overclocked them by 25%.
	M68000(config, m_cpu, 10_MHz_XTAL);
	m_cpu->set_addrmap(AS_PROGRAM, &multibus_storager_device::mem_map);
	m_cpu->set_addrmap(AS_OPCODES, &multibus_storager_device::opcodes_map);   // fetches see ROM at 0x4000+

	FLOPPY_CONNECTOR(config, m_floppy[0], storager_floppies, "525qd", multibus_storager_device::floppy_formats);
	FLOPPY_CONNECTOR(config, m_floppy[1], storager_floppies, nullptr, multibus_storager_device::floppy_formats);

	// two physical ESDI drives = two Storager LUNs (unit 0 = -hard1, unit 1 = -hard2).
	STORAGER_HD_IMAGE(config, m_hd[0], 0);
	STORAGER_HD_IMAGE(config, m_hd[1], 0);

	// two 8253 PITs.  pit[0] ctr1 = the mode-5 seek-settle one-shot (STEP-triggered on its gate);
	// pit[1] ctr0 = the system tick (IRQ1), ctr2 spare.
	PIT8253(config, m_pit[0]);
	m_pit[0]->set_clk<0>(10_MHz_XTAL / 8);
	m_pit[0]->set_clk<1>(10_MHz_XTAL / 8);
	m_pit[0]->set_clk<2>(10_MHz_XTAL / 8);
	m_pit[0]->out_handler<1>().set([this](int state) {   // F000 bit1 = seek/settle busy
		bool const settled = state && !m_settle_out;   // OUT rise = on-cylinder, heads settled
		m_settle_out = state;
		// The capture at GO decoded whatever cylinder the head was on THEN.  If the firmware
		// then seeks, that data is from the wrong track, so decode again once the drive is
		// on-cylinder and settled - which is the only moment a floppy can actually be read.
		// Arming is unaffected; this only refreshes what was decoded.
		if (settled && m_read_window && m_iopb_cmd == 0x95 && m_stepped_since_arm)
		{
			m_stepped_since_arm = false;
			capture_track();
			if (m_track_n > 0 && m_sec_count > m_track_n) m_sec_count = m_track_n;
			// Post-seek sample: $6AD0 has run (seek loop finished before settle), so [$7438]
			// is the value the $7C66 0x2012 compare will use.  Mid-STEPEDGE target= was stale.
			address_space &cs = m_cpu->space(AS_PROGRAM);
			floppy_image_device *const fd = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
			u16 const ub = cs.read_word(0x799a);
			u8 const u12 = (ub >= 0x4000 && ub < 0x7f00) ? cs.read_byte((ub + 0x12) & 0xffff) : 0xff;
			int cap_c = (m_track_n > 0) ? m_track[0].c : -1;
			int cap_h = (m_track_n > 0) ? m_track[0].h : -1;
			logerror("SEEK COMPLETE: re-captured sectors=%d drive=%d | TARGET@SETTLE [$7438]=%04x [$7436]=%04x | CACHE cyl=%04x head=%04x pos=%04x | UIB+$12=%02x bit1=%d | capt[0] C=%d H=%d t=%.5f\n",
				m_track_n, fd ? fd->get_cyl() : -1,
				cs.read_word(0x7438), cs.read_word(0x7436),
				(ub >= 0x4000 && ub < 0x7f00) ? cs.read_word((ub + 0xd2) & 0xffff) : 0xffff,
				(ub >= 0x4000 && ub < 0x7f00) ? cs.read_word((ub + 0xd0) & 0xffff) : 0xffff,
				(ub >= 0x4000 && ub < 0x7f00) ? cs.read_word((ub + 0xd6) & 0xffff) : 0xffff,
				u12, BIT(u12, 1), cap_c, cap_h, machine().time().as_double());
		}
	});

	PIT8253(config, m_pit[1]);
	m_pit[1]->set_clk<0>(10_MHz_XTAL / 4);   // ctr0 count 0xFF00 mode 3 -> ~26ms system tick
	m_pit[1]->set_clk<1>(10_MHz_XTAL / 8);
	m_pit[1]->set_clk<2>(10_MHz_XTAL / 8);
	m_pit[1]->out_handler<0>().set(FUNC(multibus_storager_device::timer0_out));
	m_pit[1]->out_handler<2>().set(FUNC(multibus_storager_device::timer2_out));
}

// Instruction-fetch (AS_OPCODES) space: the EPROM is decoded for fetches across 0x0-0xFFFF, so the
// worker code at 0x4000-0x7FFF runs directly from ROM while the data space serves RAM there.
void multibus_storager_device::opcodes_map(address_map &map)
{
	map(0x000000, 0x00ffff).rom().region("cpu", 0).mirror(0xff0000);
}

// Byte accesses resolve to the OTHER half of the stored word; word accesses pass through unchanged.
u16 multibus_storager_device::lram_r(offs_t offset, u16 mem_mask)
{
	u16 const v = m_lram[offset];
	if (mem_mask == 0xff00) return u16(v & 0x00ff) << 8;   // even byte -> low half
	if (mem_mask == 0x00ff) return (v >> 8) & 0x00ff;      // odd byte  -> high half
	return v;
}

void multibus_storager_device::lram_w(offs_t offset, u16 data, u16 mem_mask)
{
	if (mem_mask == 0xff00)      m_lram[offset] = (m_lram[offset] & 0xff00) | ((data >> 8) & 0x00ff);
	else if (mem_mask == 0x00ff) m_lram[offset] = (m_lram[offset] & 0x00ff) | ((data & 0x00ff) << 8);
	else                         m_lram[offset] = (m_lram[offset] & ~mem_mask) | (data & mem_mask);

	// cont.519: a write into the chunk/claim table ($7696, 6-byte entries: +0 chunk byte address,
	// +2 claimed sector) is the moment a held field's destination becomes known.  Read and write
	// m_lram DIRECTLY here - going through the address space from inside a write handler fires the
	// device's own dma_snoop read tap and clobbers m_term_bit0 (the class this campaign hit twice).
	if (true)   // FLUSH_ON_CLAIM - REFUTED
		return;
	u32 const wa = 0x4000 + (offset << 1);
	if (wa < 0x7696 || wa >= 0x76f6)
		return;
	flush_held_on_claim();
}

// Deposit a held data field into the chunk the firmware's claim table names for its sector.
// m_lram is read and written DIRECTLY: going through the address space would fire the device's own
// dma_snoop read tap and clobber m_term_bit0.
void multibus_storager_device::flush_held_on_claim()
{
	for (u32 a = 0x7696; a < 0x76f6; a += 6)
	{
		u32 const i = (a - 0x4000) >> 1;
		if (m_lram[i + 1] != m_held_r)
			continue;
		u32 const claimed = m_lram[i];
		if (claimed < 0x4000 || claimed + m_held_len > 0x8000)
			break;
		for (u32 k = 0; k < m_held_len; k++)
		{
			u32 const b = claimed + k, j = (b - 0x4000) >> 1;
			if (b & 1) m_lram[j] = (m_lram[j] & 0x00ff) | (u16(m_held_data[k]) << 8);
			else       m_lram[j] = (m_lram[j] & 0xff00) | m_held_data[k];
		}
		logerror("held field (R=%u) flushed ON CLAIM into chunk %04x t=%.5f\n",
			m_held_r, claimed, machine().time().as_double());
		m_held_len = 0;
		break;
	}
}

void multibus_storager_device::mem_map(address_map &map)
{
	// The board decodes only A1-A15 (A16-A23 don't-care); the firmware reaches ROM/RAM/I/O via 68000
	// short-absolute and PC-relative addressing that sign-extends to 0xFFxxxx, so everything mirrors
	// across A16-A23.
	map(0x000000, 0x00ffff).rom().region("cpu", 0).mirror(0xff0000);
	if (SRAM_BYTE_SWAPPED)
		map(0x004000, 0x007fff).rw(FUNC(multibus_storager_device::lram_r), FUNC(multibus_storager_device::lram_w));
	else
		map(0x004000, 0x007fff).ram();

	// two 8253 PITs interleaved at 0x8000 (#0 = high byte, #1 = low byte)
	map(0x008000, 0x008007).mirror(0xff0000).rw(m_pit[0], FUNC(pit8253_device::read), FUNC(pit8253_device::write)).umask16(0xff00);
	map(0x008000, 0x008007).mirror(0xff0000).rw(m_pit[1], FUNC(pit8253_device::read), FUNC(pit8253_device::write)).umask16(0x00ff);

	// VGC7219 gate array
	map(0x00c000, 0x00c7ff).mirror(0xff0000).w(FUNC(multibus_storager_device::c000_w));
	map(0x00c800, 0x00c9ff).mirror(0xff0000).rw(FUNC(multibus_storager_device::c800_r), FUNC(multibus_storager_device::c800_w));
	map(0x00d000, 0x00d001).mirror(0xff0000).w(FUNC(multibus_storager_device::d000_w));
	map(0x00d800, 0x00d801).mirror(0xff0000).w(FUNC(multibus_storager_device::d800_w));
	map(0x00e000, 0x00ffff).mirror(0xff0000).rw(FUNC(multibus_storager_device::ch_r), FUNC(multibus_storager_device::ch_w));

	// Multibus memory window (bus master): 0x010000-0xFEFFFF -> Multibus, 1:1 (CPUAP RAM at 0x0xxxxx).
	map(0x010000, 0xfeffff).rw(FUNC(multibus_storager_device::bus_mem_r), FUNC(multibus_storager_device::bus_mem_w));
}

ROM_START(storager)
	ROM_REGION16_BE(0x10000, "cpu", 0)

	// The PC-MX2 board is the v260 variant; declare it explicitly rather than relying on the
	// implicit "first ROM_SYSTEM_BIOS wins" rule.  The sgic/sgib entries are a different
	// manufacturer's board (SGI Storager 3030) whose firmware is close enough to v260 that a
	// wrong selection disassembles into plausible code at plausible addresses - it does not
	// announce itself.  siemens/disasm/storager/README.md carries the matching SHA1s.
	ROM_DEFAULT_BIOS("v260")

	// Siemens MX-300/PC-MX2, MC68000P12, 50MHz & 32MHz crystals, VGC7219-0419 II-SER--24M (default)
	ROM_SYSTEM_BIOS(0, "v260", "v260")
	ROMX_LOAD("05820084260.u84", 0x0000, 0x8000, CRC(43616528) SHA1(e7e84566b0db49d83feba4014fb319f30767d934), ROM_SKIP(1) | ROM_BIOS(0))
	ROMX_LOAD("05820085260.u85", 0x0001, 0x8000, CRC(206c064d) SHA1(a47f6f0174d0c5ffbfa61ec6724ce549585cdaeb), ROM_SKIP(1) | ROM_BIOS(0))

	// MC68000L10, 40MHz & 32MHz crystals, 74LS1801F & 74LS1802A
	ROM_SYSTEM_BIOS(1, "v180", "v180")
	ROMX_LOAD("58084180.u84", 0x0000, 0x8000, CRC(38805bbf) SHA1(f1ed5419cc14dfb75944183742597da2d4940d85), ROM_SKIP(1) | ROM_BIOS(1))
	ROMX_LOAD("58085180.u85", 0x0001, 0x8000, CRC(e21001ce) SHA1(d380b8b0a284f51ded1532d0ba05338d28d89466), ROM_SKIP(1) | ROM_BIOS(1))

	// SGI 026-0005-001 REV C S/N 6202, MC68000L10, "Storager 3030", 40MHz & 32MHz crystals
	ROM_SYSTEM_BIOS(2, "sgic", "sgic")
	ROMX_LOAD("05808423a.u84", 0x0000, 0x8000, CRC(161e6a90) SHA1(d4dcbf630a83e4c5994d8331ac85d81130400e33), ROM_SKIP(1) | ROM_BIOS(2))
	ROMX_LOAD("05808523a.u85", 0x0001, 0x8000, CRC(4c99e4b8) SHA1(899855e54c4520816ad43eb19b972b45783ccb6b), ROM_SKIP(1) | ROM_BIOS(2))

	// SGI 026-0005-001 REV B S/N 6666, MC68000L10
	ROM_SYSTEM_BIOS(3, "sgib", "sgib")
	ROMX_LOAD("0580008426b.u84", 0x0000, 0x8000, NO_DUMP, ROM_SKIP(1) | ROM_BIOS(3))
	ROMX_LOAD("0580008526b.u85", 0x0001, 0x8000, NO_DUMP, ROM_SKIP(1) | ROM_BIOS(3))
ROM_END

static INPUT_PORTS_START(storager)
INPUT_PORTS_END

const tiny_rom_entry *multibus_storager_device::device_rom_region() const
{
	return ROM_NAME(storager);
}

ioport_constructor multibus_storager_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(storager);
}

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(MULTIBUS_STORAGER, device_multibus_interface, multibus_storager_device, "storager", "Interphase 3030 Storager")
