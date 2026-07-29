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

// TEMP (STRIP): A/B knob for the record-timing model.  false = the arm-driven model (records as fast
// as the firmware re-arms) which currently transfers 8/8; true = physically timed records (one sector
// per 12.5ms at 300rpm), which is correct but exposes the dead $833C restart path (cont.409).
constexpr bool PHYSICAL_TIMING = true;

// TEMP (STRIP): A/B knob for record delivery.  false = the model walks the COMMANDED run and stops
// after [$7abc] sectors - i.e. the gate array chooses which sector, which is the firmware's job.
// true = one record per address mark as the disk rotates, and the FIRMWARE decides whether to arm the
// data phase (cont.411: $89F2 inspects the raw capture cells in software; there is no hardware sector
// comparator).  The firmware stops re-arming when it has what it wants, which is what ends the run.
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
constexpr bool Q3B_END_MARKER = false;

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
	void timer2_out(int state);

	void spin_drives();
	void floppy_swap_cb(floppy_image_device *fdd);

	// The read window: the 74LS1801/1802 ENDEC recovers the FM/MFM cell stream from the drive's real flux
	// (get_next_transition) and the gate array deserialises it, doing address-mark detect and filling the
	// gate array's capture cells + raising IRQ5/IRQ6 per field boundary.
	bool flux_density_fm() const;
	attotime sector_period() const; // one sector's rotation at 300 rpm
	void start_field_program();     // the loaded op18 program begins running: open the read window
	void advance_read();            // stage + deliver the next record while the window is armed (IRQ6 then IRQ5)
	void deliver_mark();            // arm + held-mark -> one interrupt
	TIMER_CALLBACK_MEMBER(pump_tick);

	// channel DMA: on the E800 bit12 kickoff the gate array bus-masters one field between the local
	// SRAM buffer and host memory via the C000 up-counter, then raises IRQ4 (channel/DMA done).
	void run_channel_dma();
	attotime dma_time(u32 len) const;  // bus-hold / transfer time for a len-byte bus-master DMA
	TIMER_CALLBACK_MEMBER(dma_done);   // transfer end: release the bus hold, raise the channel-done IRQ4

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
	// F000 bit1 is the floppy class's seek/settle busy, so the ack only takes over the bit once a
	// serial transaction has actually clocked (m_ser_active), and that is reset per command.
	bool m_ser_clk = true;       // last E802 bit0 seen
	bool m_ser_active = false;   // a serial transaction has driven the clock this command
	bool m_settle_out = true;    // PIT0 ctr1 mode-5 one-shot OUT; F000 bit1 seek/settle busy = !OUT
	bool m_pit2_out = false;     // PIT1 ctr2 OUT (recorded, not surfaced on F000)
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
	u8 m_sel_drive = 0;          // E804 low byte: drive select + motor/write-current/precomp controls

	// host doorbell / IOPB
	u8  m_iopb_cmd = 0;          // command byte from the auto-fetched IOPB (drives the model's channel)
	std::unique_ptr<u16[]> m_lram;   // SRAM_BYTE_SWAPPED backing store
	u16 lram_r(offs_t offset, u16 mem_mask);
	void lram_w(offs_t offset, u16 data, u16 mem_mask);
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
	emu_timer *m_dma_done = nullptr;   // async channel-done IRQ4

	// The gate array captures the whole track as it rotates (detection-is-capture) into m_track, then the
	// the engine (advance_read) delivers a record per address mark, one IRQ6 + one IRQ5 each, for as long
	// as the firmware keeps re-arming E802 bit11 - it does NOT count down a commanded run.
	struct captured_sector { u8 c = 0, h = 0, r = 0, nn = 0, dam = 0; u16 len = 0; u8 data[1200] = {}; };
	captured_sector m_track[32];
	int  m_track_n = 0;             // sectors recovered in the last full-track capture
	void capture_track();          // synchronous full-revolution flux decode into m_track
	bool m_read_active = false;     // a read window is open (records delivered until the firmware stops re-arming)
	int  m_sec_count = 0;           // commanded sectors this operation ([$7abc])
	u32  m_held_len = 0;            // first field of a run, awaiting a chunk to land in
	u8   m_held_data[1024] = {};
	int  m_sec_index = 0;           // sector currently being delivered
	int  m_sec_phase = 0;           // 0 = ID (IRQ6) next, 1 = data (IRQ5) next
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
	fdd->ss_w(m_sel_head & 1);
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
	logerror("capture_track: cyl=%d head=%d density=%s  sectors=%d  first: r=%02x len=%d  last: r=%02x len=%d\n",
		m_floppy[0] && m_floppy[0]->get_device() ? m_floppy[0]->get_device()->get_cyl() : -1,
		m_sel_head, flux_density_fm() ? "FM" : "MFM", m_track_n,
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
	if (m_iopb_cmd != 0x94 && m_iopb_cmd != 0x95)
		return;
	// The commanded sector count is carried by the program the firmware just loaded - one port-$3F
	// push per sector - so the gate array has it without reading the IOCB or any firmware cell.
	{
		int p3f = 0;
		for (int i = 0; i < m_desc_n; i++)
			if (m_desc_port[i] == 0x3f)
				p3f++;
		m_prog_count = p3f;
	}
	m_read_window = true;
	m_armed = true;
	bool const first_arm = !m_read_active;
	if (!m_read_active && m_iopb_cmd == 0x95)
	{
		capture_track();
		// per-COMMAND state.  NOTE: start_field_program is re-entered on EVERY record (the ISR
		// re-issues the program's first command, E000 <= $0a6d, bit11 set), so anything that must
		// happen once per read belongs in this block, never above it.
		m_aa_cell = 0; m_aa_armed = false; m_aa_done = false;
		m_accepted_n = 0; m_aa_last = 0; m_aa_n = 0;
		int n = m_cpu->space(AS_PROGRAM).read_word(0x7abc) & 0xff;   // commanded sectors this track [$7abc]
		if (n < 1 || n > m_track_n) n = m_track_n;
		m_sec_count = n;
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
	m_pump->adjust(attotime::from_usec(PHYSICAL_TIMING ? 100 : 200));
	if (!PHYSICAL_TIMING)
		advance_read();      // stage + deliver the first record if the firmware is armed
}

// Deliver the held mark's interrupt IFF the firmware has armed (a bit-change PRIOR to the interrupt).
// arm + held-mark -> exactly one IRQ; consumes both, so only one interrupt is ever live at a time.
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
	if (m_mark_pending || !m_read_active)
		return;                             // a record is held awaiting the arm, or the read is complete
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
	if (PHYSICAL_TIMING && machine().time() < m_next_rec)
		return;                             // this field has not passed the head yet
	captured_sector const &s = m_track[m_sec_index];   // physical order = ascending R for the boot read
	address_space &cs = m_cpu->space(AS_PROGRAM);
	// The gate array runs op18's field sequence autonomously.  Per sector it raises the ID address mark
	// (IRQ6 -> $298C -> $89F2 verify, C/H/R staged at $7DAC) then two data-field events (IRQ5 -> $29C0):
	// #1 -> $7BA8 setup (arms the chunk, leaves alternator bit0=1), #2 -> $8018 done (stamps $74C4+2=0x40
	// READY, stakes the ledger).  Measured routing per sector = VERIFY x2 / SETUP / DONE (alternator 1,0,1,0),
	// an even parity cycle; the second verify arrives via the IRQ6 HOLD_LINE re-fire (making it explicit here
	// via a second pump-delivered IRQ6 mis-times the transfer cadence and regresses to zero transfers).
	if (m_sec_phase == 0)
	{
		// Deposit the RAW RECOVERED ID FIELD at the address the firmware COMMANDED through the D800
		// latch - the gate array writes where it was told, it does not know about $7DAC.  The layout is
		// exactly what came off the surface, so it is density-dependent, and the firmware's readers
		// expect precisely that: $9884 (the real sector-select compare) combines bytes 0-2 to $A1,
		// requires $FE at byte 3 and takes the cylinder from byte 4(+5), gated on UIB+$12 bit2; the FM
		// reader $7C7A instead expects $FE at byte 0, because FM HAS NO $A1 SYNC BYTES.
		u32 dst = u32(m_d800) << 1;
		if (dst < 0x4000 || dst + 8 > 0x8000)
			dst = 0x7dac;                       // not yet latched (first record of a command)
		int k = 0;
		if (!flux_density_fm())
			for (int p = 0; p < 3; p++)
				cs.write_byte((dst + k++) & 0xffff, 0xa1);   // MFM sync preamble
		cs.write_byte((dst + k++) & 0xffff, 0xfe);           // ID address mark
		cs.write_byte((dst + k++) & 0xffff, s.c);            // C
		cs.write_byte((dst + k++) & 0xffff, s.h);            // H
		cs.write_byte((dst + k++) & 0xffff, s.r);            // R (sector)
		cs.write_byte((dst + k++) & 0xffff, s.nn);           // N
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
		logerror("FIELD r=%02x c=%02x h=%02x -> chunk %04x  (sec_index=%d, done_n=%d)\n",
			s.r, s.c, s.h, dst, m_sec_index, m_data_done_n);
		if (dst >= 0x4000 && dst + s.len <= 0x8000)
		{
			for (int k = 0; k < s.len; k++)
				cs.write_byte((dst + k) & 0xffff, s.data[k]);
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
			std::copy_n(s.data, s.len, m_held_data);
			logerror("FIELD r=%02x held - no chunk armed yet\n", s.r);
		}
		m_mark_pending = 5;                     // data field captured (IRQ5 #2 -> $8018 done)
		m_sec_phase = 0;
		m_sec_index++;                          // one sector completed after its data-done record
		m_next_rec = machine().time() + sector_period() * 15 / 100;   // trailing gap -> next ID
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
	deliver_mark();
	if (m_read_window)
		advance_read();
	// Q3b: deposit the end marker once the firmware has signalled it bootstrapped.  Done HERE, from the
	// device's own context: currently_executing() is not the local CPU, so the model's $4000-$7FFF snoop
	// taps ignore it and m_term_bit0 / m_last_bw stay clean.  From the next record onward the firmware
	// tests it at $7E9A (the [$7968]!=0 arm) and terminates via $7ED8.
	if (Q3B_END_MARKER && m_aa_armed && m_aa_cell >= 0x7654 && m_aa_cell <= 0x76bf)
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
	bool const to_host  = !BIT(e800, 14) && BIT(e800, 13);
	// A read data transfer sources the captured SRAM chunk ($8018 sets C800[0]=[$741e] = the chunk word
	// pointer), distinguishing it from the IOPB/UIB control blocks (which source the node work area).  For
	// the data field the gate array moves exactly what op18 programmed - one FM sector - so honour the
	// captured field length rather than the control-block size.
	bool const is_data = to_host && m_sec_count > 0 && ld >= 0x4000 && ld < 0x7000;   // the linear data buffer
	u32 const len = is_data ? m_track[0].len : (BIT(e800, 13) ? 0x18 : 0x20);   // node = 0x18, UIB = 0x20
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
	if ((to_local || to_host) && ld >= 0x4000 && ld + len <= 0x8000 && m_c000 >= 0x010000 && m_c000 < 0xff0000)
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
			logerror("DATA->host %06x <- chunk %04x len=%d  first16: \"%s\"\n", m_c000, ld, len, t);
		}
		// TEMP: the host's completion verdict is node+3, which the swap lands at host+2.
		if (to_host && !is_data && len > 3)
		{
			u8 const stv = cs.read_byte((ld + 3) & 0xffff);
			u8 const st2 = cs.read_byte((ld + 2) & 0xffff);
			// The swap sends node+2 -> host+3 and node+3 -> host+2.  The firmware's two stamps differ in
			// operand size: $1314 move.w #$81,($2,A0) puts BUSY in node+3; $1A54 move.b #$80,($2,A0) puts
			// the COMPLETION verdict in node+2, with node+3 then taking D5 (the clamped retry byte, $1A78).
			logerror("HOST POST: node+2=%02x node+3=%02x  -> host+2=%02x host+3=%02x  %s t=%.5f\n",
				st2, stv, stv, st2,
				st2 == 0x80 ? "*** COMPLETION (0x80) ***" : (stv == 0x81 ? "busy" : "other"),
				machine().time().as_double());
		}
		// TEMP cont.443: the UIB fetch.  It is re-fetched per operation here and carries FM
		// (bit1 clear at +$12) for a cylinder-1 read that is MFM by construction.  Log the SOURCE
		// address and what the host actually holds, to test whether the right block is fetched.
		if (to_local && !is_data && ld == m_uib_base && len >= 0x14)
		{
			char h[0x14 * 3 + 1]; h[0] = 0;
			for (u32 k = 0; k < 0x14; k++)
				sprintf(h + k * 3, "%02x ", bs.read_byte((m_c000 + k) & 0xffffff));
			logerror("UIB FETCH host %06x -> local %04x len=%d: %s\n", m_c000, ld, len, h);
		}
		for (u32 k = 0; k < len; k++)
		{
			if (to_local) cs.write_byte((ld + k) & 0xffff, bs.read_byte((m_c000 + k) & 0xffffff));
			else          bs.write_byte((m_c000 + k) & 0xffffff, cs.read_byte((ld + k) & 0xffff));
		}
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

// End of the bus-master transfer: the gate array releases the local bus and raises the channel-done IRQ4.
TIMER_CALLBACK_MEMBER(multibus_storager_device::dma_done)
{
	if (m_bus_held)
	{
		m_cpu->resume(SUSPEND_REASON_HALT);
		m_bus_held = false;
	}
	m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE);
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
		if (!m_floppy_loaded)
			spin_drives();
		floppy_image_device *const fdd = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
		bool const present = fdd && fdd->exists();
		bool const trk0 = present && !fdd->trk00_r();   // trk00_r active-low
		bool const wprot = present && !fdd->wpt_r();    // wpt_r active-low
		bool const index = present && fdd->idx_r();

		// bit12 DMA address-match (the transfer pointer reached the latched terminal)
		u32 const term = 0x4000 | m_dma_term | m_term_bit0;
		d = (d & ~0x1000) | ((m_dma_active && m_last_bw == term) ? 0x1000 : 0);
		d = (d & ~0x0800) | (m_timer_out ? 0x0800 : 0);   // bit11 = PIT timer OUT
		d = (d & ~0x0080) | (present ? 0x0080 : 0);       // bit7  = drive ready / door-closed
		d = (d & ~0x0020) | (present ? 0x0020 : 0);       // bit5  = drive READY (fw 0x767c btst #5)
		d = (d & ~0x2000) | (trk0 ? 0 : 0x2000);          // bit13 = track 0, active-low (fw 0x5de4 btst #$d)
		d = (d & ~0x0400) | (wprot ? 0x0400 : 0);         // bit10 = write-protect
		d = (d & ~0x0010) | (index ? 0x0010 : 0);         // bit4  = index pulse
		if (m_ser_active)
			d = (d & ~0x0002) | (m_ser_clk ? 0x0002 : 0);  // bit1 = ESDI transfer-acknowledge, follows the clock
		else
			d = (d & ~0x0002) | (m_settle_out ? 0 : 0x0002);  // bit1 = seek/settle busy (PIT ctr1 one-shot, low = busy)
		// V2: surface PIT1 ctr2 OUT on bit8 (suspected op42 Branch-B busy/fault).  Baseline leaves
		// bit8 clear.  Read path is Branch A so a change here should not move op42 by itself.
		d &= ~0x0100;                                    // bit8 unmodeled; keep clear
		d &= ~0x0200;                                    // bit9 unmodeled; keep clear
		d &= ~0x0009;                                    // bits 0/3 = drive faults (errors $10 / $1B); a healthy unit reports none
	}
	return d;
}

void multibus_storager_device::ch_w(offs_t offset, u16 data, u16 mem_mask)
{
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
		u16 const code = data & 0x07ff;   // mask off bit11 (window) - the code may ride the same write
		if ((code == 0x22f || code == 0x23f) && (m_iopb_cmd == 0x94 || m_iopb_cmd == 0x95)) m_cmd = CMD_IDHUNT;
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
		bool const win = BIT(data, 11) && m_prog_loaded && !m_prog_loading && (m_iopb_cmd == 0x94 || m_iopb_cmd == 0x95);
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
		m_sel_head = u8(~(data >> 8) & 0x0f);
		m_sel_drive = u8(data & 0xff);
	}
	else if (a == 0xe802)
	{
		// bit0 = the ESDI serial clock.  The drive acknowledges by following it.
		if (ACCESSING_BITS_0_7)
		{
			bool const clk = BIT(data, 0);
			if (m_ser_clk && !clk)
			{
				if (!m_ser_active)
					logerror("SER: ack takes F000 bit1 (clock 1->0) cmd=%02x e802=%04x t=%.5f\n",
						m_iopb_cmd, data, machine().time().as_double());
				m_ser_active = true;    // a real transaction: the controller drove the clock low
			}
			m_ser_clk = clk;
		}
		// bit6 = the IRQ2 ack the doorbell handler toggles LOW on entry.
		if (!BIT(data, 6))
			m_cpu->set_input_line(M68K_IRQ_2, CLEAR_LINE);
		// bit11 = per-record capture re-arm (the `ori #$800` each read ISR issues, $7BA8/$8018).  Its
		// RISING edge primes the gate array to latch the NEXT record; the arm persists (a one-shot) until
		// a record consumes it, so the decoupled 200us pump can deliver the mark whenever it reaches it.
		// When the firmware stops re-arming (read complete), records stop and it converges to completion.
		bool const b11 = BIT(data, 11) && (m_iopb_cmd == 0x94 || m_iopb_cmd == 0x95);
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

		// bit0 = HD/floppy STEP pulse, bit13 = step direction (fw @0x1066/0x1088: bit13 set = inward).
		// MAME steps on the 1->0 edge; dir_w=1 -> toward track 0.  Each STEP retriggers the PIT ctr1
		// mode-5 settle one-shot on its gate (F000 bit1 busy until it times out).
		if (floppy_image_device *const fdd = m_floppy[0] ? m_floppy[0]->get_device() : nullptr)
		{
			fdd->dir_w(BIT(data, 13) ? 0 : 1);
			fdd->stp_w(BIT(data, 0));
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
		u32 const dst = u32(m_c800[0]) << 1;
		if (dst >= 0x4000 && dst + m_held_len <= 0x8000)
		{
			address_space &cs = m_cpu->space(AS_PROGRAM);
			for (u32 k = 0; k < m_held_len; k++)
				cs.write_byte((dst + k) & 0xffff, m_held_data[k]);
			logerror("held field flushed into chunk %04x t=%.5f\n", dst, machine().time().as_double());
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
	if (m_cpu->pcbase() == 0x30e0)   // TEMP (STRIP): op18's triple loop - block ptr + the live triple
		logerror("TEMPC800 cell[%02x] <= %04x  A2=%06x D1=%02x D0=%02x D5=%02x [7938]=%04x\n",
			offset & 0xff, data, u32(m_cpu->state_int(M68K_A2)),
			u32(m_cpu->state_int(M68K_D1)) & 0xff, u32(m_cpu->state_int(M68K_D0)) & 0xff,
			u32(m_cpu->state_int(M68K_D5)) & 0xff, m_cpu->space(AS_PROGRAM).read_word(0x7938));
	else
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
	COMBINE_DATA(&m_d800);
}

// ---------------------------------------------------------------------------
// host / Multibus interface
// ---------------------------------------------------------------------------

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
		m_iopb_addr = dbi;
		m_window_seen = false; m_term_fired = false; m_idx_prev = false; m_read_active = false;   // per-command reset
		// The gate array RETAINS its field program across command boundaries - the firmware says so
		// explicitly.  $5FF6 cmpi.w #$1,$793e / beq $6028 makes the builder emit NEITHER op1A NOR op18
		// when the program already in the gate array is the one this command needs; op18 publishes the
		// identity itself at $3094 (move.w $793c,$793e), and [$793c] carries 1/2/3/4 from seven
		// builders with a proper invalidate path ($0E48/$0EC2/$3142/$317C write #$ffff, $2756 clears).
		// Clearing m_prog_loaded per command therefore threw away a program the firmware was relying on
		// us to keep: the SECOND read emitted no op18, so no C800/E000 burst ever arrived, nothing was
		// armed, and the read produced one record's worth of activity and then silence. (cont.429)
		m_desc_n = 0;   // the descriptor capture buffer is per-load, but the LOADED program persists
		// The firmware carries the STATUS/ERROR bytes back to the host IOPB itself, via its node->host
		// bus-master DMA (run_channel_dma, E800 bit13); the model transcribes nothing here.
		m_held_len = 0;   // no field carries across a command boundary
		m_ser_active = false; m_ser_clk = true;   // the serial ack does not carry across a command boundary
		// Engagement must be decided HERE for a retained program.  With op18 skipped, nothing in the
		// ladder (24 28 56 58 54 4A 42 36 00) writes E000, so the bit11 test below never evaluates and
		// start_field_program() would never re-trigger.  Both inputs are the gate array's own: it knows
		// whether it still holds a program, and it has the command byte it just fetched.
		m_read_window = false;
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

// PIT1 ctr0 OUT = the system tick -> 68000 IRQ1 (handler 0x2b58 walks the software-timeout queue and
// re-pets the counter via the E800 bit9 gate).  count 0xFF00 (mode 3, MSB-only) = ~26ms at /4.
void multibus_storager_device::timer0_out(int state)
{
	bool const rising = state && !m_timer_out;
	m_timer_out = bool(state);          // F000 bit11
	if (rising && m_gate0)
		m_cpu->set_input_line(M68K_IRQ_1, HOLD_LINE);
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
	m_pump = timer_alloc(FUNC(multibus_storager_device::pump_tick), this);
	m_dma_done = timer_alloc(FUNC(multibus_storager_device::dma_done), this);
}

void multibus_storager_device::device_reset()
{
	if (!m_installed)
	{
		// Multibus PIO window 0x7200-0x73FF -> on-board dual-port RAM 0x7E00-0x7FFF (+0xC00).
		m_bus->space(AS_IO).install_readwrite_handler(0x7200, 0x73ff,
			read16sm_delegate(*this, FUNC(multibus_storager_device::host_win_r)),
			write16s_delegate(*this, FUNC(multibus_storager_device::host_win_w)));
		// data aperture the host reads kernel blocks through (0xF00000 = bus I/O 0x0000).
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
				if (Q3B_END_MARKER && n > 1 && m_accepted_n == n - 1 && !m_aa_done)
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
	m_pit[0]->out_handler<1>().set([this](int state) { m_settle_out = state; });   // F000 bit1

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
