// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay, Dave Rand

/*
// Interphase Storager III S26361-F415
// D8253C-2 * 2  programmable interval timer
// MC3486P       quad EIA receiver
// MC3487P       quad EIA driver
// 10MHz
// VLSI 8842AV R2932  VGC7219-0419  II-SER--24M
// MCM6164C45 * 2   8kx8 SRAM
// AM2147-55PC   4kx1 SRAM
// 50MHz
// AM27S19APC * 2 32x8 bipolar PROM
// MC68000P12
// 74LS1801F     Signetics fm/mfm/differential manchester encoder/decoder
// 74LS1802A     Signetics serializer/deserialzer (CRC-16, CRC-CCITT, full duplex, CRC/ECC on receive, 8/16 bit bus)
// AM2147-55DC   4096x1 static RAM

4.2.1.2    Interphase 3030 Storager ESDI/ST-506/QIC-02
	-------------------------------------------

The Storager will support both ESDI and ST-506 drives, but in these systems
is used only for ESDI and cartridge tape.  It too supports floppies
(3, 5, and 8 inch), but, again, I have no knowledge of their being used.

There have been reports of difficulty in using large ESDI drives with
the Interphase Storager, the suggestion being that there is a size
limit on the drive that may be used.  In fact, the problem is not in the
size of the drive, but rather the higher data transfer rate that generally
accompanies larger drives.  Different versions of the Storager exist,
capable of sustaining different data rates.  According to Interphase:

Storager		CC00047-**,Rev    10MHz
Storager II		CC00058-**,Rev    12.5MHz
Storager III    CC00105-**,Rev    24MHz
Storager IIID   CC00119-**,Rev    24MHz

For example, if you're running a Storager II with a Hitachi DK512-17 disk
(134 Meg, 10Mbit/sec xfer) and want to replace it with, or add, a larger
drive, you may run into trouble with a Micropolis 1516 (678 Meg, 20Mbit/sec).
You'd likely have better results with a CDC 94186-383 (383 Meg, 10Mbit/sec).
This is largely anecdotal, and not based on personal experience.
If you know otherwise, please supply details.

NOTE:  All references to the Storager in SGI documentation explicitly
	   state "Storager II", so I believe that to be the only version
	   they shipped.  Either that, or they were sloppy in their field
	   service docs and actually shipped the Storager III with the
	   Hitachi DK514-38 (330meg, 15Mbit/sec).  Perhaps someone who has
	   one of these drives will confirm the identity of the controller.

[/usr/include/multibus/si*.h]

product may be Interphase 3030?
Siemens likely is Storager II (because 12.5MHz CPU)
mapped at multibus pio 7200-73ff (512 bytes) in CPUAP space, int 2

*/

#include "emu.h"

// cont.53 (STRIP): the grind-rate bucket - counters for the minutes-scale convergence
// question (does the $74ac drain outpace the restock; are dequeues hollow). Reported and
// reset every 10s from the E802-write path.
static struct { int match17, dequeue, restock, kick; double last; } s_grind;
// cont.55 (v2 pipeline state): armed by the E802 GO (descriptor staged); driven by the
// C800[00] counter bumps (the shared clock between disk-side deposit and host-side carry);
// drained on count-exhaust; DONE strictly after the final carry.
static struct { bool active; u32 host, done, total, base; u16 vprev, dmap, winbase; u32 aims[16]; u32 pask, pspan; u32 paims[16]; u32 phost;
	// cont.256h: a read may span SEVERAL tracks (the count-8 label read = 4096B = 32 FM
	// sectors = both cyl-0 tracks). base_trk latches the logical track (cyl*heads+head)
	// the transfer started on, so each track's sectors land in their own 2KB portion
	// instead of overwriting portion 0. sec0/ssz/spt are latched with it for the verifier.
	u32 base_trk; u8 sec0; u16 ssz; u16 spt; bool verified; } s_desc;
#include "storager.h"

#include "cpu/m68000/m68000.h"
#include "machine/pit8253.h"
#include "imagedev/floppy.h"
#include "imagedev/harddriv.h"
#include "machine/fdc_pll.h"   // cont.263: the raw-bitstream pivot - the 74LS1811 ENDEC is a PLL
                               // data separator over floppy->get_next_transition(); the 74LS1812
                               // SERDES does byte assembly + FM/MFM address-mark detect.
#include "formats/imd_dsk.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <map>
#include <vector>

#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"

// The ESDI rigid disks as raw sector images OR CHDs.  Derives from the standard harddisk image
// device, so a flat .img (hard_disk_file wraps it at 512-byte LBAs) and a .chd both load and are
// accessed identically.  The Storager firmware model addresses the medium as a flat byte array,
// so img_read/img_write translate byte offsets onto the LBA sector interface (every caller uses
// 1KB-aligned offsets/lengths).
//
// SINIX disktab `micropolis1325`: the PHYSICAL drive is 5 heads x 9 sectors (1KB physical
// sectors: pl#1024/ps#9/ls#9), but SINIX ADDRESSES it via the logical block geometry se#512,
// nc#1024, nt#8, ns#18 = 1024*8*18*512 = 75'497'472 bytes (73728 KB) -- that is the LBA layout
// on the medium.  Access here is flat by LBA, so only se#512 granularity + total size matter;
// the head/sector split is cosmetic.  A faithful CHD therefore uses the logical geometry
// (chdman createhd -chs 1024,8,18 -ss 512), NOT the physical 5x9 (which does not tile the image).
class storager_hd_image_device : public harddisk_image_device
{
public:
	storager_hd_image_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

	// accept raw .img in addition to the standard CHD/raw hard-disk container extensions
	virtual char const *file_extensions() const noexcept override { return "img,chd,hd,hdv,2mg,hdi"; }

	// flat byte-array view over the sector interface (works for raw .img and .chd alike)
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

// Byte offsets/lengths are 1KB-aligned in every caller, so they are always sector-aligned for the
// 512B (raw) or CHD sector size; the loops read/write whole sectors with no read-modify-write.
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
		if (chunk < sb)   // partial tail (never hit with 1KB-aligned callers): preserve the rest
		{
			std::memset(sec.data(), 0, sb);
			read(u32((off + done) / sb), sec.data());
		}
		std::memcpy(sec.data(), s + done, chunk);
		write(u32((off + done) / sb), sec.data());
	}
}

namespace {

// cont.255l (Dave, 2026-07-18): THE ENV-GATE FREEZE. Behavioral configuration is
// CODE, not environment. The validated boot configuration (runs 262-267) is baked
// in: the winning recipe's gates are hard-ON, every other behavioral gate is
// hard-OFF (its code is dead pending deletion). Only logging/diagnostic gates (and
// the HD image path) still read the real environment; those go away entirely with
// the probe STRIP pass. No STORAGER_* environment variable changes modeled
// hardware behavior any more.
char const *storager_getenv(char const *name)
{
	static char const *const hard_on[] = {
		"STORAGER_NOBYPASS", "STORAGER_BUSYHOLD", "STORAGER_STEPIRQ",
		"STORAGER_SEEKACTIVE", "STORAGER_STEPBIT0" };
	static char const *const passthrough[] = {
		"STORAGER_PHASELOG", "STORAGER_IAMRD", "STORAGER_UNITLOG", "STORAGER_UNITLOG2",
		"STORAGER_DMALOG", "STORAGER_WALKTRACE", "STORAGER_TRACE", "STORAGER_PCHIST",
		"STORAGER_PCHIST_WIDE", "STORAGER_PCHIST_AT", "STORAGER_PCHIST_N",
		"STORAGER_PCHIST_END", "STORAGER_IOPBDUMP",
		"STORAGER_CHAINTAP", "STORAGER_HD_IMAGE", "STORAGER_CPUAP_POLL",
		"STORAGER_LASTC0", "STORAGER_REARM", "STORAGER_FWDONE",
		"STORAGER_FMVFY", "STORAGER_C135PH", "STORAGER_SERVE", "STORAGER_STAKEV", "STORAGER_PERSEC", "STORAGER_AAFIX", "STORAGER_PLLVERIFY", "STORAGER_FILLMAP", "STORAGER_TR6", "STORAGER_DESCTRACE", "STORAGER_FAITHXFER", "STORAGER_PUMP836", "STORAGER_UNPARK", "STORAGER_LADWAIT", "STORAGER_REDISP", "STORAGER_OPTBIT4", "STORAGER_XFERDRAIN", "STORAGER_R7426", "STORAGER_IAM", "STORAGER_C0CENSUS", "STORAGER_ONEIRQ5", "STORAGER_SLOTMAP", "STORAGER_NOWALKSUBQ", "STORAGER_BULKRERUN", "STORAGER_SLOTGEO" };
	for (auto const *n : hard_on)
		if (!strcmp(name, n)) return "1";
	for (auto const *n : passthrough)
		if (!strcmp(name, n)) return getenv(name);
	return nullptr;
}

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
		, m_installed(false)
		, m_trace(false)
		, m_ch{}
		, m_dma_active(false)
		, m_c800{}
		, m_dma_term(0)
		, m_term_bit0(0)
		, m_last_bw(0)
		, m_d000(0)
		, m_d800(0)
		, m_c000(0)
		, m_c000_valid(false)
		, m_timer_out(false)
		, m_tick_prev(false)
		, m_gate0(false)
		, m_shadow_loaded(false)
		, m_iopb_fetched(false)
		, m_iopb_cmd(0)
		, m_iopb_buffer(0)
		, m_iopb_addr(0)
		, m_host_int_prev(false)
		, m_e800_bit12_prev(false)
		, m_e802_step(false)
		, m_seek_deadline(attotime::zero)
		, m_rdptr(0)
		, m_unit_heads{}
		, m_unit_spt{}
		, m_unit_secsize{}
		, m_unit_sec0{}
		, m_unit_trk{}
		, m_unit_sidx{}
		, m_unit_btrk{}
		, m_unit_bsidx{}
		, m_ioreg{}
		, m_ioreg_done(false)
		, m_floppy_loaded(false)
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
	void d000_w(offs_t offset, u16 data, u16 mem_mask);
	void d800_w(offs_t offset, u16 data, u16 mem_mask);
	u16 bus_mem_r(offs_t offset, u16 mem_mask);
	void bus_mem_w(offs_t offset, u16 data, u16 mem_mask);

	// LLE host interface: the Multibus PIO window 0x7200-0x73FF maps byte-for-byte to the Storager's
	// on-board dual-port RAM at 0x7E00-0x7FFF (+0xC00).  The CPUAP writes the command/IOPB-pointer
	// mailbox there; writing GO (0x13) to the command register (PIO 0x73F8 = fw 0x7FF8) is the
	// doorbell that raises 68000 IRQ2 (handler @0x24AA).  The firmware then runs the command itself.
	u16 host_win_r(offs_t offset);
	void host_win_w(offs_t offset, u16 data, u16 mem_mask);
	u16 bus_data_r(offs_t offset);   // Multibus I/O 0x0000 aperture (host reads kernel data via @0xF00000)

	// disk read/write channel (74LS1801/1802 + drives) at 0xE000-0xFFFF.  Logged for RE; backed by
	// RAM for now so the firmware runs.  E802 = IRQ ack (clears the doorbell IRQ2).
	u16 ch_r(offs_t offset, u16 mem_mask);
	void ch_w(offs_t offset, u16 data, u16 mem_mask);
	void read95_deliver(u32 dst);
	u16 ioreg_r(offs_t offset);
	void ioreg_w(offs_t offset, u16 data, u16 mem_mask);
	u16 c800_r(offs_t offset);
	void c800_w(offs_t offset, u16 data, u16 mem_mask);
	void c000_w(offs_t offset, u16 data, u16 mem_mask);
	void timer0_out(int state);
	void timer2_out(int state);
	void load_floppy();
	void floppy_swap_cb(floppy_image_device *fdd);   // UI media swap -> invalidate the cached sector map

	required_device<m68000_device> m_cpu;
	required_device_array<pit8253_device, 2> m_pit;
	required_device_array<floppy_connector, 2> m_floppy;
	required_device_array<storager_hd_image_device, 2> m_hd;   // ESDI unit 0 (-hard1) + unit 1 (-hard2); the second LUN is a second physical drive

	bool m_installed;
	bool m_trace;            // enable disk-channel logging once a host command arrives
	u16 m_ch[0x1000];        // 0xE000-0xFFFF backing store

	// --- gate-array DMA state machine (VGC7219, reverse-engineered) ---
	bool m_dma_active;       // DMA running (E800 bit6: 0xcd3 start / 0xc12 stop)
	u16  m_c800[0x100];      // C800 host-DMA register file, $c800-$c9ff (cont.41c: 0x100 words,
	                         // NOT 0x80 - the fw's host-page writes index (~hostaddr>>15)&$1fe,
	                         // e.g. $1e0 for host 0fc0dd; the old half-map dropped every $c9xx
	                         // write and the mailbox HLE-peek masked the loss)
	u16  m_dma_term;         // latched transfer terminal (low bits) for the address comparator
	u8   m_term_bit0;        // terminal LSB, latched from the helper's buffer read in DMA mode
	u32  m_last_bw;          // last local-bus buffer write address (snooped transfer pointer)
	u16  m_d000;             // D000 write-only DMA address latch (word address; reads = ROM)
	int  m_dma_mark = 0;     // faithful-vs-stale trace: which model DMA is writing (1=doorbell IOPB [$7a06], 2=kick IOPB m_d000<<1); 0=fw
	u16  m_d800;             // D800 write-only word-address latch = the channel parameter/status
	                         // template block ($7DAC>>1 on every traced write, 2026-07-17) - local,
	                         // NOT host-high (host addresses ride the C000 counter)
	u32  m_c000;             // C000-C7FF: the 24-bit Multibus HOST-ADDRESS up-counter.  One write
	                         // presets it with the one's-complemented host address - ~B[23:16] on
	                         // address lines A1-A8, ~B[15:0] on data (spec §3.4).  Stored as B.
	bool m_c000_valid;       // a preset has been latched since reset
	// cont.206 (STORAGER_R0STAT): the generated R0-STATUS register (2180/2190 family; MX300
	// manual: bit0=Idle, bit1=Busy, bit2/3=diag; 2180: "BUSY, OPER-DONE-INT, 4 MSB = drive
	// ready"). BUSY latches at the host GO (R0 write bit0) and clears when the FIRMWARE stamps
	// the host IOPB status through its bus-master window (the §5.1 write-through - the model
	// just observes it in bus_mem_w); OPER-DONE-INT sets on that stamp, clears on CLR-INT
	// (R0 write bit1). Ready bits 4-7 = units 0-3 (hd0, hd1, fd0, fd1).
	bool m_r0_busy = false;
	bool m_r0_doneint = false;
	// cont.209 (STORAGER_MBLATCH): the host register file's INBOUND latches (pio 73F4-73FB).
	// On real HW the host's register writes latch in the gate array separately from the fw's
	// outbound completion posts (same fw addresses, different latch banks). The model's shared
	// SRAM cells let a fw completion post {0F,E7,80} clobber a freshly-written command pointer
	// -> the doorbell auto-fetch read the OLD IOPB and deposited its +0x18/+0x19 bytes (98 E7)
	// into node+$18 = the long-arm "82/E7 INIT error". dbi readers must use these latches.
	u8 m_mb_in[8] = {};
	// cont.214: true once a per-sector truck served this command - the fw is driving delivery
	// itself; the window-mode (v6) host copy must stand down for this command.
	bool m_truck_seen = false;
	u8 m_rec_r0 = 0;
	int m_rec_quiet = 0;               // cont.241 (REC512): sectors left in the autonomous record window (no per-ID staging/raises)                   // cont.240 (REC512): record base sector, latched at launch, +4 per trucked record
	bool m_rec_open = false;           // cont.241b: record window opened by the fw's own match ([$742C] 0->1 write)
	u32 m_rec_base = 0;                // cont.241d: chunk base of the open record (latched at 742c open)
	u32 m_rec_c000 = 0;                // cont.241g: the record stream's OWN counter shadow - set by the data-descriptor consumption, advanced per record; doorbell C000 presets cannot clobber it
	u32 m_desc_lastpair = 0xffffffff;  // cont.241h: DESCGO dedupe latch - reset per doorbell command
	bool m_rec_first_valid = false;
	bool m_rec_convert = false;        // cont.242b: one-shot - the record's SECOND data-fork event (transfer-complete: $92B4 re-arm + $933C ledger C0) is owed
	bool m_c135_pending = false;       // cont.262 (STORAGER_C135PH): the carry's data-record IRQ6 fell in a [$7950]-phase-0 gap (would fork to $89f2 ID). Defer it; raise when the phase-1 window opens so the toggler takes the $92b4 DATA fork.
	u32 m_claim_field = ~0u;   // cont.222: id_end2 of the field whose fresh-slot claim raised the CARRY
	attotime m_claim_time;     // cont.222 v2: when - stream positions repeat per rev, the time disambiguates
	// cont.210 (STORAGER_MBSPLIT): true when host_win_w is depositing host register bytes into
	// fw SRAM - the mb[0] write tap must not treat the model's own deposit as a fw ack/post.
	bool m_mb_depositing = false;
	void r0_observe(u8 stv)
	{
		// DONE = bit7 set with bit0 clear (0x80 OK / 0x82 error); 0x81 = still busy (§5.1)
		if ((stv & 0x80) && !(stv & 0x01))
		{
			m_r0_busy = false;
			m_r0_doneint = true;
			if (storager_getenv("STORAGER_PHASELOG"))
				logerror("R0DONE stamp %02x @%.5f\n", stv, machine().time().as_double());
		}
	}
	bool m_timer_out;        // 8253 pit[1] counter-0 OUT -> F000 bit 11 (timer self-test)
	bool m_settle_out = true;   // 8253 pit[0] counter-1 OUT (mode-5 one-shot, STEP-triggered) = the
	                            // seek-settle timer; F000 bit1 (dumb-drive path) = !OUT (busy while low)
	bool m_tick_prev;        // previous pit[1] OUT2 level (system tick edge -> IRQ1)
	bool m_gate0;            // pit[1] ctr0 gate (E800 bit9): suppress IRQ1 on the handler's gate-toggle
	bool m_shadow_loaded;    // gate-array test has hard-copied the EPROM worker code into RAM
	bool m_iopb_fetched;     // gate array has autonomously bus-mastered the IOPB for this command
	u8   m_iopb_cmd;         // V/SMD 3200 command byte from the fetched IOPB (0x87 INIT, 0x95 READ-SEQ...)
	u32  m_iopb_buffer;      // host buffer addr = IOPB words 6-7 (BE32, low 24 bits): UIB or data target
	u32  m_iopb_addr;        // host IOPB base addr in CPUAP RAM (for the status write-back)
	bool m_host_int_prev;    // previous E802 bit7 (host completion interrupt level)
	bool m_e800_bit12_prev;  // previous E800 bit12 (kickoff is the rising edge, not the level)
	bool m_e802_step;        // last E802 bit6 (STEP); rising edge = one step pulse. VERIFIED: fw step
	                         // loop @0x24ae does andi #$ffbf/ori #$40 on the $79f8 E802 shadow (bit6),
	                         // writing each to ($6802,A5)=E802. (Was mis-documented as bit0.) DIR = a
	                         // $79f8 bit set once before the loop (candidates bit2/bit3, still to verify).
	attotime m_seek_deadline;  // F000 bit1 = seek/settle BUSY (time-windowed, per the drive datasheet: 75ms head
	                           // load, 40ms track-to-track + 10ms settle). Armed at SEEK/RESTORE and re-armed on
	                           // each STEP edge; bit1 stays 1 until this deadline, then 0 = settled.
	u32  m_rdptr;            // controller position latch from the SEEK/RESTORE doorbell fields (RESTORE->0, SEEK->target[6-7])
	std::map<u16, u8> m_depot_r;      // cont.256n: local SRAM addr -> the sector R deposited
	                                  // there, so the truck's host DMA can be verified against
	                                  // the sector the FIRMWARE aimed at (not the one that passed)
	// cont.257i (MEASURED, run318): the deposit and the COMPL5/collect that consume it carry
	// the SAME aim ([$7428]) - the mark-counting "one stage back" model (cont.257g/h) was an
	// artifact. The aim IS the sector identity and is stable across both events. So the honest
	// test for "may this completion be asserted" is simply: was a sector DEPOSITED for this
	// aim? Position 1 (aim=1) is never deposited because its field-start strobe preceded the
	// arm, so its COMPL5/collect must be held; every healthy position N has its aim=N deposit
	// before its aim=N completion. Reset per window (DESCARM); cannot be thrown off by mark
	// timing. Index 0 unused.
	bool m_aim_deposited[17] = {};    // cont.257i
	u64  m_read_hostmap = 0;          // cont.257t: bitmap of the read's logical blocks delivered
	std::vector<u8> m_win_buf;        // cont.281 FAITHXFER: the gate-array SERDES window buffer the fw's $748a transfer DMAs to host
	bool m_bulk_rearmed = false;      // cont.311: one-shot per read - [$7b10] re-armed once the window is full (convergence test)
	                                  // by the single position-keyed host-DMA path (bit n = the
	                                  // block at host+n*ssz). Reset per read window (DESCARM);
	                                  // when all total/ssz blocks are set, the read is complete.
	                                  // request/response, not a spinning stream. On each arm
	                                  // we deliver ONE sector's field (irq6,irq5,irq5) at fixed
	                                  // delays, then STOP and wait for the next arm. Data comes
	                                  // from the random-access deposit keyed to the fw's aim.
	                                  // record's address-mark framing differs (FM: bare FE at +0;
	                                  // MFM: A1 A1 A1 FE, mark at +3) and the fw tests BOTH layouts
	// THE ROTATIONAL CLOCK (build#5 cont.21, Dave's coherence principle): one source, all
	// consumers. pos(t) = absolute-time ticks at (stream_size x 5)/sec (300 rpm) mod stream_size
	// - the E000 byte served, the index, and the record-boundary interrupts are all positions on
	// the SAME rotating track, mutually consistent by construction. The marks are recorded in
	// build_serdes_stream's OWN emit pass (caution (a): one computation of the layout, never two).
	u16 m_last_warp_aim = 0;   // cont.224: debounce - the walker transiently rewrites [$7428]
	bool m_blk_half = false;   // cont.232: F000 bit6 = chunk mid-block (odd sector deposited,
	                           // the pair's second half pending) - the fw's $7F2C per-cycle gate
	// cont.258 (Dave: THE HEAD SWITCH): a multi-track read walks the fw's aim [$7428]
	// across the WHOLE transfer - aim 1..spt address the descriptor's BASE track, aim
	// spt+1..2*spt the next track (on a floppy: same cylinder, side 1), and so on. The
	// firmware writes [$7436] only as the STARTING head (measured run340: zero writes
	// during a read, always 0000); the gate array walks the sector count across the
	// track boundary itself - exactly as the delivery truck does (base_trk + n/spt). Map
	// an aim to the absolute {cyl, head, R} it addresses so the capture side (stream
	// rebuild, ID staging, deposit) follows onto track 1 instead of restaging track 0
	// forever. For aim 1..spt this yields base_trk unchanged (identical to the old
	// [$7436]=0 path), so track-0 behaviour is preserved.
	// cont.31 CONVERGENCE: stage the next (rotational) ID record at dst in the validated capture
	// format [A1 A1 A1][FE][FF] c h r n crc - the record $9884's parse wants ($990c checks the FF
	// at +4, $993e matches C vs [$7438]) before clearing the DATASTEP gate [[$72de]] at $9968.
	// cont.38: DATA-record capture completion - the record between the preceding id_end mark
	// and this data_end mark, framed as the ID capture is (sync + AM + gate-array valid byte
	// + payload + crc): A1 A1 A1 FB FF <data> 00 00 at the fw-programmed destination.
	// cont.257t: verify the delivered host window against the media, in the HLE's logical
	// order (read95_deliver: per track sidx 0..spt-1, secid=((sec0-1+sidx)%spt)+1, advancing
	// track on wrap). Called by the single position-keyed path at completion; reports the
	// first bad byte with its track/sector so any residual defect localises itself.
	void read_verify_window(u32 base, u32 ssz)
	{
		if (s_desc.verified) return;
		s_desc.verified = true;
		address_space &vbs = m_bus->space(AS_PROGRAM);
		u32 const vssz = ssz ? ssz : 128;
		u32 const vspt = s_desc.spt ? s_desc.spt : 16;
		u32 const nsec = s_desc.total / vssz;
		u32 bad = 0, firstoff = ~0u; u8 fexp = 0, fact = 0;
		u32 fcyl = 0, fhead = 0, fsec = 0, missing = 0;
		for (u32 n = 0; n < nsec; n++)
		{
			u32 const trk = s_desc.base_trk + n / vspt, sidx = n % vspt;
			u32 const cylno = trk >> 1, head = trk & 1;
			u32 const secid = ((u32(s_desc.sec0) - 1 + sidx) % vspt) + 1;
			auto const it = m_sectors.find((cylno << 16) | (head << 8) | secid);
			if (it == m_sectors.end()) { missing++; continue; }
			for (u32 k = 0; k < vssz; k++)
			{
				u8 const exp = (k < it->second.size()) ? it->second[k] : 0;
				u8 const act = vbs.read_byte((base + n * vssz + k) & 0xffffff);
				if (exp != act) { bad++; if (firstoff == ~0u) { firstoff = n * vssz + k; fexp = exp; fact = act; fcyl = cylno; fhead = head; fsec = secid; } }
			}
		}
		if (!bad && !missing)
			logerror("READVERIFY OK: %u bytes / %u sectors match the media exactly (base_trk=%u sec0=%u ssz=%u) @%.5f\n",
				s_desc.total, nsec, s_desc.base_trk, s_desc.sec0, vssz, machine().time().as_double());
		else
			logerror("READVERIFY MISMATCH: %u bad bytes (+%u sectors missing) of %u; FIRST at host+%u (cyl%u h%u R%u) exp=%02x act=%02x | base_trk=%u sec0=%u ssz=%u @%.5f\n",
				bad, missing, s_desc.total, firstoff, fcyl, fhead, fsec, fexp, fact,
				s_desc.base_trk, s_desc.sec0, vssz, machine().time().as_double());
	}
	bool m_serdes_active = false;     // E000 is MULTIPLEXED: only stream disk bytes while the ENDEC is armed
	                                  // for a read (else E000 returns channel status/echo). Set on the read
	                                  // arm, cleared on any other E000 write. (Ungated streaming corrupts
	                                  // the fw's scattered non-data E000 reads - verified false positive.)
	u8   m_unit_heads[4];    // per-unit geometry from the host's INIT/UIB: UIB[0] = heads
	u8   m_unit_spt[4];      // UIB[1] = sectors/track
	u16  m_unit_secsize[4];  // UIB[2-3] (LE) = bytes/sector (the host re-INITs to switch FM 128 <-> MFM 256)
	u8   m_unit_sec0[4];     // UIB[4] = STARTING SECTOR ID (logical sector 0's physical ID: FM=7 -> VOL1 first, MFM=13)
	u32  m_unit_trk[4];      // per-unit PHYSICAL track index (cyl*heads+head; RESTORE -> 0; advances as reads cross tracks)
	u32  m_unit_sidx[4];     // per-unit sector index within the current track (in current-config sectors)
	u32  m_unit_btrk[4];     // position at the last INIT = the base for ADDRESSED reads (IOPB opts bit0=1: the
	u32  m_unit_bsidx[4];    // addr field is in current-size sectors relative to this; the ANSI flow re-INITs at the file start)
	// ---- Phase B: the ENDEC/gate-array ID capture (firmware-driven read path) ----
	// Dev toggle (STRIP with the diagnostics): STORAGER_FWDRIVEN=1 disables the delivery/status
	// shims so the firmware must drive the disk channel itself (the Phase-B iteration loop).
	bool m_fw_driven = false;
	emu_timer *m_idcap = nullptr;      // rotational delay to the next ID field
	u32  m_idcap_dst = 0;              // latched local DMA destination (D800<<1) at capture start
	bool m_idcap_pending = false;      // cont.37: E802 bit15 HELD = capture armed; completes at the
	                                   // next id_end mark (cleared by a bit15 falling edge - the
	                                   // $88ac prime PULSE ($8200/$7fff) is too short to span a mark)
	bool m_seen_id = false;            // cont.39c: the ENDEC cannot recognize a data AM
	                                   // before it has SYNCED an IDAM - no data-boundary
	                                   // events until an id boundary has fired since the
	                                   // rotation/stream (re)start. (Run126-era chain: the
	                                   // model's first data event preceded any id capture,
	                                   // R=0 matched pre-init [$7428]=0, the $7e6c scan
	                                   // overran the terminator-less map -> $7430=$44 ->
	                                   // [$7428] poisoned -> every round rejects.)
	bool m_endec_data_mode = false;    // cont.38uu: the ENDEC's BOUNDARY SELECTOR - which sync
	                                   // it hunts: $22f = IDAM (id boundaries/irq6 only);
	                                   // $2af/$2ff = data-AM (data boundaries/irq5, AM + end).
	                                   // $23f/$a6d = window/operating controls, not selectors.
	bool m_want_ready = false;         // cont.150 (KEEPER): [$F000] bit4 - the gate array's
	                                   // want-ready/advance status level; latched at the
	                                   // segment carry, cleared at the command's DONE post.
	bool m_seg_retired = false;        // cont.127 v3 (KEEPER): the gate array's level-held
	                                   // per-segment DMA-done status for the [$7B0E] mailbox -
	                                   // latched at the segment carry, merged (bit15) into the
	                                   // fw's own $0080 lock-stamp at write time so the pop's
	                                   // $347e test sees it (a direct write gets clobbered by
	                                   // the stamp 20us before the test - run262).
	// cont.189 (STORAGER_ARMGATE): the delivery-side arm gate - cont.117's latch-at-arm class
	// extended per Dave's spec. Set at the fw's $88ac-chain E802 bit15 arm write (the fw's OWN
	// signal), cleared at each command doorbell. Mark delivery (IRQ5/6) honors the CURRENT
	// command's arm - a stale-era rotation mark cannot interrupt a pre-arm launch ladder.
	bool m_cmd_armed = false;
	// cont.190 (ARMGATE v2, per VGC7219-GATE-ARRAY-SPEC §3.1/§4): TYPE-SPECIFIC arms - the
	// E000 arm codes are the per-round window openers (0x22F arms IDAM->IRQ6; 0x2AF/0x2FF arm
	// DAM->IRQ5), written by the read ops themselves (op-driven, early - no mark-before-arm
	// circularity). Doorbell clears both; stale-era arms don't cross the command boundary.
	bool m_armed_idam = false;
	bool m_armed_dam = false;
	// cont.191 (ARMGATE v3): the per-command boundary signal = the fw's $a6d-class ENGAGEMENT
	// write (E000, bit11 unique - cont.39n: "the window opens on it"), op-driven at read-op
	// start, mark-independent. run318 (E802 bit15) and run319 (E000 arm codes) both proved
	// mark-circular or boot-only; the engagement is the fw's true per-op open.
	bool m_engaged = false;
	bool m_idcap_armed_id = true;      // cont.117 FIX: the capture's type LATCHED AT ARM. Run251
	                                   // caught the model retyping an IN-FLIGHT capture: the fw's
	                                   // per-sector choreography arms BARE (data-typed, $891a) with
	                                   // [$742c] set, then primes $22f + re-arms ($89b6) 44us later -
	                                   // and the live-read global flag retyped the flying capture to
	                                   // ID before its data mark (~1ms out) could complete it. The
	                                   // real gate array latches type when the capture arms; the
	                                   // prime programs the NEXT arm. Latched on a FRESH arm only
	                                   // (re-arm dips while pending keep the armed type).
	bool m_idcap_id_typed = true;      // cont.38d: the capture TYPE selector - an arm preceded by
	                                   // the E000 $22f prime ($88ac/$9602: sync-to-IDAM window)
	                                   // captures the next ID record; a bare E802 re-arm with no
	                                   // prime ($92b4's $8a00) captures the next DATA record.
	                                   // Run75: untyped completions fed the hunt a data record
	                                   // 0.9ms before each id, wrecking the accept bookkeeping.
	// the sector currently passing under the head, from true angular position (300 RPM)
	unsigned angular_sector(unsigned nsecs) const
	{
		u64 const rev_ticks = attotime::from_msec(200).as_ticks(1'000'000);
		u64 const now = machine().time().as_ticks(1'000'000) % rev_ticks;
		return unsigned(now * nsecs / rev_ticks) % nsecs;
	}
	// per-(cyl,side) sector cache decoded from the MOUNTED image's bitstream (behind the disk-
	// channel model only - never visible to the host path)
	struct dc_sector { u8 c, h, r, n; std::vector<u8> data; };
	struct dc_track { bool fm = false; std::vector<dc_sector> secs; };
	std::map<u32, dc_track> m_dc_ids;
	dc_track const &decode_track_ids(floppy_image_device *fdd, int cyl, int side);
	emu_timer *m_dataop = nullptr;     // the armed read-channel operation (rotational delay)
	// cont.263: THE 74LS1811 PLL DATA SEPARATOR (raw-bitstream pivot). Recovers the clock+data
	// cell stream from the real floppy flux (floppy->get_next_transition), replacing the synthetic
	// build_serdes_stream. Period set per density: 4us/cell 5.25" FM, 2us/cell 5.25" MFM.
	fdc_pll_t m_pll;
	// Read the whole track through the PLL and recover its ID fields (FM IDAM 0xf57e raw / MFM
	// A1 0x4489 sync), verifying the separator against the known geometry before it drives E000.
	// Returns the number of ID fields recovered; logs each C/H/R/N. This is the proof the flux +
	// PLL path is byte-faithful (Dave's correctness oracle for the pivot).
	unsigned pll_verify_track(floppy_image_device *fdd, bool fm)
	{
		if (!fdd || !fdd->exists()) return 0;
		u32 const cell_ns = fm ? 4000 : 2000;   // 5.25" 250kbps: FM 4us/cell, MFM 2us/cell
		m_pll.set_clock(attotime::from_nsec(cell_ns));
		attotime const start = machine().time();
		m_pll.read_reset(start);
		attotime const limit = start + attotime::from_msec(210);   // ~one 300RPM revolution + slack
		attotime tm;
		u32 shift = 0;         // raw cell shift register (clock+data interleaved, MSB = oldest)
		// FM raw AMs (data/clock $C7 interleaved): IDAM $FE=0xf57e, DAM $FB=0xf56f. MFM: A1 sync
		// (missing clock) = 0x4489; the byte after the A1 run is the mark ($FE IDAM / $FB,$F8 DAM).
		enum { HUNT, READ_MARK, READ_ID, READ_DATA } state = HUNT;
		int cells = 0, nb = 0, want = 0;
		u8 buf[300]; u8 cur_r = 0; u8 cur_n = 0;
		unsigned found = 0;
		// data byte from the low 16 cells: data bits are the EVEN positions (each bit = [clock][data]).
		auto data_byte = [&]() -> u8
		{ u8 b = 0; for (int k = 7; k >= 0; k--) b = u8((b << 1) | ((shift >> (2 * k)) & 1)); return b; };
		for (int guard = 0; guard < 800000; guard++)
		{
			int const bit = m_pll.get_next_bit(tm, fdd, limit);
			if (bit < 0) break;
			shift = (shift << 1) | unsigned(bit);
			if (state == HUNT)
			{
				if (fm && (shift & 0xffff) == 0xf57e) { state = READ_ID;   cells = 0; nb = 0; want = 4; }
				else if (fm && (shift & 0xffff) == 0xf56f) { state = READ_DATA; cells = 0; nb = 0; want = 128 << cur_n; }
				else if (!fm && (shift & 0xffff) == 0x4489) { state = READ_MARK; cells = 0; }
				continue;
			}
			if (++cells % 16) continue;    // wait for a full byte of cells
			u8 const b = data_byte();
			if (state == READ_MARK)        // MFM: skip further A1s, dispatch on the mark byte
			{
				if (b == 0xa1) continue;
				if (b == 0xfe) { state = READ_ID;   nb = 0; want = 4; continue; }
				if (b == 0xfb || b == 0xf8) { state = READ_DATA; nb = 0; want = 128 << cur_n; continue; }
				state = HUNT; continue;
			}
			buf[nb++] = b;
			if (nb < want) continue;
			if (state == READ_ID)
			{
				cur_r = buf[2]; cur_n = buf[3] & 7;
				if (storager_getenv("STORAGER_PLLVERIFY"))
					logerror("PLL-ID  #%u c=%02x h=%02x r=%02x n=%02x @%.6f\n",
						found, buf[0], buf[1], buf[2], buf[3], machine().time().as_double());
				state = HUNT;
			}
			else   // READ_DATA
			{
				if (storager_getenv("STORAGER_PLLVERIFY"))
					logerror("PLL-DATA r=%02x first8=%02x %02x %02x %02x %02x %02x %02x %02x\n",
						cur_r, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
				found++; state = HUNT;
			}
		}
		return found;
	}

	// cont.263: THE 74LS1812 SERDES LIVE-RUN (raw-bitstream pivot). A real-time byte/AM engine over
	// the PLL, driven by the wd_fdc "advance-to-now on access" idiom: flux_advance_to() steps the PLL
	// from m_flux_tm up to a target time, recovering the byte under the head into m_flux_byte and
	// running the FM/MFM address-mark state machine. E000 reads return m_flux_byte; IDAM/DAM detects
	// raise the firmware's IRQ marks. Replaces the synthetic build_serdes_stream + pump.
	attotime m_flux_tm;                // current PLL time (the byte-clock cursor)
	u32 m_flux_shift = 0;              // raw cell shift register
	int m_flux_state = 0;              // 0 HUNT, 1 READ_MARK, 2 READ_ID, 3 READ_DATA
	int m_flux_cells = 0, m_flux_nb = 0, m_flux_want = 0;
	u8  m_flux_buf[300] = {}, m_flux_r = 0, m_flux_n = 0, m_flux_byte = 0;
	bool m_flux_fm = true;
	bool m_flux_test = false;   // test mode: recover+log, don't raise CPU IRQs
	u32  m_flux_track = ~0u;    // (cyl<<1)|side the live-run is currently locked to
	floppy_image_device *m_flux_fdd = nullptr;
	attotime m_flux_next_index; // next spindle index-pulse time (IRQ3 = the $55d8 index handler)
	bool m_flux_skip = false;   // this sector is past the read window ([$7956]==0 at its ID) - suppress
	                            // its record marks so the fw does not over-scan/over-enqueue [$74ac]
	void flux_read_reset(floppy_image_device *fdd, bool fm)
	{
		m_flux_fdd = fdd; m_flux_fm = fm;
		m_pll.set_clock(attotime::from_nsec(fm ? 4000 : 2000));
		m_flux_tm = machine().time();
		m_pll.read_reset(m_flux_tm);
		m_flux_shift = 0; m_flux_state = 0; m_flux_cells = 0; m_flux_nb = 0; m_flux_want = 0; m_flux_byte = 0;
		m_flux_next_index = fdd->time_next_index();
	}
	// data address mark just passed (DAM detected, data field starting): the FM data path is a
	// two-IRQ5 contract (cont.262) - this FIRST IRQ5 is the data-AM ($29c0 -> $7ba8 setup); the
	// second, at data-record end below, is the sector-done ($7fee). Without the setup leg the
	// firmware never completes the sector (ledger orphans at f0).
	void flux_data_am()
	{
		if (storager_getenv("STORAGER_C0CENSUS")) { static int c=0; double t=machine().time().as_double();
			if (t>=7.9 && c++<40) logerror("C0CEN DAM(1stIRQ5) R=%02x skip=%d @%.6f\n", m_flux_r, m_flux_skip?1:0, t); }
		if (!m_flux_test && !m_flux_skip) m_cpu->set_input_line(M68K_IRQ_5, HOLD_LINE);
	}
	// step the PLL up to `when`, recovering bytes into m_flux_byte and firing marks at each AM.
	void flux_advance_to(const attotime &when)
	{
		if (!m_flux_fdd) return;
		attotime tm = m_flux_tm;
		for (int guard = 0; guard < 20000; guard++)
		{
			int const bit = m_pll.get_next_bit(tm, m_flux_fdd, when);
			if (bit < 0) break;                 // reached `when`
			m_flux_tm = tm;
			// cont.289 (faithful boundary - replaces the rejected $79ae/fabricated-$fe steer):
			// the Index Address Mark. When the head crosses the physical index (once/rev, already
			// tracked as m_flux_next_index) during an armed read, the SERDES decodes the IAM and
			// DMAs its real field {A1 A1 A1 FE FF} to the fw-published capture cells, raising IRQ6 -
			// the SAME pointer-directed capture path as every IDAM. POSITION=$FE is the gate array's
			// index-gap sentinel (normal sectors are 1..SPT). The fw's own residual math has already
			// armed [$79ae]; its walk reads captured [$7daf]==$FE, mismatches HEAD, routes $7c70->$7cac
			// (sets [$7426]=1 itself), and completes. Real captured mark - no control-flag poke.
			if (!m_flux_next_index.is_never() && m_flux_tm >= m_flux_next_index)
			{
				if (storager_getenv("STORAGER_IAM") && !m_flux_test
						&& (m_iopb_cmd == 0x95 || m_iopb_cmd == 0x94) && m_serdes_active)
				{
					address_space &cs = m_cpu->space(AS_PROGRAM);
					cs.write_byte(0x7dac, 0xa1);   // A1 sync (the $8a14 OR-fold; also the $7c7a alt test)
					cs.write_byte(0x7dad, 0xa1);   // A1
					cs.write_byte(0x7dae, 0xa1);   // A1  -> HEAD mismatch vs [$7436], routes $7c46 -> $7c70
					cs.write_byte(0x7daf, 0xfe);   // POSITION = $FE = the index-gap sentinel ($7c70 boundary)
					cs.write_byte(0x7db0, 0xff);   // N = $FF
					m_cpu->set_input_line(M68K_IRQ_6, HOLD_LINE);
					if (storager_getenv("STORAGER_PHASELOG"))
						logerror("IAM index mark presented (A1 A1 A1 FE FF) trk=%u @%.6f\n", m_flux_track, machine().time().as_double());
				}
				if (m_flux_fdd) m_flux_next_index = m_flux_fdd->time_next_index();   // schedule next rev's index
			}
			m_flux_shift = (m_flux_shift << 1) | unsigned(bit);
			if (m_flux_state == 0)   // HUNT
			{
				if (m_flux_fm && (m_flux_shift & 0xffff) == 0xf57e) { m_flux_state = 2; m_flux_cells = 0; m_flux_nb = 0; m_flux_want = 4; }
				else if (m_flux_fm && (m_flux_shift & 0xffff) == 0xf56f) { m_flux_state = 3; m_flux_cells = 0; m_flux_nb = 0; m_flux_want = 128 << m_flux_n; flux_data_am(); }
				else if (!m_flux_fm && (m_flux_shift & 0xffff) == 0x4489) { m_flux_state = 1; m_flux_cells = 0; }
				continue;
			}
			if (++m_flux_cells % 16) continue;
			u8 b = 0; for (int k = 7; k >= 0; k--) b = u8((b << 1) | ((m_flux_shift >> (2 * k)) & 1));
			m_flux_byte = b;                    // the byte under the head, for E000
			if (m_flux_state == 1)   // MFM READ_MARK
			{
				if (b == 0xa1) continue;
				if (b == 0xfe) { m_flux_state = 2; m_flux_nb = 0; m_flux_want = 4; continue; }
				if (b == 0xfb || b == 0xf8) { m_flux_state = 3; m_flux_nb = 0; m_flux_want = 128 << m_flux_n; flux_data_am(); continue; }
				m_flux_state = 0; continue;
			}
			m_flux_buf[m_flux_nb++] = b;
			if (m_flux_nb < m_flux_want) continue;
			if (m_flux_state == 2)   // ID complete: stage the captured field + raise IRQ6 (ID mark)
			{
				m_flux_r = m_flux_buf[2]; m_flux_n = m_flux_buf[3] & 7;
				// Per-sector over-scan guard (cont.268/269): suppress this sector's record marks when the
				// fw's read window no longer wants it, so it stops completing/re-enqueuing [$74ac] and the
				// $32ac drain can reach [$74ac]==0 -> exit ($741c=0) -> $1a54 0x80. Skip when: (a) the
				// countdown [$7956] is satisfied (0); (b) the sector is outside the N-block window; or
				// (c) the sector's ledger block is ALREADY c0 (captured) - do not re-complete it every
				// revolution. Latched at the ID so both IRQ5s (the $7ba8<->$7fee ping-pong) skip together.
				m_flux_skip = false;
				if (m_iopb_cmd == 0x95 || m_iopb_cmd == 0x94)
				{
						// cont.278 (Dave): do NOT gate on [$7956]==0 or ledger-c0 - the completion ($82b2 /
						// $7bf0 $99-path) fires only when a mark ARRIVES while [$7956]==0; suppressing it
						// there is what stalled at testend. Keep only out-of-window; AAFIX protects $aa.
						if (s_desc.host && s_desc.ssz)
					{
						u32 const spt = s_desc.spt ? s_desc.spt : 16;
						u32 const sec0 = s_desc.sec0 ? s_desc.sec0 : 7;
						u32 const nblk = s_desc.total / s_desc.ssz;
						u32 const ntrk = (m_flux_track >= s_desc.base_trk) ? (m_flux_track - s_desc.base_trk) : 0;
						u32 const n = ntrk * spt + ((u32(m_flux_r) + spt - sec0) % spt);
						if (n >= nblk) m_flux_skip = true;                                            // out of window
					}
				}
				if (!m_flux_test && !m_flux_skip)   // "detection is capture": the field is in the capture cells at the IRQ
				{
					address_space &cs = m_cpu->space(AS_PROGRAM);
					cs.write_byte(0x7dac, 0xfe);          // FM/normalized IDAM
						// cont.277 (Dave): the POSITION cell [$7daf] must be the position-space ORDINAL (matches
						// the aim [$7428]), not the raw sector R - else $7e58 never matches and the aim never steps.
						u32 const _spt = s_desc.spt ? s_desc.spt : 16;
						u32 const _sec0 = s_desc.sec0 ? s_desc.sec0 : 7;
						u8  const _pos = u8(((u32(m_flux_buf[2]) + _spt - _sec0) % _spt) + 1);
					cs.write_byte(0x7dad, m_flux_buf[0]); // C
					cs.write_byte(0x7dae, m_flux_buf[1]); // H
						cs.write_byte(0x7daf, storager_getenv("STORAGER_FILLMAP") ? _pos : m_flux_buf[2]); // POSITION (ordinal under FILLMAP) / raw R
					cs.write_byte(0x7db0, m_flux_buf[3]); // N
					m_cpu->set_input_line(M68K_IRQ_6, HOLD_LINE);
					if (storager_getenv("STORAGER_PHASELOG")) { static int _i=0; if(_i++<120)
						logerror("CAD-ID   R=%02x aim=%04x ledger[aim]=%02x @%.6f\n", m_flux_r,
							cs.read_word(0x7428), cs.read_byte((0x7654 + cs.read_word(0x7428)) & 0xffff), machine().time().as_double()); }
					// cont.277 (Dave): per-IRQ6, capture the two decisive gate operands - $7e58 (cmp
					// [$7428] aim vs [$7daf] cursor) and $7bea (cmpi #$99,[[$71bc]] = TRUE completion).
					if (storager_getenv("STORAGER_TR6"))
					{
						u32 const spt = s_desc.spt ? s_desc.spt : 16;
						u32 const sec0 = s_desc.sec0 ? s_desc.sec0 : 7;
						u32 const ord = ((u32(m_flux_r) + spt - sec0) % spt) + 1;
						u16 const rtp = cs.read_word(0x71bc);
						static int _t6 = 0; if (_t6++ < 120)
							logerror("TR6 R=%02x ord=%u aim=%04x rt=%02x 7956=%04x | 79ba=%04x 7958=%08x 741c=%04x 796a=%04x 727e=%04x 7968=%04x @%.6f\n",
								m_flux_r, ord, cs.read_word(0x7428), cs.read_byte(rtp & 0xffff), cs.read_word(0x7956),
								cs.read_word(0x79ba), cs.read_dword(0x7958), cs.read_word(0x741c), cs.read_word(0x796a),
								cs.read_word(0x727e), cs.read_word(0x7968), machine().time().as_double());
					}
				}
			}
			else   // DATA complete: deliver the recovered field to the host, then raise IRQ5
			{
				if (storager_getenv("STORAGER_C0CENSUS")) { static int c=0; double t=machine().time().as_double();
					if (t>=7.9 && c++<40) logerror("C0CEN DATADONE R=%02x nb=%u want=%u skip=%d map=%04x @%.6f\n",
						m_flux_r, m_flux_nb, m_flux_want, m_flux_skip?1:0, unsigned(m_read_hostmap), t); }
				if (!m_flux_test)
				{
					// "detection is capture" for the DATA field: the just-recovered sector is in
					// m_flux_buf. Deliver it to the host at its logical position in the read window
					// (the 74LS1812 SERDES -> Multibus DMA the firmware armed), and fire the
					// transfer-complete IRQ4 once every block of the window has landed. One
					// revolution delivers the whole window - the timely completion the fw waits on.
					if (s_desc.active && (m_iopb_cmd == 0x95 || m_iopb_cmd == 0x94) && s_desc.host && s_desc.ssz)
					{
						u32 const ssz = s_desc.ssz;
						u32 const spt = s_desc.spt ? s_desc.spt : 16;
						u32 const sec0 = s_desc.sec0 ? s_desc.sec0 : 7;
						u32 const nblk = ssz ? (s_desc.total / ssz) : 0;
						u32 const ntrk = (m_flux_track >= s_desc.base_trk) ? (m_flux_track - s_desc.base_trk) : 0;
						u32 const n = ntrk * spt + ((u32(m_flux_r) + spt - sec0) % spt);   // logical block of this sector
							if (n < nblk && n < 64 && m_flux_want >= ssz && !BIT(m_read_hostmap, n))
							{
								// cont.281 (FAITHXFER, Dave): capture stops at the SERDES window buffer - do NOT deliver to
								// the host and do NOT fire a bare DESCDONE here (both pre-empt the fw's own $748a transfer
								// and derail its build -> [$7a14]=0). The E800 bit12 kick (Piece 2) does the buffer->host
								// DMA and raises the identity-gated IRQ4. Baseline: unchanged immediate flux->host.
								if (storager_getenv("STORAGER_FAITHXFER"))
								{
									if (m_win_buf.size() < std::size_t(nblk) * ssz) m_win_buf.resize(std::size_t(nblk) * ssz, 0);
									for (u32 k = 0; k < ssz; k++) m_win_buf[std::size_t(n) * ssz + k] = m_flux_buf[k];
								}
								else
								{
									address_space &hbs = m_bus->space(AS_PROGRAM);
									for (u32 k = 0; k < ssz; k++)
										hbs.write_byte((s_desc.host + n * ssz + k) & 0xffffff, m_flux_buf[k]);
								}
								m_read_hostmap |= (u64(1) << n);
								m_ch_op_ok = true;
								// cont.276 (Dave): the FILL-MAP mark - the sector's slot index (the actual SECTOR NUMBER)
								// as a POSITIVE ledger byte at its logical position, so the fw's $6f44 queues slot $74c4+R*8.
								if (storager_getenv("STORAGER_FILLMAP"))
									m_cpu->space(AS_PROGRAM).write_byte((0x7654 + n + 1) & 0xffff, u8(m_flux_r & 0x7f));
								u64 const full = (nblk >= 64) ? ~u64(0) : ((u64(1) << nblk) - 1);
								// cont.311 (Dave's convergence test): the window is captured (SLOTMAP populated the
								// ledger). Re-arm the fw's $6f44 re-run trigger [$7b10] ONCE so the walk re-runs $6f44
								// against the FULL ledger -> $70a0 D3=8 -> [$7956]==0 -> $70a6 sets [$7a64] -> the
								// DESCGO/$3dbc completion ($4102 gates on [$7a64]!=0). Tests whether [$7a64] is the
								// single root reachable via a full-ledger $6f44 pass (the bulk arm, no stake).
								if (storager_getenv("STORAGER_BULKRERUN") && (m_read_hostmap & full) == full && !m_bulk_rearmed)
								{
									m_cpu->space(AS_PROGRAM).write_word(0x7b10, 0xffff);
									m_bulk_rearmed = true;
									if (storager_getenv("STORAGER_PHASELOG"))
										logerror("BULKRERUN [$7b10] re-armed, ledger full @%.6f\n", machine().time().as_double());
								}
								if ((m_read_hostmap & full) == full && !storager_getenv("STORAGER_FAITHXFER"))
								{   // baseline only: the synthetic window-done DESCDONE. Under FAITHXFER the fw's own arm+kick
								    // drives IRQ4 per sector, so we must NOT fire this bare edge here.
									s_desc.active = false;
									m_read_pending = false;
									m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE);
									if (storager_getenv("STORAGER_PHASELOG"))
										logerror("FLUX-DESCDONE %u blocks landed at host %06x @%.6f\n", nblk, s_desc.host, machine().time().as_double());
									if (storager_getenv("STORAGER_TR6"))
									{
										address_space &ls = m_cpu->space(AS_PROGRAM);
										u16 const node743a = ls.read_word(0x743a);
										u16 const rt = ls.read_word(0x71bc);
										logerror("DESCDONE-IRQ4 [743a]=%04x +14=%08x | [[71bc]]=%02x +26=%02x | 7424=%04x 7426=%04x @%.6f\n",
											node743a, ls.read_dword((node743a + 0x14) & 0xffff), ls.read_byte(rt & 0xffff),
											ls.read_byte((rt + 0x26) & 0xffff), ls.read_word(0x7424), ls.read_word(0x7426),
											machine().time().as_double());
									}
								}
							}
					}
					// cont.299 (Dave's SLOTMAP): the gate array's "data landed in slot N" event (cont.275),
					// done right. At the data record-end the SERDES has recovered this sector; report the
					// slot index into the ledger at the AIM the stake used, overwriting the fw's f0. $6f44
					// then takes the POSITIVE branch ($6fde bge $6fe8) and converts it to c0 ($6ff0); the aim
					// advances (two-event contract). FILLMAP fixed: writes at [$7428] (the aim), NOT the
					// geometric block index n+1 - the pos mismatch that made cont.276 collide with the scan.
					// Staged: flux->host delivery kept for now; the faithful slot-DMA replaces it only once
					// this encoding path is proven to convert + advance.
					if (storager_getenv("STORAGER_SLOTMAP") && !m_flux_test
							&& (m_iopb_cmd == 0x95 || m_iopb_cmd == 0x94) && s_desc.active)
					{
						address_space &cs = m_cpu->space(AS_PROGRAM);
						if (storager_getenv("STORAGER_SLOTGEO"))
						{   // cont.312: GEOMETRIC - write every window position ledger[1+n]=slot# (not aim),
							// so the BULK $6f44 sees the FULL 8-sector window (D3=8) -> $70a0 lands [$7956]==0
							// -> $70a6 sets [$7a64] -> $4102 launch / $3dbc unpark. The aim-write (Detail 1) is
							// for the per-sector scan; the bulk convert wants the whole window present at once.
							u32 const spt = s_desc.spt ? s_desc.spt : 16;
							u32 const sec0 = s_desc.sec0 ? s_desc.sec0 : 7;
							u32 const nblk = s_desc.ssz ? (s_desc.total / s_desc.ssz) : 8;
							u32 const ntrk = (m_flux_track >= s_desc.base_trk) ? (m_flux_track - s_desc.base_trk) : 0;
							u32 const gn = ntrk * spt + ((u32(m_flux_r) + spt - sec0) % spt);
							if (gn < nblk && gn < 63)
								cs.write_byte((0x7654 + 1 + gn) & 0xffff, u8(m_flux_r & 0x7f));
						}
						else
						{
							u16 const aim = cs.read_word(0x7428);              // Detail 1: pos = the aim, not geometric
							if (aim >= 1 && aim < 64)
								cs.write_byte((0x7654 + aim) & 0xffff, u8(m_flux_r & 0x7f));  // Detail 2: slot# = R
						}
					}
					// cont.294 (Dave's parity A/B): the SECOND data-IRQ5 (cont.266 data-done). The $7950
					// alternator toggles on each mark; 2 IRQ5/sector = EVEN parity -> every ID-IRQ6 lands
					// old-bit 0 -> $89f2 no-stake, $92b4 never fires. STORAGER_ONEIRQ5 drops back to ONE
					// IRQ5/sector (odd parity) to test whether the next ID-IRQ6 then stakes.
					if (!m_flux_skip && !storager_getenv("STORAGER_ONEIRQ5")) m_cpu->set_input_line(M68K_IRQ_5, HOLD_LINE);
				}
				if (storager_getenv("STORAGER_PHASELOG")) { static int _d=0; if(_d++<120) {
					address_space &cs = m_cpu->space(AS_PROGRAM);
					logerror("CAD-DATA R=%02x aim=%04x host=%06x map=%04x buf=%02x%02x%02x%02x%02x%02x%02x%02x @%.6f\n",
						m_flux_r, cs.read_word(0x7428), s_desc.host, unsigned(m_read_hostmap),
						m_flux_buf[0],m_flux_buf[1],m_flux_buf[2],m_flux_buf[3],m_flux_buf[4],m_flux_buf[5],m_flux_buf[6],m_flux_buf[7],
						machine().time().as_double()); } }
			}
			m_flux_state = 0;
		}
	}
	emu_timer *m_pump = nullptr;       // VGC7219 gate-array pump: raises IRQ6 to run the queued read
	// The rotational event tick (build#5 cont.21): fires at each recorded mark position while the
	// SERDES window is armed - the gate array's THREE interrupt levels off the one rotating track
	// (the decoded contract: ROM vectors IRQ3->[[$72f8]] index/$55d8, IRQ5->[[$7300]] data-record
	// togglers $29ce/dc/ea, IRQ6->[[$7304]] ID-record toggler $299a - all installed by the fw's
	// own $3a30 builder). The count-expiry/free-run pumps are RETIRED.
	TIMER_CALLBACK_MEMBER(pump_tick)
	{
		// The 74LS1812 SERDES mark clock: advance the live-run to now (firing the ID/DATA
		// marks at their flux times against the live spindle angle) and reschedule.
		if (m_flux_fdd && m_serdes_active)
			flux_advance_to(machine().time());
		if (storager_getenv("STORAGER_PHASELOG"))   // TEMP: reliable ledger-change watch ($7654[0..15])
		{
			static u8 shad[16] = {0}; static bool init = false; static int _ln = 0;
			address_space &ls = m_cpu->space(AS_PROGRAM);
			u8 cur[16]; bool chg = false;
			for (int k = 0; k < 16; k++) { cur[k] = ls.read_byte((0x7654 + k) & 0xffff); if (cur[k] != shad[k]) chg = true; }
			if ((chg || !init) && _ln++ < 200)
			{
				std::string s; for (int k = 0; k < 16; k++) s += util::string_format(" %02x", cur[k]);
				logerror("LEDGER%s aim=%04x 742c=%04x pc=%06x @%.6f\n", s.c_str(),
					ls.read_word(0x7428), ls.read_word(0x742c), m_cpu->pc(), machine().time().as_double());
			}
			for (int k = 0; k < 16; k++) shad[k] = cur[k]; init = true;
		}
		m_pump->adjust(attotime::from_usec(200));
	}
	// task#4: the CHANCOMPLETE IRQ4 (ch_w L2532) is deferred through this timer under NOBYPASS.  Firing it
	// synchronously at the fw's E800 channel-kick preempts the main thread AT THE NEXT INSTRUCTION, so its
	// channel-program stamps the shared CCB [$71f0] BEFORE the main thread dispatches the staged 0x95 at $d06
	// (measured: clobber ipl=4, dispatch ipl=0).  The real gate array raises channel-complete us after the kick.
	// Deferring lets the dispatch consume 0x95 first; the IRQ4 then stamps an already-consumed CCB (fw-intended
	// overlap).  A counter honors every kick (each -> one IRQ4), spaced so none are lost to timer re-arm.
	emu_timer *m_chancomplete = nullptr;
	int m_chan_pending = 0;
	TIMER_CALLBACK_MEMBER(chancomplete_tick)
	{
		m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE);
		if (--m_chan_pending > 0) m_chancomplete->adjust(attotime::from_usec(30));
	}
	// Deferred gate-array CHANNEL completion (Dave #2): the completion belongs to the queued channel
	// descriptor node the fw armed, NOT the transient m_iopb_cmd host shadow. Latched at the DISKOP kick,
	// asserted asynchronously a few us later - but only if the armed descriptor still looks like a
	// pending channel op at fire time ($743a == latched node, node+0x12/$749c still 1, node+0x1a -> 0x7208).
	emu_timer *m_hd_chan = nullptr;
	bool m_hd_chan_pending = false;   // a latched HD verify node is awaiting its channel completion
	bool m_hd_chan_irq4 = false;      // the level-held IRQ4 source is currently asserted
	u16  m_hd_chan_node = 0;
	unsigned m_hd_chan_tries = 0;
	TIMER_CALLBACK_MEMBER(hd_chan_tick)
	{
		if (!m_hd_chan_pending || m_hd_chan_irq4) return;
		address_space &xs = m_cpu->space(AS_PROGRAM);
		u16 const node = m_hd_chan_node;
		bool const pending = node != 0 && xs.read_word((node + 0x12) & 0xffff) == 1;
		bool const valid = pending && xs.read_word(0x743a) == node
			&& xs.read_word((node + 0x1a) & 0xffff) == 0x7208;
		if (valid)
		{
			// gate-array channel completion: assert IRQ4 as a HELD LEVEL. It stays asserted (taken the
			// instant the fw's interrupt level drops) until the ISR acks the source by clearing E800
			// bit12 (0x3c02: andi #efff,$79f6 ; write $79f6->E800), handled in ch_w.
			m_hd_chan_irq4 = true;
			m_ch_op_ok = true;
			u16 const sr = u16(m_cpu->state_int(M68K_SR));
			logerror("HDCHAN assert node=%04x IRQ4 level SR=%04x iplmask=%u pc=%06x @%.4f\n", node, sr, (sr >> 8) & 7, m_cpu->pc(), machine().time().as_double());
			if (storager_getenv("STORAGER_NOBYPASS")) logerror("IRQ4fire L%d(hd_chan) pc=%06x @%.4f\n", __LINE__, m_cpu->pc(), machine().time().as_double());
			m_cpu->set_input_line(M68K_IRQ_4, ASSERT_LINE);
		}
		else if (pending && ++m_hd_chan_tries < 200)
			m_hd_chan->adjust(attotime::from_usec(20));   // re-poll until the descriptor is armed
		else
			m_hd_chan_pending = false;   // node resolved elsewhere or gave up
	}
	bool m_bare_irq4 = false;          // TEMP probe (STRIP): dataop fires IRQ4 only, no data/E01E
	bool m_seek_fired = true;          // task#4 per-op completion latch: true = no seek pending; set false when a
	                                   // seek/restore (0x89/0x98) is armed, back to true when its completion posts
	u32  m_seek_iopb = 0;              // task#4: the ARMED seek/restore's own host IOPB, latched at its doorbell -
	                                   // post its completion HERE, not to m_iopb_addr (which advances to the next op)
	// task#5 (STRIP): prefetch-immune PC-sampling histogram, bracketed to ONE read's window (Dave). Samples the
	// real PC register (post-fetch execution-truth) on a fine timer, so the $28e0 prefetch ghost cannot recur.
	// Triggered at the first cmd=0x95 Xdisp after t=7.0 (the last read, after which the fw goes quiescent); runs
	// a fixed 100ms window; dumps the top PC bins = the terminal loop declares itself, no milestone label.
	emu_timer *m_pcsamp = nullptr;
	std::map<u32, u32> m_pchist;
	bool m_pcsamp_on = false;
	bool m_pcsamp_done = false;
	u32 m_pcsamp_n = 0;
	u16 m_pcsamp_749c_or = 0;   // OR of [$749c] across the window (0 => channel idle the whole time)
	TIMER_CALLBACK_MEMBER(pcsamp_tick)
	{
		if (!m_pcsamp_on) return;
		m_pchist[u32(m_cpu->state_int(M68K_PC)) & 0xffffff]++;
		m_pcsamp_749c_or |= m_cpu->space(AS_PROGRAM).read_word(0x749c);   // was the channel busy at any sample?
		// WIDE mode (STORAGER_PCHIST_WIDE): map ALL pipeline stages in one pass - sample at 4us
		// continuously until PCHIST_END (default 29.5s); the INLINE-36de tap dumps + resets the
		// histogram at each stage boundary, so each window = one stage's poll loop, labeled.
		if (storager_getenv("STORAGER_PCHIST_WIDE"))
		{
			double const tend = storager_getenv("STORAGER_PCHIST_END") ? atof(storager_getenv("STORAGER_PCHIST_END")) : 29.5;
			++m_pcsamp_n;
			if (machine().time().as_double() >= tend) { logerror("PCHIST WIDE final window:\n"); dump_pchist(); m_pcsamp_on = false; return; }
			m_pcsamp->adjust(attotime::from_usec(4));
			return;
		}
		u32 const cap = storager_getenv("STORAGER_PCHIST_N") ? u32(atoi(storager_getenv("STORAGER_PCHIST_N"))) : 200000;
		if (++m_pcsamp_n >= cap) { logerror("PCHIST window: [$749c] OR over window = %04x\n", m_pcsamp_749c_or); dump_pchist(); m_pcsamp_on = false; return; }
		m_pcsamp->adjust(attotime::from_nsec(500));
	}
	void dump_pchist()
	{
		std::vector<std::pair<u32, u32>> v(m_pchist.begin(), m_pchist.end());
		std::sort(v.begin(), v.end(), [](auto const &a, auto const &b){ return a.second > b.second; });
		logerror("PCHIST n=%u distinct=%u top:\n", m_pcsamp_n, u32(v.size()));
		for (int i = 0; i < 30 && i < int(v.size()); i++)
			logerror("  %2d. pc=%06x  %6u  (%4.1f%%)\n", i + 1, v[i].first, v[i].second, 100.0 * v[i].second / m_pcsamp_n);
	}
	emu_timer *m_seek_done = nullptr;  // task#4: gate-array seek-complete delivery, armed at the 0x89/0x98 doorbell
	// The seek settles after real time (m_seek_deadline); at that instant the gate array posts its completion
	// status to the host IOPB + local mailbox and raises IRQ2.  The 68000 firmware's own $24ea walk then bumps
	// [$71b2] (node+26 has reached 0 by settle, so the $251e gate passes) and the $369a seek-wait releases.
	// GUARDRAIL: the timer raises IRQ2 + posts the HW mailbox; the FIRMWARE bumps [$71b2] - never poked from C++.
	TIMER_CALLBACK_MEMBER(seek_done_tick)
	{
		if (m_seek_fired || !m_seek_iopb || !storager_getenv("STORAGER_NOBYPASS")) return;   // NOBYPASS-only; shipped HLE boot untouched
		// cont.171 (TEST, STORAGER_FWRESTORE): the C++ host-post below is the residual HLE shim that
		// completes 0x89/0x98 in 75ms and robs the restore of its warm-up ladder ({op-24 head-load,
		// op-26} built at $5f74; fw's own DONE = the sole $1a54 stamp). With this env set, the fw
		// must complete the restore itself - the LLE-mandate path. Watch: {24,26} op-walk at 6.40x,
		// head-load 60t + motor 70t paid DURING the restore, fw posts 0x80 ~9.8, read rides warm.
		if (storager_getenv("STORAGER_FWRESTORE"))
		{ logerror("SEEK-DONE(timer) suppressed (FWRESTORE) - fw owns the restore @%.5f\n", machine().time().as_double()); m_seek_fired = true; return; }
		// A SEEK-COMPLETE IS A LOCAL EVENT, not a command completion (Dave): the storager's own CPU learns its
		// seek settled and advances its state machine.  It must NOT post the host-IOPB DONE - that is reserved
		// for the whole-transaction completion the fw posts at $0BF6.  Posting host DONE for a mere seek made
		// the CPUAP read "command done" and re-drive the storager -> IRQ2 storm thrashing the aliased $71f0
		// window (task-#2 0x02 garbage).  So: assert channel-op-OK + raise IRQ2 ONLY; the fw's own $24fe walk
		// then bumps [$71b2] (its conditions - [$7ff8] bit0, node+26==0, [$71b2]!=2, [$7ffa]!=0xff - are all
		// already satisfied during the spin, so the walk bumps as soon as it runs).  No host footprint.
		// 0x89 RESTORE is a HOST command; its completion is reported to the HOST (every command completion is,
		// posted at $0BF6 in the fw's own flow).  At the seek settle, post the restore's host-IOPB DONE - the
		// CPUAP polls it and issues the next command (0x95), and THAT host mailbox edge releases the $369a
		// handshake wait itself (measured: the working boot releases $369a on the host's next [$7ff8] GO, not a
		// channel signal).  So NO IRQ2 from here - the host drives the release.  The fw tolerates the D000/$71f0
		// window overlap on pickup exactly as it does in the working boot (the garbage pickup is not the bug).
		address_space &bs = m_bus->space(AS_PROGRAM);
		// The host reuses the IOPB slot for the next command: if it no longer holds the armed seek
		// (0x89/0x98), this fire is stale - posting DONE would falsely complete the successor (measured
		// run6: posted DONE on the 0x95 READ mid-ID-scan -> host 95/89/95 retry churn). Retire silently.
		u8 const cur_cmd = bs.read_byte(m_seek_iopb & 0xffffff);
		if (cur_cmd != 0x89 && cur_cmd != 0x98)
		{
			m_seek_fired = true;
			if (storager_getenv("STORAGER_NOBYPASS")) logerror("SEEK-DONE(timer) STALE - iopb=%06x now cmd=%02x, no host post @%.5f\n", m_seek_iopb, cur_cmd, machine().time().as_double());
			return;
		}
		if (storager_getenv("STORAGER_NOBYPASS")) logerror("IOPB80-POST(seek) L%d @%.5f\n", __LINE__, machine().time().as_double());
		if (storager_getenv("STORAGER_FWDONE")) { logerror("IOPB80-SKIPPED(FWDONE) @%.5f\n", machine().time().as_double()); } else {
							bs.write_byte((m_seek_iopb + 2) & 0xffffff, 0x80);   // restore complete -> host IOPB status = DONE
		bs.write_byte((m_seek_iopb + 3) & 0xffffff, 0x80);
		}
		m_ch_op_ok = true;          // E01E bit4 = channel-op-OK (the fw's retry gate)
		m_read_pending = false;
		m_seek_fired = true;        // one delivery per armed seek
		if (storager_getenv("STORAGER_NOBYPASS")) logerror("SEEK-DONE(timer,host-post) iopb=%06x @%.5f\n", m_seek_iopb, machine().time().as_double());
	}
	// task#5 (STRIP): LLE seek-complete. The read's IMPLICIT seek ($1e2e->$367a, MEASURED as the terminal spin)
	// steps the head via $e800 and waits on [$71b2]/[$7b1a], which only a channel IRQ2 (fw $262c) bumps. Under
	// LLE no next-command doorbell comes (the host waits for the read), so the seek needs a REAL hardware
	// seek-complete: re-arm this settle timer on each $e800 step-write (pc in the $369a loop); on settle fire ONE
	// IRQ2 (HOLD_LINE = auto-clear, cannot storm). The fw's own $24fe/$262c walk bumps [$71b2] - never from C++.
	emu_timer *m_stepdone = nullptr;
	// cont.168 (TEST, STORAGER_E807ACK): the jam end-to-end probe - answer every E807
	// channel-op dispatch with the fw's own completion contract (mbox[0]=0x02 = bit1
	// "channel-op complete" -> the $2510/$2634 ring-consume path; desc+2 match at $265e)
	// after a PLACEHOLDER 20ms. NOT faithful timing - proves/disproves the ring jam only.
	emu_timer *m_e807ack = nullptr;
	int m_bustap_hits = 0;   // cont.176: bus-tap self-test counter
	bool m_stepdone_armed = false;
	// task#5 (STRIP): drive's per-step seek-active response. On the E802 step pulse the real drive asserts
	// seek-active (f000 bit1 SET) then clears it on settle. The pit0 mode-5 one-shot that should model this
	// (m_settle_out) is never programmed in this flow, so drive it directly: SET on the step, CLEAR after a
	// short settle so $a118's wait-SET-then-wait-CLEAR per step is satisfied. Gated STORAGER_SEEKACTIVE.
	emu_timer *m_settle_clear = nullptr;
	TIMER_CALLBACK_MEMBER(settle_clear_tick) { m_settle_out = true; }   // settled -> f000 bit1 CLEAR
	// Channel-op FRAME-complete interrupt (build#5 cont.5, MEASURED): the $3bfe IRQ4 walker is the
	// only agent that processes a finished op's queue node (NULL continuation [$14] -> clr busy
	// [$749c] + cancel the 0x18 timeout at $2aba) and it ONLY runs on IRQ4. In steady state no IRQ4
	// ever fires after the op frame ends (run24: real walker passes only at doorbell/dispatch
	// edges; the post-frame silence is the whole 2.61s retry loop). On real HW the gate array
	// raises the completion interrupt when its op queue runs dry: model = a short settle after the
	// last step completion with NO new E800 op submission in between (m_stepdone_armed re-set
	// cancels). The walker's clear path is proven executable (749c<-0000 pc=3c7c, run24 @6.40075/
	// 6.40207) - this pump lands on a walker that WILL step. NOTE: IRQ4, not IRQ6 - the
	// measurement corrected the earlier guess (IRQ6 = the SERDES/microseq pump, a later phase).
	emu_timer *m_framedone = nullptr;
	// cont.43probe (STRIP): the ch0-resume DIAGNOSTIC. Hypothesis: the gate array's capture
	// channel, on running dry after the batch, posts the ch0 mailbox + IRQ2 (same event class
	// as the modeled step completion) - the resume circuit's missing upstream event. One-shot
	// stimulus at t=10.5 (parked, wedged) under STORAGER_CH0PROBE=1: post the calibrated ch0
	// signature and watch whether op-4 -> worker -> $7444 descriptor -> kick -> op-8 -> $c
	// runs on the firmware's own legs. Response mapping ONLY - not a model behavior yet.
	emu_timer *m_ch0probe = nullptr;
	// cont.44 (KEEPER candidate): the CAPTURE channel's completion post - structural twin of
	// stepdone-per-settle. The gate array's capture channel, on running DRY (no new capture
	// completion within 30ms - two+ sector times past the 12.5ms cadence), posts the channel
	// mailbox + IRQ2, same event class as the modeled step completion. This is the IRQ-plumbing
	// absence of the bit6-ack family: the fw's control flow assumes the post ([$71b2] pumping,
	// idle-node op-4 worker passes); without it collection ends into silence and the node parks
	// into a wedge that cannot exist on real hardware. Posts the ch1/idle-node event with the
	// run13-calibrated signature (13/e7/00/00 - the proven-consumable flavor, run169).
	emu_timer *m_capdone = nullptr;
	int m_capdone_phase = 0;
	u16 m_e802_prev_c44 = 0;
	TIMER_CALLBACK_MEMBER(capdone_tick)
	{
		if (!storager_getenv("STORAGER_CAPCH0")) return;
		address_space &ds = m_cpu->space(AS_PROGRAM);
		if (m_capdone_phase == 0)
		{
			if (ds.read_byte(0x7ff8))
			{ logerror("CAPDONE skip: ch1 mailbox unconsumed (7ff8=%02x) @%.5f\n", ds.read_byte(0x7ff8), machine().time().as_double()); return; }
			ds.write_byte(0x7ffa, storager_getenv("STORAGER_MBOXW") ? 0x00 : 0xe7);
			ds.write_byte(0x7ffc, 0x00); ds.write_byte(0x7ffe, 0x00);
			ds.write_byte(0x7ff8, 0x13);
			m_cpu->set_input_line(M68K_IRQ_2, HOLD_LINE);
			logerror("CAPDONE post ch1 + IRQ2 (capture channel dry) @%.5f\n", machine().time().as_double());
			m_capdone_phase = 1;
			m_capdone->adjust(attotime::from_msec(1));
		}
		else
		{
			m_capdone_phase = 0;
			m_cpu->set_input_line(M68K_IRQ_2, HOLD_LINE);
		}
	}
	TIMER_CALLBACK_MEMBER(iopbdump_tick)
	{
		address_space &bs = m_bus->space(AS_PROGRAM);
		std::string h;
		for (u32 k = 0; k < 0x20; k++) h += util::string_format(" %02x", bs.read_byte(0x0fe780 + k));
		logerror("IOPBDUMP 0fe780:%s @%.5f\n", h.c_str(), machine().time().as_double());
		std::string h2;
		for (u32 k = 0; k < 0x10; k++) h2 += util::string_format(" %02x", bs.read_byte(0x0fc0dd + k));
		logerror("IOPBDUMP buf@0fc0dd:%s @%.5f\n", h2.c_str(), machine().time().as_double());
	}
	TIMER_CALLBACK_MEMBER(ch0probe_tick)
	{
		address_space &ds = m_cpu->space(AS_PROGRAM);
		static int phase = 0;
		if (phase == 0)
		{
			// v2: run166 found 7ff0 ALREADY 01 (stale unconsumed completion from the seek era) -
			// the walk's changed-gate ([$7ff0] vs [$71e0]) saw no edge. Log the gate pair, clear,
			// then post fresh 30us later.
			logerror("CH0PROBE pre: 7ff0=%02x 71e0=%02x 7ff2/4/6=%02x/%02x/%02x 71b2=%02x nstat=%04x @%.5f\n",
				ds.read_byte(0x7ff0), ds.read_byte(0x71e0), ds.read_byte(0x7ff2), ds.read_byte(0x7ff4),
				ds.read_byte(0x7ff6), ds.read_byte(0x71b2), ds.read_word(0x71ec), machine().time().as_double());
			ds.write_byte(0x7ff0, 0x00);
			phase = 1;
			m_ch0probe->adjust(attotime::from_usec(30));
		}
		else if (phase == 1)
		{
			// v3: the $251e gate ignores bit0 events on a BUSY node (the stale ch0 01). The
			// calibrated seek-release ran via CH1 ($7ff8, sig 13/e7/00/00) against the IDLE node
			// $71f0 - channel completions stamp op-4 on the idle node, whose op-4 pass runs the
			// SHARED worker (both nodes' descriptors). Post the ch1 signature.
			ds.write_byte(0x7ffa, storager_getenv("STORAGER_MBOXW") ? 0x00 : 0xe7);
			ds.write_byte(0x7ffc, 0x00); ds.write_byte(0x7ffe, 0x00);
			ds.write_byte(0x7ff8, 0x13);
			m_cpu->set_input_line(M68K_IRQ_2, HOLD_LINE);
			logerror("CH0PROBE posted ch1 13/e7/00/00 + IRQ2 (idle-node route) @%.5f\n", machine().time().as_double());
			phase = 2;
			m_ch0probe->adjust(attotime::from_msec(1));   // second edge: pass 1 consumes the stale
		}                                                 // ch0 dedupe; pass 2 reaches ch1
		else if (phase == 2)
		{
			m_cpu->set_input_line(M68K_IRQ_2, HOLD_LINE);
			logerror("CH0PROBE second IRQ2 edge @%.5f\n", machine().time().as_double());
			phase = 3;
			m_ch0probe->adjust(attotime::from_msec(100));
		}
		else
		{
			logerror("CH0PROBE +100ms: 7ff8=%02x 720a=%04x 71b2=%02x nstat=%04x n2stat=%04x 7220=%04x 7228=%04x @%.5f\n",
				ds.read_byte(0x7ff8), ds.read_word(0x720a), ds.read_byte(0x71b2), ds.read_word(0x71ec),
				ds.read_word(0x7216), ds.read_word(0x7220), ds.read_word(0x7228), machine().time().as_double());
		}
	}
	TIMER_CALLBACK_MEMBER(frame_done_tick)
	{
		if (!storager_getenv("STORAGER_STEPIRQ")) return;
		if (storager_getenv("STORAGER_NOFRAME4")) return;   // A/B (cont.30): possibly redundant since the
		                                           // CHANCOMPLETE un-suppression (real per-submit
		                                           // IRQ4s); suspect = the premature node cleanup
		                                           // (state $a -> 0) that blocks the wait-scan resume
		if (m_stepdone_armed) return;   // a new op was submitted - frame not dry, no completion yet
		logerror("FRAME-IRQ4 (channel op frame complete) @%.5f\n", machine().time().as_double());
		m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE);
	}
	TIMER_CALLBACK_MEMBER(e807ack_tick)
	{
		// cont.168 (TEST): post the channel-op completion the gate array owes.
		u16 const mbox = param & 0xffff;
		address_space &ds = m_cpu->space(AS_PROGRAM);
		u8 const last = (mbox == 0x7ff0) ? ds.read_byte(0x71e0) : ds.read_byte(0x720a);
		u8 stamp = 0x02;
		if (stamp == last) stamp = 0x22;   // must differ from the last-consumed id (the $24de gate)
		ds.write_byte(mbox, stamp);
		logerror("E807ACK post mbox=%04x stamp=%02x (last=%02x) @%.6f\n", mbox, stamp, last, machine().time().as_double());
		m_cpu->set_input_line(M68K_IRQ_2, HOLD_LINE);
	}
	TIMER_CALLBACK_MEMBER(stepdone_tick)
	{
		if (!storager_getenv("STORAGER_STEPIRQ")) return;
		address_space &ds = m_cpu->space(AS_PROGRAM);
		// NOTE: the $7e00 ring-stamp resume (RINGDONE bit7 + [$7e1c]) is deliberately NOT built.
		// Measured (runs 9-11): stamping an empty slot resumes a zero payload -> junk re-dispatch.
		// Build it ONLY if the fw measurably PARKS ($1b90 snapshot into the ring) for this op -
		// the restore may complete inline via $36de once completion keys on real steps.
		m_stepdone_armed = false;
		// The seek-complete is a CHANNEL-0 MAILBOX event, not a bare IRQ2 (MEASURED: bare IRQ2 skips the $24fe
		// walk's bit0 gate). ch0 = $7ff0/$7fe0/$71c6 (the internal disk channel); the fw only CLEARS [$7ff0]
		// (never sets bit0) -> bit0 is HARDWARE-set. Stamp [$7ff0] bit0 (models the channel controller marking
		// op-complete) + raise IRQ2; the fw's OWN $24fe/$262c walk reads the mailbox and bumps [$71b2]/[$7b1a]
		// (shared -> releases the read's ch1 seek). NEVER write [$71b2]/[$7b1a] from C++ - only the HW mailbox.
		u8 const before71b2 = ds.read_byte(0x71b2);
		// ch0 completion signature, CALIBRATED against the working 0x87 completion (run13 log: the
		// $2632 BUMP ran with ch1 mailbox 13/e7/00/00): status byte $e7, rest clear. This steers the
		// IRQ2 walk to the BUMP path ($2632: [$71b2]++ -> the $369a INLINE release the seek needs),
		// NOT the $2570 ring-claim path - the earlier ff/ff/ff triple steered to the ring/coroutine
		// path, which the measurement (run13: PARK-1b90 = 0, INLINE-36de only) proved this op never
		// uses. The triple is HW/dual-port status, same class as bit0.
		logerror("STEPDONE pre: 7ff0=%02x 7ff2/4/6=%02x/%02x/%02x 71b2=%02x @%.5f\n",
			ds.read_byte(0x7ff0), ds.read_byte(0x7ff2), ds.read_byte(0x7ff4), ds.read_byte(0x7ff6), before71b2, machine().time().as_double());
		// cont.207: the "calibrated $E7 status" was the OLD-LAYOUT GARBAGE fossilised - run13's
		// ch1 "13/e7/00/00" was the IOPB pointer's MIDDLE byte (0x0F_E7_80) read through the
		// byte-contiguous relay, never a status. Under MBOXW the fw consumes the status byte
		// for real (it lands in node+$18 and an 0x87 fetch-wait publishes it as the 0x82's
		// error code) - the hardware's OK status is 0x00.
		ds.write_byte(0x7ff2, storager_getenv("STORAGER_MBOXW") ? 0x00 : 0xe7);
		ds.write_byte(0x7ff4, 0x00); ds.write_byte(0x7ff6, 0x00);
		ds.write_byte(0x7ff0, 0x01);   // stamp bit0 LAST (the walk keys on [$7ff0] != [$71e0])
		// cont.170 (STRIP): the fast-tick consume race - the IRQ2 bit0 path's entry guards
		// at delivery time: node $71c6+$26 status (must be 0), [$71b2] (must != 2), [$71e0]
		// (the last-consumed stamp - mbox[0] must differ). At clk10M the fw never consumes;
		// which guard is failing?
		logerror("STEPDONE guards: 71c6+26=%04x 71b2=%02x 71e0=%02x 720a=%02x 7ff8=%02x @%.5f\n",
			ds.read_word(0x71c6 + 0x26), ds.read_byte(0x71b2), ds.read_byte(0x71e0),
			ds.read_byte(0x720a), ds.read_byte(0x7ff8), machine().time().as_double());
		m_cpu->set_input_line(M68K_IRQ_2, HOLD_LINE);
		// frame-dry check 2ms out: gives the fw's IRQ2 walk + list-walk time to consume this
		// completion; canceled implicitly if a new op-entry re-arms m_stepdone_armed meanwhile.
		m_framedone->adjust(attotime::from_msec(2));
	}
	u16 m_e802_prev = 0;               // for the transfer-arm (bit15) edge
	bool m_ch_op_ok = false;           // fw-driven: the last channel op succeeded (E01E bit4 level)
	bool m_read_pending = false;       // fw-driven READ: hold BUSY (suppress the 0x7fe8 DONE tap) until the kickoff delivers the data (STEP 324: deliver-THEN-done)
	unsigned m_dc_seq = 0;             // rotation: successive captures see successive marks
	TIMER_CALLBACK_MEMBER(idcap_tick);
	TIMER_CALLBACK_MEMBER(dataop_tick);

	u8   m_ioreg[8];         // Multibus I/O 0x0800-0x0807 register file: the OS driver's command channel
	bool m_ioreg_done;       // controller DONE: reads of the cmd byte return bit7 set
	emu_timer *m_ioreg_int = nullptr;  // deferred completion interrupt (a real controller takes ms; an
	                                   // instant int races the driver setting its "command outstanding"
	                                   // state -> handler early-out -> unquenched level int -> stack overflow)
	// TEMP (STRIP): read-delivery overlap detector - catch a read depositing disk data into a host
	// range a DIFFERENT device recently read (buffer-cache clobber = the false-full hypothesis)
	struct rd_deliv { u32 req = 0, len = 0, pos = 0; u8 unit = 0xff; };
	rd_deliv m_rd_ring[24];
	unsigned m_rd_ridx = 0;
	void rd_check(u8 unit, u32 pos, u32 req, u32 len)
	{
		for (auto const &e : m_rd_ring)
			if (e.len && e.unit != unit && req < e.req + e.len && e.req < req + len)
			{
				static int n = 0;
				if (n++ < 40)
					logerror("RDOVERLAP: unit%u pos=%u req=%06x len=%u OVERLAPS prior unit%u pos=%u req=%06x len=%u @%.4f\n",
							unit, pos, req, len, e.unit, e.pos, e.req, e.len, machine().time().as_double());
				break;
			}
		m_rd_ring[m_rd_ridx++ % 24] = { req, len, pos, unit };
	}

	int m_ioreg_pending = 0;           // outstanding interrupt-driven completions - the shared timer +
	                                   // single level line lost stacked completions => guest livelock
	                                   // (the "beachball"); the counter holds INT2 asserted until every
	                                   // completion is acknowledged by a status read
	u32 m_temp_cid = 0;   // TEMPORAL PROBE (STRIP): per-command instance id (Dave STEP 367b-d)
	TIMER_CALLBACK_MEMBER(ioreg_int) { if (m_ioreg_pending > 0) { logerror("TEMPO INT2 cid=%u t=%.6f\n", m_temp_cid, machine().time().as_double()); int_w<2>(0); } }   // Multibus INT ACTIVE-LOW; ICU IR3 level

	// fallback (decoded-sector feed): IMD parsed into (cyl<<16 | head<<8 | sector) -> bytes.
	std::map<u32, std::vector<u8>> m_sectors;
	bool m_floppy_loaded;
};

void multibus_storager_device::device_start()
{
	std::fill(std::begin(m_ch), std::end(m_ch), 0);
	save_item(NAME(m_ch));
	save_item(NAME(m_trace));
	save_item(NAME(m_c000));
	save_item(NAME(m_c000_valid));
	m_ioreg_int = timer_alloc(FUNC(multibus_storager_device::ioreg_int), this);
	m_idcap = timer_alloc(FUNC(multibus_storager_device::idcap_tick), this);
	m_dataop = timer_alloc(FUNC(multibus_storager_device::dataop_tick), this);
	m_pump = timer_alloc(FUNC(multibus_storager_device::pump_tick), this);
	m_hd_chan = timer_alloc(FUNC(multibus_storager_device::hd_chan_tick), this);
	m_pcsamp = timer_alloc(FUNC(multibus_storager_device::pcsamp_tick), this);
	m_stepdone = timer_alloc(FUNC(multibus_storager_device::stepdone_tick), this);
	m_e807ack = timer_alloc(FUNC(multibus_storager_device::e807ack_tick), this);
	m_settle_clear = timer_alloc(FUNC(multibus_storager_device::settle_clear_tick), this);
	m_framedone = timer_alloc(FUNC(multibus_storager_device::frame_done_tick), this);
	m_ch0probe = timer_alloc(FUNC(multibus_storager_device::ch0probe_tick), this);
	// cont.64 (STRIP): what the CPUAP actually read - the host IOPB block right after the verdict.
	if (storager_getenv("STORAGER_IOPBDUMP"))
		for (double tt : { 8.185, 8.196, 8.3 })
			machine().scheduler().timer_set(attotime::from_double(tt), timer_expired_delegate(FUNC(multibus_storager_device::iopbdump_tick), this));
	m_capdone = timer_alloc(FUNC(multibus_storager_device::capdone_tick), this);
	if (storager_getenv("STORAGER_CH0PROBE")) m_ch0probe->adjust(attotime::from_double(10.5));
	m_seek_done = timer_alloc(FUNC(multibus_storager_device::seek_done_tick), this);
	m_chancomplete = timer_alloc(FUNC(multibus_storager_device::chancomplete_tick), this);
	// DIAGNOSTIC: definitive "IRQ2 taken" probe - the m68k fetches the level-2 autovector from $68 on every IRQ2
	// acknowledge, regardless of handler path.  (The re-entrant $71b2 read-tap IRQ2 assert was abandoned - a tap
	// asserts on the CPU's own bus cycle, which is void; delivery works from a clean timer context.)
	if (storager_getenv("STORAGER_PHASELOG"))
		m_cpu->space(AS_PROGRAM).install_read_tap(0x006c, 0x006f, "irq3vec",
			[this](offs_t, u16 &, u16)
			{ static int n3 = 0; double const t3 = machine().time().as_double();
			if (n3++ < 30)
				logerror("IRQ3-TAKEN v72f8=%04x%04x @%.5f\n",
					m_cpu->space(AS_PROGRAM).read_word(0x72f8), m_cpu->space(AS_PROGRAM).read_word(0x72fa),
					t3); });
		m_cpu->space(AS_PROGRAM).install_read_tap(0x0068, 0x006b, "irq2vec",
			[this](offs_t, u16 &, u16)
			{ logerror("IRQ2-TAKEN (autovec $68 fetch) pc=%06x @%.5f\n", m_cpu->pc(), machine().time().as_double()); });
		// cont.262c (STORAGER_STAKEV): RELIABLE IRQ census via the 68000 autovector fetches
		// (AS_PROGRAM reads, like irq2/irq3 above - not the flaky narrow AS_OPCODES taps). $74 =
		// IRQ5 ACK, $78 = IRQ6 ACK. Log where each dispatches ([[$7300]] IRQ5 / [[$7304]] IRQ6 soft
		// vectors) + phase/[$742c]. Answers: is IRQ6 (the DATA-fork carrier) actually taken, and to $298c?
		for (auto iv : { std::pair<u32,char const*>{0x0074u,"IRQ5"}, {0x0078u,"IRQ6"} })
		{
			u32 const va=iv.first; char const* nm=iv.second;
			m_cpu->space(AS_PROGRAM).install_read_tap(va, va+3, nm,
				[this,nm](offs_t, u16 &, u16){ static std::map<char const*,int> c;
				double const it = machine().time().as_double();
				if (!storager_getenv("STORAGER_STAKEV") || it < 7.99 || it > 8.06 || c[nm]++ >= 60) return;
				address_space &is = m_cpu->space(AS_PROGRAM);
				u32 const v5 = (u32(is.read_word(0x7300))<<16)|is.read_word(0x7302);
				u32 const v6 = (u32(is.read_word(0x7304))<<16)|is.read_word(0x7306);
				logerror("%s-TAKEN vec5=%06x vec6=%06x 7950=%04x 742c=%04x aim=%04x @%.6f\n", nm,
					v5&0xffffff, v6&0xffffff, is.read_word(0x7950), is.read_word(0x742c), is.read_word(0x7428), it); });
		}
		// task#5: $1c8c (Xdisp, the channel-op dispatch the read is diverted into) is reached by an INDIRECT jsr -
		// tap its entry and read the return addr off the stack ([A7]) to NAME the main-thread router that routes
		// 0x95 -> Xdisp vs 89/87/98 -> pickup/$d06.  (Static read impossible: $1c8c is never a literal branch target.)
		// task#5 reassess: does $5FC0 (read handler) / $60a4 ($28e0 enqueue) / $60e4 ($7434 build) EVER run, even at
		// boot? (if $5FC0 ran once at init it seeded the stale $28e0 the IRQ6 pump keeps re-running.)  Whole boot.
		m_cpu->space(AS_OPCODES).install_read_tap(0x5fc0, 0x5fc1, "rs_5fc0",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ < 40) logerror("HIT $5fc0 READ-handler cmd=%02x @%.5f\n", m_iopb_cmd, machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x60a4, 0x60a5, "rs_60a4",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ < 40) logerror("HIT $60a4 enqueue-$28e0 cmd=%02x @%.5f\n", m_iopb_cmd, machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x60e4, 0x60e5, "rs_60e4",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ < 40) logerror("HIT $60e4 build-$7434 cmd=%02x @%.5f\n", m_iopb_cmd, machine().time().as_double()); });
		// task#5 gate: $9400 = read/write channel setup. reads [A6+$20], btst #14 -> arm node+6 = $5fc0 (READ, $9412)
		// or $6102 (WRITE, $940a). Does this setup EVER run for our command, and does it pick READ or WRITE?
		m_cpu->space(AS_OPCODES).install_read_tap(0x9400, 0x9401, "rs_9400",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ < 40) logerror("HIT $9400 rw-setup cmd=%02x @%.5f\n", m_iopb_cmd, machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x9412, 0x9413, "rs_9412",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ < 40) logerror("HIT $9412 arm-READ($5fc0) cmd=%02x @%.5f\n", m_iopb_cmd, machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x940a, 0x940b, "rs_940a",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ < 40) logerror("HIT $940a arm-WRITE($6102) cmd=%02x @%.5f\n", m_iopb_cmd, machine().time().as_double()); });
		// also: does the channel-service routine that CONTAINS $9400 run at all? tap its head $93b0.
		m_cpu->space(AS_OPCODES).install_read_tap(0x93b0, 0x93b1, "rs_93b0",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ < 40) logerror("HIT $93b0 chan-setup-entry cmd=%02x @%.5f\n", m_iopb_cmd, machine().time().as_double()); });
		// task#5: capture the read's requested sector COUNT (IOPB) at READ-START, non-invasively (opcode tap reads
		// host IOPB via bs - a read, not a write-tap). The model pump feeds rotationally w/o count; the fix is a
		// count-driven transfer-complete. Dump the read IOPB fields to find the count.
		// task#5: does the fw's target sector MATCH the model's rotationally-fed sectors? tap IDCHK ($89f2): log
		// the fw target (7438 cyl / 7436 head / 7428 R) vs the delivered ID at $7dac (fe c h r n). Non-invasive.
		// cont.260 (STRIP): the channel-op completion trace - who sets/clears [$7454]
		// (channel-busy = $7442+$12; the $1E34 wait), and does $3FD0/$45EE (the clearer)
		// or $417A/$42FC (the setter) ever run? Names the missing channel-op-done state.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7454, 0x7455, "w7454",
			[this](offs_t, u16 &data, u16){ static int n=0; if(n++<120)
				logerror("W7454 <- %04x pc=%06x 71b2=%04x @%.6f\n", data, m_cpu->pc(),
					m_cpu->space(AS_PROGRAM).read_word(0x71b2), machine().time().as_double()); });
		// cont.260 (STRIP): the re-arm-vs-complete fork - who clears [$741C] (capture-in-flight),
		// which clears [$7968] at $82C6, which drops the fw off the completion path into re-arm.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x741c, 0x741d, "w741c",
			[this](offs_t, u16 &data, u16){ static int n=0; if(n++<120)
				logerror("W741C <- %04x pc=%06x 7968=%04x 7956=%04x aim=%04x @%.6f\n", data, m_cpu->pc(),
					m_cpu->space(AS_PROGRAM).read_word(0x7968), m_cpu->space(AS_PROGRAM).read_word(0x7956),
					m_cpu->space(AS_PROGRAM).read_word(0x7428), machine().time().as_double()); });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7968, 0x7969, "w7968c",
			[this](offs_t, u16 &data, u16){ static int n=0; if(n++<120)
				logerror("W7968 <- %04x pc=%06x 741c=%04x 7956=%04x aim=%04x @%.6f\n", data, m_cpu->pc(),
					m_cpu->space(AS_PROGRAM).read_word(0x741c), m_cpu->space(AS_PROGRAM).read_word(0x7956),
					m_cpu->space(AS_PROGRAM).read_word(0x7428), machine().time().as_double()); });
		for (auto pcp : { std::pair<u16,char const*>{0x3fd0,"CHANCOMPL-3FD0"}, {0x45ee,"CHANCOMPL-45EE"},
				{0x1e34,"WAIT-1E34"}, {0x417a,"CHANLAUNCH-417A"}, {0x42fc,"CHANLAUNCH-42FC"} })
		{
			u16 const a=pcp.first; char const* nm=pcp.second;
			m_cpu->space(AS_OPCODES).install_read_tap(a, a+1, nm,
				[this,nm](offs_t, u16 &, u16){ static std::map<char const*,int> cnt; if(cnt[nm]++<40)
					logerror("%s pc=%06x 7454=%04x 71b2=%04x @%.6f\n", nm, m_cpu->pc(),
						m_cpu->space(AS_PROGRAM).read_word(0x7454),
						m_cpu->space(AS_PROGRAM).read_word(0x71b2), machine().time().as_double()); });
		}
		// cont.260 (STRIP): the REAL completion path (not the dead pump/watch chain) -
		// closer $810E (gates [$7424]==0), op-00 $16C6, the 0x80 stamp $1A54, and the
		// walker's phase write $15A0 (node+26 <- 0xC = ladder-end). Where does it stall
		// after the 8th sector? Plus the closer-gate cells at each.
		for (auto pcp : { std::pair<u16,char const*>{0x810e,"CLOSER-810E"}, {0x16c6,"OP00-16C6"},
				{0x1a54,"STAMP80-1A54"}, {0x15a0,"PHASEW-15A0"}, {0x9342,"BATCHEND-9342"} })
		{
			u16 const a=pcp.first; char const* nm=pcp.second;
			m_cpu->space(AS_OPCODES).install_read_tap(a, a+1, nm,
				[this,nm](offs_t, u16 &, u16){ static std::map<char const*,int> c; if(c[nm]++<30)
					logerror("%s pc=%06x 7424=%04x 7968=%04x 7956=%04x 7454=%04x aim=%04x @%.6f\n", nm,
						m_cpu->pc(), m_cpu->space(AS_PROGRAM).read_word(0x7424),
						m_cpu->space(AS_PROGRAM).read_word(0x7968), m_cpu->space(AS_PROGRAM).read_word(0x7956),
						m_cpu->space(AS_PROGRAM).read_word(0x7454), m_cpu->space(AS_PROGRAM).read_word(0x7428),
						machine().time().as_double()); });
		}
		// cont.262 (STORAGER_SERVE): does the serve ($7106->$7114) run and MATCH our aim,
		// setting [$742c] (stake-owed) coincident with the data pass? $7150 skip-gate,
		// $715a/$715e window compares, $7162 the SET.
		for (auto sp : { std::pair<u16,char const*>{0x7106u,"SERVE-ENTRY"}, {0x7150u,"SERVE-7150"}, {0x7162u,"SERVE-SET742C"}, {0x716au,"SERVE-SKIP"} })
		{
			u16 const a=sp.first; char const* nm=sp.second;
			m_cpu->space(AS_OPCODES).install_read_tap(a, a+1, nm,
				[this,nm](offs_t, u16 &, u16){ static std::map<char const*,int> c;
				double const st = machine().time().as_double();
				if (!storager_getenv("STORAGER_SERVE") || st < 7.99 || st > 8.06 || c[nm]++ >= 50) return;
				address_space &ss = m_cpu->space(AS_PROGRAM);
				logerror("%s aim=%04x win[%04x..+%04x] 7426=%04x 742c=%04x 7968=%04x d4=%04x @%.6f\n", nm,
					ss.read_word(0x7428), ss.read_word(0x7954), ss.read_word(0x7abc), ss.read_word(0x7426),
					ss.read_word(0x742c), ss.read_word(0x7968), u16(m_cpu->state_int(M68K_D4)), st); });
		}
		m_cpu->space(AS_OPCODES).install_read_tap(0x89f2, 0x89f3, "idmatch",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ >= 14) return; address_space &ds = m_cpu->space(AS_PROGRAM);
				u16 const b = 0x7dac; u16 const off = ds.read_byte(b)==0xa1 ? 3 : 0;   // skip MFM A1 A1 A1
				logerror("IDMATCH target{cyl=%04x head=%04x R=%02x} delivered{AM=%02x c=%02x h=%02x r=%02x n=%02x} @%.5f\n",
					ds.read_word(0x7438), ds.read_word(0x7436), ds.read_byte(0x7428),
					ds.read_byte(b+off), ds.read_byte(b+off+1), ds.read_byte(b+off+2), ds.read_byte(b+off+3), ds.read_byte(b+off+4), machine().time().as_double()); });
		// TEMP cont.240b (STRIP): counter WRITE taps - exact writers, no prefetch shadow
		if (storager_getenv("STORAGER_CHAINTAP"))
		{
			// cont.241b (REC512): the fw's aim-match writes [$742C]=1 ($7EB2) - THE record
			// start. Open the autonomous window there: the GA owns the next 3 sectors.
			if (storager_getenv("STORAGER_REC512"))
				m_cpu->space(AS_PROGRAM).install_write_tap(0x742c, 0x742d, "w742c",
					[this](offs_t, u16 &data, u16)
					{
						if (data == 1 && !m_rec_open && m_iopb_cmd == 0x95)
						{
							m_rec_open = true;
							m_rec_quiet = 3;
							m_rec_base = 0;   // cont.241e: latched at the record's FIRST deposit (C800[0] arms after the match)
							if (storager_getenv("STORAGER_PHASELOG")) { static int _ro = 0; if (_ro++ < 80)
								logerror("REC-OPEN (742c match) pc=%06x @%.6f\n", m_cpu->pc(), machine().time().as_double()); }
						}
					});
			m_cpu->space(AS_OPCODES).install_read_tap(0x15fe, 0x15ff, "PSELECT",
				[this](offs_t,u16&,u16){ static int n=0; double t=machine().time().as_double(); if(t<7.9||n>=8)return; n++;
					address_space &s = m_cpu->space(AS_PROGRAM);
					logerror("PSELECT rec727e+6=%04x  71f0+26=%02x 71f0+27=%02x  [71bc]=%04x [71b6]=%04x  721a=%04x @%.6f\n",
						s.read_word(0x7284), s.read_byte(0x71f0+0x26), s.read_byte(0x71f0+0x27),
						s.read_word(0x71bc), s.read_word(0x71b6), s.read_word(0x721a), t); });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7956, 0x7957, "w7956",
				[this](offs_t, u16 &data, u16)
				{ static int wn = 0; if (wn++ < 400)
					logerror("W7956 <- %04x pc=%06x @%.6f\n", data, m_cpu->pc(), machine().time().as_double());
				// cont.241: at the $6F5C seed, dump the whole count chain + the live local IOPB
				// copies ($71F0 ch0 / $71C6 alt) - settles the units question per command
				if (m_cpu->pc() >= 0x6f50 && m_cpu->pc() <= 0x6f70)
				{ static int sn2 = 0; if (sn2++ < 30)
					{ address_space &cs2 = m_cpu->space(AS_PROGRAM);
					std::string i1;
					for (u32 k = 0; k < 0x1c; k++) i1 += util::string_format(" %02x", cs2.read_byte(0x71f0 + k));
					logerror("SEED7956=%04x 7abe=%04x%04x 7abc=%04x 79a2=%04x 79a8=%04x uib=%04x | 71f0:%s @%.6f\n",
						data, cs2.read_word(0x7abe), cs2.read_word(0x7ac0), cs2.read_word(0x7abc),
						cs2.read_word(0x79a2), cs2.read_word(0x79a8), cs2.read_word(0x799a), i1.c_str(),
						machine().time().as_double()); } } });
			// TEMP cont.243 (STRIP): the $15FE watch-record walker's live state - which
			// cell/value the data-phase completion awaits. Slots $72D6-$72EC hold record
			// pointers; record = {+0 flag, +2 watch-ptr, +4 expected, +6 handler}.
			m_cpu->space(AS_OPCODES).install_read_tap(0x1646, 0x1647, "pumpconsume",
				[this](offs_t, u16 &, u16)
				{ static int pn = 0; if (pn++ < 40)
					logerror("PUMP-CONSUME reached @%.6f\n", machine().time().as_double()); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x1606, 0x1607, "watchdump",
				[this](offs_t, u16 &, u16)
				{ static int wd = 0; static double lastt = 0;
				double const nowt = machine().time().as_double();
				if (nowt - lastt > 0.5 && wd++ < 60)
					{ lastt = nowt;
					{ address_space &ws = m_cpu->space(AS_PROGRAM);
					std::string o;
					for (u16 slot = 0x72d6; slot <= 0x72ec; slot += 2)
					{
						u16 const rec = ws.read_word(slot);
						if (!rec) continue;
						u16 const fl = ws.read_word(rec), wp = ws.read_word((rec + 2) & 0xffff);
						u16 const ex = ws.read_word((rec + 4) & 0xffff), hd = ws.read_word((rec + 6) & 0xffff);
						o += util::string_format(" [%04x]rec=%04x{fl=%04x wp=%04x cur=%04x exp=%04x hd=%04x}",
							slot, rec, fl, wp, wp ? ws.read_word(wp) : 0, ex, hd);
					}
					logerror("WATCHDUMP%s @%.6f\n", o.c_str(), machine().time().as_double()); } } });
			// TEMP cont.244 (STRIP): the walker-selector record at $7224 - every step
			// pointer advance and return-code store, with pc
			// TEMP cont.246 (STRIP): who READS the staged IAM/capture record in the
			// wind-down (a8d3) era - the reader's pc names the floppy IAM consumer
			m_cpu->space(AS_PROGRAM).install_read_tap(0x7dac, 0x7db7, "iamreader",
				[this](offs_t offset, u16 &data, u16)
				{ static int rn = 0; static u32 lastpc = 0;
				double const rt = machine().time().as_double();
				u32 const rpc = m_cpu->pc();
				if (storager_getenv("STORAGER_IAMRD") && rt > 9.75 && rpc != lastpc && rn++ < 120)
					{ lastpc = rpc;
					logerror("IAMRD %04x pc=%06x d=%04x @%.6f\n", u32(offset) & 0xffff, data, rpc, rt); } });
			// TEMP cont.246b (STRIP): the E000 stream-pull in the wind-down - does the fw
			// pull, and what does the model serve?
			// TEMP cont.246e (STRIP): who writes the capture cells in the wind-down -
			// $7DAE holds cycling code addresses (live fw state); name every writer
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7dac, 0x7db7, "capwr",
				[this](offs_t offset, u16 &data, u16 mem_mask)
				{ static int cw = 0; double const wt2 = machine().time().as_double();
				u16 const hb = data >> 8, lb = data & 0xff;
				bool const codey = (hb >= 0x7b && hb <= 0x7f) || (lb >= 0x7b && lb <= 0x7f) || (mem_mask == 0xffff && data >= 0x1000 && data < 0xb000);
				if (storager_getenv("STORAGER_IAMRD") && wt2 > 9.75 && (codey || cw < 40) && cw++ < 400)
					logerror("CAPWR %04x <- %04x mm=%04x pc=%06x @%.6f\n", u32(offset) & 0xffff, data, mem_mask, m_cpu->pc(), wt2); });
			// TEMP cont.246d (STRIP): vector-cell write guard - who clobbers the IRQ3/5/6
			// trampoline pointers ($72F8/$7300/$7304)? The census shows execution at
			// impossible pcs (7dac/74/23f) + the level-7 panic handler running.
			// TEMP cont.254 (STRIP): [$7454] channel-busy set/clear with pc - the
			// never-completing final launch
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7454, 0x7455, "w7454",
				[this](offs_t, u16 &data, u16)
				{ static int cw4 = 0; static u16 last4 = 0xdead;
				double const ct4 = machine().time().as_double();
				if (storager_getenv("STORAGER_IAMRD") && ct4 > 3.5 && ct4 < 11.0 && data != last4 && cw4++ < 120)
					{ last4 = data;
					logerror("W7454 <- %04x pc=%06x @%.6f\n", data, m_cpu->pc(), ct4); } });
			// TEMP cont.253d (STRIP): rec2 ($94EC) execution detectors - [$794C] (only
			// setter $9524 inside the routine) + rec2's flag cell $7296 (activation)
			m_cpu->space(AS_PROGRAM).install_write_tap(0x794c, 0x794d, "w794c",
				[this](offs_t, u16 &data, u16)
				{ static int rw = 0; if (storager_getenv("STORAGER_IAMRD") && rw++ < 40)
					logerror("W794C <- %04x pc=%06x @%.6f\n", data, m_cpu->pc(), machine().time().as_double()); });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7296, 0x7297, "w7296",
				[this](offs_t, u16 &data, u16)
				{ static int rw2 = 0; if (storager_getenv("STORAGER_IAMRD") && rw2++ < 40)
					logerror("W7296 <- %04x pc=%06x @%.6f\n", data, m_cpu->pc(), machine().time().as_double()); });
			// cont.255b: the w7216 phase tap MOVED to the late install site (after the last
			// handler reinstall) - at this early site it was silently WIPED: run257 proved
			// phase stamps happen (HEADWR +26=0006) while this tap logged ZERO writes all
			// session. cont.255's "test (b) DEAD" verdict from tap silence is UNSOUND.
			// TEMP cont.253 (STRIP): UIB+$20 ($6E80) writers with pc - the pump-class
			// flags word's builder (bit4 gates the $1144 [$71B6] install)
			m_cpu->space(AS_PROGRAM).install_write_tap(0x6e80, 0x6e81, "w6e80",
				[this](offs_t, u16 &data, u16 mem_mask)
				{ static int uw = 0; if (storager_getenv("STORAGER_IAMRD") && uw++ < 60)
					logerror("W6E80 <- %04x mm=%04x pc=%06x @%.6f\n", data, mem_mask, m_cpu->pc(), machine().time().as_double()); });
			// TEMP cont.252b (STRIP): [$79BA] writers with pc - who consumes the stake flag
			m_cpu->space(AS_PROGRAM).install_write_tap(0x79ba, 0x79bb, "w79ba",
				[this](offs_t, u16 &data, u16)
				{ static int bw = 0; double const bt3 = machine().time().as_double();
				if (storager_getenv("STORAGER_IAMRD") && bt3 > 3.5 && bt3 < 11.0 && bw++ < 100)
					logerror("W79BA <- %04x pc=%06x @%.6f\n", data, m_cpu->pc(), bt3); });
			// TEMP cont.251c (STRIP): [$7968] writers with pc - which pop flow runs
			// (the $79D6 setter = "transfer outstanding" -> matches CLEAR 742c; the
			// $7EE0-pop never sets it -> matches re-arm -> the loop)
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7968, 0x7969, "w7968",
				[this](offs_t, u16 &data, u16)
				{ static int ow = 0; double const ot3 = machine().time().as_double();
				if (storager_getenv("STORAGER_IAMRD") && ot3 > 3.5 && ot3 < 11.0 && ow++ < 200)
					logerror("W7968 <- %04x pc=%06x @%.6f\n", data, m_cpu->pc(), ot3); });
			// TEMP cont.251 (STRIP): [$7424] writers with pc - which node launcher runs
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7424, 0x7425, "w7424",
				[this](offs_t, u16 &data, u16)
				{ static int nw = 0; double const nt3 = machine().time().as_double();
				if (storager_getenv("STORAGER_IAMRD") && nt3 > 3.5 && nt3 < 11.0 && nw++ < 200)
					logerror("W7424 <- %04x pc=%06x @%.6f\n", data, m_cpu->pc(), nt3); });
			// TEMP cont.250f (STRIP): toggler writes with pc - names the consuming fork
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7950, 0x7951, "togglerwr",
				[this](offs_t, u16 &data, u16)
				{ static int tw = 0; double const tt3 = machine().time().as_double();
				if (storager_getenv("STORAGER_IAMRD") && ((tt3 > 9.385 && tt3 < 9.44) || (tt3 > 4.345 && tt3 < 4.40)) && tw++ < 120)
					logerror("TOGWR <- %04x pc=%06x @%.6f\n", data, m_cpu->pc(), tt3); });
			// TEMP cont.250 (STRIP): ledger writes with pc - stakes (F0) and converts (C0)
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7654, 0x7667, "ledgerwr",
				[this](offs_t offset, u16 &data, u16 mem_mask)
				{ static int lw = 0; double const lt3 = machine().time().as_double();
				if (storager_getenv("STORAGER_IAMRD") && lt3 > 3.5 && lt3 < 11.0 && lw++ < 300)
					{ address_space &lgs = m_cpu->space(AS_PROGRAM);
					logerror("LEDGWR %04x <- %04x mm=%04x pc=%06x | 7958=%04x%04x 7986=%04x 79b6=%04x 79ba=%04x 7968=%04x @%.6f\n",
						u32(offset) & 0xffff, data, mem_mask, m_cpu->pc(),
						lgs.read_word(0x7958), lgs.read_word(0x795a), lgs.read_word(0x7986),
						lgs.read_word(0x79b6), lgs.read_word(0x79ba), lgs.read_word(0x7968), lt3); } });
			// cont.261 (STORAGER_REARM, STRIP): the over-read driver - who sets [$7968]=1 /
			// advances [$7424] during 7.97-7.99 when [$796c]=0 and owed satisfied. Names the pc.
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7968, 0x7969, "rearm68",
				[this](offs_t, u16 &data, u16)
				{ static int r6 = 0; double const rt = machine().time().as_double();
				if (storager_getenv("STORAGER_REARM") && rt > 7.90 && rt < 8.05 && r6++ < 80)
					{ address_space &rs = m_cpu->space(AS_PROGRAM);
					logerror("REARM68 <- %04x pc=%06x | 7424=%04x 796c=%04x 7956=%04x 79a8=%04x 7428=%04x @%.6f\n",
						data, m_cpu->pc(), rs.read_word(0x7424), rs.read_word(0x796c), rs.read_word(0x7956),
						rs.read_word(0x79a8), rs.read_word(0x7428), rt); } });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7424, 0x7425, "rearm24",
				[this](offs_t, u16 &data, u16)
				{ static int r2 = 0; double const rt = machine().time().as_double();
				if (storager_getenv("STORAGER_REARM") && rt > 7.90 && rt < 8.05 && r2++ < 80)
					logerror("REARM24 <- %04x pc=%06x 7968=%04x 7428=%04x @%.6f\n",
						data, m_cpu->pc(), m_cpu->space(AS_PROGRAM).read_word(0x7968), m_cpu->space(AS_PROGRAM).read_word(0x7428), rt); });
			// cont.262 (STORAGER_FMVFY, STRIP): confirm the $9934 FM-IDAM verify matches +
			// loads the aim for all 8 FM sectors. Taps: $9934 fe-check, $98e0 cyl, $98f6 head,
			// $992e aim-store, and the reject/error exits ($9938/$98e6/$98fc).
			for (u32 vpc : { 0x9884u, 0x9934u, 0x9938u, 0x98e0u, 0x98e6u, 0x98f6u, 0x98fcu, 0x992eu, 0x92b4u })
			{
				m_cpu->space(AS_OPCODES).install_read_tap(vpc, vpc+1, "fmvfy",
					[this, vpc](offs_t, u16 &, u16)
					{ static int fv = 0; double const ft = machine().time().as_double();
					if (!storager_getenv("STORAGER_FMVFY") || ft < 5.5 || fv >= 300) return;
					fv++; address_space &vs = m_cpu->space(AS_PROGRAM);
					u16 const buf = vs.read_word(0x7a66);
					logerror("FMVFY pc=%04x d1=%04x | 7dac:%02x %02x %02x %02x %02x | buf[%04x]:%02x %02x %02x %02x %02x | tgt cyl=%04x head=%04x aim=%04x @%.6f\n",
						vpc, u16(m_cpu->state_int(M68K_D1)),
						vs.read_byte(0x7dac), vs.read_byte(0x7dad), vs.read_byte(0x7dae), vs.read_byte(0x7daf), vs.read_byte(0x7db0),
						buf, vs.read_byte(buf), vs.read_byte((buf+1)&0xffff), vs.read_byte((buf+2)&0xffff), vs.read_byte((buf+3)&0xffff), vs.read_byte((buf+4)&0xffff),
						vs.read_word(0x7438), vs.read_word(0x7436), vs.read_word(0x7428), ft); });
			}
			// cont.266 (STORAGER_XFER): the TRANSFER-ACTIVE flag [$79b6] is the crux (cont.141: never
			// set -> owed [$79a8] never decrements). Tap every write to [$79b6] to name the v2.60 arm
			// site, and the sector-done re-seed at $81d0 to see the decrement-vs-reseed branch context.
			// cont.273: the exit-enable is [$7968]=1 at $82b2, gated on [$7956]==0 && [$79ba]==0 &&
			// [$7958]==0 && [$741c]!=0 && [$796a]!=0 && [$727e]==0. Sample all six at each [$7956]==0
			// window so we see which condition blocks [$7968]=1 (the missing gate-array state).
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7968, 0x7969, "exitgate",
				[this](offs_t, u16 &data, u16)
				{ static int qn = 0; double const qt = machine().time().as_double();
					if (!storager_getenv("STORAGER_XFER") || qt < 7.9 || qn >= 30) return; qn++;
					address_space &xs = m_cpu->space(AS_PROGRAM);
					logerror("W7968 <-%04x pc=%06x | 7956=%04x 79ba=%04x 7958=%08x 741c=%04x 796a=%04x 727e=%04x @%.6f\n",
						data, m_cpu->pc(), xs.read_word(0x7956), xs.read_word(0x79ba), xs.read_dword(0x7958),
						xs.read_word(0x741c), xs.read_word(0x796a), xs.read_word(0x727e), qt); });
			// also: sample the six conditions periodically (poll) so we see them even when [$7968] isn't written
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7956, 0x7957, "cnt0",
				[this](offs_t, u16 &data, u16)
				{ static int cn = 0; double const ct = machine().time().as_double();
					if (!storager_getenv("STORAGER_XFER") || data != 0 || ct < 7.9 || cn >= 20) return; cn++;
					address_space &xs = m_cpu->space(AS_PROGRAM);
					logerror("CNT0 [7956]->0 pc=%06x | 79ba=%04x 7958=%08x 741c=%04x 796a=%04x 727e=%04x 7968=%04x @%.6f\n",
						m_cpu->pc(), xs.read_word(0x79ba), xs.read_dword(0x7958), xs.read_word(0x741c),
						xs.read_word(0x796a), xs.read_word(0x727e), xs.read_word(0x7968), ct); });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7428, 0x7429, "aim7428",
				[this](offs_t, u16 &data, u16)
				{ static int an = 0; double const at = machine().time().as_double();
					if (!storager_getenv("STORAGER_TR6") || at < 7.9 || an >= 80) return; an++;
					logerror("AIM7428 <-%04x pc=%06x 7956=%04x @%.6f\n", data, m_cpu->pc(),
						m_cpu->space(AS_PROGRAM).read_word(0x7956), at); });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x743a, 0x743b, "desc743a",
				[this](offs_t, u16 &data, u16)
				{ static int dn = 0; double const dt = machine().time().as_double();
					if (!storager_getenv("STORAGER_TR6") || dt < 7.9 || dn >= 40) return; dn++;
					logerror("W743A <-%04x pc=%06x 7424=%04x 7426=%04x @%.6f\n", data, m_cpu->pc(),
						m_cpu->space(AS_PROGRAM).read_word(0x7424), m_cpu->space(AS_PROGRAM).read_word(0x7426), dt); });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7426, 0x7427, "lat7426",
				[this](offs_t, u16 &data, u16)
				{ static int en = 0; double const et = machine().time().as_double();
					if (!storager_getenv("STORAGER_TR6") || et < 7.9 || en >= 40) return; en++;
					logerror("W7426 <-%04x pc=%06x 7956=%04x @%.6f\n", data, m_cpu->pc(),
						m_cpu->space(AS_PROGRAM).read_word(0x7956), et); });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7a0e, 0x7a0f, "x7a0e",
				[this](offs_t, u16 &data, u16)
				{ static int an = 0; double const at = machine().time().as_double();
					if (!storager_getenv("STORAGER_XFER") || an >= 60) return; an++;
					logerror("W7A0E <-%04x pc=%06x 7428=%04x @%.6f\n", data, m_cpu->pc(),
						m_cpu->space(AS_PROGRAM).read_word(0x7428), at); });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x79b6, 0x79b7, "xb6",
				[this](offs_t, u16 &data, u16)
				{ static int bn = 0; double const bt = machine().time().as_double();
					if (!storager_getenv("STORAGER_XFER") || bn >= 80) return; bn++;
					logerror("W79B6 <-%04x pc=%06x 79a8=%04x 742c=%04x 7426=%04x @%.6f\n", data, m_cpu->pc(),
						m_cpu->space(AS_PROGRAM).read_word(0x79a8), m_cpu->space(AS_PROGRAM).read_word(0x742c),
						m_cpu->space(AS_PROGRAM).read_word(0x7426), bt); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x81d0, 0x81d1, "reseed81d0",
				[this](offs_t, u16 &, u16)
				{ static int rn = 0; double const rt = machine().time().as_double();
					if (!storager_getenv("STORAGER_XFER") || rt < 7.9 || rn >= 40) return; rn++;
					address_space &xs = m_cpu->space(AS_PROGRAM);
					logerror("RESEED81D0 79b6=%04x 79b8=%04x 79b0=%04x 7426=%04x 742c=%04x d0=%04x a0=%06x @%.6f\n",
						xs.read_word(0x79b6), xs.read_word(0x79b8), xs.read_word(0x79b0), xs.read_word(0x7426),
						xs.read_word(0x742c), u16(m_cpu->state_int(M68K_D0)), u32(m_cpu->state_int(M68K_A0)), rt); });
			// cont.262 (STORAGER_C135PH): raise a DEFERRED data-record IRQ6 the instant the
			// pump opens the phase-1 window ([$7950] bit0 -> 1), so the toggler forks to the
			// $92b4 DATA staker instead of $89f2. Fires inside the pump handler; the fw takes
			// the held IRQ6 after it returns, with phase still 1.
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7950, 0x7951, "c135ph",
				[this](offs_t, u16 &data, u16)
				{ if (!storager_getenv("STORAGER_C135PH")) return;
					{ static int _dbg = 0; if (storager_getenv("STORAGER_PHASELOG") && _dbg++ < 30)
						logerror("C135PH-TAP data=%04x pending=%d @%.6f\n", data, m_c135_pending?1:0, machine().time().as_double()); }
					if (m_c135_pending && (data & 1))
					{ m_c135_pending = false;
					m_claim_field = ~0u; m_claim_time = machine().time();
					m_cpu->set_input_line(M68K_IRQ_6, HOLD_LINE);
					if (storager_getenv("STORAGER_PHASELOG")) { static int _pr = 0; if (_pr++ < 200)
						logerror("C135-RAISE (phase-1 window, deferred data IRQ6) @%.6f\n", machine().time().as_double()); } } });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x72f8, 0x7307, "vecguard",
				[this](offs_t offset, u16 &data, u16)
				{ static int vg = 0; if (storager_getenv("STORAGER_IAMRD") && machine().time().as_double() > 7.5 && vg++ < 60)
					logerror("VECWR %04x <- %04x pc=%06x @%.6f\n", u32(offset) & 0xffff, data, m_cpu->pc(), machine().time().as_double()); });
			// TEMP cont.246c (STRIP): the wind-down READ CENSUS - every unique (pc, addr)
			// data read in 9.80-9.90; names what the fw actually polls
			// cont.262b (STORAGER_STAKEV): is the DATA fork even REACHABLE? which toggler runs
		// ($298c alternator vs $299a always-ID), does $2996 (bra $92b4) execute, and where
		// does the IRQ6 vector [[$7304]] point?
		for (auto tp : { std::pair<u16,char const*>{0x298cu,"TOG-298C"}, {0x299au,"TOG-299A"}, {0x2996u,"TOG-BRA92B4"} })
		{
			u16 const a=tp.first; char const* nm=tp.second;
			m_cpu->space(AS_OPCODES).install_read_tap(a, a+1, nm,
				[this,nm](offs_t, u16 &, u16){ static std::map<char const*,int> c;
				double const tt = machine().time().as_double();
				if (!storager_getenv("STORAGER_STAKEV") || tt < 7.99 || tt > 8.06 || c[nm]++ >= 40) return;
				address_space &ts = m_cpu->space(AS_PROGRAM);
				logerror("%s 7950=%04x 742c=%04x [7304]=%04x%04x @%.6f\n", nm, ts.read_word(0x7950),
					ts.read_word(0x742c), ts.read_word(0x7304), ts.read_word(0x7306), tt); });
		}
		// cont.262 (STORAGER_STAKEV): does the DATA fork reach the f0 stake for aim=1?
		// $92b4 entry (tst $742c), $92f6 stake-path, $9312 the move.b #$f0,ledger[aim].
		for (auto kp : { std::pair<u16,char const*>{0x92b4u,"STK-92B4"}, {0x92f6u,"STK-92F6"}, {0x9312u,"STK-9312"} })
		{
			u16 const a=kp.first; char const* nm=kp.second;
			m_cpu->space(AS_OPCODES).install_read_tap(a, a+1, nm,
				[this,nm](offs_t, u16 &, u16){ static std::map<char const*,int> c;
				double const kt = machine().time().as_double();
				if (!storager_getenv("STORAGER_STAKEV") || kt < 7.99 || kt > 9.85 || c[nm]++ >= 60) return;
				address_space &ks = m_cpu->space(AS_PROGRAM);
				u16 const aim = ks.read_word(0x7428);
				logerror("%s aim=%04x 742c=%04x ledger[aim]=%02x 7950=%04x @%.6f\n", nm, aim,
					ks.read_word(0x742c), ks.read_byte((0x7654 + aim) & 0xffff), ks.read_word(0x7950), kt); });
		}
		m_cpu->space(AS_PROGRAM).install_read_tap(0x0000, 0xffff, "census",
				[this](offs_t offset, u16 &data, u16)
				{ static std::set<u64> seen; static int cn3 = 0;
				double const ct = machine().time().as_double();
				if (!storager_getenv("STORAGER_IAMRD") || ct < 9.80 || ct > 9.90 || cn3 >= 250) return;
				u64 const key = (u64(m_cpu->pc()) << 24) | (u32(offset) & 0xffffff);
				if (seen.insert(key).second)
					{ cn3++;
					logerror("CENSUS pc=%06x rd %06x = %04x @%.6f\n", m_cpu->pc(), u32(offset) & 0xffffff, data, ct); } });
			for (offs_t eb : { offs_t(0xe000), offs_t(0xffe000) })
				m_cpu->space(AS_PROGRAM).install_read_tap(eb, eb + 0x1f, "e000pull",
					[this](offs_t offset, u16 &data, u16)
					{ static int en2 = 0; static double lt2 = 0;
					double const et = machine().time().as_double();
					if (storager_getenv("STORAGER_IAMRD") && et > 9.75 && et - lt2 > 0.002 && en2++ < 80)
						{ lt2 = et;
						logerror("E000RD %06x pc=%06x d=%04x @%.6f\n", u32(offset) & 0xffffff, data, m_cpu->pc(), et); } });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7224, 0x722b, "walksel",
				[this](offs_t offset, u16 &data, u16)
				{ static int wn = 0; static u32 lasta = 0; static u16 lastd = 0xdead;
				double const wt = machine().time().as_double();
				u32 const ba = u32(offset) & 0xffff;
				if (wt > 7.5 && (ba != lasta || data != lastd) && wn++ < 400)
					{ lasta = ba; lastd = data;
					logerror("WALKSEL %04x <- %04x pc=%06x @%.6f\n", ba, data, m_cpu->pc(), wt); } });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7428, 0x7429, "w7428log",
				[this](offs_t, u16 &data, u16)
				{ static int wn = 0; static u16 last = 0xdead;
				if (data != last && wn++ < 400)
					{ last = data;
					logerror("W7428 <- %04x pc=%06x @%.6f\n", data, m_cpu->pc(), machine().time().as_double()); } });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x796e, 0x796f, "w796e",
				[this](offs_t, u16 &data, u16)
				{ static int wn = 0; static u16 last = 0xdead;
				if (data != last && wn++ < 300)
					{ last = data;
					logerror("W796E <- %04x pc=%06x @%.6f\n", data, m_cpu->pc(), machine().time().as_double()); } });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7abc, 0x7abd, "w7abc",
				[this](offs_t, u16 &data, u16)
				{ static int wn = 0; if (wn++ < 200)
					logerror("W7ABC <- %04x pc=%06x @%.6f\n", data, m_cpu->pc(), machine().time().as_double()); });
		}
		// TEMP cont.240 (STRIP): THE BLOCK-CYCLE CHAIN TAPS - the frontier doctrine's
		// link-by-link walk. Block 1 (R1-R4) kicks; block 2 (R5+) never does. Tap every
		// link of the stake->pop->launch->kick ladder with the live cycle cells.
		if (storager_getenv("STORAGER_CHAINTAP"))
		{
			static constexpr struct { u16 a; char const *n; } chainsites[] = {
				{ 0x26a8, "26a8" }, { 0x26ae, "26ae" }, { 0x92b4, "92b4-stamp" },
				{ 0x92f6, "92f6-STAKE" }, { 0x7fee, "7fee-KICK" }, { 0x8018, "8018-rearm" },
				{ 0x80c0, "80c0-cq" }, { 0x32ac, "32ac-pop" }, { 0x7ec4, "7ec4-cycdec" },
				{ 0x8214, "8214-postSM" }, { 0x3e8a, "3e8a-hostadr" }, { 0x933c, "933c-found" },
				{ 0x0bba, "0bba-LAUNCH" }, { 0x0cd4, "0cd4-lwait" }, { 0x3cd4, "3cd4-lrec" },
				{ 0x7ed8, "7ed8-clr741c" }, { 0x7ee0, "7ee0-pop-path" }, { 0x7f1a, "7f1a-exit" },
				{ 0x3dbc, "3dbc-SELFFEED" }, { 0x830e, "830e-no-F0" }, { 0x8318, "8318-F0-aim" },
				{ 0x8344, "8344-feedcall" }, { 0x7e58, "7e58-CMP" }, { 0x7e60, "7e60-MATCH" },
			};
			for (auto const &cs2 : chainsites)
				m_cpu->space(AS_OPCODES).install_read_tap(cs2.a, cs2.a | 1, cs2.n,
					[this, name = cs2.n](offs_t, u16 &, u16)
					{ static int cn = 0; if (cn++ < 900)
						{ address_space &xs = m_cpu->space(AS_PROGRAM);
						logerror("CHAIN %s 7956=%04x 742c=%04x 79b8=%04x 741c=%04x 7428=%04x c8000=%04x 796a=%04x 7958=%04x%04x 7abc=%04x @%.6f\n",
							name, xs.read_word(0x7956), xs.read_word(0x742c), xs.read_word(0x79b8),
							xs.read_word(0x741c), xs.read_word(0x7428), m_c800[0],
							xs.read_word(0x796a), xs.read_word(0x7958), xs.read_word(0x795a), xs.read_word(0x7abc),
							machine().time().as_double()); } });
		}
		m_cpu->space(AS_OPCODES).install_read_tap(0x5fc0, 0x5fc1, "read_count",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ >= 8) return; address_space &bs = m_bus->space(AS_PROGRAM); u32 const ia = m_iopb_addr & 0xffffff;
				logerror("READ-IOPB @%06x: %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x @%.5f\n", ia,
					bs.read_byte(ia),bs.read_byte(ia+1),bs.read_byte(ia+2),bs.read_byte(ia+3),bs.read_byte(ia+4),bs.read_byte(ia+5),bs.read_byte(ia+6),bs.read_byte(ia+7),
					bs.read_byte(ia+8),bs.read_byte(ia+9),bs.read_byte(ia+10),bs.read_byte(ia+11),bs.read_byte(ia+12),bs.read_byte(ia+13),bs.read_byte(ia+14),bs.read_byte(ia+15), machine().time().as_double()); });
		// task#5: with BUSY-HOLD the 0x95 read reaches CMD-DISPATCH -> $dd6 (handler table @$92). Two gates decide
		// $5fc0-vs-error: $de6 (D7=[$92+cmd*4]==0 -> DISP-ERR 0x14) and $df4 (node[4]>=8 -> ERR 0x11). Tap them.
		m_cpu->space(AS_OPCODES).install_read_tap(0x0dd6, 0x0dd7, "disp_dd6",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ >= 20) return; address_space &ds = m_cpu->space(AS_PROGRAM);
				u16 const a0 = u16(m_cpu->state_int(M68K_A0)); u16 const cmd = ds.read_byte(a0); u16 const idx = 0x92 + (cmd - 0x70) * 4;
				logerror("DISP-DD6 cmd=%02x tbl[$%04x]=%04x hdlr[$%04x]=%04x node[4]=%02x @%.5f\n",
					cmd, idx, ds.read_word(idx), idx+2, ds.read_word(idx+2), ds.read_byte((a0+4)&0xffff), machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x0e02, 0x0e03, "disp_e02",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ < 20) logerror("DISP-E02 (dispatch->handler) cmd=%02x @%.5f\n", m_iopb_cmd, machine().time().as_double()); });
		// task#5: THE DIVERGENCE. $f84 `tst.w D0; bne $1206` errors the read; D0 = return of $651c ($f56). $651c
		// reads drive status [$f000] + [$7944] + node[4] + [$799a+$12]. Tap $f5a (right after $651c returns) -> log
		// D0 (0=ok, !=0=err) + the inputs, for read(95) vs restore(89). Names why 0x95 errors where 0x89 succeeds.
		// task#5: confirm the read's error is $a118's step-wait on f000 bit1 (seek-active). $a1a2 = btst#1 check;
		// $a1ac = timeout->error 0x201e. Log f000 at the wait + count the timeouts. bit1 must TOGGLE per step-pulse.
		m_cpu->space(AS_OPCODES).install_read_tap(0x0a1a2, 0x0a1a3, "wait_a1a2",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ >= 12) return; logerror("A118-WAIT f000=%04x(b1=%d) cmd=%02x @%.5f\n",
				m_cpu->space(AS_PROGRAM).read_word(0xf000), (m_cpu->space(AS_PROGRAM).read_word(0xf000)>>1)&1, m_iopb_cmd, machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x0a1ac, 0x0a1ad, "to_a1ac",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ < 12) logerror("A118-TIMEOUT@a1ac (bit1 never SET) -> err 0x201e cmd=%02x @%.5f\n", m_iopb_cmd, machine().time().as_double()); });
		// which commands reach $a118, and do 0x89/0x98 (seek cmds) PASS it (contrast with the read's timeout)?
		m_cpu->space(AS_OPCODES).install_read_tap(0x0a118, 0x0a119, "a118_entry",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ < 20) logerror("A118-ENTRY cmd=%02x f000=%04x @%.5f\n", m_iopb_cmd, m_cpu->space(AS_PROGRAM).read_word(0xf000), machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x0a1b2, 0x0a1b3, "a118_step_ok",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ < 20) logerror("A118-STEP-OK (bit1 set, step registered) cmd=%02x @%.5f\n", m_iopb_cmd, machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x0f5a, 0x0f5b, "div_f5a",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ >= 20) return; address_space &ds = m_cpu->space(AS_PROGRAM);
				u16 const a1 = ds.read_word(0x799a);
				logerror("DIV-651c-ret D0=%04x | f000=%04x 7944=%04x node[4]=%02x [799a+12]=%02x cmd=%02x @%.5f\n",
					u16(m_cpu->state_int(M68K_D0)), ds.read_word(0xf000), ds.read_word(0x7944),
					ds.read_byte((ds.read_word(0x71bc)+4)&0xffff), ds.read_byte((a1+0x12)&0xffff), m_iopb_cmd, machine().time().as_double()); });
		// task#5: pinpoint the read's setup-time error. $1206 = error handler; D0 = error code (0x14/0x11/0x1091).
		// Log D0 + the return addr (error source) + cmd, and verify it's real (not prefetch) via the actual PC.
		m_cpu->space(AS_OPCODES).install_read_tap(0x1206, 0x1207, "err1206",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ >= 20) return; address_space &ds = m_cpu->space(AS_PROGRAM);
				u32 const sp = u32(m_cpu->state_int(M68K_SP)) & 0xffffff;
				logerror("ERR1206 D0(errcode)=%04x pc=%06x ret=%04x cmd=%02x @%.5f\n",
					u16(m_cpu->state_int(M68K_D0)), m_cpu->pc(), ds.read_word(sp), m_iopb_cmd, machine().time().as_double()); });
		// task#5: the node+26 2-vs-4 decision at $260c. D0 (=[$7e1e]&0xff) + [A3+$28] bit9 pick park(2) vs advance(4).
		// Log both inputs + [$7e1e] so we see WHY it parks. And who WRITES [$7e1e] (host dual-port vs anything).
		m_cpu->space(AS_OPCODES).install_read_tap(0x260c, 0x260d, "gate260c",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ >= 20) return; address_space &ds = m_cpu->space(AS_PROGRAM);
				u16 const a3 = u16(m_cpu->state_int(M68K_A3));
				logerror("GATE260c D0=%04x 7e1e=%04x A3=%04x [A3+28]=%04x(b9=%d) @%.5f\n",
					u16(m_cpu->state_int(M68K_D0)), ds.read_word(0x7e1e), a3, ds.read_word((a3 + 0x28) & 0xffff),
					(ds.read_word((a3 + 0x28) & 0xffff) >> 9) & 1, machine().time().as_double()); });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7e1e, 0x7e1f, "w_7e1e",
			[this](offs_t, u16 &data, u16){ static int n = 0; if (n++ < 20) logerror("7e1e<-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), machine().time().as_double()); });
		// task#5: cross-device tap - watch the CPUAP (:slot6:cpuap:cpu) READ the IOPB status in its host RAM to
		// NAME the BUSY/status bit it waits on between commands. IOPB at 0xfe780; status at +2/+3 (0x81 BUSY/0x80 DONE).
		if (storager_getenv("STORAGER_CPUAP_POLL"))
		{
			device_t *const d = machine().root_device().subdevice("slot6:cpuap:cpu");
			device_memory_interface *const mi = dynamic_cast<device_memory_interface *>(d);
			if (mi)
			{
				mi->space(AS_PROGRAM).install_read_tap(0x0fe780, 0x0fe78f, "cpuap_iopb_rd",
					[this](offs_t off, u16 &data, u16 mem_mask){ static int n = 0; double const t = machine().time().as_double();
						if (t > 6.39 && t < 6.46 && n++ < 250) logerror("CPUAP-RD iopb+%02x -> %04x (mask %04x) @%.6f\n", off - 0x0fe780, data, mem_mask, t); });
				logerror("STORAGER_CPUAP_POLL: tap installed on CPUAP 0xfe780-8f\n");
			}
			else logerror("STORAGER_CPUAP_POLL: CPUAP device NOT FOUND (d=%p)\n", (void*)d);
		}
		// task#5 path-2: the data-phase op ($9398/$9984) runs ONLY when its node's node+26 == 0xa ($15fe executor
		// gate $161c/$162a). Statically node+26 is set to 2/4/6/8/c, NEVER 0xa. MEASURE the ladder: does ch0/ch1
		// node+26 ([$71ec]/[$7216]) ever reach 0xa? If never, the data phase is structurally unreachable = the root.
		for (u32 na : { 0x71ecu, 0x7216u })
			m_cpu->space(AS_PROGRAM).install_write_tap(na, na | 1, "nodep26",
				[this, na](offs_t, u16 &data, u16){ static int n = 0; if (n++ >= 80) return;
					logerror("node+26[%04x]<-%04x pc=%06x @%.5f\n", na, data, m_cpu->pc(), machine().time().as_double()); });
		// task#5 fork-decider: $15a0 sets node+26 = D1, where D1=0xa (DATA PHASE) iff the microseq step [A1]==0x36,
		// D1=0xc (done) iff [A1]==0. Tap $158e (after D0=[A1] loaded) -> is 0x36 in the read's list, or is A1 past
		// it (reading 0)? Dump [$721a], A1, [A1], and the list around A1 to see the microseq the walk actually sees.
		// task#5 construction: does the $3900/$39xx builder (which writes the 0x36 data step at $3956) run for the
		// read? Tap the entry $3900 (+[$793e]/[$793c]/node[$20]), the selector $3920, and the 0x36 write $3956.
		m_cpu->space(AS_OPCODES).install_read_tap(0x3900, 0x3901, "build3900",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ >= 20) return; address_space &ds = m_cpu->space(AS_PROGRAM);
				u16 const a0 = u16(m_cpu->state_int(M68K_A0));
				logerror("BUILD3900 793e=%04x 793c=%04x [799a]=%04x node[20]=%04x(b4=%d) cmd=%02x @%.5f\n",
					ds.read_word(0x793e), ds.read_word(0x793c), a0, ds.read_word((a0+0x20)&0xffff),
					(ds.read_word((a0+0x20)&0xffff)>>4)&1, m_iopb_cmd, machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x3956, 0x3957, "step36",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ < 20) logerror("STEP36 (0x36 data-step BUILT @3956) cmd=%02x @%.5f\n", m_iopb_cmd, machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x3798, 0x3799, "step36b",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ < 20) logerror("STEP36b (0x36 built @3798, the $37xx builder) cmd=%02x @%.5f\n", m_iopb_cmd, machine().time().as_double()); });
		// who populates the read's walked microseq list at $7250 ([$721a]=$7224 -> $7250)?
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7250, 0x7257, "list7250",
			[this](offs_t offset, u16 &data, u16){ static int n = 0; if (n++ >= 40) return;
				logerror("LIST7250[%04x]<-%04x pc=%06x cmd=%02x @%.5f\n", 0x7250 + offset*2, data, m_cpu->pc(), m_iopb_cmd, machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x158e, 0x158f, "microseq",
			[this](offs_t, u16 &, u16){ static int n = 0; if (n++ >= 30) return; address_space &ds = m_cpu->space(AS_PROGRAM);
				u16 const a1 = u16(m_cpu->state_int(M68K_A1)), a2 = u16(m_cpu->state_int(M68K_A2));
				logerror("MICROSEQ D0=[A1]=%04x A1=%04x A2(node)=%04x 721a=%04x list[A1..+8]=%04x %04x %04x %04x %04x @%.5f\n",
					u16(m_cpu->state_int(M68K_D0)), a1, a2, ds.read_word(0x721a),
					ds.read_word(a1), ds.read_word((a1+2)&0xffff), ds.read_word((a1+4)&0xffff),
					ds.read_word((a1+6)&0xffff), ds.read_word((a1+8)&0xffff), machine().time().as_double()); });
		// TEMP (STRIP): EXEC-stall measurement (build#5 cont.5). $156a = the microseq WALK entry (no
		// literal callers in the ROM text -> computed entry); log the LIVE stack return addr to NAME
		// the caller + the gate state: [$71b2] (the walk's continue-gate at $15ec), [$71b6], the
		// cursor cell [$721a] and its pointee (which list + position), and the node states. Prefetch
		// caveat: $156a follows $1568 rts - identify real entries by a repeated plausible ROM return
		// addr on (A7) across hits.
		m_cpu->space(AS_OPCODES).install_read_tap(0x156a, 0x156b, "walkentry",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n++ >= 40) return; address_space &xs = m_cpu->space(AS_PROGRAM);
				u32 const sp = u32(m_cpu->state_int(M68K_SP));
				u16 const cur = xs.read_word(0x721a), lp = xs.read_word(cur);
				logerror("WALKENTRY ret=[%04x %04x] 721a=%04x ->[%04x]=%04x %04x %04x | 71b2=%04x 71b6=%04x st71f0=%04x st71c6=%04x @%.5f\n",
					xs.read_word(sp & 0xffff), xs.read_word((sp + 2) & 0xffff), cur, lp,
					xs.read_word(lp), xs.read_word((lp + 2) & 0xffff), xs.read_word((lp + 4) & 0xffff),
					xs.read_word(0x71b2), xs.read_word(0x71b6), xs.read_word(0x7216), xs.read_word(0x71ec),
					machine().time().as_double()); });
		// TEMP (STRIP): the $3bfe IRQ4-walker path-namer (build#5 cont.5). $3bfe head: A0=[$743a]; if
		// A0==[$7a14] (pending-dispatch marker) -> $1310/$1348 dispatch; else $3c32.. -> $3c6e: if
		// continuation [$A0+$14].l==0 -> $3c76 clr [$A0+$12] (BUSY CLEAR = the wanted completion!).
		// Run14's IRQ4 pass did NOT clear $749c - log entry state + whether $3c76 is reached.
		m_cpu->space(AS_OPCODES).install_read_tap(0x3bfe, 0x3bff, "wkr_entry",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n++ >= 40) return; address_space &xs = m_cpu->space(AS_PROGRAM);
				logerror("WKR-ENTRY 743a=%04x 7a14=%04x 7a76=%04x 7a18=%04x 749c=%04x @%.5f\n",
					xs.read_word(0x743a), xs.read_word(0x7a14), xs.read_word(0x7a76), xs.read_word(0x7a18),
					xs.read_word(0x749c), machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x3c76, 0x3c77, "wkr_clr",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n++ >= 40) return; address_space &xs = m_cpu->space(AS_PROGRAM);
				logerror("WKR-CLR12 (busy-clear path) A0=%04x 749c=%04x @%.5f\n",
					u16(m_cpu->state_int(M68K_A0)), xs.read_word(0x749c), machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x3c6e, 0x3c6f, "wkr_cont",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n++ >= 40) return; address_space &xs = m_cpu->space(AS_PROGRAM);
				u16 const a0 = u16(m_cpu->state_int(M68K_A0));
				logerror("WKR-CONT A0=%04x [A0+14].l=%04x%04x @%.5f\n", a0,
					xs.read_word((a0 + 0x14) & 0xffff), xs.read_word((a0 + 0x16) & 0xffff), machine().time().as_double()); });
		// task#5 Q1: WHO owns the stuck $369a seek? 5 entries jump to $367a: $1e2e (the read's channel-op Xdisp
		// flow) vs $9c90/$9d6a/$a0f4 (explicit 0x89/0x98 seek handlers, each clr $71b2 first). Tap each entry +
		// log cmd/node-cmd/timestamp; the one firing last before the t=7.06 spin owns it.
		for (u32 ea : { 0x1e2eu, 0x9c90u, 0x9d6au, 0xa0f4u, 0x07dau })
			m_cpu->space(AS_OPCODES).install_read_tap(ea, ea | 1, "seekentry",
				[this, ea](offs_t, u16 &, u16)
				{ static int n = 0; if (n++ >= 60) return; address_space &ps = m_cpu->space(AS_PROGRAM);
					logerror("SEEKENTRY via $%04x cmd=%02x node[71f0]=%04x 71b2=%04x @%.5f\n",
						ea, m_iopb_cmd, ps.read_word(0x71f0), ps.read_word(0x71b2), machine().time().as_double()); });
	// task#5 (STRIP): PC-histogram, TIME-triggered (STORAGER_PCHIST_AT seconds, default 9.0) so it catches the
	// terminal loop wherever the flow lands - not tied to a $1c8c Xdisp that may not fire in the new stuck state.
	// Prefetch-immune PC-register sampler, 100ms window -> the terminal loop self-declares, label-free.
	if (storager_getenv("STORAGER_PCHIST"))
	{
		double const at = storager_getenv("STORAGER_PCHIST_AT") ? atof(storager_getenv("STORAGER_PCHIST_AT")) : 9.0;
		m_pcsamp_on = true; m_pcsamp_done = true;
		m_pcsamp->adjust(attotime::from_double(at));
	}
		// task#5 ROUTER: $cd4 gates command-dispatch vs channel-service on [$749c] (beq $d06 if idle, else chan-svc).
		// Confirm the read's routing: does $cd4 ever run with [$749c]=0 AND $71f0[0]=0x95 (=> read dispatches to $5FC0)?
		m_cpu->space(AS_OPCODES).install_read_tap(0x0cd4, 0x0cd5, "router_cd4",
			[this](offs_t, u16 &, u16)
			{ address_space &ps = m_cpu->space(AS_PROGRAM);
				u16 const b = ps.read_word(0x749c);
				double const t = machine().time().as_double();
				// window measurement (build#5 cont.6): log EVERY idle-pass (rare, the decisive ones)
				// + everything inside one retry-cleanup window (9.00-9.05) - does the main loop reach
				// the router at 749c==0 with the 95 pending, and if so where does it go?
				static int _n = 0, _iw = 0;
				bool const inwin = (t >= 9.00 && t <= 9.05);
				if (b == 0 || inwin) { if (_iw++ >= 400) return; }
				else if (_n++ >= 80) return;
				logerror("ROUTER-cd4 749c=%04x %s 71f0[0]=%02x cmd=%02x @%.5f\n", b, b ? "BUSY->chan-svc" : "IDLE->$d06/$5FC0", ps.read_byte(0x71f0), m_iopb_cmd, t); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x1c8c, 0x1c8d, "xdisp_caller",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n++ >= 30) return; address_space &ps = m_cpu->space(AS_PROGRAM);
				address_space &bs = m_bus->space(AS_PROGRAM);
					// task#5: $9400 tests [A6+$20] bit14 where A6=[$799a] (NOT the $71f0 node). Dump the [$799a]-based
					// transfer flag AND the raw HOST IOPB the CPUAP programmed (m_iopb_addr). Decides Dave's fork:
					// host delivered a read-descriptor field we dropped, vs a field the fw must compute (hardware).
					u16 const a6 = ps.read_word(0x799a);
					u32 const ia = m_iopb_addr & 0xffffff;
					logerror("XDISP a6=[799a]=%04x a6[20]=%04x(b14=%d) | HOST-IOPB @%06x:"
						" %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x cmd=%02x @%.5f\n",
						a6, ps.read_word((a6 + 0x20) & 0xffff), (ps.read_word((a6 + 0x20) & 0xffff) >> 14) & 1, ia,
						bs.read_byte(ia), bs.read_byte(ia+1), bs.read_byte(ia+2), bs.read_byte(ia+3),
						bs.read_byte(ia+4), bs.read_byte(ia+5), bs.read_byte(ia+6), bs.read_byte(ia+7),
						bs.read_byte(ia+8), bs.read_byte(ia+9), bs.read_byte(ia+10), bs.read_byte(ia+11),
						bs.read_byte(ia+12), bs.read_byte(ia+13), bs.read_byte(ia+14), bs.read_byte(ia+15),
						m_iopb_cmd, machine().time().as_double()); });
	// task#4 writer-trace: who writes the ch1 mailbox [$7ff8]?  HOST writes are tagged in host_win_w; anything
	// here with a fw pc (not a host write) is the FIRMWARE.  Decides: is [$7ff8] host-only (=> seek-complete
	// must raise a channel IRQ, fw updates the mailbox) or does a channel/gate path write it (=> that value).
	// cont.210 (STORAGER_MBSPLIT): the dbi doorbell cells' IN/OUT latch split, fw side. The fw
	// at $36A2 posts {01,00,00,00} into a channel's mailbox block as an OUTBOUND host attention
	// doorbell; with shared SRAM the fw's own walk reads it back and claims a phantom inbound
	// command (empty node, [$71B2] leak -> the 5th-cmd death-spin). Real HW: separate latch
	// banks. fw READS of mb[0] ($7FF0/$7FF8 even lane) see the INBOUND latch (host's R0);
	// fw WRITES with bit0 clear = the doorbell ack (clears the in-latch); bit0 set = outbound
	// post, SRAM only. R1-R3 stay shared (host deposit order is protected by MBLATCH).
	if (storager_getenv("STORAGER_MBSPLIT"))
	{
		m_cpu->space(AS_PROGRAM).install_read_tap(0x7ff0, 0x7ff1, "mb0r_ch0",
			[this](offs_t, u16 &data, u16 mem_mask)
			{ if (mem_mask & 0xff00) data = (data & 0x00ff) | (u16(m_mb_in[0]) << 8); });
		m_cpu->space(AS_PROGRAM).install_read_tap(0x7ff8, 0x7ff9, "mb0r_ch1",
			[this](offs_t, u16 &data, u16 mem_mask)
			{ if (mem_mask & 0xff00) data = (data & 0x00ff) | (u16(m_mb_in[4]) << 8); });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7ff0, 0x7ff1, "mb0w_ch0",
			[this](offs_t, u16 &data, u16 mem_mask)
			{ if (!m_mb_depositing && (mem_mask & 0xff00) && !(data & 0x0100)) m_mb_in[0] = data >> 8; });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7ff8, 0x7ff9, "mb0w_ch1",
			[this](offs_t, u16 &data, u16 mem_mask)
			{ if (!m_mb_depositing && (mem_mask & 0xff00) && !(data & 0x0100)) m_mb_in[4] = data >> 8; });
	}
	// cont.214 (STRIP): the staged host-pair cells $7446/$7448 - who writes them, when, with
	// what B (the +0x100 continuation bug: the pair advanced only 2/8 during read-A).
	if (storager_getenv("STORAGER_PHASELOG"))
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7446, 0x7449, "pairw",
			[this](offs_t off, u16 &data, u16 mem_mask)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 3.0 && _n++ < 120)
				{ address_space &qs = m_cpu->space(AS_PROGRAM);
					u16 const hi = (off <= 0x7447 && (mem_mask & 0xff00)) ? (data & 0xffff) : qs.read_word(0x7446);
					u16 const lo = qs.read_word(0x7448);
					logerror("PAIRW [%04x]<-%04x mm=%04x pc=%06x B={%04x:%04x -> %06x} @%.6f\n",
						off, data, mem_mask, m_cpu->pc(), hi, lo,
						((u32(u8(~(hi >> 1))) << 16) | (u16(~lo) & 0xffff)) & 0xffffff, t); } });
	// cont.224 (STORAGER_JIT): the aim-write warp trigger - when the fw sets [$7428], jump
	// the rotational clock so the aimed sector's ID mark is ~30 bytes ahead. "The head
	// happened to be exactly positioned." All delivery machinery runs unchanged.
	// cont.233 (STRIP): [$7956] decrement tape - kicks vs decrements vs stakes, the decisive count
	if (storager_getenv("STORAGER_PHASELOG"))
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7956, 0x7957, "c7956",
			[this](offs_t, u16 &data, u16)
			{ static int _c9 = 0; double const t = machine().time().as_double();
				if (((t > 3.10 && t < 3.45) || (t > 8.15 && t < 8.45)) && _c9++ < 40)
				logerror("C7956 <-%04x pc=%06x @%.6f\n", data, m_cpu->pc(), t); });
	// cont.227 (STRIP): the four $74C4 chunk-queue slots' status lifecycle - where the batch-2
	// recycle stops after truck 4.
	if (storager_getenv("STORAGER_PHASELOG"))
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7544, 0x75e3, "cqslot",
			[this](offs_t off, u16 &data, u16 mem_mask)
			{ static int _cq = 0; double const t = machine().time().as_double();
				if (((t > 3.13 && t < 3.47) || (t > 8.18 && t < 8.47)) && _cq++ < 150)
				logerror("CQSLOT [%04x]<-%04x mm=%04x pc=%06x @%.6f\n", off, data, mem_mask, m_cpu->pc(), t); });
	// cont.212 (STRIP): the fw's UIB field-consumption map - which local UIB offsets are READ
	// (pc-attributed) after a fetch; names the zone-base field the fw uses (or ignores).
	if (storager_getenv("STORAGER_PHASELOG"))
		m_cpu->space(AS_PROGRAM).install_read_tap(0x6e60, 0x6e7f, "uibrd",
			[this](offs_t off, u16 &data, u16 mem_mask)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (((t > 8.39 && t < 8.99) || (t > 3.39 && t < 3.99)) && _n++ < 400)
				logerror("UIBRD [%04x]=%04x mm=%04x pc=%06x @%.6f\n", off, data, mem_mask, m_cpu->pc(), t); });
	// cont.210 (STRIP): read2's expected head/cyl - who writes [$7436]/[$7438] and when,
	// against the ROT-START snoop (the 0x202A head-mismatch ordering question)
	if (storager_getenv("STORAGER_PHASELOG"))
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7436, 0x7439, "wanttrk",
			[this](offs_t off, u16 &data, u16 mem_mask)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 3.0 && _n++ < 60)
				logerror("WANTTRK [%04x]<-%04x mm=%04x pc=%06x @%.6f\n", off, data, mem_mask, m_cpu->pc(), t); });
	// cont.210 (STRIP): the ch0 mailbox block's writers - the phantom ch0 claim's trigger hunt
	if (storager_getenv("STORAGER_PHASELOG"))
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7ff0, 0x7ff7, "ch0mbox_w",
			[this](offs_t off, u16 &data, u16 mem_mask)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 6.3 && _n++ < 80)
				logerror("MB0WR [%04x]<-%04x mm=%04x pc=%06x @%.6f\n", off, data, mem_mask, m_cpu->pc(), t); });
	if (storager_getenv("STORAGER_PHASELOG"))
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7ff8, 0x7ff9, "ch1mbox_w",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; if (_n++ >= 120) return;
				logerror("7ff8<-%04x pc=%06x cmd=%02x @%.5f\n", data, m_cpu->pc(), m_iopb_cmd, machine().time().as_double()); });
	// task#4: who writes the channel-complete status bytes [$7ffa/c/e]?  All-0xff routes the $24ea walk to
	// $256c ([$7b1a] setter = the seek/channel release path).  Is 0xff ever set by a gate-array/channel path
	// (=> that's the faithful seek-complete signal) or only host IOPB-pointer data?
	if (storager_getenv("STORAGER_PHASELOG"))
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7ffa, 0x7fff, "chstat_w",
			[this](offs_t off, u16 &data, u16)
			{ static int _n = 0; if (_n++ >= 200) return;
				logerror("chstat[%04x]<-%04x pc=%06x @%.5f\n", off, data, m_cpu->pc(), machine().time().as_double()); });
	// firmware-driven mode is the faithful (and now only) mode: the 68000 firmware serves the
	// ROM's command channel itself (HD identify 0x87 + HD reads 0x95), which the hard-disk boot
	// requires; the older delivery/status shims remain compiled but disabled
	// TEMP (STRIP): STORAGER_HLE95=1 re-enables the doorbell-HLE delivery for A/B against the
	// label-check failure (cont.191: the monitor rejects the LLE-delivered label the HLE-era
	// boot accepted - diff the check-era state).
	m_fw_driven = !storager_getenv("STORAGER_HLE95");
	// Live floppy swap for the multi-disk install: when the user mounts/unmounts an image via the UI
	// (Tab -> File Manager), invalidate the cached sector map so the next disk access reloads it.
	for (auto &conn : m_floppy)
		if (floppy_image_device *const fdd = conn->get_device())
		{
			fdd->setup_load_cb(floppy_image_device::load_cb(&multibus_storager_device::floppy_swap_cb, this));
			fdd->setup_unload_cb(floppy_image_device::unload_cb(&multibus_storager_device::floppy_swap_cb, this));
		}
	if (storager_getenv("STORAGER_TRACE")) m_trace = true;   // TEMP (STRIP): channel trace without fw-driven mode
	// TEMP (STRIP): who rewrites the super-buffer page's PTE2 @0x57360 (DMA side)
	m_bus->space(AS_PROGRAM).install_write_tap(0x57360, 0x57363, "ptewatch_bus",
		[this](offs_t offset, u16 &data, u16 mem_mask)
		{
			static int n = 0;
			if (n++ < 30) logerror("PTEWR-BUS @%06x = %04x/%04x [%s] @%.4f\n", offset, data, mem_mask, machine().describe_context(), machine().time().as_double());
		});
	// TEMP (STRIP): who clobbers the root super's magic word @0x3B15C (DMA side)
	m_bus->space(AS_PROGRAM).install_write_tap(0x3b158, 0x3b163, "magicwatch",
		[this](offs_t offset, u16 &data, u16 mem_mask)
		{
			static int n = 0;
			if (n++ < 20) logerror("MAGICWR-BUS @%06x = %04x/%04x [%s] @%.4f\n", offset, data, mem_mask, machine().describe_context(), machine().time().as_double());
		});
	// TEMP (STRIP): capture the fw's 0x87 status reply (the ROM's HD-boot decision input)
	m_bus->space(AS_PROGRAM).install_write_tap(0xfe948, 0xfe95f, "reply87",
		[this](offs_t offset, u16 &data, u16 mem_mask)
		{
			static int n = 0;
			if (n++ < 24) logerror("REPLY87 [%02x] = %04x/%04x @%.4f\n", unsigned(offset - 0xfe948), data, mem_mask, machine().time().as_double());
		});
	// TEMP (STRIP): the ID parser @0x9884 reads its buffer pointer from $7a66 - does it point at my
	// 0x4000 fill? + what are the targets ($7438 cyl / $7436 head) vs my buffer's bytes?
	// VGC7219 gate-array PUMP: the firmware arms the pump by writing its soft-vector $7304 with the
	// per-command handler (0x9884 the read parser); the gate array then raises IRQ6 to run it. Model
	// that - on a NON-ZERO $7304 write, arm the pump timer to fire IRQ6 shortly after (the real gate
	// array paces it once there is queued work). ($7304 has no init value, so gate strictly on the
	// arm write - never fire IRQ6 through a stale/zero vector.)
	// LLE PUMP ARM: the firmware queues a disk op at $328e (move.l $7940,($72ec,$7942)); the VGC7219
	// then raises IRQ6 to run it -> $26ae trampoline -> *($7304)=$9884 read parser -> $5e98 (E800 arm,
	// disk->SRAM) -> $13ce SRAM->host DMA.  Model: on that op-table store, arm the pump to fire IRQ6.
	// (Gated on STORAGER_NOBYPASS during the cutover so it doesn't double-fire with the cmd-0x95 HLE.)
	if (m_fw_driven)
		m_cpu->space(AS_PROGRAM).install_write_tap(0x72ec, 0x76ff, "opqueue_irq6",
			[this](offs_t offset, u16 &data, u16)
			{
				if (!storager_getenv("STORAGER_NOBYPASS")) return;
				// task#5: flag exactly which op-table slot/pc stores $28e0 (the step the executor later jmps to).
				if (data == 0x28e0) { static int _e = 0; if (_e++ < 20) logerror("OPTBL<-28e0 addr=%04x pc=%06x @%.5f\n", 0x72ec + offset * 2, m_cpu->pc(), machine().time().as_double()); }
				u32 const pc = m_cpu->pc();
				if (pc >= 0x328e && pc <= 0x32aa)   // the $328e read op-queue store -> arm IRQ6
				{
					static int _pn = 0;
					if (_pn++ < 60) logerror("OPQUEUE->IRQ6 addr=%04x pc=%06x @%.4f\n", 0x72ec + offset * 2, pc, machine().time().as_double());
					// task#5 reversible experiment: does the pump hijack the channel before the fw runs its own
					// read-setup ($9400/$5fc0)? STORAGER_NOPUMP suppresses the arm to see if $9400 then runs.
					if (storager_getenv("STORAGER_NOPUMP")) return;
					m_pump->adjust(attotime::from_usec(30));
				}
				else if (machine().time().as_double() > 1.0)   // else: find whatever else writes the op table
				{ static int _on = 0; if (_on++ < 30) logerror("OPTBL wr addr=%04x pc=%06x @%.4f\n", 0x72ec + offset * 2, pc, machine().time().as_double()); }
			});
	// TEMP (STRIP): STORAGER_PHASELOG - the runtime read-phase map (Dave).  Opcode-fetch taps at the read
	// milestones + D800/E000/E802 arm events -> one chronological "PHASE" stream.  Run HLE first (ground
	// truth), then diff NOBYPASS.  Answers: first D800=$7dac time, verify-vs-transfer order, which IRQ,
	// which arm.  Fetches route through AS_OPCODES (the fw runs from ROM at 0x4000+).
	// task#5: capture $28e0's CALLER — the executor branch that routes here. If jsr'd, [A7] is the return
	// addr = the diverging branch Dave wants. Dump the $7940 microsequence state + $7950 toggle too.
	if (storager_getenv("STORAGER_PHASELOG"))
		m_cpu->space(AS_OPCODES).install_read_tap(0x28e0, 0x28e1, "rdstep_caller",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n++ >= 20) return; address_space &ps = m_cpu->space(AS_PROGRAM);
				u32 const sp = u32(m_cpu->state_int(M68K_SP)) & 0xffffff;
				logerror("RDSTEP-CALLER stack=[%04x %04x %04x %04x] 7940=%04x 7942=%04x 7950=%04x 7434=%04x cmd=%02x @%.5f\n",
					ps.read_word(sp), ps.read_word((sp+2)&0xffffff), ps.read_word((sp+4)&0xffffff), ps.read_word((sp+6)&0xffffff),
					ps.read_word(0x7940), ps.read_word(0x7942), ps.read_word(0x7950), ps.read_word(0x7434),
					m_iopb_cmd, machine().time().as_double()); });
	if (storager_getenv("STORAGER_PHASELOG"))
	{
		static const struct { u32 addr; char const *name; } pts[] = {
			{ 0x5fc0, "READ-START" }, { 0x60d6, "SET7434" },    { 0x89f2, "IDCHK-sync" },
			{ 0x9884, "IDPUMP-irq6" },{ 0x9984, "DATASTEP" },   { 0x3bfe, "IRQ4-entry" },
			{ 0x26ae, "IRQ6-entry" }, { 0x369a, "SEEKWAIT" },   { 0x28e0, "RDSTEP-op0" },
			{ 0x290c, "RDSTEP-op1" }, { 0x2928, "RDSTEP-op2" }, { 0x92b4, "OPH1($92b4)" },
			{ 0x7ba8, "OPH2($7ba8)" },{ 0x1f82, "CHANSVC" },
			{ 0x0d54, "CMD-DISPATCH" }, { 0x5e64, "H87-ident" }, { 0x5f74, "H89-restore" },
			{ 0x0bf2, "INTAKE($0bf2)" }, { 0x24aa, "IRQ2-entry" },
			{ 0x24fe, "CH-PROCESS" }, { 0x2698, "CH-SKIP2698" }, { 0x269c, "CH-SKIP269c" },
			{ 0x0cd4, "PICKUP($cd4)" }, { 0x3c02, "IRQ4-RAN" }, { 0x0d06, "DISP-PRE" },
			{ 0x260c, "GATE260c" }, { 0x1d66, "Xdecr1d66" }, { 0x1c8c, "Xdisp1c8c" },
			{ 0x31f0, "EXEC31f0" }, { 0x24ae, "STEP24ae" }, { 0x352e, "Q352e" },
			{ 0x0bf6, "DONE-POST" }, { 0x0d60, "DISP-ERR" }, { 0x1206, "ERR1206" },
		};
		for (auto const &p : pts)
		{
			char const *const nm = p.name;
			m_cpu->space(AS_OPCODES).install_read_tap(p.addr, p.addr | 1, "phaselog",
				[this, nm](offs_t, u16 &, u16)
				{
					static int n = 0, nseek = 0;
					if (!strcmp(nm, "SEEKWAIT")) { if (nseek++ >= 30) return; }   // cap the spin separately so it can't starve the other taps' budget
					else if (n++ >= 12000) return;
					address_space &ds = m_cpu->space(AS_PROGRAM);
					logerror("PHASE %-11s D800<<1=%04x 7434=%04x 7dac[0:7]=%02x%02x%02x%02x%02x%02x%02x%02x cmd=%02x @%.5f\n",
							nm, u32(m_d800) << 1, ds.read_word(0x7434),
							ds.read_byte(0x7dac), ds.read_byte(0x7dad), ds.read_byte(0x7dae), ds.read_byte(0x7daf),
							ds.read_byte(0x7db0), ds.read_byte(0x7db1), ds.read_byte(0x7db2), ds.read_byte(0x7db3),
							m_iopb_cmd, machine().time().as_double());
					if (!strcmp(nm, "SEEKWAIT"))  // measure the CPU interrupt mask during the spin - is IRQ2 (level 2) blocked?
						logerror("       SR=%04x mask=%u | 71b2=%04x 7b1a=%04x\n",
							u16(m_cpu->state_int(M68K_SR)), (u16(m_cpu->state_int(M68K_SR)) >> 8) & 7,
							ds.read_word(0x71b2), ds.read_word(0x7b1a));
					if (nm[0]=='I'||nm[0]=='C'||nm[0]=='P')  // IRQ2/INTAKE/CH-*/PICKUP : channel-busy flag + seq state + the $251e bump-vs-skip selectors ($24fe: node+26==0 & [$71b2]!=2 & [$7ffa]!=0xff -> bump $260c)
						logerror("       749c=%04x | ch1[7ff8]=%02x seq[720a]=%02x | SEL node+26[7216]=%04x 71b2=%04x 7ffa=%02x 7ffc=%02x | 71f0cmd=%02x D000=%04x(->%06x)\n",
							ds.read_word(0x749c), ds.read_byte(0x7ff8), ds.read_byte(0x720a),
							ds.read_word(0x7216), ds.read_word(0x71b2), ds.read_byte(0x7ffa), ds.read_byte(0x7ffc),
							ds.read_byte(0x71f0), m_d000, u32(m_d000) << 1);
					if (nm[0]=='D' && nm[1]=='I')  // DISP-PRE ($d06): the dispatch selection + the restore-retry status words
					{
						u16 const a0 = u16(m_cpu->state_int(M68K_A0));
						u16 const dsr = u16(m_cpu->state_int(M68K_SR));
						logerror("       DISPATCH-SEL A0=%04x cmd(A0[0])=%02x ipl=%u(%s) 749c=%04x CCB($26,A0)=%04x | E01E=%04x F000=%04x seekdone=%d\n",
							a0, ds.read_byte(a0), (dsr >> 8) & 7, ((dsr >> 8) & 7) >= 2 ? "INTR" : "main",
							ds.read_word(0x749c), ds.read_word((a0 + 0x26) & 0xffff),
							m_ch[(0xe01e - 0xe000) / 2], m_ch[(0xf000 - 0xe000) / 2], machine().time() >= m_seek_deadline ? 1 : 0);
					}
					if (nm[0]=='E' && nm[1]=='R')  // ERR1206 ($1206): post-handler epilogue - D0 is the handler RESULT (0=success, non-0=error code posted to IOPB)
						logerror("       HANDLER-RESULT D0=%04x cmd(A0[0])=%02x (0=OK; nonzero posted to IOPB result)\n",
							u16(m_cpu->state_int(M68K_D0)), ds.read_byte(u16(m_cpu->state_int(M68K_A0))));
					if (nm[0]=='Q')  // Q352e (scan-op insert): which node is queued, for which cmd
						logerror("       QUEUE-INSERT D4(node)=%04x A0=%04x A1(head)=%04x cmd=%02x\n",
							u16(m_cpu->state_int(M68K_D4)), u16(m_cpu->state_int(M68K_A0)), u16(m_cpu->state_int(M68K_A1)), m_iopb_cmd);
					if (nm[0]=='G')  // GATE260c (working bump): node A0 + ring
					{
						u16 const a0 = u16(m_cpu->state_int(M68K_A0));
						std::string nd;
						for (int k = 0; k <= 0x28; k += 2) nd += util::string_format(" +%02x=%04x", k, ds.read_word((a0 + k) & 0xffff));
						logerror("       RING A0=%04x D0=%04x 7e1e=%04x node+28=%04x |%s | anchor743a=%04x 7224=%04x 7250=%04x 71b2=%04x\n",
							a0, u16(m_cpu->state_int(M68K_D0)), ds.read_word(0x7e1e), ds.read_word((a0 + 0x28) & 0xffff), nd.c_str(),
							ds.read_word(0x743a), ds.read_word(0x7224), ds.read_word(0x7250), ds.read_word(0x71b2));
					}
					if (nm[0]=='X')  // Xdisp1c8c (dispatch entry) / Xdecr1d66 (decrement): node 748a's OWN fields
					{
						std::string nd;
						for (int k = 0; k <= 0x28; k += 2) nd += util::string_format(" +%02x=%04x", k, ds.read_word((0x748a + k) & 0xffff));
						logerror("       NODE748a D0=%04x |%s | anchor743a=%04x 7224=%04x 7250=%04x 721c=%04x 71b2=%04x\n",
							u16(m_cpu->state_int(M68K_D0)), nd.c_str(),
							ds.read_word(0x743a), ds.read_word(0x7224), ds.read_word(0x7250), ds.read_word(0x721c), ds.read_word(0x71b2));
					}
				});
		}
	}
	// TEMP (STRIP): does the boot read reach the command-block builder? tap $72d6 (the watch-record
	// setup ptr written @0x37ae) - if it fires, the builder/read-setup ran; log the UIB gate byte too.
	if (m_fw_driven)
		m_cpu->space(AS_PROGRAM).install_write_tap(0x72d6, 0x72d7, "cmdblk",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				static int n = 0;
				if (n++ < 8)
				{
					address_space &cs = m_cpu->space(AS_PROGRAM);
					u16 const uib = cs.read_word(0x799a);
					logerror("CMDBLK watch-rec $72d6=%04x (builder ran) UIB=%04x UIB+12=%02x pc=%06x @%.4f\n",
							data, uib, cs.read_byte((uib + 0x12) & 0xffff), m_cpu->pc(), machine().time().as_double());
				}
			});
	// TEMP (STRIP): does the verify chain READ $7dac (0x9618) during the failing op? + the value it sees
	if (m_fw_driven)
		m_cpu->space(AS_PROGRAM).install_read_tap(0x7dac, 0x7db3, "idrd",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ static int _n = 0; double t = machine().time().as_double(); if (t > 5.7 && _n++ < 30) logerror("IDrd @%04x=%04x pc=%06x @%.4f\n", offset, data, m_cpu->pc(), t); });
	// TEMP (STRIP): $7a18 cleared at 0x3c0e = the IRQ4 queue-walker ISR (0x3bfe) actually ran; capture the
	// descriptor state Dave asked for at entry ($743a active node, $7a14/$7a6c/$7928/$7a76, node+0x20, $749c)
	if (m_fw_driven)
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7a18, 0x7a19, "isrran",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ static int _n = 0; double t = machine().time().as_double(); if (t > 5.7 && _n++ < 40) { address_space &xs = m_cpu->space(AS_PROGRAM);
				logerror("ISR3bfe ran 743a=%04x 749c=%04x 7a14=%04x 7a6c=%04x 7928=%04x 7a76=%04x node+20=%04x pc=%06x @%.4f\n",
					xs.read_word(0x743a), xs.read_word(0x749c), xs.read_word(0x7a14), xs.read_word(0x7a6c),
					xs.read_word(0x7928), xs.read_word(0x7a76), xs.read_word(0x74aa), m_cpu->pc(), t); } });
	// TEMP (STRIP): outcome probe - $749c writes (0x3c76 clear = op completed via queue-walker)
	if (m_fw_driven)
		m_cpu->space(AS_PROGRAM).install_write_tap(0x749c, 0x749d, "o749c",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ double t = machine().time().as_double(); static int _n = 0; if (t > 6.35 && _n++ < 120) logerror("749c<-%04x %s pc=%06x cmd=%02x @%.5f\n", data, data ? "BUSY" : "IDLE", m_cpu->pc(), m_iopb_cmd, t); });
	// TEMP (STRIP): the SEEK-DONE CONTRACT tap (Dave). Capture the working 0x87 completion's [$71b2] 0->1
	// bump + [$7b1a] set, and the CCB/node state consumed - that state IS the seek-complete contract.
	if (m_fw_driven)
	{
		m_cpu->space(AS_PROGRAM).install_write_tap(0x71b2, 0x71b3, "b2contract",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; if (_n++ >= 80) return; address_space &xs = m_cpu->space(AS_PROGRAM);
				u16 const node = xs.read_word(0x743a);
				logerror("71b2<-%04x pc=%06x | 7ff8/a/c/e=%02x/%02x/%02x/%02x node743a=%04x n[+26]=%04x [+12]=%04x 7a76=%04x 7a14=%04x 7928=%04x 7b1a=%04x cmd=%02x @%.5f\n",
					data, m_cpu->pc(), xs.read_byte(0x7ff8), xs.read_byte(0x7ffa), xs.read_byte(0x7ffc), xs.read_byte(0x7ffe),
					node, xs.read_word((node + 0x26) & 0xffff), xs.read_word((node + 0x12) & 0xffff),
					xs.read_word(0x7a76), xs.read_word(0x7a14), xs.read_word(0x7928), xs.read_word(0x7b1a), m_iopb_cmd, machine().time().as_double());
				// cont.210 (STRIP): claim-site context - the phantom [$71B2]++ (claim with no
				// doorbell) is the count leak behind the 5th-cmd death. A0 = the claimed node.
				u32 const bpc = m_cpu->pc();
				if (bpc >= 0x2618 && bpc <= 0x2640)
				{ u16 const a0 = u16(m_cpu->state_int(M68K_A0)), a2 = u16(m_cpu->state_int(M68K_A2));
					logerror("CLAIM-CTX A0=%04x [A0+26]=%04x [A0+0]=%04x A2=%04x [A2]=%02x q7e1c=%04x q7e1e=%04x 7b1a=%04x 7b1c=%04x n0ph=%04x n1ph=%04x mb0=%02x %02x %02x %02x %02x %02x %02x %02x ccb0=%02x %02x @%.6f\n",
						a0, xs.read_word((a0 + 0x26) & 0xffff), xs.read_word(a0), a2, xs.read_byte(a2),
						xs.read_word(0x7e1c), xs.read_word(0x7e1e), xs.read_word(0x7b1a), xs.read_word(0x7b1c),
						xs.read_word(0x71ec), xs.read_word(0x7216),
						xs.read_byte(0x7ff0), xs.read_byte(0x7ff1), xs.read_byte(0x7ff2), xs.read_byte(0x7ff3),
						xs.read_byte(0x7ff4), xs.read_byte(0x7ff5), xs.read_byte(0x7ff6), xs.read_byte(0x7ff7),
						xs.read_byte(0x7fe0), xs.read_byte(0x7fe1), machine().time().as_double()); } });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7b1a, 0x7b1b, "b1acontract",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; if (_n++ >= 80) return; address_space &xs = m_cpu->space(AS_PROGRAM);
				u16 const node = xs.read_word(0x743a);
				logerror("7b1a<-%04x pc=%06x | 7ff8/a/c/e=%02x/%02x/%02x/%02x node743a=%04x n[+26]=%04x [+12]=%04x 7a76=%04x 71b2=%04x cmd=%02x @%.5f\n",
					data, m_cpu->pc(), xs.read_byte(0x7ff8), xs.read_byte(0x7ffa), xs.read_byte(0x7ffc), xs.read_byte(0x7ffe),
					node, xs.read_word((node + 0x26) & 0xffff), xs.read_word((node + 0x12) & 0xffff),
					xs.read_word(0x7a76), xs.read_word(0x71b2), m_iopb_cmd, machine().time().as_double()); });
		// TEMP (STRIP): step-2 probe - the ID-scan continuation cells. $72d6-$72e2 = the waiter-pointer
		// table the $37xx/$39xx setup builders register their descriptor blocks in ([$72de] -> the $9984
		// DATASTEP gate the $89f2 lock-on success clears); $7968/$7986 = the other two $89f2 continuation
		// hooks. In run6 lock-on completed into [$72de]=0 (clr went to unmapped 0) - was the cell
		// registered-then-cleared ($7a6a/$8446/$91fe) or never written (the $3736/$390e builder never ran)?
		m_cpu->space(AS_PROGRAM).install_write_tap(0x72d6, 0x72e5, "waiterptrs",
			[this](offs_t offset, u16 &data, u16)
			{ static int _n = 0; if (_n++ >= 120) return;
				logerror("WAITER[%04x]<-%04x pc=%06x cmd=%02x @%.5f\n", offset, data, m_cpu->pc(), m_iopb_cmd, machine().time().as_double()); });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7968, 0x7969, "hook7968",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; if (_n++ >= 60) return;
				logerror("HOOK 7968<-%04x pc=%06x cmd=%02x @%.5f\n", data, m_cpu->pc(), m_iopb_cmd, machine().time().as_double()); });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7986, 0x7987, "hook7986",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; if (_n++ >= 60) return;
				logerror("HOOK 7986<-%04x pc=%06x cmd=%02x @%.5f\n", data, m_cpu->pc(), m_iopb_cmd, machine().time().as_double()); });
		// TEMP (STRIP): ring-protocol branch taps - the $7e00 step-generator result ring has two fw
		// consumers (CHANSVC/$1bbe on $7b1c/$7b20, the $2000 walk on $7b2a/$7b2c) and no fw producer;
		// the branch pattern + cursor evolution names the hardware's produce contract.
		auto ringdump = [this](char const *what)
		{ static int _n = 0; if (_n++ >= 160) return; address_space &xs = m_cpu->space(AS_PROGRAM);
			logerror("RING %-10s 7b1c=%04x 7b20=%04x 7b2a=%04x 7b2c=%04x 7b22=%04x 7e1c=%04x ent[7e00/02/04]=%04x/%04x/%04x cmd=%02x @%.5f\n",
				what, xs.read_word(0x7b1c), xs.read_word(0x7b20), xs.read_word(0x7b2a), xs.read_word(0x7b2c),
				xs.read_word(0x7b22), xs.read_word(0x7e1c), xs.read_word(0x7e00), xs.read_word(0x7e02), xs.read_word(0x7e04),
				m_iopb_cmd, machine().time().as_double()); };
		m_cpu->space(AS_OPCODES).install_read_tap(0x1bbe, 0x1bbf, "ring_1bbe",
			[ringdump](offs_t, u16 &, u16){ ringdump("CONSUME1bbe"); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x209c, 0x209d, "ring_209c",
			[ringdump](offs_t, u16 &, u16){ ringdump("WALK-HIT"); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x223c, 0x223d, "ring_223c",
			[ringdump](offs_t, u16 &, u16){ ringdump("WALK-END"); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x2244, 0x2245, "ring_2244",
			[ringdump](offs_t, u16 &, u16){ ringdump("STATE8"); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x2056, 0x2057, "ring_2056",
			[ringdump](offs_t, u16 &, u16){ ringdump("BBA-OK"); });
		// TEMP (STRIP): the op0->op1 advance-event hunt (build#5 cont.3). The $1d1a idle wait exits on
		// [$749c]==0 (op complete via $3c76 queue-walker = the WANTED advance) or node[$18]!=0 (error
		// = the 2.61s retry). Tap the READ node's ($748a) error cell $74a2 (names the timeout OWNER +
		// code) and its state field $74b0 (the re-park state the advance must move).
		m_cpu->space(AS_PROGRAM).install_write_tap(0x74a2, 0x74a3, "node18err",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; if (_n++ >= 60) return;
				logerror("NODE18ERR 74a2<-%04x pc=%06x cmd=%02x @%.5f\n", data, m_cpu->pc(), m_iopb_cmd, machine().time().as_double()); });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x74b0, 0x74b1, "node26st",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; if (_n++ >= 80) return;
				logerror("NODE26ST 74b0<-%04x pc=%06x cmd=%02x @%.5f\n", data, m_cpu->pc(), m_iopb_cmd, machine().time().as_double()); });
		// TEMP (STRIP): measurement 2 (Dave) - the DIRECT host-IOPB status tap (bypasses the E802
		// bit7 LEVEL mask): every write to the host IOPB @0fe780 status bytes, with writer pc.
		// Together with DOORCMD times this answers: does the CPUAP doorbell 0x95 mid-restore, or
		// only after a DONE edge?
		m_bus->space(AS_PROGRAM).install_write_tap(0xfe780, 0xfe787, "hostiopb",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ static int _n = 0; if (_n++ >= 60) return;
				logerror("HOSTIOPB [%06x]<-%04x&%04x pc(68k)=%06x @%.5f\n", offset, data, mem_mask, m_cpu->pc(), machine().time().as_double()); });
		// build#5 cont.9 KEEPER: the pending-IOPB re-fetch. Measured (STALECMD run33): the fw's
		// pickup preamble ($c84-$c96: mark [$7a14]=$748a pending, store its CURRENT CCB node into
		// [$7a06] at $c8e) re-runs on a dispatch retry with the channel rotation on the OTHER node
		// ($71c6) - the fw expects the gate array to deliver the still-pending doorbell IOPB into
		// the NEWLY declared destination (mailbox ptr + doorbell latch are still live on real HW).
		// The model fetched only once at GO time into the then-current [$7a06] ($71f0), leaving the
		// dispatcher to read cmd=00 from $71c6 -> the 0x14 loop. Honor the fw's declared dst: on the
		// preamble's [$7a06] store, re-deliver the pending IOPB there.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7a06, 0x7a07, "iopbdst",
			[this](offs_t, u16 &data, u16)
			{
				if (!storager_getenv("STORAGER_NOBYPASS")) return;
				u16 const pc = u16(m_cpu->pc());
				if (pc < 0x0c84 || pc > 0x0c9a) return;   // only the pickup preamble's $c8e store
				if (!m_iopb_addr) return;                  // no doorbell pending
				u16 const dst = data;
				if (dst < 0x4000 || dst >= 0x7e00) return;
				address_space &cs = m_cpu->space(AS_PROGRAM);
				address_space &bs = m_bus->space(AS_PROGRAM);
				for (u32 k = 0; k < 0x18; k++)   // cont.209: 12 words - +0x18/+0x19 is the fw's error cell, NOT host IOPB
					cs.write_byte((dst + k) & 0xffff, bs.read_byte((m_iopb_addr + k) & 0xffffff));
				logerror("IOPB-REFETCH %06x -> %04x cmd=%02x src[2..5]=%02x %02x %02x %02x (pickup re-point) @%.5f\n",
					m_iopb_addr, dst, cs.read_byte(dst),
					bs.read_byte((m_iopb_addr + 2) & 0xffffff), bs.read_byte((m_iopb_addr + 3) & 0xffffff),
					bs.read_byte((m_iopb_addr + 4) & 0xffffff), bs.read_byte((m_iopb_addr + 5) & 0xffffff),
					machine().time().as_double());
			});
		// TEMP (STRIP): lock-on countdown tracer (build#5 cont.9, run34: 457k deliveries, perfect
		// IDs, lock-on never fires). $6ab2 = the $7a0c #$3 seeder; $8a32 = the per-valid-ID
		// decrement; $8a0e = the node[$20]-bit14 reset; $8a42 = LOCKED-ON. Counts + samples name
		// which leg starves.
		m_cpu->space(AS_OPCODES).install_read_tap(0x6ab2, 0x6ab3, "lockseed",
			[this](offs_t, u16 &, u16){ static int n = 0; if ((++n & 0xff) == 1) logerror("LOCKSEED ($7a0c<-3) #%d @%.5f\n", n, machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x8a32, 0x8a33, "lockdec",
			[this](offs_t, u16 &, u16){ static int n = 0; address_space &xs = m_cpu->space(AS_PROGRAM);
				if ((++n & 0xfff) == 1 || n <= 6) logerror("LOCKDEC #%d 7a0c=%04x 79a4=%04x @%.5f\n", n, xs.read_word(0x7a0c), xs.read_word(0x79a4), machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x8a0e, 0x8a0f, "lockrst",
			[this](offs_t, u16 &, u16){ static int n = 0; address_space &xs = m_cpu->space(AS_PROGRAM);
				if ((++n & 0xfff) == 1 || n <= 6) logerror("LOCKRST #%d (bit14) 7a0c=%04x node20=%04x @%.5f\n", n, xs.read_word(0x7a0c), xs.read_word((xs.read_word(0x799a) + 0x20) & 0xffff), machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x8a42, 0x8a43, "locked",
			[this](offs_t, u16 &, u16){ static int n = 0; if (++n <= 6) logerror("LOCKED-ON #%d @%.5f\n", n, machine().time().as_double()); });
		// TEMP (STRIP): UIB flag-byte writer namer (build#5 cont.10). UIB table @$20a: unit0=$6c00
		// unit1=$6d30 unit2=$6e60(FLOPPY 0) unit3=$6f90. The mode byte = UIB+$12: $6e72 (floppy,
		// honest-path value 0x45 bit1-clear) and $6c12 (HD unit 0 = the MAPPING reference - its
		// interrogation worked for the label read). Name the fw pc that writes each + the value.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x6e72, 0x6e73, "uibflag2",
			[this](offs_t, u16 &data, u16 mem_mask)
			{ static int _n = 0; if (_n++ >= 30) return;
				logerror("UIBFLAG-u2 6e72<-%04x&%04x pc=%06x cmd=%02x @%.5f\n", data, mem_mask, m_cpu->pc(), m_iopb_cmd, machine().time().as_double()); });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x6c12, 0x6c13, "uibflag0",
			[this](offs_t, u16 &data, u16 mem_mask)
			{ static int _n = 0; if (_n++ >= 30) return;
				logerror("UIBFLAG-u0 6c12<-%04x&%04x pc=%06x cmd=%02x @%.5f\n", data, mem_mask, m_cpu->pc(), m_iopb_cmd, machine().time().as_double()); });
		// TEMP (STRIP): soft-vector routing tracer (build#5 cont.14). IRQ6 -> trampoline $26ae ->
		// jump [[$7304]] (disasm header); $9602 = the data-phase setup that installs $9884 there
		// (callers: $7492 walker + $99xx DATASTEP family). Does $9602 ever run, and where does
		// level-6 actually land per fire?
		m_cpu->space(AS_OPCODES).install_read_tap(0x9602, 0x9603, "dsetup",
			[this](offs_t, u16 &, u16){ static int _n = 0; if (_n++ < 12) logerror("DSETUP-9602 (install $9884) @%.5f\n", machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x26ae, 0x26af, "tramp6",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0, _m = 0; double const t = machine().time().as_double();
				address_space &xs = m_cpu->space(AS_PROGRAM);
				u32 const v = (u32(xs.read_word(0x7304)) << 16) | xs.read_word(0x7306);
				// cont.38: second window t>7.9 - is level-6 TAKEN in the timeout-retry era, and
				// through which vector? (the 12.5ms cycle = pure $7986 timeouts if not)
				if (t <= 7.9 ? _n++ < 8 : (t > 7.995 && _m++ < 30))
					logerror("TRAMP6 [$7304]=%08x @%.5f\n", v, t); });
		// cont.38: same question for IRQ5's trampoline ($26a8) - the $7b02 match path installs
		// [[$7300]]=$7ba8 via a word write to $7302; which level actually carries ID-complete?
		m_cpu->space(AS_OPCODES).install_read_tap(0x26a8, 0x26a9, "tramp5",
			[this](offs_t, u16 &, u16)
			{ static int _m = 0; double const t = machine().time().as_double();
				if (t > 9.79 && t < 10.1 && _m++ < 30)
				{ address_space &xs = m_cpu->space(AS_PROGRAM);
					logerror("TRAMP5 [$7300]=%04x%04x @%.5f\n", xs.read_word(0x7300), xs.read_word(0x7302), t); } });
		// TEMP (STRIP): $7696 geometry-table writer namer (build#5 cont.15). Entry = 6B/unit:
		// word[0] = record len (>>1 = the C800 count), byte[2] = $77f8-table index, word[4] = ?.
		// No textual writer in the ROM - tap unit 0's entry (the HD's writer pc = the identify
		// store site) and unit 2's (expected silent = the gap).
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7696, 0x769b, "geom0",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ static int _n = 0; if (_n++ >= 24) return;
				logerror("GEOM-u0 [%04x]<-%04x&%04x pc=%06x @%.5f\n", offset, data, mem_mask, m_cpu->pc(), machine().time().as_double()); });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x76a2, 0x76a7, "geom2",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ static int _n = 0; if (_n++ >= 24) return;
				logerror("GEOM-u2 [%04x]<-%04x&%04x pc=%06x @%.5f\n", offset, data, mem_mask, m_cpu->pc(), machine().time().as_double()); });
		// TEMP (STRIP): the host-transfer watch (build#5 cont.23). $13be-$13ce = the fw's SRAM->host
		// Multibus DMA fire (old notes); the host DATA buffer for the sys-floppy read = 0fc0dd.
		// Real bytes landing there = the CPUAP's first honest floppy data.
		m_cpu->space(AS_OPCODES).install_read_tap(0x13be, 0x13bf, "dmafire",
			[this](offs_t, u16 &, u16){ static int _n = 0; if (_n++ < 12) logerror("DMAFIRE-13be @%.5f\n", machine().time().as_double()); });
		m_bus->space(AS_PROGRAM).install_write_tap(0xfc000, 0xfc9ff, "hostdata",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ static int _n = 0; if (_n++ >= 24) return;
				logerror("HOSTDATA [%06x]<-%04x&%04x pc(68k)=%06x @%.5f\n", offset, data, mem_mask, m_cpu->pc(), machine().time().as_double()); });
		// TEMP (STRIP): the $808a transfer-gate capture (build#5 cont.24, Dave's whole-vector rule):
		// a five-flag AND can hide two gaps behind the first one noticed. $79a8 leads (==0 =>
		// buffering completed, the gate is transfer-phase; !=0 => upstream regression).
		m_cpu->space(AS_OPCODES).install_read_tap(0x808a, 0x808b, "xfergate",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n++ >= 16) return; address_space &xs = m_cpu->space(AS_PROGRAM);
				u16 const unit = xs.read_word(0x742a);
				u16 const slot = 0x74c4 + unit * 8;
				u16 const uib = xs.read_word(0x799a);
				logerror("XFERGATE 79a8=%04x | 79b8=%04x 79b6=%04x 79b0=%04x UIB[9]=%02x | 7424=%04x 7426=%04x | slot%u@%04x={%04x %04x %04x %04x} @%.5f\n",
					xs.read_word(0x79a8), xs.read_word(0x79b8), xs.read_word(0x79b6), xs.read_word(0x79b0),
					xs.read_byte((uib + 9) & 0xffff), xs.read_word(0x7424), xs.read_word(0x7426),
					unit, slot, xs.read_word(slot), xs.read_word(slot + 2), xs.read_word(slot + 4), xs.read_word(slot + 6),
					machine().time().as_double()); });
		// TEMP (STRIP): the [$792e] source capture (build#5 cont.25). $4a7c writes [$792e] =
		// UIB[$20] & 0x2800 via [$799c]; the dispatch wrote UIB[$20]=0x8c27 (bit11 SET) via
		// [$799a] at $e7e. If [$799c] is stale (bit4-gated update at $e0e; 0x8c27 has bit4 clear),
		// the builder reads the wrong unit's config.
		m_cpu->space(AS_OPCODES).install_read_tap(0x4a7c, 0x4a7d, "srcgate",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n++ >= 10) return; address_space &xs = m_cpu->space(AS_PROGRAM);
				u16 const ua = xs.read_word(0x799a), uc = xs.read_word(0x799c);
				logerror("SRCGATE 799a=%04x [%04x+20]=%04x | 799c=%04x [%04x+20]=%04x | 790e=%08x @%.5f\n",
					ua, ua, xs.read_word((ua + 0x20) & 0xffff), uc, uc, xs.read_word((uc + 0x20) & 0xffff),
					(u32(xs.read_word(0x790e)) << 16) | xs.read_word(0x7910), machine().time().as_double()); });
		// TEMP (STRIP): kickoff-chain append counter (build#5 cont.26, Dave's count-not-presence):
		// every write to the unit slots $74c4-$74d3 with pc - 8 appends w/ 7 resets = append
		// FAILURE (a gate clears the links); 1 append = ENGAGEMENT gap (per-record appender
		// doesn't fire). Opposite fixes.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x74c4, 0x74d3, "slotwr",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 6.35 && _n++ < 60)
					logerror("SLOTWR [%04x]<-%04x&%04x pc=%06x @%.5f\n", offset, data, mem_mask, m_cpu->pc(), t); });
		// TEMP (STRIP): the $25e death-loop namer (build#5 cont.28) - run51's wide PCHIST: 100.0%
		// at $25e post-6.6 = an exception/spurious handler self-loop. Read the 68000 exception
		// frame at entry: SR at (A7), faulting/return PC at (A7)+2.
		m_cpu->space(AS_OPCODES).install_read_tap(0x025e, 0x025f, "deathloop",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n++ >= 3) return; address_space &xs = m_cpu->space(AS_PROGRAM);
				u32 const sp = u32(m_cpu->state_int(M68K_SP));
				logerror("DEATH-25e SP=%06x frame={%04x %04x %04x %04x %04x} @%.5f\n", sp,
					xs.read_word(sp & 0xffffff), xs.read_word((sp + 2) & 0xffffff), xs.read_word((sp + 4) & 0xffffff),
					xs.read_word((sp + 6) & 0xffffff), xs.read_word((sp + 8) & 0xffffff), machine().time().as_double()); });
		// TEMP (STRIP): the WAITER-REGISTRY dump (build#5 cont.30, the one-tap opener): in the
		// settle era, dump all 11 waiter cells ($72d6-$72ea), each registered descriptor's GATE
		// word, and the parked nodes' states - names WHICH descriptor waits and WHICH clear-event
		// is owed. One-shot on the scan loop body after t=8.
		m_cpu->space(AS_OPCODES).install_read_tap(0x1606, 0x1607, "waitdump",
			[this](offs_t, u16 &, u16)
			{ static int shot = 0; double const t = machine().time().as_double();
				if (shot >= 2 || (shot == 0 && t < 6.9) || (shot == 1 && t < 12.0)) return; shot++;
				address_space &xs = m_cpu->space(AS_PROGRAM);
				std::string s;
				for (u16 c = 0x72d6; c <= 0x72ea; c += 2)
				{
					u16 const d = xs.read_word(c);
					s += util::string_format(" [%04x]=%04x", c, d);
					if (d >= 0x4000 && d < 0x7e00)
						s += util::string_format("{g=%04x m=%04x h=%04x}", xs.read_word(d), xs.read_word((d + 2) & 0xffff), xs.read_word((d + 6) & 0xffff));
				}
				u16 const n6 = xs.read_word(0x71b6), nc = xs.read_word(0x71bc);
				// cont.34: [$7a66] = the parse-buffer pointer - where it resolves on the floppy
				// path decides in-buffer ($7dac staged) vs in-stream (E000) ID matching.
				u16 const p66 = xs.read_word(0x7a66);
				logerror("WAITDUMP%s | 71b6=%04x st=%02x 71bc=%04x st=%02x 727c=%04x | 7a66=%04x [7a66]:%02x %02x %02x %02x %02x %02x @%.5f\n",
					s.c_str(), n6, xs.read_byte((n6 + 0x26) & 0xffff), nc, xs.read_byte((nc + 0x26) & 0xffff),
					xs.read_word(0x727c), p66,
					xs.read_byte(p66 & 0xffff), xs.read_byte((p66 + 1) & 0xffff), xs.read_byte((p66 + 2) & 0xffff),
					xs.read_byte((p66 + 3) & 0xffff), xs.read_byte((p66 + 4) & 0xffff), xs.read_byte((p66 + 5) & 0xffff), t);
				// cont.36 OVERLAY CHECK: the ROM listing decodes CODE at $7964-$7bxx (the live
				// [$72d6] handler address) - but that range is our measured VARIABLE space
				// ($7a0e/$7a16/$7a66 fall inside its instruction operands).  Dump live RAM at
				// $7964 and $7a0c: ROM listing bytes are 4a78 79b6 6600 0130 / 41f8 79f6 47f8 063e.
				// Match = code-in-RAM overlay (self-modifying, variable reads suspect);
				// mismatch = handler $7964 is NOT these ROM bytes (listing block relocated).
				std::string c1, c2;
				for (u16 a = 0x7964; a < 0x7974; a += 2) c1 += util::string_format(" %04x", xs.read_word(a));
				for (u16 a = 0x7a0c; a < 0x7a1c; a += 2) c2 += util::string_format(" %04x", xs.read_word(a));
				logerror("OVERLAYCHK 7964:%s | 7a0c:%s @%.5f\n", c1.c_str(), c2.c_str(), t);
				{
					u16 const cur = xs.read_word(0x721a);       // op-list cursor cell ($721c floppy / $7224)
					u16 const pos = cur ? xs.read_word(cur) : 0; // current list position
					std::string ol;
					for (int k = -0x20; k <= 0x08; k += 2)
						ol += util::string_format(" %04x%s", xs.read_word((pos + k) & 0xffff), k == 0 ? "<" : "");
					logerror("OPLIST 721a=%04x [cur]=%04x 7220=%04x 7228=%04x 7b10=%04x 7abc=%04x 7abe.l=%08x list[-20..+8]:%s @%.5f\n",
						cur, pos, xs.read_word(0x7220), xs.read_word(0x7228), xs.read_word(0x7b10),
						xs.read_word(0x7abc), xs.read_dword(0x7abe), ol.c_str(), t);
				}
				{
					std::string n1, n2;
					for (u16 k = 0; k < 0x28; k += 2) { n1 += util::string_format(" %04x", xs.read_word(0x71c6 + k)); n2 += util::string_format(" %04x", xs.read_word(0x71f0 + k)); }
					logerror("NODE71C6:%s @%.5f\n", n1.c_str(), t);
					logerror("NODE71F0:%s @%.5f\n", n2.c_str(), t);
				}
				{
					std::string dsc;
					for (u16 k = 0x7438; k < 0x7460; k += 2) dsc += util::string_format(" %04x", xs.read_word(k));
					logerror("DESCAREA 7438-745e:%s | 749c=%04x 7a14=%04x 7a76=%04x @%.5f\n",
						dsc.c_str(), xs.read_word(0x749c), xs.read_word(0x7a14), xs.read_word(0x7a76), t);
				}
				logerror("RELAYFLAGS 7956=%04x 7958=%08x 796c=%04x 79ba=%04x 79b6=%04x 79b8=%04x 7968=%04x 79a8=%04x 7424=%04x 74ac=%04x 74ae=%04x 742c=%04x 7a30=%04x @%.5f\n",
					xs.read_word(0x7956), xs.read_dword(0x7958), xs.read_word(0x796c), xs.read_word(0x79ba), xs.read_word(0x79b6),
					xs.read_word(0x79b8), xs.read_word(0x7968), xs.read_word(0x79a8), xs.read_word(0x7424), xs.read_word(0x74ac),
					xs.read_word(0x74ae), xs.read_word(0x742c), xs.read_word(0x7a30), t);
				// cont.38: the capture cycle rejects every record without reaching the C-compare
				// ($202a never loads) - the $7ba8 dispatch forks on $796e sign and $79ae/$79b6/
				// $79b8; the ID-check needs $79ae!=0. Dump the fork state + the UIB record-layout
				// pointers ($ca/$cc/$ce) + the live capture buffer $7dac.
				u16 const uib = xs.read_word(0x799a);
				std::string cb;
				for (u16 a = 0x7dac; a < 0x7db8; a++) cb += util::string_format(" %02x", xs.read_byte(a));
				std::string sm;
				for (u16 a = 0x7654; a < 0x7666; a++) sm += util::string_format(" %02x", xs.read_byte(a));
				logerror("CYCLESTATE 7436=%04x 796e=%04x 79ae=%04x 79b6=%04x 79b8=%04x 7968=%04x 741c=%04x 7986=%04x 79a0=%04x 79a2=%04x 79a4=%04x | uib=%04x [ca]=%04x [cc]=%04x [ce]=%04x | 7dac:%s | 7654:%s @%.5f\n",
					xs.read_word(0x7436), xs.read_word(0x796e), xs.read_word(0x79ae), xs.read_word(0x79b6),
					xs.read_word(0x79b8), xs.read_word(0x7968), xs.read_word(0x741c), xs.read_word(0x7986),
					xs.read_word(0x79a0), xs.read_word(0x79a2), xs.read_word(0x79a4),
					uib, xs.read_word((uib + 0xca) & 0xffff), xs.read_word((uib + 0xcc) & 0xffff),
					xs.read_word((uib + 0xce) & 0xffff), cb.c_str(), sm.c_str(), t); });
		// cont.38h (STRIP): does the spin-up flag [$7a3e] EVER get set in the retry era?
		// op $42 arms a $29f8 timer (countdown=UIB[$18], sets $7a3e via $7a40 bookkeeping)
		// and the 25ms retry (= ONE PIT1-ctr0 tick) restarts the walk - fire-vs-cancel race.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7a3e, 0x7a41, "spinflag",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 7.9 && _n++ < 16)
					logerror("SPINFLAG [%04x]<-%04x&%04x pc=%06x @%.5f\n", offset, data, mem_mask, m_cpu->pc(), t); });
		// cont.38nn (STRIP): is $16c6 (the main-loop resume wrapper, bsr'd from $204c/
		// $2418) CALLED in our runs - and which UIB gate diverts it from the $17f8
		// re-stamp? Log entry with both gate bytes.
		m_cpu->space(AS_OPCODES).install_read_tap(0x16c6, 0x16c7, "reswrap",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0, _m = 0; double const t = machine().time().as_double();
				if (t <= 6.6 ? _n++ < 6 : (t > 9.5 && _m++ < 12))
				{ address_space &xs = m_cpu->space(AS_PROGRAM);
					u32 const sp = u32(m_cpu->state_int(M68K_SP));
					u16 const nb = xs.read_word(0x71bc), n6 = xs.read_word(0x71b6);
					// D0 at entry = the status word that indexed the $222 table (caller $2314);
					// A3 = the descriptor context ((-2,A3) = the slot's descriptor ptr).
					logerror("RESWRAP caller=%06x D0=%04x A3=%04x [A3-2]=%04x | 71bc=%04x st=%04x 71b6=%04x st=%04x @%.5f\n",
						xs.read_dword(sp & 0xffffff), u16(m_cpu->state_int(M68K_D0)),
						u16(m_cpu->state_int(M68K_A3)), xs.read_word((u16(m_cpu->state_int(M68K_A3)) - 2) & 0xffff),
						nb, xs.read_word((nb + 0x26) & 0xffff), n6,
						n6 ? xs.read_word((n6 + 0x26) & 0xffff) : 0, t); } });
		// cont.38qq (STRIP): the wrapper's EXIT census in the read era - which branch turns
		// the 110us dispatch away: $1718 (past the first gate), $17dc, $17a6/$17a0 (deeper),
		// or the $16f2/$170c early path only.
		for (auto ent : { std::pair<u16, char const *>{0x1718, "X-1718"}, {0x17dc, "X-17dc"},
				{0x17a6, "X-17a6"}, {0x1774, "X-1774"}, {0x172c, "X-172c"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> ns; double const t = machine().time().as_double();
					if (t > 9.5 && ns[name]++ < 6)
						logerror("%s reached @%.5f\n", name, t); });
		// cont.38pp (STRIP): the TWO-ERA EVENT DIFFERENTIAL - same taps, both eras; the
		// setup era's flowing events certify the instruments, the read era's silence is
		// then a REAL zero (immune to the stale-window artifact, 3x burned). Tap the
		// $17f8 re-stamp (unwindowed) + the $2244 status-8 stamp: the setup-era posters
		// are the reference; whatever stops posting after the $36 op is the owed event.
		m_cpu->space(AS_OPCODES).install_read_tap(0x17f8, 0x17f9, "restamp2",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0, _m = 0; double const t = machine().time().as_double();
				if (t <= 6.6 ? _n++ < 6 : (t > 9.5 && _m++ < 20))
					logerror("RESTAMP2 @%.5f\n", t); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x2244, 0x2245, "stat8",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0, _m = 0; double const t = machine().time().as_double();
				if (t <= 6.6 ? _n++ < 6 : (t > 9.5 && _m++ < 20))
					logerror("STAT8 @%.5f\n", t); });
		// cont.38ll (STRIP): DO THE INSTALLED HANDLERS RUN? Opcode taps on all six
		// template-A handlers (op $54's install, matched live) + the completion writer
		// $8140 + its $8290 continuation. Counts only - execution truth per handler.
		for (auto ent : { std::pair<u16, char const *>{0x3dbc, "H-3dbc"}, {0x9188, "H-9188"},
				{0x94ec, "H-94ec"}, {0x9398, "H-9398"}, {0x8140, "H-8140-COMPL"}, {0x8290, "H-8290-CONT"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> ns; int const n = ns[name]++;
					if (n < 6 || (n % 500) == 0)
						logerror("%s run#%d @%.5f\n", name, n, machine().time().as_double()); });
		// cont.39j (STRIP): the UIB[$20]-maintainer - the fw rewrites $8c over the copied
		// $68 between 6.4005 and 6.4134; name the writer and its source.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x6e80, 0x6e81, "uib20wr",
			[this](offs_t, u16 &data, u16 mem_mask)
			{ static int _n = 0; if (_n++ < 10)
				logerror("UIB20WR <-%04x&%04x pc=%06x @%.5f\n", data, mem_mask, m_cpu->pc(), machine().time().as_double()); });
		// cont.42a (STRIP): the resume circuit - $2244 (op-8 re-stamp) with its entry flavor,
		// the IRQ2 tail ($2626/$261a stamps), the worker invocation ($2040 bsr $16c6 = op-4's
		// worker pass), late-era census. Plus the $7444 descriptor area at the second waitdump.
		for (auto ent : { std::pair<u16, char const *>{0x2244, "RS-2244-OP8"}, {0x223c, "RS-223c-FALL"},
				{0x2626, "RS-2626-IRQ2OP4"}, {0x261a, "RS-261a-IRQ2OP2"}, {0x2040, "RS-2040-WKRCALL"},
				{0x1f82, "RS-1f82-OP4ENTRY"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, std::pair<int, int>> ns; auto &e = ns[name];
					double const t = machine().time().as_double();
					bool const late = t > 9.0;
					if ((late ? e.second : e.first) < 10)
					{ (late ? e.second : e.first)++;
						logerror("%s @%.5f\n", name, t); } });
		// cont.53 (STRIP): grind-rate counting taps (no per-hit logging - counters only).
		// match17 = $7e60 (the $7e58 compare's EQUAL fall-through); dequeue = $32d8 ($32ac's
		// queue-found side); restock = $352e (the append). Kicks counted in ch_w.
		for (auto ent : { std::pair<u16, int *>{0x7e60, &s_grind.match17}, {0x32d8, &s_grind.dequeue},
				{0x352e, &s_grind.restock} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, "grindcnt",
				[ctr = ent.second](offs_t, u16 &, u16) { (*ctr)++; });
		// cont.69: RESTAGE-KEYED INVALIDATION (the queued refinement). The op-56 restage
		// ($79d8 write) retires the live staging - the next GO arms fresh. The retiring
		// staging's extra slots (span beyond ask) are remembered for the continue re-carry.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x79d8, 0x79d9, "restage-inval",
			[this](offs_t, u16 &, u16)
			{ if (storager_getenv("STORAGER_NOBYPASS") && s_desc.active)
				{ s_desc.pask = s_desc.total; s_desc.phost = s_desc.host;
					for (int k = 0; k < 16; k++) s_desc.paims[k] = s_desc.aims[k];
					s_desc.pspan = 16; s_desc.active = false;
					logerror("RESTAGE-INVAL prior staging retired (dmap=%04x) @%.5f\n", s_desc.dmap, machine().time().as_double()); } });
		// cont.75 (STRIP): THE WAIT-STATE DIFFERENTIAL. Tap the op-list walker's dispatch
		// ($15ba jsr) - every op executed, with the wait flags live: read1's "clear->wait->set"
		// vs reads 2/3's "already-set->instant" names the stale flags by difference.
		// cont.85 (STRIP): THE PARK TRAP. $24a = the fw's fatal handler (move #$2600,SR + spin
		// at $256) - and vectors 2 (bus err), 3 (addr err), 15 (uninit), 24 (spurious) ALL
		// point here. Run216 AND run224 both sit in it by 9.8: the "honest wait" was a parked
		// board. Dump the exception frame at entry - SP + 8 words names the fault type,
		// faulting PC, and (bus/addr err) the access address.
		// cont.91 (STRIP): teardown-vs-accept ordering - (a) $ef0 = the accept path's head
		// queue write site: log its inputs + head state each visit (does it RUN and skip, or
		// never run, for read2?); (b) $1b42 = teardown entry: trigger context + return stack.
		// Windows: setup accepts, read1 completion era, read2 era.
		// $ee4 = `btst #5,D5; beq $113a` - the accept queues the node ONLY if D5 bit5 set.
		// cont.99 (STRIP): THE SEGMENT-TABLE DIFFERENTIAL - $849a = the batch-start limit
		// load ([$7430] <- $7696-table[idx*6].+2). Log every visit: the queue entry, the
		// index, the loaded entry, and the table's first 3 entries raw - read1's batch
		// start (6.40-6.42), read1's close era (7.95-7.99), read2's start (7.97-8.01).
		// cont.99b (STRIP): [$7A62]'s BIOGRAPHY - every write, pc+value, 6.40-8.06. Its reset
		// at read1's batch start vs the (expected-missing) reset at read2's names the
		// resetter read2 skips.
		// cont.107 (STRIP): THE REBUILD DIFFERENTIAL - $a4c = the queue initializer
		// (entries = [$7966]/128 clamped 50; head <- pool $74c4). Callers: $8e6 (boot),
		// $1922 (dispatch/accept region), $a3be, $a4b2. Does read1's accept reach it and
		// read2's not? Every visit, return stack + the sizing input.
		// cont.109: true-return read INSIDE the routine, past the movem (D0-D6/A0-A1 = 36
		// bytes) - run243's entry-fetch dump was prefetch-polluted (SP not yet meaningful).
		// Window from ZERO (doctrine: no capped/windowed absences).
		m_cpu->space(AS_OPCODES).install_read_tap(0xa90, 0xa91, "qrebuild",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (n >= 40 || t > 8.30) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				u32 const sp = m_cpu->state_int(M68K_SP);
				u32 const ret = (u32(ds.read_word(sp + 36)) << 16) | ds.read_word(sp + 38);
				logerror("QREBUILD ret=%06x 7966=%04x 791a=%04x 74ac=%04x @%.5f\n",
					ret & 0xffffff, ds.read_word(0x7966), ds.read_word(0x791a), ds.read_word(0x74ac), t); });
		// cont.111 (STRIP): THE LEDGER-SCAN TAPE - $3320 = $32ac's queue-empty path, which
		// SCANS THE $7654 LEDGER to answer MORE (next segment) or DRY. Dump the ledger at
		// every scan, windows-from-zero: read1's final (DRY-verdict) ledger vs read2's
		// (perpetual MORE) = the content differential that defines DONE.
		// cont.156 (STRIP): THE PIT RELOAD FRAME - the fw's writes to the pit[1] data/ctrl
		// ports ($8000-$8007 odd bytes = pit1 per the interleave). The running-era reload
		// values resolve cont.154's paradox (0xFF00 can't tick fast; the model observed
		// one boot programming). Windows-from-zero, dedup.
		for (offs_t base : { offs_t(0x8000), offs_t(0xff8000) })   // cont.157: + the ff-mirror
			m_cpu->space(AS_PROGRAM).install_write_tap(base, base + 7, "pitreload",
				[this](offs_t offset, u16 &data, u16 mem_mask)
				{ static int n = 0; double const t = machine().time().as_double();
					if (n >= 80 || t > 8.30) return; n++;
					// cont.158: the LANE is the answer - mask 00ff = ODD byte = pit[1],
					// ff00 = EVEN = pit[0]. Which counter really got each control word.
					logerror("PITWR [%04x]<-%04x mask=%04x (%s) pc=%06x @%.6f\n",
						0x8000 + (offset & 7), data, mem_mask,
						mem_mask == 0x00ff ? "pit1/odd" : mem_mask == 0xff00 ? "pit0/even" : "word",
						m_cpu->pc(), t); });
		// cont.155 (STRIP, Dave's inversion): $6c54 - the settle-delay read. desc[$18]==0
		// -> $6c5c set-flag-NOW (warm, no settle - read2's rightful path); !=0 -> arm the
		// timer (read1's legitimate cold spin). A0 names the structure; the byte names
		// the value; cold (6.41) vs warm (7.99) names the wrongness. Then: the writer.
		m_cpu->space(AS_OPCODES).install_read_tap(0x6c54, 0x6c55, "settleread",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (n >= 16 || t > 8.30) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				u32 const a0 = m_cpu->state_int(M68K_A0) & 0xffffff;
				logerror("SETTLERD A0=%06x [18]=%02x @%.6f\n", a0,
					ds.read_byte((a0 + 0x18) & 0xffffff), t); });
		// cont.154 (STRIP): THE TICK-AND-RECORD FRAME - $2b58 (the IRQ1 tick handler:
		// does it fire in the era?) with the [$736c] record head + its count/cell fields,
		// and $6cde (op-42's waiting-pass divert: is the record registered?). Measure
		// before wiring - the contract is healthy iff tick fires AND record exists.
		m_cpu->space(AS_OPCODES).install_read_tap(0x2b58, 0x2b59, "tickwalk",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (t < 7.98 || t > 8.20 || n >= 25) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				u16 const rec = ds.read_word(0x736c);
				logerror("TICKWALK rec=%04x cnt=%04x cell=%04x 7a3e=%04x 7a40=%04x @%.6f\n",
					rec, rec ? ds.read_word(rec & 0xffff) : 0xdead,
					rec ? ds.read_word((rec + 4) & 0xffff) : 0xdead,
					ds.read_word(0x7a3e), ds.read_word(0x7a40), t); });
		// cont.153 (STRIP, Dave's spec): THE OP-TABLE TAP - $15b4's own registers (D0 = op
		// index, A0 = $192), no pc attribution step, structurally skew-immune. Dedup on
		// index-change with repeat counts; m_want_ready on every frame so found-the-poller
		// and re-enters-with-the-bit-up answer on the same line. Window 7.99-8.30.
		m_cpu->space(AS_OPCODES).install_read_tap(0x15b4, 0x15b5, "optable",
			[this](offs_t, u16 &, u16)
			{ static unsigned last = ~0u; static int rep = 0, n = 0;
				double const t = machine().time().as_double();
				if (t < 7.99 || t > 8.30) return;
				unsigned const idx = unsigned(m_cpu->state_int(M68K_D0)) & 0xffff;
				if (idx == last) { rep++; return; }
				if (n < 120)
				{ n++;
					u16 const hdlr = m_cpu->space(AS_PROGRAM).read_word((0x192 + idx) & 0xffff);
					logerror("OPTAB idx=%02x hdlr=%04x (prev x%d) wr=%d @%.6f\n",
						idx, hdlr, rep, m_want_ready ? 1 : 0, t); }
				last = idx; rep = 0; });
		// cont.152 (STRIP): $6c10 - the btst itself, D1 in hand. Is the $6bc2 poll running
		// post-8.0125, and what does its F000 read hold? (Kills the pc-skew inference.)
		m_cpu->space(AS_OPCODES).install_read_tap(0x6c10, 0x6c11, "pollbtst",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				bool const w = (t > 7.99 && t < 7.995) || (t > 8.012 && t < 8.30);
				if (!w || n >= 20) return; n++;
				logerror("POLLBTST D1=%04x wr=%d @%.6f\n",
					unsigned(m_cpu->state_int(M68K_D1)) & 0xffff, m_want_ready ? 1 : 0, t); });
		// cont.151 (STRIP, Dave's decider): $6c18 - the $6bc2 poll's second conjunct.
		// A1 (=[$71bc]+$18, inside the host IOPB), the field's live value, [$f000], and
		// the never-tapped [$7a60] re-init machine. Both eras. Host-fed-and-zero -> the
		// model presents IOPB[$18]; firmware-written-and-skipped -> the builder's ordering.
		m_cpu->space(AS_OPCODES).install_read_tap(0x6c18, 0x6c19, "conjunct2",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (n >= 24 || t > 8.30) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				u32 const a1 = m_cpu->state_int(M68K_A1) & 0xffffff;
				logerror("CONJ2 A1=%06x (A1)=%04x f000=%04x 7a60=%04x 71bc=%04x @%.6f\n",
					a1, ds.read_word(a1 & 0xffffff), m_ch[(0xf000 - 0xe000) / 2],
					ds.read_word(0x7a60), ds.read_word(0x71bc), t); });
		// cont.147 (STRIP): [$7968] + [$7B10] live at the walk's consume/match instants -
		// extend the paired exit tap's columns. If both up at any of read2's matches, the
		// stamp should fire there; if [$7b10] down, its $6ed6 re-arm never re-runs
		// mid-command.
		m_cpu->space(AS_OPCODES).install_read_tap(0x7d68, 0x7d69, "walkgate",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (n >= 30 || t > 8.30) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				logerror("WALKGATE 7b10=%04x 7968=%04x 7428=%04x @%.6f\n",
					ds.read_word(0x7b10), ds.read_word(0x7968), ds.read_word(0x7428), t); });
		// cont.145 (STRIP, Dave's spec): THE TWO-BIT GATE - DONE = descriptor[$18]!=0 AND
		// D1.bit4 at $17fe/$1810 ($82 post vs the $1816 reset). Reach-check first:
		// $7106 (the chain's ledger-accounting step) + $17f8 (the c-stamp/gate entry),
		// logging A0, desc[$18], D1, windows-from-zero. Read1: both bits (expect $82);
		// read2's post-f0 passes: reach-and-one-bit vs never-reach.
		for (auto csite : { std::pair<offs_t, char const *>{0x7106, "CHAIN7106"}, {0x17f8, "CSTAMPGATE"} })
			m_cpu->space(AS_OPCODES).install_read_tap(csite.first, csite.first | 1, csite.second,
				[this, csite](offs_t, u16 &, u16)
				{ static int n = 0; double const t = machine().time().as_double();
					if (n >= 30 || t > 8.30) return; n++;
					address_space &ds = m_cpu->space(AS_PROGRAM);
					u32 const a0 = m_cpu->state_int(M68K_A0) & 0xffffff;
					logerror("%s A0=%06x d18=%04x D1=%04x @%.6f\n", csite.second, a0,
						ds.read_word((a0 + 0x18) & 0xffffff),
						unsigned(m_cpu->state_int(M68K_D1)) & 0xffff, t); });
		// cont.143 (STRIP): THE ARM'S BIOGRAPHY - [$7968] (the pop-result/want-present
		// flag that gates the c-stamp arm at $6ed2: ==0 -> $6f3a disarms [$7b10]) and
		// [$7B10] itself. Writers, pc+value, windows-from-zero. Why read1's $6ed2 pass
		// finds [$7968]!=0 and read2's finds 0 - the last cell before $ba.
		for (auto bc : { std::pair<u16, char const *>{0x7968, "FLAG7968"}, {0x7b10, "ARM7B10"} })
			m_cpu->space(AS_PROGRAM).install_write_tap(bc.first, bc.first | 1, bc.second,
				[this, bc](offs_t, u16 &data, u16)
				{ static int n = 0; double const t = machine().time().as_double();
					if (n >= 60 || t > 8.30) return; n++;
					logerror("%s <-%04x pc=%06x @%.6f\n", bc.second, data, m_cpu->pc(), t); });
		// cont.142 (STRIP, Dave's pivot): THE COMMAND BLOCK at the owed-stager - $a486
		// (owed = block[7], copied not computed). Capture A6 + the block's first 24 bytes
		// (block[7] owed, [2:3] byte-count, [$12] two-sided bit1) - name the 8 before
		// trusting any exit-vs-count logic. Windows-from-zero.
		m_cpu->space(AS_OPCODES).install_read_tap(0xa486, 0xa487, "owedstager",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (n >= 12 || t > 8.30) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				u32 const a6 = m_cpu->state_int(M68K_A6) & 0xffffff;
				std::string blk;
				for (int k = 0; k < 24; k++) blk += util::string_format(" %02x", ds.read_byte((a6 + k) & 0xffffff));
				logerror("OWEDSTAGER A6=%06x blk:%s | [12]=%02x @%.6f\n", a6, blk.c_str(),
					ds.read_byte((a6 + 0x12) & 0xffffff), t); });
		// cont.140 (STRIP, Dave's spec): THE ORDERING TAP - (a) [$79A8] owed-count writes
		// (the $932c decrement, gated at $9322 on [$79b6]!=0 - a completion landing with
		// [$79b6]==0 DROPS its decrement); (b) the $9322 gate state per completion;
		// (c) the $9506 done-check firing ([$79a8] value + timestamp vs the 4th $932c).
		// Ordering decides: done-check-before-4th-decrement = wire the $7b18 edge to
		// re-dispatch the done op; [$79a8] never zeroes = the $79b6 coupling drops one.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x79a8, 0x79a9, "owed79a8",
			[this](offs_t, u16 &data, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (n >= 40 || t > 8.30) return; n++;
				logerror("OWED79A8 <-%04x pc=%06x @%.6f\n", data, m_cpu->pc(), t); });
		for (auto dsite : { std::pair<offs_t, char const *>{0x9322, "DECGATE"}, {0x9506, "DONECHK"} })
			m_cpu->space(AS_OPCODES).install_read_tap(dsite.first, dsite.first | 1, dsite.second,
				[this, dsite](offs_t, u16 &, u16)
				{ static int n = 0; double const t = machine().time().as_double();
					if (n >= 40 || t > 8.30) return; n++;
					address_space &ds = m_cpu->space(AS_PROGRAM);
					logerror("%s 79a8=%04x 79b6=%04x @%.6f\n", dsite.second,
						ds.read_word(0x79a8), ds.read_word(0x79b6), t); });
		// cont.138 (STRIP, Dave's spec): THE PAIRED EXIT TAP - $7ed8 (hunt-exit, the ONLY
		// predecessor of the $7f22 consume) + $7f22 (the consume). At each firing: the
		// three walk-body fork operands ([$79a0]/[$79b6] route-gate, [$7428] position
		// match, [$7968] final-delivery fork) + ledger[[want]]. Read1's context names the
		// branch; read2's inverted condition is the one static behind. Prediction: $7968.
		for (auto site : { std::pair<offs_t, char const *>{0x7ed8, "HUNTEXIT"}, {0x7f22, "CONSUME"} })
			m_cpu->space(AS_OPCODES).install_read_tap(site.first, site.first | 1, site.second,
				[this, site](offs_t, u16 &, u16)
				{ static int n = 0; double const t = machine().time().as_double();
					if (n >= 40 || t > 8.30) return; n++;
					address_space &ds = m_cpu->space(AS_PROGRAM);
					u16 const want = ds.read_word(0x7428);
					logerror("%s 79a0=%04x 79b6=%04x 7968=%04x 7428=%04x led[w]=%02x 7b18=%04x @%.6f\n",
						site.second, ds.read_word(0x79a0), ds.read_word(0x79b6), ds.read_word(0x7968),
						want, ds.read_byte((0x7654 + want) & 0xffff), ds.read_word(0x7b18), t); });
		// cont.132 (STRIP, Dave's spec): THE COUNT RACE - [$79A4]'s trajectory (decrement
		// $8a3c vs reload $7e60) across read2's grind, + the phase and the countdown. Does
		// the count ever approach 0, or does every want-match reload it?
		m_cpu->space(AS_PROGRAM).install_write_tap(0x79a4, 0x79a5, "count79a4",
			[this](offs_t, u16 &data, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (n >= 100 || t > 8.30 || t < 7.95) return; n++;
				logerror("CNT79A4 <-%04x pc=%06x @%.6f\n", data, m_cpu->pc(), t); });
		// The two UIB gates preceding everything: log the live UIB bytes at the $89f2
		// ID-handler entry (UIB[$12] bit1 = hunting-probe gate; UIB[$20] bit14 = no-stop).
		m_cpu->space(AS_OPCODES).install_read_tap(0x89f2, 0x89f3, "uibgates",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (n >= 20 || t > 8.30 || t < 7.95) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				u16 const uib = ds.read_word(0x799a);
				logerror("UIBGATES uib=%04x [12]=%02x [20]=%04x 79a4=%04x 79a2=%04x 7a0c=%04x 7950=%04x @%.6f\n",
					uib, ds.read_byte((uib + 0x12) & 0xffff), ds.read_word((uib + 0x20) & 0xffff),
					ds.read_word(0x79a4), ds.read_word(0x79a2), ds.read_word(0x7a0c),
					ds.read_word(0x7950), t); });
		// cont.131 (STRIP): $7402 ENTRY - the data-service dispatch (post-statics single
		// tap). Read1's entry context names the dispatcher; read2's absence against
		// passing gates names the withheld dispatch.
		m_cpu->space(AS_OPCODES).install_read_tap(0x7402, 0x7403, "dsvcentry",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (n >= 30 || t > 8.30) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				u32 const sp = m_cpu->state_int(M68K_SP);
				logerror("DSVC7402 sp=%06x [%04x %04x %04x %04x] 71bc=%04x n26=%04x 7956=%04x @%.6f\n",
					sp, ds.read_word(sp), ds.read_word(sp + 2), ds.read_word(sp + 4), ds.read_word(sp + 6),
					ds.read_word(0x71bc),
					ds.read_word(0x71bc) ? ds.read_word((ds.read_word(0x71bc) + 0x26) & 0xffff) : 0xdead,
					ds.read_word(0x7956), t); });
		// cont.130 (STRIP, Dave's spec): THE BYTE-COUNT FEED - [$7966]'s writers (pc+value,
		// windows-from-zero) + the $a3b2 rebuild gate's live comparison (D0 vs [$7966],
		// [$7b3e]) at both launches. Is read2's IOPB byte-count right for a 4-sector read
		// (0x200) or the observed 1-sector 0080 (mis-fed -> mis-sized pool/split -> broken
		// DATA delivery -> no $92b4/f0 downstream)?
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7966, 0x7967, "bytecount",
			[this](offs_t, u16 &data, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (n >= 40 || t > 8.30) return; n++;
				logerror("BYTECNT <-%04x pc=%06x @%.6f\n", data, m_cpu->pc(), t); });
		m_cpu->space(AS_OPCODES).install_read_tap(0xa3b2, 0xa3b3, "rebuildgate",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (n >= 40 || t > 8.30) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				logerror("REBGATE D0=%04x 7966=%04x 7b3e=%04x @%.6f\n",
					unsigned(m_cpu->state_int(M68K_D0)) & 0xffff,
					ds.read_word(0x7966), ds.read_word(0x7b3e), t); });
		// cont.127 v3 (KEEPER): THE [$7B0E] MERGE TAP - the gate array's level-held
		// per-segment retired-status (m_seg_retired, latched at the segment carry) merges
		// bit15 into the fw's own $0080 lock-stamp as it is written, so the pop's $347e
		// byte-bclr (high byte) sees $80 and takes $34c8 (retire/drain) instead of the
		// degenerate re-append. Consumed on merge (edge-per-segment).
		if (storager_getenv("STORAGER_NOBYPASS"))
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7b0e, 0x7b0f, "seg_retired_merge",
				[this](offs_t, u16 &data, u16)
				{
					if (m_seg_retired && data == 0x0080)
					{
						data |= 0x8000;
						m_seg_retired = false;
						logerror("RETMERGE $7b0e stamp -> %04x @%.6f\n", data, machine().time().as_double());
					}
				});
		// cont.166 (STRIP): THE E807 CHANNEL-OP PROTOCOL CENSUS. The fw queues 8-byte
		// descriptors {+0 param, +2 mailbox, +4 aux, +6 cmd} in the $737c ring ($27be),
		// dispatches cmd->E807 (byte, E802-bit6 ack pulse) + control shadow $7fe0/$7fe8;
		// the gate array does the PHYSICAL op and posts the mailbox; IRQ2 matches. The
		// model ignores E807 entirely (heuristic stepdone/capdone instead) - the settle
		// class ops never complete -> tick-watchdog fallbacks = the stall. Census first.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x737c, 0x73ff, "opring",
			[this](offs_t offset, u16 &data, u16)
			{ static int n = 0; u32 const rpc = m_cpu->pc();
				if (rpc >= 0x27e0 && rpc <= 0x2810 && n++ < 200)
				logerror("OPRING [%04x]<-%04x pc=%06x @%.6f\n", offset, data, rpc, machine().time().as_double()); });
		// cont.175 (KEEPER-candidate, FWDONE): THE HONEST TRANSCRIPTION - the fw never writes
		// host RAM (verified: $bf6 stamps ring+2 [$7b20], $c06 fast-reset 0x80 there, $1a54 the
		// local IOPB copy $71f2). The bus interface's real job is write-through of those stamps.
		// Tap the fw's status cells; mirror the byte to the host IOPB status + the monitor's
		// polled cell (fe782/3). C++ initiates nothing; the fw's own write is the trigger.
		if (storager_getenv("STORAGER_FWDONE"))
		{
			auto mirror = [this](u8 v)
			{
				address_space &hb = m_bus->space(AS_PROGRAM);
				if (m_iopb_addr) { hb.write_byte((m_iopb_addr + 2) & 0xffffff, v); hb.write_byte((m_iopb_addr + 3) & 0xffffff, v); r0_observe(u8(v)); }
				hb.write_byte(0x0fe782, v); hb.write_byte(0x0fe783, v);
				logerror("FWSTAMP-MIRROR %02x -> host (iopb=%06x) @%.6f\n", v, m_iopb_addr, machine().time().as_double());
				// cont.262e: at the stamp, capture the ERROR CODE (node[$18]) + the ledger state,
				// so we know what error the 0x82 carries and whether the ledger was complete yet.
				{ address_space &es = m_cpu->space(AS_PROGRAM);
					u16 const node = es.read_word(0x71bc);
					std::string lg; for (u16 k = 0; k < 12; k++) lg += util::string_format(" %02x", es.read_byte((0x7654 + k) & 0xffff));
					logerror("  STAMP%02x node18=%04x 7956=%04x 79a8=%04x 7428=%04x ledger:%s @%.6f\n",
						v, node ? es.read_word((node + 0x18) & 0xffff) : 0xdead, es.read_word(0x7956),
						es.read_word(0x79a8), es.read_word(0x7428), lg.c_str(), machine().time().as_double()); }
			};
			// cont.176 (Dave): pc-gate the WHEN ($bf6/$c06 accept/fast-done, $1a54/$184e complete);
			// follow [[$7b20]]+2 for the WHERE - the ring slot MOVES per command, never a static
			// address (the fixed-$7e22 tap transcribed the mailbox GO 0x13 - the wrong-because-
			// static trap again). Region taps bound the cost; the pc + slot checks do the work.
			m_cpu->space(AS_PROGRAM).install_write_tap(0x71f2, 0x71f3, "fwstamp_iopb",
				[this, mirror](offs_t, u16 &data, u16 mem_mask)
				{ u32 const spc = m_cpu->pc();
					if ((spc >= 0x1a50 && spc <= 0x1a60) || (spc >= 0x1848 && spc <= 0x1858))
					{ u8 const v = (mem_mask & 0xff00) ? (data >> 8) : (data & 0xff);
						if (v == 0x80 || v == 0x81 || v == 0x82) mirror(v); } });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7e00, 0x7eff, "fwstamp_ring",
				[this, mirror](offs_t offset, u16 &data, u16 mem_mask)
				{ u32 const spc = m_cpu->pc();
					if (spc >= 0xbf0 && spc <= 0xc10)
					{ u16 const slot = m_cpu->space(AS_PROGRAM).read_word(0x7b20);
						if (slot && offset == u32((slot + 2) & ~1))
						{ u8 const v = (mem_mask & 0xff00) ? (data >> 8) : (data & 0xff);
							if (v == 0x80 || v == 0x81 || v == 0x82) mirror(v); } } });
		}
		// cont.174 (STRIP): ALL bus-side writers of the host status slots (fe780-fe787) -
		// catches any 0x80/0x82 write my nine tagged sites don't cover. 68k pc attributed.
		m_bus->space(AS_PROGRAM).install_write_tap(0x0fe780, 0x0fe787, "busfe780",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ m_bustap_hits++; static int n = 0; double const t = machine().time().as_double();
				if (t > 6.3 && t < 8.3 && n++ < 100)
					logerror("BUS-FE780-WR [%06x] <- %04x&%04x 68kpc=%06x @%.6f\n", offset, data, mem_mask, m_cpu->pc(), t); });
		// cont.176 SELF-TEST (Dave's landmine): probe-write through the exact path the shims use;
		// if the tap doesn't fire, say so LOUDLY - a quiet instrument failure caused cont.174's
		// false read-through. (The probe writes then restores the byte - side-effect free.)
		{ address_space &tb = m_bus->space(AS_PROGRAM);
			int const before = m_bustap_hits;
			u8 const saved = tb.read_byte(0x0fe786);
			tb.write_byte(0x0fe786, saved);
			logerror("BUSTAP-SELFTEST: %s (hits %d -> %d)\n",
				(m_bustap_hits > before) ? "TAP FIRES - instrument trusted" : "TAP SILENT - DO NOT TRUST bus-side write taps",
				before, m_bustap_hits); }
		// cont.184 (STRIP): THE SCAN'S TRIGGER - at the $7e8a store, capture the interrupt
		// context (SR = which handler level) + the stacked return addresses (the caller chain).
		// Dave's lean (held loosely): a capture strobe landing before the ledger fill, ungated
		// on fw readiness. The stack says; we don't.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7430, 0x7431, "scantrig",
			[this](offs_t, u16 &data, u16)
			{ u32 const tpc = m_cpu->pc();
				if (tpc >= 0x7e80 && tpc <= 0x7e90)
				{ static int n = 0; if (n++ < 12) { address_space &ts = m_cpu->space(AS_PROGRAM);
					u32 const sp = m_cpu->state_int(M68K_SP);   // cont.185: the ACTIVE SP (A7 alias read 0 under supervisor - the run315 artifact)
					{ std::string sd;
						for (u32 sw = 0; sw < 24; sw++) sd += util::string_format(" %04x", ts.read_word(sp + sw * 2));
						logerror("SCANTRIG [7430]<-%04x pc=%06x sr=%04x a7=%06x stack24:%s @%.6f\n",
							data, tpc, u16(m_cpu->state_int(M68K_SR)), sp, sd.c_str(), machine().time().as_double()); } } } });
		// cont.181 (STRIP): the $92b4 ENTRY FRAME at its own PC - a read-tap on $742c gated
		// to the tst at $92b8. Entries, values, branch (derivable: !=0 -> $92f6 stakes f0).
		// Statics say the f0 stamp is ON the !=0 branch ($9312) -> empty ledger implies zero
		// entries; this tape converts the implication to a measurement.
		m_cpu->space(AS_PROGRAM).install_read_tap(0x742c, 0x742d, "entry92b4",
			[this](offs_t, u16 &data, u16)
			{ u32 const epc = m_cpu->pc();
				if (epc >= 0x92b4 && epc <= 0x92c0)
				{ static int n = 0; if (n++ < 40)
					logerror("92B4-ENTRY tst742c=%04x pc=%06x 7950=%04x 7428=%04x @%.6f\n",
						data, epc, m_cpu->space(AS_PROGRAM).read_word(0x7950),
						m_cpu->space(AS_PROGRAM).read_word(0x7428), machine().time().as_double()); } });
		// cont.178 (STRIP): node[$18] ($71de) = the ERROR CODE the $17fe chain publishes as
		// 0x82+code. Who stores it before the 8.025 stamp, and what value - the whole frontier.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7208, 0x7209, "errcode",
			[this](offs_t, u16 &data, u16 mem_mask)
			{ static int n = 0; if (n++ < 60)
				logerror("ERRCODE [7208]<-%04x mm=%04x pc=%06x @%.6f\n", data, mem_mask, m_cpu->pc(), machine().time().as_double());
				// cont.209 (STRIP): at the 98E7 store, full register + mailbox context - who composed it
				if (data == 0x202a || data == 0x2012)
				{ address_space &es = m_cpu->space(AS_PROGRAM);
					logerror("MISMATCH-CTX code=%04x want7436=%04x want7438=%04x cap7dac=%02x %02x %02x %02x (t,C,H,pos) 799a=%04x @%.6f\n",
						data, es.read_word(0x7436), es.read_word(0x7438),
						es.read_byte(0x7dac), es.read_byte(0x7dad), es.read_byte(0x7dae), es.read_byte(0x7daf),
						es.read_word(0x799a), machine().time().as_double()); }
				if (data == 0x98e7)
				{ address_space &es = m_cpu->space(AS_PROGRAM); std::string mb;
					for (u32 k = 0; k < 8; k++) mb += util::string_format(" %02x", es.read_byte(0x7ff8 + k));
					logerror("98E7-CTX D0=%08x D1=%08x D2=%08x D7=%08x A0=%08x A1=%08x A3=%08x A4=%08x SP=%08x mb7ff8:%s 79dc=%08x @%.6f\n",
						u32(m_cpu->state_int(M68K_D0)), u32(m_cpu->state_int(M68K_D1)), u32(m_cpu->state_int(M68K_D2)),
						u32(m_cpu->state_int(M68K_D7)), u32(m_cpu->state_int(M68K_A0)), u32(m_cpu->state_int(M68K_A1)),
						u32(m_cpu->state_int(M68K_A3)), u32(m_cpu->state_int(M68K_A4)), u32(m_cpu->state_int(M68K_SP)),
						mb.c_str(), es.read_dword(0x79dc), machine().time().as_double());
					std::string stk; u32 const sp = u32(m_cpu->state_int(M68K_SP)) & 0xffff;
					for (u32 k = 0; k < 0x18; k += 2) stk += util::string_format(" %04x", es.read_word((sp + k) & 0xffff));
					logerror("98E7-STK sp=%04x:%s | old7208=%04x 71de=%04x\n", sp, stk.c_str(),
						es.read_word(0x7208), es.read_word(0x71de)); }
				// cont.179 (STRIP): at the $69 store, dump the ledger the scan just walked -
				// which slot holds (or lacks) the wanted $f0, laid against the carry/SECMAP times.
				if ((data & 0xff) == 0x69)
				{ address_space &es = m_cpu->space(AS_PROGRAM); std::string lh;
					for (u32 lk = 0; lk < 0x20; lk++) lh += util::string_format(" %02x", es.read_byte(0x7654 + lk));
					logerror("LEDGER@69 7654:%s | 7428=%04x 7430=%04x 79a8=%04x 74ac=%04x @%.6f\n",
						lh.c_str(), es.read_word(0x7428), es.read_word(0x7430), es.read_word(0x79a8),
						es.read_word(0x74ac), machine().time().as_double()); } });
		// cont.169 (STRIP): [$7a34] (head-unloaded state) writers - the head-load settle
		// only arms when this is nonzero; $64ce loads it from IOPB byte7 bit0. Which setter
		// runs, and is the staged IOPB byte authentic?
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7a34, 0x7a35, "hl7a34",
			[this](offs_t, u16 &data, u16)
			{ static int n = 0; if (n++ < 40) { address_space &ds = m_cpu->space(AS_PROGRAM);
				logerror("W7A34 <-%04x pc=%06x iopb71f0[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x @%.6f\n",
					data, m_cpu->pc(),
					ds.read_byte(0x71f0), ds.read_byte(0x71f1), ds.read_byte(0x71f2), ds.read_byte(0x71f3),
					ds.read_byte(0x71f4), ds.read_byte(0x71f5), ds.read_byte(0x71f6), ds.read_byte(0x71f7),
					machine().time().as_double()); } });
		// cont.165 (STRIP): the installable vector block [$72f8]=IRQ3 [$7300]=IRQ5 [$7304]=IRQ6
		// (+$72fc slot) - who installs what handler, whole run. The physical drive-event handler
		// (motor-ready / head-load / seek-settle completions per Dave) should appear here.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x72f8, 0x7307, "vecinstall",
			[this](offs_t offset, u16 &data, u16)
			{ static int n = 0; if (n++ < 60)
				logerror("VECINST [%04x]<-%04x pc=%06x @%.6f\n", offset, data, m_cpu->pc(), machine().time().as_double()); });
		// cont.162 (STRIP): [$7A36]'s writers (op-28 SEEK spins $fe on it 6.41->7.96 = read1's
		// lost 1.55s) + the $736c timeout-record queue arms (each record's {count,value,cell,
		// handler} as queued) - is the seek record's count authentic or step-engine-inflated?
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7a36, 0x7a37, "seekflag7a36",
			[this](offs_t, u16 &data, u16 mem_mask)
			{ static int n = 0; if (n++ < 60)
				logerror("W7A36 <-%04x mm=%04x pc=%06x @%.6f\n", data, mem_mask, m_cpu->pc(), machine().time().as_double()); });
		// cont.163: the REAL timeout-queue nodes live at $730c+ (the $736c-$7372 head/links
		// point at them); tap the node pool, excluding the walker's own link juggling.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x730c, 0x736b, "recq",
			[this](offs_t offset, u16 &data, u16)
			{ static int n = 0; u32 const rpc = m_cpu->pc();
				if ((rpc < 0x2a00 || rpc > 0x2b60) && n++ < 300 && machine().time().as_double() > 6.35)
				logerror("RECQ [%04x]<-%04x pc=%06x @%.6f\n", offset, data, rpc, machine().time().as_double()); });
		// cont.160 (STRIP): [$7A3E]'s writers, whole run - op-42's warm-motor flag.
		// Read1 rode it warm (no 1.83s settle); read2's op-42 armed a fresh 70-tick
		// cold-motor record at ~7.96. Who clears it (or never set it)?
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7a3e, 0x7a3f, "warm7a3e",
			[this](offs_t, u16 &data, u16 mem_mask)
			{ static int n = 0; if (n++ < 120)
				logerror("WARM7A3E <-%04x mm=%04x pc=%06x @%.6f\n", data, mem_mask, m_cpu->pc(), machine().time().as_double()); });
		// cont.126 (STRIP): [$7B0E]'s writers - $32ac sets $80 on entry; the re-append
		// ($3474-$34c2, tail<-78c4 every pop) is gated on bit7 CLEAR, and it fires EVERY
		// pop: something clears bit7 mid-pop. That clearer is the final cell.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7b0e, 0x7b0f, "gate7b0e",
			[this](offs_t, u16 &data, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				bool const w = (t > 7.960 && t < 7.968) || (t > 7.999 && t < 8.190);
				if (!w || n >= 60) return; n++;
				logerror("GATE7B0E <-%04x pc=%06x @%.6f\n", data, m_cpu->pc(), t); });
		// cont.124 (STRIP): THE PHASE-BIT RESETTER - [$7950]'s writers. Read2's toggler
		// flips 0->1 each pass but re-enters at 0000 every 12.5ms; read1's flip survived
		// its 24us to the DATA pass. The resetter between read2's passes is the final
		// divergence.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x742c, 0x742d, "w742c",
			[this](offs_t, u16 &data, u16)
			{ double const wt = machine().time().as_double();
			if (!storager_getenv("STORAGER_SERVE") || wt < 7.99 || wt > 8.06) return;
			static int _w = 0; if (_w++ >= 80) return;
			logerror("W742C <-%04x pc=%06x aim=%04x @%.6f\n", data, m_cpu->pc(), m_cpu->space(AS_PROGRAM).read_word(0x7428), wt); });
		// cont.262c (STORAGER_STAKEV): LATE, WIDE f0-stake tap (survives the RAM handler
		// reinstall that wipes early $7654 taps). Catches $9312 wherever it stakes - in-ledger
		// ($7654-$7667) for read1's slot-0 anchor, or out-of-ledger ($7668-$775f) when the aim
		// poisoned to $fe. Also tap the aim-advance ($7428) reliably here.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7654, 0x775f, "stakewide",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ if (!storager_getenv("STORAGER_STAKEV")) return;
			u8 const lo = data & 0xff, hi = (data>>8)&0xff;
			if (lo != 0xf0 && hi != 0xf0) return;
			static int _s=0; if (_s++ >= 60) return;
			logerror("STAKE-F0 addr=%04x data=%04x pc=%06x aim=%04x 742c=%04x 7950=%04x @%.6f\n",
				0x7654 + (u32(offset)&0x1ff)*0, u32(offset)&0xffff, m_cpu->pc(),
				m_cpu->space(AS_PROGRAM).read_word(0x7428), m_cpu->space(AS_PROGRAM).read_word(0x742c),
				m_cpu->space(AS_PROGRAM).read_word(0x7950), machine().time().as_double()); });
		// cont.262f: reliable terminator-region tap - EVERY write to positions 8-14 ($765c-$7662),
		// so we see the $aa placement (fw $706c) and what over-writes it (the over-scan garbage).
		m_cpu->space(AS_PROGRAM).install_write_tap(0x765c, 0x7663, "termtap",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ if (!storager_getenv("STORAGER_STAKEV") && !storager_getenv("STORAGER_AAFIX")) return;
			// cont.262f (STORAGER_AAFIX): HOLD the $aa END-MARKER at the window boundary
			// (position [$7954]+[$7abc]). The fw's over-scan ($812c) writes aim garbage (07,08..)
			// there, which the completion scan skips ($aa/$ff/$fe only) and over-runs. Force that
			// one byte to $aa so the scan STOPS at the boundary -> $70ba $aa==$aa -> completion.
			if (storager_getenv("STORAGER_AAFIX"))
			{
				address_space &bs = m_cpu->space(AS_PROGRAM);
				u32 const boundary = 0x7654u + bs.read_word(0x7954) + bs.read_word(0x7abc);
				u32 const wa = u32(offset) & ~1u;   // word address of this write (offset is absolute)
				if (wa == (boundary & ~1u))   // this word contains the boundary byte
				{
					if (boundary & 1) { if (mem_mask & 0x00ff) data = (data & 0xff00) | 0x00aa; }
					else              { if (mem_mask & 0xff00) data = (data & 0x00ff) | 0xaa00; }
				}
			}
			if (!storager_getenv("STORAGER_STAKEV")) return;
			double const tt = machine().time().as_double();
			if (tt < 7.99 || tt > 8.10) return; static int _t=0; if (_t++ >= 80) return;
			logerror("TERMWR addr=%04x data=%04x pc=%06x 7954=%04x 7abc=%04x 7428=%04x @%.6f\n",
				u32(offset)&0xffff, data, m_cpu->pc(), m_cpu->space(AS_PROGRAM).read_word(0x7954),
				m_cpu->space(AS_PROGRAM).read_word(0x7abc), m_cpu->space(AS_PROGRAM).read_word(0x7428), tt); });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7428, 0x7429, "aimwide",
			[this](offs_t, u16 &data, u16)
			{ if (!storager_getenv("STORAGER_STAKEV")) return; double const at = machine().time().as_double();
			if (at < 7.955 || at > 8.06) return; static int _a=0; if (_a++ >= 80) return;
			logerror("AIMW <-%04x pc=%06x @%.6f\n", data, m_cpu->pc(), at); });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7950, 0x7951, "phase7950",
			[this](offs_t, u16 &data, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				// cont.262 (STORAGER_C135PH): HOLD the data-record-valid phase externally. While a
				// data-record completion is pending, force [$7950] bit0 = 1 against the hunt's re-arm
				// clears ($88d6/$8a3e) so the phase survives to the fw's data pass. Release when the
				// $298c ID/DATA alternator consumes a phase-1 into the DATA fork (its bchg writes 1->0
				// from that pc) - that IS the $92b4 stake pass; let that clear stand.
				// NB: the fw's phase bit is bit0 of the BYTE at $7950 = bit8 of the word
				// (bchg #0,$7950.w is a byte op on the big-endian high byte); phase=1 is $0101.
				if (storager_getenv("STORAGER_C135PH") && m_c135_pending)
				{
					u32 const wpc = m_cpu->pc();
					bool const alt = (wpc >= 0x298c && wpc <= 0x299a);   // the ID/DATA alternator toggle
					if (alt && !(data & 0x0100))
					{
						m_c135_pending = false;   // alternator forked DATA (old phase 1 -> 0): consumed
						if (storager_getenv("STORAGER_PHASELOG")) { static int _pc = 0; if (_pc++ < 200)
							logerror("C135-CONSUMED (alternator $298c -> DATA fork) @%.6f\n", t); }
					}
					else if (!alt && !(data & 0x0100))
					{
						data |= 0x0101;   // a re-arm is clearing the phase: HOLD it at 1 (high-byte bit0)
						m_cpu->set_input_line(M68K_IRQ_6, HOLD_LINE);
						if (storager_getenv("STORAGER_PHASELOG")) { static int _ph = 0; if (_ph++ < 200)
							logerror("C135-HOLD (phase re-armed to 1, pc=%06x) @%.6f\n", wpc, t); }
					}
				}
				bool const w = (t > 7.960 && t < 7.968) || (t > 7.999 && t < 8.055);
				if (!w || n >= 60) return; n++;
				logerror("PHASE7950 <-%04x pc=%06x @%.6f\n", data, m_cpu->pc(), t); });

		// cont.280 (Dave's DESCTRACE): lock the $748A descriptor contract before the line-758
		// surgery. Installed LATE (after phase7950) on ROM handlers (AS_OPCODES, reliable - the
		// RAM-handler reinstall wipes RAM taps, not ROM). Answers: does the READ reach $3d4e/$3cd4
		// to ARM $748A, or park at 0x36 before arming? Is [$743a]==[$7a14]==$748a at IRQ4 ($3c16)?
		if (storager_getenv("STORAGER_DESCTRACE"))
		{
			auto snap = [this](char const *tag) {
				address_space &s = m_cpu->space(AS_PROGRAM);
				logerror("%s 743a=%04x 7a14=%04x 7a76=%04x 749c=%04x 7a64=%04x 74b4=%04x 72d6=%04x 7454=%04x ph7216=%02x 7424=%04x ~host=%04x%04x n14=%08x n1c=%04x pc=%06x @%.6f\n",
					tag, s.read_word(0x743a), s.read_word(0x7a14), s.read_word(0x7a76), s.read_word(0x749c),
					s.read_word(0x7a64), s.read_word(0x74b4), s.read_word(0x72d6), s.read_word(0x7454),
					s.read_byte(0x7216), s.read_word(0x7424),
					s.read_word(0x748e), s.read_word(0x7490), s.read_dword(0x749e), s.read_word(0x74a6),
					m_cpu->pc(), machine().time().as_double());
			};
			address_space &os = m_cpu->space(AS_OPCODES);
			for (auto pr : { std::pair<u32,char const*>{0x3d4e,"BUILD "},{0x3cd4,"ARM   "},
				{0x159c,"PARK36"},{0x3e30,"UNPARK"},{0x417a,"DESCGO"},
				{0x3c16,"IRQ4id"},{0x1310,"DONE10"},{0x1348,"DONE48"} })
				os.install_read_tap(pr.first, pr.first | 1, pr.second,
					[snap, t = pr.second](offs_t, u16 &, u16) { snap(t); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x158c, 0x158d, "LADOP",
				[this](offs_t, u16 &, u16)
				{ static int ln = 0; double const lt = machine().time().as_double();
					if (lt < 7.9 || ln >= 40) return; ln++;
					address_space &s = m_cpu->space(AS_PROGRAM);
										u32 const a1 = u32(m_cpu->state_int(M68K_A1)) & 0xffff;   // op pointer
					u16 const op = s.read_word(a1) & 0xffff;
						logerror("LADOP 721a=%04x optr=%04x op=%04x hnd=%04x 7956=%04x 7302=%04x 7abc=%04x F000=%04x @%.6f\n",
						s.read_word(0x721a) & 0xffff, a1, op, s.read_word((0x192+op) & 0xffff),
						s.read_word(0x7956), s.read_word(0x7302), s.read_word(0x7abc), s.read_word(0xf000), lt); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x65d8, 0x65d9, "SETTLECHK",
				[this](offs_t,u16&,u16){ double t=machine().time().as_double(); if(t<7.9)return; static int n=0; if(++n<=4) logerror("SETTLECHK reached #%d pc=%06x @%.6f\n",n,m_cpu->pc(),t); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x6612, 0x6613, "TIMERSCHD",
				[this](offs_t,u16&,u16){ double t=machine().time().as_double(); if(t<7.9)return; static int n=0; if(++n<=4) logerror("TIMERSCHD(settle timer armed) #%d @%.6f\n",n,t); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x661a, 0x661b, "SET7NOW",
				[this](offs_t,u16&,u16){ double t=machine().time().as_double(); if(t<7.9)return; static int n=0; if(++n<=4) logerror("SET7a36-immediate($661a) #%d @%.6f\n",n,t); });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7a36, 0x7a37, "W7a36",
				[this](offs_t, u16 &d, u16){ double const t=machine().time().as_double(); if(t<7.9)return;
					static int n=0; if(++n<=6) logerror("W7a36 <- %04x pc=%06x @%.6f\n", d, m_cpu->pc(), t); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x6788, 0x6789, "SEEK",
				[this](offs_t, u16 &, u16)
				{ static int sn = 0; double const st = machine().time().as_double();
					if (st < 7.9 || sn >= 12) return; sn++;
					address_space &s = m_cpu->space(AS_PROGRAM);
					u32 const uib = s.read_word(0x799a) & 0xffff;
					logerror("SEEK uib=%04x uib12=%02x tgtCyl[7946]=%04x tgtHd[7948]=%04x curCyl[+d0]=%04x curHd[+d2]=%04x F000=%04x @%.6f\n",
						uib, s.read_byte((uib+0x12)&0xffff), s.read_word(0x7946), s.read_word(0x7948),
						s.read_word((uib+0xd0)&0xffff), s.read_word((uib+0xd2)&0xffff), s.read_word(0xf000), st); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x683e, 0x683f, "SEEKok",
				[this](offs_t, u16 &, u16)
				{ static int n=0; if (machine().time().as_double()<7.9) return; if(++n<=3) logerror("SEEK->positioned($683e) #%d @%.6f\n", n, machine().time().as_double()); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x67fc, 0x67fd, "SEEKgo",
				[this](offs_t, u16 &, u16)
				{ static int n=0; if (machine().time().as_double()<7.9) return; if(++n<=3) logerror("SEEK->reposition($67fc) #%d @%.6f\n", n, machine().time().as_double()); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x6ba6, 0x6ba7, "SEEKret",
				[this](offs_t, u16 &, u16)
				{ static int n=0; double const t=machine().time().as_double(); if (t<7.9) return; if(++n<=6) logerror("SEEK ret D0=%04x @%.6f\n", u16(m_cpu->state_int(M68K_D0)), t); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x159c, 0x159d, "OP36",
				[this](offs_t, u16 &, u16)
				{ static int on = 0; double const ot = machine().time().as_double();
					if (ot < 7.9 || on >= 30) return; on++;
					address_space &s = m_cpu->space(AS_PROGRAM);
					u32 const a2 = u32(m_cpu->state_int(M68K_A2)) & 0xffff;   // the stamp target
					logerror("OP36 stampA2=%04x 721a=%04x 71c6+26=%02x 71f0+26=%02x @%.6f\n",
						a2, s.read_word(0x721a) & 0xffff, s.read_byte(0x71c6+0x26), s.read_byte(0x71f0+0x26), ot); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x161c, 0x161d, "GATEA",
				[this](offs_t, u16 &, u16)
				{ static int gn = 0; double const gt = machine().time().as_double();
					if (gt < 7.9 || gn >= 40) return; gn++;
					address_space &s = m_cpu->space(AS_PROGRAM);
					u32 const n71b6 = s.read_word(0x71b6) & 0xffff;
					u32 const n71bc = s.read_word(0x71bc) & 0xffff;
					logerror("GATEA[hi] 71b6->%04x[+26=%02x] 71bc->%04x[+26=%02x] @%.6f\n",
						n71b6, s.read_byte((n71b6+0x26)&0xffff), n71bc, s.read_byte((n71bc+0x26)&0xffff), gt); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x162a, 0x162b, "GATEB",
				[this](offs_t, u16 &, u16)
				{ static int gn = 0; double const gt = machine().time().as_double();
					if (gt < 7.9 || gn >= 40) return; gn++;
					address_space &s = m_cpu->space(AS_PROGRAM);
					u32 const n71bc = s.read_word(0x71bc) & 0xffff;
					logerror("GATEB[lo] 71bc->%04x[+26=%02x] @%.6f\n",
						n71bc, s.read_byte((n71bc+0x26)&0xffff), gt); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x1646, 0x1647, "PUMPFOUND",
				[this](offs_t, u16 &, u16)
				{ static int fn = 0; if (machine().time().as_double() < 7.9) return; fn++;
					if (fn <= 5) logerror("PUMPFOUND #%d @%.6f\n", fn, machine().time().as_double()); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x15fe, 0x15ff, "PUMP36",
				[this](offs_t, u16 &, u16)
				{ static int pn = 0; double const pt = machine().time().as_double();
					if (pt < 7.9 || pn >= 60) return; pn++;
					address_space &s = m_cpu->space(AS_PROGRAM);
					u32 const a0 = u32(m_cpu->state_int(M68K_A0)) & 0xffffff;
					u32 const a1 = u32(m_cpu->state_int(M68K_A1)) & 0xffffff;
					u32 const a2 = u32(m_cpu->state_int(M68K_A2)) & 0xffffff;
					logerror("PUMP36 A0=%06x[+0=%04x +2=%04x +4=%04x] A1=%06x[+2=%04x +4=%04x] A2=%06x[+2=%04x +4=%04x] D0=%04x @%.6f\n",
						a0, s.read_word(a0 & 0xffff), s.read_word((a0+2) & 0xffff), s.read_word((a0+4) & 0xffff),
						a1, s.read_word((a1+2) & 0xffff), s.read_word((a1+4) & 0xffff),
						a2, s.read_word((a2+2) & 0xffff), s.read_word((a2+4) & 0xffff),
						u16(m_cpu->state_int(M68K_D0)), pt); });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7216, 0x7217, "ph26",
				[this](offs_t, u16 &d, u16 mm){ if (mm & 0x00ff)
					logerror("PHASE7216 <- %02x pc=%06x @%.6f\n", d & 0xff, m_cpu->pc(), machine().time().as_double()); });
		}
		// cont.282 (Dave's PUMP836): what does the $836c segment-transfer engine wait on at the
		// 0x36 park? Three gates in order: $8370 tst $74b8 (A: next descriptor armed -> $846a ARM),
		// $8378 tst $7b42 (B: segment-complete IRQ -> $83c8 COMPL), $8382 tst $7956 (C: counts ->
		// idle). IRQ6 data-pump vector [$7302]=$8552 is installed ONLY in the arm leg $846a, so
		// pre-arm the flux IRQ6 feeds the ID-walk, not the transfer pump. Watch which gate unsticks
		// first + whether [$74c0]/[$74b8] ever populate + F000 bit2 (segment/drive ready).
		if (storager_getenv("STORAGER_PUMP836"))
		{
			address_space &os = m_cpu->space(AS_OPCODES);
			auto snap = [this](char const *tag) {
				address_space &s = m_cpu->space(AS_PROGRAM);
				logerror("%s 74b8=%04x 7b42=%04x 7956=%04x 74c0=%04x 7abc=%04x 7302=%04x 7a64=%04x "
					"F000=%04x(bit2=%d) 743a=%04x 7a14=%04x ph7216=%02x pc=%06x @%.6f\n",
					tag, s.read_word(0x74b8), s.read_word(0x7b42), s.read_word(0x7956), s.read_word(0x74c0),
					s.read_word(0x7abc), s.read_word(0x7302), s.read_word(0x7a64),
					s.read_word(0xf000), BIT(s.read_word(0xf000), 2), s.read_word(0x743a), s.read_word(0x7a14),
					s.read_byte(0x7216), m_cpu->pc(), machine().time().as_double());
			};
			static int s_cap[8] = {};
			struct site { u32 a; char const *t; };
			int idx = 0;
			for (site pr : { site{0x836c,"P836 "}, site{0x846a,"ARM  "}, site{0x83c8,"COMPL"},
				site{0x4696,"7B42="}, site{0x8552,"PUMP6"}, site{0x70a0,"956--"} })
			{
				os.install_read_tap(pr.a, pr.a | 1, pr.t,
					[this, snap, t = pr.t, i = idx](offs_t, u16 &, u16) {
						if (machine().time().as_double() < 7.9) return;
						if (++s_cap[i] > 200) return;
						snap(t); });
				idx++;
			}
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7b42, 0x7b43, "w7b42",
				[this](offs_t, u16 &d, u16){ logerror("W7B42 <- %04x pc=%06x @%.6f\n", d, m_cpu->pc(),
					machine().time().as_double()); });
		}
		// cont.283 (Dave's UNPARK): why doesn't [$7956]==0 reach the $8460 unpark? Run on BASELINE
		// (FAITHXFER off) where [$7956] does drain 8->0. Unpark at $845c/$8460 fires the instant
		// [$7956]==0 && [$7958]==0 && [$72e2]==0 -> +$26=$c on [$71bc]. F000 bit2 is on the $846a
		// still-working branch, NOT a completion gate. Three outcomes: PUMPH never fires (handler
		// off-route), PUMPH fires w/ [$7958]/[$72e2] nonzero (those are the gate), or UNPARK fires
		// but hangs (block is downstream $3dbc/$1a54).
		if (storager_getenv("STORAGER_UNPARK"))
		{
			address_space &os = m_cpu->space(AS_OPCODES);
			auto snap = [this](char const *tag) {
				address_space &s = m_cpu->space(AS_PROGRAM);
				logerror("%s 7956=%04x 7958=%08x 72e2=%04x 791a=%04x 7a62=%04x ph7216=%02x pc=%06x @%.6f\n",
					tag, s.read_word(0x7956), s.read_dword(0x7958), s.read_word(0x72e2), s.read_word(0x791a),
					s.read_word(0x7a62), s.read_byte(0x7216), m_cpu->pc(), machine().time().as_double());
			};
			static int s_ucap[4] = {};
			struct usite { u32 a; char const *t; };
			int uidx = 0;
			for (usite pr : { usite{0x8416,"PUMPH "}, usite{0x8428,"g7958 "}, usite{0x845c,"UNPARK"}, usite{0x846a,"WORK  "} })
			{
				os.install_read_tap(pr.a, pr.a|1, pr.t,
					[snap, t = pr.t, i = uidx](offs_t,u16&,u16){ if (s_ucap[i]++ < 300) snap(t); });
				uidx++;
			}
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7956, 0x7957, "w7956",
				[this](offs_t,u16&d,u16){ if((d & 0xffff)==0) logerror("W7956 <- 0 pc=%06x @%.6f\n", m_cpu->pc(),
					machine().time().as_double()); });
		}
		// cont.284 (Dave's LADWAIT): which ladder op is the read actually stuck on, and what
		// completion cell does the $6bc2 (op-42) wait poll? Plus whether [$71b6] (record-eligibility,
		// installed at $1144 only for IOPB options bit4) is ever set for this options-0x00 read.
		// Run on BASELINE.
		if (storager_getenv("STORAGER_LADWAIT"))
		{
			auto snap = [this](char const *tag) {
				address_space &s = m_cpu->space(AS_PROGRAM);
				logerror("%s 7424=%04x 7a3e=%04x 7a40=%04x 7a36=%04x 71b6=%04x 7286=%04x ph7216=%04x "
					"7956=%04x pc=%06x @%.6f\n",
					tag, s.read_word(0x7424), s.read_word(0x7a3e), s.read_word(0x7a40), s.read_word(0x7a36),
					s.read_word(0x71b6), s.read_word(0x7286), s.read_word(0x7216), s.read_word(0x7956),
					m_cpu->pc(), machine().time().as_double());
			};
			m_cpu->space(AS_OPCODES).install_read_tap(0x6bc2, 0x6bc3, "OP42",
				[snap,this](offs_t,u16&,u16){ if(machine().time().as_double()<7.9)return; static int c=0; if(c++<400) snap("OP42 "); });
			m_cpu->space(AS_OPCODES).install_read_tap(0x156a, 0x156b, "WALK",
				[snap,this](offs_t,u16&,u16){ if(machine().time().as_double()<7.9)return; static int c=0; if(c++<400) snap("WALK "); });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x71b6, 0x71b7, "w71b6",
				[this](offs_t,u16&d,u16){ logerror("W71B6 <- %04x pc=%06x @%.6f\n", d, m_cpu->pc(), machine().time().as_double()); });
			m_cpu->space(AS_PROGRAM).install_write_tap(0x7a3e, 0x7a41, "w7a3e",
				[this](offs_t o,u16&d,u16){ logerror("W%04x <- %04x pc=%06x @%.6f\n", unsigned(o), d, m_cpu->pc(), machine().time().as_double()); });
		}
		// cont.285 (Dave's REDISP): after op-36 parks (phase $A), does the dispatcher $2250 loop
		// back and dispatch $A (run the pump), or exit to the main loop? What re-enters it? Run on
		// BASELINE. Outcome A: DISP-LOOP/PH-READ fire after PARK36 but PUMP doesn't -> phase-cell
		// placement bug (pure state fix). Outcome B: DISP-LOOP stops after PARK36 -> dispatcher
		// exits, command off-list; need the re-queue trigger the gate array returns.
		if (storager_getenv("STORAGER_REDISP"))
		{
			auto snap = [this](char const *tag) {
				address_space &s = m_cpu->space(AS_PROGRAM);
				logerror("%s ph7216=%04x 7424=%04x cmd=%02x pc=%06x @%.6f\n",
					tag, s.read_word(0x7216), s.read_word(0x7424), m_iopb_cmd, m_cpu->pc(), machine().time().as_double());
			};
			static int s_rcap[6] = {};
			struct rsite { u32 a; char const *t; };
			int ridx = 0;
			for (rsite pr : { rsite{0x2244,"DISP-ENTER"}, rsite{0x2250,"DISP-LOOP "}, rsite{0x2258,"PH-READ   "},
				rsite{0x159c,"PARK36    "}, rsite{0x15fe,"PUMP      "} })
			{
				m_cpu->space(AS_OPCODES).install_read_tap(pr.a, pr.a|1, pr.t,
					[snap, t = pr.t, i = ridx, this](offs_t,u16&,u16){ if(machine().time().as_double()<7.9)return; if(s_rcap[i]++ < 250) snap(t); });
				ridx++;
			}
		}
		// cont.286 (Dave's OPTBIT4, adapted): the [$71b6] install ($1144) is gated by btst #4,D5
		// where D5=[UIB+$20], written from D6 = the $92 op-descriptor FLAGS word (A1=$92+cmd*4) --
		// so bit4 is a per-COMMAND ROM flag, NOT a host-IOPB options field. Observe whether the boot
		// read command's descriptor sets bit4 (-> high-gate/[$71b6] install) or clears it (-> low gate).
		if (storager_getenv("STORAGER_OPTBIT4"))
		{
			// $0e0e: btst #4,D6 (D6 = the $92-descriptor flags for this command)
			m_cpu->space(AS_OPCODES).install_read_tap(0x0e0e, 0x0e0f, "DESC-BIT4",
				[this](offs_t,u16&,u16){ static int c=0; if(c++<24){
					u16 d6 = u16(m_cpu->state_int(M68K_D6));
					logerror("DESC-BIT4 cmd=%02x D6=%04x bit4=%d D1=%04x pc=%06x @%.6f\n",
						m_iopb_cmd, d6, BIT(d6,4), u16(m_cpu->state_int(M68K_D1)), m_cpu->pc(), machine().time().as_double()); } });
			// $113c: btst #4,D5 (D5 = [UIB+$20]) -- the actual $71b6-install guard
			m_cpu->space(AS_OPCODES).install_read_tap(0x113c, 0x113d, "INST-BIT4",
				[this](offs_t,u16&,u16){ static int c=0; if(c++<24){
					u16 d5 = u16(m_cpu->state_int(M68K_D5));
					logerror("INST-BIT4 cmd=%02x D5=%04x bit4=%d -> %s71b6 pc=%06x @%.6f\n",
						m_iopb_cmd, d5, BIT(d5,4), BIT(d5,4)?"INSTALL ":"SKIP ", m_cpu->pc(), machine().time().as_double()); } });
			// confirm the install actually runs (or not)
			m_cpu->space(AS_OPCODES).install_read_tap(0x1144, 0x1145, "INSTALL1144",
				[this](offs_t,u16&,u16){ static int c=0; if(c++<8){ address_space &s=m_cpu->space(AS_PROGRAM);
					logerror("INSTALL1144 (71b6<-A3) cmd=%02x 71b6=%04x pc=%06x @%.6f\n",
						m_iopb_cmd, s.read_word(0x71b6), m_cpu->pc(), machine().time().as_double()); } });
		}
		// cont.287 (Dave's XFERDRAIN): the read's completion is $6f44 (fill-map consumer), NOT the
		// pump. $6f44 builds the xfer queue from POSITIVE ledger slots, $702e D3=blocks placed,
		// $70a0 sub.w D3,$7956, $70a6 (if $7956==0) $7a64=1, $70f4 bsr $3dbc -> $3ef6 activates
		// $7286 + $3e30 posts +$26=$c = UNPARK. Question: does $70a0 ever land $7956==0 with real
		// D3, or does the $7c34/$7ebe walk pre-drain $7956 and steal the zero? Run on BASELINE.
		if (storager_getenv("STORAGER_XFERDRAIN"))
		{
			auto snap = [this](char const *tag) {
				address_space &s = m_cpu->space(AS_PROGRAM);
				int pos = 0; for (int k = 1; k <= 8; k++) if (!(s.read_byte(0x7654 + k) & 0x80)) pos++;
				logerror("%s 7956=%04x 7a64=%04x fillpos=%d pc=%06x @%.6f\n",
					tag, s.read_word(0x7956), s.read_word(0x7a64), pos, m_cpu->pc(), machine().time().as_double());
			};
			static int s_xcap[6] = {};
			struct xsite { u32 a; char const *t; };
			int xidx = 0;
			for (xsite pr : { xsite{0x6f44,"BUILD "}, xsite{0x702e,"XFER  "}, xsite{0x70a0,"DRAIN "},
				xsite{0x70a6,"WIN64 "}, xsite{0x7ebe,"WALK--"} })
			{
				m_cpu->space(AS_OPCODES).install_read_tap(pr.a, pr.a|1, pr.t,
					[snap, t = pr.t, i = xidx](offs_t,u16&,u16){ if(s_xcap[i]++ < 300) snap(t); });
				xidx++;
			}
		}
		// cont.288 (Dave's R7426): the single completion gate. $808a: tst [$7426]; ==0 -> $810e
		// (dead-end per-sector closer), !=0 -> ... -> $82e2 (ledger-scan ISR) -> $7106/$6f44 re-run
		// (full map, [$7956]=0, [$7a64]=1) -> $3dbc UNPARK. [$7426]=1 is set at $7cac ($fe-terminator
		// ID-verify tail) or $7d62 (walk generic tail); Gate 2 clears it at $7f1a and never reaches
		// $7d62. Which setter should the read hit at the window boundary? Run on BASELINE.
		if (storager_getenv("STORAGER_R7426"))
		{
			auto snap = [this](char const *t){ address_space &s = m_cpu->space(AS_PROGRAM);
				int p = 0; for (int k = 1; k <= 8; k++) if (!(s.read_byte(0x7654 + k) & 0x80)) p++;
				logerror("%s 7426=%04x 7daf=%02x 7dac=%02x aim7428=%04x fillpos=%d pc=%06x @%.6f\n",
					t, s.read_word(0x7426), s.read_byte(0x7daf), s.read_byte(0x7dac), s.read_word(0x7428),
					p, m_cpu->pc(), machine().time().as_double()); };
			for (auto wp : { std::pair<u32,char const*>{0x7c34,"WALK34"},{0x7ce6,"P-7ce6"},{0x7c52,"MATCH52"},{0x7c50,"MISS50"} })
				m_cpu->space(AS_OPCODES).install_read_tap(wp.first, wp.first|1, wp.second,
					[this,t=wp.second](offs_t,u16&,u16){ static int c=0; if(c++<500){ address_space &s=m_cpu->space(AS_PROGRAM);
						logerror("%s 7436=%04x 7daf=%02x 7dac=%02x @%.6f\n", t, s.read_word(0x7436), s.read_byte(0x7daf), s.read_byte(0x7dac), machine().time().as_double()); } });
			m_cpu->space(AS_OPCODES).install_read_tap(0x7c70, 0x7c71, "DEC7c70",
				[this](offs_t,u16&,u16){ static int c=0; if(c++<60){ address_space &s=m_cpu->space(AS_PROGRAM);
					int p=0; for(int k=1;k<=8;k++) if(!(s.read_byte(0x7654+k)&0x80)) p++;
					logerror("DEC7c70 7daf=%02x 7dac=%02x (fe->7cac ACCEPT / else 7d62) fillpos=%d aim=%04x @%.6f\n",
						s.read_byte(0x7daf), s.read_byte(0x7dac), p, s.read_word(0x7428), machine().time().as_double()); } });
			static int s_gcap[6] = {};
			struct gsite { u32 a; char const *t; };
			int gidx = 0;
			for (gsite pr : { gsite{0x808a,"GATE808a"}, gsite{0x7cac,"SET-7cac"}, gsite{0x7d62,"SET-7d62"},
				gsite{0x82e2,"SCAN82e2"}, gsite{0x810e,"DEAD810e"} })
			{
				m_cpu->space(AS_OPCODES).install_read_tap(pr.a, pr.a|1, pr.t,
					[snap, t = pr.t, i = gidx](offs_t,u16&,u16){ if(s_gcap[i]++ < 300) snap(t); });
				gidx++;
			}
		}
		// cont.123 (STRIP): the $92b4 INVOCATION ROUTE - the toggler bank ($2970/$297e/
		// $298c: bchg #0,$7950; old-bit routes ID/DATA) + [$7940]/[$7950] state. Read1's
		// set->consume pair is $7eb2 -> $92b4 (24us); read2's four sets go unconsumed.
		// Which dispatch runs the toggler for read1 and not read2?
		for (offs_t tpc : { offs_t(0x2970), offs_t(0x297e), offs_t(0x298c), offs_t(0x299a) })
			m_cpu->space(AS_OPCODES).install_read_tap(tpc, tpc | 1, "toggler",
				[this, tpc](offs_t, u16 &, u16)
				{ static int n = 0; double const t = machine().time().as_double();
					bool const w = (t > 7.960 && t < 7.968) || (t > 7.999 && t < 8.055);
					if (!w || n >= 60) return; n++;
					address_space &ds = m_cpu->space(AS_PROGRAM);
					logerror("TOGGLER %04x 7950=%04x 7940=%04x 742c=%04x @%.6f\n", unsigned(tpc),
						ds.read_word(0x7950), ds.read_word(0x7940), ds.read_word(0x742c), t); });
		// cont.121 (STRIP, Dave's spec): THE BIFURCATING TAP SET. Invariant: read2 exits
		// correctly <=> at the instant [$7956] (remaining count) reaches 0, $32ac returns
		// "no want" ($7968=0 -> $7eb2 -> $742c=1 -> pull -> f0 -> $ba). It currently
		// returns one. Two sources only: the segment queue ($32d0, [$74ac]!=0 - over-
		// delivered ID captures) or the ledger scan ($3320 - the DATA completion never
		// retired the want). The $32ac branch on the $7956==0 pass IS the whole report.
		{
			auto qtap = [this](offs_t pc, char const *tag)
			{
				m_cpu->space(AS_OPCODES).install_read_tap(pc, pc | 1, tag,
					[this, tag](offs_t, u16 &, u16)
					{ static int n = 0; double const t = machine().time().as_double();
						bool const w = (t > 7.955 && t < 7.975) || (t > 7.995 && t < 8.30);
						if (!w || n >= 120) return; n++;
						address_space &ds = m_cpu->space(AS_PROGRAM);
						logerror("POPSRC %s 7956=%04x 74ac=%04x 7968=%04x @%.6f\n", tag,
							ds.read_word(0x7956), ds.read_word(0x74ac), ds.read_word(0x7968), t); });
			};
			qtap(0x32d0, "QUEUE");    // pop served from the segment queue
			qtap(0x3320, "LEDGER");   // pop fell through to the ledger scan
			qtap(0x7e8a, "FORK");     // the fork outcome site ($7968 at decision)
		}
		// cont.125 (STRIP, Dave's fork -> QUEUE arm): THE ENTRY-FIELDS TAPE - the pop
		// advances +8 by arithmetic; the terminator is an entry-CONTENT test. Dump the
		// head entry's 8 bytes + the NEXT entry's 8 at every QUEUE pop, and watch the
		// pool's writers: who validates entry 5's fields, and when.
		{
			m_cpu->space(AS_OPCODES).install_read_tap(0x32d0, 0x32d1, "entryfields",
				[this](offs_t, u16 &, u16)
				{ static int n = 0; double const t = machine().time().as_double();
					bool const w = (t > 7.960 && t < 7.968) || (t > 7.999 && t < 8.190);
					if (!w || n >= 20) return; n++;
					address_space &ds = m_cpu->space(AS_PROGRAM);
					u16 const hd = ds.read_word(0x74ac);
					std::string e1, e2;
					for (int k = 0; k < 8; k++) e1 += util::string_format(" %02x", ds.read_byte((hd + k) & 0xffff));
					for (int k = 0; k < 8; k++) e2 += util::string_format(" %02x", ds.read_byte((hd + 8 + k) & 0xffff));
					logerror("ENTRYF hd=%04x [%s ] next[%s ] @%.6f\n", hd, e1.c_str(), e2.c_str(), t); });
			// pool-field writers: entries 5-6 live at $74f4-$7503 (pool base $74c4 + 6*8..7*8)
			m_cpu->space(AS_PROGRAM).install_write_tap(0x74f4, 0x7503, "poolwr",
				[this](offs_t offset, u16 &data, u16)
				{ static int n = 0; double const t = machine().time().as_double();
					if (t < 7.95 || t > 8.07 || n >= 40) return; n++;
					logerror("POOLWR [%04x]<-%04x pc=%06x @%.6f\n", offset, data, m_cpu->pc(), t); });
		}
		for (auto wc : { std::pair<u16, char const *>{0x7956, "CNT7956"}, {0x79a8, "OWED79A8"},
				{0x74ac, "PUSH74AC"}, {0x74ae, "PUSH74AE"} })
			m_cpu->space(AS_PROGRAM).install_write_tap(wc.first, wc.first | 1, wc.second,
				[this, wc](offs_t, u16 &data, u16)
				{ static int n = 0; double const t = machine().time().as_double();
					bool const w = (t > 7.955 && t < 7.975) || (t > 7.995 && t < 8.30);
					if (!w || n >= 80) return; n++;
					logerror("%s <-%04x pc=%06x @%.6f\n", wc.second, data, m_cpu->pc(), t); });
		// cont.120 (STRIP): THE PULL-PATH DIFFERENTIAL - $2964 = the walk's fork (tst $741c
		// -> $804c/$7f6c), $7f6c/$804c = the branch targets, $7fc6 = the data-pull leg
		// ($2af/$2ff/pull). Read1's launch (7.9634+) vs read2's armed windows (8.000-8.052):
		// never-branches (control flow) vs pulls-into-silence (E000 stream serving).
		// cont.118 (STRIP): the delivery chain after the (now working) data-typed
		// completion - IRQ5 hard handler ($26a8) + the $92xx handler-family entries.
		// Read1's completion reaches $92f6; read2's IRQ5s at 7.9999+/8.0012+ die where?
		for (offs_t hpc : { offs_t(0x26a8), offs_t(0x925a), offs_t(0x92a8), offs_t(0x92b4) })
			m_cpu->space(AS_OPCODES).install_read_tap(hpc, hpc | 1, "dlvchain",
				[this, hpc](offs_t, u16 &, u16)
				{ static int n = 0; double const t = machine().time().as_double();
					if (n >= 60 || t < 7.95 || t > 8.30) return; n++;
					logerror("DLVCHAIN %04x sr=%04x 742c=%04x @%.6f\n", unsigned(hpc),
						unsigned(m_cpu->state_int(M68K_SR)) & 0xffff,
						m_cpu->space(AS_PROGRAM).read_word(0x742c), t); });
		// cont.115 (STRIP): THE FLAG'S BIOGRAPHY - [$742c] gates the f0-stake (all three
		// $92f6 entries: tst $742c; bne). Setters: $7162 (the $7106-stocker leg), $7bf6,
		// $7ea4/$7eb2 (walk exits, routed by [$79b6]/$aa), $95da. Clears: $7ca8/$7e42/$7e90.
		// Who sets it for read1's launch (pre-7.96358) and what read2's era does instead.
		for (u16 cell : { u16(0x742c), u16(0x79b6) })
			m_cpu->space(AS_PROGRAM).install_write_tap(cell, cell | 1, cell == 0x742c ? "flag742c" : "flag79b6",
				[this, cell](offs_t, u16 &data, u16)
				{ static int n = 0; double const t = machine().time().as_double();
					if (n >= 60 || t > 8.30) return; n++;
					logerror("FLAGWR %04x<-%04x pc=%06x @%.5f\n", cell, data, m_cpu->pc(), t); });
		// cont.114 (STRIP): READ1'S REAL COMPLETION LEG - $92f6 = the data-completion
		// handler ($f0 staker). DATASTAGE's zero-count falsified the assumed data-typed-
		// capture leg; catch the actual path: entry context (stack window - may be a branch
		// target, not a call), the IRQ soft-vectors, the capture-engine state, and the
		// live E802/E000 at the moment. Windows-from-zero.
		m_cpu->space(AS_OPCODES).install_read_tap(0x92f6, 0x92f7, "f0leg",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (n >= 25 || t > 8.30) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				u32 const sp = m_cpu->state_int(M68K_SP);
				logerror("F0LEG sp=%06x [%04x %04x %04x %04x %04x %04x] vec7300=%04x vec7304=%04x idcap=%d idtyp=%d dmode=%d seen=%d e802=%04x e000mux=%d 7428=%04x @%.5f\n",
					sp, ds.read_word(sp), ds.read_word(sp + 2), ds.read_word(sp + 4),
					ds.read_word(sp + 6), ds.read_word(sp + 8), ds.read_word(sp + 10),
					ds.read_word(0x7300), ds.read_word(0x7304),
					m_idcap_pending ? 1 : 0, m_idcap_id_typed ? 1 : 0, m_endec_data_mode ? 1 : 0,
					m_seen_id ? 1 : 0, m_ch[(0xe802 - 0xe000) / 2], m_serdes_active ? 1 : 0,
					ds.read_word(0x7428), t); });
		// cont.112 (STRIP): THE FINAL CELL - $17f8 = the c-stamp (completion). Caller stack
		// + every count/consumed candidate at the stamp. Read1 fires ~7.96505; read2 never.
		m_cpu->space(AS_OPCODES).install_read_tap(0x17f8, 0x17f9, "cstamp",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (n >= 30 || t > 8.30) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				u32 const sp = m_cpu->state_int(M68K_SP);
				logerror("CSTAMP A0=%06x ret=[%04x %04x %04x %04x] 79a4=%04x 7a0c=%04x 7966=%04x 71b2=%04x 79a2=%04x @%.5f\n",
					unsigned(m_cpu->state_int(M68K_A0)) & 0xffffff,
					ds.read_word(sp), ds.read_word(sp + 2), ds.read_word(sp + 4), ds.read_word(sp + 6),
					ds.read_word(0x79a4), ds.read_word(0x7a0c), ds.read_word(0x7966),
					ds.read_word(0x71b2), ds.read_word(0x79a2), t); });
		// cont.111b: $3320 was the UNINIT branch ([$74ac] holds the pool base, nonzero since
		// boot - beq never takes; sixth wrong site). THE ANSWER lives in the $7ee0 caller's
		// frame: flag at A3-6, index at A3-8, tested at $7eee. Tap the test; read the frame.
		m_cpu->space(AS_OPCODES).install_read_tap(0x7eee, 0x7eef, "seganswer",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (n >= 50 || t > 8.30) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				u32 const a3 = m_cpu->state_int(M68K_A3);
				std::string led;
				for (int k = 0; k < 18; k++) led += util::string_format(" %02x", ds.read_byte(0x7654 + k));
				logerror("SEGANSWER flag=%04x idx=%04x 74ac=%04x 7428=%04x 79a4=%04x 7a0c=%04x 79a2=%04x |%s @%.5f\n",
					ds.read_word((a3 - 6) & 0xffffff), ds.read_word((a3 - 8) & 0xffffff),
					ds.read_word(0x74ac), ds.read_word(0x7428), ds.read_word(0x79a4),
					ds.read_word(0x7a0c), ds.read_word(0x79a2), led.c_str(), t); });
		// cont.110 (STRIP): THE RELEASE TRACE - $3270 = the per-entry release (clear +
		// unlink via $352e, keyed [$7926]). Windows-from-zero. Read1's releases (when,
		// how many, what drives them) vs read2's era (expected zero) = the differential
		// that names the consumption event the model owes the encore.
		m_cpu->space(AS_OPCODES).install_read_tap(0x3270, 0x3271, "qrelease",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (n >= 60 || t > 8.30) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				u32 const sp = m_cpu->state_int(M68K_SP);
				logerror("QRELEASE 7926=%04x 74ac=%04x ret=%04x%04x @%.5f\n",
					ds.read_word(0x7926), ds.read_word(0x74ac),
					ds.read_word(sp), ds.read_word(sp + 2), t); });
		// cont.109: the four known caller sites, individually - which fires, when, guard state.
		for (offs_t cpc : { offs_t(0x8e6), offs_t(0x1922), offs_t(0xa3be), offs_t(0xa4b2) })
			m_cpu->space(AS_OPCODES).install_read_tap(cpc, cpc | 1, "qcaller",
				[this, cpc](offs_t, u16 &, u16)
				{ static int n = 0; double const t = machine().time().as_double();
					if (n >= 40 || t > 8.30) return; n++;
					logerror("QCALLER %04x 791a=%04x @%.5f\n", unsigned(cpc),
						m_cpu->space(AS_PROGRAM).read_word(0x791a), t); });
		// cont.102 (STRIP): THE CLOSE SITE - $8a48 = the batch-close's sustained disarm
		// (andi #$67ff on the [$79f8] shadow, cont.44). Read1's visits carry the trigger's
		// return stack; read2's absence + that context = the missing event, directly.
		m_cpu->space(AS_OPCODES).install_read_tap(0x8a48, 0x8a49, "closesite",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (t < 6.40 || t > 8.20 || n >= 20) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				u32 const sp = m_cpu->state_int(M68K_SP);
				logerror("CLOSESITE ret=[%04x %04x %04x %04x %04x %04x] 79a4=%04x 7a0c=%04x 741c=%04x @%.5f\n",
					ds.read_word(sp), ds.read_word(sp + 2), ds.read_word(sp + 4),
					ds.read_word(sp + 6), ds.read_word(sp + 8), ds.read_word(sp + 10),
					ds.read_word(0x79a4), ds.read_word(0x7a0c), ds.read_word(0x741c), t); });
		// cont.101 (STRIP): THE BATCH-INIT TRIGGER - who invokes the $8480-$84c6 init for
		// read1, and why never for read2. Tap both entries, all visits, with return stack.
		for (offs_t entry : { offs_t(0x846a), offs_t(0x8480) })
			m_cpu->space(AS_OPCODES).install_read_tap(entry, entry | 1, entry == 0x846a ? "init846a" : "init8480",
				[this, entry](offs_t, u16 &, u16)
				{ static int n = 0; double const t = machine().time().as_double();
					if (t < 6.40 || t > 8.06 || n >= 30) return; n++;
					address_space &ds = m_cpu->space(AS_PROGRAM);
					u32 const sp = m_cpu->state_int(M68K_SP);
					logerror("BATCHINIT@%04x ret=[%04x %04x %04x %04x] 74b8=%04x 791a=%04x 7a62=%04x @%.5f\n",
						unsigned(entry), ds.read_word(sp), ds.read_word(sp + 2), ds.read_word(sp + 4),
						ds.read_word(sp + 6), ds.read_word(0x74b8), ds.read_word(0x791a),
						ds.read_word(0x7a62), t); });
		// cont.100 (STRIP): [$7430]'s biography - the 0044's writer pc closes the limit's
		// provenance the way $73ac closed [$7a62]'s.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7430, 0x7431, "lim7430",
			[this](offs_t, u16 &data, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (t > 7.960 && t < 8.010 && n < 40)
				{ n++;
					logerror("LIM7430 <-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), t); } });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7a62, 0x7a63, "cnt7a62",
			[this](offs_t, u16 &data, u16)
			{ static u16 last = 0xbeef; static int n = 0; double const t = machine().time().as_double();
				if (t > 6.40 && t < 8.06 && data != last && n < 50)
				{ n++;
					logerror("CNT7A62 <-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), t); }
				last = data; });
		m_cpu->space(AS_OPCODES).install_read_tap(0x849a, 0x849b, "segload",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (t < 6.40 || n >= 25) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				std::string tab;
				for (int k = 0; k < 18; k += 2) tab += util::string_format(" %04x", ds.read_word(0x7696 + k));
				logerror("SEGLOAD A0=%06x entry+0=%04x +2=%04x +4=%04x 74b8=%04x 7424=%04x | 7696tab:%s @%.5f\n",
					unsigned(m_cpu->state_int(M68K_A0)) & 0xffffff,
					ds.read_word(unsigned(m_cpu->state_int(M68K_A0)) & 0xffff),
					ds.read_word((unsigned(m_cpu->state_int(M68K_A0)) + 2) & 0xffff),
					ds.read_word((unsigned(m_cpu->state_int(M68K_A0)) + 4) & 0xffff),
					ds.read_word(0x74b8), ds.read_word(0x7424), tab.c_str(), t); });
		// cont.96 (STRIP): THE JUDGMENT DUMP - at each walk entry (OPH2 $7ba8) in read2's
		// R=01 era, the compare's full input set: the staged record, the want cells, and the
		// $7654 consumed-LEDGER (prime suspect: read1's pass left sector 1 marked consumed;
		// the walk declines a re-presented sector).
		m_cpu->space(AS_OPCODES).install_read_tap(0x7ba8, 0x7ba9, "judgment",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				// cont.97: both eras - read1's SUCCESSFUL close (7.955-7.975) vs read2's
				// fe-grind (8.045-8.078).
				bool const w = (t > 8.44 && t < 8.97);   // cont.215: read-B's starve era (accepts 5-8)
				if (!w || n >= 40) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				std::string rec, led, rb;
				for (int k = 0; k < 8; k++) rec += util::string_format("%02x", ds.read_byte(0x7dac + k));
				for (int k = 0; k < 18; k++) led += util::string_format(" %02x", ds.read_byte(0x7654 + k));
				u16 const a66 = ds.read_word(0x7a66);
				for (int k = 0; k < 8; k++) rb += util::string_format("%02x", ds.read_byte((a66 + k) & 0xffff));
				logerror("JUDGE rec=%s 7a66=%04x [[7a66]]=%s 7426=%04x want 7428=%04x 7430=%04x | led:%s @%.5f\n",
					rec.c_str(), a66, rb.c_str(), ds.read_word(0x7426), ds.read_word(0x7428),
					ds.read_word(0x7430), led.c_str(), t); });
		// cont.92c THE MOVIE: teardown tap at the REAL entry $1b48 ($1b42 was the previous
		// function's epilogue - the run231 "caught nothing" was tap placement, not absence).
		for (auto site : { std::pair<offs_t, char const *>{0xee4, "ACCEPTQ"}, {0x1b48, "TEARDOWN"} })
			m_cpu->space(AS_OPCODES).install_read_tap(site.first, site.first | 1, site.second,
				[this, site](offs_t, u16 &, u16)
				{ static int n = 0; double const t = machine().time().as_double();
					bool const w = (t > 6.400 && t < 6.420) || (t > 7.960 && t < 8.000) || (t > 8.170 && t < 8.190);
					if (!w || n >= 40) return; n++;
					address_space &ds = m_cpu->space(AS_PROGRAM);
					u32 const sp = m_cpu->state_int(M68K_SP);
					logerror("%s pc=%06x D5=%04x D0=%04x A0=%06x 71bc=%04x 71b6=%04x 71b2=%04x 743a=%04x ret=[%04x %04x %04x %04x] @%.5f\n",
						site.second, m_cpu->pc(), unsigned(m_cpu->state_int(M68K_D5)) & 0xffff,
						unsigned(m_cpu->state_int(M68K_D0)) & 0xffff,
						unsigned(m_cpu->state_int(M68K_A0)) & 0xffffff,
						ds.read_word(0x71bc), ds.read_word(0x71b6), ds.read_word(0x71b2), ds.read_word(0x743a),
						ds.read_word(sp), ds.read_word(sp + 2), ds.read_word(sp + 4), ds.read_word(sp + 6), t); });
		// cont.92c THE MOVIE (STRIP): every +26 status transition on BOTH nodes, with pc -
		// node 71c6's cell = $71ec, node 71f0's cell = $7216. Attribution before absence:
		// the healthy 6.40-6.42 contrast shows who stamps the successor's 4; the fatal
		// 8.17-8.19 window shows that pc not firing / misfiring / firing late.
		for (u16 cell : { u16(0x71ec), u16(0x7216) })
			m_cpu->space(AS_PROGRAM).install_write_tap(cell, cell | 1, cell == 0x71ec ? "st71c6" : "st71f0",
				[this, cell](offs_t, u16 &data, u16)
				{ static int n = 0; double const t = machine().time().as_double();
					bool const w = (t > 6.400 && t < 6.420) || (t > 7.960 && t < 8.000) || (t > 8.170 && t < 8.190);
					if (!w || n >= 80) return; n++;
					logerror("STATMOVIE node=%s +26<-%04x pc=%06x @%.5f\n",
						cell == 0x71ec ? "71c6" : "71f0", data, m_cpu->pc(), t); });
		// cont.89 (STRIP): THE HEAD-EMPTIER - every write to the dispatcher queue heads
		// [$71bc]/[$71b6], pc+value. cont.255: UNCAPPED full-boot (test (a) of cont.254c -
		// hunting a THIRD [$71bc] writer beyond the $0EEC install / $1B6A clear; a repoint
		// to a node whose +$26 byte is 0x0A would be the discovery). Safety cap only.
		for (u16 cell : { u16(0x71bc), u16(0x71b6) })
			m_cpu->space(AS_PROGRAM).install_write_tap(cell, cell | 1, cell == 0x71bc ? "head71bc" : "head71b6",
				[this, cell](offs_t, u16 &data, u16)
				{ static int n = 0; if (n >= 400) return; n++;
					address_space &ds = m_cpu->space(AS_PROGRAM);
					logerror("HEADWR %04x<-%04x pc=%06x +26=%04x @%.5f\n", cell, data, m_cpu->pc(),
						data ? ds.read_word((data + 0x26) & 0xffff) : 0xdead, machine().time().as_double()); });
		// TEMP cont.255b (STRIP): the phase-cell taps, installed LATE so handler reinstalls
		// can't wipe them (see the dead early w7216 tap above). BOTH work nodes' phase
		// cells: $7216 ($71F0+$26) and $71EC ($71C6+$26). PHASEHI mutation kept on $7216.
		for (u16 pcell : { u16(0x7216), u16(0x71ec) })
			m_cpu->space(AS_PROGRAM).install_write_tap(pcell, pcell | 1, pcell == 0x7216 ? "w7216b" : "w71ecb",
				[this, pcell](offs_t, u16 &data, u16 mem_mask)
				{
					if (pcell == 0x7216 && storager_getenv("STORAGER_PHASEHI") && data == 0x000a && mem_mask == 0xffff)
						data = 0x0a0a;
					if ((data & mem_mask) & 0xff00)
						logerror("WPHASEHI %04x <- %04x mm=%04x pc=%06x @%.6f\n", pcell, data, mem_mask, m_cpu->pc(), machine().time().as_double());
					static int pw = 0; if (storager_getenv("STORAGER_IAMRD") && pw++ < 160)
						logerror("WPHASE %04x <- %04x mm=%04x pc=%06x @%.6f\n", pcell, data, mem_mask, m_cpu->pc(), machine().time().as_double()); });
		// TEMP cont.256k (STRIP): op-24's HEAD-LOAD DELAY. $65E8 reads (A1+$14)&$0F as
		// the delay in timer ticks/10; ZERO takes $661C = set [$7A36] immediately (no
		// wait), non-zero schedules a poke via $29F8 that only IRQ1 can deliver. Our
		// IRQ1 tick is ~26ms (pit1 ctr0, 10MHz/4, count $FF00), so a non-zero delay
		// costs ~1.5s and read1 times out before it ever reaches the data phase.
		// Log A1, the delay byte and D0 to see whether the parameter is the fault.
		m_cpu->space(AS_OPCODES).install_read_tap(0x65e8, 0x65e9, "hldelay",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; if (n++ >= 12) return;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				u16 const a1 = u16(m_cpu->state_int(M68K_A1));
				logerror("HLDELAY A1=%04x (A1+14)=%02x low4=%u uib=%04x uib+14=%02x 7a36=%04x 7a38=%04x @%.6f\n",
					a1, ds.read_byte((a1 + 0x14) & 0xffff), ds.read_byte((a1 + 0x14) & 0xffff) & 0x0f,
					ds.read_word(0x799a), ds.read_byte((ds.read_word(0x799a) + 0x14) & 0xffff),
					ds.read_word(0x7a36), ds.read_word(0x7a38), machine().time().as_double()); });
		// TEMP cont.256u (STRIP, Dave: "measure what differs between position 1 and 2"):
		// $92B4 is the stamper's decision point - `tst ($742C); bne $92F6` = STAKE, else
		// $92BE = TRANSFER RE-ARM. Position 1 took the transfer leg and completed
		// byte-perfect; position 2 takes the stake leg and orphans. Log the whole cycle
		// vector at every visit so the two can be diffed directly.
		m_cpu->space(AS_OPCODES).install_read_tap(0x92b4, 0x92b5, "stamp92b4",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; if (n++ >= 40) return;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				logerror("STAMP aim=%04x 742c=%04x 7968=%04x 79b8=%04x 79ba=%04x 79b6=%04x "
					"741c=%04x 7426=%04x 7424=%04x 742a=%04x 741e=%04x 7430=%04x 7956=%04x "
					"74ac=%04x 796c=%04x -> %s @%.6f\n",
					ds.read_word(0x7428), ds.read_word(0x742c), ds.read_word(0x7968),
					ds.read_word(0x79b8), ds.read_word(0x79ba), ds.read_word(0x79b6),
					ds.read_word(0x741c), ds.read_word(0x7426), ds.read_word(0x7424),
					ds.read_word(0x742a), ds.read_word(0x741e), ds.read_word(0x7430),
					ds.read_word(0x7956), ds.read_word(0x74ac), ds.read_word(0x796c),
					ds.read_word(0x742c) ? "STAKE" : "TRANSFER", machine().time().as_double()); });
		// TEMP cont.256j (STRIP): THE LADDER TRACE - the walker's jump-table dispatch
		// ($15B4: move.w (0,A0,D0),D0 with A0=$0192, D0 = the step opcode) names every op
		// the walker runs, with its time. read1 spends 1.48s between dispatch (6.489) and
		// completion (7.965) with NO capture arm and never writes phase 0x0A (= the walker
		// never processes a 0x36 data-phase step), so a step handler is consuming the whole
		// command. This says which one.
		m_cpu->space(AS_OPCODES).install_read_tap(0x15b4, 0x15b5, "ladder",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; static u16 lastop = 0xffff; static u32 rep = 0;
				double const t = machine().time().as_double();
				if (t < 1.0 || t > 10.0 || n >= 400) return;   // cont.257: cover the SHORT boot arm too
				u16 const op = u16(m_cpu->state_int(M68K_D0)) & 0xff;
				// cont.256k: collapse the op-28 spin (17k+ re-entries) so the ops that
				// follow the head-load delay actually fit inside the cap.
				if (op == lastop) { rep++; return; }
				address_space &ds = m_cpu->space(AS_PROGRAM);
				n++;
				logerror("LADDER op=%02x (prev x%u) node=%04x phase=%04x err+18=%04x 741c=%04x @%.6f\n",
					op, rep, ds.read_word(0x71bc),
					ds.read_word(0x7216), ds.read_word(0x7208), ds.read_word(0x741c), t);
				lastop = op; rep = 0; });
		// TEMP cont.255h (STRIP): THE CONTRACT-LEG CENSUS - execution truth for the
		// count-4 read's F0->C0 conversion chain. Per cont.242's contract: FE-leg
		// ($7E1E) -> [$742C] clear ($7E42, or $7E90 when [$7968] set) -> next capture's
		// transfer-complete -> $933C convert. Run264 showed stakes ($9318) but zero
		// converts; sampled [$742C] stuck at 1 (possibly aliasing). These name which
		// leg starves, with aim/[$742C]/[$7968]/ledger[1..4] at each hit.
		{
			static constexpr struct { u16 pc; char const *name; } legs[] = {
				{ 0x7e1e, "FELEG" }, { 0x7e42, "CLR742C" }, { 0x7e90, "CLR742C-7968" },
				{ 0x7ea4, "SET742C-a" }, { 0x7eb2, "SET742C-b" }, { 0x933c, "CONVERT" },
				{ 0x7e0a, "FFLEG" }, { 0x7d4a, "NOHANDLER" },
				// cont.255m: the pop chain's verdict points (all subject to prefetch
				// ghosting - corroborate with NODESTATE/[$7968] value changes)
				{ 0x80c0, "POPSCAN" }, { 0x79d6, "POPHIT" }, { 0x79ca, "POPEMPTY" },
				{ 0x810e, "CLOSER" }, { 0x8096, "Q-GATES" },
				// cont.255p: the four links of the node-arm chain - where does the
				// count-4's chain stop? {window-build enable / stage->ARM promote /
				// the ARM popper} + the launcher flavors
				{ 0x70c8, "POPEN-796C" }, { 0x83e2, "QPROMOTE-B8" }, { 0x8480, "ARMPOP" },
				{ 0x85f4, "LAUNCH-85F4" }, { 0x84b6, "PREMARK40" },
				// cont.255q/r: the partial-window ($7A30-mode) completion machine
				{ 0x8348, "TICK-BSR" }, { 0x3dc0, "PWDRV" }, { 0x3e50, "PWNODE" },
				{ 0x3e30, "PWDONE-C" }, { 0x3f54, "PWDEFER" },
				// cont.255v: the parser-tail exit census - which edge of the
				// cont.250d route is wrong for the count-4's services
				{ 0x8018, "T-8018" }, { 0x808a, "T-808A" }, { 0x8096, "T-8096" },
				{ 0x8214, "T-8214" }, { 0x82e2, "T-82E2" }, { 0x8330, "T-8330" },
				{ 0x8340, "T-8340" }, { 0x3dbc, "T-3DBC" },
				// cont.255af: THE IGNITION COMPARE - op-4A's $709E `sub.w ($7956),D3`
				// decides {matched -> $70A4: [$7A64]=1 + pop-enable [$796C]=1, one-shot
				// SURVIVES} vs {mismatch -> $70D2: [$7A30] mode word kills the one-shot
				// AND clears [$7968]}. D3 vs [$7956] are the two numbers that decide the boot.
				{ 0x709e, "IGNCMP" }, { 0x70a4, "IGN-MATCH" }, { 0x70d2, "IGN-KILL" },
				{ 0x82b2, "SET7968" }, { 0x825c, "L825C" }, { 0x8232, "L8232-BA" },
				// cont.256b: the $7C6C ID-ADDRESS-MARK record handler - the ONLY path
				// that RESETS [$742C]/[$741C] ($7CA8). Its fork: [$7968]==0 -> $7C8A
				// posts error $2012 to [$71BE]; !=0 -> $7C92 clears [$7968]/[$7B10].
				// Does it run at all? Which leg? This is the [$742C] reset question.
				{ 0x7c70, "IDAM-TEST" }, { 0x7c84, "IDAM-FORK" }, { 0x7c8a, "IDAM-ERR2012" },
				{ 0x7c92, "IDAM-CLR7968" }, { 0x7ca8, "IDAM-CLR742C" }, { 0x7d4a, "IDAM-DEFAULT" } };
			for (auto const &lg : legs)
				m_cpu->space(AS_OPCODES).install_read_tap(lg.pc, lg.pc | 1, lg.name,
					[this, name = lg.name](offs_t, u16 &, u16)
					{ static int n = 0; double const t = machine().time().as_double();
						if (t < 1.0 || t > 10.0 || n >= 700) return; n++;   // cont.257: short arm too
						address_space &ds = m_cpu->space(AS_PROGRAM);
						logerror("LEGCEN %s aim=%04x 742c=%04x 7968=%04x L1-4=%02x %02x %02x %02x | D3=%04x 7956=%04x 79ba=%04x 79b6=%04x 74ac=%04x @%.6f\n",
							name, ds.read_word(0x7428), ds.read_word(0x742c), ds.read_word(0x7968),
							ds.read_byte(0x7655), ds.read_byte(0x7656), ds.read_byte(0x7657), ds.read_byte(0x7658),
							u16(m_cpu->state_int(M68K_D3)), ds.read_word(0x7956), ds.read_word(0x79ba),
							ds.read_word(0x79b6), ds.read_word(0x74ac),
							t); });
		}
		// TEMP cont.255 (STRIP): DISPATCH CENSUS - which commands reach which install.
		// Execution detectors on the $0D54 dispatch spine: table lookup ($0DD6), flags
		// store ($0E7E), preamble mode selects ($0EB2/$0EC6/$0ED0/$0EE4), the BC install
		// ($0EEC), the bit5-clear alternate ($113A) and the B6 install ($1144), plus the
		// teardown ($1B6A). Logs node cmd byte, D5/D6, A6 (UIB) at each hit. Uncapped
		// (dispatches are rare); prefetch noise possible - correlate pc with neighbors.
		{
			static constexpr struct { u16 pc; char const *name; } census[] = {
				{ 0x0dd6, "TBLLOOK" }, { 0x0e7e, "FLGSTORE" }, { 0x0eb2, "BIT9SEL" },
				{ 0x0ec6, "BIT4SEL" }, { 0x0ed0, "W791A" }, { 0x0ee4, "BIT5SEL" },
				{ 0x0eec, "BCINSTALL" }, { 0x113a, "ALTPATH" }, { 0x1144, "B6INSTALL" },
				{ 0x1b6a, "TEARDOWN" } };
			for (auto const &c : census)
				m_cpu->space(AS_OPCODES).install_read_tap(c.pc, c.pc | 1, c.name,
					[this, name = c.name](offs_t, u16 &, u16)
					{ static int n = 0; if (n >= 200) return; n++;
						address_space &ds = m_cpu->space(AS_PROGRAM);
						u16 const node = ds.read_word(0x7a06);
						logerror("DCENSUS %s pc=%06x d5=%04x d6=%04x a6=%06x a0=%06x nodecmd=%04x 71bc=%04x @%.6f\n",
							name, m_cpu->pc(),
							u16(m_cpu->state_int(M68K_D5)), u16(m_cpu->state_int(M68K_D6)),
							u32(m_cpu->state_int(M68K_A6)), u32(m_cpu->state_int(M68K_A0)),
							node ? ds.read_word(node) : 0xdead,
							ds.read_word(0x71bc), machine().time().as_double()); });
		}
		// cont.86 (STRIP): node+26 provenance at the $2290 dispatcher entry - which node the
		// dispatcher trusts and what its status cell holds, LIVE, windowed on both accepts.
		m_cpu->space(AS_OPCODES).install_read_tap(0x2290, 0x2291, "dispatch26",
			[this](offs_t, u16 &, u16)
			{ static int n1 = 0, n2 = 0; double const t = machine().time().as_double();
				bool const w1 = t > 6.400 && t < 6.415, w2 = t > 8.175 && t < 8.190;
				if (!((w1 && n1 < 12) || (w2 && n2 < 12))) return;
				(w1 ? n1 : n2)++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				u16 const bc = ds.read_word(0x71bc), b6 = ds.read_word(0x71b6);
				logerror("DISP26 71bc=%04x 71b6=%04x +26of[bc]=%04x +26of[b6]=%04x 7216=%04x 71ec=%04x @%.5f\n",
					bc, b6, bc ? ds.read_word((bc + 0x26) & 0xffff) : 0xdead,
					b6 ? ds.read_word((b6 + 0x26) & 0xffff) : 0xdead,
					ds.read_word(0x7216), ds.read_word(0x71ec), t); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x24a, 0x24b, "parktrap",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; if (n >= 4) return; n++;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				u32 const sp = m_cpu->state_int(M68K_SP);
				logerror("PARKTRAP sp=%06x frame: %04x %04x %04x %04x %04x %04x %04x %04x @%.5f\n",
					sp, ds.read_word(sp), ds.read_word(sp + 2), ds.read_word(sp + 4), ds.read_word(sp + 6),
					ds.read_word(sp + 8), ds.read_word(sp + 10), ds.read_word(sp + 12), ds.read_word(sp + 14),
					machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x15ba, 0x15bb, "opdispatch",
			[this](offs_t, u16 &, u16)
			{ static int n = 0, rep = 0; static unsigned last = ~0u; double const t = machine().time().as_double();
				unsigned const h = unsigned(m_cpu->state_int(M68K_D0)) & 0xffff;
				if (t < 6.0) return;
				if (h == last) { rep++; return; }
				if (n < 60)
				{ n++;
					address_space &ds = m_cpu->space(AS_PROGRAM);
					logerror("OPDISP h=%04x (prev x%d) | 7a36=%04x 7a3e=%04x 741c=%04x @%.5f\n",
						h, rep, ds.read_word(0x7a36), ds.read_word(0x7a3e), ds.read_word(0x741c), t); }
				last = h; rep = 0; });
		// cont.68 (STRIP): read2's true status - the node result cells, full write history
		// (node1 result $71de, node2 result $7208/$720a-family), against the raw $7fe8 values.
		for (auto ent : { std::pair<u16, char const *>{0x71de, "WRES1"}, {0x7208, "WRES2"} })
			m_cpu->space(AS_PROGRAM).install_write_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &data, u16)
				{ static std::map<std::string, std::pair<u16, bool>> st; auto &e = st[name]; static std::map<std::string, int> ns;
					if ((!e.second || e.first != data) && ns[name]++ < 30)
						logerror("%s<-%04x pc=%06x @%.5f\n", name, data, m_cpu->pc(), machine().time().as_double());
					e.first = data; e.second = true; });
		// cont.63 (STRIP): [$7a0c]'s three numbers - seed (quota-sized 32 vs read-sized 16?),
		// decrement count vs seed (short = IAM miscount / exact = clean), timestamps vs the
		// expiry (late = race). Window from zero, every change, pc + time.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7a0c, 0x7a0d, "w7a0c",
			[this](offs_t, u16 &data, u16)
			{ static std::pair<u16, bool> e; static int n = 0;
				if ((!e.second || e.first != data) && n++ < 60)
					logerror("W7A0C<-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), machine().time().as_double());
				e.first = data; e.second = true; });
		// cont.58 (STRIP): Q2's tape - [$79a8] (the fw's owed count) full write history,
		// windowed from zero, change-deduped: does it exhaust within one command (DONE should
		// key on it) or span two (seven-plus-continuation)?
		m_cpu->space(AS_PROGRAM).install_write_tap(0x79a8, 0x79a9, "w79a8",
			[this](offs_t, u16 &data, u16)
			{ static std::pair<u16, bool> e; static int n = 0;
				if ((!e.second || e.first != data) && n++ < 40)
					logerror("W79A8<-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), machine().time().as_double());
				e.first = data; e.second = true; });
		// cont.52 (STRIP): THE READER-TAP - the inversion. 180 runs of writer tape; the missing
		// agent is a READER of the staged ingredients. Tap data-reads of $7444 (the descriptor),
		// $79d8 (the inverted host pair), and $743c (the $7458-record's wired watch-cell - the
		// suggestive collision: the read never registers on it). Window from ZERO (the standing
		// default now), dedupe by reader pc, count per site. C++-dump noise filterable by the
		// dump timestamps (9.79/12.0).
		for (auto ent : { std::pair<u16, char const *>{0x7444, "RD-7444"}, {0x79d8, "RD-79D8"}, {0x743c, "RD-743C"} })
			m_cpu->space(AS_PROGRAM).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &data, u16)
				{ static std::map<std::string, std::map<u32, int>> sites; auto &m = sites[name];
					u32 const pc = m_cpu->pc(); auto &c = m[pc];
					if (c++ < 2)
						logerror("%s val=%04x pc=%06x n=%d @%.5f\n", name, data, pc, c, machine().time().as_double()); });
		// cont.51 (STRIP): the pipelining discriminator, full-chain. (1) W793A: every [$793a]
		// write with pc - does #1's op-$18 deposit at $3090 (the seed EXISTS to inherit)?
		// (2) OP1A-ENTRY: at each $3182 entry, [$793a] + the count word AT it - does #2's
		// op-$1a find the seed? Both halves on one tape: "seed written by #1, read by #2".
		m_cpu->space(AS_PROGRAM).install_write_tap(0x793a, 0x793b, "w793a",
			[this](offs_t, u16 &data, u16)
			{ static std::pair<u16, bool> e; static int n = 0;
				if ((!e.second || e.first != data) && n++ < 30)
					logerror("W793A<-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), machine().time().as_double());
				e.first = data; e.second = true; });
		m_cpu->space(AS_OPCODES).install_read_tap(0x3182, 0x3183, "op1a-entry",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (t > 6.0 && n++ < 12)
				{ address_space &ds = m_cpu->space(AS_PROGRAM);
					u16 const sp = ds.read_word(0x793a);
					logerror("OP1A-ENTRY 793a=%04x count@[793a]=%04x 793e=%04x 7938=%04x @%.5f\n",
						sp, sp ? ds.read_word(sp & 0xffff) : 0xdead, ds.read_word(0x793e), ds.read_word(0x7938), t); } });
		// cont.49b (STRIP): the SUBMIT pair - [$743a]/[$7a14] write history (the worker's
		// current-descriptor registration, proven live for the CCB/UIB fetches at $1464/$1418)
		// + the submit primitives $13d2 (channel-op submitter, writes the C800 file) and $27be
		// (the ring submitter). The read's $7444 descriptor must pass through here to run.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x743a, 0x743b, "w743a",
			[this](offs_t, u16 &data, u16)
			{ static std::pair<u16, bool> e; static int n = 0;
				if ((!e.second || e.first != data) && n++ < 30)
					logerror("W743A<-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), machine().time().as_double());
				e.first = data; e.second = true; });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7a14, 0x7a15, "w7a14",
			[this](offs_t, u16 &data, u16)
			{ static std::pair<u16, bool> e; static int n = 0;
				if ((!e.second || e.first != data) && n++ < 30)
					logerror("W7A14<-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), machine().time().as_double());
				e.first = data; e.second = true; });
		for (auto ent : { std::pair<u16, char const *>{0x13d2, "SUB-13D2"}, {0x27be, "SUB-27BE"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> ns; double const t = machine().time().as_double();
					if (t > 6.0 && ns[name]++ < 25)
						logerror("%s @%.5f\n", name, t); });
		// cont.49a (STRIP): DAVE'S DISCRIMINATOR - does the floppy read stock $77c2 (world (i):
		// a floppy stocker exists, road complete once found) or only $74b4 (world (ii): the
		// doors are class-mis-routed)? Write-history on the floppy queue head/tail + the $5b02
		// append region + the $58xx dispatcher entry points.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x77c2, 0x77c5, "w77c2",
			[this](offs_t a, u16 &data, u16)
			{ static std::map<u32, std::pair<u16, bool>> st; static int n = 0; auto &e = st[a & ~1];
				if ((!e.second || e.first != data) && n++ < 40)
					logerror("W77C2 %04x<-%04x pc=%06x @%.5f\n", a & ~1, data, m_cpu->pc(), machine().time().as_double());
				e.first = data; e.second = true; });
		for (auto ent : { std::pair<u16, char const *>{0x5b02, "STK-5B02"}, {0x58a2, "DSP-58A2-ENTRY"},
				{0x58c8, "DSP-58C8-WORK"}, {0x5970, "DSP-5970-FPOP"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> ns; double const t = machine().time().as_double();
					if (t > 6.0 && ns[name]++ < 25)
					{ address_space &ds = m_cpu->space(AS_PROGRAM);
						logerror("%s 77c2=%04x 77c4=%04x 74b4=%04x 791a=%04x 790e=%08x @%.5f\n", name,
							ds.read_word(0x77c2), ds.read_word(0x77c4), ds.read_word(0x74b4),
							ds.read_word(0x791a), ds.read_dword(0x790e), t); } });
		// cont.48 (STRIP): THE TIMER-POKE PUMP on tape. The IRQ1 tick ($2b58) walks the $736c
		// chain; expiry = "write VALUE to CELL" ($2b84: (A1)<-(2,A0); the $7b12 special:
		// $2b6c). Every deferred poke across a command = the complete fifth-road traffic;
		// plus every $29f8 registration (pc names the registrar, frame words = the record).
		for (auto ent : { std::pair<u16, char const *>{0x2b84, "POKE-CHAIN"}, {0x2b6c, "POKE-7B12"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> ns; double const t = machine().time().as_double();
					if (t > 6.0 && ns[name]++ < 40)
					{ address_space &ds = m_cpu->space(AS_PROGRAM);
						u16 const rec = (name[5] == 'C') ? ds.read_word(0x7374) : 0x7b12;   // POKE-CHAIN latches A0 later; use $736c head
						u16 const head = ds.read_word(0x736c);
						logerror("%s head=%04x rec[%04x]={cnt=%04x val=%04x cell=%04x nxt=%04x} @%.5f\n", name, head, rec ? rec : head,
							ds.read_word(head), ds.read_word((head + 2) & 0xffff), ds.read_word((head + 4) & 0xffff),
							ds.read_word((head + 8) & 0xffff), t); } });
		m_cpu->space(AS_OPCODES).install_read_tap(0x2a06, 0x2a07, "reg29f8",
			[this](offs_t, u16 &, u16)
			{ static int n = 0; double const t = machine().time().as_double();
				if (t > 6.0 && n++ < 40)
				{ address_space &ds = m_cpu->space(AS_PROGRAM);
					u32 const a3 = m_cpu->state_int(M68K_A3);
					logerror("REG29F8 key=%04x delay=%04x val=%04x cell=%04x pc-caller-frame a3=%08x @%.5f\n",
						ds.read_word((a3 - 8) & 0xffff), ds.read_word((a3 - 6) & 0xffff),
						ds.read_word((a3 - 4) & 0xffff), ds.read_word((a3 - 2) & 0xffff), a3, t); } });
		// cont.47a (STRIP): [$7a30] = TWO byte flags (high = the door-tail's $3dbc call arm,
		// tested by $833c's BYTE bclr; low = the $70d4/$7338-family word-write flag). Arm
		// lifecycle decides how $3dbc first enters in the delivery era: $6042 (read dispatch)
		// arms high; word-writes clobber; $3de2/$400c re-arm. Full write history with pc.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7a30, 0x7a31, "w7a30",
			[this](offs_t, u16 &data, u16 mem_mask)
			{ static std::tuple<int, u16, bool> e; 
				if ((!std::get<2>(e) || std::get<1>(e) != data) && std::get<0>(e) < 50)
				{ std::get<0>(e)++;
					logerror("W7A30=%04x mask=%04x pc=%06x @%.5f\n", data, mem_mask, m_cpu->pc(), machine().time().as_double()); }
				std::get<1>(e) = data; std::get<2>(e) = true; });
		// cont.41b (STRIP): fork-(b) probes. N2STAT = the SECOND node's status word ($71f0+$26 =
		// $7216) - the CCB/UIB reference commands' lifecycle (does any real completion touch the
		// scanner, and how does the reference mark done). OPCUR = the dual-node op-list cursor
		// cells ($721a cursor-select, $721c/$7224 positions, $7220/$7228 result cells) - the
		// $2258-$2288 juggle live. W79D8 = the $7442 descriptor's embedded host-address source.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7216, 0x7217, "n2stat",
			[this](offs_t, u16 &data, u16)
			{ static int n = 0; if (n++ < 40)
				logerror("N2STAT %04x pc=%06x @%.5f\n", data, m_cpu->pc(), machine().time().as_double()); });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x721a, 0x7229, "opcur",
			[this](offs_t a, u16 &data, u16)
			{ static std::map<u32, std::pair<u16, bool>> st; static int n = 0; auto &e = st[a & ~1];
				if ((!e.second || e.first != data) && n++ < 60)
					logerror("OPCUR %04x<-%04x pc=%06x @%.5f\n", a & ~1, data, m_cpu->pc(), machine().time().as_double());
				e.first = data; e.second = true; });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x79d8, 0x79db, "w79d8",
			[this](offs_t a, u16 &data, u16)
			{ static int n = 0; if (n++ < 30)
				logerror("W79D8 a=%04x %04x pc=%06x @%.5f\n", a & ~1, data, m_cpu->pc(), machine().time().as_double()); });
		// cont.40f (STRIP): E804 write history - the drive-select/side register (high byte =
		// ~drive mask, low byte = control incl. side). If the fw switches sides at the track-1
		// window crossing (~9.2x) and the model's stream stays side 0, the hunt for position 17
		// (= next side's R1) never sees a matching ID - the model-signal gap.
		m_cpu->space(AS_PROGRAM).install_write_tap(0xe804, 0xe805, "e804wr",
			[this](offs_t, u16 &data, u16)
			{ static u16 last = 0xbeef; static int n = 0;
				if (data != last && n++ < 40)
					logerror("E804WR %04x pc=%06x @%.5f\n", data, m_cpu->pc(), machine().time().as_double());
				last = data; });
		// cont.40e (STRIP): the $796a fork (read-ahead/class flag) and $79a0 (walk position-mode)
		// write histories with the $a520 guard's inputs: UIB[$11] (via [$799a]) and node byte 0.
		for (auto ent : { std::pair<u16, char const *>{0x796a, "W796A"}, {0x79a0, "W79A0"} })
			m_cpu->space(AS_PROGRAM).install_write_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &data, u16)
				{ static std::map<std::string, std::pair<u16, bool>> st; auto &e = st[name];
					static std::map<std::string, int> ns;
					if ((!e.second || e.first != data) && ns[name]++ < 30)
					{ address_space &ds = m_cpu->space(AS_PROGRAM);
						u16 const uib = ds.read_word(0x799a), node = ds.read_word(0x71bc);
						logerror("%s=%04x pc=%06x uib=%04x [11]=%02x node=%04x [0]=%02x @%.5f\n", name, data,
							m_cpu->pc(), uib, uib ? ds.read_byte((uib + 0x11) & 0xffff) : 0xdd,
							node, node ? ds.read_byte(node & 0xffff) : 0xdd, machine().time().as_double()); }
					e.first = data; e.second = true; });
		// cont.40c (STRIP): IRQ5/IRQ6 soft-vector history ($7300.l/$7304.l; word installs hit
		// $7302/$7306) - names each phase's continuation; diff our read against the 6.40-era
		// delivering flow (CCB/UIB host transfers = the truck's reference programming).
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7300, 0x7307, "softvec",
			[this](offs_t a, u16 &data, u16 mem_mask)
			{ static std::map<u32, std::pair<int, u16>> st; auto &e = st[a & ~1];
				if (e.second != data && e.first < 50)
				{ e.first++;
					logerror("SOFTVEC %04x<-%04x pc=%06x @%.5f\n", a & ~1, data, m_cpu->pc(), machine().time().as_double()); }
				e.second = data; });
		// cont.39x (STRIP): the post-drain re-arm walk - what did it hunt? Tap the re-arm exit
		// ($7d62 = the $741c re-set) and the walk entry ($7ba8) in the drain window, logging the
		// live capture record ($7dac), targets, and ledger cells around the target.
		for (auto ent : { std::pair<u16, char const *>{0x7ba8, "PDW-ENTRY"}, {0x7d62, "PDW-REARM"}, {0x7d4a, "PDW-7d4a"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> ns; double const t = machine().time().as_double();
					if (t > 9.25 && t < 9.40 && ns[name]++ < 8)
					{ address_space &ds = m_cpu->space(AS_PROGRAM);
						std::string cap, led;
						for (u16 k = 0; k < 8; k++) cap += util::string_format(" %02x", ds.read_byte(0x7dac + k));
						u16 const w54 = ds.read_word(0x7954);
						for (u16 k = 0; k < 20; k++) led += util::string_format(" %02x", ds.read_byte((0x7654 + w54 + k) & 0xffff));
						logerror("%s 7428=%04x 7430=%04x 7436=%04x 7438=%04x 742a=%04x cap:%s led@%04x:%s @%.5f\n", name,
							ds.read_word(0x7428), ds.read_word(0x7430), ds.read_word(0x7436), ds.read_word(0x7438),
							ds.read_word(0x742a), cap.c_str(), 0x7654 + w54, led.c_str(), t); } });
		// cont.39w (STRIP): door-pass census vs the 33us drain window - every $804c door tail
		// after t=9.2 with the resume triple's live values; plus the $8268 fork point and $826e taken.
		for (auto ent : { std::pair<u16, char const *>{0x804c, "DOORTAIL"}, {0x8268, "F8268"}, {0x826e, "F826E-TAKEN"}, {0x82c0, "F82C0"},
				{0x81b4, "T-81B4-DELIVERQ"}, {0x3abc, "T-3ABC-EXECUTOR"}, {0x3e50, "T-3E50-SLOTPOP"}, {0x3f26, "T-3F26-ARM"},
				{0x8338, "SEQ-8338-BCLR"}, {0x3dbc, "SEQ-3DBC-ENTRY"}, {0x3ddc, "SEQ-3DDC-QCHECK"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> ns; double const t = machine().time().as_double();
					if (t > 9.2 && ns[name]++ < 40)
					{ address_space &ds = m_cpu->space(AS_PROGRAM);
						logerror("%s 741c=%04x 7968=%04x 742c=%04x 7426=%04x | 7a30=%04x 74b4=%04x 74b6=%04x @%.5f\n", name,
							ds.read_word(0x741c), ds.read_word(0x7968), ds.read_word(0x742c), ds.read_word(0x7426),
							ds.read_word(0x7a30), ds.read_word(0x74b4), ds.read_word(0x74b6), t); } });
		// cont.39v (STRIP): the resume triple - full write history of $7968 (batch-has-content),
		// $741c (batch-in-progress) and $742c (delivery switch): who flips them, when, and why the
		// post-collection scan never sees {7968=0} -> never sets 742c=1 -> never delivers.
		for (auto ent : { std::pair<u16, char const *>{0x7968, "W7968"}, {0x741c, "W741C"}, {0x742c, "W742C"} })
			m_cpu->space(AS_PROGRAM).install_write_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &data, u16)
				{ static std::map<std::string, std::tuple<int, u16, bool>> st; auto &e = st[name];
					if ((!std::get<2>(e) || std::get<1>(e) != data) && std::get<0>(e) < 40)
					{ std::get<0>(e)++;
						logerror("%s=%04x pc=%06x @%.5f\n", name, data, m_cpu->pc(), machine().time().as_double()); }
					std::get<1>(e) = data; std::get<2>(e) = true; });
		// cont.39u (STRIP): $7abe = the host-buffer-address cell op $56 seeds - who writes it, what value.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7abe, 0x7ac1, "habe",
			[this](offs_t a, u16 &data, u16 mem_mask)
			{ static int n = 0; if (n++ < 20)
				logerror("7ABEWR a=%04x %04x mask=%04x pc=%06x @%.5f\n", a, data, mem_mask, m_cpu->pc(), machine().time().as_double()); });
		// cont.39s (STRIP): does the task scanner EVER match ($1646 = the byte==0a pass point)?
		// And the dispatch point $1676 (jsr handler). Dump the node bytes it saw.
		for (auto ent : { std::pair<u16, char const *>{0x1646, "SCANMATCH"}, {0x1676, "SCANDISPATCH"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> ns; if (ns[name]++ < 20)
					{ address_space &ds = m_cpu->space(AS_PROGRAM);
						u16 const n6 = ds.read_word(0x71b6), nc = ds.read_word(0x71bc);
						logerror("%s 71b6=%04x[+26]=%04x 71bc=%04x[+26]=%04x 727c=%04x @%.5f\n", name,
							n6, n6 ? ds.read_word((n6 + 0x26) & 0xffff) : 0xdead,
							nc, nc ? ds.read_word((nc + 0x26) & 0xffff) : 0xdead,
							ds.read_word(0x727c), machine().time().as_double()); } });
		// cont.39r (STRIP): NODE-STATUS HISTORY - every write to the node status word $71ec
		// (byte $71ec = the task-scanner gate: must be $0a; word = the op scheduler's next-op).
		m_cpu->space(AS_PROGRAM).install_write_tap(0x71ec, 0x71ed, "nstat-hist",
			[this](offs_t a, u16 &data, u16 mem_mask)
			{ static int n = 0; if (n++ < 60)
				logerror("NSTATWR %04x mask=%04x pc=%06x @%.5f\n", data, mem_mask, m_cpu->pc(), machine().time().as_double()); });
		// cont.39q (STRIP): THE BATCH-COMPLETE CROSSING - who runs at the $79a4 expiry
		// (~10.9s) and does anything arm the transfer phase: the content-fork drains
		// ($8a5a id-side, $7db4 data-side), the $79b6 arm ($95e4 inside $94ec = the
		// transfer driver), the $9328 counter block, and the $94ec entry itself.
		for (auto ent : { std::pair<u16, char const *>{0x8a5a, "X-8a5a-IDDRAIN"}, {0x95e4, "X-95e4-ARM79B6"},
				{0x94ec, "X-94ec-ENTRY"}, {0x7992, "R-7992-7958SIDE"}, {0x799a, "R-799a-WAKE9398"},
				{0x9398, "R-9398-ENTRY"}, {0x3dbc, "R-3dbc-ENTRY"}, {0x3e1c, "R-3e1c-UNREG"},
				{0x7a4c, "R-7a4c-TEARDOWN"}, {0x7a9a, "R-7a9a-XFERSIDE"}, {0x797e, "R-797e-7956CHK"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> ns; double const t = machine().time().as_double();
					if (ns[name]++ < 8)
						logerror("%s @%.5f\n", name, t);
					static bool relaydump = false;
					if (t > 12.0 && !relaydump)
					{
						relaydump = true;
						address_space &ds = m_cpu->space(AS_PROGRAM);
						logerror("RELAYSTATE parks 727e=%04x 7286=%04x 728e=%04x 7296=%04x 729e=%04x 72a6=%04x | cells 72d6=%04x 72d8=%04x 72da=%04x 72dc=%04x 72de=%04x 72e0=%04x | 7956=%04x 7958=%08x 796c=%04x 79ba=%04x 79b6=%04x 7968=%04x 79a8=%04x 7424=%04x 74ac=%04x @%.5f\n",
							ds.read_word(0x727e), ds.read_word(0x7286), ds.read_word(0x728e), ds.read_word(0x7296), ds.read_word(0x729e), ds.read_word(0x72a6),
							ds.read_word(0x72d6), ds.read_word(0x72d8), ds.read_word(0x72da), ds.read_word(0x72dc), ds.read_word(0x72de), ds.read_word(0x72e0),
							ds.read_word(0x7956), ds.read_dword(0x7958), ds.read_word(0x796c), ds.read_word(0x79ba), ds.read_word(0x79b6),
							ds.read_word(0x7968), ds.read_word(0x79a8), ds.read_word(0x7424), ds.read_word(0x74ac), t);
					} });
		// cont.39d (STRIP): THE POISONER TAP - first writers of $7428 and $7430, value+
		// pc+t, FROM BOOT (the pre-doorbell window is the discriminator: already-poisoned
		// = init/stale, and every during-the-read story dies). Caught in the act, not
		// reconstructed from residue.
		for (auto ent : { std::pair<u16, char const *>{0x7428, "W7428"}, {0x7430, "W7430"} })
			m_cpu->space(AS_PROGRAM).install_write_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &data, u16)
				{ static std::map<std::string, int> n; double const t = machine().time().as_double();
					if (n[name]++ < 14)
						logerror("%s <-%04x pc=%06x @%.5f\n", name, data, m_cpu->pc(), t); });
		// cont.39b (STRIP): THE OP CENSUS at $22a2 (the loop's first compare - every
		// scheduled status/op passes through in D0). Deduped per (op,era): the setup
		// progression vs the read era's stop-op, in the scheduler's own vocabulary.
		m_cpu->space(AS_OPCODES).install_read_tap(0x22a2, 0x22a3, "sched",
			[this](offs_t, u16 &, u16)
			{ static std::map<int, int> seen; static int ns = 0, nr = 0;
				double const t = machine().time().as_double();
				u16 const op = u16(m_cpu->state_int(M68K_D0));
				bool const setup = (t > 6.400 && t < 6.470), rd = (t > 9.790 && t < 9.860);
				if (!setup && !rd) return;
				int &c = seen[(int(op) << 1) | (rd ? 1 : 0)];
				if (c++ < 3 && (setup ? ns++ : nr++) < 60)
					logerror("SCHED %s op=%04x @%.5f\n", rd ? "READ " : "SETUP", op, t); });
		// cont.38ad (STRIP): the BUILDER GATE ($7432-block) - the $7522 sequence builder
		// is gated on NODE STATUS == $000a (the PARK is the precondition, not the wait!).
		// Log every visit to the check with the full guard vector; name the failing guard.
		m_cpu->space(AS_OPCODES).install_read_tap(0x741a, 0x741b, "bgate",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (_n++ < 20)
				{ address_space &xs = m_cpu->space(AS_PROGRAM);
					u16 const uib = xs.read_word(0x799a), nb = xs.read_word(0x71bc);
					logerror("BGATE 791a=%04x uib11=%02x nstat=%04x uibE=%02x @%.5f\n",
						xs.read_word(0x791a), xs.read_byte((uib + 0x11) & 0xffff),
						xs.read_word((nb + 0x26) & 0xffff), xs.read_byte((uib + 0x0e) & 0xffff), t); } });
		// cont.38ac (STRIP): does the $7522 SEQUENCE BUILDER (the $ff-wanted/$fe-end map
		// writer) EVER run in this regime - and a map+targets snapshot at 9.79.
		m_cpu->space(AS_OPCODES).install_read_tap(0x7522, 0x7523, "seqbuild",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n++ < 8)
				logerror("SEQBUILD-7522 @%.5f\n", machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x1606, 0x1607, "mapsnap",
			[this](offs_t, u16 &, u16)
			{ static bool done = false; double const t = machine().time().as_double();
				if (done || t < 9.79) return; done = true;
				address_space &xs = m_cpu->space(AS_PROGRAM);
				std::string sm;
				for (u16 a = 0x7654; a < 0x766c; a++) sm += util::string_format(" %02x", xs.read_byte(a));
				logerror("MAPSNAP 7654:%s | 7428=%04x 7430=%04x 7954=%04x 7abc=%04x @%.5f\n",
					sm.c_str(), xs.read_word(0x7428), xs.read_word(0x7430),
					xs.read_word(0x7954), xs.read_word(0x7abc), t); });
		// cont.38ab (STRIP): THE ONE DOOR-OPENING dissected - in 6.400-6.420, trace the
		// first data-ISR's exit path (GRD points + $7ba8 entry + the consumer $7f6c/$7f22
		// + $8044's fork) and what $8018 did with its single opening.
		for (auto ent : { std::pair<u16, char const *>{0x7ba8, "T-7ba8"}, {0x7c34, "T-7c34"},
				{0x7ce6, "T-7ce6"}, {0x7d4a, "T-7d4a"}, {0x7f6c, "T-7f6c-CONSUMER"},
				{0x7f22, "T-7f22-CLEAR"}, {0x804c, "T-804c-SLOTS"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> ns, nr; double const t = machine().time().as_double();
					bool const setup = (t > 6.400 && t < 6.425), rd = (t > 9.790 && t < 9.815);
					if ((setup && ns[name]++ < 8) || (rd && nr[name]++ < 8))
						logerror("%s @%.5f\n", name, t); });
		// cont.38zz (STRIP): WHICH RE-ARM CALL SITE - the read era reaches $88ac within
		// 5us of every IRQ5 (phase knocked down); setup's pairs arrive with no re-arm
		// between (second lands phase-1 -> $8018). Tap the three bsr-$88ac sites per era.
		for (auto ent : { std::pair<u16, char const *>{0x7da2, "ARM@7da2"}, {0x8ab8, "ARM@8ab8"},
				{0x7b94, "ARM@7b94"}, {0x8018, "DOOR-8018"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> ns, nr; double const t = machine().time().as_double();
					bool const setup = (t > 6.400 && t < 6.450), rd = (t > 9.790 && t < 9.840);
					if ((setup && ns[name]++ < 10) || (rd && nr[name]++ < 10))
						logerror("%s @%.5f\n", name, t); });
		// cont.38yy (STRIP): THE TRIANGLE DIFFERENTIAL - write-taps on all three cells
		// ($742c retry-flag, $7950 phase toggle, $7a0e match flag), dual-era windows: one
		// certified setup cycle (6.40-6.45) vs one read cycle (9.79-9.84), same run.
		// Read causally: the FIRST divergent write is the root; the other two are echoes.
		for (auto ent : { std::pair<u16, char const *>{0x742c, "TRI-742c"}, {0x7950, "TRI-7950"}, {0x7a0e, "TRI-7a0e"} })
			m_cpu->space(AS_PROGRAM).install_write_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &data, u16)
				{ static std::map<std::string, int> ns, nr; double const t = machine().time().as_double();
					bool const setup = (t > 6.400 && t < 6.450), rd = (t > 9.790 && t < 9.840);
					if ((setup && ns[name]++ < 30) || (rd && nr[name]++ < 30))
						logerror("%s <-%04x pc=%06x @%.5f\n", name, data, m_cpu->pc(), t); });
		// cont.38ee (STRIP): THE PHASE TRACE - every [$7950] write with pc between the id
		// capture and the following data mark. The $298c id-toggle's 1 must stand to the
		// data-mark IRQ5 ($29c0 bit-was-1 -> $8018 = the consume). The clearer's PC names
		// whether it's a firmware round the correct flow avoids (phase-state issue) or a
		// round the model provokes (model over-run).
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7950, 0x7951, "phasewr",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 9.795 && t < 9.845 && _n++ < 30)
					logerror("PHASEWR [7950]<-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), t); });
		// cont.38cc (STRIP): THE [[${'$'}7300]] ROUTING WATCH (Dave) - the level-5 soft vector
		// across the pull->consume boundary: every WRITE to the cell (who installs which
		// target, under what phase) and every TAKE (TRAMP5). Does it ever hold the
		// consumer branch ($7f0e-$7f5c region), and what installs it? The working pull
		// routing is the reference for a correct install.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7300, 0x7303, "vec5wr",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 9.79 && t < 10.1 && _n++ < 30)
					logerror("VEC5WR [%04x]<-%04x&%04x pc=%06x 7950=%04x @%.5f\n", offset, data, mem_mask,
						m_cpu->pc(), m_cpu->space(AS_PROGRAM).read_word(0x7950), t); });
		// cont.38bb (STRIP): THE STOPWATCH - the [$7426] race on camera. Writes to $7426
		// (set per round / clear by the consumer), the $8092 walk-gate test, and the $7f1a
		// consumer entry, one timestamped window. The consumer's clock: entries aligning to
		// the ~25ms round cadence / 26ms PIT tick = poll-shaped lag (the missing prompt
		// event is the fix); µs-offsets from the rounds = event-shaped.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7426, 0x7427, "cnt7426",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 9.79 && t < 10.1 && _n++ < 30)
					logerror("CNT7426 <-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), t); });
		for (auto ent : { std::pair<u16, char const *>{0x8092, "WALKGATE"}, {0x7f1a, "CONSUMER"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> ns; double const t = machine().time().as_double();
					if (t > 9.79 && t < 10.1 && ns[name]++ < 30)
					{ address_space &xs = m_cpu->space(AS_PROGRAM);
						logerror("%s 7426=%04x 79b8=%04x @%.5f\n", name,
							xs.read_word(0x7426), xs.read_word(0x79b8), t); } });
		// cont.38s (STRIP): THE DIFFERENTIAL GUARD READ (Dave) - the ID round parks ($36/
		// $000a) while the data round poisons ($7eb2); both pass the same guard chain. Tap
		// the guard vector at the fork's approach points with the buffer's AM byte (FE=id
		// round, FB=data round): the state that reads differently between them names the
		// discriminator. Plus: does the $17f8 re-stamp site EVER run?
		for (auto ent : { std::pair<u16, char const *>{0x7ce6, "GRD-7ce6"}, {0x7e8a, "GRD-7e8a"}, {0x7d4a, "GRD-7d4a"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> ns; double const t = machine().time().as_double();
					if (t > 9.79 && ns[name]++ < 12)
					{ address_space &xs = m_cpu->space(AS_PROGRAM);
						logerror("%s am=%02x r=%02x | 79b6=%04x 79a0=%04x 79ae=%04x 7428=%04x 742c=%04x 7968=%04x 7986=%04x @%.5f\n",
							name, xs.read_byte(0x7daf), xs.read_byte(0x7db2),
							xs.read_word(0x79b6), xs.read_word(0x79a0), xs.read_word(0x79ae), xs.read_word(0x7428),
							xs.read_word(0x742c), xs.read_word(0x7968), xs.read_word(0x7986), t); } });
		m_cpu->space(AS_OPCODES).install_read_tap(0x815e, 0x815f, "happystamp",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n++ < 8)
				logerror("HAPPYSTAMP-815e (slot status 2) @%.5f\n", machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x17f8, 0x17f9, "restamp",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 9.79 && _n++ < 8)
					logerror("RESTAMP-17f8 RUNS @%.5f\n", t); });
		// cont.38r (STRIP): measure-execution-first (Dave). (a) READ-tap on $71ec - whoever
		// loads the stamped $000a status word IS the machinery that judges the record; it
		// names itself instead of being inferred. (b) $7968's REAL setter - its assumed one
		// ($79d6 inside never-executed $7964) is dead code; where does it actually come from?
		m_cpu->space(AS_PROGRAM).install_read_tap(0x71ec, 0x71ed, "statrd",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 9.79 && _n++ < 24)
					logerror("STATRD [71ec] pc=%06x @%.5f\n", m_cpu->pc(), t); });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7968, 0x7969, "seqact2",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 6.35 && data != 0 && _n++ < 12)
					logerror("SEQACT2 [7968]<-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), t); });
		// cont.38q (STRIP): the DISPATCH-SITE truth tap - does the wait-scan EVER reach its
		// dispatch path ($1646) in any era, and through which node/byte? (The static decode
		// says the byte test can never match the word stamp - one of the decodes is wrong;
		// measure instead of deriving.)
		m_cpu->space(AS_OPCODES).install_read_tap(0x1646, 0x1647, "scandisp",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n++ >= 16) return; address_space &xs = m_cpu->space(AS_PROGRAM);
				logerror("SCANDISP-1646 71b6=%04x 71bc=%04x [71ec/w]=%04x 727c=%04x @%.5f\n",
					xs.read_word(0x71b6), xs.read_word(0x71bc), xs.read_word(0x71ec),
					xs.read_word(0x727c), machine().time().as_double()); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x1676, 0x1677, "scanjsr",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n++ >= 16) return;
				logerror("SCANJSR-1676 handler-call @%.5f\n", machine().time().as_double()); });
		// cont.38p (STRIP): the STATUS-WORD resolver (Dave's constraint) - byte-resolved
		// write-tap on node $71c6's +$24..+$29 ($71ea-$71ef): the mem_mask names WHICH byte
		// each setter writes, resolving the $25-base vs $26-base encoding AND whether the
		// $36 op's $0a stamp fires at all post-spin-up. Plus: does $7964 EVER dispatch?
		m_cpu->space(AS_PROGRAM).install_write_tap(0x71ea, 0x71ef, "nstat",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 9.79 && _n++ < 24)
					logerror("NSTAT [%04x]<-%04x&%04x pc=%06x @%.5f\n", offset, data, mem_mask, m_cpu->pc(), t); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x7964, 0x7965, "drv7964",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 9.79 && _n++ < 8)
					logerror("DRV7964 DISPATCHED @%.5f\n", t); });
		// cont.38o (STRIP): the $7b18 data-ready discriminator - $92b4 sets it ($92ce happy /
		// $92f6 fail), $7f22 clears it (the $7f52 consumer). Read-era outcomes: set-then-
		// cleared = consumed (break past $7f52); set-never-cleared = consumer unscheduled;
		// never-set = break at $92b4 (its post-data round never runs its [$742c]==0 branch).
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7b18, 0x7b19, "dready",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 9.79 && _n++ < 24)
					logerror("DREADY [7b18]<-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), t); });
		// cont.38g (STRIP): THE OP-WALK TRACE - every op the $15ac dispatcher consumes (D0 =
		// the op at $15b0's lea), with the drive-state cells. Names the op where each retry
		// cycle dies. Windows: the op setup era + one retry cycle.
		m_cpu->space(AS_OPCODES).install_read_tap(0x15b0, 0x15b1, "opwalk",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0, _m = 0; double const t = machine().time().as_double();
				if (t <= 6.55 ? _n++ >= 40 : (t <= 7.99 || _m++ >= 40)) return;
				address_space &xs = m_cpu->space(AS_PROGRAM);
				logerror("OPWALK op=%02x | f000=%04x 798e=%04x 7a36=%04x 79e2=%04x 7968=%04x 71bc=%04x st=%02x @%.5f\n",
					u16(m_cpu->state_int(M68K_D0)) & 0xff, xs.read_word(0xf000), xs.read_word(0x798e),
					xs.read_word(0x7a36), xs.read_word(0x79e2), xs.read_word(0x7968),
					xs.read_word(0x71bc), xs.read_byte((xs.read_word(0x71bc) + 0x27) & 0xffff), t); });
		// cont.38f (STRIP): the op-list writer namer - node $71c6's op list at $7254 stays
		// EMPTY (MICROSEQ [A1]=0000), so the $36 wait-op never runs, status never reaches
		// $0a, $7964 never dispatches, the ring starves ($7968=0 -> $742c=1 -> $92f6 fails).
		// Who writes $7254-$725f, and who sets $7968=1?
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7240, 0x725f, "oplist",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 6.35 && _n++ < 40)
					logerror("OPLIST [%04x]<-%04x&%04x pc=%06x @%.5f\n", offset, data, mem_mask, m_cpu->pc(), t); });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7968, 0x7969, "seqact",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 6.35 && data != 0 && _n++ < 12)
					logerror("SEQACT <-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), t); });
		// cont.38e (STRIP): the $742c retry-flag writer namer - [$742c]=1 on the FIRST $92b4
		// entry (6.41338) sends every stream-era sector down the $92f6 fail path before any
		// data capture is attempted. Setters: $7162/$7bf6/$7ea4/$7eb2/$95da; clearers:
		// $7ca8 (good-ID continuation)/$7e42/$7e90.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x742c, 0x742d, "flag742c",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 6.35 && _n++ < 24)
					logerror("FLAG742C <-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), t); });
		// cont.38d (STRIP): the $79a4 countdown-refresh namer - the $89f2 floppy branch
		// decrements it per capture and its EXPIRY is the DATASTEP wake ($8aa4 clears
		// [[$72de]]); it never expires because something refreshes it from $79a2. Name the
		// refresher (candidates: $7a06/$7b3e/$7e0e/$7e3c/$7e60/$84d0/$866e).
		m_cpu->space(AS_PROGRAM).install_write_tap(0x79a4, 0x79a5, "cnt79a4",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 8.5 && _n++ < 24)
					logerror("CNT79A4 <-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), t); });
		// cont.38 (STRIP): the $298c phase-fork dispatch trace - per IRQ6 the toggler routes
		// phase0->$89f2 (ID processor) / phase1->$92b4 (the error-stamp region). Log both
		// entries with the phase + verdict cells in the retry era: the alternation pattern
		// names why every capture lands in the error path.
		for (auto ent : { std::pair<u16, char const *>{0x89f2, "89f2-IDPROC"}, {0x92b4, "92b4-PH1"},
			{0x7ba8, "7ba8-IRQ5a"}, {0x8018, "8018-IRQ5b"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> ns; double const t = machine().time().as_double();
					if (t > 7.995 && ns[name]++ < 30)
					{ address_space &xs = m_cpu->space(AS_PROGRAM);
						logerror("PHFORK %s 7950=%04x 7428=%04x 741c=%04x 79b8=%04x 7dac.r=%02x @%.5f\n", name,
							xs.read_word(0x7950), xs.read_word(0x7428), xs.read_word(0x741c),
							xs.read_word(0x79b8), xs.read_byte(0x7db2), t); } });
		// cont.297 (Dave): aim-writers + the STAKE($92f6, event1) vs CONVERT($933c, event2) of the
		// two-event contract, in the PROVEN PHFORK block with $89f2 as the must-fire control. Log
		// [$7428] + [$742c] to read: does $933c fire at all, and does [$742c] alternate 1->0?
		for (auto ent : { std::pair<u16, char const *>{0x89f2, "CTL-89f2"},
			{0x3968, "AW-3968"}, {0x70e0, "AW-70e0"}, {0x7e0a, "AW-7e0a"}, {0x7e32, "AW-7e32"},
			{0x8318, "AW-8318"}, {0x8628, "AW-8628"}, {0x945e, "AW-945e"}, {0x992e, "AW-992e"},
			{0x71ec, "AW-71ec"}, {0x7bf0, "AW-7bf0"}, {0x92f6, "STAKE-92f6"}, {0x933c, "CONVERT-933c"},
			{0x6f44, "BUILD-6f44"}, {0x6ff0, "CONV-6ff0"}, {0x726c, "CONV-726c"}, {0x9362, "CONV-9362"},
			{0x6fde, "SCAN-6fde"}, {0x7c34, "WALK-7c34"}, {0x7d02, "WALK-7d02"}, {0x7e58, "GATE-7e58"},
			{0x7d4a, "RERUN-7d4a"}, {0x7e6c, "SUBQ-7e6c"}, {0x7106, "RE7106-6f44"},
			{0x6ed2, "OP4A-6ed2"}, {0x159c, "PARK36-159c"}, {0x417a, "DESCGO-417a"},
			{0x70a0, "DRAIN-70a0"}, {0x7ebe, "WSUBQ-7ebe"}, {0x7ed8, "DISARM-7ed8"}, {0x1646, "PUMPSEL-1646"},
			{0x822c, "EVAL-822c"}, {0x82b2, "ENABLE-82b2"}, {0x931c, "SET79ba-931c"}, {0xa476, "SET79ba-a476"},
			{0x92b4, "FORK-92b4"}, {0x92be, "CONVENTRY-92be"}, {0x9354, "C0WRITE-9354"}, {0x7e1e, "FELEG-7e1e"},
			{0x7ea4, "SET-7ea4"}, {0x7eb2, "SET-7eb2"}, {0x7e42, "CLR-7e42"}, {0x7e90, "CLR-7e90"}, {0x7e58, "SCAN-7e58"},
			{0x3bfe, "IRQ4-3bfe"}, {0x3fd0, "CLR7454-3fd0"}, {0x3dbc, "UNPARK-3dbc"}, {0x413c, "SET7454-413c"}, {0x4102, "CHLAUNCH-4102"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> ac; double const t = machine().time().as_double();
					if (t > 7.9 && ac[name]++ < 20)
					{ address_space &xs = m_cpu->space(AS_PROGRAM);
						logerror("AIMCV %-14s 7454=%04x 7a64=%04x 72d6=%04x 741c=%04x 7956=%04x aim=%04x @%.5f\n", name,
							xs.read_word(0x7454), xs.read_word(0x7a64), xs.read_word(0x72d6), xs.read_word(0x741c),
							xs.read_word(0x7956), xs.read_word(0x7428), t);
						logerror("  ^%s D3=%04x 741c=%04x\n", name, u16(m_cpu->state_int(M68K_D3)), xs.read_word(0x741c)); } });
		// cont.305 (Dave's Part A): read inventory + arming + completion. Control = $89f2 (must fire).
		// Per PC log m_iopb_cmd (the command) so we can correlate: which cmds reach $9400 (armed) and
		// which complete ($1a54=0x80) vs stall ($184e=0x82). No time gate - catch all commands.
		for (auto ent : { std::pair<u16, char const *>{0x89f2, "PA-CTL"},
			{0x24aa, "PA-DOORBELL"}, {0x1a54, "PA-DONE80"}, {0x184e, "PA-ERR82"},
			{0x9400, "PA-ARM9400"}, {0x9412, "PA-INST5fc0"}, {0x940a, "PA-INST6102"},
			{0x739a, "PA-ARM79ae"}, {0x6ed6, "PA-ARM7b10"}, {0x1144, "PA-INST71b6"} })
			m_cpu->space(AS_OPCODES).install_read_tap(ent.first, ent.first | 1, ent.second,
				[this, name = ent.second](offs_t, u16 &, u16)
				{ static std::map<std::string, int> pc; if (pc[name]++ >= 24) return;
					address_space &xs = m_cpu->space(AS_PROGRAM);
					logerror("PARTA %-12s cmd=%02x 71bc=%04x 799a=%04x uib20=%04x @%.5f\n", name,
						m_iopb_cmd, xs.read_word(0x71bc), xs.read_word(0x799a),
						xs.read_word((xs.read_word(0x799a) + 0x20) & 0xffff), machine().time().as_double()); });
		// cont.304 (Dave's A/B): suppress the WALK's [$7956] subq ($7ebe) so $70a0/$6f44 can own the
		// counter - isolates whether "the walk steals the counter" is the true half of the coupling.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7956, 0x7957, "nowalksubq",
			[this](offs_t, u16 &data, u16)
			{ if (storager_getenv("STORAGER_NOWALKSUBQ") && m_cpu->pc() == 0x7ebe) data = u16(data + 1); });
		// cont.38 (STRIP): the $7654 SECTOR-MAP write-tap - the map codes ($c0/$f0/$fe/$ff/$aa)
		// are the floppy engine's per-sector state language; each transition names its writer.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7654, 0x7665, "secmap",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ static int _n = 0, _n1280 = 0; double const t = machine().time().as_double();
				u32 const pc = m_cpu->pc();
				// cont.255i: $1280 (the completion-side wholesale C0-fill) INCLUDED but
				// separately capped - read1's closes were invisible under the old filter.
				bool const is1280 = (pc == 0x1280);
				if (t > 6.3 && ((is1280 && _n1280++ < 60) || (!is1280 && _n++ < 400)))
					logerror("SECMAP%s [%04x]<-%04x&%04x pc=%06x 7428=%04x 7430=%04x @%.5f\n", is1280 ? "-C0FILL" : "",
						offset, data, mem_mask,
						pc, m_cpu->space(AS_PROGRAM).read_word(0x7428), m_cpu->space(AS_PROGRAM).read_word(0x7430), t); });
		// cont.255ad (THE CAPTURE-STATUS WRITE - the GA's owed state, cont.255m/n/ad):
		// hardware marks the captured sector's chunk node collectible at capture-
		// complete; op-4A's fresh window build zeroes all node statuses and nothing
		// else ever writes them (run269/270: status 0x0000 forever, POPSCANs starve).
		// The fw's own F0 stake ($9314, pc~$9318) IS the capture-complete instant and
		// names the captured POSITION (idx==pos, cont.255n). Mark $74C4[pos]*8+2=0x40
		// (the pop-collectible/ARMED state the $80C0 scan and $8480 popper consume).
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7654, 0x7665, "capstat",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				u32 const pc = m_cpu->pc();
				if (pc < 0x9310 || pc > 0x9322) return;   // the $9314 stake only
				u8 const b = (mem_mask & 0x00ff) ? u8(data) : u8(data >> 8);
				if (b != 0xf0) return;
				unsigned const pos = unsigned(offset - 0x7654) + ((mem_mask & 0x00ff) ? 1 : 0);
				if (pos < 1 || pos > 16) return;
				address_space &ns = m_cpu->space(AS_PROGRAM);
				u16 const node = u16(0x74c4 + pos * 8);
				ns.write_word((node + 2) & 0xffff, 0x0040);
				logerror("CAPSTAT node[%u]+2 <- 0040 (stake @pc=%06x) @%.6f\n", pos, pc, machine().time().as_double());
			});
		// TEMP cont.255j (STRIP): the TOGGLER walk - every write to [$7950] (the $29xx
		// ISRs' bchg) with pc + new value = the complete parity sequence, service-side,
		// prefetch-immune. pc names the handler ($299A id / $29C0 data-fork / $2970-8C
		// family); the OLD parity (new^1) routed the fork. One tap replaces logging all
		// 13 model raise sites - a raise that is never serviced flips nothing.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7950, 0x7951, "w7950",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 7.98 && t < 8.30 && _n++ < 300)
					logerror("TOGWALK <-%04x pc=%06x aim=%04x 742c=%04x @%.6f\n", data, m_cpu->pc(),
						m_cpu->space(AS_PROGRAM).read_word(0x7428),
						m_cpu->space(AS_PROGRAM).read_word(0x742c), t); });
		// TEMP cont.255ac (STRIP): [$7ABC] (the window ask-count pair head) - every
		// write with pc: when does the count-4's 4 replace read1's stale 8, vs op-4A's
		// $6F5C copy into [$7956] (the mode-select comparand).
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7abc, 0x7abd, "w7abc",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 6.3 && _n++ < 120)
					logerror("W7ABC <-%04x pc=%06x @%.6f\n", data, m_cpu->pc(), t); });
		// TEMP cont.255w (STRIP): the [$7A30] one-shot's DEFINITIVE arm/consume ledger
		// (value+pc, prefetch-immune) - who arms it, who consumes it, when, vs the
		// per-sector $8340 passes and op-4A's tail clears.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7a30, 0x7a31, "w7a30",
			[this](offs_t, u16 &data, u16 mem_mask)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 6.3 && _n++ < 300)
					logerror("W7A30 <-%04x mm=%04x pc=%06x @%.6f\n", data, mem_mask, m_cpu->pc(), t); });
		// TEMP cont.255i (STRIP): the [$742C] write-tap - the capture-in-flight cell's
		// TRUE set/clear cycling with pc (prefetch-immune; the LEGCEN opcode taps ghost).
		m_cpu->space(AS_PROGRAM).install_write_tap(0x742c, 0x742d, "w742c",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 6.3 && _n++ < 400)
					logerror("W742C <-%04x pc=%06x aim=%04x 7968=%04x @%.6f\n", data, m_cpu->pc(),
						m_cpu->space(AS_PROGRAM).read_word(0x7428), m_cpu->space(AS_PROGRAM).read_word(0x7968), t); });
		// TEMP (STRIP): $34c8 work-path tap (build#5 cont.27) - does the node processor's
		// first-entry work path run, and with what node?
		m_cpu->space(AS_OPCODES).install_read_tap(0x34c8, 0x34c9, "nodework",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n++ >= 12) return; address_space &xs = m_cpu->space(AS_PROGRAM);
				logerror("NODEWORK-34c8 7b0e=%04x 74ac=%04x 74ae=%04x @%.5f\n",
					xs.read_word(0x7b0e), xs.read_word(0x74ac), xs.read_word(0x74ae), machine().time().as_double()); });
		// TEMP (STRIP): the 0x14 stale-cmd namer (build#5 cont.9). $d54 = `subi.b #$70,D1` - D1 holds
		// the cmd byte the dispatcher fetched. Log ONLY when the range check will FAIL (cmd outside
		// [$70,$af) - a genuine 0x95 passes), i.e. the prefetch gate: what value did the dispatcher
		// read, from where (A0), while [$71f0] still holds what the router saw?
		m_cpu->space(AS_OPCODES).install_read_tap(0x0d54, 0x0d55, "stalecmd",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n >= 30) return;
				u8 const c = u8(m_cpu->state_int(M68K_D1));
				if (c >= 0x70 && c < 0xaf) return;   // would PASS - prefetch or a healthy dispatch
				_n++;
				address_space &xs = m_cpu->space(AS_PROGRAM);
				u16 const a0 = u16(m_cpu->state_int(M68K_A0));
				logerror("STALECMD D1=%02x A0=%04x [A0]=%04x | 71f0=%04x 7ff8=%02x 7ffa=%02x d000<<1=%04x @%.5f\n",
					c, a0, xs.read_word(a0), xs.read_word(0x71f0), xs.read_byte(0x7ff8), xs.read_byte(0x7ffa),
					u32(m_d000) << 1, machine().time().as_double()); });
		// TEMP (STRIP): the retry-edge namer. $1d2a = the $1d1a wait's ERROR-branch entry (subq $71b2)
		// - runs once per 2.61s retry. Log the LIVE A0 (the node whose [$18] flipped nonzero = the
		// timeout owner's target) + its fields. PREFETCH-GATED: $1d2a is prefetched every loop pass
		// (fall-through of the taken beq), so only log when the exit condition is actually true.
		m_cpu->space(AS_OPCODES).install_read_tap(0x1d2a, 0x1d2b, "retryedge",
			[this](offs_t, u16 &, u16)
			{ static int _n = 0; if (_n >= 20) return; address_space &xs = m_cpu->space(AS_PROGRAM);
				u16 const a0 = u16(m_cpu->state_int(M68K_A0));
				u16 const err = xs.read_word((a0 + 0x18) & 0xffff);
				if (!err || !xs.read_word(0x749c)) return;   // prefetch during the loop - not the real exit
				_n++;
				logerror("RETRYEDGE A0=%04x [A0]=%04x [A0+18]=%04x [A0+26]=%04x 749c=%04x 71b2=%04x | q748a[+12/14/16/18]=%04x %04x %04x %04x @%.5f\n",
					a0, xs.read_word(a0), err, xs.read_word((a0 + 0x26) & 0xffff),
					xs.read_word(0x749c), xs.read_word(0x71b2),
					xs.read_word(0x749c), xs.read_word(0x749e), xs.read_word(0x74a0), xs.read_word(0x74a2),
					machine().time().as_double()); });
		// TEMP (STRIP): THE inline-vs-park measurement (Dave's fork). $1b90 = the fw PARKS a 24-byte
		// node snapshot into the $7e00 ring (build the ring-stamp resume ONLY if this fires for the
		// sys-floppy check); $36de = the seek-wait releases INLINE and continues synchronously (park
		// machinery not needed for this op).
		m_cpu->space(AS_OPCODES).install_read_tap(0x1b90, 0x1b91, "park_1b90",
			[ringdump](offs_t, u16 &, u16){ ringdump("PARK-1b90"); });
		m_cpu->space(AS_OPCODES).install_read_tap(0x36de, 0x36df, "inline_36de",
			[this, ringdump](offs_t, u16 &, u16)
			{
				ringdump("INLINE-36de");
				// WIDE PCHIST: each inline release = a stage boundary; dump this stage's poll map
				// and start a fresh window for the next stage.
				if (m_pcsamp_on && storager_getenv("STORAGER_PCHIST_WIDE") && m_pcsamp_n > 100)
				{
					logerror("PCHIST STAGE-BOUNDARY (INLINE-36de) @%.5f:\n", machine().time().as_double());
					dump_pchist();
					m_pchist.clear();
					m_pcsamp_n = 0;
				}
			});
		// TASK#2: who writes the CCB command byte $71f0 (the value that dispatches as 89/02/0c)?  pc=0 => emulator DMA.
		m_cpu->space(AS_PROGRAM).install_write_tap(0x71f0, 0x71f1, "ccbcmd",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; if (_n++ >= 220) return; address_space &xs = m_cpu->space(AS_PROGRAM);
				char const *who = m_dma_mark == 1 ? "MODEL-doorbell-iopb[$7a06]" : m_dma_mark == 2 ? "MODEL-kick-iopb[m_d000<<1]" : "FW";
				logerror("71f0<-%04x pc=%06x WRITER=%s | m_d000<<1=%04x [$7a06]=%04x %s cmd=%02x @%.5f\n",
					data, m_cpu->pc(), who, u32(m_d000) << 1, xs.read_word(0x7a06),
					m_dma_mark ? ((u32(m_d000) << 1) == 0x71f0 ? "(dst=$71f0)" : "(dst!=$71f0)") : "", m_iopb_cmd, machine().time().as_double()); });
		// TASK#4: the op STATE field node+26=[$7216] (the $251e bump gate).  Watch the restore's state sequence
		// at runtime (which state-setter pc, what value, when) - where does it park and what's it waiting on?
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7216, 0x7217, "state26",
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; if (_n++ >= 80) return; address_space &xs = m_cpu->space(AS_PROGRAM);
				logerror("STATE26 [7216]<-%04x pc=%06x | 71b2=%04x 743a=%04x e01e=%04x f000=%04x cmd=%02x @%.5f\n",
					data, m_cpu->pc(), xs.read_word(0x71b2), xs.read_word(0x743a),
					m_ch[(0xe01e - 0xe000) / 2], m_ch[(0xf000 - 0xe000) / 2], m_iopb_cmd, machine().time().as_double()); });
	}
	// TEMP (STRIP): node 748a +1c/+26 WRITE taps (Dave) - what advances the op (+1c) vs marks it done (+26).
	if (m_fw_driven)
	{
		m_cpu->space(AS_PROGRAM).install_write_tap(0x74a6, 0x74a7, "n1c",   // 748a+0x1c
			[this](offs_t, u16 &data, u16)
			{ static int _n = 0; double t = machine().time().as_double(); if (t > 6.3 && _n++ < 60)
				logerror("748a+1c<-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), t); });
		m_cpu->space(AS_PROGRAM).install_write_tap(0x74b0, 0x74b1, "n26",   // 748a+0x26 (the done marker)
			[this](offs_t, u16 &data, u16)
			{ double t = machine().time().as_double(); if (t > 6.3) logerror("748a+26<-%04x pc=%06x @%.5f\n", data, m_cpu->pc(), t); });
	}
	// TEMP (STRIP): FULL WALK-TRACE (Dave) - every fetch PC + tested regs across the routing range, one boot,
	// capturing the working 0x87 path (->$2626 bump) and the seek 748a path (->$193a re-scan). STORAGER_WALKTRACE.
	if (storager_getenv("STORAGER_WALKTRACE"))
		m_cpu->space(AS_OPCODES).install_read_tap(0x1900, 0x26ff, "walktrace",
			[this](offs_t, u16 &, u16)
			{
				double t = machine().time().as_double();
				if (t < 6.399 || t > 6.404) return;
				static int _n = 0; if (_n++ >= 1200) return;
				address_space &ds = m_cpu->space(AS_PROGRAM);
				logerror("W pc=%06x D0=%04x D1=%04x D3=%04x A0=%04x A3=%04x | n26=%04x 71b2=%04x 743a=%04x\n",
					m_cpu->pc(), u16(m_cpu->state_int(M68K_D0)), u16(m_cpu->state_int(M68K_D1)), u16(m_cpu->state_int(M68K_D3)),
					u16(m_cpu->state_int(M68K_A0)), u16(m_cpu->state_int(M68K_A3)),
					ds.read_word(0x74b0), ds.read_word(0x71b2), ds.read_word(0x743a));
			});
	// TEMP LOG (STRIP): $7A74 = the controller-status word (per-unit fault flags from the drive scanner)
	if (m_fw_driven)
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7a74, 0x7a75, "fwfault",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				static u16 prev = 0xffff;
				static int n = 0;
				if (data != prev && n++ < 25)
				{
					logerror("FW7A74 = %04x (pc=%06x) @%.4f\n", data, m_cpu->pc(), machine().time().as_double());
					prev = data;
				}
			});
	// TEMP LOG (STRIP): $7950 = the pump's phase toggle - fires iff the pump handler actually runs
	if (m_fw_driven)
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7950, 0x7951, "fwpumprun",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				static int n = 0;
				if (n++ < 20) logerror("FWPUMPRUN $7950 = %04x (pc=%06x) @%.4f\n", data, m_cpu->pc(), machine().time().as_double());
			});
	// TEMP LOG (STRIP): the pump-handler pointer $7940 - who installs the DMA-phase pump, and which
	if (m_fw_driven)
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7940, 0x7941, "fwpump",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				static int n = 0;
				if (n++ < 30)
				{
					logerror("FWPUMP $7940 = %04x (pc=%06x) @%.4f\n", data, m_cpu->pc(), machine().time().as_double());
					if (data == 0x298c || data == 0x29c0)
					{
						address_space &cs = m_cpu->space(AS_PROGRAM);
						for (u16 a = 0x72d6; a < 0x72ec; a += 2)
						{
							u16 const rec = cs.read_word(a);
							if (!rec) continue;
							logerror("FWPUMP rec@%04x -> %04x: flag=%04x watch=%04x(*=%04x) cmp=%04x hdlr=%04x\n",
									a, rec, cs.read_word(rec), cs.read_word(rec + 2),
									cs.read_word(cs.read_word(rec + 2)), cs.read_word(rec + 4), cs.read_word(rec + 6));
						}
					}
				}
			});
	// TEMP LOG (STRIP): fw error/completion codes - the result word lands at local IOPB+0x18 ($7208)
	if (m_fw_driven)
		m_cpu->space(AS_PROGRAM).install_write_tap(0x7208, 0x7209, "fwresult",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				static int n = 0;
				if (machine().time().as_double() > 6.5 && n++ < 40) logerror("O7208 = %04x pc=%06x @%.4f  (0x98e7=success  0x18@0x2b8a=timeout)\n", data, m_cpu->pc(), machine().time().as_double());
			});
}

// fallback: parse the SINIX0 floppy IMD into a (cyl,head,sector) -> bytes map (unused in the LLE path).
void multibus_storager_device::load_floppy()
{
	m_floppy_loaded = true;
	m_sectors.clear();
	// Read the floppy sector image from the mounted MAME image device (floppy0).  That device is
	// swappable live from the UI (Tab -> File Manager -> mount a new .imd), which is how the multi-disk
	// SINIX installation changes disks: floppy_swap_cb() drops this cache on swap and the next disk
	// access reloads from the newly-inserted image.  No media mounted = drive empty (the
	// hard-disk boot case: the ROM reports "no sys-floppy" and boots the rigid disk).
	floppy_image_device *const fdd0 = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
	if (!fdd0 || !fdd0->exists())
	{
		LOG("STORAGER: no floppy media\n");
		return;
	}
	// cont.263: PROOF-OF-CONCEPT - run the new 74LS1811 PLL separator over the real flux and
	// verify it recovers this track's ID fields byte-faithfully (cyl0 = FM label track). This is
	// the correctness oracle for the raw-bitstream pivot before it replaces build_serdes_stream.
	if (storager_getenv("STORAGER_PLLVERIFY"))
	{
		fdd0->mon_w(0);   // motor on so the flux passes under the head
		unsigned const nf = pll_verify_track(fdd0, true /*FM*/);   // cyl0 = FM label track
		logerror("PLL-VERIFY: 74LS1811 recovered %u FM sectors from cyl%d flux\n", nf, fdd0->get_cyl());
		// step to cyl1 (MFM 256B SINIX data) and verify the MFM path
		fdd0->dir_w(0);   // step in (toward higher cylinders)
		fdd0->stp_w(1); fdd0->stp_w(0); fdd0->stp_w(1);
		logerror("PLL-VERIFY: stepped to cyl%d (expect MFM)\n", fdd0->get_cyl());
		unsigned const nm = pll_verify_track(fdd0, false /*MFM*/);
		logerror("PLL-VERIFY: 74LS1811 recovered %u MFM sectors from cyl%d flux\n", nm, fdd0->get_cyl());
		// LIVE-RUN self-test (74LS1812): the RESUMABLE advance-to-now engine, in test mode (no CPU
		// IRQs). Reset at cyl1 (MFM) and advance in 2ms slices over one revolution; the FLUX-ID
		// marks must fire once per sector, proving the real-time byte/AM engine matches the offline
		// separator. (This is what E000 + the read-arm will drive once wired.)
		m_flux_test = true;
		flux_read_reset(fdd0, false /*MFM*/);
		attotime const t0 = machine().time();
		for (int s = 1; s <= 105; s++) flux_advance_to(t0 + attotime::from_usec(2000 * s));
		m_flux_test = false;
	}
	std::string const fpath = fdd0->filename();
	std::ifstream f(fpath, std::ios::binary);
	if (!f) { LOG("STORAGER: cannot open floppy image\n"); return; }
	std::vector<u8> d((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
	size_t p = 0;
	while (p < d.size() && d[p] != 0x1a) p++;
	p++;
	while (p + 5 <= d.size())
	{
		u8 const cyl = d[p+1], head = d[p+2], nsec = d[p+3], ssz = d[p+4];
		p += 5;
		if (ssz > 6) break;
		u32 const size = 128u << ssz;
		std::vector<u8> smap(d.begin()+p, d.begin()+p+nsec); p += nsec;
		if (head & 0x80) p += nsec; // cylinder map
		if (head & 0x40) p += nsec; // head map
		u8 const h = head & 0x3f;
		for (int i = 0; i < nsec && p < d.size(); i++)
		{
			u8 const t = d[p++];
			std::vector<u8> sec(size, 0);
			if (t==1 || t==3 || t==5 || t==7) { for (u32 j=0;j<size && p<d.size();j++) sec[j]=d[p++]; }
			else if (t==2 || t==4 || t==6 || t==8) { u8 const fill=d[p++]; std::fill(sec.begin(),sec.end(),fill); }
			m_sectors[(u32(cyl)<<16) | (u32(h)<<8) | smap[i]] = std::move(sec);
		}
	}
	LOG("STORAGER: floppy loaded, %u sectors\n", (unsigned)m_sectors.size());
	if (storager_getenv("STORAGER_UNITLOG"))   // TEMP (STRIP): confirm WHICH image + its VOL1 label (cyl0 h0 sec7)
	{
		auto const it = m_sectors.find((0u << 16) | (0u << 8) | 7u);
		char lbl[24] = {0};
		if (it != m_sectors.end())
			for (int k = 0; k < 20 && k < int(it->second.size()); k++)
				lbl[k] = (it->second[k] >= 0x20 && it->second[k] < 0x7f) ? char(it->second[k]) : '.';
		logerror("FLOPLOAD file=%s sectors=%u VOL1(c0h0s7)=[%s] @%.4f\n", fpath.c_str(), unsigned(m_sectors.size()), lbl, machine().time().as_double());
	}
	// spin the drive so INDEX pulses - done HERE (first access, after all device_reset) so mon_w(0) sticks
	// (the floppy's own device_reset runs after the storager's and would re-raise m_mon).
	for (int i = 0; i < 2; i++)
		if (floppy_image_device *const fdd = m_floppy[i]->get_device())
			fdd->mon_w(0);
}

void multibus_storager_device::floppy_swap_cb(floppy_image_device *)
{
	// The UI mounted or unmounted a disk image - drop the cached sector map so the next disk access
	// reloads it from the newly-inserted image (the SINIX install's disk-change step).
	m_floppy_loaded = false;
	m_sectors.clear();
	LOG("STORAGER: floppy media changed - sector map invalidated, will reload\n");
}

void multibus_storager_device::device_reset()
{
	if (!m_installed)
	{
		// Multibus PIO window 0x7200-0x73FF -> on-board dual-port RAM 0x7E00-0x7FFF (+0xC00).
		m_bus->space(AS_IO).install_readwrite_handler(0x7200, 0x73ff,
			read16sm_delegate(*this, FUNC(multibus_storager_device::host_win_r)),
			write16s_delegate(*this, FUNC(multibus_storager_device::host_win_w)));
		// data aperture the host reads kernel blocks through (@0xF00000 = bus I/O 0x0000).
		m_bus->space(AS_IO).install_read_handler(0x0000, 0x00ff,
			read16sm_delegate(*this, FUNC(multibus_storager_device::bus_data_r)));
		// STEP 138 (Dave: NSC-driver handshake): the loaded OS driver's command channel is a register
		// file at Multibus I/O 0x0800-0x0807: {cmd byte (written LAST; bit7 = controller DONE flag the
		// driver polls), bytes 4-7 = host IOPB pointer (LE)}. Previously unmapped - its writes vanished
		// and the poll read garbage forever (the post-"Boot:" stall).
		m_bus->space(AS_IO).install_readwrite_handler(0x0800, 0x0807,
			read16sm_delegate(*this, FUNC(multibus_storager_device::ioreg_r)),
			write16s_delegate(*this, FUNC(multibus_storager_device::ioreg_w)));
		// snoop the local-bus buffer (0x4000-0x7FFF): the gate array's address comparator watches
		// writes to advance the transfer pointer, and latches the terminal LSB from the helper's
		// read performed while DMA is active.
		address_space &cs = m_cpu->space(AS_PROGRAM);
		cs.install_read_tap(0x4000, 0x7fff, "dma_snoop_r",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ if (m_dma_active) m_term_bit0 = (mem_mask == 0x00ff) ? 1 : 0; });
		cs.install_write_tap(0x4000, 0x7fff, "dma_snoop_w",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ if (m_dma_active) m_last_bw = offset + ((mem_mask == 0x00ff) ? 1 : 0); });
		// TEMP: trace what the firmware writes back into the dual-port mailbox (the host result/status).
		cs.install_write_tap(0x7e00, 0x7fff, "mbox_w",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				if (m_trace) LOG("  MBOX fw wr %04x = %04x (mask %04x) [%06x]\n", offset, data, mem_mask, m_cpu->pc());
				// TEMP: on command-done (firmware clears the command word 0x7ff8), pulse a Multibus
				// INT to the CPUAP's ICU to test the completion-interrupt hypothesis.
				// command done (firmware posts to mailbox 0x7fe8 @pc 0x2604) -> post DONE (0x80) to the
				// host IOPB status in CPUAP RAM, which the monitor polls (IOPB+2: 0x81 busy -> 0x80 done).
				// task#5 BUSY-HOLD: a restore/seek (0x89/0x98) is a 75ms physical op; the CPUAP polls IOPB+2 and
				// waits for DONE (measured cross-device).  The fw posts $7fe8 within us (command-accepted), but the
				// real controller HOLDS BUSY (0x81) through the seek and posts DONE only at settle.  Suppress the
				// premature CMDDONE for the pending seek's IOPB; seek_done_tick (75ms) posts its DONE.  This spaces
				// the CPUAP's command sequence so the read arrives at a clean channel and dispatches to $5fc0.
				bool const seek_busy = storager_getenv("STORAGER_BUSYHOLD") && m_seek_iopb && !m_seek_fired && m_iopb_addr == m_seek_iopb;
				// cont.77: THE PHANTOM-COMPLETION FIX (the $7fe8 ghost's true form). The fw posts
				// $7fe8 at ACCEPT ($0101) and at COMPLETION ($00ba-family) - bit7 of the posted
				// byte distinguishes them (0x80|code = done). The blind translation posted host-
				// DONE at every accept once cont.60 released m_read_pending - phantom completions
				// driving the CPUAP's 10ms probe cascade. Translate COMPLETION posts only.
				if (offset == 0x7fe8 && m_iopb_addr && (data & 0x0080) && !m_read_pending && !seek_busy)   // STEP 324 + cont.77
				{
					address_space &bs = m_bus->space(AS_PROGRAM);
					if (storager_getenv("STORAGER_NOBYPASS")) logerror("IOPB80-POST L%d @%.5f\n", __LINE__, machine().time().as_double());
					if (storager_getenv("STORAGER_FWDONE"))
					{
						// cont.247b: FWDONE = the fw posts its own status; the GA's job is the
						// TRANSCRIPTION of that byte to the host - the IOPB slots AND the fixed
						// status slot (EXT(6)+0x22 = 0xFE782) the monitor's fe3EBE bit0 poll
						// watches. The old blanket skip left the fixed slot 0x81 forever.
						// the fw's $1A54/$184E stamps live in the LOCAL work-area IOPB (+2/+3),
						// not the $7fe8 mailbox code byte - transcribe those
						address_space &lcs = m_cpu->space(AS_PROGRAM);
						u8 const st2 = lcs.read_byte(0x71f2), st3 = lcs.read_byte(0x71f3);
						// cont.247c: transcribe only a REAL stamp (bit7 set: 0x80/0x82) - the
						// local cells are 00 before the fw's $1A54/$184E write and forwarding
						// that released the monitor 9ms after issue with no data delivered
						if (!(st2 & 0x80))
						{
							logerror("IOPB-FWSTAT premature (local=%02x/%02x) - not transcribed @%.5f\n", st2, st3, machine().time().as_double());
							return;
						}
						bs.write_byte((m_iopb_addr + 2) & 0xffffff, st2);
						bs.write_byte((m_iopb_addr + 3) & 0xffffff, st3);
						bs.write_byte(0x0fe782, st2);
						bs.write_byte(0x0fe783, st3);
						logerror("IOPB-FWSTAT transcribed %02x/%02x (local 71f2/3 -> iopb+2/3 + fixed fe782) @%.5f\n", st2, st3, machine().time().as_double());
					} else {
							bs.write_byte((m_iopb_addr + 2) & 0xffffff, 0x80);
					bs.write_byte((m_iopb_addr + 3) & 0xffffff, 0x80);
					}
					// cont.247: the R0 status register observes the DONE edge REGARDLESS of who
					// transcribes the IOPB status - under FWDONE the fw's own post is the
					// transcription, but the model's R0 busy latch still needs the edge (the
					// monitor's next command polls R0-idle before its GO; a stuck busy parks
					// it forever at the fe3EC3-class pre-issue poll).
					r0_observe(0x80);
					LOG("CMDDONE 7fe8=%04x -> iopb %06x +2/+3 <- 0x80 (done) pc=%06x @%.4f\n", data, m_iopb_addr, m_cpu->pc(), machine().time().as_double());
					{ m_want_ready = false;   // cont.150: the DONE post consumes the want-ready level
					if (storager_getenv("STORAGER_PHASELOG")) logerror("CMDDONE fw wrote $7fe8 -> host-DONE to iopb=%06x cmd=%02x pc=%06x @%.5f\n", m_iopb_addr & 0xffffff, m_iopb_cmd, m_cpu->pc(), machine().time().as_double()); }
				}
				// cont.255b: the ACCEPT edge ($7fe8 = $0101, bit7 clear) must ALSO transcribe to
				// the host as BUSY - fixed slot 0xFE782 bit0 SET (0x01) + IOPB status 0x81. The
				// completion-only translation left the PREVIOUS command's 0x80 in the fixed slot,
				// so the monitor's fe3EBE poll saw instant-done on the next command and issued a
				// THIRD 95 mid-read2 (run257 @8.0003: second 95 on node $71C6 stomped [$71bc]).
				else if (offset == 0x7fe8 && m_iopb_addr && !(data & 0x0080))
				{
					address_space &bs = m_bus->space(AS_PROGRAM);
					bs.write_byte((m_iopb_addr + 2) & 0xffffff, 0x81);
					bs.write_byte(0x0fe782, 0x01);
					bs.write_byte(0x0fe783, 0x01);
					if (storager_getenv("STORAGER_NOBYPASS")) logerror("IOPB81-BUSY accept transcribed (7fe8=%04x) iopb=%06x cmd=%02x @%.5f\n", data, m_iopb_addr & 0xffffff, m_iopb_cmd, machine().time().as_double());
				}
				else if (offset == 0x7fe8 && seek_busy && storager_getenv("STORAGER_PHASELOG"))
					logerror("BUSY-HOLD: suppressed premature DONE for seek iopb=%06x cmd=%02x (seek_done@75ms) @%.5f\n", m_iopb_addr & 0xffffff, m_iopb_cmd, machine().time().as_double());
				else if (offset == 0x7fe8)
					LOG("CMDDONE 7fe8=%04x but m_iopb_addr=0 (NO host status posted!) pc=%06x @%.4f\n", data, m_cpu->pc(), machine().time().as_double());
			});
		m_installed = true;
	}
	m_cpu->set_input_line(M68K_IRQ_2, CLEAR_LINE);
	m_dma_active = false;
	m_shadow_loaded = false;
	m_iopb_fetched = false;
	m_host_int_prev = false;
	m_e800_bit12_prev = false;
	// completion machinery: a soft reset must not carry outstanding completions or a held
	// INT2 into the next boot (stale state wedged the kernel's first disk wait after a
	// mid-session MAME reset)
	m_ioreg_done = false;
	if (m_ioreg_pending > 0)
		int_w<2>(1);
	m_ioreg_pending = 0;
	m_ioreg_int->adjust(attotime::never);
	// Spin the drive so INDEX pulses (5.25"): path (A) - the firmware runs its full "real floppy" logic
	// and polls F000 for INDEX/READY/TRACK0, so the media must present as a real spinning floppy.
	for (int i = 0; i < 2; i++)
		if (floppy_image_device *const fdd = m_floppy[i]->get_device())
			fdd->mon_w(0);
}

// C000-C7FF: the Multibus HOST-ADDRESS up-counter (VGC7219-GATE-ARRAY-SPEC.md §3.4, decoded
// 2026-07-17).  The whole range decodes ONE 24-bit register: a single write loads the one's-
// complemented host address with bits 23-16 on address lines A1-A8 and bits 15-0 on the data
// bus (the $3CD4 launcher: mem[$C000 + (~B>>15)&$1FE] = ~B&$FFFF, pair from [$79D8], computed
// by $128a from the $36E6 IOPB buffer-address extraction).  Complement-load = up-counter
// preset; the bus-master engine (IOPB fetch / status write-back / SRAM->host data DMA) walks
// host memory on this counter.  The descriptor-block flavor of the same preset (the ~B pair
// staged at $7446/$7448 by micro-op $56) loads the counter at descriptor consumption (DESCGO).
void multibus_storager_device::c000_w(offs_t offset, u16 data, u16 mem_mask)
{
	m_c000 = ~((u32(offset & 0xff) << 16) | data) & 0xffffff;
	m_c000_valid = true;
	if (storager_getenv("STORAGER_PHASELOG"))
		logerror("C000 preset %06x (write [%03x]=%04x) pc=%06x @%.5f\n",
			m_c000, (offset & 0x3ff) << 1, data, m_cpu->pc(), machine().time().as_double());
	if (m_trace) LOG("  C000 preset = %06x [%06x]\n", m_c000, m_cpu->pc());
}

// C800 field-boundary offset file (spec §3.4: cumulative field-boundary positions via the $3066
// triple encoding; cell 0 doubles as the live SRAM chunk pointer in the per-record cycle).  A
// write while DMA is active latches the transfer terminal: index = byte offset (offset*2) =
// address bits 7-11, value = address bits 1-6 (the single-boundary degenerate case).
u16 multibus_storager_device::c800_r(offs_t offset)
{
	return m_c800[offset & 0xff];
}

void multibus_storager_device::c800_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_c800[offset & 0xff]);
	// cont.57: the v2 bump-carry RETIRED - superseded by carry-by-aim at deposit time
	// (run197: arrival-order carries put VOL1 at host+512; the aim is the logical map).
	// cont.40b (STRIP): C800 INGEST PROOF - the fw's $308c script loader populates the per-page
	// translation file; the truck's kick-time lookup is only trustworthy if these cells land.
	if (storager_getenv("STORAGER_PHASELOG"))
	{ static int _cn = 0, _c1a = 0; double const _ct = machine().time().as_double();
		bool const _is1a = m_cpu->pc() >= 0x31c2 && m_cpu->pc() <= 0x31d8;
		if (_ct > 8.42 && (_is1a ? _c1a++ < 40 : _cn++ < 120))
		logerror("C800WR%s [%02x]=%04x pc=%06x @%.5f\n", _is1a ? "-1A" : "", offset & 0xff, data, m_cpu->pc(), _ct); }
	if (m_trace) LOG("  C800[%02x] = %04x dma=%d [%06x]\n", offset & 0x7f, data, m_dma_active, m_cpu->pc());
	if (m_dma_active)
		m_dma_term = (u16((offset & 0x7f) * 2) << 6) | ((data & 0x3f) << 1);   // address bits 1-11
	// cont.38n: the C800-load arm (cont.38m) is RETIRED - C800 is one RAM serving both the
	// terminal count and the fw's buffer free list ($30xx walks hit offset 0 too). The arm
	// is the E802 bit15 RISING WRITE (both flavors produce one at write granularity - even
	// $92b4's andi #$77ff / ori #$8a00 pair); see the pump-arm hook. No E800 kicks occur in
	// the per-record capture cycles (run89: only the 26ms timer-ISR touches).
}

// D000 = write-only DMA address latch (word address; reads of 0xD000 fall through to ROM).
void multibus_storager_device::d000_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_d000);
	if (m_trace) LOG("  D000 wr = %04x (byte addr %06x) [%s]\n", m_d000, u32(m_d000) << 1, machine().describe_context());
}

// D800 = write-only word-address latch: the channel parameter/status template block (every traced
// write is $3ED6 = $7DAC>>1; $7D9E variant for media $8B).  Local, NOT host-high - host addresses
// ride the C000 counter (spec §3.4, corrected 2026-07-17).  Sources: $7434 -> $d800, $741e -> $c800
// (both local word addresses: template / $7696 buffer chunk).
void multibus_storager_device::d800_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_d800);
	// cont.41a (STRIP): D800 writes in the command era - the truck's host-road candidates.
	if (storager_getenv("STORAGER_PHASELOG"))
	{ static int _dn = 0; double const _dt = machine().time().as_double();
		if (_dt > 6.0 && _dn++ < 40)
			logerror("D800WR %04x pc=%06x @%.5f\n", data, m_cpu->pc(), _dt); }
	if (storager_getenv("STORAGER_PHASELOG")) logerror("PHASE D800wr      D800<<1=%04x pc=%06x cmd=%02x @%.5f\n", u32(m_d800) << 1, m_cpu->pc(), m_iopb_cmd, machine().time().as_double());
	if (m_trace) LOG("  D800 wr = %04x  (template byte addr %05x) [%s]\n", m_d800, u32(m_d800) << 1, machine().describe_context());
}

// Multibus I/O 0x0000 data aperture: the host reads loaded kernel blocks here (@0xF00000 = bus I/O 0x0000).
// STEP 1 (Dave): LOG the access pattern (offset/count) before serving data; returns 0 for now.
u16 multibus_storager_device::bus_data_r(offs_t offset)
{
	LOG("F00000-aperture rd off=%04x [%s]\n", offset, machine().describe_context());
	return 0;
}

// Multibus memory window: the storager (a bus master) dereferences CPUAP RAM pointers (IOPB at
// 0x0fe780, data buffers) directly as 68000 addresses.  68000 is big-endian: high byte at the lower
// Multibus byte address.  Local ROM/RAM/registers (<0x10000) and the ROM mirror (0xff0000+) win.
u16 multibus_storager_device::bus_mem_r(offs_t offset, u16 mem_mask)
{
	u32 const addr = 0x010000 + offset * 2;
	address_space &bs = m_bus->space(AS_PROGRAM);
	u16 v = 0;
	if (ACCESSING_BITS_8_15) v |= u16(bs.read_byte(addr)) << 8;
	if (ACCESSING_BITS_0_7)  v |= bs.read_byte(addr + 1);
	if (m_trace) LOG("  BUSMEM rd %06x -> %04x (mask %04x) [%06x]\n", addr, v, mem_mask, m_cpu->pc());
	return v;
}

void multibus_storager_device::bus_mem_w(offs_t offset, u16 data, u16 mem_mask)
{
	u32 const addr = 0x010000 + offset * 2;
	address_space &bs = m_bus->space(AS_PROGRAM);
	if (ACCESSING_BITS_8_15) bs.write_byte(addr,     data >> 8);
	if (ACCESSING_BITS_0_7)  bs.write_byte(addr + 1, data & 0xff);
	// cont.206: observe the firmware's own IOPB status stamp (the §5.1 write-through) - the
	// R0-status DONE edge. The stamp covers m_iopb_addr+2 (status 0x80/0x82, bit7 set).
	if (m_iopb_addr)
	{
		u32 const st = (m_iopb_addr + 2) & 0xffffff;
		u8 stv = 0; bool hit = false;
		if (ACCESSING_BITS_8_15 && addr == st)     { stv = data >> 8;   hit = true; }
		if (ACCESSING_BITS_0_7  && addr + 1 == st) { stv = data & 0xff; hit = true; }
		if (hit && (stv & 0x80))
		{
			m_r0_busy = false;
			m_r0_doneint = true;
			if (storager_getenv("STORAGER_PHASELOG"))
				logerror("R0DONE fw-stamp %02x at iopb+2 pc=%06x @%.5f\n", stv, m_cpu->pc(), machine().time().as_double());
		}
	}
	if (m_trace) LOG("  BUSMEM wr %06x = %04x (mask %04x) [%06x]\n", addr, data, mem_mask, m_cpu->pc());
}

// CPUAP reads/writes the dual-port mailbox; map byte-for-byte into the 68000's RAM at +0xC00.
u16 multibus_storager_device::host_win_r(offs_t offset)
{
	u32 const fa = 0x7e00 + offset * 2;
	u32 const rpio = 0x7200 + offset * 2;
	address_space &cs = m_cpu->space(AS_PROGRAM);
	u16 v = cs.read_byte(fa) | (u16(cs.read_byte(fa + 1)) << 8);
	// cont.206 (STORAGER_R0STAT): serve the GENERATED R0-status for host reads of the command
	// register (pio 0x73F8 low byte) instead of the mailbox echo: bit0=Idle, bit1=Busy,
	// bit2=OPER-DONE-INT [R: exact bit position unconfirmed - MX300 says bit2/3=diag],
	// bits4-7 = units 0-3 ready (hd0, hd1, fd0, fd1).
	if (rpio == 0x73f8 && storager_getenv("STORAGER_R0STAT"))
	{
		u8 r0 = 0;
		if (m_r0_busy) r0 |= 0x02; else r0 |= 0x01;
		if (m_r0_doneint) r0 |= 0x04;
		if (m_hd[0] && m_hd[0]->exists()) r0 |= 0x10;
		if (m_hd[1] && m_hd[1]->exists()) r0 |= 0x20;
		if (m_floppy[0] && m_floppy[0]->get_device() && m_floppy[0]->get_device()->exists()) r0 |= 0x40;
		if (m_floppy[1] && m_floppy[1]->get_device() && m_floppy[1]->get_device()->exists()) r0 |= 0x80;
		{ static int _rn = 0; if (storager_getenv("STORAGER_PHASELOG") && _rn++ < 40)
			logerror("R0STAT rd -> %02x (mirror was %02x) @%.5f\n", r0, v & 0xff, machine().time().as_double()); }
		v = (v & 0xff00) | r0;
	}
	// cont.206 watch: EVERY host read of the register file (pio 73F4-73FF) - the instrument for
	// any OTHER bit-pattern the monitor/kernel expects the gate array to generate.
	{ static int _fn = 0;
		if (storager_getenv("STORAGER_PHASELOG") && rpio >= 0x73f4 && rpio <= 0x73ff && _fn++ < 60)
			logerror("REGFILE rd pio=%04x -> %04x @%.5f\n", rpio, v, machine().time().as_double()); }
	if (m_trace) LOG("  HOSTWIN rd pio=%04x fw=%04x -> %04x\n", rpio, fa, v);
	// task#5 (STRIP): what does the CPUAP POLL between commands? Log every host-window read in the command-burst
	// window (t 6.39-6.45) so we can see the BUSY/status field the SINIX driver waits on before ringing the next.
	if (storager_getenv("STORAGER_POLL")) { double const t = machine().time().as_double(); static int pn = 0, pn2 = 0;
		if (t > 6.39 && t < 6.46 && pn++ < 200) logerror("CPUAP-POLL fw=%04x(pio=%04x) -> %04x @%.6f\n", fa, 0x7200 + offset*2, v, t);
		// cont.172: the give-up era - what does the monitor poll and what is it told, 7.9-8.3
		if (t > 7.90 && t < 8.30 && pn2++ < 200) logerror("CPUAP-POLL2 fw=%04x(pio=%04x) -> %04x @%.6f\n", fa, 0x7200 + offset*2, v, t); }
	// STEP 138 (Dave: decode the NSC-driver handshake): log window READS after the boot line, deduped
	// on (addr,value) change so a poll loop logs once per state change.  STRIP.
	{
		double const t = machine().time().as_double();
		if (t > 3.8 && !machine().side_effects_disabled())
		{
			static u32 lastfa = ~0u; static u16 lastv = 0; static int n = 0;
			if ((fa != lastfa || v != lastv) && n < 150)
			{
				n++;
				LOG("HOSTRD fw=%04x -> %04x @%.4f\n", fa, v, t);
				lastfa = fa; lastv = v;
			}
		}
	}
	return v;
}

void multibus_storager_device::host_win_w(offs_t offset, u16 data, u16 mem_mask)
{
	u32 const pio = 0x7200 + offset * 2;
	u32 const fa  = 0x7e00 + offset * 2;
	address_space &cs = m_cpu->space(AS_PROGRAM);
	// cont.210 (STRIP): every host register write, unfiltered - which pio the monitor really
	// touches (the phantom ch0 claim needs a ch0-side write trigger or a model deposit).
	if (storager_getenv("STORAGER_PHASELOG") && machine().time().as_double() > 6.3)
	{ static int _hw = 0; if (_hw++ < 120)
		logerror("HOSTWR pio=%04x data=%04x mm=%04x @%.6f\n", pio, data, mem_mask, machine().time().as_double()); }
	// cont.203 (STORAGER_MBOXW): the host I/O REGISTER FILE maps register-per-WORD into fw RAM
	// (the dual-port spreads each 8-bit host register into the low byte of its own 16-bit word:
	// ch0 R0-R3 @73F4-73F7 -> $7FF0/2/4/6, ch1 R0-R3 @73F8-73FB -> $7FF8/A/C/E). The fw reads
	// the mailbox at +0/+2/+4/+6 ($2018-$2030) - the byte-contiguous relay fed it R2 as the
	// address HIGH byte (B=0xE7xxxx, the whole varying-preset family). Writer-trace proof:
	// the monitor's R1 write arrives as the 0x73F8 word's HIGH lane (data=0F00).
	if (storager_getenv("STORAGER_MBOXW") && pio >= 0x73f4 && pio <= 0x73fb)
	{
		m_mb_depositing = true;
		if (ACCESSING_BITS_0_7)  { cs.write_byte(0x7ff0 + (pio - 0x73f4) * 2, data & 0xff); m_mb_in[pio - 0x73f4] = data & 0xff; }
		if (ACCESSING_BITS_8_15) { cs.write_byte(0x7ff0 + (pio + 1 - 0x73f4) * 2, data >> 8); if (pio + 1 <= 0x73fb) m_mb_in[pio + 1 - 0x73f4] = data >> 8; }
		m_mb_depositing = false;
	}
	else
	{
		if (ACCESSING_BITS_0_7)  cs.write_byte(fa,     data & 0xff);
		if (ACCESSING_BITS_8_15) cs.write_byte(fa + 1, data >> 8);
	}
	if ((fa == 0x7ff8 || fa == 0x7ff9) && storager_getenv("STORAGER_PHASELOG"))   // task#4 writer-trace: tag HOST writes to the ch1 mailbox (observe-only)
		logerror("7ff8-WRITER=HOST pio=%04x data=%04x @%.5f\n", pio, data, machine().time().as_double());

	// cont.206: R0 command-register lifecycle (any GO family: bit0 = GO, bit1 = CLR-INT)
	if (pio == 0x73f8 && ACCESSING_BITS_0_7)
	{
		u8 const r0cmd = data & 0xff;
		if (r0cmd & 0x02) m_r0_doneint = false;
		if (r0cmd & 0x01) m_r0_busy = true;
	}
	// doorbell: the CPUAP writes GO (0x13) to the command register (PIO 0x73F8) last -> raise IRQ2.
	if (pio == 0x73f8 && ACCESSING_BITS_0_7 && (data & 0xff) == 0x13)
	{
		// The gate array auto-fetches the IOPB (14 words = 0x1c bytes) from the mailbox pointer into the
		// firmware's IOPB buffer ($7a06) on EVERY doorbell - the firmware reads it without a kickoff
		// (e.g. RESTORE).  Copy ONLY 0x1c bytes so the work area at +0x1c is preserved.
		{
			u32 const dbi = storager_getenv("STORAGER_MBLATCH")
				? ((u32(m_mb_in[5]) << 16) | (u32(m_mb_in[6]) << 8) | m_mb_in[7])
				: storager_getenv("STORAGER_MBOXW")
				? ((u32(cs.read_byte(0x7ffa)) << 16) | (u32(cs.read_byte(0x7ffc)) << 8) | cs.read_byte(0x7ffe))
				: ((u32(cs.read_byte(0x7ff9)) << 16) | (u32(cs.read_byte(0x7ffa)) << 8) | cs.read_byte(0x7ffb));
			u32 dst = cs.read_word(0x7a06);
			if (dst < 0x4000 || dst >= 0x7e00) dst = 0x71f0;
			address_space &bs = m_bus->space(AS_PROGRAM);
			m_dma_mark = 1;   // trace: doorbell IOPB fetch, dst from [$7a06] (fw pointer)
			for (u32 k = 0; k < 0x18; k++)   // cont.209: 12 words - +0x18/+0x19 is the fw's error cell, NOT host IOPB
				cs.write_byte((dst + k) & 0xffff, bs.read_byte((dbi + k) & 0xffffff));
			m_dma_mark = 0;
			m_iopb_cmd = cs.read_byte(dst);
			m_iopb_buffer = ((u32(cs.read_byte(dst + 0xc)) << 24) | (u32(cs.read_byte(dst + 0xd)) << 16)
				| (u32(cs.read_byte(dst + 0xe)) << 8) | cs.read_byte(dst + 0xf)) & 0xffffff;
			m_iopb_addr = dbi;
			if (storager_getenv("STORAGER_PHASELOG") && machine().time().as_double() > 6.3) { static int _cn = 0; if (_cn++ < 120) logerror("DOORCMD cmd=%02x 749c=%04x(%s) iopb=%06x [2..5]=%02x %02x %02x %02x pc=%06x @%.4f\n", m_iopb_cmd, cs.read_word(0x749c), cs.read_word(0x749c) ? "BUSY->channel-svc" : "IDLE->dispatch", dbi & 0xffffff, bs.read_byte((dbi + 2) & 0xffffff), bs.read_byte((dbi + 3) & 0xffffff), bs.read_byte((dbi + 4) & 0xffffff), bs.read_byte((dbi + 5) & 0xffffff), m_cpu->pc(), machine().time().as_double()); }
			// cont.229: which UIB governs each READ - dump $6E60/$6C00 + the active ptr [$799A]
			if (storager_getenv("STORAGER_PHASELOG") && m_iopb_cmd == 0x95 && machine().time().as_double() > 2.5)
			{ static int _ud = 0; if (_ud++ < 8)
				{ std::string u1, u2;
					for (u32 k = 0; k < 8; k++) { u1 += util::string_format(" %02x", cs.read_byte(0x6e60 + k)); u2 += util::string_format(" %02x", cs.read_byte(0x6c00 + k)); }
					logerror("READ-UIBS 799a=%04x 6e60:%s 6c00:%s @%.5f\n", cs.read_word(0x799a), u1.c_str(), u2.c_str(), machine().time().as_double()); } }
			if (m_iopb_cmd == 0x80 || (dbi & 0xff0000) == 0)
			{
				std::string ib; for (u32 k = 0; k < 0x1c; k++) ib += util::string_format(" %02x", cs.read_byte((dst + k) & 0xffff));
				std::string mb; for (u32 k = 0x7ff8; k <= 0x7fff; k++) mb += util::string_format(" %02x", cs.read_byte(k));
				LOG("CMD-%02x IOPB@%06x:%s  mailbox7ff8:%s buf=%06x\n", m_iopb_cmd, dbi, ib.c_str(), mb.c_str(), m_iopb_buffer);
			}
			if (m_iopb_cmd == 0x95 || m_iopb_cmd == 0x98)
			{
				// DIAG (Dave): decode the READ-SEQ / SEEK(0x98) fields.  Dump raw + BE-word/long candidates so
				// we can find the block-address / target-cylinder field.
				std::string ib, w, l;
				for (u32 k = 0; k < 0x1c; k++) ib += util::string_format(" %02x", cs.read_byte((dst + k) & 0xffff));
				for (u32 k = 0; k < 0x1c; k += 2) w += util::string_format(" [%x]%04x", k, (u32(cs.read_byte((dst+k)&0xffff))<<8)|cs.read_byte((dst+k+1)&0xffff));
				for (u32 k = 0; k <= 0x18; k += 4) l += util::string_format(" [%x]%08x", k, (u32(cs.read_byte((dst+k)&0xffff))<<24)|(u32(cs.read_byte((dst+k+1)&0xffff))<<16)|(u32(cs.read_byte((dst+k+2)&0xffff))<<8)|cs.read_byte((dst+k+3)&0xffff));
				LOG("CMD%02x IOPB:%s\n      BEw:%s\n      BEl:%s buf=%06x\n", m_iopb_cmd, ib.c_str(), w.c_str(), l.c_str(), m_iopb_buffer);
			}
			m_iopb_fetched = true;            // already fetched; the kickoff does the data/UIB transfer
			if (storager_getenv("STORAGER_REC512") && (m_iopb_cmd == 0x95 || m_iopb_cmd == 0x94))
			{
				m_desc_lastpair = 0xffffffff;   // cont.241h: new command - its staging is fresh even if same-valued
				m_rec_open = false; m_rec_quiet = 0; m_rec_base = 0; m_rec_first_valid = false; m_rec_convert = false;
			}
			if (m_iopb_cmd == 0x89) { m_rdptr = 0; u32 const ru = cs.read_byte((dst + 4) & 0xffff) & 3; m_unit_trk[ru] = 0; m_unit_sidx[ru] = 0; m_seek_deadline = machine().time() + attotime::from_msec(75); m_seek_fired = false; m_seek_iopb = m_iopb_addr; m_seek_done->adjust(attotime::from_msec(75)); }   // cont.239: unconditional - the HLE oracle needs it too // RESTORE: unit position to track0/sec0 + head-load busy 75ms (datasheet); arm the per-op seek-complete timer on THIS iopb
			else if (m_iopb_cmd == 0x98) { m_rdptr = (u32(cs.read_byte((dst + 6) & 0xffff)) << 8) | cs.read_byte((dst + 7) & 0xffff); m_seek_deadline = machine().time() + attotime::from_msec(75); m_seek_fired = false; m_seek_iopb = m_iopb_addr; m_seek_done->adjust(attotime::from_msec(75)); }   // cont.239: unconditional (HLE oracle) // SEEK: latch target[6-7] + ARM seek-active (F000 bit1) so the seek loop entry sees busy (75ms head-load, datasheet); arm the per-op completion timer on THIS iopb
			else if (m_iopb_cmd == 0x95 && !m_fw_driven) read95_deliver(dst);   // STEP 133 shim: deliver at the doorbell (fw-driven mode: the fw reads through the real channel)
			if (m_iopb_cmd == 0xa1)
			{
				// warm-reboot re-init: the firmware's handler ($4A66) restores its power-on
				// channel configuration, and the ROM then SKIPS the INIT before its label read
				// (observed: the reboot flow has no cmd-0x80 INIT). Mirror it in the model's
				// per-unit UIB latches or the label read decodes track 0 with the kernel-era
				// geometry (256B MFM instead of 128B FM) and returns zeros -> "no sys-floppy".
				// The 68000 itself is NOT reset (the fw runs its own handler).
				for (unsigned u = 0; u < 4; u++)
				{
					m_unit_heads[u] = 2;
					m_unit_spt[u] = 16;
					m_unit_secsize[u] = 128;
					m_unit_sec0[u] = (u == 1) ? 6 : 7;
					m_unit_btrk[u] = 0;
					m_unit_bsidx[u] = 0;
					m_unit_trk[u] = 0;
					m_unit_sidx[u] = 0;
				}
				LOG("CMD A1 REINIT: unit UIB latches -> power-on defaults @%.4f\n", machine().time().as_double());
			}
			// (the firmware also runs its own 0xA1 handler - a model-side 68000 reset here
			// makes the ROM race the firmware's power-on self-test and hang)
			// cmd 0x87 (HD identify): deliberately UNANSWERED - the ROM tolerates the probe
			// timeout at power-on and falls back to the floppy; a completed reply (any status)
			// diverts it into the HD-boot path, which needs the fw-driven HD read (Phase C)
			bs.write_byte((dbi + 2) & 0xffffff, 0x81);   // BUSY (per-IOPB)
			bs.write_byte((dbi + 3) & 0xffffff, 0x81);
			bs.write_byte(0x0fe782, 0x81);   // TEST (Dave): also the FIXED host status slot (EXT(6)+0x22)
			bs.write_byte(0x0fe783, 0x81);
			// cont.240 (HLE oracle only): post the 95/94's DONE directly - the fw's $7fe8
			// heartbeat misattributes the completion to whatever IOPB m_iopb_addr last held
			// (observed: the 95 at 0fe948 starved while the 87's 0fe780 got the post) - the
			// LLE path (m_fw_driven) is untouched
			if (!m_fw_driven && (m_iopb_cmd == 0x95 || m_iopb_cmd == 0x94))
			{
				bs.write_byte((dbi + 2) & 0xffffff, 0x80);
				bs.write_byte((dbi + 3) & 0xffffff, 0x80);
				bs.write_byte(0x0fe782, 0x80);
				bs.write_byte(0x0fe783, 0x80);
				r0_observe(0x80);
				m_iopb_addr = 0;
				logerror("HLE95-DONE posted iopb=%06x cmd=%02x @%.5f\n", dbi, m_iopb_cmd, machine().time().as_double());
			}
			m_read_pending = (m_fw_driven && !storager_getenv("STORAGER_NOBYPASS") && (m_iopb_cmd == 0x95 || m_iopb_cmd == 0x94));   // STEP 324/326: HLE holds BUSY until the shim delivers.  Under NOBYPASS the read is FW-DRIVEN (no shim), and the shim's read_pending clears are off - so setting it here permanently suppresses the mbox DONE and wedges the host handshake after the 0x95 doorbell.  The fw posts its own DONE at $0BF6.
			LOG("DOORBELL fetch: IOPB %06x -> %04x cmd=%02x buffer=%06x\n", dbi, dst, m_iopb_cmd, m_iopb_buffer);
			// TEMP cont.255d (STRIP): the full 0x18-byte IOPB ask per fetch - read1-vs-read2
			// difference lives in these bytes (logical address +6..+9, count +A/+B, options)
			if (storager_getenv("STORAGER_NOBYPASS"))
			{
				std::string ib;
				for (int k = 0; k < 0x18; k++) ib += util::string_format(" %02x", bs.read_byte((dbi + k) & 0xffffff));
				logerror("IOPBDUMP cmd=%02x:%s @%.5f\n", m_iopb_cmd, ib.c_str(), machine().time().as_double());
			}
			// TEMP (STRIP): the UIB status shadow around the 0x87 interrogation
			{
				static int un = 0;
				if (un++ < 12)
				{
					u16 const uib = cs.read_word(0x799a);
					std::string ub;
					for (u32 k = 0x10; k < 0x18; k++) ub += util::string_format(" %02x", cs.read_byte((uib + k) & 0xffff));
					logerror("UIB@%04x [+10..+17]:%s  7a74=%04x 7a36=%04x @%.4f\n", uib, ub.c_str(), cs.read_word(0x7a74), cs.read_word(0x7a36), machine().time().as_double());
				}
			}
		}
		// command word @0x7ff8 = cmd byte + 24-bit big-endian IOPB pointer into CPUAP RAM.
		u32 const iopb = storager_getenv("STORAGER_MBLATCH") ? ((u32(m_mb_in[5]) << 16) | (u32(m_mb_in[6]) << 8) | m_mb_in[7]) : storager_getenv("STORAGER_MBOXW") ? ((u32(cs.read_byte(0x7ffa)) << 16) | (u32(cs.read_byte(0x7ffc)) << 8) | cs.read_byte(0x7ffe)) : ((u32(cs.read_byte(0x7ff9)) << 16) | (u32(cs.read_byte(0x7ffa)) << 8) | cs.read_byte(0x7ffb));
		LOG("CMD %02x IOPB-ptr=%06x\n", cs.read_byte(0x7ff8), iopb);
		m_cpu->set_input_line(M68K_IRQ_2, ASSERT_LINE);
	}
	else if (pio >= 0x73e0 && pio <= 0x73f1)   // ch0 CCB + ch0 GO byte ONLY - 0x73f2+ is ch1's IOPB pointer/GO area (waking on those half-written mailbox writes breaks the ROM path)
	{
		// STEP 135 (experimental): the fw dispatcher @0x24c6 services TWO command channels (ch0: mailbox
		// byte $7ff0/CCB $7fe0/work $71c6; ch1: $7ff8/$7fe8/$71f0 = the ROM's GO path). The loaded NSC
		// Boot driver talks via the ch0/CCB area (writes $7fe2/$7fe8/$7fea then polls its cmd byte bit7).
		// Wake the firmware (IRQ2) on any host write to the mailbox-control tail so its own dispatcher
		// can examine the channels - the real gate array likely raises the host-command interrupt for
		// these writes too, not just the 0x13 GO.
		LOG("CH0/CCB wr pio=%04x fw=%04x = %04x (mask %04x) -> IRQ2 @%.4f\n", pio, fa, data, mem_mask, machine().time().as_double());
		m_cpu->set_input_line(M68K_IRQ_2, ASSERT_LINE);
	}
	else if (pio == 0x73f8 && ACCESSING_BITS_0_7)
	{
		// STEP 138 (Dave: handshake decode): a NON-0x13 write to the ch1 GO byte (low lane; the ROM's
		// IOPB-pointer bytes ride the HIGH lane so this can't fire mid-setup).  The gate array cannot
		// inspect values - on real hardware any GO-byte write raises the host-command interrupt and the
		// fw dispatcher @0x24c6 edge-detects the byte itself.  The NSC driver's last visible act is a
		// 16-bit clear of GO+ptr-hi (0x13 -> 0x00): wake the fw and let ITS dispatcher decide.  No
		// gate-array IOPB auto-fetch here (that is modeled only for the 0x13 protocol).
		LOG("GO-EDGE wr pio=73f8 GO=%02x (was-nonstd) -> IRQ2 @%.4f\n", data & 0xff, machine().time().as_double());
		m_cpu->set_input_line(M68K_IRQ_2, ASSERT_LINE);
	}
	else if (m_trace)
		LOG("HOSTWIN wr pio=%04x fw=%04x = %04x (mask %04x)\n", pio, fa, data, mem_mask);
}

// Multibus I/O 0x0800-0x0807: the OS-driver command channel (layout confirmed against the kernel's
// sacmd @0x9504, sasi.c 5.54):
//   [0]   = command byte, written LAST (bit0=1 -> interrupt-driven, else the driver polls bit7 = DONE)
//   [1]   = channel status, read back as (status & 3), 0 = OK
//   [2-3] = request/REPLY buffer pointer (low 16 bits; [6] = bits 16-23) - data commands read/write
//           through it, status commands (c0 sub-7) get their result structure WRITTEN here
//   [4-7] = device block pointer (LE): dev[0]=op, dev[1]=unit<<5|pos-hi, dev[2-3]=position (BE),
//           dev[4-7]=count (LE), dev[5]=subcode for c0/c2
u16 multibus_storager_device::ioreg_r(offs_t offset)
{
	u16 v = u16(m_ioreg[offset * 2]) | (u16(m_ioreg[offset * 2 + 1]) << 8);
	if (offset == 0 && m_ioreg_done)
	{
		v |= 0x80;   // cmd byte bit7 = DONE
		if (!machine().side_effects_disabled() && m_ioreg_pending > 0)
		{
			// acknowledge ONE completion; keep INT2 asserted while others remain outstanding
			// so the level-triggered ICU re-triggers the handler until the count reaches zero
			int_w<2>(1);
			if (--m_ioreg_pending > 0)
				m_ioreg_int->adjust(attotime::from_usec(20));   // re-edge for the next outstanding completion
		}
	}
	{
		double const t = machine().time().as_double();
		static int n = 0;
		if (t > 6.9 && t < 7.1 && n++ < 40 && !machine().side_effects_disabled())
			LOG("IOREG rd 0x80%x -> %04x (%s) @%.5f\n", offset * 2, v, machine().describe_context(), t);
	}
	return v;
}

void multibus_storager_device::ioreg_w(offs_t offset, u16 data, u16 mem_mask)
{
	if (ACCESSING_BITS_0_7)  m_ioreg[offset * 2]     = data & 0xff;
	if (ACCESSING_BITS_8_15) m_ioreg[offset * 2 + 1] = data >> 8;
	LOG("IOREG wr 0x80%x = %04x (mask %04x) [%s] @%.4f\n", offset * 2, data, mem_mask, machine().describe_context(), machine().time().as_double());
	if (offset == 0 && ACCESSING_BITS_0_7)
	{
		m_ioreg_done = false;   // new command accepted: DONE drops until this command completes
		m_temp_cid++;   // TEMPORAL PROBE (STRIP)
		logerror("TEMPO SUBMIT cid=%u cmd=%02x t=%.6f\n", m_temp_cid, data & 0xff, machine().time().as_double());
		// command-byte write: decode + dump the host block at the pointer, then ACK (bit7) so the driver
		// proceeds and reveals its next command - STEP 138 probe: observed cmd 0x00 (presence/reset?) and
		// 0x02 (retry) with ptr=0xbd74; reg[1] is read back as a status byte (&3, 0=OK) after DONE.
		// pointer lane assembly per the kernel's sacmd @0x9504: dev-blk = regs [4],[5],[7] (24-bit),
		// request/reply buffer = regs [2],[3],[6] (24-bit). Reg [6] is NOT the dev-blk's third byte.
		u32 const ptr = u32(m_ioreg[4]) | (u32(m_ioreg[5]) << 8) | (u32(m_ioreg[7]) << 16);
		u32 const req = u32(m_ioreg[2]) | (u32(m_ioreg[3]) << 8) | (u32(m_ioreg[6]) << 16);
		address_space &bs = m_bus->space(AS_PROGRAM);
		std::string ib, rb;
		for (u32 k = 0; k < 0x20; k++) ib += util::string_format(" %02x", bs.read_byte((ptr + k) & 0xffffff));
		if (req) for (u32 k = 0; k < 0x20; k++) rb += util::string_format(" %02x", bs.read_byte((req + k) & 0xffffff));
		LOG("IOREG CMD=%02x dev-blk@%06x:%s  req-blk@%06x:%s @%.4f\n",
			data & 0xff, ptr & 0xffffff, ib.c_str(), req, req ? rb.c_str() : " (none)", machine().time().as_double());
		{
			// STEP 142: if dev-ptr = the standalone-lib iob, the deep fields are (NS32k sys.c 4.9):
			// +0x9A i_boff, +0xA6 i_bn, +0xAA i_ma (BUFFER!), +0xAE i_cc, +0xC2 i_buf (sblock heap buf)
			std::string dp;
			for (u32 k = 0x98; k < 0xc8; k++) dp += util::string_format(" %02x", bs.read_byte((ptr + k) & 0xffffff));
			LOG("IOREG iob+98..c7:%s\n", dp.c_str());
		}
		// cmd 0x02 with device-block op 0x08 = READ: deliver from the media. Fields (STEP 138 decode):
		// dev[2-3] LE = host buffer, dev[4-7] LE = position in 1KB BLOCKS absolute on the media (0x20 =
		// byte 0x8000 = cyl4/hd0 = the SINIX filesystem volume start - the fs region begins after the
		// ANSI boot area at cyl4; units match the loader's DIVD 0x400). 1KB per request (superblock size).
		// dispatch the device-block op for ANY command submission: the kernel uses cmd byte 0x00
		// (plain), 0x02, and 0x03 (bit0 = interrupt-driven) - the op lives in the device block
		if (ptr)
		{
			u8 const op = bs.read_byte(ptr & 0xffffff);
			u8 const chunit = bs.read_byte((ptr + 1) & 0xffffff) >> 5;   // dev-blk[1] bits 5-7 = unit (2/3 = floppy, 0/1 = ESDI HD)
			if ((op == 0x08 || op == 0x0a) && chunit < 2)
			{
				// HD unit: serve from the MC1325 image (working copy; Dave's original stays read-only).
				// Position and count are in 1KB sectors (the kernel's c2 SET-CONFIG for the HD entries
				// carries 1024 bytes/sector; sasiopen reads the 1KB label at position 0).
				u32 const buf = req;
				// position = 21 bits: dev-blk [1] bits 0-4 = the HIGH bits (the builder ORs
				// EXTSD(pos,16,5) into the unit byte), [2-3] BE = the low 16 - without the
				// high bits every track past 64MB wraps and the format's verify pass marks
				// all upper tracks (7281+) defective in an endless spare-assignment loop
				u32 const psec = (u32(bs.read_byte((ptr + 1) & 0xffffff) & 0x1f) << 16)
							   | (u32(bs.read_byte((ptr + 2) & 0xffffff)) << 8) | bs.read_byte((ptr + 3) & 0xffffff);
				// TEMP (STRIP): 64MB-wrap probe. Log HD ops near/past the 64MB boundary with the raw
				// dev-blk position bytes so a dropped high bit (driver-side wrap) shows as pos_hi==0
				// for a track that should be >7281. Also flag any op that lands in the boot/label
				// area (psec < 256) after the format has begun writing high tracks (= a wrap victim).
				{
					static bool hi_seen = false; static int hn = 0, ln = 0;
					u32 const rawtrk = psec / 9;
					if (psec >= 0xff00 && hn++ < 40)
						logerror("HIPOS op=%02x psec=%u trk=%u devblk[1]=%02x pos_hi=%u @%.3f\n",
								op, psec, rawtrk, bs.read_byte((ptr + 1) & 0xffffff), bs.read_byte((ptr + 1) & 0xffffff) & 0x1f, machine().time().as_double());
					if (rawtrk >= 7000) hi_seen = true;
					if (hi_seen && op == 0x0a && psec < 256 && ln++ < 40)
						logerror("WRAP-VICTIM! op=%02x psec=%u (low sector written during high-track format) @%.3f\n",
								op, psec, machine().time().as_double());
				}
				u32 nsec = u32(bs.read_byte((ptr + 4) & 0xffffff)) | (u32(bs.read_byte((ptr + 5) & 0xffffff)) << 8)
						 | (u32(bs.read_byte((ptr + 6) & 0xffffff)) << 16) | (u32(bs.read_byte((ptr + 7) & 0xffffff)) << 24);
				// Interphase 2180 doctrine (the SINIX builder @0x9504 disassembled, STEP 251):
				// dev-blk [4] = the 8-bit SECTOR COUNT with "0 implies 256"; [5] = the BYTES/
				// TRANSACTION Multibus burst-size parameter (0xC0 = 192-byte bursts - a DMA
				// tuning knob, NOT part of the transfer length; the C0-status op carries its
				// subcode 7 there instead). Sectors are 1KB for the HD units (the c2 SET-CONFIG
				// declares 1024 bytes/sector). The transfer is a single contiguous DMA from the
				// request pointer.
				nsec = bs.read_byte((ptr + 4) & 0xffffff);
				// count 0: NOT the 2180's "256" here - the SINIX driver posts count-0 writes
				// whose data arrives in immediately-following per-buffer 4KB commands at the
				// same and successive sectors (observed throughout the mkfs phase); the count-0
				// command itself carries only its first 4KB buffer. A 256-sector reading
				// sprays 256KB of raw kernel memory across the disk (scattered superblock
				// images on the platter proved it).
				if (nsec == 0) nsec = 4;
				if (storager_getenv("STORAGER_UNITLOG2"))   // TEMP (STRIP): verbose per-op sector-unit trace (format-phase 512-vs-1024 analysis)
				{
					u8 db[8]; for (int i = 0; i < 8; i++) db[i] = bs.read_byte((ptr + i) & 0xffffff);
					logerror("UNIT op=%02x u=%u psec=%u nsec=%u byteoff=%llx devblk=%02x%02x%02x%02x%02x%02x%02x%02x @%.4f\n",
							op, chunit, psec, nsec, (unsigned long long)(u64(psec) * 1024),
							db[0], db[1], db[2], db[3], db[4], db[5], db[6], db[7], machine().time().as_double());
				}
				if (m_hd[chunit]->exists())
				{
					if (op == 0x08)
					{
						std::vector<u8> sec(nsec * 1024, 0);
						m_hd[chunit]->img_read(u64(psec) * 1024, sec.data(), u32(sec.size()));
						// Op 0x08 always DMAs to the host buffer. The STEP 368h "verify-flagged reads are
						// controller-local, no host DMA" suppression proved WRONG for the format's per-track
						// check (2026-07-08): those reads carry the same iob+0xb0=0x04/iob+0xb4=0x01 signature
						// yet the installer byte-compares the delivered data against its 0x1A pattern
						// ("Byte number 0 = 0 should be 1a" on every Spur with the suppression on). A real
						// controller transfers on every READ; the old 0x3470f4 corruption must have had a
						// different root (mis-timed/duplicated delivery, since fixed) - TEMPO/HDWR-OVERLAP
						// instrumentation stays armed below to catch it if it ever returns.
						bool const verify_op = (bs.read_byte((ptr + 0xb0) & 0xffffff) == 0x04)
										 && (bs.read_byte((ptr + 0xb4) & 0xffffff) == 0x01);
						if (verify_op)
							logerror("TEMPO VERIFY-OP cid=%u psec=%u req=%06x (verify-flagged; DMA delivered) t=%.6f\n",
									m_temp_cid, psec, buf, machine().time().as_double());
						for (u32 k = 0; k < u32(sec.size()); k++)
							bs.write_byte((buf + k) & 0xffffff, sec[k]);
						rd_check(chunit, psec, buf, nsec * 1024);   // TEMP (STRIP): read-delivery overlap
						// TEMPORAL PROBE (STRIP): DMA-complete event + cmd_id + source tag (Dave STEP 367d)
						{
							char const *src = "first-run";
							static u32 ls = ~0u, lb = ~0u; static double lt = 0;
							double const nt = machine().time().as_double();
							if (m_ioreg_done) src = "delayed-completion";   // DMA after the command already completed
							else if (psec == ls && buf == lb && (nt - lt) < 0.05) src = "retry";
							ls = psec; lb = buf; lt = nt;
							logerror("TEMPO DMA cid=%u op=%02x psec=%u nsec=%u req=%06x len=%u src=%s t=%.6f\n",
									m_temp_cid, op, psec, nsec, buf, nsec * 1024, src, nt);
						}
						// TEMP (STRIP): Dave's IOREG verify re-issue probe. A verify read the kernel re-issues
						// (same sector+buffer, back-to-back, short dt) = it never saw completion -> the retry
						// loop.  Log pending depth so a stacked/dropped INT2 shows as growing depth.
						{
							static u32 lastsec = ~0u, lastbuf = ~0u; static double lastt = 0; static int rn = 0;
							double const nt = machine().time().as_double();
							if (psec == lastsec && buf == lastbuf && (nt - lastt) < 0.05 && rn++ < 80)
								logerror("IOREG-REISSUE op08 sec=%u buf=%06x pending=%d dt=%.5f @%.4f\n",
										psec, buf, m_ioreg_pending, nt - lastt, nt);
							lastsec = psec; lastbuf = buf; lastt = nt;
						}
						LOG("IOREG HD READ sec=%u x%u -> host %06x  first16: %.16s @%.4f\n", psec, nsec, buf,
								reinterpret_cast<char *>(sec.data()), machine().time().as_double());
					}
					else   // op 0x0a = SASI WRITE, 0x0b = write-with-verify (the installer's label/mkfs path)
					{
						std::vector<u8> sec(nsec * 1024);
						for (u32 k = 0; k < u32(sec.size()); k++)
							sec[k] = bs.read_byte((buf + k) & 0xffffff);
						// TEMP EXPERIMENT (STRIP): skip the format phase's pattern writes (all-0x1a) so
						// only the verify READS run against a pre-filled disk - isolates format-write vs
						// verify-read as the corruption source. STORAGER_SKIPFMT=N skips format writes
						// for track >= N (N=0/empty skips all); sweep N to bisect the corrupting region.
						bool all1a = true;
						for (u8 b : sec) if (b != 0x1a) { all1a = false; break; }
						char const *const sf = storager_getenv("STORAGER_SKIPFMT");
						if (sf && all1a && (psec / 9) >= u32(atoi(sf)))
						{
							LOG("SKIPFMT: dropped format write sec=%u trk=%u x%u @%.4f\n", psec, psec / 9, nsec, machine().time().as_double());
							goto wr_done;
						}
						m_hd[chunit]->img_write(u64(psec) * 1024, sec.data(), u32(sec.size()));
						// TEMP (STRIP): 64MB boundary (psec 65536 / byteoff 0x4000000): position needs dev-blk[1]
						// bits 0-4.  Runs on EVERY write incl. the format 0x1a fill, never capped -- a dropped
						// high bit shows as poshi==0 for a byteoff that should be PAST-64M (or vice versa).
						if (storager_getenv("STORAGER_UNITLOG"))
						{
							u8 const db1x = bs.read_byte((ptr + 1) & 0xffffff);
							if (psec >= 0xf800 || (db1x & 0x1f) != 0)
								logerror("CROSS64 psec=%u (0x%x) poshi=%u db1=%02x byteoff=%llx %s nsec=%u all1a=%d buf=%06x @%.4f\n",
										psec, psec, db1x & 0x1f, db1x, (unsigned long long)(u64(psec) * 1024),
										(u64(psec) * 1024 >= 0x4000000ULL) ? "PAST-64M" : "under-64M", nsec, all1a ? 1 : 0, buf, machine().time().as_double());
						}
						// TEMP (STRIP): makeboot/mkfs corruption trace.  Skip the uniform 0x1a format fill;
						// for REAL data writes log (psec/nsec/rawcnt/buf/first bytes) and flag any sector
						// rewritten with DIFFERENT real data = the "fs full" clobber signature.  rawcnt==0
						// = the count-0 writes (the emulation expanded to nsec via the ==0->4 heuristic).
						if (storager_getenv("STORAGER_UNITLOG") && !all1a)
						{
							static std::map<u32,u64> rseen; static int nlog = 0, nclob = 0;
							u8 const raw4 = bs.read_byte((ptr + 4) & 0xffffff);
							for (u32 s = 0; s < nsec; s++)
							{
								u64 h = 1469598103934665603ULL;
								for (u32 j = 0; j < 1024; j++) { h ^= sec[s*1024+j]; h *= 1099511628211ULL; }
								auto it = rseen.find(psec + s);
								if (it != rseen.end() && it->second != h && nclob++ < 80)
									logerror("HDCLOBBER sec=%u oldh=%016llx newh=%016llx rawcnt=%u nsec=%u buf=%06x @%.4f\n",
											psec + s, (unsigned long long)it->second, (unsigned long long)h, raw4, nsec, buf, machine().time().as_double());
								rseen[psec + s] = h;
							}
							u8 const db1 = bs.read_byte((ptr + 1) & 0xffffff);   // bits 5-7 = unit, bits 0-4 = position HIGH (>64MB)
							if (nlog++ < 600)
								logerror("HDREAL psec=%u nsec=%u rawcnt=%u db1=%02x poshi=%u byteoff=%llx buf=%06x first8=%02x%02x%02x%02x%02x%02x%02x%02x @%.4f\n",
										psec, nsec, raw4, db1, db1 & 0x1f, (unsigned long long)(u64(psec) * 1024), buf,
										sec[0], sec[1], sec[2], sec[3], sec[4], sec[5], sec[6], sec[7], machine().time().as_double());
						}
						LOG("IOREG HD WRITE sec=%u x%u <- host %06x @%.4f\n", psec, nsec, buf, machine().time().as_double());
						wr_done:;
					}
				}
				else
					LOG("IOREG HD: no rigid-disk image mounted\n");
			}
			else if (op == 0xc0)
			{
				// c0 sub-7 = read device/drive status (the kernel's sacmd hardcodes subcode 7 for op
				// 0xc0). A real controller returns a status structure in the reply buffer; attach
				// marks the unit dead without it. Conservative reply: unit present + ready, no
				// fault/seek error (Panther drive-status semantics: Unit Ready|Present|Drive Ready).
				u8 const sub = bs.read_byte((ptr + 5) & 0xffffff);
				std::string db;
				for (u32 k = 0; k < 8; k++) db += util::string_format(" %02x", bs.read_byte((ptr + k) & 0xffffff));
				if (req)
				{
					for (u32 k = 1; k < 16; k++) bs.write_byte((req + k) & 0xffffff, 0x00);
					bs.write_byte(req & 0xffffff, 0xc1);   // Unit Ready | Present | Drive Ready
				}
				LOG("IOREG C0 STATUS unit %u sub %02x dev-blk:%s reply->%06x @%.4f\n", chunit, sub, db.c_str(), req, machine().time().as_double());
			}
			else if (op == 0xc2)
			{
				// c2 = SET configuration: the kernel downloads its geometry block (built @0x88a0 from
				// the compiled table). Log it for the per-unit geometry latch (bytes/sector etc.).
				std::string pb;
				if (req) for (u32 k = 0; k < 12; k++) pb += util::string_format(" %02x", bs.read_byte((req + k) & 0xffffff));
				LOG("IOREG C2 SETCONF unit %u params@%06x:%s @%.4f\n", chunit, req, pb.c_str(), machine().time().as_double());
			}
			else if (op == 0x03)
			{
				// SASI REQUEST SENSE: 4-byte reply, all zero = no error
				if (req) for (u32 k = 0; k < 4; k++) bs.write_byte((req + k) & 0xffffff, 0x00);
				LOG("IOREG SENSE unit %u reply->%06x @%.4f\n", chunit, req, machine().time().as_double());
			}
			else if (op == 0x00 || op == 0x01 || op == 0x0b)
			{
				// SASI TEST UNIT READY / REZERO / SEEK: no data, status-only (ACK below).
				// SEEK (0x0B) applies to the HD units too - the mkfs-era count-0 SEEKs were
				// long misread as "write-with-verify" data ops; the real data always arrives
				// in the accompanying per-buffer 0x0A writes (a phantom transfer here
				// overwrites freshly-written sectors with stale request-buffer bytes).
				LOG("IOREG %s unit %u @%.4f\n", op == 0 ? "TUR" : op == 1 ? "REZERO" : "SEEK", chunit, machine().time().as_double());
			}
			else if (op == 0x04 && chunit < 2)
			{
				// SASI FORMAT UNIT (the installer's full format, one op for the whole drive):
				// lay down the controller's fill pattern across the medium. 0x1A is the fill
				// byte the SINIX format's own per-track check expects to read back.
				if (m_hd[chunit]->exists())
				{
					u64 const size = m_hd[chunit]->img_length();
					std::vector<u8> const fill(64 * 1024, 0x1a);
					for (u64 off = 0; off < size; off += fill.size())
						m_hd[chunit]->img_write(off, fill.data(), u32(std::min<u64>(fill.size(), size - off)));
					LOG("IOREG FORMAT unit %u: filled %llu bytes with 1a @%.4f\n", chunit,
							(unsigned long long)size, machine().time().as_double());
				}
				else
					LOG("IOREG FORMAT: no rigid-disk image mounted\n");
			}
			else if (op == 0x87)
			{
				// extended status/identify (the ROM's hard-disk boot probes with this before
				// reading the label): reply like c0 sub-7 - unit present and ready
				if (req)
				{
					for (u32 k = 1; k < 16; k++) bs.write_byte((req + k) & 0xffffff, 0x00);
					bs.write_byte(req & 0xffffff, 0xc1);
				}
				LOG("IOREG 87 IDENT unit %u reply->%06x @%.4f\n", chunit, req, machine().time().as_double());
			}
			else if (op != 0x08)
				LOG("IOREG UNKNOWN OP %02x unit %u @%.4f\n", op, chunit, machine().time().as_double());
			if ((op == 0x08 || op == 0x0a) && chunit >= 2)
			{
				// READ (STEP 143 final decode, proven by the runtime iob scan: i_bn=16/i_ma=0xBD8C/i_cc=0x2000
				// at iob~0xB1D0): DATA BUFFER = I/O reg [2-3] (= iob->i_ma, the heap sblock/iob buffer);
				// device block: [1] = flags (0x40 const), [2-3] LE = ABSOLUTE position in current-size
				// sectors (0xa0 = 160 x 256B = blk 40 = partition base 24 + i_bn 16 - where the fs magic
				// 0x00090545 lives), [4-7] LE = count in sectors (0x20 = 8KB = i_cc). Uniform 8KB cylinders.
				if (!m_floppy_loaded) load_floppy();
				u32 const buf = req;   // I/O reg [2-3] = iob->i_ma
				u32 const psec = (u32(bs.read_byte((ptr + 2) & 0xffffff)) << 8) | bs.read_byte((ptr + 3) & 0xffffff);   // BE (68000-heritage field): 00 a0 = 160
				// dev-blk[4] = 8-bit SECTOR COUNT; [5] = the Multibus BYTES/TRANSACTION burst-size
				// parameter (0xC0), NOT part of the count -- same as the HD channel path (~line 924).
				// Bug: reading [4-7] as a 32-bit count pulled [5]=0xC0 in, so an 8-sector label read
				// (dev-blk[4]=0x08) became 0xC008 -> capped to 64 = a 16KB over-read that trampled the
				// adjacent host buffers (the root-fs read buffer).  The SINIX install then saw corrupted
				// data and rejected the data floppies during stage-2 ("Falsche Diskette eingelegt").
				u32 nsec;
				if (op == 0x0b)   // write-with-verify: [4-7] LE = count in BYTES -> 1KB sectors
				{
					nsec = u32(bs.read_byte((ptr + 4) & 0xffffff)) | (u32(bs.read_byte((ptr + 5) & 0xffffff)) << 8)
						 | (u32(bs.read_byte((ptr + 6) & 0xffffff)) << 16) | (u32(bs.read_byte((ptr + 7) & 0xffffff)) << 24);
					nsec = (nsec + 1023) / 1024;
				}
				else   // op 0x08 read / 0x0a write: count = dev-blk[4] sectors ([5]=0xC0 is the burst param)
					nsec = bs.read_byte((ptr + 4) & 0xffffff);
				if (nsec > 64) nsec = 64;   // sanity cap (16KB)
				for (u32 k = 0; k < nsec * 256; k++)
				{
					u32 const off = psec * 256 + k;
					u32 const cylno = off / 0x2000;
					u32 const rest = off % 0x2000;
					u32 const head = rest / 0x1000;
					u32 const loc = rest % 0x1000;
					u32 secid, bib;
					if (cylno == 0)
					{
						// cylinder 0 is FM: 16 x 128-byte sectors per head, logical order starting
						// at R7 (the VOL1 label sector); the kernel's uniform 256B-sector view maps
						// each of its sectors onto two consecutive FM sectors. Offsets past the FM
						// capacity (2KB per head) have no medium behind them.
						if (loc >= 0x800)
						{
							if (op != 0x0a)
								bs.write_byte((buf + k) & 0xffffff, 0);
							continue;
						}
						secid = ((6 + loc / 128) % 16) + 1;
						bib = loc % 128;
					}
					else
					{
						secid = loc / 256 + 1;
						bib = loc % 256;
					}
					auto const it = m_sectors.find((cylno << 16) | (head << 8) | secid);
					if (op == 0x0a)
					{
						// WRITE: host -> the in-memory floppy sector (the miniroot's writable root).
						if (it != m_sectors.end() && bib < it->second.size())
							it->second[bib] = bs.read_byte((buf + k) & 0xffffff);
						else
						{
							// TEMP (STRIP): DROPPED write byte = an INCOMPLETE write (Dave's free-list
							// corruption hypothesis: the fs's free-bitmap/metadata write loses bytes)
							static int drop = 0;
							if (drop++ < 30)
								logerror("FWDROP: byte @sec-off %u (cyl%u h%u s%u bib%u) NO SECTOR/size=%zu @%.4f\n",
										psec * 256 + k, cylno, head, secid, bib,
										it != m_sectors.end() ? it->second.size() : 0, machine().time().as_double());
						}
					}
					else
						bs.write_byte((buf + k) & 0xffffff, (it != m_sectors.end() && bib < it->second.size()) ? it->second[bib] : 0);
				}
				if (op != 0x0a)
					rd_check(chunit, psec, buf, nsec * 256);   // TEMP (STRIP): floppy read-delivery overlap
				if (op == 0x0a)
				{
					// TEMP (STRIP): SELF-VERIFY the write landed correctly (Dave's "wrong data" hypothesis)
					u32 bad = 0, badoff = 0;
					for (u32 k = 0; k < nsec * 256; k++)
					{
						u32 const off = psec * 256 + k, cylno = off / 0x2000, rest = off % 0x2000;
						u32 const head = rest / 0x1000, loc = rest % 0x1000, secid = loc / 256 + 1, bib = loc % 256;
						auto const it = m_sectors.find((cylno << 16) | (head << 8) | secid);
						u8 const want = bs.read_byte((buf + k) & 0xffffff);
						u8 const got = (it != m_sectors.end() && bib < it->second.size()) ? it->second[bib] : 0xAA;
						if (got != want) { if (!bad) badoff = k; bad++; }
					}
					if (bad)
					{
						static int vf = 0;
						if (vf++ < 30)
							logerror("FWVERIFY-FAIL: sec=%u x%u  %u/%u bytes WRONG (first @+%u) = INCOMPLETE/WRONG WRITE @%.4f\n",
									psec, nsec, bad, nsec * 256, badoff, machine().time().as_double());
					}
				}
				std::string dd;
				for (u32 k = 0; k < 12; k++) dd += util::string_format("%02x ", bs.read_byte((buf + k) & 0xffffff));
				LOG("IOREG %s sec=%u (byte %06x) x%usec %s host %06x  first12: %s@%.4f\n", op == 0x0a ? "FWRITE" : "READ",
						psec, psec * 256, nsec, op == 0x0a ? "<-" : "->", buf, dd.c_str(), machine().time().as_double());
			}
		}
		m_ioreg[1] = 0;        // status = OK
		m_ioreg_done = true;   // ACK
		logerror("TEMPO DONE cid=%u t=%.6f\n", m_temp_cid, machine().time().as_double());   // TEMPORAL PROBE (STRIP)
		if (data & 1)          // cmd bit0 = interrupt-driven: count the outstanding completion and arm
		{                      // the deferred INT2 (MULTIBUS INT2 -> CPUAP ICU IR3 = the kernel sa handler)
			if (m_ioreg_pending > 0)
				LOG("IOREG INT-STACK: completion while %d already pending @%.4f\n",
						m_ioreg_pending, machine().time().as_double());
			m_ioreg_pending++;
			m_ioreg_int->adjust(attotime::from_usec(500));   // deferred so it can't beat the driver into its wait
		}
	}
}

// Disk read/write channel (74LS1801/1802 ENDEC + drives), logged for RE.
// cmd 0x95 READ delivery (STEP 131-133 request-honoring probe; diagnostic, not the final firmware-driven
// model): serve EXACTLY what the IOPB asks, from what the media contains.  Evidence-based model:
//  - geometry = the unit's last INIT/UIB (UIB[0]=heads, [1]=spt, [2-3]=bytes/sector LE, [4]=FM start sector);
//  - the IOPB count is in 512-BYTE LOGICAL BLOCKS (HDR1 extent=blk 8 = cyl1; fe44a9 byte count 0x400;
//    drive struct stores 8 blks/trk + 512): one block = 4x128 (FM) or 2x256 (MFM) physical sectors;
//  - POSITION IS PHYSICAL (track, sector-index): the label read (8 blks = 32 FM sectors = exactly 2 tracks)
//    leaves the head at track 2 = cyl1/hd0; the re-INIT switches density in place; the boot-image read
//    continues there with NO seek - which is why the command stream has none;
//  - within an FM track the sector order starts at UIB[4] (=7 -> the ANSI VOL1 sector delivered first);
//    MFM tracks deliver in plain ID order (UIB[4]=13's MFM meaning UNRESOLVED - not modeled);
//  - called at the DOORBELL (synchronous with the CPUAP's GO write) so the data is in the host buffer
//    before any completion status the CPUAP might see.
// Doorbell-HLE delivery (runs only when !m_fw_driven - the LLE boot recipe never enters here for
// the floppy leg, whose carry runs on the C000 host-address counter).  The m_iopb_buffer host
// destination below is the HLE shortcut for the HD/tape legs pending their drive-side LLE
// (ESDI Phase C): the faithful path presets the C000 counter from the IOPB pair ($36E6/$128a)
// and DMAs on it.  Retire these uses when those legs go LLE.
void multibus_storager_device::read95_deliver(u32 dst)
{
	address_space &cs = m_cpu->space(AS_PROGRAM);
	address_space &bs = m_bus->space(AS_PROGRAM);
	if (!m_floppy_loaded) load_floppy();
	u32 const unit = cs.read_byte((dst + 4) & 0xffff) & 3;
	u32 const count = (u32(cs.read_byte((dst + 0xa) & 0xffff)) << 8) | cs.read_byte((dst + 0xb) & 0xffff);
	if (unit == 2 && count && count <= 64 && m_unit_secsize[2])
	{
		u32 const heads = m_unit_heads[2], spt = m_unit_spt[2], ssz = m_unit_secsize[2];
		u32 const sec_per_blk = 512 / ssz;
		u32 trk = m_unit_trk[2], sidx = m_unit_sidx[2];
		// IOPB options bit0: 0 = SEQUENTIAL from the current position ("Seq/Disable-Address-Bump");
		// 1 = ADDRESSED: bytes[6-9] (BE) = sector address in current-size sectors, relative to the
		// position at the last INIT (the ANSI flow re-INITs with the head at the file base, so file-
		// relative addressing lands correctly; evidence: the 53-blk read addr=4 = file byte 0x400 =
		// the ZMAGIC text offset).  PROVISIONAL: the base rule needs firmware confirmation.
		if (cs.read_byte((dst + 1) & 0xffff) & 1)
		{
			u32 const addr = (u32(cs.read_byte((dst + 6) & 0xffff)) << 24) | (u32(cs.read_byte((dst + 7) & 0xffff)) << 16)
						   | (u32(cs.read_byte((dst + 8) & 0xffff)) << 8)  |  u32(cs.read_byte((dst + 9) & 0xffff));
			u32 const abs_s = m_unit_btrk[2] * spt + m_unit_bsidx[2] + addr;
			trk = abs_s / spt;
			sidx = abs_s % spt;
		}
		u32 const otrk = trk, osidx = sidx;
		u32 const dstb = m_iopb_buffer & 0xffffff;
		u32 off = 0;
		for (u32 n = 0; n < count * sec_per_blk; n++)
		{
			u32 const cylno = trk / heads, head = trk % heads;
			u32 const sec0 = (ssz == 128 && m_unit_sec0[2] >= 1 && m_unit_sec0[2] <= spt) ? m_unit_sec0[2] : 1;
			u32 const secid = ((sec0 - 1 + sidx) % spt) + 1;
			auto const it = m_sectors.find((cylno << 16) | (head << 8) | secid);
			for (u32 k = 0; k < ssz; k++)
				bs.write_byte((dstb + off + k) & 0xffffff, (it != m_sectors.end() && k < it->second.size()) ? it->second[k] : 0);
			off += ssz;
			if (++sidx >= spt) { sidx = 0; trk++; }
		}
		std::string dd;
		for (u32 k = 0; k < 12; k++) dd += util::string_format("%02x ", bs.read_byte((dstb + k) & 0xffffff));
		LOG("READ95 unit2: trk%u.%u -> trk%u.%u (%u blk x512, %uB sectors) -> %06x  first12: %s@%.4f\n",
			otrk, osidx, trk, sidx, count, ssz, dstb, dd.c_str(), machine().time().as_double());
		m_unit_trk[2] = trk; m_unit_sidx[2] = sidx;
	}
	else if (unit < 2)
	{
		// hard-disk unit: serve from the MC1325 image (the same source as the kernel-era ioreg
		// path). The IOPB count is in 512-byte blocks; the position (addressed mode, options
		// bit0) is a sector address in the unit's current sector size (the ROM boot flow INITs
		// the HD at 1KB/sector; default to 1KB when no INIT has been seen).
		u32 const ssz = m_unit_secsize[unit] ? m_unit_secsize[unit] : 1024;
		u64 pos = 0;
		if (cs.read_byte((dst + 1) & 0xffff) & 1)
		{
			u32 const addr = (u32(cs.read_byte((dst + 6) & 0xffff)) << 24) | (u32(cs.read_byte((dst + 7) & 0xffff)) << 16)
						   | (u32(cs.read_byte((dst + 8) & 0xffff)) << 8)  |  u32(cs.read_byte((dst + 9) & 0xffff));
			pos = u64(addr) * ssz;
		}
		if (m_hd[unit]->exists() && count && count <= 64)
		{
			std::vector<u8> buf(size_t(count) * 512, 0);
			m_hd[unit]->img_read(pos, buf.data(), u32(buf.size()));
			u32 const dstb = m_iopb_buffer & 0xffffff;
			for (u32 k = 0; k < u32(buf.size()); k++)
				bs.write_byte((dstb + k) & 0xffffff, buf[k]);
			std::string dd;
			for (u32 k = 0; k < 12; k++) dd += util::string_format("%02x ", buf[k]);
			LOG("READ95 unit%u HD: pos=%llx (%u blk x512, ssz=%u) -> %06x  first12: %s@%.4f\n",
					unit, (unsigned long long)pos, count, ssz, dstb, dd.c_str(), machine().time().as_double());
		}
		else
			LOG("READ95 unit%u HD: cannot serve (img=%d count=%u) @%.4f\n", unit, m_hd[unit]->exists() ? 1 : 0, count, machine().time().as_double());
	}
	else
		LOG("READ95 unit%u: no media modeled (tape) - not delivered @%.4f\n", unit, machine().time().as_double());
}

// Adapter scope to reach floppy_image_format_t's protected static bitstream helper (never instantiated).
struct storager_trkdec : floppy_image_format_t
{
	using floppy_image_format_t::generate_bitstream_from_track;
};

// Decode the ID fields of the mounted image's track at (cyl, side), in track order, from the real
// cell data (Phase B: the gate-array/ENDEC model's view of the media; sector-granular per the fw's
// consumption - the fw itself verifies C/H and reads its angular position from the returned R).
multibus_storager_device::dc_track const &multibus_storager_device::decode_track_ids(floppy_image_device *fdd, int cyl, int side)
{
	u32 const key = (cyl << 4) | side;
	auto it = m_dc_ids.find(key);
	if (it != m_dc_ids.end())
		return it->second;

	dc_track result;
	floppy_image const *const img = fdd->get_image();
	auto decode_bytes = [](std::vector<bool> const &bits, size_t pos, unsigned count)
	{
		std::vector<u8> out;
		for (unsigned k = 0; k < count && pos + 16 <= bits.size(); k++, pos += 16)
		{
			u8 b = 0;
			for (unsigned j = 0; j < 8; j++)
				b = (b << 1) | (bits[pos + j * 2 + 1] ? 1 : 0);
			out.push_back(b);
		}
		return out;
	};
	if (img)
	{
		// MFM (0x4489 sync runs): collect ID (FE) and DATA (FB/F8) fields in track order.
		// cont.248: decode at BOTH cell rates and KEEP THE BEST harvest - the wrong rate
		// resamples into occasional phantom 4489 runs with garbage IDs (cyl 1's 2000ns
		// MFM track decoded at 1667 gave 16 junk "sectors" R=C7/EF/FF and READ2 fed the
		// fw garbage), and the old first-nonempty-wins order poisoned the stream.
		dc_track best{};
		for (int cell : { 1667, 2000 })
		{
			dc_track cand{};
			auto const bits = storager_trkdec::generate_bitstream_from_track(cyl, side, cell, *img);
			// cont.248b: the old sync-counting decode NEVER worked (the count reset one
			// bit after every 4489 match, and the byte decode started 15 bits back on
			// the A1 itself) - MFM was dead code until READ2 first exercised it. One
			// 4489 match is enough: the address-mark byte follows the LAST A1 directly.
			u32 sr = 0;
			dc_sector pend{}; bool have_id = false;
			for (size_t i = 0; i + 16 < bits.size(); i++)
			{
				sr = ((sr << 1) | (bits[i] ? 1 : 0)) & 0xffff;
				if (sr != 0x4489)
					continue;
				auto const b = decode_bytes(bits, i + 1, 5);
				if (b.size() >= 5 && b[0] == 0xfe)
				{
					pend = dc_sector{ b[1], b[2], b[3], b[4], {} };
					have_id = true;
				}
				else if (!b.empty() && (b[0] == 0xfb || b[0] == 0xf8) && have_id)
				{
					unsigned const len = 128u << (pend.n & 3);
					pend.data = decode_bytes(bits, i + 1 + 16, len);
					cand.secs.push_back(std::move(pend));
					have_id = false;
				}
			}
			logerror("DCID cyl%d side%d cell=%d -> %u MFM secs\n", cyl, side, cell, unsigned(cand.secs.size()));
			if (cand.secs.size() > best.secs.size()) best = std::move(cand);
		}
		result = std::move(best);
		if (result.secs.empty())
		{
			// FM: IDAM = 0xF57E, DAM = 0xF56F (FB) / 0xF56A (F8)
			for (int cell : { 3333, 4000 })
			{
				auto const bits = storager_trkdec::generate_bitstream_from_track(cyl, side, cell, *img);
				u32 sr = 0;
				dc_sector pend{}; bool have_id = false;
				for (size_t i = 0; i < bits.size(); i++)
				{
					sr = ((sr << 1) | (bits[i] ? 1 : 0)) & 0xffff;
					if (sr == 0xf57e)
					{
						auto const b = decode_bytes(bits, i + 1, 4);
						if (b.size() >= 4) { pend = dc_sector{ b[0], b[1], b[2], b[3], {} }; have_id = true; }
					}
					else if ((sr == 0xf56f || sr == 0xf56a) && have_id)
					{
						unsigned const len = 128u << (pend.n & 3);
						pend.data = decode_bytes(bits, i + 1, len);
						result.secs.push_back(std::move(pend));
						have_id = false;
					}
				}
				if (!result.secs.empty()) { result.fm = true; break; }
			}
		}
	}
	LOG("TRKDEC cyl %d side %d: %s, %u sectors\n", cyl, side, result.fm ? "FM" : "MFM", unsigned(result.secs.size()));
	return m_dc_ids.emplace(key, std::move(result)).first->second;
}

// Build the current track's raw ENDEC byte stream (what the 74LS1802 SERDES clocks out under the head),
// from the mounted image's decoded fields. Per sector: [A1 A1 A1] FE C H R N crc crc  [A1 A1 A1] FB
// <data...> crc crc. The firmware reads E000 to pull these bytes, syncs on the A1 runs, matches the
// target sector's ID, and takes its data. No protocol here - just the bytes the head would see.

// ID-capture completion: deposit the next ID field (in the format the fw's parser @0x9884 consumes:
// MFM = A1 A1 A1 FE C H R N + 2 crc filler, FM = FE C H R N + crc) at the D800-programmed local
// destination, then raise the channel-completion IRQ4 - the firmware's own ISR/continuation posts
// the result; the model never writes status words.
TIMER_CALLBACK_MEMBER(multibus_storager_device::idcap_tick)
{
	floppy_image_device *const fdd = m_floppy[0]->get_device();
	if (!fdd) return;
	address_space &cs = m_cpu->space(AS_PROGRAM);
	int const cyl = fdd->get_cyl();
	if (storager_getenv("STORAGER_NOBYPASS") && m_iopb_cmd == 0x95)
	{
		// LLE read (fw-driven): the fw's read step just programmed D800 = $7434 = $7dac>>1, and
		// m_idcap_dst = D800<<1 = the parse buffer $7a66 reads.  Stage ONE full contiguous sector -
		// ID + data-AM + data + CRC, in the ENDEC byte format the $9884 ID-matcher and the inline
		// $9984 data step consume (FM: FE.. / FB..; MFM: A1 A1 A1 FE.. / A1 A1 A1 FB..) - then raise
		// the IRQ6 pump.  $9884 matches the ID off $7dac, the step machine reads the data behind it;
		// the fw walks successive sectors by re-arming (E802 bit15), so deliver in physical rotation.
		int const side = cs.read_word(0x7436) & 1;   // the fw's target head
		auto const &trk = decode_track_ids(fdd, cyl, side);
		if (trk.secs.empty())
			return;   // no marks: let the fw's own $201C timeout judge it (faithful)
		auto const &sec = trk.secs[m_dc_seq++ % trk.secs.size()];
		u32 a = m_idcap_dst;
		// sync run: the $89f2 lock-on ISR ORs bytes 0-2 and requires $A1 even for FM (as $9884 - the
		// ENDEC presents the sync marks as A1 A1 A1 in the byte stream regardless of density)
		cs.write_byte(a++, 0xa1); cs.write_byte(a++, 0xa1); cs.write_byte(a++, 0xa1);
		cs.write_byte(a++, 0xfe);   // ID address mark
		cs.write_byte(a++, 0xff);   // ID-valid marker: the $89f2 lock-on ISR requires [A1 A1 A1][FE][FF] at $7dac
		cs.write_byte(a++, sec.c); cs.write_byte(a++, sec.h); cs.write_byte(a++, sec.r); cs.write_byte(a++, sec.n);
		cs.write_byte(a++, 0x00); cs.write_byte(a++, 0x00);   // ID CRC (gate array verified)
		if (!trk.fm) { cs.write_byte(a++, 0xa1); cs.write_byte(a++, 0xa1); cs.write_byte(a++, 0xa1); }
		cs.write_byte(a++, 0xfb);   // data address mark
		for (u8 v : sec.data) cs.write_byte(a++, v);
		cs.write_byte(a++, 0x00); cs.write_byte(a++, 0x00);   // data CRC
		LOG("IDCAP+DATA (LLE) cyl%d/s%d id={%02x %02x %02x %02x} data=%uB -> %04x, IRQ6 @%.4f\n",
				cyl, side, sec.c, sec.h, sec.r, sec.n, unsigned(sec.data.size()), m_idcap_dst, machine().time().as_double());
		{ static int _q6 = 0; double const _t6 = machine().time().as_double();
				if (storager_getenv("STORAGER_PHASELOG") && _q6++ < 400)
					logerror("RAISE6-%s @%.6f\n", "IDCAP", _t6); }
		m_cpu->set_input_line(M68K_IRQ_6, HOLD_LINE);
		return;
	}
	int const side = 0;   // B0 open item: the dumb-path side-select line (the fw's 0x202A error will name it)
	auto const &trk = decode_track_ids(fdd, cyl, side);
	if (trk.secs.empty())
		return;   // no marks: let the fw's 80k poll hit its own 0x201C timeout - faithful
	auto const &sec = trk.secs[angular_sector(trk.secs.size())];
	u32 a = m_idcap_dst;
	if (!trk.fm)
		for (u8 v : { 0xa1, 0xa1, 0xa1 }) cs.write_byte(a++, v);
	cs.write_byte(a++, 0xfe);
	for (u8 v : { sec.c, sec.h, sec.r, sec.n }) cs.write_byte(a++, v);
	cs.write_byte(a++, 0x00); cs.write_byte(a++, 0x00);   // CRC filler (the gate array checked it)
	LOG("IDCAP deliver cyl%d/s%d id={%02x %02x %02x %02x} -> %04x, IRQ4\n", cyl, side, sec.c, sec.h, sec.r, sec.n, m_idcap_dst);
	{ if (storager_getenv("STORAGER_NOBYPASS")) logerror("IRQ4fire L%d pc=%06x @%.4f\n", __LINE__, m_cpu->pc(), machine().time().as_double()); m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE); }
}

// The armed read-channel data operation: deliver the next sector under the head into the D000
// local staging destination, post channel status, raise the completion IRQ4. (The fw's IRQ4 ISR
// armed the channel by writing the $79CA/CC mode pair into the E000 file - see ch_w.)
TIMER_CALLBACK_MEMBER(multibus_storager_device::dataop_tick)
{
	if (m_bare_irq4)
	{
		m_bare_irq4 = false;
		LOG("DATAOP bare IRQ4 @%.4f\n", machine().time().as_double());
		{ if (storager_getenv("STORAGER_NOBYPASS")) logerror("IRQ4fire L%d pc=%06x @%.4f\n", __LINE__, m_cpu->pc(), machine().time().as_double()); m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE); }
		return;
	}
	floppy_image_device *const fdd = m_floppy[0]->get_device();
	if (!fdd) return;
	int const cyl = fdd->get_cyl();
	address_space &cs0 = m_cpu->space(AS_PROGRAM);
	// side-select correlation (B0's last unknown): candidates = E802 bits 4/5 vs the fw's target
	// head cell $7436. Until the hardware line is pinned, follow the fw's target (logged for the
	// correlation; the E802-bit hypothesis gets confirmed or refuted by the log).
	u16 const e802v = m_ch[(0xe802 - 0xe000) / 2];
	u16 const want_h = cs0.read_word(0x7436);
	u16 const want_c = cs0.read_word(0x7438);
	int const side = (want_h & 1);
	logerror("DATAOP tgt C=%u H=%u | e802 b4=%d b5=%d | cyl=%d side=%d @%.4f\n",
			want_c, want_h, BIT(e802v, 4), BIT(e802v, 5), cyl, side, machine().time().as_double());
	auto const &trk = decode_track_ids(fdd, cyl, side);
	if (trk.secs.empty())
		return;   // no data: the fw's own op timeout judges it
	// each capture waits for the NEXT address mark - successive attempts see successive sectors
	// in physical track order (the rotation; the fw's verify hunts across them)
	auto const &sec = trk.secs[m_dc_seq++ % trk.secs.size()];
	// This DATAOP is the HLE read delivery (reads the m_sectors direct-IMD map) and stages the ID/sector to
	// dst=m_d000<<1=$71f0 (a STALE CCB pointer - m_d800, the real parse buffer $7dac, is 0 until $5FC0 runs).
	// Under NOBYPASS (LLE) it must NOT run: it stamps 0xfe into $71f0[0], clobbering the staged 0x95 before the
	// read dispatches (the 2nd model clobber after the UIB fix).  Retire it per the LLE mandate; the fw drives
	// its own read->parse->DONE tail.  (Gating removes the HLE completion too -> exposes the read-dispatch <->
	// completion <-> retry-flood cycle: 0x95 is staged but the flood's IOPB fetches overwrite it before $d06.)
	if (storager_getenv("STORAGER_NOBYPASS")) return;
	address_space &cs = m_cpu->space(AS_PROGRAM);
	u32 const dst = u32(m_d000) << 1;
	// TEMP LOG (STRIP): the request block's expected-buffer pointer + surroundings before delivery
	{
		u16 const wrk = cs.read_word(0x71c0);
		std::string ctx;
		for (int k = 0x18; k <= 0x28; k += 2) ctx += util::string_format(" +%02x=%04x", k, cs.read_word((wrk + k) & 0xffff));
		logerror("DATAOP pre: workblk=%04x%s  D000<<1=%04x D800=%04x @%.4f\n", wrk, ctx.c_str(), dst, m_d800, machine().time().as_double());
	}
	// the op's transfer length comes from the fw ($79D4, in words; @0x1CE0: 0x18 or 0xC minus 3) -
	// this phase is header/verify-sized, NOT a full sector; a full-sector write here would clobber
	// the live request block (D000 dest == the local IOPB at $71F0!)
	u16 const opwords = cs.read_word(0x79d4);
	size_t oplen = size_t(opwords) * 2;
	// dispatch the delivered content by the channel op (E800 bits 5-7, from the $63E table):
	//   0x40/0x20 families = ID capture/verify -> the decoded mark stream (A1 A1 A1 FE C H R N / FM)
	//   0x00 family        = DATA transfer     -> the target sector's data (fw's accepted R @ $7428)
	u16 const e800 = m_ch[(0xe800 - 0xe000) / 2];
	unsigned const opfam = (e800 >> 5) & 7;
	// OBSERVE (STRIP): what is the fw searching for vs what the gate array supplies this pass?
	logerror("GAPASS seq=%u opfam=%u supply{C%02x H%02x R%02x N%02x} target{C=%u H=%u R=%02x} e800=%04x @%.4f\n",
			m_dc_seq, opfam, sec.c, sec.h, sec.r, sec.n, cs.read_word(0x7438), cs.read_word(0x7436), cs.read_word(0x7428)&0xff, e800, machine().time().as_double());
	std::vector<u8> payload;
	if (opfam == 0)
	{
		u16 const want_r = cs.read_word(0x7428) & 0xff;
		dc_sector const *pick = &sec;
		for (auto const &c : trk.secs)
			if (c.r == want_r) { pick = &c; break; }
		payload = pick->data;
		oplen = payload.size();   // the data op moves the full sector
		LOG("DATAOP data-phase R=%02x (want %02x)\n", pick->r, want_r);
		// THE HOST DMA LEG: the gate array bus-masters the requested transfer into the CPUAP buffer.
		// Same contract as the retired read95_deliver (count = local IOPB [0xA..B] in 512-byte blocks;
		// [1] bit0 = addressed mode, address [6..9] relative to the INIT-latched base), but the sector
		// bytes come from the mounted image's decoded bitstream and the delivery runs at the firmware's
		// own data phase.  LLE (narrowed cut): this is the DATA SHIM - suppress under NOBYPASS so the fw
		// drives its own SRAM->host DMA (D800/$7dac).  The IRQ4 completion below still fires (it's the
		// gate-array signal the fw consumes; only the C++ data-write is the shim).
		if (m_unit_secsize[2] && !storager_getenv("STORAGER_NOBYPASS"))
		{
			if (storager_getenv("STORAGER_PHASELOG")) logerror("SHIM-DATAOP (data-write) cmd=%02x 71b2=%04x @%.5f\n", m_iopb_cmd, m_cpu->space(AS_PROGRAM).read_word(0x71b2), machine().time().as_double());
			address_space &bs = m_bus->space(AS_PROGRAM);
			u32 const heads = m_unit_heads[2], spt = m_unit_spt[2], ssz = m_unit_secsize[2];
			u32 const count = (u32(cs.read_byte(0x71fa)) << 8) | cs.read_byte(0x71fb);
			u32 const sec_per_blk = 512 / ssz;
			u32 htrk = m_unit_trk[2], hsidx = m_unit_sidx[2];
			if (cs.read_byte(0x71f1) & 1)
			{
				u32 const addr = (u32(cs.read_byte(0x71f6)) << 24) | (u32(cs.read_byte(0x71f7)) << 16)
							   | (u32(cs.read_byte(0x71f8)) << 8)  |  cs.read_byte(0x71f9);
				u32 const abs_s = m_unit_btrk[2] * spt + m_unit_bsidx[2] + addr;
				htrk = abs_s / spt;
				hsidx = abs_s % spt;
			}
			u32 const dstb = m_iopb_buffer & 0xffffff;
			u32 off = 0;
			for (u32 n = 0; n < count * sec_per_blk && count <= 64; n++)
			{
				u32 const cylno = htrk / heads, head = htrk % heads;
				u32 const sec0 = (ssz == 128 && m_unit_sec0[2] >= 1 && m_unit_sec0[2] <= spt) ? m_unit_sec0[2] : 1;
				u32 const secid = ((sec0 - 1 + hsidx) % spt) + 1;
				auto const &htrkdec = decode_track_ids(fdd, cylno, head);
				dc_sector const *hs = nullptr;
				for (auto const &c : htrkdec.secs)
					if (c.r == secid) { hs = &c; break; }
				for (u32 k = 0; k < ssz; k++)
					bs.write_byte((dstb + off + k) & 0xffffff, (hs && k < hs->data.size()) ? hs->data[k] : 0);
				off += ssz;
				if (++hsidx >= spt) { hsidx = 0; htrk++; }
			}
			std::string dd;
			for (u32 k = 0; k < 12; k++) dd += util::string_format("%02x ", bs.read_byte((dstb + k) & 0xffffff));
			LOG("DATAOP host-DMA: trk%u.%u -> trk%u.%u (%u blk, %uB sec) -> host %06x first12: %s@%.4f\n",
					m_unit_trk[2], m_unit_sidx[2], htrk, hsidx, count, ssz, dstb, dd.c_str(), machine().time().as_double());
			m_unit_trk[2] = htrk; m_unit_sidx[2] = hsidx;
		}
	}
	else
	{
		if (!trk.fm)
			for (u8 v : { 0xa1, 0xa1, 0xa1 }) payload.push_back(v);
		payload.push_back(0xfe);
		for (u8 v : { sec.c, sec.h, sec.r, sec.n }) payload.push_back(v);
		payload.push_back(0x00); payload.push_back(0x00);   // CRC
		payload.resize(oplen, 0x00);
	}
	for (size_t k = 0; k < oplen; k++)
		cs.write_byte((dst + k) & 0xffff, payload[k]);
	// post the channel status into the CCB low nibble (the verdict @0x1D86 reads ($7FE8) & 0xF)
	cs.write_byte(0x7fe9, cs.read_byte(0x7fe9) & 0xf0);   // low nibble 0 = no error (word LE/BE? post both clean)
	logerror("DATAOP opwords=%u oplen=%u ccb=%04x @%.4f\n", opwords, unsigned(oplen), cs.read_word(0x7fe8), machine().time().as_double());
	// TEMP LOG (STRIP): the live watch records at delivery time - which cell must the gate array update?
	{
		static int n = 0;
		if (n++ < 6)
			for (u16 a = 0x72d6; a < 0x72ec; a += 2)
			{
				u16 const rec = cs.read_word(a);
				if (!rec) continue;
				u16 const wptr = cs.read_word(rec + 2);
				logerror("WATCHREC @%04x -> %04x: f=%04x watch=%04x(*=%04x) cmp=%04x h=%04x\n",
						a, rec, cs.read_word(rec), wptr, wptr ? cs.read_word(wptr) : 0, cs.read_word(rec + 4), cs.read_word(rec + 6));
			}
	}
	// post the read-channel status: E01E bit4 = read-OK/no-error, as a held LEVEL (see ch_r)
	m_ch_op_ok = true;
	// the disk-channel op completion is LEVEL 6 (autovector trampoline @0x26AE jumps *($7304) =
	// soft-vector slot 0x18 = the per-command pump the fw registers via $328E) - NOT the level-4
	// descriptor-engine interrupt
	LOG("DATAOP deliver cyl%d/s%d C%02x H%02x R%02x N%02x len=%u -> %04x, E01E|=10, IRQ4 @%.4f\n",
			cyl, side, sec.c, sec.h, sec.r, sec.n, unsigned(sec.data.size()), dst, machine().time().as_double());
	// NOTE (LLE read-channel refactor, 2026-07-11): this DATAOP path stages to D000<<1 ($71f0) and fires
	// IRQ4 - but the fw-driven read parser ($9884) reads D800<<1 ($7dac) and is released by the IRQ6 pump,
	// not IRQ4.  So this whole path is the wrong buffer + wrong IRQ for the LLE read.  The correct model:
	// on the fw's E800 step-toggle in $369a, stage the next FULL contiguous sector (ID+gap+data, per
	// inline.  Retire this DATAOP delivery for the fw-driven read once that lands.  (HLE path unchanged.)
	{ if (storager_getenv("STORAGER_NOBYPASS")) logerror("IRQ4fire L%d pc=%06x @%.4f\n", __LINE__, m_cpu->pc(), machine().time().as_double()); m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE); }
}

u16 multibus_storager_device::ch_r(offs_t offset, u16 mem_mask)
{
	u32 const a = 0xe000 + offset * 2;
	u16 d = m_ch[offset];
	if (a == 0xf000)   // status: bit12 (0x1000) = DMA transfer address reached the terminal
	{
		u32 const term = 0x4000 | m_dma_term | m_term_bit0;
		bool const reached = m_dma_active && (m_last_bw == term);
		// TEMP (STRIP): Dave's DMA-boundary probe - does bit12 (DMA terminal) ever latch for the init read?
		// task#5: DATA-PHASE capture (t>6.52, past the setup/recal). Does term move into the 0x79xx DMA range
		// (fw programs C800 terminal), or stay 0x4000? + what pc polls F000 at the 699 timeout.
		{ double const _ft = machine().time().as_double();
			static int _n = 0;
			if (m_fw_driven && m_iopb_cmd == 0x95
					&& ((_ft > 3.14 && _ft < 3.30) || (_ft > 8.19 && _ft < 8.35))
					&& _n++ < 60)
				logerror("F000TERM-DATA dma=%d last_bw=%06x term=%06x dterm=%04x reached=%d pc=%06x @%.5f\n",
					m_dma_active, m_last_bw, term, m_dma_term, reached, m_cpu->pc(), _ft); }
		d = (d & ~0x1000) | (reached ? 0x1000 : 0);        // bit12 = DMA address-match
		d = (d & ~0x0800) | (m_timer_out ? 0x0800 : 0);    // bit11 = 8253 timer OUT
		// drive status (floppy 0) - EMPIRICAL bit map (iterating): bit7 = ready/door-closed (5.25" has
		// no 8" READY -> media present), bit13 = track0, bit10 = write-protect.
		if (!m_floppy_loaded) load_floppy();   // ensure the motor is spinning early so INDEX pulses
		floppy_image_device *const fdd = m_floppy[0]->get_device();
		bool const present = fdd && fdd->exists();
		bool const trk0 = present && !fdd->trk00_r();   // trk00_r active-low (0 at track 0)
		bool const wprot = present && !fdd->wpt_r();     // wpt_r active-low (0 = protected)
		// F000 bit map — VERIFIED against the v260 firmware (NOT the PCMX2-INTEGRATION.md guess, which
		// mis-placed drive-ready at b8/b9): the drive-scanner @fw 0x767c does btst #5 (ready, must be
		// SET) with bits 3/0/4 clear; the READ handler @fw 0x5de4 does btst #$d (bit13 = position/track0).
		// So drive-ready = bit5 (empirical, firmware-confirmed), track0 = bit13, NOT the doc's b8/b9.
		d = (d & ~0x0080) | (present ? 0x0080 : 0);   // bit7  = drive ready / door-closed
		d = (d & ~0x0020) | (present ? 0x0020 : 0);   // bit5  = drive READY (VERIFIED: fw 0x767c btst #5)
		d = (d & ~0x2000) | (trk0 ? 0 : 0x2000);      // bit13 = track 0, ACTIVE-LOW (VERIFIED: fw 0x5de4 btst #$d)
		d = (d & ~0x0400) | (wprot ? 0x0400 : 0);     // bit10 = write-protect
		if (m_fw_driven)
			d = (d & ~0x0002) | (m_settle_out ? 0 : 0x0002);   // bit1 = settle busy (empirical; seek-done polarity UNVERIFIED - keep known-good)
		else
			d = (d & ~0x0002) | ((machine().time() < m_seek_deadline) ? 0x0002 : 0);  // bit1 = seek-active (time-windowed shim)
		// cont.150 (KEEPER, Dave's adjudication): [$F000] bit4 doubles as the gate array's
		// WANT-READY / ADVANCE status - the hardware half of the ONE condition checked at
		// both gates ($6bc2's re-dispatch: f000.4 AND desc[$18]; $17fe's done-stamp: D1.4
		// AND desc[$18]). Read1's inline launch-stamp never needs it; read2's deferred
		// passes poll it and found only the index pulse. m_want_ready latches at the carry
		// (the same event that stakes the f0) and holds until the command's DONE post -
		// the fw then re-inits ($94a0), re-arms ($6ed2), and rides its own path to $17fe
		// with both bits set. One status bit; the machine completes itself.
		d = (d & ~0x0010) | (((present && fdd->idx_r()) || m_want_ready) ? 0x0010 : 0);
		// cont.232 (STORAGER_PAIR): bit6 = chunk MID-BLOCK - the odd sector deposited, the
		// 256B block's second half pending. The fw's $7F2C per-cycle gate: SET -> wait for
		// the pair; CLEAR -> proceed to the append/kick. The per-block cadence's true signal.
		if (storager_getenv("STORAGER_PAIR"))
			d = (d & ~0x0040) | (m_blk_half ? 0x0040 : 0);
		// cont.161 (STRIP): op-42's REAL floppy gate = (F000 & [$7a0a]?$220:$120) == $20
		// (fw $6bf2-$6c16; bit5 SET + bit8/bit9 CLEAR), NOT bit4. Frame every op-42 poll:
		// full d, mask select, UIB flags, node[$18] - which bit fails, read1 vs read2.
		{ u32 const opc = m_cpu->pc();
			// cont.162 (STRIP): one-shot UIB $6e60 timing bytes ($0e-$19) at the read era.
			{ static bool once = false;
				if (!once && machine().time().as_double() > 6.5)
				{ once = true; address_space &ds = m_cpu->space(AS_PROGRAM);
					logerror("UIBTIME 6e60[0e..19]= %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x @%.5f\n",
						ds.read_byte(0x6e6e), ds.read_byte(0x6e6f), ds.read_byte(0x6e70), ds.read_byte(0x6e71),
						ds.read_byte(0x6e72), ds.read_byte(0x6e73), ds.read_byte(0x6e74), ds.read_byte(0x6e75),
						ds.read_byte(0x6e76), ds.read_byte(0x6e77), ds.read_byte(0x6e78), ds.read_byte(0x6e79),
						machine().time().as_double()); } }
			if (opc >= 0x6bf0 && opc <= 0x6c1a && machine().time().as_double() > 6.3)
			{ static int _o = 0; if (_o++ < 100) { address_space &ds = m_cpu->space(AS_PROGRAM);
				u16 const uib = ds.read_word(0x799a), node = ds.read_word(0x71bc);
				logerror("OP42POLL d=%04x 7a0a=%04x flags=%02x node18=%04x pc=%06x @%.6f\n",
					d, ds.read_word(0x7a0a), ds.read_byte(uib + 0x12), ds.read_word(node + 0x18), opc, machine().time().as_double()); } } }
		if (storager_getenv("STORAGER_NOBYPASS") && machine().time().as_double() > 6.0)   // TEMP (STRIP): F000 track0 (bit13) during the restore seek?
		{ static int _fn = 0; if (_fn++ < 100) logerror("F000rd=%04x bit13=%d cyl=%d trk00=%d m_unit_trk0=%d pc=%06x @%.5f\n",
			d, (d >> 13) & 1, present ? fdd->get_cyl() : -1, present ? fdd->trk00_r() : -1, m_unit_trk[0], m_cpu->pc(), machine().time().as_double()); }
		if (m_trace) LOG("  F000 rd pc=%06x bw=%06x term=%06x A0=%04x -> %04x\n", m_cpu->pc(), m_last_bw, term, u16(m_cpu->state_int(M68K_A0)), d);
	}
	if ((a == 0xe01e || a == 0xe000) && m_fw_driven && machine().time().as_double() > 5.7)
	{ static int _n = 0; if (_n++ < 50) logerror("CHrd %04x op_ok=%d serdes=%d pc=%06x @%.4f\n", a, m_ch_op_ok, m_serdes_active, m_cpu->pc(), machine().time().as_double()); }
	// cont.196 (STRIP): the mid-pair PEN CALLER - the $88AC chain's E01E ack read at $88CA;
	// dump the stack top (the bsr return address names which of the six callers ran this pen).
	if (a == 0xe01e && storager_getenv("STORAGER_PHASELOG") && m_cpu->pc() >= 0x88c6 && m_cpu->pc() <= 0x88d6)
	{ static int _pn = 0; double const _pt = machine().time().as_double();
		if (_pt > 7.99 && _pt < 8.30 && _pn++ < 40)
		{ address_space &ps = m_cpu->space(AS_PROGRAM);
			u32 const sp = u32(m_cpu->state_int(M68K_SP)) & 0xffff;
			logerror("PENCALL sp=%04x stk=%04x %04x %04x %04x %04x %04x @%.6f\n", sp,
				ps.read_word(sp), ps.read_word(sp + 2), ps.read_word(sp + 4),
				ps.read_word(sp + 6), ps.read_word(sp + 8), ps.read_word(sp + 10), _pt); } }
	if (a == 0xe01e && (!m_fw_driven || m_ch_op_ok))   // disk-channel error-status register; bit4 = read-OK/
		d |= 0x0010;   // no-error (result handler 0x1814: btst #4,D1 posts 0x82 ERROR when clear). Non-fw-driven:
		               // always forced (shim). fw-driven: a LEVEL, held while the last channel op succeeded -
		               // the fw parameterizes E01E by writing it, so a store-only bit gets clobbered.
	// E000 = the 74LS1801/1802 ENDEC/SERDES data register: on a read op the firmware pulls each disk
	// byte in sequence. Serve the current track's raw byte stream (built lazily from the mounted image's
	// decoded ID+data fields, consumed with wraparound = continuous rotation) - the real hardware path.
	// The 74LS1812 SERDES serves E000 from the LIVE flux: advance the PLL to now and return the
	// byte under the head. No synthetic stream, no snoop; the ID/DATA marks fire from
	// flux_advance_to as the address marks pass. This is the raw-bitstream read path.
	if (m_fw_driven && a == 0xe000 && m_serdes_active)
	{
		floppy_image_device *const fdd = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
		if (fdd && fdd->exists())
		{
			int const side = m_cpu->space(AS_PROGRAM).read_word(0x7436) & 1;   // fw target head
			u32 const tkey = (u32(fdd->get_cyl()) << 1) | unsigned(side);
			if (m_flux_fdd != fdd || tkey != m_flux_track)   // (re)start the live-run for this track
			{
				fdd->mon_w(0); fdd->ss_w(side);
				m_flux_track = tkey;
				flux_read_reset(fdd, decode_track_ids(fdd, fdd->get_cyl(), side).fm);
				m_pump->adjust(attotime::from_usec(200));   // start the 74LS1812 mark clock
				if (m_trace) LOG("  FLUX-START cyl%d head%d fm=%d @%.6f\n", fdd->get_cyl(), side, m_flux_fm, machine().time().as_double());
			}
			flux_advance_to(machine().time());
			d = m_flux_byte;
			m_ch_op_ok = true;
		}
		if (m_trace) LOG("  FLUX E000 = %02x @%.6f\n", d, machine().time().as_double());
		return d;
	}
	if (m_trace && a != 0xf000) LOG("  CH rd %04x = %04x [%s]\n", a, d, machine().describe_context());
	return d;
}

void multibus_storager_device::ch_w(offs_t offset, u16 data, u16 mem_mask)
{
	u32 const a = 0xe000 + offset * 2;
	COMBINE_DATA(&m_ch[offset]);
	if (m_trace) LOG("  CH wr %04x = %04x [%s]\n", a, data, machine().describe_context());
	// task#5 (STRIP): the OLD trigger here (arm stepdone on any E800 write from $3690-$36c0) was WRONG -
	// measured runs 9-11: the $3694 E800 writes are the wait loop's own polling WAGGLE, not submissions;
	// arming on them manufactured 746 spurious completions (31.6ms cycle) and the real $36ac step train
	// never ran. Completion now keys on the PHYSICAL step pulses reaching stp_w (the E802 stepbit edge,
	// below at the drive block) - a drive completes a seek because the head stepped, not because the
	// controller polled it. Log-only tap kept for the phase trace:
	if (storager_getenv("STORAGER_STEPIRQ") && a == 0xe800 && m_cpu->pc() >= 0x3690 && m_cpu->pc() <= 0x36c0)
	{
		// The $3694 loop-ENTRY write is the op submission; a ZERO-step seek (run13: read at cyl 0,
		// head at cyl 0 - no stp_w edge ever fires) still completes after the settle. Arm ONCE per
		// op entry (m_stepdone_armed); real step trains re-arm per stp_w edge below, so a long seek
		// completes after its LAST step + settle, a zero-step seek after settle alone.
		// 1ms not 30: the fw's inline $369a poll window is short - run14 measured the 30ms bump
		// landing AFTER the fw parked back to the router idle, turning the pipeline into a 2.86s
		// phase-mismatch cycle (bump@+30ms sat unconsumed until the next periodic channel re-visit).
		// The known-good HLE-era restore->read handoff is a ~55ms BURST - completions are fast.
		static int _wn = 0; if (_wn++ < 4) logerror("E800-OPENTRY %04x pc=%06x (arm settle) @%.5f\n", data, m_cpu->pc(), machine().time().as_double());
		if (!m_stepdone_armed)
		{
			m_stepdone_armed = true;
			m_stepdone->adjust(attotime::from_msec(1));
		}
	}
	// Per-op seek completion (task #4) is delivered by the m_seek_done TIMER (armed at the 0x89/0x98 doorbell
	// for the settle time), NOT from this E800 write hook - the fw pure-spins $369a post-burst without writing
	// E800, so a hook here fires too late.  The timer (seek_done_tick) raises IRQ2 at settle; the fw's own
	// $24ea walk then bumps [$71b2] (node+26 is 0 by then, so the $251e gate passes).  See seek_done_tick.
	// TEMP (STRIP): LLE-keystone validation.  At the firmware's host-DMA GO (E800 bit12 set, fw $13ce)
	// log the host address the FIRMWARE programmed (D000 local, D800:C800 host) next to the HLE's
	// m_iopb_buffer.  If D800:C800 tracks the real host buffer, the firmware-driven DMA lands correctly.
	if (storager_getenv("STORAGER_DMALOG") && a == 0xe800 && (m_d000 || m_d800 || m_c800[0]))
	{
		static int _dn = 0;
		if (_dn++ < 120)
			logerror("E800wr=%04x D000=%04x(loc=%06x) D800:C800=%04x:%04x host=%06x term=%04x iopb_buf=%06x pc=%06x @%.4f\n",
					data, m_d000, u32(m_d000) << 1, m_d800, m_c800[0],
					(u32(m_d800) << 16) | m_c800[0], m_dma_term, m_iopb_buffer, m_cpu->pc(), machine().time().as_double());
	}
	// ACK the level-held HD channel completion IRQ4: the ISR clears the source at 0x3c02-0x3c08 by writing
	// E800 with bit12 cleared. Level semantics = hold asserted until the fw acknowledges the cause.
	if (m_hd_chan_irq4 && a == 0xe800 && !BIT(data, 12) && m_cpu->pc() >= 0x3c00 && m_cpu->pc() <= 0x3c10)
	{
		m_hd_chan_irq4 = false;
		m_hd_chan_pending = false;
		logerror("HDCHAN ack (E800 bit12 clr pc=%06x) -> IRQ4 CLEAR @%.4f\n", m_cpu->pc(), machine().time().as_double());
		m_cpu->set_input_line(M68K_IRQ_4, CLEAR_LINE);
	}
	// SERDES read-arm gating: E000 is multiplexed. The read command (0x2ff) / data-phase mode (0x?fff)
	// opens the data-read window; the idle/config value (0x23f) closes it. Only while armed does an
	// E000 READ clock out disk bytes; otherwise the fw is reading channel status/echo (must NOT stream).
	if (m_fw_driven && a == 0xe000)
	{
		// cont.38vv (v2 latch): MODE BIT7 = THE SYNC-TYPE SELECT - $22f/$23f/$a6d (bit7=0)
		// = IDAM side; $2af/$2ff (bit7=1) = data side. Every window close/operating
		// restore resets to IDAM by construction - the v1 stickiness dissolves.
		// cont.39n: the fw's OPERATING-mode write ($a6d-class, uniquely bit11 among the
		// mode words) = its channel ENGAGEMENT - the window opens on it. (Run140: with
		// ROT-START no longer opening, nothing honored the fw's own $a6d open at op
		// start and the event engine went silent; run97's failure was the everything-
		// opens rule, not this one - $22f/$23f stay window-neutral/close.)
		if (BIT(data, 11))
		{
				m_serdes_active = true;
				// The $a6d engagement opens the read window: START the 74LS1812 live-run here (over the
				// real flux, at the track's density) and run the mark clock, so its ID/DATA marks drive
				// the fw through its setup exactly as real hardware does.
				if (floppy_image_device *const efdd = m_floppy[0] ? m_floppy[0]->get_device() : nullptr)
				{
					int const eside = m_cpu->space(AS_PROGRAM).read_word(0x7436) & 1;
					u32 const ekey = (u32(efdd->get_cyl()) << 1) | unsigned(eside);
					if (m_flux_fdd != efdd || ekey != m_flux_track)
					{
						efdd->mon_w(0); efdd->ss_w(eside);
						m_flux_track = ekey;
						flux_read_reset(efdd, decode_track_ids(efdd, efdd->get_cyl(), eside).fm);
						m_pump->adjust(attotime::from_usec(200));
						logerror("FLUX-START cyl%d head%d fm=%d @%.6f\n", efdd->get_cyl(), eside, m_flux_fm, machine().time().as_double());
					}
				}
			if (!m_engaged) logerror("ARMGATE: ENGAGED ($a6d-class bit11 write %03x) @%.6f\n", data & 0xfff, machine().time().as_double());
			m_engaged = true;   // cont.191: the fw's own per-op engagement opens the mark window
		}
		// cont.190 (ARMGATE v2): track the fw's type-specific arm codes (spec §3.1).
		{ u16 const ac = data & 0xfff;
			if (ac == 0x22f) { if (!m_armed_idam) logerror("ARMGATE: IDAM armed (0x22f) @%.6f\n", machine().time().as_double()); m_armed_idam = true; }
			if (ac == 0x2af || ac == 0x2ff) { if (!m_armed_dam) logerror("ARMGATE: DAM armed (%03x) @%.6f\n", ac, machine().time().as_double()); m_armed_dam = true; } }
		// cont.38xx (STRIP): log every sync-select TRANSITION, both eras - the accept-
		// first diff: does the read era ever flip to data-mode (the post-accept $2af/$2ff
		// arm), as the setup era does?
		if (storager_getenv("STORAGER_PHASELOG") && BIT(data, 7) != m_endec_data_mode)
		{ static int _n = 0; double const t = machine().time().as_double();
			if (t > 6.35 && _n++ < 40)
				logerror("SYNCSEL -> %s (E000<-%04x) pc=%06x @%.5f\n",
					BIT(data, 7) ? "DATA" : "ID", data, m_cpu->pc(), t); }
		m_endec_data_mode = BIT(data, 7);
		if (data == 0x02ff || (data & 0x0fff) == 0x0fff)
		{
			m_serdes_active = true;
		}
		else if (data == 0x023f)
		{
			// cont.37: $23f closes the E000 read MUX only - the spindle (rotational tick)
			// keeps running; the fw's capture-arm sequence ($88ac) closes the window with a
			// bit15 capture armed that must still complete at the next ID mark (run63: the
			// old pump-stop here at pc $892e starved the armed capture forever).
			m_serdes_active = false;
		}
		else if (data == 0x022f)
		{
			// cont.38d: the $22f prime (sync-to-IDAM window, $88ac/$9602) types the armed
			// capture as ID - it completes at the next ID boundary. A bare E802 re-arm with
			// no prime ($92b4's $8a00) stays DATA-typed.
			m_idcap_id_typed = true;
		}
		// cont.38t REVERTED (run97): opening the mux on any non-$23f write re-created the
		// long-refuted ungated-streaming failure (the fw's scattered non-data E000 status
		// reads got stream bytes; the op restarted once per rev, 126 READ-STARTs). The old
		// open-set ($2ff / $xfff) was ALWAYS sufficient: the data-pull path ($7fc6) issues
		// $2af then $2ff before pulling - the mux opens on the fw's own $2ff. Wire 2 was
		// unnecessary; only wire 1 (the bit9 IRQ5 strobe) was real.
		// cont.37 instrument (STRIP): every E000 MODE write with pc + serdes state - names who
		// killed the rotation and which mode sequence the capture arm uses ($22f/$23f/$2af/$2ff).
		if (storager_getenv("STORAGER_NOBYPASS"))
		{ static int _n = 0, _m = 0; double const t = machine().time().as_double();
			if ((t > 6.3 && t < 6.5 && _n++ < 40) || (t > 9.795 && t < 9.81 && _m++ < 40))
			logerror("E000MODE <-%04x serdes=%d pc=%06x @%.5f\n", data,
				m_serdes_active, m_cpu->pc(), t); }
	}
	// The gate-array READ DMA (VERIFIED $5FC0 @0x5ede/0x5ee2): the firmware arms the channel for a read
	// by writing E800 = (op-family from the $63e table, bits 5-7) | bit7; the gate array then DMAs the
	// raw track bytes into the local SRAM sector buffer at 0x4000, which the fw parses for its sector.
	// Model it: on the arm, fill 0x4000 with the mounted image's raw track byte stream (build_serdes_
	// stream - the byte content the ENDEC would clock in). One track = up to 4096 bytes (the fw's count).
	// TEMP LOG (STRIP): B0 calibration - E802/E800/E804 writes with the physical cylinder
	if (a == 0xe802)
	{
		static int n = 0; static u16 last = 0xeeee;
		if (data != last && n++ < 150)
		{
			floppy_image_device *const fdd = m_floppy[0]->get_device();
			logerror("B0 E802 = %04x  cyl=%d pc=%06x @%.4f\n", data, fdd ? fdd->get_cyl() : -1, m_cpu->pc(), machine().time().as_double());
			last = data;
		}
	}
	// Phase B: the read-channel DATA arm - the fw's IRQ4 sequencer writes the $79CA/CC mode pair
	// into the E000 file (0x2FFF family, bits 8-11 set distinguish it from the 0x023F idle value).
	if (m_fw_driven && a == 0xe000 && (data & 0x0fff) == 0x0fff)
	{
		LOG("DATAARM E000=%04x (pc=%06x) -> disk op scheduled @%.4f\n", data, m_cpu->pc(), machine().time().as_double());
		m_dataop->adjust(attotime::from_msec(3));   // ~rotational latency to the next sector
	}
	// The E000 ID-capture start ritual: {0x2AF, rd, 0x2FF, rd, mode} with the E802 arm bits (15+11)
	// set and D800 = the local ID buffer ($7DAC/$7D9E as word addrs).
	if (a == 0xe000 && data == 0x02ff)
	{
		u16 const e802 = m_ch[(0xe802 - 0xe000) / 2];
		u32 const dst = u32(m_d800) << 1;
		if (storager_getenv("STORAGER_PHASELOG")) logerror("PHASE E000-2ff    D800<<1=%04x e802=%04x pc=%06x cmd=%02x @%.5f\n", dst, e802, m_cpu->pc(), m_iopb_cmd, machine().time().as_double());
		// TEMP LOG (STRIP): every E000 start, unconditionally
		{ static int n = 0; if (n++ < 40) logerror("E000START e802=%04x d800=%04x(byte %06x) d000=%04x c800[0]=%04x pc=%06x @%.4f\n",
				e802, m_d800, dst, m_d000, m_c800[0], m_cpu->pc(), machine().time().as_double()); }
		if ((e802 & 0x8800) == 0x8800 && (dst == 0x7dac || dst == 0x7d9e))
		{
			floppy_image_device *const fdd = m_floppy[0]->get_device();
			// B0 assertion (Dave): unit / direction / physical cylinder at every capture start
			LOG("IDCAP start: unit-sel=%02x dir13=%d cyl=%d dst=%04x pc=%06x @%.4f\n",
					m_ch[(0xe804 - 0xe000) / 2] & 0xff, BIT(e802, 13), fdd ? fdd->get_cyl() : -1,
					dst, m_cpu->pc(), machine().time().as_double());
			m_idcap_dst = dst;
			m_idcap->adjust(attotime::from_msec(2));   // ~next-ID rotational delay (refine later)
		}
	}

	// E800 = DMA/channel control.  bit6 = DMA enable (0x?d3 start / 0x?12 stop);
	// bit9 = 8253 counter-0 gate (timer self-test: 0xc14 gate off -> 0xe14 gate on).
	// cont.40f v2 (STRIP): E804 (drive-select/side) history - in-handler (the write tap missed).
	if (a == 0xe804 && storager_getenv("STORAGER_PHASELOG"))
	{ static u16 _e4last = 0xbeef; static int _e4n = 0, _b4n = 0;
		if (data != _e4last && _e4n++ < 40)
			logerror("E804WR %04x pc=%06x @%.5f\n", data, m_cpu->pc(), machine().time().as_double());
		// cont.83 (STRIP): bit4 EDGES across the whole run - when is the "motor" bit ever set?
		if (BIT(data, 4) != BIT(_e4last, 4) && _b4n++ < 60)
			logerror("E804-B4 %d->%d (%04x) pc=%06x @%.5f\n", BIT(_e4last, 4), BIT(data, 4), data, m_cpu->pc(), machine().time().as_double());
		_e4last = data; }
	// cont.166 (STRIP): E807 channel-command census - the byte the $283c dispatch sends.
	if (a == 0xe806 && ACCESSING_BITS_0_7)
	{ static int n = 0; if (n++ < 200)
		logerror("E807CMD %02x e806hi=%02x pc=%06x @%.6f\n", data & 0xff, (data >> 8) & 0xff, m_cpu->pc(), machine().time().as_double()); }
	// cont.168 (TEST, env STORAGER_E807ACK): on a ring dispatch ($28c6/$28d2 write E807),
	// read the dispatching descriptor (ring head [$7378] -> $737c+head*8) and schedule the
	// completion post on its own mailbox (desc+2). 20ms placeholder delay - jam probe only.
	// Also log the full dispatch context (shadows + E802/E804) as cross-check material for
	// Dave's E807->shadow->E802/E804 static table.
	if (a == 0xe806 && ACCESSING_BITS_0_7 && storager_getenv("STORAGER_E807ACK"))
	{
		u32 const dpc = m_cpu->pc();
		if (dpc >= 0x28c0 && dpc <= 0x28e0)
		{
			address_space &ds = m_cpu->space(AS_PROGRAM);
			u16 const head = ds.read_word(0x7378) % 20;
			u16 const desc = 0x737c + head * 8;
			u16 const mbox = ds.read_word(desc + 2);
			logerror("E807DISP head=%u desc={%04x %04x %04x %04x} shadows 7fe0=%04x 7fe4=%04x 7fe8=%04x 7fec=%04x e802=%04x e804=%04x @%.6f\n",
				head, ds.read_word(desc), ds.read_word(desc + 2), ds.read_word(desc + 4), ds.read_word(desc + 6),
				ds.read_word(0x7fe0), ds.read_word(0x7fe4), ds.read_word(0x7fe8), ds.read_word(0x7fec),
				m_ch[(0xe802 - 0xe000) / 2], m_ch[(0xe804 - 0xe000) / 2], machine().time().as_double());
			if (mbox == 0x7ff0 || mbox == 0x7ff8)
				m_e807ack->adjust(attotime::from_msec(20), mbox);
		}
	}
	// cont.170b (STRIP): boot-window E802 writes - the IRQ2 handler's entry fingerprint is
	// the bit6 ack pulse ($24ae andi/$24ba ori). Does IRQ2 get TAKEN after the 0.4027 post?
	if (a == 0xe802 && machine().time().as_double() > 0.400 && machine().time().as_double() < 0.470)
	{ static int n = 0; if (n++ < 60)
		logerror("E802WIN %04x pc=%06x @%.6f\n", data, m_cpu->pc(), machine().time().as_double()); }
	if (a == 0xe800)
	{
		m_dma_active = BIT(data, 6);
		// cont.160 (STRIP): GATE0 edges, whole run - the tick's gate regime.
		{ static int g0 = -1, gn = 0; int const nb = BIT(data, 9);
			if (nb != g0 && gn++ < 120)
				logerror("GATE0 %d->%d pc=%06x @%.6f\n", g0, nb, m_cpu->pc(), machine().time().as_double());
			g0 = nb; }
		m_gate0 = BIT(data, 9);
		m_pit[1]->write_gate0(BIT(data, 9));

		// kickoff = the RISING EDGE of bit12 (the disk-op does ori #$1000; the IRQ4 handler clears it).
		// Edge-detect so incidental E800 writes that carry bit12 (e.g. the IRQ1 timer handler) don't
		// re-trigger a transfer mid-DMA.
		bool const kick = BIT(data, 12) && !m_e800_bit12_prev;
		if (kick) s_grind.kick++;
		// cont.53 (STRIP): the 10s bucket reporter (piggybacked on frequent E800 writes)
		{ double const gt = machine().time().as_double();
			if (storager_getenv("STORAGER_PHASELOG") && gt - s_grind.last >= 10.0)
			{ address_space &gs = m_cpu->space(AS_PROGRAM);
				logerror("GRIND t=%.0f match17=%d dequeue=%d restock=%d kick=%d | 74ac=%04x 74ae=%04x 7424=%04x pc=%06x @%.5f\n",
					gt, s_grind.match17, s_grind.dequeue, s_grind.restock, s_grind.kick,
					gs.read_word(0x74ac), gs.read_word(0x74ae), gs.read_word(0x7424), m_cpu->pc(), gt);
				logerror("GRIND-SR sr=%04x @%.5f\n", u16(m_cpu->state_int(M68K_SR)), gt);
				s_grind.match17 = s_grind.dequeue = s_grind.restock = s_grind.kick = 0;
				s_grind.last = gt; } }
		m_e800_bit12_prev = BIT(data, 12);
		// cont.281 (FAITHXFER, Dave): the firmware-driven per-sector transfer. On the E800 bit12
		// kick, if the read descriptor $748a is fully armed ([$743a]==[$7a14]==$748a) and C000 presets
		// a host address inside the window, DMA that sector from the SERDES window buffer to the host,
		// then raise IRQ4 so $3bfe's identity branch routes to the $1310/$1348 re-arm (NOT the generic
		// null-callback path). This is the transfer the flux->host shortcut used to pre-empt.
		if (kick && storager_getenv("STORAGER_FAITHXFER") && m_c000_valid
				&& (m_iopb_cmd == 0x95 || m_iopb_cmd == 0x94))
		{
			address_space &ks = m_cpu->space(AS_PROGRAM);
			u16 const n743a = ks.read_word(0x743a);
			u16 const n7a14 = ks.read_word(0x7a14);
			u32 const hbase = s_desc.host ? s_desc.host : (m_iopb_buffer & 0xffffff);
			u32 const kssz  = s_desc.ssz ? s_desc.ssz : 128;
			bool const armed = (n743a == 0x748a) && (n7a14 == 0x748a);   // the identity $3bfe keys on
			if (armed && hbase && m_c000 >= hbase && m_c000 < 0x100000)
			{
				u32 const kn = (m_c000 - hbase) / kssz;
				bool const have = (std::size_t(kn) * kssz < m_win_buf.size()) && BIT(m_read_hostmap, kn);
				if (have)
				{
					address_space &tbs = m_bus->space(AS_PROGRAM);
					for (u32 k = 0; k < kssz; k++)
						tbs.write_byte((m_c000 + k) & 0xffffff, m_win_buf[std::size_t(kn) * kssz + k]);
				}
				m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE);   // identity-armed transfer-complete
				if (storager_getenv("STORAGER_PHASELOG"))
					logerror("FAITHXFER kick n=%u c000=%06x have=%d -> IRQ4 (743a=%04x 7a14=%04x 7a76=%04x) @%.6f\n",
						kn, m_c000, have, n743a, n7a14, ks.read_word(0x7a76), machine().time().as_double());
			}
			else if (storager_getenv("STORAGER_PHASELOG"))
				logerror("FAITHXFER kick NOT-ARMED c000=%06x 743a=%04x 7a14=%04x @%.6f\n",
					m_c000, n743a, n7a14, machine().time().as_double());
		}
		// TEMP (STRIP): the CHANCOMPLETE kick-signature diff (Dave) - 0x87 scan (completes) vs restore re-scan.
		if (storager_getenv("STORAGER_PHASELOG") && BIT(data, 12))
		{ static int _kn = 0; double t = machine().time().as_double(); if (t > 6.39 && t < 8.0 && _kn++ < 100)
			logerror("E800kick data=%04x bit12=%d kick=%d d000=%04x CHANCOMPLETE=%d cmd=%02x 791a=%04x pc=%06x @%.5f\n",
				data, (data >> 12) & 1, kick, m_d000, (kick && m_d000) ? 1 : 0, m_iopb_cmd,
				m_cpu->space(AS_PROGRAM).read_word(0x791a), m_cpu->pc(), t); }
		// cont.38n (STRIP): ALL E800 writes in the read era - the kick pattern the capture
		// arm must key on (value, bit12, edge, pc) per capture cycle.
		if (storager_getenv("STORAGER_PHASELOG") && m_cpu->pc() != 0x31cc && m_cpu->pc() != 0x31d8)
		{ static int _en = 0; double t = machine().time().as_double(); if (t > 7.97 && t < 9.5 && _en++ < 120)
			logerror("E800WR data=%04x b12=%d edge=%d c800=%04x D800<<1=%04x pc=%06x @%.5f\n",
				data, (data >> 12) & 1, kick, m_c800[0], u32(m_d800) << 1, m_cpu->pc(), t); }

		// channel-start: E800 bit12 (0x1000) launches the transfer (disk-op sets it via ori #$1000).
		// Dump the DMA setup so the data path can be mapped: D000 (local word addr), the active C800
		// scatter/gather entries (Multibus page translation), and CPU regs (descriptor pointer A3).
		if (kick && m_trace)
		{
			address_space &cs = m_cpu->space(AS_PROGRAM);
			LOG("CHANSTART E800=%04x D000=%04x(byte %06x) A2=%04x A3=%04x D0=%04x D1=%04x\n",
				data, m_d000, u32(m_d000) << 1,
				u16(m_cpu->state_int(M68K_A2)), u16(m_cpu->state_int(M68K_A3)),
				u16(m_cpu->state_int(M68K_D0)), u16(m_cpu->state_int(M68K_D1)));
			std::string sg;
			for (unsigned i = 0; i < 0x100; i++)
				if (m_c800[i]) sg += util::string_format(" [%02x]=%04x", i, m_c800[i]);
			LOG("CHANSTART C800:%s\n", sg.empty() ? " (empty)" : sg.c_str());
			u16 const a3 = u16(m_cpu->state_int(M68K_A3));
			std::string ds;
			for (int k = -8; k <= 12; k += 2) ds += util::string_format(" %04x", cs.read_word((a3 + k) & 0xffff));
			LOG("CHANSTART A3-descr:%s\n", ds.c_str());
			// dump the D000 local buffer + the 0x7440 work buffer: look for the Multibus addr 0x0fe780
			// (bytes 0f e7 80, or word 0xf3c0 = 0x0fe780>>1) = a memory-resident DMA descriptor.
			u16 const db = u16(m_d000) << 1;
			std::string lb;
			for (int k = -16; k <= 16; k += 2) lb += util::string_format(" %04x", cs.read_word((db + k) & 0xffff));
			LOG("CHANSTART D000buf @%04x:%s\n", db, lb.c_str());
			std::string wb;
			for (u16 k = 0x7430; k < 0x7460; k += 2) wb += util::string_format(" %04x", cs.read_word(k));
			LOG("CHANSTART 7440buf:%s\n", wb.c_str());
		}

		// cont.47 THE TRUCK: the gate array's host-DMA bus copy, fully specified by the fw's own
		// construction: cell byte-offset = (~host >> 15) & $1fe ($12f0), so host[23:16] =
		// ~(cell_index) and the cell VALUE = ~host[15:0] (the inverted-counter idiom; verified:
		// idx $f0/value $3f22 -> 0fc0dd). Armed by the $3abc signature: a HIGH-file C800 cell
		// (index >= 0x80, the $c9xx range the half-map used to drop) freshly nonzero at kick
		// time. Direction: cmd 0x95 = local -> host; src = D000<<1; length = the unit's sector
		// size (v1; the real count register TBD from the first live kick's programming).
		if (kick && storager_getenv("STORAGER_NOBYPASS"))
		{
			for (unsigned ci = 0x80; ci < 0x100; ci++)
				if (m_c800[ci])
				{
					u32 const host = (u32(~ci & 0xff) << 16) | (u16(~m_c800[ci]) & 0xffff);
					u32 const src = u32(m_d000) << 1;
					u32 const len = m_unit_secsize[2] ? m_unit_secsize[2] : 128;
					address_space &tcs = m_cpu->space(AS_PROGRAM);
					address_space &tbs = m_bus->space(AS_PROGRAM);
					logerror("TRUCKRUN cell[%02x]=%04x -> host=%06x src(local)=%04x len=%u first4=%02x %02x %02x %02x @%.5f\n",
						ci, m_c800[ci], host, src, len, tcs.read_byte(src), tcs.read_byte(src + 1),
						tcs.read_byte(src + 2), tcs.read_byte(src + 3), machine().time().as_double());
					if (host >= 0x080000 && host < 0x100000)   // CPUAP RAM window sanity
						for (u32 k = 0; k < len; k++)
							tbs.write_byte((host + k) & 0xffffff, tcs.read_byte((src + k) & 0xffff));
					// the inverted counter advances as the DMA runs (the fw reads it back to
					// track progress; page-crossing TBD from live programming)
					m_c800[ci] = u16(~((host + len) & 0xffff));
					break;
				}
		}
		// cont.203 (STORAGER_MBFETCH): serve the HOST->LOCAL fetch class from the modeled C000
		// counter - the UIB block fetch (B = the IOPB's buffer field, NOT the IOPB pointer:
		// the doorbell engine already moved the IOPB). Without this the launch completed with
		// no transfer and the fw validated a STALE UIB (cont.202's six-byte diff).
		// cont.210 (STRIP): every kick's guard decision - which clause blocks a fetch (the 4th
		// command's UIB fetch was silently rejected; the deferred IRQ4 then announced a
		// completion for a transfer that never ran -> the phase-0 dispatch death).
		if (kick && storager_getenv("STORAGER_PHASELOG"))
		{ static int _gn = 0; if (_gn++ < 60)
			logerror("KICKGUARD c000=%06x(v%d) d000=%04x buf=%06x iopb=%06x cmd=%02x -> %s @%.5f\n",
				m_c000, m_c000_valid ? 1 : 0, m_d000, m_iopb_buffer & 0xffffff, m_iopb_addr & 0xffffff,
				m_iopb_cmd,
				(m_c000_valid && m_d000 && m_c000 == (m_iopb_buffer & 0xffffff)
					&& m_c000 != (m_iopb_addr & 0xffffff) && m_c000 >= 0x010000 && m_c000 < 0xff0000)
					? "FETCH" : "no-fetch", machine().time().as_double()); }
		// cont.214 (STORAGER_TRUCK): THE PER-SECTOR LOCAL->HOST DMA - the fw's own read
		// delivery: per accepted sector it stages D000 = the local chunk, presets C000 = the
		// host sector address ($3E9C pair advance -> $3D08 preset, rotational order), and
		// kicks. Serve it: copy one sector (live UIB secsize) local->host. This is the
		// honest transfer whose absence forced the v6 carry's window arithmetic (and whose
		// pair-advance the model starved into the +0x100 continuation bug).
		if (kick && storager_getenv("STORAGER_TRUCK") && m_c000_valid && m_d000
				&& (m_iopb_cmd == 0x95 || m_iopb_cmd == 0x94)
				&& m_c000 >= 0x080000 && m_c000 < 0x100000
				&& m_c000 != (m_iopb_addr & 0xffffff))
		{
			address_space &tcs = m_cpu->space(AS_PROGRAM);
			address_space &tbs = m_bus->space(AS_PROGRAM);
			u32 const src = u32(m_d000) << 1;
			{
			u16 const uibp = tcs.read_word(0x799a);
			u32 ssz = tcs.read_byte((uibp + 2) & 0xffff) | (u32(tcs.read_byte((uibp + 3) & 0xffff)) << 8);
			if (ssz < 0x80 || ssz > 0x800) ssz = 128;
			if (storager_getenv("STORAGER_REC512") || storager_getenv("STORAGER_PAIR512")) ssz = 512;
			else if (storager_getenv("STORAGER_PAIR256")) ssz = 256;   // cont.233: only under the sub-env - the
			                                             // decisive run tests per-sector kicks (128B)
			                                             // with paired deposits
			// cont.257s (DAVE'S MODEL, the transfer side): the HOST ADDRESS the fw presets
			// (C000) DETERMINES which sector belongs there - host+0 = logical block 0 = VOL1,
			// host+128 = block 1, etc. So the host-DMA does NOT depend on deposit/aim timing:
			// compute the sector from the host offset (exactly read95_deliver / READVERIFY) and
			// deliver it by random access. This decouples the transfer from the arm-paced
			// deposit timing entirely - fully synchronous, robust to pacing.
			bool served_by_pos = false;
			u32 const pos_base2 = m_iopb_buffer ? (m_iopb_buffer & 0xffffff) : s_desc.host;
			if ((m_iopb_cmd == 0x95 || m_iopb_cmd == 0x94) && pos_base2
					&& m_c000 >= pos_base2 && !m_sectors.empty())
			{
				u32 const spt_h = 16;
				u32 const sec0_h = (ssz != 128) ? 1 : (m_unit_sec0[2] ? m_unit_sec0[2] : 7);
				u32 const n = (m_c000 - pos_base2) / ssz;   // logical block index in the read
				u32 const trk = s_desc.base_trk + n / spt_h, sidx = n % spt_h;
				u32 const cyl_h = trk >> 1, head_h = trk & 1;
				u32 const wantR = ((sec0_h - 1 + sidx) % spt_h) + 1;
				auto const hit = m_sectors.find((cyl_h << 16) | (head_h << 8) | wantR);
				if (hit != m_sectors.end())
				{
					for (u32 k = 0; k < ssz; k++)
						tbs.write_byte((m_c000 + k) & 0xffffff, (k < hit->second.size()) ? hit->second[k] : 0);
					served_by_pos = true;
					static int _sp = 0; if (_sp++ < 40)
						logerror("TRUCK-POS host+%u -> cyl%u h%u R=%02x first4=%02x %02x %02x %02x @%.5f\n",
							m_c000 - pos_base2, cyl_h, head_h, wantR,
							hit->second.size()>3?hit->second[0]:0, hit->second.size()>3?hit->second[1]:0,
							hit->second.size()>3?hit->second[2]:0, hit->second.size()>3?hit->second[3]:0,
							machine().time().as_double());
					// cont.257t: record window coverage; fire DESCDONE when every block landed.
					if (s_desc.active && ssz && n < 64 && n < s_desc.total / ssz)
					{
						m_read_hostmap |= (u64(1) << n);
						// cont.262d (STORAGER_PERSEC): fire the DATA-completion IRQ6 PER position-
						// delivered block (this IS the per-sector delivery path for the label read),
						// phase held by C135PH, so each sector's stake/convert ($92b4) lands early -
						// filling the ledger with c0 BEFORE the fw's error/give-up window.
						if (storager_getenv("STORAGER_PERSEC")
								&& (m_cpu->space(AS_PROGRAM).read_word(0x79f8) & 0x0800))
						{
							m_c135_pending = true;
							m_cpu->set_input_line(M68K_IRQ_6, HOLD_LINE);
							if (storager_getenv("STORAGER_PHASELOG")) { static int _pn = 0; if (_pn++ < 60)
								logerror("PERSEC-POS6 block=%u @%.6f\n", n, machine().time().as_double()); }
						}
						u32 const nblk = s_desc.total / ssz;
						u64 const full = (nblk >= 64) ? ~u64(0) : ((u64(1) << nblk) - 1);
						// cont.260: the read's FIRST sector's truck can land a few us BEFORE the model
						// detects DESCARM (which resets the bitmap), losing bit 0's coverage though the
						// data IS delivered (the read-ahead VOL1 at host+0). Treat bit 0 as covered - it
						// is always the read's first sector and always delivered; without this the window
						// sat at 0xfe forever and the fw (done at 7968=1) never got its channel completion.
						if (((m_read_hostmap | 1) & full) == full)
						{
							s_desc.active = false;
							m_read_pending = false;
							m_ch_op_ok = true;
							m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE);
							logerror("DESCDONE-POS %u bytes / %u blocks at host %06x (single position path) @%.5f\n",
								s_desc.total, nblk, pos_base2, machine().time().as_double());
							// cont.261 (STORAGER_LASTC0 VALIDATION PROBE, not the fix): the per-sector
							// accounting orphans the LAST sector (its $c0 transfer-complete rides the NEXT
							// capture; the final one has none), so owed [$79a8] never hits 0 and the ledger
							// tail stays $fe -> over-read. Satisfy it fully here and see if 0x80 posts.
							if (storager_getenv("STORAGER_LASTC0"))
							{
								address_space &ls = m_cpu->space(AS_PROGRAM);
								u16 const wbase = ls.read_word(0x7954);
								u16 const wcnt  = ls.read_word(0x7abc);
								u16 const owed0 = ls.read_word(0x79a8);
								std::string bfr, aft;
								for (u16 k = 0; k < 22; k++) bfr += util::string_format(" %02x", ls.read_byte((0x7654 + k) & 0xffff));
								for (u16 k = wbase; k < wbase + wcnt; k++)
									if (ls.read_byte((0x7654 + k) & 0xffff) != 0xc0) ls.write_byte((0x7654 + k) & 0xffff, 0xc0);
								ls.write_byte((0x7654 + wbase + wcnt) & 0xffff, 0xaa);   // END-MARKER
								ls.write_word(0x79a8, 0);                                // owed satisfied
								for (u16 k = 0; k < 22; k++) aft += util::string_format(" %02x", ls.read_byte((0x7654 + k) & 0xffff));
								logerror("LASTC0 wbase=%u wcnt=%u owed=%u->0 BEFORE:%s AFTER:%s @%.5f\n",
									wbase, wcnt, owed0, bfr.c_str(), aft.c_str(), machine().time().as_double());
							}
							read_verify_window(pos_base2, ssz);
						}
					}
				}
			}
			// cont.257t: the local-copy is a FALLBACK only when position delivery could not
			// resolve the sector (setup era / no base). The cont.256w empty-buffer skip stays
			// there - it never applies to a position-served read.
			if (!served_by_pos)
			{
				if (m_depot_r.find(u16(src)) != m_depot_r.end())
					for (u32 k = 0; k < ssz; k++)
						tbs.write_byte((m_c000 + k) & 0xffffff, tcs.read_byte((src + k) & 0xffff));
				else { static int _te = 0; if (_te++ < 20)
					logerror("TRUCK-EMPTY skipped local %04x @%.5f\n", src, machine().time().as_double()); }
			}
			// cont.256n (Dave's method - verify the ACTUAL bytes at every stage): the truck
			// moves local -> the fw's own C000 host address, so ORDER is the firmware's
			// business; what the model owes is that the local buffer holds the sector the fw
			// AIMED at. Check exactly that: expected R = ((sec0-1 + aim-1) mod spt)+1, and
			// diff the moved bytes against that sector on the medium.
			{
				address_space &vs = m_cpu->space(AS_PROGRAM);
				u32 const spt_v = 16;
				u32 const sec0_v = (ssz != 128) ? 1 : (m_unit_sec0[2] ? m_unit_sec0[2] : 7);
				// cont.256t: verify against the sector actually IN the buffer (what we
				// deposited), not the live aim - a truck carries the sector captured at the
				// PREVIOUS aim, so an aim-derived expectation is off by one and reported
				// false mismatches (run297: R7/VOL1 correctly landing at host+0 was flagged
				// against R8). The question this must answer is "did the bytes we moved match
				// that sector on the medium".
				u16 const aim_v = vs.read_word(0x7428);
				auto const dit = m_depot_r.find(u16(src));
				u8 const gotR = (dit != m_depot_r.end()) ? dit->second : 0xff;
				u32 const expR = (gotR != 0xff) ? gotR
					: (aim_v ? (((sec0_v - 1 + (aim_v - 1)) % spt_v) + 1) : 0);
				u32 const cyl_v = m_flux_track >> 1, head_v = m_flux_track & 1;
				auto const mit = m_sectors.find((cyl_v << 16) | (head_v << 8) | expR);
				u32 bad = 0; u32 firstk = ~0u; u8 e0 = 0, a0v = 0;
				if (mit != m_sectors.end())
					for (u32 k = 0; k < ssz; k++)
					{
						u8 const e = (k < mit->second.size()) ? mit->second[k] : 0;
						u8 const a = tbs.read_byte((m_c000 + k) & 0xffffff);
						if (e != a) { bad++; if (firstk == ~0u) { firstk = k; e0 = e; a0v = a; } }
					}
				logerror("TRUCKVERIFY aim=%u expR=%u gotR=%02x host=%06x len=%u -> %s (bad=%u first k=%d exp=%02x act=%02x) @%.5f\n",
					aim_v, expR, gotR, m_c000, ssz,
					(mit == m_sectors.end()) ? "NO-MEDIA-SECTOR" : (bad ? "MISMATCH" : "OK"),
					bad, int(firstk), e0, a0v, machine().time().as_double());
			}
			m_truck_seen = true;
			if (storager_getenv("STORAGER_REC512"))
				m_rec_r0 = u8(((unsigned(m_rec_r0 ? m_rec_r0 : 1) - 1 + 4) % 16) + 1);   // cont.240: next record base
			// cont.218: the served truck IS the segment DMA - latch the GA's segment-retired
			// status ([$7B0E] bit15 via the merge tap) + want-ready, exactly as the v6 claim
			// did. Without it the $74AC completed-chunk queue's pop (the launcher loop,
			// cont.127) stalls after the fresh-slot claims dry up -> no kick #2.
			m_seg_retired = true;
			m_want_ready = true;
			if (storager_getenv("STORAGER_PHASELOG")) { static int _tk = 0; if (_tk++ < 60)
				logerror("TRUCK sector local=%04x -> host=%06x len=%u first4=%02x %02x %02x %02x | c800[0]=%04x 741e=%04x 7428=%04x 7434=%04x @%.5f\n",
					src, m_c000, ssz, tcs.read_byte(src), tcs.read_byte(src + 1),
					tcs.read_byte(src + 2), tcs.read_byte(src + 3),
					m_c800[0], tcs.read_word(0x741e), tcs.read_word(0x7428), tcs.read_word(0x7434),
					machine().time().as_double()); }
			}
		}
		if (kick && storager_getenv("STORAGER_MBFETCH") && m_c000_valid && m_d000
				&& m_iopb_cmd != 0x95 && m_iopb_cmd != 0x94
				&& m_c000 == (m_iopb_buffer & 0xffffff) && m_c000 != (m_iopb_addr & 0xffffff)
				&& m_c000 >= 0x010000 && m_c000 < 0xff0000)
		{
			address_space &fcs = m_cpu->space(AS_PROGRAM);
			address_space &fbs = m_bus->space(AS_PROGRAM);
			u32 const dst = u32(m_d000) << 1;
			// cont.212: the UIB fetch length is 0x20 CONFIRMED - a 0x40 copy clobbers live fw
			// state at local dst+0x20 (boot stalls after the first INIT; the +0x20 host pointer
			// table is NOT part of this fetch). A/B: STORAGER_MBF40 re-tries the long copy.
			u32 const mbf_len = storager_getenv("STORAGER_MBF40") ? 0x40 : 0x20;
			for (u32 k = 0; k < mbf_len; k++)
				fcs.write_byte((dst + k) & 0xffff, fbs.read_byte((m_c000 + k) & 0xffffff));
			if (storager_getenv("STORAGER_PHASELOG"))
			{
				std::string full;
				for (u32 k = 0; k < 0x40; k++) full += util::string_format(" %02x", fbs.read_byte((m_c000 + k) & 0xffffff));
				logerror("MBFETCH host=%06x -> local=%04x x20 host[0..3f]:%s @%.5f\n",
					m_c000, dst, full.c_str(), machine().time().as_double());
			}
		}
		// A real transfer (D000 set) launched: the gate array runs it and raises IRQ4 on completion
		// (the IRQ4 handler at 0x3bfe clears E800 bit12).  Pulse IRQ4 so the command completes.
		// cont.40d (STRIP): TRUCKDRY - the truck's reference-programming dump. At EVERY kick,
		// log what the fw programmed: D000 (local), D800, C800 file (nonzero cells), op family
		// (bits 5-7), direction table byte. The 6.40 CCB/UIB kicks are the delivering reference;
		// the (future) data kicks get diffed against them before the copy is trusted.
		if (kick && storager_getenv("STORAGER_NOBYPASS"))
		{ static int _tn = 0; if (_tn++ < 40)
			{ std::string sg;
				for (unsigned i = 0; i < 0x100; i++)
					if (m_c800[i]) sg += util::string_format(" [%02x]=%04x", i, m_c800[i]);
				logerror("TRUCKDRY E800=%04x fam=%u D000=%04x(loc %05x) D800=%04x C800:%s iopb_buf=%06x cmd=%02x pc=%06x @%.5f\n",
					data, (data >> 5) & 7, m_d000, u32(m_d000) << 1, m_d800,
					sg.empty() ? " (empty)" : sg.c_str(), m_iopb_buffer & 0xffffff, m_iopb_cmd,
					m_cpu->pc(), machine().time().as_double()); } }
		if (kick && m_d000)
		{
			// The gate array autonomously bus-masters CPUAP RAM: on the first transfer of a command it
			// fetches the IOPB (the firmware never relocates the pointer - it lives only in the mailbox
			// at 0x7ff9-0x7ffb).  Byte-for-byte copy CPUAP RAM -> local D000<<1 (no endian swap; a real
			// byte DMA).  TEST: verify the destination via the debugger - 0x71f0 may be worker code.
			address_space &cs = m_cpu->space(AS_PROGRAM);
			address_space &bs = m_bus->space(AS_PROGRAM);
			u32 const dst = u32(m_d000) << 1;
			if (!m_iopb_fetched)
			{
				// 1st kickoff: fetch the IOPB from the mailbox pointer (V/SMD 3200 format).
				m_iopb_fetched = true;
				u32 const iopb = storager_getenv("STORAGER_MBLATCH") ? ((u32(m_mb_in[5]) << 16) | (u32(m_mb_in[6]) << 8) | m_mb_in[7]) : storager_getenv("STORAGER_MBOXW") ? ((u32(cs.read_byte(0x7ffa)) << 16) | (u32(cs.read_byte(0x7ffc)) << 8) | cs.read_byte(0x7ffe)) : ((u32(cs.read_byte(0x7ff9)) << 16) | (u32(cs.read_byte(0x7ffa)) << 8) | cs.read_byte(0x7ffb));
				m_dma_mark = 2;   // trace: kick IOPB fetch, dst from m_d000<<1 (stale-prone)
				// the host IOPB is 14 words (0x1c bytes); copy ONLY that - 0x1c+ is the firmware's
				// own work area (clobbering it with host data faults on a stale odd pointer).
				for (u32 k = 0; k < 0x18; k++)   // cont.209: 12 words - +0x18/+0x19 is the fw's error cell, NOT host IOPB
					cs.write_byte((dst + k) & 0xffff, bs.read_byte((iopb + k) & 0xffffff));
				m_dma_mark = 0;
				m_iopb_cmd = cs.read_byte(dst);   // w0 high byte = command
			m_cmd_armed = false;   // cont.189: new command boundary - not yet armed; stale-era marks must wait
			m_truck_seen = false;  // cont.214: new command - window-mode delivery allowed until a truck fires
			m_last_warp_aim = 0;   // cont.224: fresh aim space per command
			m_armed_idam = m_armed_dam = false;   // cont.190: per-type arms reset at the boundary
			m_engaged = false;   // cont.191: the previous command's engagement ends at the doorbell
				m_iopb_buffer = ((u32(cs.read_byte(dst + 0xc)) << 24) | (u32(cs.read_byte(dst + 0xd)) << 16)
					| (u32(cs.read_byte(dst + 0xe)) << 8) | cs.read_byte(dst + 0xf)) & 0xffffff;  // words 6-7
				m_iopb_addr = iopb;
				// post BUSY (0x81) to the host IOPB status (CPUAP polls IOPB+2 for 0x81->0x80).
				bs.write_byte((iopb + 2) & 0xffffff, 0x81);
				bs.write_byte((iopb + 3) & 0xffffff, 0x81);
				if (m_trace) LOG("GATE-DMA IOPB fetch: CPUAP %06x -> local %04x; cmd=%02x buffer=%06x\n", iopb, dst, m_iopb_cmd, m_iopb_buffer);
			}
			else if (m_iopb_cmd == 0x89)
			{
				// RESTORE: the position model returns to track 0 sector 0 (the physical drive
				// recalibrates via the fw's own step loop; keep the logical model in step)
				u32 const unit = cs.read_byte((dst + 4) & 0xffff) & 3;
				m_unit_trk[unit] = 0;
				m_unit_sidx[unit] = 0;
			}
			else if (m_iopb_cmd == 0x87)
			{
				// INITIALIZE: the controller reads the host-supplied UIB (drive geometry) from the
				// IOPB buffer addr into local RAM.  Host -> controller.  Copy only 0x1c bytes so the
				// firmware work area at dst+0x1c (read by later commands, e.g. RESTORE's ($24,A0)) survives.
				// STEP 131: latch the per-unit geometry the host declares - UIB[0]=heads, UIB[1]=sectors/track,
				// UIB[2-3]=bytes/sector (LE, NS32016-authored).  Unit from the IOPB byte[4] BEFORE the UIB
				// copy overwrites the local block.
				u32 const unit = cs.read_byte((dst + 4) & 0xffff) & 3;
				// The 0x87 UIB DMA must land in the fw's PER-UNIT UIB struct, NOT m_d000<<1=$71f0.  Measured
				// (line-2221 probe, match=0): D000 is STALE here - a leftover CCB pointer from a prior op - and
				// $71f0 is the SHARED command block, so DMAing the UIB there stamps a staged command (the read's
				// 0x95) mid-dispatch (the $71f0 clobber, ipl=4 red herring = this C++ DMA on the fw's IRQ4 ctx).
				// The gate array DMAs the host UIB to the controller's per-unit UIB storage = the fw's own pointer
				// [$20a+unit*2] (read it, don't hardcode - it's the dst the firmware programmed).  HLE keeps $71f0
				// (its read is shimmed, so the clobber is tolerated there).
				u32 uibcopy_dst = dst;
				if (storager_getenv("STORAGER_NOBYPASS")) uibcopy_dst = cs.read_word((0x20a + unit * 2) & 0xffff);
				// cont.39i: THE TRUNCATED-UIB DEFECT (runs 132-133): the hardcoded 0x1c copy
				// dropped bytes [$1c..$20+] - including UIB[$20], the class byte the $1718
				// re-stamp gate and the template/flavor selects read ($8c stale residue vs
				// the host's authentic $68 - the gate FLIPS). Honor the IOPB's own count
				// (bytes [$a..$b], like every transfer); sane fallback if unset.
				u32 uibcnt = (u32(cs.read_byte((dst + 0xa) & 0xffff)) << 8) | cs.read_byte((dst + 0xb) & 0xffff);
				// run135: the raw count ($100) over-copies and clobbers the fw-maintained
				// fields past the image ($ca+ pointers destroyed, the door's opening lost).
				// Cap at the image extent until the true DMA length semantics are decoded.
				// cont.50 RESULT: the 0x60-cap experiment DISPROVED "script ships in the host
				// image" - the host bytes at [24..5f] are a pointer table (0fe968+4n), and
				// copying them CLOBBERS the fw's script WORKSPACE at UIB+$24 (flaky boots,
				// op-18's own pour shrank). The $5fc0 builder's [$7938]=UIB+$24 names fw-built
				// workspace; the 0x22 cap is CORRECT and stays. Op $1a's empty pour has a
				// different cause (the part-2 script fill / the $1a-before-$18 order semantics).
				if (uibcnt == 0 || uibcnt > 0x22) uibcnt = 0x22;
				{ std::string uh;
					for (u32 hk = 0; hk < 0x22; hk++) uh += util::string_format(" %02x", bs.read_byte((m_iopb_buffer + hk) & 0xffffff));
					logerror("UIBCOPY unit%u len=%02x dst=%04x src=%06x img:%s @%.5f\n",
						unit, uibcnt, uibcopy_dst & 0xffff, m_iopb_buffer & 0xffffff, uh.c_str(), machine().time().as_double()); }
				for (u32 k = 0; k < uibcnt; k++)
					cs.write_byte((uibcopy_dst + k) & 0xffff, bs.read_byte((m_iopb_buffer + k) & 0xffffff));
				m_unit_heads[unit] = bs.read_byte(m_iopb_buffer & 0xffffff);
				m_unit_spt[unit] = bs.read_byte((m_iopb_buffer + 1) & 0xffffff);
				m_unit_secsize[unit] = u16(bs.read_byte((m_iopb_buffer + 2) & 0xffffff)) | (u16(bs.read_byte((m_iopb_buffer + 3) & 0xffffff)) << 8);
				m_unit_sec0[unit] = bs.read_byte((m_iopb_buffer + 4) & 0xffffff);
				m_unit_btrk[unit] = m_unit_trk[unit];   // latch the INIT-time position = base for addressed reads
				m_unit_bsidx[unit] = m_unit_sidx[unit];
				LOG("INIT unit%u: heads=%u spt=%u secsize=%u sec0=%u base=trk%u.%u @%.4f\n", unit, m_unit_heads[unit], m_unit_spt[unit], m_unit_secsize[unit], m_unit_sec0[unit], m_unit_btrk[unit], m_unit_bsidx[unit], machine().time().as_double());
			if (storager_getenv("STORAGER_NOBYPASS"))   // TEMP (STRIP): Phase-C UIB question - does D000<<1 == the fw UIB struct [$20a+unit*2]?
			{
				u16 const uibp = cs.read_word((0x20a + unit * 2) & 0xffff);   // fw per-unit UIB pointer
				logerror("UIB87 unit%u D000<<1=%04x uibptr[$20a+%u]=%04x uibp[0]=%04x match=%d @%.4f\n",
					unit, dst & 0xffff, unit * 2, uibp, cs.read_word(uibp & 0xffff),
					(dst & 0xffff) == uibp, machine().time().as_double());
			}
				if (storager_getenv("STORAGER_UNITLOG"))   // TEMP (STRIP): the sector size the KERNEL declares for this HD unit (512 vs 1024 = the question)
					logerror("UNITINIT unit%u: heads=%u spt=%u secsize=%u sec0=%u @%.4f\n", unit, m_unit_heads[unit], m_unit_spt[unit], m_unit_secsize[unit], m_unit_sec0[unit], machine().time().as_double());
			}
			// The REAL fix for the re-delivery is the channel-completion signalling below (m_ch_op_ok +
			// IRQ4): without it the firmware never sees "op OK" and re-kicks the command (a late retry that
			// clobbers the buffer).  With it the read completes on the first pass; m_read_pending additionally
			// gates the DATA transfer to once per command so the firmware's per-phase (seek/ID/data) kickoffs
			// don't each re-DMA the whole block.  A real controller streams a read's sectors exactly once.
			else if (m_fw_driven && cs.read_byte(dst) == 0x95 && m_read_pending && !storager_getenv("STORAGER_NOBYPASS"))   // fw-driven; HLE uses read95_deliver
			{                                     // (STORAGER_NOBYPASS=1 disables this HLE delivery leg so the firmware drives its own read->parse->DMA tail — LLE cutover probe)
				if (storager_getenv("STORAGER_PHASELOG")) logerror("SHIM-READ (data-write) cmd=%02x 71b2=%04x @%.5f\n", m_iopb_cmd, m_cpu->space(AS_PROGRAM).read_word(0x71b2), machine().time().as_double());
				// STEP 327: ESDI HD read leg. IOPB[4] = unit (2=floppy, 0/1=ESDI HD). For an HD unit, read
				// the mounted MC1325 image at LBA (IOPB[6-9] BE) x 512, count (IOPB[a-b] BE) x 512 bytes ->
				// the CPUAP host buffer. 512B physical sectors (ESDI Micropolis, 35 spt); the SINIX label is
				// at LBA 0 ("SINIXdisklayout MC1325"). Same deliver-then-done completion as the floppy.
				u32 const runit = cs.read_byte((dst + 4) & 0xffff) & 7;
				logerror("BYPASS entry runit=%u iopb_buf=%06x rp=%d @%.4f\n", runit, m_iopb_buffer & 0xffffff, m_read_pending, machine().time().as_double());   // TEMP (STRIP)
				if (runit < 2 && m_iopb_buffer)
				{
					u32 const lba = (u32(cs.read_byte((dst + 6) & 0xffff)) << 24) | (u32(cs.read_byte((dst + 7) & 0xffff)) << 16)
								  | (u32(cs.read_byte((dst + 8) & 0xffff)) << 8) | cs.read_byte((dst + 9) & 0xffff);
					u32 const count = std::max<u32>(1u, (u32(cs.read_byte((dst + 0xa) & 0xffff)) << 8) | cs.read_byte((dst + 0xb) & 0xffff));
					if (m_hd[runit]->exists() && count <= 512)
					{
						std::vector<u8> sec(size_t(count) * 1024, 0);
						m_hd[runit]->img_read(u64(lba) * 1024, sec.data(), u32(sec.size()));
						for (u32 k = 0; k < u32(sec.size()); k++)
							bs.write_byte((m_iopb_buffer + k) & 0xffffff, sec[k]);
						if (m_iopb_addr)
						{
							if (storager_getenv("STORAGER_NOBYPASS")) logerror("IOPB80-POST L%d @%.5f\n", __LINE__, machine().time().as_double());
							if (storager_getenv("STORAGER_FWDONE")) { logerror("IOPB80-SKIPPED(FWDONE) @%.5f\n", machine().time().as_double()); } else {
							bs.write_byte((m_iopb_addr + 2) & 0xffffff, 0x80); r0_observe(0x80);
							bs.write_byte((m_iopb_addr + 3) & 0xffffff, 0x80);
							}
							if (storager_getenv("STORAGER_FWDONE")) { logerror("FIXEDSLOT80-SKIPPED @%.5f\n", machine().time().as_double()); } else {
							bs.write_byte(0x0fe782, 0x80);
							bs.write_byte(0x0fe783, 0x80);
							}
						}
						// Signal CHANNEL completion the same way the floppy DATAOP path does: set E01E bit4
						// (read-OK, polled by the fw in ch_r) and raise the completion IRQ4.  Without these
						// the firmware's channel wait never sees "op OK", so it RE-KICKS the same command (the
						// retry loop) - a subsequent kickoff then re-delivers and clobbers the buffer.  Posting
						// the host IOPB DONE (above) alone is not enough; the fw waits on the channel status.
						m_ch_op_ok = true;
						{ if (storager_getenv("STORAGER_NOBYPASS")) logerror("IRQ4fire L%d pc=%06x @%.4f\n", __LINE__, m_cpu->pc(), machine().time().as_double()); m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE); }
						m_read_pending = false;
						LOG("HD READ host-DMA: unit%u lba%u x%ublk*1024 -> CPUAP %06x (%u B) first16: %.16s@%.4f\n",
								runit, lba, count, m_iopb_buffer, u32(sec.size()), reinterpret_cast<char *>(sec.data()), machine().time().as_double());
					}
				}
				// READ host-DMA. The 68K has read the sector(s) through the disk channel (the DATAOP
				// staging) and programmed this DMA (IOPB words 6-7 = host dest, count in the IOPB); the
				// gate array now MOVES the data across the Multibus to the CPUAP. Model the gate-array
				// DMA: serve the sought sectors' raw bytes from the mounted floppy (decode_track_ids -
				// the same source the disk channel decoded) into the host buffer, in logical order from
				// the drive's current position. This is find-by-position + gate-array DMA, no protocol.
				floppy_image_device *const fdd = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
				if (runit >= 2 && fdd && m_iopb_buffer)
				{
					if (!m_floppy_loaded) load_floppy();   // a live swap invalidates the map; reload from the newly-mounted image
					int const cyl = fdd->get_cyl();
					int const side = cs.read_word(0x7436) & 1;   // the fw's target head
					// PORTED from the HLE overlay (the proven mapping): the CPUAP reads 512-BYTE LOGICAL
					// BLOCKS; each = sec_per_blk=512/ssz media sectors. Deliver count*sec_per_blk sectors,
					// logical->physical via sec0 (FM trk0: 7=VOL1 first), advancing across tracks. fw-driven
					// doesn't INIT the units, so derive geometry from the decoded track (heads=2).
					// Geometry keyed off the POSITION (physical track layout, VERIFIED from the IMD):
					// htrk 0-1 (cyl0/h0-1) = FM 128B / sec0=7 (VOL1 label); htrk 2+ = MFM 256B / sec0=13
					// (boot loader + kernel). This is authoritative + sidesteps (a) the fw drive-probe
					// re-INIT that clobbers m_unit_secsize, and (b) decode_track_ids reading the MFM ID's
					// N=2 as 512 (the physical sector is 256). The CPUAP's read is uniform per its re-INIT.
					u32 const heads = m_unit_heads[2] ? m_unit_heads[2] : 2u;
					u32 const spt = m_unit_spt[2] ? m_unit_spt[2] : 16u;
					u32 htrk = m_unit_trk[2], hsidx = m_unit_sidx[2];
					u32 const ssz = (htrk < 2) ? 128u : 256u;
					u32 const sec0u = (htrk < 2) ? 7u : 1u;   // FM: R7=VOL1 (logical 0); MFM: R1 (the HLE
					                                          // uses sec0 only for FM; MFM logical 0 = R1)
					u32 const sec_per_blk = std::max<u32>(1u, 512u / ssz);
					u32 const count = (u32(cs.read_byte((dst + 0xa) & 0xffff)) << 8) | cs.read_byte((dst + 0xb) & 0xffff);   // BLOCKS
					if (cs.read_byte((dst + 1) & 0xffff) & 1)   // addressed mode: position = INIT base + addr
					{
						u32 const addr = (u32(cs.read_byte((dst + 6) & 0xffff)) << 24) | (u32(cs.read_byte((dst + 7) & 0xffff)) << 16)
									   | (u32(cs.read_byte((dst + 8) & 0xffff)) << 8) | cs.read_byte((dst + 9) & 0xffff);
						u32 const abs_s = m_unit_btrk[2] * spt + m_unit_bsidx[2] + addr;
						htrk = abs_s / spt; hsidx = abs_s % spt;
					}
					u32 off = 0; u8 firstr = 0;
					for (u32 n = 0; n < count * sec_per_blk && count <= 64; n++)
					{
						u32 const cylno = htrk / heads, head = htrk % heads;
						u32 const secid = ((sec0u - 1 + hsidx) % spt) + 1;
						// serve from m_sectors (the direct IMD sector map from load_floppy) - the SAME source
						// the HLE read95_deliver uses; decode_track_ids' bitstream decode fails on these MFM
						// tracks (falls to FM garbage). The direct map has the correct 128B FM / 256B MFM data.
						auto const it = m_sectors.find((cylno << 16) | (head << 8) | secid);
						if (!n) firstr = u8(secid);
						for (u32 k = 0; k < ssz; k++)
							bs.write_byte((m_iopb_buffer + off + k) & 0xffffff, (it != m_sectors.end() && k < it->second.size()) ? it->second[k] : 0);
						off += ssz;
						if (++hsidx >= spt) { hsidx = 0; htrk++; }
					}
					m_unit_trk[2] = htrk; m_unit_sidx[2] = hsidx;   // advance the running position
					// STEP 324: deliver-THEN-done. The data is now in the host buffer, so NOW post DONE
					// (0x81->0x80) + clear m_read_pending. The CPUAP's read helper (CXP 0x2B) polls
					// IOPB+2 / 0x0fe782 for 0x80 and returns; with the data already written it reads the
					// fresh boot-image magic 0x10B (STEP 323: else it read stale VOL1 1us early).
					if (m_iopb_addr && !(storager_getenv("STORAGER_SHIMTEST") && strchr(storager_getenv("STORAGER_SHIMTEST"), 'b'))) // (b) IOPB/host DONE post
					{
						if (storager_getenv("STORAGER_NOBYPASS")) logerror("IOPB80-POST L%d @%.5f\n", __LINE__, machine().time().as_double());
						if (storager_getenv("STORAGER_FWDONE")) { logerror("IOPB80-SKIPPED(FWDONE) @%.5f\n", machine().time().as_double()); } else {
							bs.write_byte((m_iopb_addr + 2) & 0xffffff, 0x80); r0_observe(0x80);
						bs.write_byte((m_iopb_addr + 3) & 0xffffff, 0x80);
						}
						if (storager_getenv("STORAGER_FWDONE")) { logerror("FIXEDSLOT80-SKIPPED @%.5f\n", machine().time().as_double()); } else {
						bs.write_byte(0x0fe782, 0x80);
						bs.write_byte(0x0fe783, 0x80);
						}
					}
					if (!(storager_getenv("STORAGER_SHIMTEST") && strchr(storager_getenv("STORAGER_SHIMTEST"), 'c'))) m_ch_op_ok = true;   // (c) E01E bit4; channel-op-OK - see the HD read leg: without
					if (!(storager_getenv("STORAGER_SHIMTEST") && strchr(storager_getenv("STORAGER_SHIMTEST"), 'd'))) { if (storager_getenv("STORAGER_NOBYPASS")) logerror("IRQ4fire L%d pc=%06x @%.4f\n", __LINE__, m_cpu->pc(), machine().time().as_double()); m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE); }   // (d) gate-array completion IRQ4
					if (!(storager_getenv("STORAGER_SHIMTEST") && strchr(storager_getenv("STORAGER_SHIMTEST"), 'e'))) m_read_pending = false;   // (e) shim-internal flag
					std::string dd;
					for (u32 k = 0; k < 12 && k < off; k++) dd += util::string_format("%02x ", bs.read_byte((m_iopb_buffer + k) & 0xffffff));
					LOG("READ host-DMA: cyl%d s%d x%ublk*%u ssz%u R%u.. -> CPUAP %06x (%u B) first12: %s@%.4f\n",
							cyl, side, count, sec_per_blk, ssz, firstr, m_iopb_buffer, off, dd.c_str(), machine().time().as_double());
				}
			}
			else if (m_fw_driven && cs.read_byte(dst) == 0x94 && storager_getenv("STORAGER_NOBYPASS"))
			{
				// LLE (Dave): a stale/garbage local IOPB at $71f0 (=D000<<1, the null descriptor) reading
				// 0x94 during a 0x95 READ was misfiring this WRITE leg - throwing the spurious IRQ4 (below)
				// that busied [$749c] and blocked the 0x95 dispatch, and worse img_write'ing to the HD on a
				// stale IOPB (a real corruption hazard).  The fw drives writes itself; suppress the leg here.
				logerror("WRITE-LEG SUPPRESSED (stale $71f0 cmd=94 wunit=%u) pc=%06x @%.5f\n",
						cs.read_byte((dst + 4) & 0xffff) & 7, m_cpu->pc(), machine().time().as_double());
			}
			else if (m_fw_driven && cs.read_byte(dst) == 0x94)   // EXTRAPOLATED floppy WRITE (STEP 326)
			{
				if (storager_getenv("STORAGER_PHASELOG")) logerror("SHIM-WRITE (data-write) cmd=%02x 71b2=%04x @%.5f\n", m_iopb_cmd, m_cpu->space(AS_PROGRAM).read_word(0x71b2), machine().time().as_double());
				// WRITE host-DMA = the MIRROR of the 0x95 READ leg, REVERSED (host buffer -> media).
				// *** UNTESTED: the boot path issues no writes, so the write IOPB cmd (0x94 here) is
				// EXTRAPOLATED, NOT observed - confirm the opcode + option/direction bits against a real
				// write (SINIX install/mkfs on the floppy) when a write scenario is testable. ***
				// Flow (from the read + the SASI 0x0a IOREG write ~line 909): the CPUAP has already placed
				// the source data in the host buffer (IOPB words 6-7); the fw seeks + ID-verifies via the
				// SAME DATAOP path, then programs the gate-array host-DMA in the WRITE direction (gate array
				// reads the host buffer + writes the disk). Model: consume the host bytes into m_sectors (the
				// same map the read serves from). Geometry IDENTICAL to the read (position-keyed).
				// STEP 364: HD-unit (0/1) writes go to the HD IMAGE, mirroring the cmd-0x95 HD read leg
				// (~line 1636) REVERSED (host buffer -> HD). The 0x94 leg was floppy-only (m_sectors), so HD
				// writes were mis-routed to the floppy and never reached the HD - the "broken after format" /
				// "/: file system full" root cause. Same LBA[6..9]/count[a..b] units + deliver-then-DONE/IRQ4.
				u32 const wunit = cs.read_byte((dst + 4) & 0xffff) & 7;
				if (wunit < 2 && m_iopb_buffer)
				{
					u32 const lba = (u32(cs.read_byte((dst + 6) & 0xffff)) << 24) | (u32(cs.read_byte((dst + 7) & 0xffff)) << 16)
								  | (u32(cs.read_byte((dst + 8) & 0xffff)) << 8) | cs.read_byte((dst + 9) & 0xffff);
					u32 const count = std::max<u32>(1u, (u32(cs.read_byte((dst + 0xa) & 0xffff)) << 8) | cs.read_byte((dst + 0xb) & 0xffff));
					if (m_hd[wunit]->exists() && count <= 512)
					{
						std::vector<u8> sec(size_t(count) * 1024, 0);
						for (u32 k = 0; k < u32(sec.size()); k++)
							sec[k] = bs.read_byte((m_iopb_buffer + k) & 0xffffff);   // HOST buffer -> staging
						m_hd[wunit]->img_write(u64(lba) * 1024, sec.data(), u32(sec.size()));   // staging -> HD image
						if (m_iopb_addr)
						{
							if (storager_getenv("STORAGER_NOBYPASS")) logerror("IOPB80-POST L%d @%.5f\n", __LINE__, machine().time().as_double());
							if (storager_getenv("STORAGER_FWDONE")) { logerror("IOPB80-SKIPPED(FWDONE) @%.5f\n", machine().time().as_double()); } else {
							bs.write_byte((m_iopb_addr + 2) & 0xffffff, 0x80); r0_observe(0x80);
							bs.write_byte((m_iopb_addr + 3) & 0xffffff, 0x80);
							}
							if (storager_getenv("STORAGER_FWDONE")) { logerror("FIXEDSLOT80-SKIPPED @%.5f\n", machine().time().as_double()); } else {
							bs.write_byte(0x0fe782, 0x80);
							bs.write_byte(0x0fe783, 0x80);
							}
						}
						m_ch_op_ok = true;
						if (storager_getenv("STORAGER_NOBYPASS")) logerror("IRQ4fire L%d(cmd94wr) pc=%06x @%.4f\n", __LINE__, m_cpu->pc(), machine().time().as_double());
						m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE);
						m_read_pending = false;
						LOG("HD WRITE cmd94 unit%u LBA%06x x%ublk -> HD image @%.4f\n", wunit, lba, count, machine().time().as_double());
					}
				}
				floppy_image_device *const fdd = m_floppy[0] ? m_floppy[0]->get_device() : nullptr;
				if (wunit >= 2 && fdd && m_iopb_buffer)
				{
					int const cyl = fdd->get_cyl();
					int const side = cs.read_word(0x7436) & 1;
					u32 const heads = m_unit_heads[2] ? m_unit_heads[2] : 2u;
					u32 const spt = m_unit_spt[2] ? m_unit_spt[2] : 16u;
					u32 htrk = m_unit_trk[2], hsidx = m_unit_sidx[2];
					u32 const ssz = (htrk < 2) ? 128u : 256u;
					u32 const sec0u = (htrk < 2) ? 7u : 1u;
					u32 const sec_per_blk = std::max<u32>(1u, 512u / ssz);
					u32 const count = (u32(cs.read_byte((dst + 0xa) & 0xffff)) << 8) | cs.read_byte((dst + 0xb) & 0xffff);
					if (cs.read_byte((dst + 1) & 0xffff) & 1)   // addressed mode: position = INIT base + addr
					{
						u32 const addr = (u32(cs.read_byte((dst + 6) & 0xffff)) << 24) | (u32(cs.read_byte((dst + 7) & 0xffff)) << 16)
									   | (u32(cs.read_byte((dst + 8) & 0xffff)) << 8) | cs.read_byte((dst + 9) & 0xffff);
						u32 const abs_s = m_unit_btrk[2] * spt + m_unit_bsidx[2] + addr;
						htrk = abs_s / spt; hsidx = abs_s % spt;
					}
					u32 off = 0; u8 firstr = 0; u32 dropped = 0;
					for (u32 n = 0; n < count * sec_per_blk && count <= 64; n++)
					{
						u32 const cylno = htrk / heads, head = htrk % heads;
						u32 const secid = ((sec0u - 1 + hsidx) % spt) + 1;
						auto it = m_sectors.find((cylno << 16) | (head << 8) | secid);
						if (!n) firstr = u8(secid);
						for (u32 k = 0; k < ssz; k++)
						{
							u8 const b = bs.read_byte((m_iopb_buffer + off + k) & 0xffffff);   // HOST -> DISK
							if (it != m_sectors.end() && k < it->second.size())
								it->second[k] = b;
							else
								dropped++;   // no target sector/size = an INCOMPLETE write (would corrupt the fs)
						}
						off += ssz;
						if (++hsidx >= spt) { hsidx = 0; htrk++; }
					}
					m_unit_trk[2] = htrk; m_unit_sidx[2] = hsidx;
					// completion: data consumed to the media map; post DONE + clear pending (same order-safe
					// pattern as the read). NOTE: m_sectors is the in-SESSION floppy image; like the HLE IOREG
					// write it is NOT flushed to the .imd on exit (miniroot writes are session-local). For
					// durable floppy writes, also write the floppy_image_device here.
					if (m_iopb_addr)
					{
						if (storager_getenv("STORAGER_NOBYPASS")) logerror("IOPB80-POST L%d @%.5f\n", __LINE__, machine().time().as_double());
						if (storager_getenv("STORAGER_FWDONE")) { logerror("IOPB80-SKIPPED(FWDONE) @%.5f\n", machine().time().as_double()); } else {
							bs.write_byte((m_iopb_addr + 2) & 0xffffff, 0x80); r0_observe(0x80);
						bs.write_byte((m_iopb_addr + 3) & 0xffffff, 0x80);
						}
						if (storager_getenv("STORAGER_FWDONE")) { logerror("FIXEDSLOT80-SKIPPED @%.5f\n", machine().time().as_double()); } else {
						bs.write_byte(0x0fe782, 0x80);
						bs.write_byte(0x0fe783, 0x80);
						}
					}
					m_ch_op_ok = true;   // channel-op-OK (E01E bit4) + IRQ4: a WRITE the fw never sees complete
					{ if (storager_getenv("STORAGER_NOBYPASS")) logerror("IRQ4fire L%d pc=%06x @%.4f\n", __LINE__, m_cpu->pc(), machine().time().as_double()); m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE); }   // is retried too -> re-delivery corrupts (the
					                                                // "format failure": retried verify/write stomps the miniroot)
					m_read_pending = false;
					LOG("WRITE host-DMA (EXTRAPOLATED/UNTESTED): cyl%d s%d x%ublk*%u ssz%u R%u.. <- CPUAP %06x (%u B, %u dropped)@%.4f\n",
							cyl, side, count, sec_per_blk, ssz, firstr, m_iopb_buffer, off, dropped, machine().time().as_double());
				}
			}
			// Completion edge: post DONE (0x80) to the IOPB status for non-data commands. For cmd 0x95
			// (READ) DON'T post the CPUAP DONE directly - the fw drives the completion IRQ with its own
			// pacing (gate-array m_ch_op_ok/m_dataop below). Posting it instantly here short-circuited the
			// fw's ~2s processing window so the CPUAP issued read 2 early and never reached read 3
			// (STEP 317). Matches the HLE, which also never posts DONE here for cmd 0x95.
			// cont.255: 0x89/0x98 (RESTORE/SEEK) are excluded too - they have real time-domain
			// execution and their completion is owned by seek_done_tick at the settle deadline.
			// Posting instantly here made the CPUAP ring the next command (0x95) into the still-
			// live work node while the fw was mid-restore: the deposit clobbered the node, cmd 89's
			// teardown wiped [$71bc], and the 0x95 was never dispatched (run256 DCENSUS: no TBLLOOK
			// for 95; fw idles forever). Whether the race was won was RTC-seed dependent = the boot
			// flake. With the seek-class post deferred to the timer, the 0x95 arrives at an idle fw.
			if (m_iopb_addr && m_iopb_cmd != 0x95 && m_iopb_cmd != 0x89 && m_iopb_cmd != 0x98)
			{
				if (storager_getenv("STORAGER_NOBYPASS")) logerror("IOPB80-POST L%d @%.5f\n", __LINE__, machine().time().as_double());
				if (storager_getenv("STORAGER_FWDONE")) { logerror("IOPB80-SKIPPED(FWDONE) @%.5f\n", machine().time().as_double()); } else {
							bs.write_byte((m_iopb_addr + 2) & 0xffffff, 0x80); r0_observe(0x80);
				bs.write_byte((m_iopb_addr + 3) & 0xffffff, 0x80);
				}
				if (storager_getenv("STORAGER_FWDONE")) { logerror("FIXEDSLOT80-SKIPPED @%.5f\n", machine().time().as_double()); } else {
				bs.write_byte(0x0fe782, 0x80);
				bs.write_byte(0x0fe783, 0x80);
				}
			}
			// fw-driven mode: the IRQ4 ISR is the phase sequencer - it builds the descriptor,
			// programs E800 from the +0xA/+0xC masks and the $63E op table, loads D000, and RE-KICKS
			// (fw @0x3D42). That second kick IS the disk operation: perform it with rotational pacing
			// and let its completion IRQ4 drive the ISR's continuation.
			// fw-driven cmd95: every post-fetch kick is a channel op - the gate array performs it
			// (content by the E800 op family) and completes on the level-4 descriptor interrupt.
			// (Level 6 = the transfer-stage pump, reached only after the verify chain concludes.)
			if (m_fw_driven && m_iopb_cmd == 0x95)
			{
				// the fw's op wait window is short (~1.5ms observed) - the baseline completed
				// synchronously at the kick; deliver just-async (100us) to stay inside it
				m_ch_op_ok = false;   // the new op is in flight
				LOG("DISKOP kick E800=%04x D000=%04x -> delivery in 100us @%.4f\n", data, m_d000, machine().time().as_double());
				// TEMP (STRIP): Dave's ID-verify comparison probe at every cmd-0x95 kick
				{
					address_space &xs = m_cpu->space(AS_PROGRAM);
					logerror("IDVFY E800=%04x | want C=%04x H=%04x R=%04x | ID7dac=%02x%02x%02x.%02x C%02x H%02x R%02x N%02x"
							 " | 7950=%04x 79a4=%04x 7a0c=%04x 741c=%04x | req 71bc=%04x 71be=%04x"
							 " | node +12=%04x +14=%04x +18=%04x +1a=%04x +20=%04x | E01E=%04x @%.4f\n",
						data, xs.read_word(0x7438), xs.read_word(0x7436), xs.read_word(0x7428),
						xs.read_byte(0x7dac), xs.read_byte(0x7dad), xs.read_byte(0x7dae), xs.read_byte(0x7daf),
						xs.read_byte(0x7db0), xs.read_byte(0x7db1), xs.read_byte(0x7db2), xs.read_byte(0x7db3),
						xs.read_word(0x7950), xs.read_word(0x79a4), xs.read_word(0x7a0c), xs.read_word(0x741c),
						xs.read_word(0x71bc), xs.read_word(0x71be),
						xs.read_word(0x749c), xs.read_word(0x749e), xs.read_word(0x74a2), xs.read_word(0x74a4), xs.read_word(0x74aa),
						m_ch[(0xe01e - 0xe000) / 2], machine().time().as_double());
				}
				// The old suppression ("these IRQ4s hold [$749c]=1 so 0x95 never dispatches") was
				// MEASURED BACKWARDS (M1, build#5 cont.6): the per-submit CHANCOMPLETE IRQ4 is what
				// CLEARS [$749c] via the $3bfe walker ($3c76), and suppressing it strangled every
				// honest-path completion (the 2.61s retry loop). Under NOBYPASS, fall through to the
				// generic CHANCOMPLETE below - the same per-submit completion the walker consumed for
				// the 87/89s in the measured-working run6 flow.
				if (!storager_getenv("STORAGER_NOBYPASS"))
				{
					m_dataop->adjust(attotime::from_usec(100));
					// Dave #2: latch a deferred gate-array channel completion for the armed descriptor node.
					if (!m_hd_chan_irq4)
					{
						u16 node = m_cpu->space(AS_PROGRAM).read_word(0x743a);
						if (node != 0x748a && node != 0x7442) node = 0x748a;   // fallback to the known verify node
						m_hd_chan_node = node;
						m_hd_chan_pending = true;
						m_hd_chan_tries = 0;
						m_hd_chan->adjust(attotime::from_usec(20));
					}
					return;
				}
			}
			if (m_fw_driven && m_iopb_cmd == 0x95 && m_trace)
			{
				LOG("CHANKICK (fw-driven, no auto-complete) cmd=%02x D000=%04x D800=%04x C800[0]=%04x @%.4f\n",
						m_iopb_cmd, m_d000, m_d800, m_c800[0], machine().time().as_double());
				// TEMP LOG (STRIP): the live descriptor chain - anchor *($743A) + the two static queues
				u16 const anchor = cs.read_word(0x743a);
				std::string dd;
				for (int k = 0; k < 0x20; k += 2) dd += util::string_format(" %04x", cs.read_word((anchor + k) & 0xffff));
				LOG("DESCR anchor=%04x:%s\n", anchor, dd.c_str());
				for (u16 base : { u16(0x7442), u16(0x748a) })
				{
					std::string qd;
					for (int k = 0; k < 0x28; k += 2) qd += util::string_format(" %04x", cs.read_word(base + k));
					LOG("DESCR q%04x:%s\n", base, qd.c_str());
				}
				LOG("DESCR e000file: %04x %04x %04x %04x  e800=%04x e802=%04x e804=%04x\n",
						m_ch[0], m_ch[1], m_ch[2], m_ch[3],
						m_ch[(0xe800 - 0xe000) / 2], m_ch[(0xe802 - 0xe000) / 2], m_ch[(0xe804 - 0xe000) / 2]);
				std::string wb;
				for (u16 k = 0x7430; k < 0x7460; k += 2) wb += util::string_format(" %04x", cs.read_word(k));
				LOG("DESCR work7430:%s\n", wb.c_str());
			}
			if (m_trace) LOG("CHANCOMPLETE -> IRQ4 (D000 byte %06x)\n", u32(m_d000) << 1);
			if (storager_getenv("STORAGER_NOBYPASS"))   // NODE-STATE DUMP (Dave): what $3bfe walks - the field it finds not-done
			{
				address_space &xs = m_cpu->space(AS_PROGRAM);
				u16 const node = xs.read_word(0x743a);
				logerror("NODEWALK cmd=%02x anchor743a=%04x node[+12]=%04x [+14link]=%04x [+1a]=%04x [+26]=%04x | 7a14=%04x 7a76=%04x 7928=%04x 71b2=%04x 7b1a=%04x @%.5f\n",
						m_iopb_cmd, node,
						xs.read_word((node + 0x12) & 0xffff), xs.read_word((node + 0x14) & 0xffff),
						xs.read_word((node + 0x1a) & 0xffff), xs.read_word((node + 0x26) & 0xffff),
						xs.read_word(0x7a14), xs.read_word(0x7a76), xs.read_word(0x7928),
						xs.read_word(0x71b2), xs.read_word(0x7b1a), machine().time().as_double());
			}
			// CHANCOMPLETE IRQ4.  (Deferring this via m_chancomplete was REFUTED 2026-07-12: delaying it just
			// scales the fw's whole channel loop - dispatches+flood move together, 0x95-clean stays 0 at every
			// delay.  The fw loop is deterministically stamp-then-dispatch, so pacing can't reorder it; the read
			// working on real HW means the $3d4a stamp must not hit the CCB there = a D000/CCB model-address
			// artifact, branch 2.  m_chancomplete kept but unused.)
			// cont.87: defer THIS CHANCOMPLETE past the main line's dispatcher pass. Run226 caught
			// the park mechanism live: the synchronous IRQ4 (raised mid-ISR at the $3d4a kick)
			// pends, re-enters at RTE, and its worker pass consumes the node queue 15us BEFORE the
			// main line's $2290 dispatcher read - which then runs with both heads null and a stale
			// D0 -> table $222 null entry -> jsr 0 -> the $24a park at 8.1844, every run since
			// <=215. A real channel op takes real time (an ID verify waits for a mark - us..
			// 12.5ms), so on hardware the dispatcher ALWAYS reads the node first. 300us = after
			// the main line's pass (+24us observed), inside the fw's ~1.5ms op wait window. (The
			// 2026-07-12 "pacing can't reorder" refutation addressed stamp-then-dispatch INSIDE
			// the fw loop, not this ISR-vs-mainline interleave.)
			if (storager_getenv("STORAGER_NOBYPASS"))
			{
				// cont.88: complete the channel op on the ROTATIONAL CLOCK - the next id-mark
				// passage is an ID-verify's true latency (run227: any wall-clock guess is wrong;
				// the fw's dispatcher is built on the op staying queued while genuinely in
				// flight). Fallback 300us when no stream exists.
				attotime delay = attotime::from_usec(300);
				// cont.210 (STORAGER_DMAFAST): a kick whose staged C000 counter targets the host
				// IOPB (completion write-back) or the UIB (mailbox fetch) is a MULTIBUS DMA, not
				// a rotational channel op - it completes in bus time. The rotational schedule
				// (2824us) left the op pending long enough for the next doorbell to slip in;
				// the late IRQ4 then collided with the queued-command state ($1D66 branch ->
				// phase-0 dispatch -> the $222 table's null entry -> jsr into data = the 5th-cmd
				// death-spin). The 300us floor stays (the documented ISR-vs-mainline interleave).
				logerror("IRQ4defer L%d pc=%06x pend=%d delay=%.0fus @%.4f\n", __LINE__, m_cpu->pc(), m_chan_pending + 1, delay.as_double() * 1e6, machine().time().as_double());
				if (++m_chan_pending == 1)
					m_chancomplete->adjust(delay);
			}
			else
				m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE);
		}
		// (the old EPROM->RAM shadow-load is gone: the worker code is fetched directly from ROM via the
		// AS_OPCODES program/data split; RAM at 0x4000-0x7fff is data-only, so it stays uncorrupted.)
	}

	// E802 = interrupt control.  bit6 = the IRQ2 ack the handler toggles on entry.  bit7 (set only at
	// command completion, e.g. 0x00c1 @pc 0x18ec) = assert the host completion interrupt to the CPUAP.
	if (a == 0xe802)
	{
		// cont.117 (STRIP): ARM-TYPE - every bit15 RISING edge with the prime state that
		// types the capture (m_idcap_id_typed: true = $22f-primed ID; false = bare/DATA)
		// and the [$742c] gate. Windows-from-zero. Read1's launch arms vs read2's four
		// armed-and-starved cycles.
		{ static int b15a = -1, na = 0; double const ta = machine().time().as_double();
			int const now15a = BIT(data, 15);
			if (now15a && !b15a && na < 60 && ta < 8.30)
			{ na++;
				logerror("ARMTYPE idtyp=%d 742c=%04x pc=%06x @%.6f\n",
					m_idcap_id_typed ? 1 : 0, m_cpu->space(AS_PROGRAM).read_word(0x742c),
					m_cpu->pc(), ta); }
			b15a = now15a; }
		// cont.97 (STRIP): THE DISARM SHAPE (Dave: read this fork first) - every bit15
		// TRANSITION on E802 in read1's close era vs read2's fe era. Sustained-low ->
		// the close path ran (candidate a: a check bounced it downstream); dip-and-re-arm
		// only -> the sustained disarm never fires for read2 (candidate b: model owes the
		// batch-end event's trigger... or the fw never commands it - pc says which).
		{ static int b15 = -1; static int n = 0; double const t = machine().time().as_double();
			int const now15 = BIT(data, 15);
			if (now15 != b15 && n < 60
					&& ((t > 7.955 && t < 7.985) || (t > 8.045 && t < 8.085)))
			{ n++;
				logerror("B15EDGE %d->%d (E802<-%04x) pc=%06x @%.6f\n", b15, now15, data, m_cpu->pc(), t); }
			b15 = now15; }
		// task#5: what does $a118 (the read's dispatch step-wait loop) write to E802? which bit is the step?
		if (storager_getenv("STORAGER_A118") && m_cpu->pc() >= 0xa140 && m_cpu->pc() <= 0xa1b6)
		{ static int n = 0; if (n++ < 40) {
			int cyl[4]; for (int u=0;u<4;u++){ auto *f=m_floppy[u]?m_floppy[u]->get_device():nullptr; cyl[u]=f?f->get_cyl():-1; }
			logerror("A118-E802wr=%04x b0=%d b1=%d b6=%d | E804(select)=%04x | floppy cyl[0..3]=%d/%d/%d/%d pc=%06x cmd=%02x @%.6f\n",
			data, BIT(data,0), BIT(data,1), BIT(data,6), m_ch[(0xe804-0xe000)/2], cyl[0],cyl[1],cyl[2],cyl[3], m_cpu->pc(), m_iopb_cmd, machine().time().as_double()); } }
		// cont.44 v2: the capture channel's op-frame end = SUSTAINED bit15 disarm. The fw's
		// re-arm dips ($92ce andi/ori) last microseconds; the batch-close mask ($8a48 andi
		// #$67ff) holds low for ~0.4s. Falling edge arms a 1ms confirm; a re-rise cancels;
		// firing posts the channel completion (capdone_tick).
		if (storager_getenv("STORAGER_CAPCH0"))
		{
			// v3: run171 - the raw falling-edge trigger fired on the BOOT SELF-TEST's bit15
			// twiddle (0.41) and wrecked the whole boot (permanent kick-retry storm, blank
			// screen). A real capture op FRAME holds bit15 high for a long stretch (collection:
			// ~1.3s); self-test twiddles and the hunt's re-arm dips are us..ms. Qualify the
			// fall: only a frame held HIGH >= 50ms posts on its end.
			// v4: the 50ms high-hold qualifier (v3) never matched - the fw's ~12.5ms re-arm
			// dips reset the rise clock. Back to v2's dip-tolerant shape (1ms LOW-confirm,
			// canceled by any re-rise), gated on a live command (m_iopb_cmd != 0 excludes the
			// boot self-test twiddle that wrecked run171).
			bool const b15 = BIT(data, 15), b15p = BIT(m_e802_prev_c44, 15);
			if (m_iopb_cmd != 0)
			{
				if (b15p && !b15) { m_capdone_phase = 0; m_capdone->adjust(attotime::from_msec(1)); }
				else if (!b15p && b15 && m_capdone_phase == 0) m_capdone->adjust(attotime::never);
			}
			m_e802_prev_c44 = data;
		}
		// cont.43a: E802 bit6 = the IRQ2 ack/mask LEVEL. The fw's handler acks with a bit6 LOW
		// pulse ($24ae: andi #$ffbf then ori #$40); ordinary channel writes keep bit6 high. The
		// old unconditional CLEAR_LINE on every E802 write let the hunt loop (several writes/ms)
		// eat any pending IRQ2 before the CPU's ipl window - runs 166-168's probes proved the
		// line was never taken. Clear only while bit6 is written LOW (the ack).
		if (!BIT(data, 6))
			m_cpu->set_input_line(M68K_IRQ_2, CLEAR_LINE);
		// cont.54 THE DESCRIPTOR ENGINE (the truck's real face; correctly placed in the E802
		// path this time - run191's nesting bug made it dead code). The gate array as consumer
		// of the staged channel-program block at $7442+ (forced by two proofs: reader-tap cold
		// + hollow drain). GO = an E802 control write carrying the descriptor's own $2010 word
		// while the block is staged. On GO: full read-back + candidate source dumps
		// (self-diagnosing), then execute the block's letter.
		if (storager_getenv("STORAGER_NOBYPASS"))
		{
			// cont.67: the one-shot latch RETIRED (it structurally couldn't serve chained
			// reads - read2 never re-armed). Arm whenever a staged descriptor exists and no
			// transfer is active: per-staging re-arm.
			address_space &es = m_cpu->space(AS_PROGRAM);
			u16 const typ = es.read_word(0x7444);
			bool const staged = typ == 1 && es.read_word(0x7446) != 0;
			bool desc_done = s_desc.active;   // re-arm gate = no active transfer
			// cont.241e (REC512): the C000 counter ADVANCES across a command's records;
			// the staged descriptor is consumed ONCE per fresh staging (the per-record
			// re-consumptions were resetting the counter to the command base - every
			// record landed at +0, THE wrong-place error). Dedupe by the staged pair.
			u32 const stagepair = (u32(es.read_word(0x7446)) << 16) | es.read_word(0x7448);
			if (storager_getenv("STORAGER_REC512"))
				desc_done = s_desc.active || (stagepair == m_desc_lastpair);
			// cont.66 (STRIP): trace-first for cmd#2 - log EVERY eligible staging (the one-shot
			// desc_done latch is itself a found bug: the engine can never re-arm for read2).
			{ static int _gn = 0; static u16 _lastlo = 0;
				u16 const _lo = es.read_word(0x7448);
				if (staged && (data & 0x2010) == 0x2010 && _lo != _lastlo && _gn++ < 10)
				{ _lastlo = _lo;
					logerror("DESCGO-ELIG idx=%04x ~lo=%04x latched=%d @%.5f\n",
						es.read_word(0x7446), _lo, desc_done ? 1 : 0, machine().time().as_double()); } }
			if (staged && !desc_done && (data & 0x2010) == 0x2010)
			{
				u16 const hdr = es.read_word(0x7442);
				u16 const idx = es.read_word(0x7446), nlo = es.read_word(0x7448);
				u16 const rD000 = es.read_word(0x744c), cnt = es.read_word(0x7450);
				u32 const host = (u32(~(idx >> 1) & 0xff) << 16) | (u16(~nlo) & 0xffff);
				u32 const src = u32(rD000) << 1;
				std::string d1, d2, d3;
				for (int k = 0; k < 8; k++) { d1 += util::string_format(" %02x", es.read_byte((src + k) & 0xffff));
					d2 += util::string_format(" %02x", es.read_byte(0x78c4 + k)); d3 += util::string_format(" %02x", es.read_byte(0x7dac + k)); }
				logerror("DESCGO hdr=%04x e802=%04x idx=%04x ~lo=%04x -> host=%06x srcD000=%04x cnt=%04x | src:%s | 78c4:%s | 7dac:%s @%.5f\n",
					hdr, data, idx, nlo, host, src & 0xffff, cnt, d1.c_str(), d2.c_str(), d3.c_str(), machine().time().as_double());
				// cont.55 v2: the GO ARMS the pipeline only (v1's bulk copy at staging time
				// delivered a punctual kilobyte of zeros). cnt = [$7450] = logical 512-byte
				// blocks (the builder's =2 -> 1024 bytes = the CPUAP's 8 FM sectors).
				if (host >= 0x080000 && host < 0x100000)
				{
					// cont.190 (C000 decode): the staged pair at $7446/$7448 IS the host-address
					// counter preset in memory-resident form ([$79D8] = {(~B>>15)&$1FE, ~B&$FFFF},
					// spec §3.4/§6.4).  Consuming the block, the gate array loads its C000 counter
					// from it - the descriptor-fetch flavor of the same preset the $3CD4 launches
					// write to $C000 directly.  The carry below runs on the modeled counter; the
					// private s_desc.host shadow is retired to a snapshot of it.
					m_c000 = host;
					m_c000_valid = true;
					m_desc_lastpair = stagepair;   // cont.241e: consumed - do not re-consume this staging
					m_rec_c000 = host;             // cont.241g: the record stream's counter
					s_desc.active = true; s_desc.host = m_c000; s_desc.done = 0;
					// cont.256e (Dave): the transfer SIZE must follow the COMMAND'S ASK, not a
					// constant. [$7450]'s cnt is 2 for every launch, so total was always
					// 2*512 = 1024 - correct for read1 (IOPB count 8 x 128B = 1024, which is
					// why read1 alone ever completed) and DOUBLE for the count-4 label read
					// (4 x 128 = 512). The model was carrying 8 sectors where the firmware
					// asked for 4, running the delivery past the ask and signalling DESCDONE
					// at the wrong sector. [$7ABC] is the firmware's own ask-count cell
					// (W7ABC, run275: <-0008 at read1, <-0004 at the count-4, written by op-58
					// $7356 before op-4A copies it to [$7956]); the unit is UIB bytes/sector.
					// cont.256f: the IOPB count is in 512-BYTE BLOCKS, not sectors - the HLE
					// is explicit (`sec_per_blk = 512 / ssz; n < count * sec_per_blk`, and
					// "the label read (8 blks = 32 FM sectors = exactly 2 tracks)"). So the
					// first read is 8 x 512 = 4096 bytes, and the label re-read 4 x 512 =
					// 2048 (one whole FM track). cont.256e's `ask x sector-size` was still
					// a quarter of the truth; [$7450]'s constant 2 was an eighth.
					// cont.260 (Dave, MEDIA-VERIFIED): the count is in SECTORS, not 512-byte blocks.
					// Media: this label read data is ENTIRELY on cyl0 head0 (R7=VOL1SINIX0, R8=HDR1 NSC
					// Boot; R1-6/R9-16 = 00); cyl0 head1 is UNFORMATTED - all 16 sectors 0xe5. There is
					// NO second track of data. The fw [$7956]=8 decrements per SECTOR, completes at 8 =
					// logical 0-7 on head 0 (VOL1+HDR1). cont.256f x512 (32 sectors/2 tracks) mistook the
					// HLE harmless over-delivery of the blank head 1 for the ask; sizing to 4096 made
					// m_read_hostmap wait for 32 sectors that don't exist, so DESCDONE never fired and the
					// fw (done at 7968=1) hung for a channel completion. total = ask * live sector size.
					u16 const askn = es.read_word(0x7abc);
					u32 const dssz = m_unit_secsize[2] ? m_unit_secsize[2] : 128;
					s_desc.total = askn ? u32(askn) * dssz : u32(cnt ? cnt : 2) * dssz;
					logerror("DESCARM-SIZE ask7abc=%u blocks -> total=%u bytes (%u sectors of %u) @%.5f\n",
						askn, s_desc.total, s_desc.total / (m_unit_secsize[2] ? m_unit_secsize[2] : 128),
						m_unit_secsize[2] ? m_unit_secsize[2] : 128, machine().time().as_double());
					s_desc.vprev = 0;
					s_desc.base = 0; s_desc.dmap = 0;
					// cont.256h: latch the read's STARTING TRACK + geometry. The transfer
					// walks forward from here (HLE: `if (++sidx >= spt) { sidx = 0; trk++; }`),
					// so a sector's host offset is ((cur_trk - base_trk) * spt + logical) * ssz.
					// The count-8 label read crosses cyl0h0 -> cyl0h1 by a HEAD SWITCH (the fw
					// drives it via [$7436]); both tracks are FM, so ssz/sec0 hold across it.
					m_depot_r.clear();   // cont.256w: deposits are per-transfer
					for (auto &d : m_aim_deposited) d = false;   // cont.257i: per-window
					m_read_hostmap = 0;   // cont.257t: per-window position-delivery coverage
					m_bulk_rearmed = false;   // cont.311
					m_win_buf.clear();   // cont.281 FAITHXFER: drop the prior read's window buffer
					s_desc.base_trk = m_flux_track;
					s_desc.ssz = m_unit_secsize[2] ? m_unit_secsize[2] : 128;
					s_desc.sec0 = (s_desc.ssz != 128) ? 1 : (m_unit_sec0[2] ? m_unit_sec0[2] : 7);
					s_desc.spt = 16;
					s_desc.verified = false;
					// cont.67: window-relative logical slots - the fw's window base [$7954]
					// (position of this staging's first sector) latched per ARM; read2's
					// window starts at position 9, so its R15 = slot 0.
					s_desc.winbase = es.read_word(0x7954);
					m_rec_r0 = es.read_byte(0x7daf);   // cont.240 (REC512): record base = template R at launch
					for (int k = 0; k < 16; k++) s_desc.aims[k] = 0;
					// TEMP cont.255d (STRIP): live-aim candidates at ARM time - [$7428] (aim),
					// first-FF ledger position, continuation flag [$79E2]. read2's stale
					// [$7954]=1 re-delivers read1's window; one of these must hold the live 9.
					{
						std::string lg;
						int firstff = 0;
						for (int k = 1; k <= 16; k++)
						{
							u8 const lb = es.read_byte(0x7654 + k);
							lg += util::string_format(" %02x", lb);
							if (!firstff && lb == 0xff) firstff = k;
						}
						logerror("DESCARM-LIVE 7428=%04x 7430=%04x 79e2=%04x firstFF=%d ledger:%s @%.5f\n",
							es.read_word(0x7428), es.read_word(0x7430), es.read_word(0x79e2), firstff, lg.c_str(), machine().time().as_double());
						// cont.255m: the CHUNK-NODE state at each re-arm - the $80C0 pop scans
						// exactly the [$742A]- and [$7424]-indexed $74C4 8-byte nodes (status
						// at +2: 0x40/0x80 = collectible). Plus the $808A dispatcher's gates.
						{
							std::string nd;
							for (u16 cell : { es.read_word(0x742a), es.read_word(0x7424) })
							{
								nd += util::string_format(" | idx=%04x", cell);
								if (cell <= 0x40)
								{
									u16 const nb = u16(0x74c4 + cell * 8);
									nd += " [";
									for (int k = 0; k < 8; k += 2) nd += util::string_format(" %04x", es.read_word((nb + k) & 0xffff));
									nd += " ]";
								}
							}
							logerror("NODESTATE%s | 79b8=%04x 79b6=%04x 79a8=%04x 79b0=%04x 7968=%04x 741e=%04x | q b4=%04x b6=%04x b8=%04x c0=%04x 796c=%04x @%.5f\n",
								nd.c_str(), es.read_word(0x79b8), es.read_word(0x79b6), es.read_word(0x79a8),
								es.read_word(0x79b0), es.read_word(0x7968), es.read_word(0x741e),
								es.read_word(0x74b4), es.read_word(0x74b6), es.read_word(0x74b8),
								es.read_word(0x74c0), es.read_word(0x796c), machine().time().as_double());
						logerror("PWSTATE 7a30=%04x 7a64=%04x 7462=%04x 7454=%04x 7442=%04x 7958=%08x @%.5f\n",
							es.read_word(0x7a30), es.read_word(0x7a64), es.read_word(0x7462),
							es.read_word(0x7454), es.read_word(0x7442),
							(u32(es.read_word(0x7958)) << 16) | es.read_word(0x795a), machine().time().as_double());
						}
					}
					logerror("DESCARM-WIN winbase=%04x 7abc=%04x @%.5f\n", s_desc.winbase, es.read_word(0x7abc), machine().time().as_double());
					// cont.76: the cont.69 RE-CARRY RETIRED. Built for the dead served-from-cache
					// theory, its instant DONE-at-arm short-circuited every continue-read before one
					// sector could rotate under the head (run213: reads 2/3 built full windows and
					// staked wants - they just never got the 100-200ms to collect). The continue
					// reads COLLECT like any read; consumption is the protocol.
					logerror("DESCARM host=%06x total=%u @%.5f\n", host, s_desc.total, machine().time().as_double());
				}
			}
		}
		bool const prev_step = m_e802_step;
		// task#5: the "fw step = bit6" was a MISREAD - $24ae is the IRQ2-handler ACK (andi #ffbf/ori #40 = clear/set
		// bit6, one-time at $24aa entry), NOT a step loop.  The genuine E802 step is bit0: $a118 pulses it 16x and
		// waits for f000 bit1 (seek-active) per pulse (a recalibrate).  Stepping on bit0 routes $a118's pulses through
		// the settle one-shot (write_gate1) so f000 bit1 goes SET-then-CLEAR per step, satisfying $a118.
		unsigned const stepbit = (m_fw_driven && !storager_getenv("STORAGER_STEPBIT0")) ? 6 : 0;
		m_e802_step = BIT(data, stepbit);
		if (floppy_image_device *const fdd = m_floppy[0]->get_device())
		{
			// transfer ARM: bit15 rising while a fw-driven READ is active = the gate-array channel
			// op starts (the fw's transfer stage arms after installing the level-6 pump); deliver
			// the target sector one sector-time later and interrupt on level 6
			if (m_fw_driven && m_iopb_cmd == 0x95 && BIT(data, 15) && !BIT(m_e802_prev, 15))
			{
				if (storager_getenv("STORAGER_PHASELOG")) logerror("PHASE E802-bit15  D800<<1=%04x data=%04x pc=%06x @%.5f\n", u32(m_d800) << 1, data, m_cpu->pc(), machine().time().as_double());
				if (storager_getenv("STORAGER_NOBYPASS"))
				{
					// TWO ID CHANNELS, selected by the fw's own UIB flag (build#5 cont.11, measured):
					// bit1 of [[$799a]]+$12 (ROM constant per unit TYPE). SET (HDs, 0x06) = the $7dac
					// ID-CAPTURE channel: stage the record at D800<<1, IRQ6, the $89f2 counted lock-on.
					// CLEAR (floppies, 0x44) = the E000/SERDES STREAM channel: the fw's $8924 loop pulls
					// raw track bytes from E000 and parses them itself ($9884) - measured run37: the pull
					// loop runs dry (serdes=0) while idcap fed the wrong channel. Arm the stream instead.
					address_space &xs = m_cpu->space(AS_PROGRAM);
					u16 const uib = xs.read_word(0x799a);
					bool const hd_chan = BIT(xs.read_byte((uib + 0x12) & 0xffff), 1);
					if (hd_chan)
					{
						// HD/ESDI ID-capture channel (the run6-validated path)
						m_idcap_dst = u32(m_d800) << 1;
						LOG("LLE READ arm (E802 bit15, HD idcap) D800<<1=%04x (pc=%06x) @%.4f\n", m_idcap_dst, m_cpu->pc(), machine().time().as_double());
						m_idcap->adjust(attotime::from_usec(30));
					}
					else
					{
						// floppy SERDES channel (cont.21, the coherent wiring): build the track's
						// stream + marks, open the E000 window, and start the rotational event tick
						// - IRQ3 at the index, IRQ6 at each ID record, IRQ5 at each data record, all
						// positions on the one clock that also serves the E000 bytes. The fw's own
						// builder-installed handlers ($55d8/$299a/$29ce-family) consume them.
						// cont.260 (Dave: the head switch is NOT special - handle aim 16->17
						// exactly like aim 3->4; each sector read is independent): do NOT rebuild
						// the serdes stream for the head crossing. Both tracks have the identical
						// 16-sector layout, so the stream only supplies MARK TIMING; the sector's
						// data and ID come by random access from aim_to_target (deposit + ID
						// staging), which already carry the aim's head. Rebuilding mid-read (the
						// cont.258 attempt) disrupted the pump and stalled track 1. The stream
						// tracks only the PHYSICAL cylinder/head (rebuilt on a genuine seek).
						// cont.256x (Dave: "it should not need two complete passes - that smells
						// like a retry"): the capture engine LOCKS ONTO AN ID MARK and captures
						// the data field that follows. A sector already in progress when the arm
						// occurs cannot be captured - its ID has gone by. m_seen_id was only ever
						// cleared on a fresh track build, so it stayed true forever and the first
						// data-end mark after an arm was honoured as a completion. Measured
						// (run302): the arm landed mid-sector at stream pos 2018, the next data
						// mark was reported as a capture, and the fw's parser->collect chain
						// converted position 1 to C0 and advanced to 2 WITHOUT ANY DATA (W7428
						// <-0001 pc=7e0e @7.97601 then <-0002 pc=8114 @7.97634, no capture
						// between) - so VOL1 was never captured, the read could not be satisfied,
						// and the fw retried the whole track. Require a fresh ID lock per arm.
						// cont.256y REVERTED (run304, no effect): clearing m_want_ready /
						// m_seg_retired at the arm changed nothing - the premature parser+collect
						// still ran at the same instants. The stale-readiness hypothesis is dead,
						// and both flags are KEEPERs from earlier sessions, so they stay as they
						// were. The offending ISR was already IN FLIGHT before the arm, which no
						// arm-time state change can retract.
							m_seen_id = false;
							m_serdes_active = true;
							// On the arm, (re)start the 74LS1812 live-run over the real flux at the current spindle
							// time and the medium's density. It then runs continuously (E000 reads + the mark timer
							// advance it); the fw's per-sector re-arms don't restart it - the disk just keeps spinning.
							if (floppy_image_device *const rfdd = m_floppy[0] ? m_floppy[0]->get_device() : nullptr)
							{
								int const rside = m_cpu->space(AS_PROGRAM).read_word(0x7436) & 1;
								u32 const rkey = (u32(rfdd->get_cyl()) << 1) | unsigned(rside);
								if (m_flux_fdd != rfdd || rkey != m_flux_track)   // start/restart once per track
								{
									rfdd->mon_w(0); rfdd->ss_w(rside);
									m_flux_track = rkey;
									flux_read_reset(rfdd, decode_track_ids(rfdd, rfdd->get_cyl(), rside).fm);
									m_pump->adjust(attotime::from_usec(200));   // run the mark clock so marks bootstrap the fw
								}
							}
						// cont.256c: the DENSITY question - UIB+$12 bit2 gates $9884's A1-fold
						// (clear -> $9934, the FM parse) and bit1 gates $89F2's. Our ID staging
						// writes A1 A1 A1 FE unconditionally while DATA staging branches on
						// trk.fm - if bit2 is CLEAR here the ID framing is wrong for this track.
						{ static int _uz = 0; if (_uz++ < 24)
							{ u16 const uibb = xs.read_word(0x799a);
							logerror("DENSITY uib=%04x +11=%02x +12=%02x (bit1=%d bit2=%d) secsize=%u @%.5f\n",
								uibb, xs.read_byte((uibb + 0x11) & 0xffff), xs.read_byte((uibb + 0x12) & 0xffff),
								BIT(xs.read_byte((uibb + 0x12) & 0xffff), 1), BIT(xs.read_byte((uibb + 0x12) & 0xffff), 2),
								m_unit_secsize[2], machine().time().as_double()); } }
						// cont.255y (THE FIX): "every post-fetch kick is a channel op - it
						// completes on the level-4 descriptor interrupt" - INCLUDING the E802
						// bit15 capture ARMS. The model completed only bit12 kicks; after the
						// last one (7.9888) the $3BFE worker loop starved, so the $3FE0
						// partial-window leg's $400C one-shot re-arm never ran and the $3DC0
						// tick machinery died disarmed (the count-4 deadlock, cont.255t-y).
						// Complete the arm on the rotational clock (next data mark) like every
						// other channel op; the fw's own worker then cycles per-op, re-arming
						// the tick each pass exactly as the partial-window design expects.
						if (m_iopb_cmd == 0x95)
						{
							attotime adelay = attotime::from_usec(300);
							logerror("IRQ4defer-ARM pc=%06x pend=%d delay=%.0fus @%.5f\n",
								m_cpu->pc(), m_chan_pending + 1, adelay.as_double() * 1e6, machine().time().as_double());
							if (++m_chan_pending == 1)
								m_chancomplete->adjust(adelay);
						}
						// cont.38x (STRIP): the ARM TUPLE (Dave's differential) - {[$7a16],[$7a18],
						// table[$10+[$7a18]], E802 low byte} per arm; the field differing between
						// the ID arm and the post-accept data arm IS the selector.
						{ static int _tn = 0, _tm = 0; double const tt = machine().time().as_double();
							if (tt <= 8.2 ? _tn++ < 20 : (tt > 9.79 && _tm++ < 20))
							{ u16 const i16 = xs.read_word(0x7a16), i18 = xs.read_word(0x7a18);
								logerror("ARMTUPLE 7a16=%04x 7a18=%04x tbl[%02x]=%02x e802lo=%02x pc=%06x @%.5f\n",
									i16, i18, (0x10 + i18) & 0xff, xs.read_byte((0x63e + 0x10 + i18) & 0xffff),
									data & 0xff, m_cpu->pc(), tt); } }
						// cont.37 (replaces cont.31's instant stage): a held bit15 = capture PENDING;
						// the record lands at the NEXT ID mark (pump_tick case 6 stages it at the
						// fw-programmed D800 dst, posts E01E bit4, then IRQ6 per the [$79f8] enable) -
						// rotational latency the fw's own timeout machinery expects.
						m_idcap_dst = u32(m_d800) << 1;
						// cont.117 v2 (run255): latch type at EVERY arm write - the fw's
						// dip-and-re-arm cycle (bare rise -> $22f prime -> second rise) is a
						// deliberate re-arm with the new type; the operative capture is the
						// last-armed one. (v1's fresh-arm-only guard starved the hunt: the
						// $89b6 ID re-arm couldn't retype, captures completed data-class, the
						// walk got no ID records.) The semantic fix stands: a PRIME ALONE
						// (no arm write) still cannot retype an in-flight capture.
						m_idcap_armed_id = m_idcap_id_typed;
						m_cmd_armed = true;   // cont.189: the fw's own arm = the delivery gate opens
						m_idcap_pending = true;
						// cont.226 (STORAGER_JIT v5): THE HUNT-LOOP ARM IS THE TRIGGER - this
						// flow never runs the $97xx arm ([$7A68] untouched); the fw re-arms
						// here ($891A/$89B6) per aim. Warp so the aimed sector is next; the
						// >400-byte dead-time guard leaves sequential aims on natural rotation
						// (their fields are in flight; distant aims have nothing pending).
					}
				}
				else
				{
					LOG("DATAARM E802 %04x (pc=%06x) -> delivery in 4ms @%.4f\n", data, m_cpu->pc(), machine().time().as_double());
					m_dataop->adjust(attotime::from_msec(4));
				}
			}
			// cont.38n: the cont.37 bit15-fall CANCEL is RETIRED - the fw drops bit15
			// INCIDENTALLY in its routine $67ff/$77ff enable juggles ($7cca after an accept,
			// $7f84 in the data flow) and the cancel was killing legitimately-armed DATA
			// captures mid-flight (run86 CAPCANCEL type=DATA = the murder on camera).
			// Nothing disarms a pending capture but its completion.
			// cont.38 instrument (STRIP): every bit15 EDGE with pc in the retry era - names
			// who cancels the pending capture before the next id mark (idcap=0 at every mark).
			if (storager_getenv("STORAGER_NOBYPASS") && BIT(data, 15) != BIT(m_e802_prev, 15))
			{ static int _n = 0; double const t = machine().time().as_double();
				if (t > 7.995 && _n++ < 60)
					logerror("B15EDGE %s data=%04x pending=%d pc=%06x @%.5f\n",
						BIT(data, 15) ? "RISE" : "FALL", data, m_idcap_pending, m_cpu->pc(), t); }
			m_e802_prev = data;
			// E802 bit13 = step DIRECTION (fw @0x1066/0x1088: bit13 SET pairs with the +1 position
			// delta = inward/higher cylinders, CLEAR with -1 = outward; MAME dir_w: 0 = increment).
			// B0-calibrated 2026-07-02: the RESTORE full-stroke (bit13 clear) must move toward cyl 0.
			fdd->dir_w(BIT(data, 13) ? 0 : 1);
			fdd->stp_w(BIT(data, stepbit));   // MAME steps on the 1->0 edge; dir_w=1 -> toward track 0
			m_pit[0]->write_gate1(BIT(data, stepbit));   // STEP retriggers the settle one-shot (pit0 ch1 mode 5)
			if (!prev_step && BIT(data, stepbit)) m_seek_deadline = machine().time() + attotime::from_msec(50);   // STEP edge -> 40ms track-to-track + 10ms settle (datasheet)
			// task#5: drive's per-step seek-active. On the STEP edge assert seek-active (f000 bit1 SET via
			// m_settle_out=false), clear after a short settle - so $a118's wait-SET-then-CLEAR per step is met.
			if (storager_getenv("STORAGER_SEEKACTIVE") && !prev_step && BIT(data, stepbit))
			{
				m_settle_out = false;   // f000 bit1 SET = seek-active
				m_settle_clear->adjust(attotime::from_usec(200));   // CLEAR after settle (calibration; << $a118's ~ms poll timeout)
			}
			// task#5: seek-COMPLETION keys on the same physical event as seek-active: the real STEP edge
			// reaching stp_w. Re-arm per pulse; when the train quiesces (30ms > inter-pulse gap) the
			// stepdone timer fires ONCE = train done + settle -> ch0 mailbox + IRQ2 (stepdone_tick).
			// Unified principle: both f000 bit1 and the op completion key on stp_w edges, never on E800
			// polling writes (the old E800 trigger manufactured completions from the wait loop's waggle).
			if (storager_getenv("STORAGER_STEPIRQ") && !prev_step && BIT(data, stepbit))
			{
				if (!m_stepdone_armed) logerror("STEPARM (stp_w edge) e802=%04x pc=%06x @%.5f\n", data, m_cpu->pc(), machine().time().as_double());
				m_stepdone_armed = true;
				m_stepdone->adjust(attotime::from_msec(30));
			}
			if (storager_getenv("STORAGER_A118") && m_cpu->pc() >= 0xa140 && m_cpu->pc() <= 0xa1b6)
			{ static int n = 0; if (n++ < 30) logerror("A118-STEP stepbit=%u data-bit=%d prev=%d edge=%d | m_settle_out=%d f000-b1=%d pc=%06x @%.6f\n",
				stepbit, BIT(data,stepbit), prev_step, (!prev_step && BIT(data,stepbit)), m_settle_out, m_settle_out?0:1, m_cpu->pc(), machine().time().as_double()); }
		}
		bool const host_int = BIT(data, 7);   // bit7 = host completion interrupt LEVEL (set 0x18e0, cleared 0x506e)
		if (host_int && !m_host_int_prev && m_iopb_addr)
		{
			// rising edge = command completion: post DONE (0x80) to the host IOPB status.
			address_space &bs = m_bus->space(AS_PROGRAM);
			if (storager_getenv("STORAGER_NOBYPASS")) logerror("IOPB80-POST L%d @%.5f\n", __LINE__, machine().time().as_double());
							bs.write_byte((m_iopb_addr + 2) & 0xffffff, 0x80); r0_observe(0x80);
			bs.write_byte((m_iopb_addr + 3) & 0xffffff, 0x80);
			if (m_trace) LOG("HOST-INT rise: iopb %06x status 0x80 + CPUAP int asserted\n", m_iopb_addr);
		}
		// The classic channel's completion is consumed by POLLING the IOPB status (ROM monitor and
		// boot loader both do); do NOT drive the Multibus interrupt lines from it - INT2 is the
		// kernel's sa interrupt (ioreg channel), and a stale level here sits pending until the
		// kernel first unmasks, then storms IR3.
		m_host_int_prev = host_int;
	}
}

void multibus_storager_device::floppy_formats(format_registration &fr)
{
	fr.add_mfm_containers();
	fr.add(FLOPPY_IMD_FORMAT);
}

static void storager_floppies(device_slot_interface &device)
{
	device.option_add("525qd", FLOPPY_525_QD);   // 5.25" 80-track DS (SINIX media); refine for FM/MFM rate
}

void multibus_storager_device::device_add_mconfig(machine_config &config)
{
	M68000(config, m_cpu, 50_MHz_XTAL / 4);
	m_cpu->set_addrmap(AS_PROGRAM, &multibus_storager_device::mem_map);
	m_cpu->set_addrmap(AS_OPCODES, &multibus_storager_device::opcodes_map);   // fetches see ROM at 0x4000+

	// floppy drives (5.25" DS); the SINIX boot media is on drive 0.  Status (track0/wp/door-closed) ->
	// F000, control (motor/dir/step/side/select) <- E804, per Dave's hardware guidance.
	FLOPPY_CONNECTOR(config, m_floppy[0], storager_floppies, "525qd", multibus_storager_device::floppy_formats);
	FLOPPY_CONNECTOR(config, m_floppy[1], storager_floppies, nullptr, multibus_storager_device::floppy_formats);

	// ESDI rigid disks (MC1325 et al) as raw images.  Two physical drives = two Storager LUNs:
	// unit 0 = -hard1 (SINIX root+usr), unit 1 = -hard2 (usr_db+usr_sma).  SINIX minor encodes the
	// LUN in bit 3 (minor>>3), so 0,11 / 0,13 select drive 1 -- the disk paths index m_hd[unit].
	STORAGER_HD_IMAGE(config, m_hd[0], 0);
	STORAGER_HD_IMAGE(config, m_hd[1], 0);

	// two 8253 PITs.  pit[1] counter 0 (loaded via $8001/$8007) drives F000 bit 11 in the timer
	// self-test; clock estimated at 10MHz/8 = 1.25MHz (pending schematic confirmation).
	PIT8253(config, m_pit[0]);
	m_pit[0]->set_clk<0>(10_MHz_XTAL / 8);
	m_pit[0]->set_clk<1>(10_MHz_XTAL / 8);
	m_pit[0]->set_clk<2>(10_MHz_XTAL / 8);
	// ch1 = mode-5 hardware-triggered one-shot per the fw's init ($8006 <- 0x7A): the seek-settle
	// timer, (re)triggered by the STEP line on its gate; OUT low = settling (F000 bit1 busy)
	m_pit[0]->out_handler<1>().set([this](int state) { m_settle_out = state; });
	PIT8253(config, m_pit[1]);
	// ctr0 = the fw's system tick: count 0xFF00 (ctrl 0x26 = MSB-only, mode 3), so tick = 65280/clk.
	// At /4 (2.5MHz): 26.112ms tick - INIT-era choreography fits (boots, read runs) but the host-staged
	// UIB settle counts (head-load 60t, spin-up 70t) overshoot the monitor patience (cont.162-164).
	// At full 10MHz (8254-2 territory): 6.528ms tick - the UIB timeouts become textbook (392ms/457ms/
	// 2ms-per-step) but the INIT-era step engine loops on the 2.86s retry signature (run293) because
	// the model's fixed synthetic delays (seek_done 75ms, step quiesce, settle) were tuned at 26ms.
	// Part number + clock tree + delay rescale = Dave's adjudication (cont.164).
	m_pit[1]->set_clk<0>(storager_getenv("STORAGER_CLK10M") ? (10_MHz_XTAL).value() : (10_MHz_XTAL / 4).value());   // cont.170 differential knob
	m_pit[1]->set_clk<1>(10_MHz_XTAL / 8);
	m_pit[1]->set_clk<2>(10_MHz_XTAL / 8);
	m_pit[1]->out_handler<0>().set(FUNC(multibus_storager_device::timer0_out));
	m_pit[1]->out_handler<2>().set(FUNC(multibus_storager_device::timer2_out));
}

void multibus_storager_device::timer0_out(int state)
{
	// PIT1 OUT0 is the system tick -> 68000 IRQ1 (vector lvl1 = handler 0x2b58, which ticks the timeout
	// queue $736c, setting flags like $7a36 that command waits poll for; the handler re-arms THIS counter
	// via gate0 + the $8001 count reload).  Wiring it is the real fix for the post-mount hang, BUT the
	// PIT input clock is unknown (10_MHz/8 here is a GUESS - the divider/source isn't from any dump), so
	// the tick rate vs the firmware's software-timeout can't be trusted yet; left unwired pending the
	// actual clock.  See STEP 21.
	bool const rising = state && !m_timer_out;
	// TEMP LOG (STRIP): fw-driven delay diagnosis
	{ static int n = 0; if (m_fw_driven && n++ < 60) logerror("PIT1-OUT0 -> %d @%.5f\n", state, machine().time().as_double()); }
	// cont.159 (STRIP): OUT transitions in the settle window, same timeline as GATE0.
	{ static int on = 0; double const ot = machine().time().as_double();
		if (on < 40 && ot > 7.95 && ot < 8.10)
		{ on++; logerror("OUT0 -> %d @%.6f\n", state, ot); } }
	m_timer_out = bool(state);                       // F000 bit11 (polled in the timer self-test)
	if (rising && m_gate0)                            // TEST: ctr0 OUT -> IRQ1 (does it autovector 0x2b58 or go spurious 0x24a?)
		m_cpu->set_input_line(M68K_IRQ_1, HOLD_LINE);
}

void multibus_storager_device::timer2_out(int state)
{
	m_tick_prev = bool(state);
}

// Instruction-fetch (AS_OPCODES) space: the EPROM is decoded for fetches across the whole 0x0-0xFFFF,
// so the worker code at 0x4000-0x7FFF runs DIRECTLY from ROM.  The data space (mem_map) has RAM there.
// This program/data (FC) split is why the firmware bsr's worker routines (0x75c2 etc.) it never installs:
// they are fetched from EPROM, while the same addresses serve RAM for data (variables/stack/mailbox).
void multibus_storager_device::opcodes_map(address_map &map)
{
	map(0x000000, 0x00ffff).rom().region("cpu", 0).mirror(0xff0000);
}

void multibus_storager_device::mem_map(address_map &map)
{
	// 8kx8 * 2 sram, 4kx1 sram, 64kx8 eprom, 32x8 * 2 prom.
	// The firmware reaches on-board I/O both A5-relative (A5=0x8000 -> 0x00xxxx) and via 68000
	// short-absolute sign-extension (0xFFxxxx), so the I/O blocks are mirrored across A16-A23.
	// The board decodes only A1-A15 (A16-A23 are don't-care), so the whole map aliases across
	// A16-A23.  The firmware relies on this: it reaches ROM/RAM/I/O via 68000 short-absolute and
	// PC-relative addressing that sign-extends to 0xFFxxxx (e.g. bsr from 0x874 targets 0xFF9018 =
	// ROM 0x9018).  Mirror everything with 0xff0000.
	map(0x000000, 0x00ffff).rom().region("cpu", 0).mirror(0xff0000);
	map(0x004000, 0x007fff).ram();   // RAM is positive-addressed (<0x8000); no high mirror needed

	// two 8253 PITs interleaved at 0x8000 (#0 = even/high byte, #1 = odd/low byte)
	map(0x008000, 0x008007).mirror(0xff0000).rw(m_pit[0], FUNC(pit8253_device::read), FUNC(pit8253_device::write)).umask16(0xff00);
	map(0x008000, 0x008007).mirror(0xff0000).rw(m_pit[1], FUNC(pit8253_device::read), FUNC(pit8253_device::write)).umask16(0x00ff);

	// disk read/write channel + main control + drive status (74LS1801/1802 ENDEC + drives).
	// 0xC800 board/host control (written 0 during hw init); 0xE000-0xE01F channel; 0xE800-0xE807
	// control (E800/E802/E804/E806); 0xF000 status.  Logged for RE (ch_r/ch_w); E802 = IRQ2 ack.
	map(0x00c000, 0x00c7ff).mirror(0xff0000).w(FUNC(multibus_storager_device::c000_w));   // host-address counter preset (write-only; reached via sign-extended abs.w at 0xFFC0xx)
	map(0x00c800, 0x00c9ff).mirror(0xff0000).rw(FUNC(multibus_storager_device::c800_r), FUNC(multibus_storager_device::c800_w));
	map(0x00d000, 0x00d001).mirror(0xff0000).w(FUNC(multibus_storager_device::d000_w));   // write-only; reads = ROM
	map(0x00d800, 0x00d801).mirror(0xff0000).w(FUNC(multibus_storager_device::d800_w));   // DMA host high-addr (paired w/ C800)
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

	// SGI 026-0005-001 REV C S/N 6202, MC68000L10, "Storager 3030", 40MHz & 32MHz crystals, S8526-G II-0003-160-00T
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
