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
	void advance_read();            // stage + deliver the next record of the counted read (IRQ6 then IRQ5)
	void deliver_mark();            // arm + held-mark -> one interrupt
	TIMER_CALLBACK_MEMBER(pump_tick);

	// channel DMA: on the E800 bit12 kickoff the gate array bus-masters one field between the local
	// SRAM buffer and host memory via the C000 up-counter, then raises IRQ4 (channel/DMA done).
	void run_channel_dma();
	TIMER_CALLBACK_MEMBER(dma_done);   // async channel-done: raise IRQ4 after the transfer time (STORAGER_ASYNCDMA)

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
	u16  m_dma_term = 0;         // transfer terminal, latched from a C800 write while DMA active
	u8   m_term_bit0 = 0;
	u32  m_last_bw = 0;          // last local-bus write address (transfer-pointer snoop -> F000 bit12)
	bool m_timer_out = false;    // PIT1 ctr0 OUT -> F000 bit11
	bool m_settle_out = true;    // PIT0 ctr1 mode-5 one-shot OUT; F000 bit1 seek/settle busy = !OUT
	bool m_r0_busy = false;      // R0 status bit1: latched at host GO, cleared at the firmware's DONE stamp
	bool m_r0_doneint = false;   // R0 status bit2: OPER-DONE-INT, set at DONE, cleared by CLR-INT
	bool m_gate0 = false;        // E800 bit9 -> PIT1 ctr0 gate
	bool m_e800_bit12_prev = false;   // bit12 kickoff edge detect
	bool m_host_int_prev = false;     // E802 bit7 host-completion interrupt level
	bool m_floppy_loaded = false;

	// host doorbell / IOPB
	u8  m_iopb_cmd = 0;          // command byte from the auto-fetched IOPB (drives the model's channel)
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
	// counted engine (advance_read) delivers exactly the commanded run of sectors, one IRQ6 + one IRQ5 each.
	struct captured_sector { u8 c = 0, h = 0, r = 0, nn = 0, dam = 0; u16 len = 0; u8 data[1200] = {}; };
	captured_sector m_track[32];
	int  m_track_n = 0;             // sectors recovered in the last full-track capture
	void capture_track();          // synchronous full-revolution flux decode into m_track
	bool m_read_active = false;     // a counted read is in progress (delivering the commanded run)
	int  m_sec_count = 0;           // commanded sectors this operation ([$7abc])
	int  m_sec_index = 0;           // sector currently being delivered
	int  m_sec_phase = 0;           // 0 = ID (IRQ6) next, 1 = data (IRQ5) next
};

// ---------------------------------------------------------------------------
// read path (74LS1801/1802 ENDEC data separation + gate-array deserialise)
// ---------------------------------------------------------------------------

// FM vs MFM cell rate.  The firmware programs the ENDEC format via E800 bits10-11 (latched from
// UIB+$11 by op 0x16, spec §3.2); the boot FM label read was observed with bit10 cleared (spec
// §3.1), so bit10 clear = FM (4us/cell, 128B), set = MFM (2us/cell, 256B).
bool multibus_storager_device::flux_density_fm() const
{
	return !BIT(m_ch[(0xe800 - 0xe000) / 2], 10);
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
	int const side = m_cpu->space(AS_PROGRAM).read_word(0x7436) & 1;   // firmware target head [$7436]
	fdd->mon_w(0);
	fdd->ss_w(side);
	bool const fm = flux_density_fm();
	fdc_pll_t pll;
	pll.set_clock(attotime::from_nsec(fm ? 4000 : 2000));
	attotime tm = machine().time();
	pll.read_reset(tm);
	attotime const end = tm + attotime::from_msec(210);   // >1 revolution at 300 rpm (200ms)
	u32 shift = 0;
	int state = 0, cells = 0, nb = 0, want = 0, n = 0;
	u8 dam = 0, id[4] = {};
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
		}
		else              // data field recovered -> store this sector
		{
			captured_sector &s = m_track[m_track_n++];
			s.c = id[0]; s.h = id[1]; s.r = id[2]; s.nn = id[3]; s.dam = dam;
			s.len = u16(want);
			for (int k = 0; k < want && k < int(sizeof(s.data)); k++)
				s.data[k] = buf[k];
		}
		state = 0;
	}
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
// commanded sectors and terminates - it is a counted operation, not a free-running per-mark echo.  The
// whole track is decoded up front (capture_track - detection-is-capture, the AM2147 bit-buffer + the
// 1801 preamble search absorb positional slop); this delivers the wanted run to the firmware lock-step
// with its per-record arm.  Per sector: IRQ6 (ID address mark, C/H/R/N staged in $7DAC where the
// node+$c8..$ce POSPTR verify reads it) then a SINGLE IRQ5 (data field already DMA'd into the firmware-
// programmed chunk C800[0] = [$741e]) = 8xIRQ6 + 8xIRQ5 for an 8-sector read.  When the count exhausts,
// capture stops; the firmware then launches the SRAM->host transfer whose channel-done IRQ4
// (run_channel_dma) is the operation-complete op42 waits on.
void multibus_storager_device::advance_read()
{
	if (m_mark_pending || !m_read_active)
		return;                             // a record is held awaiting the arm, or the read is complete
	if (m_sec_index >= m_sec_count)
	{
		m_read_active = false;              // all commanded sectors delivered - capture complete, stop
		return;
	}
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
		cs.write_byte(0x7dac, 0xfe);            // normalised IDAM
		cs.write_byte(0x7dad, s.c);             // C
		cs.write_byte(0x7dae, s.h);             // H
		cs.write_byte(0x7daf, s.r);             // POSITION (sector R)
		cs.write_byte(0x7db0, s.nn);            // N
		m_mark_pending = 6;
		m_sec_phase = 1;
	}
	else if (m_sec_phase == 1)
	{
		// The firmware assembles the whole run in a linear SRAM buffer, advancing the local address by one
		// field per sector ($4000, $4080, ...); its per-sector host DMA sources D000 = that advancing address.
		// (This only becomes visible once the transfer chain actually runs all 8 - it needs the DMA to hold
		// the CPU so the firmware doesn't race ahead; see run_channel_dma.)  Stage each sector to its slot.
		u32 const dst = 0x4000 + m_sec_index * s.len;
		if (dst >= 0x4000 && dst + s.len <= 0x8000)
			for (int k = 0; k < s.len; k++)
				cs.write_byte((dst + k) & 0xffff, s.data[k]);
		m_mark_pending = 5;                     // data field armed (IRQ5 #1 -> $7BA8)
		m_sec_phase = 2;
	}
	else
	{
		m_mark_pending = 5;                     // data field captured (IRQ5 #2 -> $8018 done)
		m_sec_phase = 0;
		m_sec_index++;                          // one sector completed after its data-done record
	}
	deliver_mark();
}

// The mark clock: stage/deliver the next record of the counted read while the window is armed, then
// reschedule.  advance_read() holds one record at a time and stops after the commanded count; deliver_mark
// releases it once the firmware has armed and the CPU is unmasked.
TIMER_CALLBACK_MEMBER(multibus_storager_device::pump_tick)
{
	deliver_mark();
	if (m_read_window)
		advance_read();
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
	if ((to_local || to_host) && ld >= 0x4000 && ld + len <= 0x8000 && m_c000 >= 0x010000 && m_c000 < 0xff0000)
	{
		// The status write-back (local->host) byte-swaps each 16-bit word: the 68000 writes the status as
		// its BE low byte (node+3), and the CPUAP polls IOPB+2 - the swap lands node+3 at host+2.  The
		// host->local fetches (IOPB/UIB) are consumed byte-wise by the firmware and copy straight.
		for (u32 k = 0; k < len; k++)
		{
			if (to_local) cs.write_byte((ld + k) & 0xffff, bs.read_byte((m_c000 + k) & 0xffffff));
			else          bs.write_byte((m_c000 + (k ^ 1)) & 0xffffff, cs.read_byte((ld + k) & 0xffff));
		}
	}
	// The bus-master transfer completes and the gate array raises the channel-done IRQ4.  For the read's
	// per-slot channel transfers, raising it synchronously re-enters the firmware's launch sequence mid-
	// flight; schedule it a few microseconds out (real DMA time) so the launch completes first.  INIT/UIB
	// fetches (0x87) and node write-backs expect it synchronously - the firmware polls the result at once.
	if (m_iopb_cmd == 0x94 || m_iopb_cmd == 0x95)
	{
		// The gate array bus-masters the transfer and holds the 68000 off the bus for its duration (DTACK), so
		// the firmware cannot start the next sector's work until the DMA completes.  Modelling that hold (spin
		// the CPU for the transfer time) is what lets the transfer chain run to completion: without it our
		// instant DMA lets the CPU race ahead and the descriptor drain launches only 2 of 8 sectors; with it,
		// all 8 launch and the descriptor queue drains.
		if (is_data)
			m_cpu->spin_until_time(attotime::from_usec(40));
		m_dma_done->adjust(attotime::from_usec(40));
	}
	else
		m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE);
}

TIMER_CALLBACK_MEMBER(multibus_storager_device::dma_done)
{
	m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE);    // channel / DMA done (after the transfer time)
}

// ---------------------------------------------------------------------------
// gate-array registers
// ---------------------------------------------------------------------------

u16 multibus_storager_device::ch_r(offs_t offset, u16 mem_mask)
{
	u32 const a = 0xe000 + offset * 2;
	u16 d = m_ch[offset];

	// E000: the gate array's command/status register (the firmware reads it only a handful of times, as
	// status - it does NOT byte-assemble the bit stream; the gate array deserializes each record into SRAM
	// itself and signals via the IRQ6/IRQ5 marks).  Return the register echo.
	if (a == 0xe000)
		return m_ch[offset];

	// E01E: the gate array's record-status latch.  Reading it is the firmware's ACK - it CLEARS the latch
	// so the gate array can deliver the next record.  The read value itself is discarded by the firmware
	// (the CRC/status verdict rides the collected status word, not this port), so the strobe is the point.
	if (a == 0xe01e)
	{
		d |= 0x0010;
		m_rec_latch = false;
	}

	// F000: drive + auxiliary status (spec §3.3).
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
		d = (d & ~0x0002) | (m_settle_out ? 0 : 0x0002);  // bit1  = seek/settle busy (PIT ctr1 one-shot, low = busy)
	}
	return d;
}

void multibus_storager_device::ch_w(offs_t offset, u16 data, u16 mem_mask)
{
	u32 const a = 0xe000 + offset * 2;
	COMBINE_DATA(&m_ch[offset]);

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
		// and arm the counted delivery of exactly the commanded run (advance_read).  Each per-record re-arm
		// (E802 bit11) releases the next mark.
		bool const win = BIT(data, 11) && (m_iopb_cmd == 0x94 || m_iopb_cmd == 0x95);
		if (win && !m_e000b11_prev)
		{
			m_read_window = true;
			m_armed = true;
			if (!m_read_active && m_iopb_cmd == 0x95)
			{
				capture_track();
				int n = m_cpu->space(AS_PROGRAM).read_word(0x7abc) & 0xff;   // commanded sectors this track [$7abc]
				if (n < 1 || n > m_track_n) n = m_track_n;
				m_sec_count = n;
				m_sec_index = 0;
				m_sec_phase = 0;
				m_read_active = true;
			}
			m_pump->adjust(attotime::from_usec(200));   // start the mark clock
			advance_read();      // stage + deliver the first record if the firmware is armed
		}
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
			run_channel_dma();
	}
	else if (a == 0xe802)
	{
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
		for (u32 k = 0; k < 0x18; k++)
			cs.write_byte((dst + k) & 0xffff, bs.read_byte((dbi + k) & 0xffffff));
		// Capture the fetched-IOPB pointer so the firmware knows where its work IOPB is: [$7B20] is the
		// "fetched-IOPB ptr" the intake reads (A2 = [$7B20]; ($2,A2) = 0x81 busy) and the completion
		// stamps.  Without it the accept/status stamps land on garbage (addr 0).
		cs.write_word(0x7b20, dst);
		m_iopb_cmd = cs.read_byte(dst);
		m_iopb_addr = dbi;
		m_window_seen = false; m_term_fired = false; m_idx_prev = false; m_read_active = false;   // per-command reset
		// The firmware carries the STATUS/ERROR bytes back to the host IOPB itself, via its node->host
		// bus-master DMA (run_channel_dma, E800 bit13); the model transcribes nothing here.
		m_read_window = false;   // new command: a read re-arms its own window (no stale arm across the boundary)
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
		// snoop the local-bus buffer (0x4000-0x7FFF): the gate array's address comparator watches
		// writes to advance the transfer pointer (F000 bit12 terminal match).
		address_space &cs = m_cpu->space(AS_PROGRAM);
		cs.install_read_tap(0x4000, 0x7fff, "dma_snoop_r",
			[this](offs_t, u16 &, u16 mem_mask) { if (m_dma_active) m_term_bit0 = (mem_mask == 0x00ff) ? 1 : 0; });
		cs.install_write_tap(0x4000, 0x7fff, "dma_snoop_w",
			[this](offs_t offset, u16 &, u16 mem_mask) { if (m_dma_active) m_last_bw = offset + ((mem_mask == 0x00ff) ? 1 : 0); });
		m_installed = true;
	}
	m_cpu->set_input_line(M68K_IRQ_2, CLEAR_LINE);
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
	spin_drives();
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
	M68000(config, m_cpu, 50_MHz_XTAL / 4);
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

void multibus_storager_device::mem_map(address_map &map)
{
	// The board decodes only A1-A15 (A16-A23 don't-care); the firmware reaches ROM/RAM/I/O via 68000
	// short-absolute and PC-relative addressing that sign-extends to 0xFFxxxx, so everything mirrors
	// across A16-A23.
	map(0x000000, 0x00ffff).rom().region("cpu", 0).mirror(0xff0000);
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
