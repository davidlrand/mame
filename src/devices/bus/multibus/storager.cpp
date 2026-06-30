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
#include "storager.h"

#include "cpu/m68000/m68000.h"
#include "machine/pit8253.h"
#include "imagedev/floppy.h"
#include "formats/imd_dsk.h"

#include <fstream>
#include <map>
#include <vector>

#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"

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
		, m_installed(false)
		, m_trace(false)
		, m_ch{}
		, m_dma_active(false)
		, m_c800{}
		, m_dma_term(0)
		, m_term_bit0(0)
		, m_last_bw(0)
		, m_d000(0)
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
		, m_readsec(7)
		, m_writeoff(0)
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
	u16 bus_mem_r(offs_t offset, u16 mem_mask);
	void bus_mem_w(offs_t offset, u16 data, u16 mem_mask);

	// LLE host interface: the Multibus PIO window 0x7200-0x73FF maps byte-for-byte to the Storager's
	// on-board dual-port RAM at 0x7E00-0x7FFF (+0xC00).  The CPUAP writes the command/IOPB-pointer
	// mailbox there; writing GO (0x13) to the command register (PIO 0x73F8 = fw 0x7FF8) is the
	// doorbell that raises 68000 IRQ2 (handler @0x24AA).  The firmware then runs the command itself.
	u16 host_win_r(offs_t offset);
	void host_win_w(offs_t offset, u16 data, u16 mem_mask);

	// disk read/write channel (74LS1801/1802 + drives) at 0xE000-0xFFFF.  Logged for RE; backed by
	// RAM for now so the firmware runs.  E802 = IRQ ack (clears the doorbell IRQ2).
	u16 ch_r(offs_t offset, u16 mem_mask);
	void ch_w(offs_t offset, u16 data, u16 mem_mask);
	u16 c800_r(offs_t offset);
	void c800_w(offs_t offset, u16 data, u16 mem_mask);
	void timer0_out(int state);
	void timer2_out(int state);
	void load_floppy();

	required_device<m68000_device> m_cpu;
	required_device_array<pit8253_device, 2> m_pit;
	required_device_array<floppy_connector, 2> m_floppy;

	bool m_installed;
	bool m_trace;            // enable disk-channel logging once a host command arrives
	u16 m_ch[0x1000];        // 0xE000-0xFFFF backing store

	// --- gate-array DMA state machine (VGC7219, reverse-engineered) ---
	bool m_dma_active;       // DMA running (E800 bit6: 0xcd3 start / 0xc12 stop)
	u16  m_c800[0x80];       // C800 scatter/gather translation register file
	u16  m_dma_term;         // latched transfer terminal (low bits) for the address comparator
	u8   m_term_bit0;        // terminal LSB, latched from the helper's buffer read in DMA mode
	u32  m_last_bw;          // last local-bus buffer write address (snooped transfer pointer)
	u16  m_d000;             // D000 write-only DMA address latch (word address; reads = ROM)
	bool m_timer_out;        // 8253 pit[1] counter-0 OUT -> F000 bit 11 (timer self-test)
	bool m_tick_prev;        // previous pit[1] OUT2 level (system tick edge -> IRQ1)
	bool m_gate0;            // pit[1] ctr0 gate (E800 bit9): suppress IRQ1 on the handler's gate-toggle
	bool m_shadow_loaded;    // gate-array test has hard-copied the EPROM worker code into RAM
	bool m_iopb_fetched;     // gate array has autonomously bus-mastered the IOPB for this command
	u8   m_iopb_cmd;         // V/SMD 3200 command byte from the fetched IOPB (0x87 INIT, 0x95 READ-SEQ...)
	u32  m_iopb_buffer;      // host buffer addr = IOPB words 6-7 (BE32, low 24 bits): UIB or data target
	u32  m_iopb_addr;        // host IOPB base addr in CPUAP RAM (for the status write-back)
	bool m_host_int_prev;    // previous E802 bit7 (host completion interrupt level)
	bool m_e800_bit12_prev;  // previous E800 bit12 (kickoff is the rising edge, not the level)
	bool m_e802_step;        // last E802 bit0 (STEP); the seek loop polls F000 bit1 = STEP echo
	u8   m_readsec;          // READ-SEQ rotation position: next physical sector ID under the head (1..16)
	u32  m_writeoff;         // READ-SEQ host-buffer fill offset (data laid sequentially from buffer+0)

	// fallback (decoded-sector feed): IMD parsed into (cyl<<16 | head<<8 | sector) -> bytes.
	std::map<u32, std::vector<u8>> m_sectors;
	bool m_floppy_loaded;
};

void multibus_storager_device::device_start()
{
	std::fill(std::begin(m_ch), std::end(m_ch), 0);
	save_item(NAME(m_ch));
	save_item(NAME(m_trace));
}

// fallback: parse the SINIX0 floppy IMD into a (cyl,head,sector) -> bytes map (unused in the LLE path).
void multibus_storager_device::load_floppy()
{
	m_floppy_loaded = true;
	std::ifstream f("siemens/set1/mx2-001.imd", std::ios::binary);
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
}

void multibus_storager_device::device_reset()
{
	if (!m_installed)
	{
		// Multibus PIO window 0x7200-0x73FF -> on-board dual-port RAM 0x7E00-0x7FFF (+0xC00).
		m_bus->space(AS_IO).install_readwrite_handler(0x7200, 0x73ff,
			read16sm_delegate(*this, FUNC(multibus_storager_device::host_win_r)),
			write16s_delegate(*this, FUNC(multibus_storager_device::host_win_w)));
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
				if (offset == 0x7fe8 && m_iopb_addr)
				{
					address_space &bs = m_bus->space(AS_PROGRAM);
					bs.write_byte((m_iopb_addr + 2) & 0xffffff, 0x80);
					bs.write_byte((m_iopb_addr + 3) & 0xffffff, 0x80);
					if (m_trace) LOG("  HOST-STATUS: iopb %06x +2/+3 <- 0x80 (done)\n", m_iopb_addr);
				}
			});
		m_installed = true;
	}
	m_cpu->set_input_line(M68K_IRQ_2, CLEAR_LINE);
	m_dma_active = false;
	m_shadow_loaded = false;
	m_iopb_fetched = false;
	m_host_int_prev = false;
	m_e800_bit12_prev = false;
}

// C800 scatter/gather translation register file.  A write while DMA is active latches the transfer
// terminal: index = byte offset (offset*2) = address bits 7-11, value = address bits 1-6.
u16 multibus_storager_device::c800_r(offs_t offset)
{
	return m_c800[offset & 0x7f];
}

void multibus_storager_device::c800_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_c800[offset & 0x7f]);
	if (m_trace) LOG("  C800[%02x] = %04x dma=%d [%06x]\n", offset & 0x7f, data, m_dma_active, m_cpu->pc());
	if (m_dma_active)
		m_dma_term = (u16((offset & 0x7f) * 2) << 6) | ((data & 0x3f) << 1);   // address bits 1-11
}

// D000 = write-only DMA address latch (word address; reads of 0xD000 fall through to ROM).
void multibus_storager_device::d000_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_d000);
	if (m_trace) LOG("  D000 wr = %04x (byte addr %06x) [%s]\n", m_d000, u32(m_d000) << 1, machine().describe_context());
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
	if (m_trace) LOG("  BUSMEM wr %06x = %04x (mask %04x) [%06x]\n", addr, data, mem_mask, m_cpu->pc());
}

// CPUAP reads/writes the dual-port mailbox; map byte-for-byte into the 68000's RAM at +0xC00.
u16 multibus_storager_device::host_win_r(offs_t offset)
{
	u32 const fa = 0x7e00 + offset * 2;
	address_space &cs = m_cpu->space(AS_PROGRAM);
	u16 const v = cs.read_byte(fa) | (u16(cs.read_byte(fa + 1)) << 8);
	if (m_trace) LOG("  HOSTWIN rd pio=%04x fw=%04x -> %04x\n", 0x7200 + offset * 2, fa, v);
	return v;
}

void multibus_storager_device::host_win_w(offs_t offset, u16 data, u16 mem_mask)
{
	u32 const pio = 0x7200 + offset * 2;
	u32 const fa  = 0x7e00 + offset * 2;
	address_space &cs = m_cpu->space(AS_PROGRAM);
	if (ACCESSING_BITS_0_7)  cs.write_byte(fa,     data & 0xff);
	if (ACCESSING_BITS_8_15) cs.write_byte(fa + 1, data >> 8);

	// doorbell: the CPUAP writes GO (0x13) to the command register (PIO 0x73F8) last -> raise IRQ2.
	if (pio == 0x73f8 && ACCESSING_BITS_0_7 && (data & 0xff) == 0x13)
	{
		m_trace = true;
		// The gate array auto-fetches the IOPB (14 words = 0x1c bytes) from the mailbox pointer into the
		// firmware's IOPB buffer ($7a06) on EVERY doorbell - the firmware reads it without a kickoff
		// (e.g. RESTORE).  Copy ONLY 0x1c bytes so the work area at +0x1c is preserved.
		{
			u32 const dbi = (u32(cs.read_byte(0x7ff9)) << 16) | (u32(cs.read_byte(0x7ffa)) << 8) | cs.read_byte(0x7ffb);
			u32 dst = cs.read_word(0x7a06);
			if (dst < 0x4000 || dst >= 0x7e00) dst = 0x71f0;
			address_space &bs = m_bus->space(AS_PROGRAM);
			for (u32 k = 0; k < 0x1c; k++)
				cs.write_byte((dst + k) & 0xffff, bs.read_byte((dbi + k) & 0xffffff));
			m_iopb_cmd = cs.read_byte(dst);
			m_iopb_buffer = ((u32(cs.read_byte(dst + 0xc)) << 24) | (u32(cs.read_byte(dst + 0xd)) << 16)
				| (u32(cs.read_byte(dst + 0xe)) << 8) | cs.read_byte(dst + 0xf)) & 0xffffff;
			m_iopb_addr = dbi;
			m_iopb_fetched = true;            // already fetched; the kickoff does the data/UIB transfer
			if (m_iopb_cmd == 0x89) { m_readsec = 7; m_writeoff = 0; }   // RESTORE -> track0, data sector 7, buffer empty
			else if (m_iopb_cmd == 0x95)
			{
				// READ-SEQ: the monitor reads the result buffer only ~75us after this doorbell, well before
				// the firmware's own gate-array DMA would land - so deliver the data SYNCHRONOUSLY now.  One
				// 256-byte logical block = 2 physical FM sectors taken in rotation order.  The loader's label
				// buffer is at R7+0x65 = IOPB-buffer-4 (the IOPB buffer pointer aims 4 bytes in, at the VOL1
				// volume-id field), so deliver the full record there UNSHIFTED: the id lands at the buffer
				// pointer for the SINIX check (monitor 0xfe3686) AND every label field (geometry, ...) sits
				// at the offset the mount expects.
				if (!m_floppy_loaded) load_floppy();
				u32 const dstbase = (m_iopb_buffer - 4) & 0xffffff;
				u32 boff = 0;
				for (int s = 0; s < 2; s++)
				{
					auto const it = m_sectors.find((0u << 16) | (0u << 8) | u32(m_readsec));
					if (it != m_sectors.end())
					{
						for (size_t k = 0; k < it->second.size(); k++)
							bs.write_byte((dstbase + boff + k) & 0xffffff, it->second[k]);
						boff += it->second.size();
					}
					m_readsec = (m_readsec % 16) + 1;
				}
				LOG("READ-SEQ sync-deliver -> %06x (cyl=%02x hd=%02x sec=%02x; next secID%u)\n",
					dstbase, cs.read_byte(dst + 5), cs.read_byte(dst + 6), cs.read_byte(dst + 7), m_readsec);
			}
			bs.write_byte((dbi + 2) & 0xffffff, 0x81);   // BUSY
			bs.write_byte((dbi + 3) & 0xffffff, 0x81);
			LOG("DOORBELL fetch: IOPB %06x -> %04x cmd=%02x buffer=%06x\n", dbi, dst, m_iopb_cmd, m_iopb_buffer);
		}
		// command word @0x7ff8 = cmd byte + 24-bit big-endian IOPB pointer into CPUAP RAM.
		u32 const iopb = (u32(cs.read_byte(0x7ff9)) << 16) | (u32(cs.read_byte(0x7ffa)) << 8) | cs.read_byte(0x7ffb);
		LOG("CMD %02x IOPB-ptr=%06x\n", cs.read_byte(0x7ff8), iopb);
		m_cpu->set_input_line(M68K_IRQ_2, ASSERT_LINE);
	}
	else
		LOG("HOSTWIN wr pio=%04x fw=%04x = %04x (mask %04x)\n", pio, fa, data, mem_mask);
}

// Disk read/write channel (74LS1801/1802 ENDEC + drives), logged for RE.
u16 multibus_storager_device::ch_r(offs_t offset, u16 mem_mask)
{
	u32 const a = 0xe000 + offset * 2;
	u16 d = m_ch[offset];
	if (a == 0xf000)   // status: bit12 (0x1000) = DMA transfer address reached the terminal
	{
		u32 const term = 0x4000 | m_dma_term | m_term_bit0;
		bool const reached = m_dma_active && (m_last_bw == term);
		d = (d & ~0x1000) | (reached ? 0x1000 : 0);        // bit12 = DMA address-match
		d = (d & ~0x0800) | (m_timer_out ? 0x0800 : 0);    // bit11 = 8253 timer OUT
		// drive status (floppy 0) - EMPIRICAL bit map (iterating): bit7 = ready/door-closed (5.25" has
		// no 8" READY -> media present), bit13 = track0, bit10 = write-protect.
		floppy_image_device *const fdd = m_floppy[0]->get_device();
		bool const present = fdd && fdd->exists();
		bool const trk0 = present && !fdd->trk00_r();   // trk00_r active-low (0 at track 0)
		bool const wprot = present && !fdd->wpt_r();     // wpt_r active-low (0 = protected)
		d = (d & ~0x0080) | (present ? 0x0080 : 0);   // bit7  = drive ready / door-closed
		d = (d & ~0x2000) | (trk0 ? 0x2000 : 0);      // bit13 = track 0
		d = (d & ~0x0400) | (wprot ? 0x0400 : 0);     // bit10 = write-protect
		d = (d & ~0x0002) | (m_e802_step ? 0x0002 : 0);  // bit1 = step/seek-complete (echoes E802 STEP)
		if (m_trace) LOG("  F000 rd pc=%06x bw=%06x term=%06x A0=%04x -> %04x\n", m_cpu->pc(), m_last_bw, term, u16(m_cpu->state_int(M68K_A0)), d);
	}
	if (m_trace && a != 0xf000) LOG("  CH rd %04x = %04x [%s]\n", a, d, machine().describe_context());
	return d;
}

void multibus_storager_device::ch_w(offs_t offset, u16 data, u16 mem_mask)
{
	u32 const a = 0xe000 + offset * 2;
	COMBINE_DATA(&m_ch[offset]);
	if (m_trace) LOG("  CH wr %04x = %04x [%s]\n", a, data, machine().describe_context());

	// E800 = DMA/channel control.  bit6 = DMA enable (0x?d3 start / 0x?12 stop);
	// bit9 = 8253 counter-0 gate (timer self-test: 0xc14 gate off -> 0xe14 gate on).
	if (a == 0xe800)
	{
		m_dma_active = BIT(data, 6);
		m_gate0 = BIT(data, 9);
		m_pit[1]->write_gate0(BIT(data, 9));

		// kickoff = the RISING EDGE of bit12 (the disk-op does ori #$1000; the IRQ4 handler clears it).
		// Edge-detect so incidental E800 writes that carry bit12 (e.g. the IRQ1 timer handler) don't
		// re-trigger a transfer mid-DMA.
		bool const kick = BIT(data, 12) && !m_e800_bit12_prev;
		m_e800_bit12_prev = BIT(data, 12);

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
			for (unsigned i = 0; i < 0x80; i++)
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

		// A real transfer (D000 set) launched: the gate array runs it and raises IRQ4 on completion
		// (the IRQ4 handler at 0x3bfe clears E800 bit12).  Pulse IRQ4 so the command completes.
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
				u32 const iopb = (u32(cs.read_byte(0x7ff9)) << 16) | (u32(cs.read_byte(0x7ffa)) << 8) | cs.read_byte(0x7ffb);
				// the host IOPB is 14 words (0x1c bytes); copy ONLY that - 0x1c+ is the firmware's
				// own work area (clobbering it with host data faults on a stale odd pointer).
				for (u32 k = 0; k < 0x1c; k++)
					cs.write_byte((dst + k) & 0xffff, bs.read_byte((iopb + k) & 0xffffff));
				m_iopb_cmd = cs.read_byte(dst);   // w0 high byte = command
				m_iopb_buffer = ((u32(cs.read_byte(dst + 0xc)) << 24) | (u32(cs.read_byte(dst + 0xd)) << 16)
					| (u32(cs.read_byte(dst + 0xe)) << 8) | cs.read_byte(dst + 0xf)) & 0xffffff;  // words 6-7
				m_iopb_addr = iopb;
				// post BUSY (0x81) to the host IOPB status (CPUAP polls IOPB+2 for 0x81->0x80).
				bs.write_byte((iopb + 2) & 0xffffff, 0x81);
				bs.write_byte((iopb + 3) & 0xffffff, 0x81);
				if (m_trace) LOG("GATE-DMA IOPB fetch: CPUAP %06x -> local %04x; cmd=%02x buffer=%06x\n", iopb, dst, m_iopb_cmd, m_iopb_buffer);
			}
			else if (m_iopb_cmd == 0x87)
			{
				// INITIALIZE: the controller reads the host-supplied UIB (drive geometry) from the
				// IOPB buffer addr into local RAM.  Host -> controller.  Copy only 0x1c bytes so the
				// firmware work area at dst+0x1c (read by later commands, e.g. RESTORE's ($24,A0)) survives.
				for (u32 k = 0; k < 0x1c; k++)
					cs.write_byte((dst + k) & 0xffff, bs.read_byte((m_iopb_buffer + k) & 0xffffff));
				if (m_trace) LOG("GATE-DMA UIB read: CPUAP %06x -> local %04x (INIT)\n", m_iopb_buffer, dst);
			}
			// READ-SEQ (0x95) data is delivered synchronously at the doorbell (host_win_w); nothing to do here.
			if (m_trace) LOG("CHANCOMPLETE -> IRQ4 (D000 byte %06x)\n", u32(m_d000) << 1);
			m_cpu->set_input_line(M68K_IRQ_4, HOLD_LINE);
		}
		// (the old EPROM->RAM shadow-load is gone: the worker code is fetched directly from ROM via the
		// AS_OPCODES program/data split; RAM at 0x4000-0x7fff is data-only, so it stays uncorrupted.)
	}

	// E802 = interrupt control.  bit6 = the IRQ2 ack the handler toggles on entry.  bit7 (set only at
	// command completion, e.g. 0x00c1 @pc 0x18ec) = assert the host completion interrupt to the CPUAP.
	if (a == 0xe802)
	{
		m_cpu->set_input_line(M68K_IRQ_2, CLEAR_LINE);
		m_e802_step = BIT(data, 0);   // bit0 = STEP; the seek loop (0xa19e) polls F000 bit1 = STEP echo
		// route the head positioning to the drive: bit1 = DIR (1 = toward track0 = cyl--), bit0 = STEP
		// (floppy_image_device steps on the active edge using the current dir).
		if (floppy_image_device *const fdd = m_floppy[0]->get_device())
		{
			fdd->dir_w(BIT(data, 1));
			fdd->stp_w(BIT(data, 0));
		}
		bool const host_int = BIT(data, 7);   // bit7 = host completion interrupt LEVEL (set 0x18e0, cleared 0x506e)
		if (host_int && !m_host_int_prev && m_iopb_addr)
		{
			// rising edge = command completion: post DONE (0x80) to the host IOPB status.
			address_space &bs = m_bus->space(AS_PROGRAM);
			bs.write_byte((m_iopb_addr + 2) & 0xffffff, 0x80);
			bs.write_byte((m_iopb_addr + 3) & 0xffffff, 0x80);
			if (m_trace) LOG("HOST-INT rise: iopb %06x status 0x80 + CPUAP int asserted\n", m_iopb_addr);
		}
		// level-drive the CPUAP completion interrupt from bit7 (TEMP: all 8 lines until narrowed).
		int_w<0>(host_int); int_w<1>(host_int); int_w<2>(host_int); int_w<3>(host_int);
		int_w<4>(host_int); int_w<5>(host_int); int_w<6>(host_int); int_w<7>(host_int);
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

	// two 8253 PITs.  pit[1] counter 0 (loaded via $8001/$8007) drives F000 bit 11 in the timer
	// self-test; clock estimated at 10MHz/8 = 1.25MHz (pending schematic confirmation).
	PIT8253(config, m_pit[0]);
	m_pit[0]->set_clk<0>(10_MHz_XTAL / 8);
	m_pit[0]->set_clk<1>(10_MHz_XTAL / 8);
	m_pit[0]->set_clk<2>(10_MHz_XTAL / 8);
	PIT8253(config, m_pit[1]);
	m_pit[1]->set_clk<0>(10_MHz_XTAL / 8);   // ctr0 ~19Hz system tick (count 0xFF00)
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
	m_timer_out = bool(state);                       // F000 bit11 (polled in the timer self-test)
	// ctr0 OUT is the ~20Hz system tick that SHOULD drive 68000 IRQ1 (handler 0x2b58 services the $736c
	// timeout queue, setting $7a36).  UNRESOLVED (STEP 23): asserting M68K_IRQ_1 - HOLD_LINE or
	// ASSERT_LINE, at 19Hz or ~1Hz - is never taken even with SR mask=0 during the $7a36 spin, while
	// IRQ2/IRQ4 asserted identically ARE taken.  Until that's understood, leave IRQ1 unwired.
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
	map(0x00c800, 0x00c8ff).mirror(0xff0000).rw(FUNC(multibus_storager_device::c800_r), FUNC(multibus_storager_device::c800_w));
	map(0x00d000, 0x00d001).mirror(0xff0000).w(FUNC(multibus_storager_device::d000_w));   // write-only; reads = ROM
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
