// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

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
		, m_installed(false)
		, m_trace(false)
		, m_ch{}
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
	void load_floppy();

	required_device<m68000_device> m_cpu;
	required_device_array<pit8253_device, 2> m_pit;

	bool m_installed;
	bool m_trace;            // enable disk-channel logging once a host command arrives
	u16 m_ch[0x1000];        // 0xE000-0xFFFF backing store

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
		m_installed = true;
	}
	m_cpu->set_input_line(M68K_IRQ_2, CLEAR_LINE);
}

// CPUAP reads/writes the dual-port mailbox; map byte-for-byte into the 68000's RAM at +0xC00.
u16 multibus_storager_device::host_win_r(offs_t offset)
{
	u32 const fa = 0x7e00 + offset * 2;
	address_space &cs = m_cpu->space(AS_PROGRAM);
	return cs.read_byte(fa) | (u16(cs.read_byte(fa + 1)) << 8);
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
		LOG("DOORBELL: GO @0x73F8 -> IRQ2 (mailbox fw 0x7FF8)\n");
		m_cpu->set_input_line(M68K_IRQ_2, ASSERT_LINE);
	}
	else
		LOG("HOSTWIN wr pio=%04x fw=%04x = %04x (mask %04x)\n", pio, fa, data, mem_mask);
}

// Disk read/write channel (74LS1801/1802 ENDEC + drives), logged for RE.
u16 multibus_storager_device::ch_r(offs_t offset, u16 mem_mask)
{
	u32 const a = 0xe000 + offset * 2;
	u16 const d = m_ch[offset];
	if (m_trace) LOG("  CH rd %04x = %04x [%s]\n", a, d, machine().describe_context());
	return d;
}

void multibus_storager_device::ch_w(offs_t offset, u16 data, u16 mem_mask)
{
	u32 const a = 0xe000 + offset * 2;
	COMBINE_DATA(&m_ch[offset]);
	if (m_trace) LOG("  CH wr %04x = %04x [%s]\n", a, data, machine().describe_context());

	// E802 = the IRQ2 ack the handler toggles on entry; clear the doorbell IRQ2.
	if (a == 0xe802)
		m_cpu->set_input_line(M68K_IRQ_2, CLEAR_LINE);
}

void multibus_storager_device::device_add_mconfig(machine_config &config)
{
	M68000(config, m_cpu, 50_MHz_XTAL / 4);
	m_cpu->set_addrmap(AS_PROGRAM, &multibus_storager_device::mem_map);

	PIT8253(config, m_pit[0]);
	PIT8253(config, m_pit[1]);
}

void multibus_storager_device::mem_map(address_map &map)
{
	// 8kx8 * 2 sram, 4kx1 sram, 64kx8 eprom, 32x8 * 2 prom.
	// The firmware reaches on-board I/O both A5-relative (A5=0x8000 -> 0x00xxxx) and via 68000
	// short-absolute sign-extension (0xFFxxxx), so the I/O blocks are mirrored across A16-A23.
	map(0x000000, 0x00ffff).rom().region("cpu", 0);
	map(0x004000, 0x007fff).ram();

	// two 8253 PITs interleaved at 0x8000 (#0 = even/high byte, #1 = odd/low byte)
	map(0x008000, 0x008007).mirror(0xff0000).rw(m_pit[0], FUNC(pit8253_device::read), FUNC(pit8253_device::write)).umask16(0xff00);
	map(0x008000, 0x008007).mirror(0xff0000).rw(m_pit[1], FUNC(pit8253_device::read), FUNC(pit8253_device::write)).umask16(0x00ff);

	// disk read/write channel + main control + drive status (74LS1801/1802 ENDEC + drives).
	// 0xC800 board/host control (written 0 during hw init); 0xE000-0xE01F channel; 0xE800-0xE807
	// control (E800/E802/E804/E806); 0xF000 status.  Logged for RE (ch_r/ch_w); E802 = IRQ2 ack.
	map(0x00c800, 0x00c8ff).mirror(0xff0000).ram();
	map(0x00e000, 0x00ffff).mirror(0xff0000).rw(FUNC(multibus_storager_device::ch_r), FUNC(multibus_storager_device::ch_w));
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
