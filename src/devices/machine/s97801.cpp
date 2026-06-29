// license:BSD-3-Clause
// copyright-holders:Dave Rand
/***************************************************************************

    Siemens 97801 terminal (LLE) -- core device

    The serial console for SINIX on the Siemens PC-MX2 / MX300.  Low-level emulation:
      - SAB8031 microcontroller (no internal ROM; external EPROMs d26/d21)
      - SCN2672B AVDC + SCB2673-class attribute plane + d23 char-gen (512 glyphs)
      - SCN2661B EPCI -> host (SS97 = 38400 7O1 XON/XOFF)
      - detached serial keyboard on the 8031 on-chip UART (~651 baud)
    Board: Udo's photo 97801_board_DSCN4117.jpg + the firmware RE in 97801/RESEARCH-LOG.md.
    Crystals 24.000 MHz (8031 = /2 = 12 MHz, video dot clock) + 4.9152 MHz (EPCI baud).

    Packaged as a device so it can serve as a serial terminal for any host (its real use is the
    PC-MX2 SERAD port): rxd_w() = host->terminal, txd_handler() = terminal->host.

***************************************************************************/

#include "emu.h"
#include "machine/s97801.h"

#include "screen.h"
#include "speaker.h"

#define VERBOSE 0
#include "logmacro.h"


DEFINE_DEVICE_TYPE(SIEMENS_97801, s97801_device, "s97801", "Siemens 97801 Terminal")

s97801_device::s97801_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, SIEMENS_97801, tag, owner, clock)
	, m_cpu(*this, "maincpu")
	, m_avdc(*this, "avdc")
	, m_epci(*this, "epci")
	, m_beep(*this, "beep")
	, m_chargen(*this, "chargen")
	, m_keys(*this, "KB%u", 0U)
	, m_txd_cb(*this)
{
}

void s97801_device::device_start()
{
	m_kbd_timer = timer_alloc(FUNC(s97801_device::kbd_clock), this);
	m_kbd_resp_timer = timer_alloc(FUNC(s97801_device::kbd_respond), this);
	m_kbd_scan_timer = timer_alloc(FUNC(s97801_device::kbd_scan), this);
	m_cmd_timer = timer_alloc(FUNC(s97801_device::cmd_sample), this);
	m_bell_timer = timer_alloc(FUNC(s97801_device::bell_off), this);
	save_item(NAME(m_cmd_shift));
	save_item(NAME(m_cmd_count));
	save_item(NAME(m_cmd_active));
	save_item(NAME(m_kbd_frame));
	save_item(NAME(m_kbd_bits));
	save_item(NAME(m_kbd_line));
	save_item(NAME(m_kbd_txd_last));
	save_item(NAME(m_kbd_busy));
	save_item(NAME(m_kbd_resp_pending));
	save_item(NAME(m_kbd_fifo));
	save_item(NAME(m_kbd_fhead));
	save_item(NAME(m_kbd_ftail));
	save_item(NAME(m_kbd_prev));
}

void s97801_device::device_reset()
{
	m_kbd_line = 1;
	m_kbd_bits = 0;
	m_kbd_txd_last = 1;
	m_kbd_busy = false;
	m_kbd_resp_pending = false;
	m_kbd_fhead = m_kbd_ftail = 0;
	std::fill(std::begin(m_kbd_prev), std::end(m_kbd_prev), 0);
	m_kbd_scan_timer->adjust(attotime::from_hz(200), 0, attotime::from_hz(200));
	m_cmd_active = false;
	m_beep->set_state(0);
	m_txd_cb(1); // mark idle
}

// host -> terminal serial line
void s97801_device::rxd_w(int state)
{
	m_epci->rxd_w(state);
}

// 8031 P3.0 = keyboard serial RX (the on-chip UART samples bit 0)
u8 s97801_device::cpu_p3_r()
{
	return 0xfe | (m_kbd_line & 1);
}

// E000/E001 controller status: E002 bit7=1 (normal, NOT loopback -- else the self-test at L0660
// puts the EPCI into local loopback and kills host RX); E003 bit0=0 (INT1 handler processes bytes).
u8 s97801_device::e00x_status_r(offs_t offset)
{
	return (offset == 0) ? 0x80 : 0x00;
}

void s97801_device::e00x_status_w(offs_t offset, u8 data)
{
}

// 8031 P3.1 = keyboard serial TX.  On the start bit of a terminal->keyboard command, schedule the
// keyboard's 0xAA ack (one byte-time later) and start sampling the frame so we can decode the
// command byte (BEL = 0x24 rings the beeper -- the bell physically lives in the detached keyboard).
void s97801_device::cpu_p3_w(u8 data)
{
	const int txd = BIT(data, 1);
	if (m_kbd_txd_last && !txd)
	{
		if (!m_kbd_resp_pending)
		{
			m_kbd_resp_pending = true;
			m_kbd_resp_timer->adjust(attotime::from_ticks(36864 * 11, 24_MHz_XTAL));
		}
		if (!m_cmd_active)
		{
			m_cmd_active = true;
			m_cmd_count = 0;
			m_cmd_shift = 0;
			// first data-bit centre = 1.5 bit-times after the start edge
			m_cmd_timer->adjust(attotime::from_ticks(36864 * 3 / 2, 24_MHz_XTAL));
		}
	}
	m_kbd_txd_last = txd;
}

// Sample the terminal->keyboard frame at each data-bit centre (LSB first).  On a BEL command (0x24)
// pulse the keyboard beeper for ~100 ms.
TIMER_CALLBACK_MEMBER(s97801_device::cmd_sample)
{
	if (m_kbd_txd_last)
		m_cmd_shift |= 1 << m_cmd_count;
	if (++m_cmd_count < 8)
	{
		m_cmd_timer->adjust(attotime::from_ticks(36864, 24_MHz_XTAL));
		return;
	}
	m_cmd_active = false;
	if (m_cmd_shift == 0x24) // BEL
	{
		m_beep->set_state(1);
		m_bell_timer->adjust(attotime::from_msec(100));
	}
}

TIMER_CALLBACK_MEMBER(s97801_device::bell_off)
{
	m_beep->set_state(0);
}

TIMER_CALLBACK_MEMBER(s97801_device::kbd_respond)
{
	m_kbd_resp_pending = false;
	kbd_enqueue(0xaa);
}

void s97801_device::kbd_enqueue(u8 b)
{
	const u8 nt = (m_kbd_ftail + 1) % sizeof(m_kbd_fifo);
	if (nt == m_kbd_fhead) return; // full -> drop
	m_kbd_fifo[m_kbd_ftail] = b;
	m_kbd_ftail = nt;
	if (!m_kbd_busy)
	{
		m_kbd_busy = true;
		m_kbd_timer->adjust(attotime::zero);
	}
}

// Keyboard serial TX bit-clock: bit period = (24MHz/2)/18432 = 36864 ticks of the 24MHz osc
// = ~651 baud (firmware TH1=0xD0).  Shifts the 8N1 frame, then pulls the next FIFO byte.
TIMER_CALLBACK_MEMBER(s97801_device::kbd_clock)
{
	const attotime bit = attotime::from_ticks(36864, 24_MHz_XTAL);
	if (m_kbd_bits != 0)
	{
		m_kbd_line = m_kbd_frame & 1;
		m_kbd_frame >>= 1;
		m_kbd_bits--;
		m_kbd_timer->adjust(bit);
		return;
	}
	if (m_kbd_fhead != m_kbd_ftail)
	{
		const u8 b = m_kbd_fifo[m_kbd_fhead];
		m_kbd_fhead = (m_kbd_fhead + 1) % sizeof(m_kbd_fifo);
		m_kbd_frame = (u16(b) << 1) | (1 << 9);
		m_kbd_bits = 10;
		m_kbd_timer->adjust(bit);
		return;
	}
	m_kbd_line = 1;
	m_kbd_busy = false;
}

// Poll the key matrix; on each make/break edge, enqueue the Platzcode (make = code, break = code|0x80).
TIMER_CALLBACK_MEMBER(s97801_device::kbd_scan)
{
	static const u8 s_platzcode[6][8] = {
		{ 0x01,0x41,0x42,0x43,0x44,0x45,0x46,0x47 }, // Shift, a b c d e f g
		{ 0x48,0x49,0x4a,0x4b,0x4c,0x4d,0x4e,0x4f }, // h i j k l m n o
		{ 0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57 }, // p q r s t u v w
		{ 0x58,0x59,0x5a,0x30,0x31,0x32,0x33,0x34 }, // x y z 0 1 2 3 4
		{ 0x35,0x36,0x37,0x38,0x39,0x20,0x0d,0x08 }, // 5 6 7 8 9 Space Return Backspace
		{ 0x0b,0x2e,0x2c,0x2f,0x27,0xff,0xff,0xff }, // Tab . , - /
	};
	for (int p = 0; p < 6; p++)
	{
		const u8 cur = m_keys[p]->read();
		const u8 chg = cur ^ m_kbd_prev[p];
		if (chg)
		{
			for (int b = 0; b < 8; b++)
				if (BIT(chg, b) && s_platzcode[p][b] != 0xff)
					kbd_enqueue(BIT(cur, b) ? s_platzcode[p][b] : u8(s_platzcode[p][b] | 0x80));
			m_kbd_prev[p] = cur;
		}
	}
}

void s97801_device::prg_map(address_map &map)
{
	map(0x0000, 0x1fff).rom().region("maincpu", 0); // d26
	map(0x2000, 0x3fff).rom().region("unk", 0);     // d21
}

void s97801_device::data_map(address_map &map)
{
	// 8031 external data bus (MOVX) -- firmware I/O map (97801/RESEARCH-LOG.md)
	map(0x0000, 0x7fff).ram();                      // HM62256 32Kx8 work RAM @ D34 (A15 = RAM/device select)
	map(0xc000, 0xc007).rw(m_avdc, FUNC(scn2672_device::read), FUNC(scn2672_device::write)); // AVDC
	// E000/E001 = the char/attr display-plane data latches; the AVDC write cmds (0xA2/0xAB/0xBB)
	// supply the RAM address + write cycle while these latches drive the data buses, so route them
	// to the AVDC char/attr buffer (the fix for "no visible firmware text").
	map(0xe000, 0xe000).rw(m_avdc, FUNC(scn2672_device::buffer_r), FUNC(scn2672_device::buffer_w));
	map(0xe001, 0xe001).rw(m_avdc, FUNC(scn2672_device::attr_buffer_r), FUNC(scn2672_device::attr_buffer_w));
	map(0xe002, 0xe003).rw(FUNC(s97801_device::e00x_status_r), FUNC(s97801_device::e00x_status_w));
	// host EPCI (SCN2661): only A0/A1 decoded (8000 RX / 8004 TX / 8001 status / 8006 mode / 8007 cmd)
	map(0x8000, 0x8003).mirror(0x0004).rw(m_epci, FUNC(scn2661b_device::read), FUNC(scn2661b_device::write));
}

void s97801_device::char_map(address_map &map)
{
	map(0x0000, 0x3fff).ram(); // Charakter refresh plane (D4364C @ D11); AVDC scans it
}

void s97801_device::attr_map(address_map &map)
{
	map(0x0000, 0x3fff).ram(); // Attribut plane (HM6116 D13/D20) -> SCB2673
}

SCN2672_DRAW_CHARACTER_MEMBER(s97801_device::draw_character)
{
	// SCB2673 attribute byte (this board's wiring, from the firmware SGR handler sub_129A; base 0x08):
	//   b0 blank, b1 blink, b2 underline, b3 intensity(1=normal/0=dim), b4 reverse,
	//   b5 alt/graphics charset bank -> d23 upper 256-glyph (line-draw "w" set, SO/SI via 0x0400 table).
	const bool a_blank = BIT(attrcode, 0);
	const bool a_blink = BIT(attrcode, 1);
	const bool a_ul    = BIT(attrcode, 2);
	const bool a_norm  = BIT(attrcode, 3);
	const bool a_rvid  = BIT(attrcode, 4);
	const bool a_bank  = BIT(attrcode, 5);

	const unsigned glyph = (a_bank ? 0x100 : 0) | charcode;
	u16 dots = (a_blank || (a_blink && blink)) ? 0 : u16(m_chargen[(glyph << 4) | linecount]) << 1;
	if (a_ul && ul)
		dots = 0x1ff;
	if (a_rvid)
		dots = ~dots;
	if (cursor)
		dots = ~dots;

	const rgb_t fg = a_norm ? rgb_t::white() : rgb_t(0x80, 0x80, 0x80);
	for (int i = 0; i < 9; i++)
	{
		bitmap.pix(y, x++) = BIT(dots, 8) ? fg : rgb_t::black();
		dots <<= 1;
	}
}

// Keyboard matrix (natural-keyboard via PORT_CHAR).  Bit order MUST match s_platzcode[][].
static INPUT_PORTS_START(s97801)
	PORT_START("KB0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Shift") PORT_CHAR(UCHAR_SHIFT_1)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('a') PORT_CHAR('A')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('b') PORT_CHAR('B')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('c') PORT_CHAR('C')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('d') PORT_CHAR('D')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('e') PORT_CHAR('E')
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('f') PORT_CHAR('F')
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('g') PORT_CHAR('G')

	PORT_START("KB1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('h') PORT_CHAR('H')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('i') PORT_CHAR('I')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('j') PORT_CHAR('J')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('k') PORT_CHAR('K')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('l') PORT_CHAR('L')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('m') PORT_CHAR('M')
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('n') PORT_CHAR('N')
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('o') PORT_CHAR('O')

	PORT_START("KB2")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('p') PORT_CHAR('P')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('q') PORT_CHAR('Q')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('r') PORT_CHAR('R')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('s') PORT_CHAR('S')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('t') PORT_CHAR('T')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('u') PORT_CHAR('U')
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('v') PORT_CHAR('V')
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('w') PORT_CHAR('W')

	PORT_START("KB3")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('x') PORT_CHAR('X')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('y') PORT_CHAR('Y')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('z') PORT_CHAR('Z')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('0')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('1')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('2')
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('3')
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('4')

	PORT_START("KB4")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('5')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('6')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('7')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('8')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('9')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR(' ')
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Return") PORT_CHAR(13)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Backspace") PORT_CHAR(8)

	PORT_START("KB5")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tab") PORT_CHAR(9)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('.')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR(',')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('-')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CHAR('/')
	PORT_BIT(0xe0, IP_ACTIVE_HIGH, IPT_UNUSED)
INPUT_PORTS_END

ioport_constructor s97801_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(s97801);
}

void s97801_device::device_add_mconfig(machine_config &config)
{
	I8031(config, m_cpu, 24_MHz_XTAL / 2); // SAB8031P @ 12 MHz (board osc 24.000 MHz / 2)
	m_cpu->set_addrmap(AS_PROGRAM, &s97801_device::prg_map);
	m_cpu->set_addrmap(AS_DATA, &s97801_device::data_map);
	m_cpu->port_in_cb<3>().set(FUNC(s97801_device::cpu_p3_r));  // P3.0 = keyboard RX
	m_cpu->port_out_cb<3>().set(FUNC(s97801_device::cpu_p3_w)); // P3.1 = keyboard TX

	screen_device &screen(SCREEN(config, "screen", SCREEN_TYPE_RASTER));
	screen.set_color(rgb_t::green());
	screen.set_raw(24_MHz_XTAL, 912, 0, 640, 423, 0, 400);
	screen.set_screen_update(m_avdc, FUNC(scn2672_device::screen_update));

	SCN2672(config, m_avdc, 24_MHz_XTAL / 8);
	m_avdc->set_screen("screen");
	m_avdc->set_character_width(8);
	m_avdc->set_addrmap(0, &s97801_device::char_map);
	m_avdc->set_addrmap(1, &s97801_device::attr_map);
	m_avdc->set_display_callback(FUNC(s97801_device::draw_character));
	m_avdc->intr_callback().set_inputline(m_cpu, MCS51_INT0_LINE);

	SCN2661B(config, m_epci, 4.9152_MHz_XTAL);
	m_epci->rxrdy_handler().set_inputline(m_cpu, MCS51_INT1_LINE);
	m_epci->txd_handler().set(FUNC(s97801_device::epci_txd_w)); // terminal -> host

	SPEAKER(config, "mono").front_center();
	BEEP(config, m_beep, 1500).add_route(ALL_OUTPUTS, "mono", 0.25); // keyboard bell (BEL/^G)
}

ROM_START(s97801)
	ROM_REGION(0x2000, "maincpu", 0)
	ROM_LOAD("010_d26__0118_04.d26", 0x0000, 0x2000, CRC(fcf045d7) SHA1(4a98e7d2d98272970d627ce5c10e9572b87293d1))

	ROM_REGION(0x2000, "unk", 0)
	ROM_LOAD("010_d21__0118_04.d21", 0x0000, 0x2000, CRC(b9b9df32) SHA1(9a3ba060ebcf00b1ed9112493a1d73212c04d8e5))

	ROM_REGION(0x2000, "chargen", 0)
	ROM_LOAD("010_d23__0118_03.d23", 0x0000, 0x2000, CRC(23b22a7d) SHA1(649abcfde9752f427ec7d1efdc013a4f01dc271c))

	ROM_REGION(0x1000, "kbd", 0) // keyboard Platzcode table (used by the firmware's own keyboard MCU; HLE here)
	ROM_LOAD("p26361_k111_v1_3.d3", 0x0000, 0x1000, CRC(aba8f4b7) SHA1(970a45b509081603e25319e1bbf0f7941f91a056))
ROM_END

const tiny_rom_entry *s97801_device::device_rom_region() const
{
	return ROM_NAME(s97801);
}
