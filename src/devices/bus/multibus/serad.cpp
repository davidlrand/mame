// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
 * Siemens S26361-D279 SERAD serial interface board.
 *
 * This Multibus card was used with several Multibus-based computers from
 * Siemens, including the PC-MX2, MX300 and MX500. It has 3 SCN2681 DUARTs,
 * giving a total of two V24 and four SS97 serial ports. These are controlled
 * by an on-board 8085A microcontroller, with a dual-port RAM mailbox used to
 * communicate with the host.
 *
 * Sources:
 *  - https://oldcomputers-ddns.org/public/pub/rechner/siemens/mx-rm/pc-mx2/manuals/pc-mx2_pc2000_9780_logik.pdf
 *  - https://mx300i.narten.de/view_board.cfm?5EF287A1ABC3F4DCAFEA2BC2FAB8C504041A
 *
 * TODO:
 *  - multibus lock/unlock/interrupt
 *  - SS97 ports with power on/reset signals
 */

#include "emu.h"
#include "serad.h"

#define VERBOSE 0
#include "logmacro.h"

DEFINE_DEVICE_TYPE(SERAD, serad_device, "serad", "Siemens S26361-D279 SERAD")

serad_device::serad_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, SERAD, tag, owner, clock)
	, device_multibus_interface(mconfig, *this)
	, m_cpu(*this, "cpu")
	, m_mbx(*this, "mbx")
	, m_rst65(*this, "rst65")
	, m_duart(*this, "duart%u", 0U)
	, m_port(*this, "port%u", 0U)
	, m_installed(false)
{
}

ROM_START(serad)
	ROM_REGION(0x2000, "cpu", 0)
	ROM_LOAD("361d0279d031__e00422_tex.d31", 0x0000, 0x2000, CRC(369f5fd1) SHA1(6a1c2d351d5552d54d835b7726b3f9b921605d0e))
ROM_END

// firmware default port configuration: CSR 0xDD = counter/timer baud (C/T preset 3), Siemens
// house convention 38400 7O1 (same as the CPUAP diag console)
static DEVICE_INPUT_DEFAULTS_START(ss97_defaults)
	DEVICE_INPUT_DEFAULTS("RS232_TXBAUD", 0xff, RS232_BAUD_38400)
	DEVICE_INPUT_DEFAULTS("RS232_RXBAUD", 0xff, RS232_BAUD_38400)
	DEVICE_INPUT_DEFAULTS("RS232_DATABITS", 0xff, RS232_DATABITS_7)
	DEVICE_INPUT_DEFAULTS("RS232_PARITY", 0xff, RS232_PARITY_ODD)
	DEVICE_INPUT_DEFAULTS("RS232_STOPBITS", 0xff, RS232_STOPBITS_1)
DEVICE_INPUT_DEFAULTS_END

static INPUT_PORTS_START(serad)
	PORT_START("MEM")
	PORT_DIPNAME(0xfff, 0xef7, "Base Address")
	PORT_DIPSETTING(0xef7, "EF7000")
	PORT_DIPSETTING(0xef6, "EF6000")
	PORT_DIPSETTING(0xef5, "EF5000")
	PORT_DIPSETTING(0xef4, "EF4000")
	PORT_DIPSETTING(0xef3, "EF3000")

	// S9-S16 select the Multibus interrupt level (S9=INT0 .. S16=INT7, one closed; the MX300
	// runs polled with all open). The MX2's SINIX serad driver services the board on ICU IR4
	// ("vector 4 ipl 5" in its probe banner) = Multibus INT3.
	PORT_START("INT")
	PORT_DIPNAME(0xff, 0x08, "Interrupt Level")
	PORT_DIPSETTING(0x00, "None (polled)")
	PORT_DIPSETTING(0x01, "INT0")
	PORT_DIPSETTING(0x02, "INT1")
	PORT_DIPSETTING(0x04, "INT2")
	PORT_DIPSETTING(0x08, "INT3")
	PORT_DIPSETTING(0x10, "INT4")
	PORT_DIPSETTING(0x20, "INT5")
	PORT_DIPSETTING(0x40, "INT6")
	PORT_DIPSETTING(0x80, "INT7")

	PORT_START("PIO")
	PORT_DIPNAME(0xff, 0x10, "I/O Address")
	PORT_DIPSETTING(0x0f, "0F00")
	PORT_DIPSETTING(0x10, "1000")
	PORT_DIPSETTING(0x11, "1100")
	PORT_DIPSETTING(0x12, "1200")
	PORT_DIPSETTING(0x13, "1300")

	PORT_START("IRQ")
	PORT_DIPNAME(0xff, 0x00, "Interrupt")
	PORT_DIPSETTING(0x00, "None")
	PORT_DIPSETTING(0x01, "0")
	PORT_DIPSETTING(0x02, "1")
	PORT_DIPSETTING(0x04, "2")
	PORT_DIPSETTING(0x08, "3")
	PORT_DIPSETTING(0x10, "4")
	PORT_DIPSETTING(0x20, "5")
	PORT_DIPSETTING(0x40, "6")
	PORT_DIPSETTING(0x80, "7")
INPUT_PORTS_END

const tiny_rom_entry *serad_device::device_rom_region() const
{
	return ROM_NAME(serad);
}

ioport_constructor serad_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(serad);
}

void serad_device::device_start()
{
	m_hostint_rel = timer_alloc(FUNC(serad_device::hostint_release), this);
	save_item(NAME(m_buslock));
	save_item(NAME(m_lockpend));
	save_item(NAME(m_lockpend_n));
	// TEMP (STRIP): manual host-int strobe test - does the vector-4 ISR wake init's blocked ioctl?
	if (getenv("SERAD_KICK"))
	{
		m_kick = timer_alloc(FUNC(serad_device::kick_test), this);
		m_kick->adjust(attotime::from_usec(16000700));
	}
	// TEMP (STRIP): mbx state dump at 8.5s (the ring-service gate inputs)
	m_dump = timer_alloc(FUNC(serad_device::mbx_dump), this);
	m_dump->adjust(attotime::from_usec(8500000));

	// TEMP PROBE (STRIP): the console port is a pty, which drives no modem control lines, so CTS-A
	// (ip0, the fw ring/cell TX-pump gate) and DCD (ip2, the tty-open carrier) never assert -> the fw
	// never posts mbx+0xA0 bit0 -> the kernel tty driver (mod 0x220) spins to a watchdog reboot.
	// SERAD_CTS asserts them periodically (value = the ip0 level to try: fw gate wants IP bit0 = 0).
	if (getenv("SERAD_CTS"))
	{
		m_cts = timer_alloc(FUNC(serad_device::cts_probe), this);
		m_cts->adjust(attotime::from_msec(100), 0, attotime::from_msec(100));
	}
}

TIMER_CALLBACK_MEMBER(serad_device::cts_probe)
{
	int const lvl = atoi(getenv("SERAD_CTS"));
	m_duart[0]->ip0_w(lvl);              // CTS-A (stable): fw TX pump gated on ~IP bit0
	// one-shot DCD delta -> asserted (carrier for tty-open), then hold; NO toggling (that = hangup storms)
	if (!m_cts_dcd) { m_cts_dcd = 1; m_duart[0]->ip2_w(0); m_duart[0]->ip2_w(1); }
}

TIMER_CALLBACK_MEMBER(serad_device::mbx_dump)
{
	auto &sp = m_cpu->space(AS_PROGRAM);
	logerror("MBXDUMP en4=%02x en5=%02x sum2=%02x sum3=%02x ev6=%02x ring12=%02x%02x ring14=%02x%02x cell[a0]=%02x @%.4f\n",
		sp.read_byte(0x4004), sp.read_byte(0x4005), sp.read_byte(0x4002), sp.read_byte(0x4003),
		sp.read_byte(0x4006), sp.read_byte(0x4013), sp.read_byte(0x4012), sp.read_byte(0x4015), sp.read_byte(0x4014),
		sp.read_byte(0x40a0), machine().time().as_double());
}

TIMER_CALLBACK_MEMBER(serad_device::kick_test)
{
	logerror("KICK: manual host-int strobe @%.4f\n", machine().time().as_double());
	hostint_w(0);
}

void serad_device::device_reset()
{
	// Unmapped-access logging only; no behavioural change.  The 8085's probes flood a -oslog run
	// and throttle it to a few seconds of emulated time (see cpuap.cpp for the same suppression).
	for (int sp = 0; sp < 4; sp++)
		if (m_cpu->has_space(sp))
			m_cpu->space(sp).set_log_unmap(false);
	// the Multibus lock is a bus-arbitration latch cleared by INIT: drop it and flush any
	// deferred host writes, or a soft reset taken mid-lock leaves every subsequent host
	// mailbox write deferred forever (console dead after reset)
	if (m_lockpend_n)
		buslock_w<false>(0);
	m_buslock = false;
	// an unplugged serial port idles at MARK, not break: MAME's empty rs232_port never drives
	// the line (initial state 0), and with the duarts' receivers running at real baud a floating
	// low reads as continuous BREAK - the firmware queues line events and the ready word never
	// comes up clean (monitor handoff checks mbx[2..3] == 0x8000 exactly)
	// drive the strapped Multibus interrupt line to its idle (deasserted) level: the host
	// ICU's edge detector latches on the 1->0 transition, and a line that has never been
	// driven high can never produce one
	{
		u8 const sel = ioport("INT")->read();
		if (sel)
			int_w(unsigned(31 - __builtin_clz(u32(sel))), 1);
	}
	if (!m_port[0]->get_card_device()) m_duart[0]->rx_a_w(1);
	if (!m_port[1]->get_card_device()) m_duart[0]->rx_b_w(1);
	if (!m_port[2]->get_card_device()) m_duart[1]->rx_a_w(1);
	if (!m_port[3]->get_card_device()) m_duart[1]->rx_b_w(1);
	if (!m_port[4]->get_card_device()) m_duart[2]->rx_a_w(1);
	if (!m_port[5]->get_card_device()) m_duart[2]->rx_b_w(1);

	if (!m_installed)
	{
		u32 const mem = ioport("MEM")->read() << 12;
		u32 const pio = ioport("PIO")->read() << 8;

		// reads direct from the share; writes through mbx_w so the fw's Multibus lock
		// (OUT $40/$50) can hold them off during its shared-cell read-modify-writes
		m_bus->space(AS_PROGRAM).install_rom(mem, mem | 0xfff, m_mbx.target());
		m_bus->space(AS_PROGRAM).install_write_handler(mem, mem | 0xfff,
				write16s_delegate(*this, FUNC(serad_device::mbx_w)));
		m_bus->space(AS_IO).install_write_handler(pio, pio | 0xff, write8smo_delegate(*this, &serad_device::rst55_w<1>, "serad_device::rst55_w"));
		// TEMP LOG (STRIP): the PIO doorbell ring times
		m_bus->space(AS_IO).install_write_tap(pio, pio | 0xff, "dbell",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				double const t = machine().time().as_double();
				static int n = 0;
				if (t > 5.0 && n++ < 40) logerror("HOST doorbell (pio %02x) = %04x @%.5f\n", offset & 0xff, data, t);
			});

		// TEMP LOG (STRIP): watch the firmware answer the mailbox (srinit expects word 0x8000 at mbx+2)
		m_cpu->space(AS_PROGRAM).install_write_tap(0x4000, 0x4007, "mbxw",
			[this](offs_t offset, u8 &data, u8 mem_mask)
			{
				logerror("8085 mbx[%x] = %02x @%.5f\n", offset & 0xfff, data, machine().time().as_double());
			});
		// TEMP LOG (STRIP): the fw main loop spins on a test-and-set SEMAPHORE at mbx[0x9C]
		// (LXI H,409C / LOCK / MOV A,M / MVI M,1 / UNLOCK / ORA / JNZ). Watch both sides of it.
		m_cpu->space(AS_PROGRAM).install_readwrite_tap(0x4098, 0x40a3, "semtap",
			[this](offs_t offset, u8 &data, u8 mem_mask)
			{
				if (!machine().side_effects_disabled())
				{
					static int n = 0; static u8 last = 0xee;
					if (offset == 0x409c && data != last && n++ < 60) { logerror("8085 sem rd [%03x] = %02x @%.5f\n", offset & 0xfff, data, machine().time().as_double()); last = data; }
				}
			},
			[this](offs_t offset, u8 &data, u8 mem_mask)
			{
				static int n = 0;
				if (offset != 0x409c && n++ < 60) logerror("8085 mbxhi WR [%03x] = %02x @%.5f\n", offset & 0xfff, data, machine().time().as_double());
			});
		// the host acknowledges a board interrupt by clearing the event summary in the mailbox
		// head - release the held interrupt line on any head write
		m_bus->space(AS_PROGRAM).install_write_tap(0xef7000, 0xef7007, "hostack",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				hostint_ack();
			});

		// TEMP LOG (STRIP): the srinit handshake timing - host accesses to the mailbox head + the
		// fw's ready-word posts at mbx+2/+3
		m_bus->space(AS_PROGRAM).install_readwrite_tap(0xef7000, 0xef7fff, "hosthd",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				if (!machine().side_effects_disabled())
				{
					double const t = machine().time().as_double();
					static int n = 0;
					static offs_t lastoff = ~0u;
					if (t > 15.5 && (offset != lastoff) && n++ < 60) { logerror("HOST mbx rd [%03x] -> %04x/%04x (%s) @%.5f\n", offset & 0xfff, data, mem_mask, machine().describe_context(), t); lastoff = offset; }
				}
			},
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				double const t = machine().time().as_double();
				static int n = 0;
				// TEMP (STRIP): fs-full console-write anchor - log the mailbox char stream + CPUAP PC around fs-full (~t=13)
				if (t > 12.0 && (offset & 0xfff) != 0xa0 && n++ < 400)
				{
					u8 const lo = data & 0xff, hi = data >> 8;
					logerror("MBXWR [%03x]=%04x '%c%c' (%s) @%.5f\n", offset & 0xfff, data,
						(lo >= 32 && lo < 127) ? lo : '.', (hi >= 32 && hi < 127) ? hi : '.', machine().describe_context(), t);
				}
			});
		// TEMP LOG (STRIP): attention-handler execution proof - $8013 = bit1/$0FFA's mask store,
		// $8000/$8006 = bit6/$1B85's vector stores
		m_cpu->space(AS_PROGRAM).install_write_tap(0x8000, 0x8013, "fwattn",
			[this](offs_t offset, u8 &data, u8 mem_mask)
			{
				u16 const a = offset & 0xffff;
				u16 const pc = m_cpu->pc();
				if ((a == 0x8013 || a == 0x8000 || a == 0x8006) && (pc < 0x0e40 || pc > 0x0e50))
				{
					double const t = machine().time().as_double();
					static int n = 0;
					if (t > 8.5 && n++ < 20) logerror("8085 attn-var [%04x] = %02x (pc=%04x) @%.5f\n", a, data, pc, t);
				}
			});
		// TEMP LOG (STRIP): who reads the channel descriptor head/tail cells (mbx[0x12-0x15]) -
		// the ring-service routine's signature. The background scrub (pc 0x0E4A-0x0E4D) is excluded.
		// TEMP LOG (STRIP): does the fw poll the event word mbx[4]?
		m_cpu->space(AS_PROGRAM).install_read_tap(0x4004, 0x4005, "fwevt",
			[this](offs_t offset, u8 &data, u8 mem_mask)
			{
				if (!machine().side_effects_disabled())
				{
					u16 const pc = m_cpu->pc();
					if (pc >= 0x0e40 && pc <= 0x0e50) return;
					double const t = machine().time().as_double();
					static int n = 0;
					if (t > 6.5 && n++ < 30) logerror("8085 evt rd [%03x] -> %02x (pc=%04x) @%.5f\n", offset & 0xfff, data, pc, t);
				}
			});
		m_cpu->space(AS_PROGRAM).install_read_tap(0x4012, 0x4015, "fwdesc",
			[this](offs_t offset, u8 &data, u8 mem_mask)
			{
				if (!machine().side_effects_disabled())
				{
					u16 const pc = m_cpu->pc();
					if (pc >= 0x0e40 && pc <= 0x0e50) return;
					double const t = machine().time().as_double();
					static int n = 0;
					if (t > 8.258 && n++ < 40) logerror("8085 desc rd [%03x] -> %02x (pc=%04x) @%.5f\n", offset & 0xfff, data, pc, t);
				}
			});
		m_cpu->space(AS_PROGRAM).install_write_tap(0x4000, 0x4003, "fwpost",
			[this](offs_t offset, u8 &data, u8 mem_mask)
			{
				static int n = 0;
				if (n++ < 40) logerror("8085 mbx WR [%x] = %02x @%.5f\n", offset & 0xf, data, machine().time().as_double());
			});
		// TEMP LOG (STRIP): the fw's kernel-era RX->cell forwarding - [A1]/[A2] writes by the 8085
		m_cpu->space(AS_PROGRAM).install_write_tap(0x40a1, 0x40a2, "fwcell",
			[this](offs_t offset, u8 &data, u8 mem_mask)
			{
				u16 const pc = m_cpu->pc();
				if (pc >= 0x0e40 && pc <= 0x0e50) return;   // the scrub
				double const t = machine().time().as_double();
				static int n = 0;
				if (t > 8.0 && n++ < 20) logerror("8085 cell wr [%03x] = %02x (pc=%04x) @%.5f\n", offset & 0xfff, data, pc, t);
			});
		// TEMP LOG (STRIP): the fw's duart READS during the init self-test (SR/RHR polls with pc)
		m_cpu->space(AS_IO).install_read_tap(0x00, 0x0f, "d0rd",
			[this](offs_t offset, u8 &data, u8 mem_mask)
			{
				if (!machine().side_effects_disabled())
				{
					double const t = machine().time().as_double();
					unsigned const r = offset & 0xff;
					static int n = 0;
					if (t > 0.15 && t < 0.45 && (r == 0x01 || r == 0x03 || r == 0x09 || r == 0x0b || r == 0x0d) && n++ < 100)
						logerror("8085 d0rd[%02x] -> %02x (pc=%04x) @%.6f\n", r, data, m_cpu->pc(), t);
				}
			});
		// TEMP LOG (STRIP): duart0 ch-A RX reads + THR writes in the input-test era (RX chain proof)
		m_cpu->space(AS_IO).install_readwrite_tap(0x00, 0x0f, "d0all",
			[this](offs_t offset, u8 &data, u8 mem_mask)
			{
				if (!machine().side_effects_disabled() && (offset & 0xff) == 0x03)
				{
					double const t = machine().time().as_double();
					static int n = 0;
					if (t > 8.258 && n++ < 40)
						logerror("8085 RHR io03 -> %02x '%c' (pc=%04x) @%.5f\n", data, (data >= 0x20 && data < 0x7f) ? data : '.', m_cpu->pc(), t);
				}
				if (!machine().side_effects_disabled() && (offset & 0xff) == 0x0d)
				{
					double const t = machine().time().as_double();
					static int m = 0;
					if (t > 8.258 && m++ < 20)
						logerror("8085 IP io0D -> %02x (pc=%04x) @%.5f\n", data, m_cpu->pc(), t);
				}
			},
			[this](offs_t offset, u8 &data, u8 mem_mask)
			{
				double const t = machine().time().as_double();
				unsigned const r = offset & 0xff;
				static int n = 0;
				if ((r == 0x03 || r == 0x0b) && t > 8.258 && n++ < 80)
					logerror("8085 THR io%02X = %02x '%c' (pc=%04x) @%.5f\n", r, data, (data >= 0x20 && data < 0x7f) ? data : '.', m_cpu->pc(), t);
				static int m = 0;
				if (r == 0x05 && m++ < 20)
					logerror("8085 IMR io05 = %02x (pc=%04x) @%.5f\n", data, m_cpu->pc(), t);
			});
		// TEMP LOG (STRIP): the fw's DUART-0 register programming (baud/framing for the console port)
		m_cpu->space(AS_IO).install_write_tap(0x00, 0x2f, "d0wr",
			[this](offs_t offset, u8 &data, u8 mem_mask)
			{
				double const t = machine().time().as_double();
				unsigned const r = offset & 0xff;
				static int n = 0;
				// THR writes + CR (command register) writes on duart0
				if ((r == 0x02 || r == 0x0a) && n++ < 120)
					logerror("8085 CR io%02X = %02x (pc=%04x) @%.5f\n", r, data, m_cpu->pc(), t);
				else if (t > 5.5 && (r == 0x03 || r == 0x0b || r == 0x13 || r == 0x1b || r == 0x23 || r == 0x2b) && n++ < 120)
					logerror("8085 TX io%02X = %02x @%.5f\n", r, data, t);
			});

		// TEMP PROBE (STRIP): SERAD_CH0CFG=1 -> at t=7s inject a monitor-style channel-0 config
		// (attention 0x02, mask 0x01) to test the hypothesis that the kernel expects ch0's config
		// to survive its restart (the fw's pc-0x1163 leaves unconfigured channels CR=0x0A disabled)
		if (getenv("SERAD_CH0CFG"))
			machine().scheduler().timer_set(attotime::from_seconds(7), timer_expired_delegate(FUNC(serad_device::ch0cfg_probe), this));

		m_installed = true;
	}
}

TIMER_CALLBACK_MEMBER(serad_device::ch0cfg_probe)
{
	m_mbx[1] = 0x01;
	m_mbx[0] = 0x02;
	m_cpu->set_input_line(I8085_RST55_LINE, 1);   // held, like the host doorbell write
	logerror("PROBE: injected ch0 config (attention 0x02 mask 0x01) @%.4f\n", machine().time().as_double());
}

DEFINE_DEVICE_TYPE(SERAD_SCN2681, serad_scn2681_device, "serad_scn2681", "Siemens SERAD SCN2681")

serad_scn2681_device::serad_scn2681_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: scn2681_device(mconfig, tag, owner, clock)
{
	// keep the shared type's device id semantics; only board-specific behavior differs
}

int serad_scn2681_device::calc_baud(int ch, bool rx, uint8_t data)
{
	if ((data & 0xf) == 0xd)
	{
		// CSR select 0xD = the counter/timer output is the channel's 16x clock: the core model
		// drives tx/rx_16x_clock_w from the C/T tick (ACR=0xE0, CTR=3: 3.6864 MHz / (2 * 3)
		// / 16 = 38400, the SS97 console rate). The internal bit-rate clock must be DISABLED
		// (rate 0) or two clocks fight over the receive machinery and every frame misframes
		return 0;
	}
	return scn2681_device::calc_baud(ch, rx, data);
}

void serad_device::device_add_mconfig(machine_config &config)
{
	// P8085AH-2 at 5 MHz: the 20.000 MHz oscillator is halved to the 8085's 10 MHz X1 input. (The SINIX
	// kernel's srinit ready-window is met by the CPUAP's DRAM wait-state timing, not by overspeeding this.)
	I8085A(config, m_cpu, 20_MHz_XTAL / 2);
	m_cpu->set_addrmap(AS_PROGRAM, &serad_device::mem_map);
	m_cpu->set_addrmap(AS_IO, &serad_device::pio_map);

	INPUT_MERGER_ANY_HIGH(config, m_rst65);
	m_rst65->output_handler().set_inputline(m_cpu, I8085_RST65_LINE);

	SERAD_SCN2681(config, m_duart[0], 7.3728_MHz_XTAL / 2);
	SERAD_SCN2681(config, m_duart[1], 7.3728_MHz_XTAL / 2);
	SERAD_SCN2681(config, m_duart[2], 7.3728_MHz_XTAL / 2);

	m_duart[0]->irq_cb().set(
		[this](int state)
		{
			// TEMP LOG (STRIP): duart0 irq edges in the input-test era
			{
				double const t = machine().time().as_double();
				static int n = 0;
				if (t > 11.0 && n++ < 12) logerror("duart0 irq = %d @%.5f\n", state, t);
			}
			m_rst65->in_w<0>(state);
		});
	m_duart[1]->irq_cb().set(m_rst65, FUNC(input_merger_any_high_device::in_w<1>));
	m_duart[2]->irq_cb().set(m_rst65, FUNC(input_merger_any_high_device::in_w<2>));

	RS232_PORT(config, m_port[0], default_rs232_devices, "s97801"); // SS97: port 0 is the system console; a 97801 terminal is the standard fit
	m_port[0]->set_option_device_input_defaults("null_modem", DEVICE_INPUT_DEFAULTS_NAME(ss97_defaults));
	m_port[0]->set_option_device_input_defaults("terminal", DEVICE_INPUT_DEFAULTS_NAME(ss97_defaults));
	m_port[0]->set_option_device_input_defaults("pty", DEVICE_INPUT_DEFAULTS_NAME(ss97_defaults));
	RS232_PORT(config, m_port[1], default_rs232_devices, nullptr); // SS97
	RS232_PORT(config, m_port[2], default_rs232_devices, nullptr); // V24
	RS232_PORT(config, m_port[3], default_rs232_devices, nullptr); // SS97
	RS232_PORT(config, m_port[4], default_rs232_devices, nullptr); // SS97
	RS232_PORT(config, m_port[5], default_rs232_devices, nullptr); // V24

	m_duart[0]->a_tx_cb().set(m_port[0], FUNC(rs232_port_device::write_txd));
	m_duart[0]->b_tx_cb().set(m_port[1], FUNC(rs232_port_device::write_txd));
	m_duart[0]->outport_cb().set(
		[this](u8 data)
		{
			// TEMP LOG (STRIP): the TXRDY/RXRDY crosswires gate the fw's console TX pump
			{ static u8 last = 0xee; if (data != last) { logerror("duart0 outport = %02x @%.4f\n", data, machine().time().as_double()); last = data; } }
			// clear 0
			// clear 1
			// -
			// -
			m_duart[1]->ip0_w(BIT(data, 4)); // rxrdy0
			m_duart[1]->ip1_w(BIT(data, 5)); // rxrdy1
			m_duart[0]->ip0_w(BIT(data, 6)); // txrdy0
			m_duart[0]->ip1_w(BIT(data, 7)); // txrdy1
		}
	);

	m_duart[1]->a_tx_cb().set(m_port[2], FUNC(rs232_port_device::write_txd));
	m_duart[1]->b_tx_cb().set(m_port[3], FUNC(rs232_port_device::write_txd));
	m_duart[1]->outport_cb().set(
		[this](u8 data)
		{
			m_port[2]->write_rts(BIT(data, 0)); // s2.k2 (rts)
			// clear 3
			// -
			m_port[2]->write_dtr(BIT(data, 3)); // s1.k2 (dtr)
			m_duart[1]->ip2_w(BIT(data, 4)); // rxrdy2
			m_duart[1]->ip3_w(BIT(data, 5)); // rxrdy3
			m_duart[0]->ip2_w(BIT(data, 6)); // txrdy2
			m_duart[0]->ip3_w(BIT(data, 7)); // txrdy3
		}
	);

	m_duart[2]->a_tx_cb().set(m_port[4], FUNC(rs232_port_device::write_txd));
	m_duart[2]->b_tx_cb().set(m_port[5], FUNC(rs232_port_device::write_txd));
	m_duart[2]->outport_cb().set(
		[this](u8 data)
		{
			// clear 4
			m_port[5]->write_rts(BIT(data, 1)); // s2.k5 (rts)
			// resled
			m_port[5]->write_dtr(BIT(data, 3)); // s1.k5 (dtr)
			m_duart[1]->ip4_w(BIT(data, 4)); // rxrdy4
			m_duart[1]->ip5_w(BIT(data, 5)); // rxrdy5
			m_duart[0]->ip4_w(BIT(data, 6)); // txrdy4
			m_duart[0]->ip5_w(BIT(data, 7)); // txrdy5
		}
	);

	m_port[0]->rxd_handler().set(
		[this](int state)
		{
			// TEMP LOG (STRIP): rxd edges from the console pty in the input-test era
			{
				double const t = machine().time().as_double();
				static int n = 0;
				if (t > 11.0 && n++ < 24) logerror("port0 rxd = %d @%.6f\n", state, t);
			}
			m_duart[0]->rx_a_w(state);
		});
	// the console port's modem inputs: the firmware's ring/cell TX pump is gated on ~IP bit0
	// (CTS-A) and the kernel's tty open sleeps until a carrier event (IPCR delta on the DCD
	// input) - both must be wired for the console to open and drain
	m_port[0]->cts_handler().set(m_duart[0], FUNC(scn2681_device::ip0_w));
	m_port[0]->dcd_handler().set(m_duart[0], FUNC(scn2681_device::ip2_w));
	m_port[1]->rxd_handler().set(m_duart[0], FUNC(scn2681_device::rx_b_w));
	m_port[1]->cts_handler().set(m_duart[0], FUNC(scn2681_device::ip1_w));
	m_port[1]->dcd_handler().set(m_duart[0], FUNC(scn2681_device::ip3_w));
	m_port[2]->rxd_handler().set(m_duart[1], FUNC(scn2681_device::rx_a_w));
	m_port[2]->cts_handler().set(m_duart[2], FUNC(scn2681_device::ip0_w));
	m_port[3]->rxd_handler().set(m_duart[1], FUNC(scn2681_device::rx_b_w));
	m_port[4]->rxd_handler().set(m_duart[2], FUNC(scn2681_device::rx_a_w));
	m_port[5]->rxd_handler().set(m_duart[2], FUNC(scn2681_device::rx_b_w));
	m_port[5]->cts_handler().set(m_duart[2], FUNC(scn2681_device::ip1_w));
}

void serad_device::mem_map(address_map &map)
{
	map(0x0000, 0x1fff).rom().region("cpu", 0).mirror(0x2000);
	map(0x4000, 0x4fff).ram().share("mbx").mirror(0x3000);
	map(0x8000, 0x87ff).ram();
}

// Host-side window write into the shared mailbox RAM. While the fw holds the Multibus lock
// (OUT $40), defer the write so the fw's LDA/ORI/STA cell updates are atomic against the host
// (matching the real lock, which holds the host off the bus); apply deferred writes at unlock.
void serad_device::mbx_w(offs_t offset, u16 data, u16 mem_mask)
{
	if (m_buslock)
	{
		if (m_lockpend_n < std::size(m_lockpend))
		{
			m_lockpend[m_lockpend_n][0] = offset;
			m_lockpend[m_lockpend_n][1] = data;
			m_lockpend[m_lockpend_n][2] = mem_mask;
			m_lockpend_n++;
			logerror("BUSLOCK: host write deferred [%03x]=%04x/%04x @%.6f\n", offset * 2, data, mem_mask, machine().time().as_double());
			return;
		}
		logerror("BUSLOCK: pend overflow, writing through @%.6f\n", machine().time().as_double());
	}
	if (ACCESSING_BITS_0_7)
		m_mbx[(offset * 2) & 0xfff] = data & 0xff;
	if (ACCESSING_BITS_8_15)
		m_mbx[(offset * 2 + 1) & 0xfff] = data >> 8;
}

template <bool Lock> void serad_device::buslock_w(u8 data)
{
	m_buslock = Lock;
	if (!Lock)
	{
		for (u8 i = 0; i < m_lockpend_n; i++)
		{
			offs_t const off = m_lockpend[i][0];
			u16 const d = m_lockpend[i][1], m = m_lockpend[i][2];
			if (m & 0x00ff)
				m_mbx[(off * 2) & 0xfff] = d & 0xff;
			if (m & 0xff00)
				m_mbx[(off * 2 + 1) & 0xfff] = d >> 8;
		}
		m_lockpend_n = 0;
	}
}

template void serad_device::buslock_w<true>(u8 data);
template void serad_device::buslock_w<false>(u8 data);

void serad_device::hostint_w(u8 data)
{
	// TEMP LOG (STRIP): every host-int strobe with the event state
	{
		double const t = machine().time().as_double();
		static int n = 0;
		if (n++ < 30)
		{
			auto &sp = m_cpu->space(AS_PROGRAM);
			logerror("HOSTINT mbx[2]=%02x%02x mbx[6]=%02x inh[9d]=%02x @%.5f\n",
				sp.read_byte(0x4003), sp.read_byte(0x4002), sp.read_byte(0x4006), sp.read_byte(0x409d), t);
		}
	}
	// a real strobe: assert now, release after the latch width - a zero-width software pulse
	// never reaches the host ICU. The line is strap-selected (S9-S16).
	u8 const sel = ioport("INT")->read();
	if (!sel)
		return;
	int_w(unsigned(31 - __builtin_clz(u32(sel))), 0);
	m_hostint_held = true;
	// a one-shot pulse: the host ICU is programmed edge-triggered, so the latched pending
	// bit survives until the interrupt is acknowledged regardless of the pulse width - the
	// line just needs to return to idle so the NEXT strobe makes a fresh edge
	m_hostint_rel->adjust(attotime::from_usec(100));
}

TIMER_CALLBACK_MEMBER(serad_device::hostint_release)
{
	hostint_ack();
}

void serad_device::hostint_ack()
{
	if (!m_hostint_held)
		return;
	m_hostint_held = false;
	u8 const sel = ioport("INT")->read();
	if (!sel)
		return;
	unsigned const line = unsigned(31 - __builtin_clz(u32(sel)));
	int_w(line, 1);
}

void serad_device::pio_map(address_map &map)
{
	map(0x00, 0x0f).rw(m_duart[0], FUNC(scn2681_device::read), FUNC(scn2681_device::write)); // duart 0
	map(0x10, 0x1f).rw(m_duart[1], FUNC(scn2681_device::read), FUNC(scn2681_device::write)); // duart 2
	map(0x20, 0x2f).rw(m_duart[2], FUNC(scn2681_device::read), FUNC(scn2681_device::write)); // duart 4
	// the host-side Multibus interrupt strobe: the fw pulses this after posting an event
	// (config acks, line events, rx data) - the kernel's serad driver runs on vector 4 ipl 5
	// (its own probe banner); Multibus INT lines are active low
	map(0x30, 0x3f).w(FUNC(serad_device::hostint_w));
	map(0x40, 0x4f).w(FUNC(serad_device::buslock_w<true>));  // multibus lock
	map(0x50, 0x5f).w(FUNC(serad_device::buslock_w<false>)); // multibus unlock
	map(0x60, 0x6f).w(FUNC(serad_device::rst55_w<0>));
	//map(0x70, 0x7f); // TODO: power
}
