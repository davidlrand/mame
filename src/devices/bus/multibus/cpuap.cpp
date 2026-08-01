// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
 * Siemens S26361-D333 CPUAP processor board.
 *
 * This Multibus card was in Siemens PC-MX2 systems. It has a 10MHz NS32016
 * CPU with the complete set of NS32000 support chips and 1MiB of on-board RAM.
 * The board connects to optional memory expansion boards through a dedicated
 * connector (not the Multibus). It has no on-board UART, so depends on a SERAD
 * or DUEAI board installed in the system to communicate with a console.
 *
 * Sources:
 *  - https://oldcomputers-ddns.org/public/pub/rechner/siemens/mx-rm/pc-mx2/manuals/pc-mx2_pc2000_9780_logik.pdf
 *  - https://mx300i.narten.de/view_board.cfm?5EF287A1ABC3F4DCAFEA2BC2FAB8C4000E50392BD499
 *
 * TODO:
 *  - led output
 *  - nmi generation
 *  - figure out how to pass-through multibus addresses
 */

#include "emu.h"
#include "cpuap.h"

#define VERBOSE (0)
#include "logmacro.h"

// Diagnostic taps from the paged-VM / "init died" / boot-image investigations.  All their
// questions are ANSWERED; they are retained because each cost real effort to aim, but they must
// not run by default - unguarded they fire every boot and perturb the timing the storager work
// depends on.  Flip to true to re-arm the whole set.  STRIP before any upstream submission.
static constexpr bool TRACE_BOOT = false;

DEFINE_DEVICE_TYPE(CPUAP, cpuap_device, "cpuap", "Siemens S26361-D333 CPUAP")

cpuap_device::cpuap_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, CPUAP, tag, owner, clock)
	, device_multibus_interface(mconfig, *this)
	, m_cpu(*this, "cpu")
	, m_fpu(*this, "fpu")
	, m_mmu(*this, "mmu")
	, m_icu(*this, "icu")
	, m_rtc(*this, "rtc")
	, m_csuart(*this, "csuart")
	, m_diag(*this, "diag")
	, m_s7(*this, "S7")
	, m_s8(*this, "S8")
	, m_boot(*this, "boot")
	, m_ram(*this, "ram", 0x100000, ENDIANNESS_LITTLE)
	, m_installed(false)
{
}

// processor diagnostic register
enum prdia_mask : u8
{
	LED1     = 0x01,
	LED2     = 0x02,
	LED3     = 0x04,
	LED4     = 0x08,
	LED5     = 0x10,
	LED6     = 0x20,
	ERRORLED = 0x40,
	ENNMI    = 0x80,
};

enum nmi_mask : u8
{
	PAR2    = 0x01,
	DEBUGI  = 0x02,
	ACF     = 0x04,
	PERLO   = 0x08, // parity error lo
	PERHI   = 0x10, // parity error hi
	BTIMOUT = 0x20, // bus timeout
	INT7    = 0x40,
};

ROM_START(cpuap)
	ROM_REGION16_LE(0x10000, "eprom", 0)

	// Declared explicitly rather than relying on the implicit "first ROM_SYSTEM_BIOS wins" rule.
	// rev9 is the 32K monitor the PC-MX2 ships with and the one the disassembly targets; rev3 is a
	// 16K predecessor whose EXT() link-table indices differ, so annotations do not transfer.
	ROM_DEFAULT_BIOS("rev9")

	ROM_SYSTEM_BIOS(0, "rev9", "D333 Monitor Rev 9.0 16.06.1988")
	ROMX_LOAD("361d0333d053__e01735_ine.d53", 0x0000, 0x4000, CRC(b5eefb64) SHA1(a71a7daf9a8f0481d564bfc4d7ed5eb955f8665f), ROM_SKIP(1) | ROM_BIOS(0))
	ROMX_LOAD("361d0333d054__e01725_ine.d54", 0x0001, 0x4000, CRC(3a3c6b6e) SHA1(5302fd79c89e0b4d164c639e2d73f4b9a279ddcb), ROM_SKIP(1) | ROM_BIOS(0))
	ROMX_FILL(0x8000, 0x8000, 0xff, ROM_BIOS(0))

	ROM_SYSTEM_BIOS(1, "rev3", "D333 Monitor Rev 3 09.12.1985")
	ROMX_LOAD("d333__d56_g53__lb.d56", 0x0000, 0x2000, CRC(0892ff90) SHA1(e84ceb8eb3c13de3692297c46632dbfafaad675f), ROM_SKIP(1) | ROM_BIOS(1))
	ROMX_LOAD("d333__d55_g53__hb.d55", 0x0001, 0x2000, CRC(821e1e41) SHA1(0800249eab8db490c1fb6fea6d65bc7e874c9a0c), ROM_SKIP(1) | ROM_BIOS(1))
	ROMX_FILL(0x4000, 0xc000, 0xff, ROM_BIOS(1))
ROM_END

static INPUT_PORTS_START(cpuap)
	PORT_START("S7")

	// Offen: Ausgabe des Urladers über Diagnose-Stecker, keine SERAD/G Baugruppe gesteckt
	// Open: boot loader output via diagnostic plug, no SERAD/G module plugged in
	// On: service/bring-up console on the on-board CSUART "diagnostic plug" (no SERAD fitted).
	// Off (default, the production configuration): SERAD port 0 is the console (38400 7O1);
	// the manuals distinguish the operator console (SERAD/SERAG) from the CPUAP diagnostic
	// plug - a delivered MX2 boots and installs through the SERAD.
	PORT_DIPNAME(0x80, 0x00, "Diagnostic") PORT_DIPLOCATION("S7:8")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x80, DEF_STR(On))

	// Offen: Monitor-Programm nach Testende
	// Open: enter monitor after test
	PORT_DIPNAME(0x40, 0x00, "Boot Option") PORT_DIPLOCATION("S7:7")
	PORT_DIPSETTING(0x00, "Disk")
	PORT_DIPSETTING(0x40, "Monitor")

	PORT_DIPNAME(0x20, 0x00, "S7:5") PORT_DIPLOCATION("S7:6")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x20, DEF_STR(On))
	PORT_DIPNAME(0x10, 0x00, "S7:4") PORT_DIPLOCATION("S7:5")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x10, DEF_STR(On))
	PORT_DIPNAME(0x08, 0x00, "S7:3") PORT_DIPLOCATION("S7:4")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x08, DEF_STR(On))
	PORT_DIPNAME(0x04, 0x00, "S7:2") PORT_DIPLOCATION("S7:3")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x04, DEF_STR(On))
	PORT_DIPNAME(0x02, 0x00, "S7:1") PORT_DIPLOCATION("S7:2")
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x02, DEF_STR(On))

	// Offen: kein Reboot nach Systemabsturz
	// Open: no reboot after system crash
	PORT_DIPNAME(0x01, 0x00, "S7:0") PORT_DIPLOCATION("S7:1") // S7:0 not allowed, causes crash in the dips menu
	PORT_DIPSETTING(0x00, DEF_STR(Off))
	PORT_DIPSETTING(0x01, DEF_STR(On))

	PORT_START("S8")
	PORT_DIPNAME(0x01, 0x00, "IRQ 7")
	PORT_DIPSETTING(0x00, "ICU IR11")
	PORT_DIPSETTING(0x01, "NMI")
INPUT_PORTS_END

static DEVICE_INPUT_DEFAULTS_START(diag_defaults)
	DEVICE_INPUT_DEFAULTS("RS232_TXBAUD", 0xff, RS232_BAUD_38400)
	DEVICE_INPUT_DEFAULTS("RS232_RXBAUD", 0xff, RS232_BAUD_38400)
	DEVICE_INPUT_DEFAULTS("RS232_DATABITS", 0xff, RS232_DATABITS_7)
	DEVICE_INPUT_DEFAULTS("RS232_PARITY", 0xff, RS232_PARITY_ODD)
	DEVICE_INPUT_DEFAULTS("RS232_STOPBITS", 0xff, RS232_STOPBITS_1)
DEVICE_INPUT_DEFAULTS_END

const tiny_rom_entry *cpuap_device::device_rom_region() const
{
	return ROM_NAME(cpuap);
}

ioport_constructor cpuap_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(cpuap);
}

void cpuap_device::device_start()
{
	// MEMAD D303 3MB memory expansion (CPUAP-local mem bus, into the NS32016 space at 0x100000)
	m_ramext = std::make_unique<u16[]>(0x300000 / 2);
	save_pointer(NAME(m_ramext), 0x300000 / 2);

	// TEMP (STRIP): who builds the storager dev-blk (the count field writers) - the sdisk driver's
	// command builder, to be disassembled for the definitive chained-write contract
	if (TRACE_BOOT)
		m_cpu->space(AS_PROGRAM).install_write_tap(0x249a0, 0x249a7, "devblkwatch",
		[this](offs_t offset, u16 &data, u16 mem_mask)
		{
			static int n = 0;
			double const t = machine().time().as_double();
			if (t > 27.5 && n++ < 60 && offset == 0x249a0)
			{
				// dev-block first 8 bytes (op, unit|pos-hi, pos, count) + the buffer (req) fields
				auto &sp = m_cpu->space(AS_PROGRAM);
				u32 blk = 0; for (int i = 0; i < 8; i++) blk = (blk << 8) | sp.read_byte(0x249a0 + i);
				logerror("DEVBLKWR pc=%06x blk=%016llx @%.5f\n",
						m_cpu->pc(), (unsigned long long)blk, t);
			}
		});
	// TEMP (STRIP): who rewrites the super-buffer page's PTE2 @0x57360 (CPU side)
	if (TRACE_BOOT)
		m_cpu->space(AS_PROGRAM).install_write_tap(0x57360, 0x57363, "ptewatch_cpu",
		[this](offs_t offset, u16 &data, u16 mem_mask)
		{
			static int n = 0;
			if (n++ < 30) logerror("PTEWR-CPU @%06x = %04x/%04x pc=%06x @%.4f\n", offset, data, mem_mask, m_cpu->pc(), machine().time().as_double());
		});
	// TEMP (STRIP): who clobbers the root super's magic word @0x3B15C (CPU side)
	if (TRACE_BOOT)
		m_cpu->space(AS_PROGRAM).install_write_tap(0x3b158, 0x3b163, "magicwatch_cpu",
		[this](offs_t offset, u16 &data, u16 mem_mask)
		{
			static int n = 0;
			if (n++ < 20) logerror("MAGICWR-CPU @%06x = %04x/%04x pc=%06x @%.4f\n", offset, data, mem_mask, m_cpu->pc(), machine().time().as_double());
		});
}

void cpuap_device::device_reset()
{
	if (!m_installed)
	{
		// Expose the 1 MiB on-board RAM as a Multibus slave so bus masters (e.g. the Storager disk
		// controller) can DMA the IOPB and transfer buffers that live in NS32016 memory. The monitor
		// hands the controller raw 20-bit addresses (its data structures sit in low RAM), so the RAM
		// answers on the bus at 0x000000-0x0FFFFF -- clear of the SERAD mailbox window at 0xEF7000.
		m_bus->space(AS_PROGRAM).install_ram(0x000000, 0x0fffff, m_ram.target());

		// MEMAD D303 memory expansion: 3 MB at 0x100000-0x3FFFFF -> 4 MB total (this install image
		// reports System 734k / User 3362k = 4 MB). Not a normal Multibus memory - it maps directly
		// into the CPUAP's NS32016 space.
		m_cpu->space(AS_PROGRAM).install_ram(0x100000, 0x3fffff, m_ramext.get());
		// The RAM-sizing probe at fe02b6 deliberately writes/reads ABOVE installed memory (0x4FFFFC
		// etc.) to find the top of RAM - that is its job.  MAME logs every one of those as an
		// unmapped access, which floods error.log (396k lines out of every 400k) and throttles a
		// -oslog run to a few seconds of emulated time.  Logging only; no behavioural change.
		// Suppress on EVERY space the CPU has, not just AS_PROGRAM: the NS32000 also drives an `iam`
		// space, and the sizing probe floods that one too (196k of every 200k log lines).
		for (int sp = 0; sp < 8; sp++)
			if (m_cpu->has_space(sp))
				m_cpu->space(sp).set_log_unmap(false);
		// ...and, like the on-board RAM, the expansion must be visible in EVERY NS32k access
		// status, not only status 0.  The NS32016 configures a separate space for status 4, and
		// the NS32082 MMU's page-table walk / user (AS1) cycles use it.  Kernel page tables and
		// modules can live in the expansion (the known-good image's target module is at 0x320392),
		// so without this the status-4 walk finds nothing and translation of those pages fails.
		m_cpu->space(4).install_ram(0x100000, 0x3fffff, m_ramext.get());
		m_bus->space(AS_PROGRAM).install_ram(0x100000, 0x3fffff, m_ramext.get());

		// TEMP (STRIP): who writes module-descriptor 0x01D0 (prog field @0x01D8 = the bad 0x7b08)?
		// A PHYSICAL/bus tap catches DMA + user-translated writes the CPU-logical tap misses.
		if (TRACE_BOOT)
			m_bus->space(AS_PROGRAM).install_write_tap(0x0001d0, 0x0001df, "moddescwatch",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ static int n = 0; if (n++ < 30) logerror("MODDESC-BUS @%06x=%04x mask=%04x (%s)\n", offset, data, mem_mask, machine().describe_context()); });

		// TEMP (STRIP): who DMAs kernel-entry RAM 0x8000-0x80ff (boot image vs /sinix)?  bus-side
		// tap catches the storager DMA the CPU-space tap misses.  Log first bytes + writer context.
		if (TRACE_BOOT)
			m_bus->space(AS_PROGRAM).install_write_tap(0x008000, 0x0080ff, "k8000watch",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{ static int n = 0; if (offset < 0x8010 && n++ < 40) logerror("K8-BUS @%06x=%04x (%s)\n", offset, data, machine().describe_context()); });

		// TEMP (STRIP): who clobbers the root super's magic word - the BUS-side tap must be
		// installed AFTER this install_ram (an earlier tap on the range is replaced by it)
		if (TRACE_BOOT)
			m_bus->space(AS_PROGRAM).install_write_tap(0x3b140, 0x3b17f, "magicwatch_bus2",
			[this](offs_t offset, u16 &data, u16 mem_mask)
			{
				static int n = 0;
				if (n++ < 4)
				{
					u32 const fp = m_cpu->state_int(2), r6 = m_cpu->state_int(15);
					u32 const ra = m_cpu->space(AS_PROGRAM).read_dword(fp + 4);
					u32 const a8 = m_cpu->space(AS_PROGRAM).read_dword(fp + 8);
					u32 const ac = m_cpu->space(AS_PROGRAM).read_dword(fp + 12);
					logerror("MAGICWR-BUS2 @%06x = %04x/%04x pc=%06x R6=%08x FP=%08x ret=%08x args=%08x %08x @%.4f\n",
							offset, data, mem_mask, m_cpu->pc(), r6, fp, ra, a8, ac, machine().time().as_double());
				}
			});

		// TEMP (STRIP): 64MB-corruption watch. The pre/post-64M-crossing diff (vs a <64M control)
		// flagged these kernel/expansion locations as changing only when the format crosses 64MB.
		// Catch WHO writes them (a bus-master DMA shows describe_context = the storager; a CPU write
		// shows the kernel PC) during the >64M spare phase.
		for (u32 const wa : { u32(0x01ec9c), u32(0x3e2cf8), u32(0x3e8a24) })
			if (TRACE_BOOT)
				m_bus->space(AS_PROGRAM).install_write_tap(wa, wa + 3, "corrupt64",
				[this](offs_t offset, u16 &data, u16 mem_mask)
				{
					double const t = machine().time().as_double();
					static int n = 0;
					if (t > 28.0 && n++ < 80)
						logerror("CORRUPT64 @%06x = %04x/%04x cpupc=%06x [%s] @%.4f\n",
								offset, data, mem_mask, m_cpu->pc(), machine().describe_context().c_str(), t);
				});

		// XACK timeout -> NMI on absent Multibus addresses. The kernel probes optional hardware with
		// a nofault protocol (flag + probed-address cell; its NMI handler rewrites the cell and
		// resumes), so absent-device windows must FAULT, not float. Installed on the windows the
		// SINIX kernel probes and no modeled card claims: I/O 0x1400-0x1FFF (extra serial boards at
		// 0x1800/0x1900) and memory 0xED0000-0xEDFFFF (the "sl" board).
		m_bus->space(AS_IO).install_readwrite_handler(0x1400, 0x1fff,
				read16smo_delegate(*this, NAME([this]() -> u16 { if (!machine().side_effects_disabled()) bus_timeout(0); return 0xffff; })),
				write16smo_delegate(*this, NAME([this](u16 data) { bus_timeout(0); })));
		m_bus->space(AS_PROGRAM).install_readwrite_handler(0xed0000, 0xedffff,
				read16smo_delegate(*this, NAME([this]() -> u16 { if (!machine().side_effects_disabled()) bus_timeout(0); return 0xffff; })),
				write16smo_delegate(*this, NAME([this](u16 data) { bus_timeout(0); })));
		// the unpopulated SERAD slots (EF0000-EF6FFF; EF7000 = the fitted board's window): a real
		// Multibus access to an empty slot gets no XACK - the autoconf probe relies on the timeout
		// NMI to mark the slot absent, else open-bus reads make ghost boards appear (and srinit
		// then reports fatal errors probing them)
		m_bus->space(AS_PROGRAM).install_readwrite_handler(0xef0000, 0xef6fff,
				read16smo_delegate(*this, NAME([this]() -> u16 { if (!machine().side_effects_disabled()) bus_timeout(0); return 0xffff; })),
				write16smo_delegate(*this, NAME([this](u16 data) { bus_timeout(0); })));

		m_installed = true;
	}

	m_boot.select(0);
	m_nmi = 0xff;
}

void cpuap_device::bus_timeout(u8 data)
{
	m_nmi &= ~BTIMOUT;

	if (m_prdia & ENNMI)
	{
		// pulse: the core latches the edge; a held line would block the next timeout's edge
		m_cpu->set_input_line(INPUT_LINE_NMI, 1);
		m_cpu->set_input_line(INPUT_LINE_NMI, 0);
	}
}

void cpuap_device::device_add_mconfig(machine_config &config)
{
	// 20.000 MHz master oscillator (VALVO X05850 can module at the upper-left of the board, stamped
	// "20.000MHz"; see siemens/cpuap_board.webp), divided by 2 to clock the NS32016 and its complete
	// -10-grade NS32000 support set at 10 MHz.
	NS32016(config, m_cpu, 20_MHz_XTAL / 2);
	// DRAM wait states per NS32016 bus cycle. The board's TMS4256-12 (120ns) DRAM sits behind a
	// BFCLK-clocked RAS/CAS controller (sheet D333-X-*-11); 2 is calibrated so the SINIX srinit counted
	// delay - a loop decrementing an in-memory counter, so its cost is charged in ns32000 top() - spans the
	// SERAD's ~0.35s firmware init. It also absorbs the core's unmodelled instruction-fetch cycles, so it is
	// a behavioural value rather than the literal hardware wait-state count (which the fast DRAM puts at ~0-1).
	m_cpu->set_wait_states(2);
	m_cpu->set_addrmap(0, &cpuap_device::cpu_map<0>);
	m_cpu->set_addrmap(4, &cpuap_device::cpu_map<4>);

	NS32081(config, m_fpu, 20_MHz_XTAL / 2);
	m_cpu->set_fpu(m_fpu);

	NS32082(config, m_mmu, 20_MHz_XTAL / 2);
	m_cpu->set_mmu(m_mmu);

	NS32202(config, m_icu, 20_MHz_XTAL / 2);
	m_icu->out_int().set([this](int state)
	{
		// TEMP LOG (STRIP): interrupt-delivery rate probe (the post-autoconf stall diagnosis)
		{
			static int edges = 0;
			static int last_sec = -1;
			double const t = machine().time().as_double();
			if (!state) edges++;   // active-low assert
			int const sec = int(t);
			if (sec != last_sec && sec >= 1 && sec <= 20)
			{
				logerror("ICUINT second %d: %d asserts\n", last_sec, edges);
				edges = 0;
				last_sec = sec;
			}
			else if (last_sec == -1)
				last_sec = sec;
		}
		m_cpu->set_input_line(INPUT_LINE_IRQ0, !state);
	});

	int_callback<0>().set(m_icu, FUNC(ns32202_device::ir_w<0>));
	int_callback<1>().set(m_icu, FUNC(ns32202_device::ir_w<1>));
	int_callback<2>().set(m_icu, FUNC(ns32202_device::ir_w<3>));
	int_callback<3>().set(m_icu, FUNC(ns32202_device::ir_w<4>));
	int_callback<4>().set(m_icu, FUNC(ns32202_device::ir_w<6>));
	int_callback<5>().set(m_icu, FUNC(ns32202_device::ir_w<7>));
	int_callback<6>().set(m_icu, FUNC(ns32202_device::ir_w<8>));
	int_callback<7>().set([this](int state) { m_s8->read() ? m_cpu->set_input_line(INPUT_LINE_NMI, !state) : m_icu->ir_w<11>(state); });

	MC146818(config, m_rtc, 32.768_kHz_XTAL);
	// the periodic 50 Hz system tick -> ICU IR2. The SINIX kernel's IR2 handler is hardclock()
	// (callout-list decrement, ipl 6, HZ=50 per the DIVD #50 in its time conversion) and the kernel
	// programs no timer hardware itself, so the board supplies the tick from the MC146818's IRQ
	// output, driven as a level - the clock ISR acknowledges by reading RTC register C (which drops
	// the IRQ). A synthetic free-toggling tick line can never be acknowledged and storms the ICU
	// (~3000 ints/s), which is why this comes from the RTC rather than a free-running timer.
	m_rtc->irq().set([this](int state)
	{
		m_icu->ir_w<2>(!state);
	});

	// on-board "diagnostic plug" console UART, selected by the monitor when S7:8 Diagnostic is set
	// (i.e. no SERAD board). Channel A is the console; the monitor runs it at 38400 (CSR 0xC).
	SCN2681(config, m_csuart, 7.3728_MHz_XTAL / 2);
	m_csuart->a_tx_cb().set(m_diag, FUNC(rs232_port_device::write_txd));
	// CSUART irq -> ICU IR12 (kernel: "sc 0 ... vector 12 ipl 5"), INVERTED: the monitor programs
	// the ICU all level-triggered active-LOW (TPL=0), and the 2681 asserts its IRQ output high.
	// (The earlier "TxRDY storm" was this inversion - the ICU saw the IDLE level as pending.)
	m_csuart->irq_cb().set([this](int state) { m_icu->ir_w<12>(!state); });

	RS232_PORT(config, m_diag, default_rs232_devices, nullptr);
	m_diag->rxd_handler().set(m_csuart, FUNC(scn2681_device::rx_a_w));
	m_diag->set_option_device_input_defaults("null_modem", DEVICE_INPUT_DEFAULTS_NAME(diag_defaults));
	m_diag->set_option_device_input_defaults("terminal", DEVICE_INPUT_DEFAULTS_NAME(diag_defaults));
}

template <unsigned ST> void cpuap_device::cpu_map(address_map &map)
{
	// The on-board 1 MiB RAM must be visible in EVERY access status, not only ST=0.
	// The NS32016 configures separate spaces for status 0 and 4; the NS32082 MMU's
	// page-table walk (and user/AS1 cycles) use the status-4 space.  The kernel's page
	// tables live in this on-board RAM, so if it is only mapped under ST==0 the walk on
	// status 4 finds nothing and translation of remapped pages fails (CXP into garbage).
	// ICM3216 (same NS32016+NS32082) maps its RAM outside the ST==0 gate for exactly this
	// reason.  ST=0 overrides the low range below with the boot ROM/RAM view.
	map(0x000000, 0x0fffff).ram().share("ram");

	if (ST == 0)
	{
		map(0x000000, 0x0fffff).view(m_boot);

		m_boot[0](0x000000, 0x00ffff).rom().region("eprom", 0);
		m_boot[1](0x000000, 0x0fffff).ram().share("ram");

		//map(0x100000, 0x3fffff); // first memory expansion
		//map(0x400000, 0x6fffff); // second memory expansion

		map(0x700000, 0xdfffff).w(FUNC(cpuap_device::bus_timeout));

		map(0xe00000, 0xefffff).rw(FUNC(cpuap_device::bus_mem_r), FUNC(cpuap_device::bus_mem_w)); // multibus mem
		map(0xf00000, 0xf0ffff).rw(FUNC(cpuap_device::bus_pio_r), FUNC(cpuap_device::bus_pio_w)); // multibus i/o

		//map(0xf10000, 0xfdffff); // unused
		map(0xfe0000, 0xfeffff).rom().region("eprom", 0);

		//map(0xff0000, 0xf7ffff); // unused
		map(0xff8000, 0xff803f).rw(m_rtc, FUNC(mc146818_device::read_direct), FUNC(mc146818_device::write_direct));
		map(0xff8100, 0xff8100).lr8([this]() { return m_s7->read(); }, "s7_r");
		map(0xff8200, 0xff8200).lw8([this](u8 data) { LOG("prdia_w 0x%02x led 0x%x (%s)\n", data, ~data & 0x3f, machine().describe_context()); m_prdia = data; }, "prdia_w");
		map(0xff8300, 0xff8300).lrw8([this]() { return m_poff; }, "poff_r", [this](u8 data) { m_poff = data & 3; }, "poff_w"); // 3 pohopofi - power off interrupt?
		map(0xff8400, 0xff841f).rw(m_csuart, FUNC(scn2681_device::read), FUNC(scn2681_device::write)).umask16(0x00ff); // csuart (diagnostic console)
		map(0xff8500, 0xff8500).lw8([this](u8 data) { m_boot.select(BIT(data, 0)); }, "mapprom_w");
		map(0xff8600, 0xff863f).m(m_icu, FUNC(ns32202_device::map<0>)).umask16(0x00ff);
		map(0xff8700, 0xff8700).lrw8(
			[this]() { return m_nmi; }, "nmi_r",
			[this](u8 data) { LOG("nmi_w 0x%02x (%s)\n", data, machine().describe_context()); m_nmi = 0xff; }, "nmi_w");
	}

	map(0xfffe00, 0xfffeff).m(m_icu, FUNC(ns32202_device::map<BIT(ST, 1)>)).umask16(0x00ff);
}
