// license:BSD-3-Clause
// copyright-holders:Dave Rand
#ifndef MAME_MACHINE_S97801_H
#define MAME_MACHINE_S97801_H

#pragma once

#include "cpu/mcs51/i8051.h"
#include "machine/scn_pci.h"
#include "sound/beep.h"
#include "video/scn2674.h"

// Siemens 97801 terminal (LLE) -- core device.  The serial console for SINIX on the PC-MX2.
// Full detail + RE log in 97801/RESEARCH-LOG.md.  Exposed as a serial peripheral: rxd_w() is the
// host->terminal line, txd_handler() the terminal->host line (the firmware runs the SCN2661 EPCI
// at 38400 7O1 XON/XOFF).
class s97801_device : public device_t
{
public:
	s97801_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock = 0);

	auto txd_handler() { return m_txd_cb.bind(); } // terminal -> host
	void rxd_w(int state);                          // host -> terminal

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;

private:
	void prg_map(address_map &map) ATTR_COLD;
	void data_map(address_map &map) ATTR_COLD;
	void char_map(address_map &map) ATTR_COLD;
	void attr_map(address_map &map) ATTR_COLD;

	SCN2672_DRAW_CHARACTER_MEMBER(draw_character);

	void epci_txd_w(int state) { m_txd_cb(state); }
	u8 cpu_p3_r();
	void cpu_p3_w(u8 data);
	u8 e00x_status_r(offs_t offset);
	void e00x_status_w(offs_t offset, u8 data);
	TIMER_CALLBACK_MEMBER(kbd_clock);
	TIMER_CALLBACK_MEMBER(kbd_respond);
	TIMER_CALLBACK_MEMBER(kbd_scan);
	TIMER_CALLBACK_MEMBER(cmd_sample);
	TIMER_CALLBACK_MEMBER(bell_off);
	void kbd_enqueue(u8 b);

	required_device<i8031_device> m_cpu;
	required_device<scn2672_device> m_avdc;
	required_device<scn2661b_device> m_epci;
	required_device<beep_device> m_beep;
	required_region_ptr<u8> m_chargen;
	required_ioport_array<6> m_keys;
	devcb_write_line m_txd_cb;

	// terminal->keyboard command decoder (catches BEL = 0x24 to ring the keyboard beeper)
	emu_timer *m_cmd_timer = nullptr;
	emu_timer *m_bell_timer = nullptr;
	u8  m_cmd_shift = 0;
	u8  m_cmd_count = 0;
	bool m_cmd_active = false;

	// detached serial keyboard on the 8031 on-chip UART (~651 baud); make/break Platzcodes +
	// the power-on 0xAA handshake, fed through a TX FIFO to the P3.0 bit-banger.
	emu_timer *m_kbd_timer = nullptr;
	emu_timer *m_kbd_resp_timer = nullptr;
	emu_timer *m_kbd_scan_timer = nullptr;
	u16 m_kbd_frame = 0;
	u8  m_kbd_bits = 0;
	int m_kbd_line = 1;
	int m_kbd_txd_last = 1;
	bool m_kbd_busy = false;
	bool m_kbd_resp_pending = false;
	u8  m_kbd_fifo[32] = {0};
	u8  m_kbd_fhead = 0, m_kbd_ftail = 0;
	u8  m_kbd_prev[6] = {0};
};

DECLARE_DEVICE_TYPE(SIEMENS_97801, s97801_device)

#endif // MAME_MACHINE_S97801_H
