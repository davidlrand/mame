// license:BSD-3-Clause
// copyright-holders:Dave Rand

#include "emu.h"
#include "s97801.h"

DEFINE_DEVICE_TYPE(SERIAL_TERMINAL_S97801, s97801_terminal_device, "s97801_terminal", "Siemens 97801 Terminal")

s97801_terminal_device::s97801_terminal_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, SERIAL_TERMINAL_S97801, tag, owner, clock)
	, device_rs232_port_interface(mconfig, *this)
	, m_term(*this, "term")
{
}

void s97801_terminal_device::device_add_mconfig(machine_config &config)
{
	SIEMENS_97801(config, m_term, 0);
	m_term->txd_handler().set(FUNC(s97801_terminal_device::output_rxd)); // terminal -> host
}

void s97801_terminal_device::device_start()
{
}

void s97801_terminal_device::input_txd(int state) // host -> terminal
{
	m_term->rxd_w(state);
}
