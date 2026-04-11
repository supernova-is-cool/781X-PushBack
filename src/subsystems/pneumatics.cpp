#include "subsystems/pneumatics.h"

#include <cstdint>
#include <mutex>
#include <utility>

PneumaticGroup::PneumaticGroup(std::vector<pros::adi::Port> ports,
															 bool start_extended, bool extended_is_low)
		: m_ports(std::move(ports)), m_is_extended(start_extended),
			m_extended_is_low(extended_is_low) {
	for (auto &port : m_ports) {
		port.set_config(pros::E_ADI_DIGITAL_OUT);
	}
	set(start_extended);
}

PneumaticGroup::PneumaticGroup(std::vector<int> ports, bool start_extended,
															 bool extended_is_low)
		: m_is_extended(start_extended), m_extended_is_low(extended_is_low) {
	m_ports.reserve(ports.size());

	for (const int port : ports) {
		m_ports.emplace_back(static_cast<std::uint8_t>(port),
												 pros::E_ADI_DIGITAL_OUT);
	}

	set(start_extended);
}

void PneumaticGroup::extend() { set(true); }

void PneumaticGroup::retract() { set(false); }

void PneumaticGroup::toggle() {
	std::lock_guard<pros::Mutex> lock(m_mutex);
	setLocked(!m_is_extended);
}

void PneumaticGroup::set(bool is_extended) {
	std::lock_guard<pros::Mutex> lock(m_mutex);
	setLocked(is_extended);
}

void PneumaticGroup::setLocked(bool is_extended) {
	m_is_extended = is_extended;
	const bool output_high = m_extended_is_low ? !is_extended : is_extended;

	for (auto &port : m_ports) {
		port.set_value(output_high ? 1 : 0);
	}
}
