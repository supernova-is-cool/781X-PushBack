#pragma once

#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include <vector>

/** Controls the pneumatics as a single unit, preventing the from being in
 * conflicting states. */
class PneumaticGroup {
private:
  std::vector<pros::adi::Port> m_ports;
  pros::Mutex m_mutex;
  bool m_is_extended;
  const bool m_extended_is_low;
  /**
   * INTERNAL USE ONLY!!
   *
   * Applies requested extension state to internal state and all output ports.
   * Precondition: m_mutex is already held by the caller.
   */
  void setLocked(bool is_extended);

public:
  PneumaticGroup(std::vector<pros::adi::Port> ports, bool start_extended,
                 bool extended_is_low = false);
  PneumaticGroup(std::vector<int> ports, bool start_extended,
                 bool extended_is_low = false);
  void extend();
  void retract();
  void toggle();
  void set(bool is_extended);
};