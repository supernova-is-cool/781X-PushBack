#include "subsystems/intake.h"
#include "color.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include "robot.h"
#include <cmath>
#include <cstdio>
#include <optional>

#define POWER 127
#define SLOW_POWER 127 * .8

Intake::Intake(pros::MotorGroup &motors, pros::adi::Pneumatics &bottom_gate,
               pros::adi::Pneumatics &top_gate)
    : m_motors(motors), m_spinState(SpinState::IDLE),
      m_gateState(GateState::STORING), m_bottom_gate(bottom_gate),
      m_top_gate(top_gate) {}

void Intake::runTask() {
  if (m_spinState == SpinState::EMERGENCY_STOP)
    m_motors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  else
    m_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  switch (m_spinState) {
  case SpinState::EMERGENCY_STOP:
    m_motors.brake();
    break;
  case SpinState::IDLE:
    m_motors.brake();
    break;
  case SpinState::OUTTAKING:
    m_motors.move(-POWER);
    break;
  case SpinState::INTAKING:
    m_motors.move(POWER);
    break;
  case SpinState::SLOW_INTAKING:
    m_motors.move(SLOW_POWER);
    break;
  case SpinState::SLOW_OUTTAKING:
    m_motors.move(-SLOW_POWER);
    break;
  }
  switch (m_gateState) {
  case GateState::STORING:
    m_bottom_gate.extend();
    m_top_gate.retract();
    break;
  case GateState::MIDDLE_GOAL:
    m_bottom_gate.extend();
    m_top_gate.extend();
    break;
  case GateState::LONG_GOAL:
    m_bottom_gate.retract();
    m_top_gate.retract();
    break;
  }
}

const Intake::SpinState &Intake::getSpinState() const { return m_spinState; }
const Intake::GateState &Intake::getGateState() const { return m_gateState; }
void Intake::setSpinState(SpinState state) { m_spinState = state; }
void Intake::setGateState(GateState state) { m_gateState = state; }

void Intake::emergencyStop() { setSpinState(SpinState::EMERGENCY_STOP); }
void Intake::goToIdle() { setSpinState(SpinState::IDLE); }
void Intake::goToOuttaking() { setSpinState(SpinState::OUTTAKING); }
void Intake::goToTOP() {
  setSpinState(SpinState::INTAKING);
  setGateState(GateState::LONG_GOAL);
}
void Intake::goToMIDDLE() {
  setSpinState(SpinState::SLOW_INTAKING);
  setGateState(GateState::MIDDLE_GOAL);
}
void Intake::goToStoring() {
  setSpinState(SpinState::INTAKING);
  setGateState(GateState::STORING);
}

void Intake::enableFiltering() { enableFilter = true; }
void Intake::disableFiltering() { enableFilter = false; }
