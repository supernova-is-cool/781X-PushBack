#include "subsystems/intake.h"

#define POWER 127
#define SLOW_POWER 127 * .85

Intake::Intake(pros::MotorGroup &motors)
    : m_motors(motors), m_spinState(SpinState::IDLE) {}

void Intake::runTask() {
  switch (m_spinState) {
  case SpinState::IDLE:
    m_motors.brake();
    break;
  case SpinState::OUTTAKING:
    m_motors.move(-POWER);
    break;
  case SpinState::INTAKING:
    m_motors.move(POWER);
    break;
  case SpinState::SLOW_OUTTAKING:
    m_motors.move(-SLOW_POWER);
    break;
  case SpinState::EVEN_SLOWER:
    m_motors.move(POWER * 0.7);
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

void Intake::intake() { m_spinState = SpinState::INTAKING; }
void Intake::outtake() { m_spinState = SpinState::OUTTAKING; }
void Intake::slowOuttake() { m_spinState = SpinState::SLOW_OUTTAKING; }
void Intake::stop() { m_spinState = SpinState::IDLE; }
