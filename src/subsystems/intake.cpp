#include "subsystems/intake.h"

#define POWER 127
#define SLOW_POWER 127 * .7

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
  }
}

void Intake::intake() { m_spinState = SpinState::INTAKING; }
void Intake::outtake() { m_spinState = SpinState::OUTTAKING; }
void Intake::slowOuttake() { m_spinState = SpinState::SLOW_OUTTAKING; }
void Intake::stop() { m_spinState = SpinState::IDLE; }
