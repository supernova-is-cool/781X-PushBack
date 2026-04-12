#include "subsystems/lever.h"

Lever::Lever(pros::MotorGroup &motors, pros::adi::Pneumatics gate)
    : m_state(State::INTAKE_READY), m_motors(motors), m_gate(gate) {
  m_motors.tare_position_all();
}

void Lever::runTask() {
  switch (m_state) {
  case State::COMPRESS:
  case State::INTAKE_READY:
    m_motors.move_absolute(0., 100);
    m_gate.retract();
    break;
  case State::SCORE_ONE:
  case State::SCORE_ALL:
    m_motors.move_voltage(12000);
    m_gate.extend();
    break;
  }
}

void Lever::reset() { m_state = State::INTAKE_READY; }

void Lever::scoreOne() { m_state = State::SCORE_ONE; }

void Lever::scoreAll() { m_state = State::SCORE_ALL; }

void Lever::compress() { m_state = State::COMPRESS; }