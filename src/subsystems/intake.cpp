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

Intake::Intake(pros::Motor &top, pros::Motor &bottom,
               pros::adi::Pneumatics &bottom_gate,
               pros::adi::Pneumatics &top_gate)
    : m_top(top), m_bottom(bottom), m_state(IDLE), m_bottom_gate(bottom_gate),
      m_top_gate(top_gate) {}

void Intake::runTask() {

  /*
  while (true) {
    bool blockDetected = (m_distance.get() < 80);
    auto sensedBlock = getSensedRing();
    printf("%i\n", (int)blockDetected);

    if (sensedBlock.has_value() && enableFilter) {
      COLOR color = sensedBlock.value();

      if (color != m_targetColor) {
        m_filter.retract();
        isOpen = true;
      } else {
        m_filter.extend();
        isOpen = false;
      }
    }
    */

  switch (m_state) {
  case State::EMERGENCY_STOP:
    m_top.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    m_bottom.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    m_top.brake();
    m_bottom.brake();
    return;

  case State::IDLE:
    m_top.brake();
    m_bottom.brake();
    break;

  case State::OUTAKE:
    m_bottom_gate.retract();
    m_top_gate.retract();
    m_top.move(-POWER);
    m_bottom.move(-POWER);
    break;

  case State::MIDDLE:
    m_bottom_gate.extend();
    m_top_gate.extend();
    m_top.move(POWER * 0.65);
    m_bottom.move(POWER * 0.65);
    break;

  case State::TOP:
    m_bottom_gate.retract();
    m_top_gate.retract();
    m_top.move(POWER);
    m_bottom.move(POWER);
    break;

  case State::STORING:
    m_bottom_gate.extend();
    m_top_gate.retract();
    m_bottom.move(POWER);
    m_top.move(POWER);
    break;

  case State::SLOW_OUTAKE:
    m_bottom.move(-80);
    m_top.move(-80);

  default:
    break;
  }

  m_top.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  m_bottom.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  pros::delay(10);
}

const Intake::State &Intake::getState() { return m_state; }
void Intake::setState(State state) { m_state = state; }

void Intake::emergencyStop() { setState(State::EMERGENCY_STOP); }
void Intake::goToIdle() { setState(State::IDLE); }
void Intake::goToOutaking() { setState(State::OUTAKE); }
void Intake::goToTOP() { setState(State::TOP); }
void Intake::goToMIDDLE() { setState(State::MIDDLE); }
void Intake::goToStoring() { setState(State::STORING); }

void Intake::enableFiltering() { enableFilter = true; }
void Intake::disableFiltering() { enableFilter = false; }
