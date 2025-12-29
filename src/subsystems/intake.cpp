#include "subsystems/intake.h"
#include "color.h"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include "robot.h"
#include <cmath>
#include <cstdio>
#include <optional>

#define POWER 127

Intake::Intake(pros::Motor &top, pros::Motor &bottom, pros::Distance &distance)
    : m_top(top), m_bottom(bottom), m_distance(distance), m_state(IDLE) {}

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
    m_top.move(-POWER);
    m_bottom.move(-POWER);
    break;

  case State::SCORING:
    m_top.move(POWER);
    m_bottom.move(POWER);
    break;

  case State::STORING:
    m_bottom.move(POWER);
    m_top.brake();
    break;

  case State::SLOW_OUTAKE:
    m_bottom.move(80);
    m_top.move(80);

  case State::SKILLS:
    m_bottom.move(POWER);
    m_top.move(0.8 * POWER);

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
void Intake::goToScoring() { setState(State::SCORING); }
void Intake::goToStoring() { setState(State::STORING); }
void Intake::goToSlowOutake() { setState(State::SLOW_OUTAKE); }
void Intake::goToSkills() { setState(State::SKILLS); }

void Intake::enableFiltering() { enableFilter = true; }
void Intake::disableFiltering() { enableFilter = false; }

