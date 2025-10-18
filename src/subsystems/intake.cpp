#include "subsystems/intake.h"
#include "color.h"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include "robot.h"
#include <cmath>
#include <cstdio>
#include <optional>

#define POWER 127

Intake::Sensor::Sensor(pros::Optical &optical) : m_optical(optical) {}

std::optional<COLOR> Intake::Sensor::getBlock() const {
  if (m_optical.get_proximity() < 200)
    return std::nullopt;

  const float hueRem = std::remainder(m_optical.get_hue(), 360);

  if (std::abs(hueRem) < 60) {
    return COLOR::RED;
  } else {
    return COLOR::BLUE;
  }
}

Intake::Intake(pros::Motor &top, pros::Motor &bottom, pros::Distance &distance,
               pros::adi::Pneumatics &filter, Sensor &sensor)
    : m_top(top), m_bottom(bottom), m_distance(distance), m_filter(filter),
      m_state(IDLE), m_sensor(sensor) {}

void Intake::runTask() {

  while (true) {
    bool blockDetected = (m_distance.get() < 80);
    auto sensedBlock = getSensedRing();
    printf("%i\n", (int)blockDetected);

    if (sensedBlock.has_value()) {
      COLOR color = sensedBlock.value();

      if (color != m_targetColor) {
          m_filter.retract();
          isOpen = true;
      } else {
          m_filter.extend();
          isOpen = false;
      }
    }

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
      break;

    default:
      break;
    }

    m_top.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    m_bottom.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    pros::delay(10);
  }
}

const Intake::State &Intake::getState() { return m_state; }
void Intake::setState(State state) { m_state = state; }

void Intake::emergencyStop() { setState(State::EMERGENCY_STOP); }
void Intake::goToIdle() { setState(State::IDLE); }
void Intake::goToOutaking() { setState(State::OUTAKE); }
void Intake::goToScoring() { setState(State::SCORING); }
void Intake::goToStoring() { setState(State::STORING); }

std::optional<COLOR> Intake::getSensedRing() { return m_sensor.getBlock(); }

void Intake::setFilterColor() {
  // Toggle target color
  if (m_targetColor == COLOR::RED)
    m_targetColor = COLOR::BLUE;
  else
    m_targetColor = COLOR::RED;

  printf("Filter color set to: %s\n",
         m_targetColor == COLOR::RED ? "RED" : "BLUE");
}

COLOR Intake::getFilterColor() const { return m_targetColor; }
