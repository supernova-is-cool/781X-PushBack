#include "auton/laser.h"
#include "dimensions.h"
#include "lemlib/chassis/chassis.hpp"
#include "log.h"
#include "pros/error.h"
#include "robot.h"
#include "util.h"
#include <cstddef>

Laser::Laser(size_t port, NlPose offset) : m_sensor(port), m_offset(offset) {}

constexpr float MM_TO_IN = 0.0393700787;
std::optional<float> Laser::measure() {
  float pros_measurement = m_sensor.get_distance();
  if (pros_measurement == 9999 || pros_measurement == PROS_ERR) {
    return std::nullopt;
  } else {
    return pros_measurement * MM_TO_IN;
  }
}

std::optional<float> Laser::getBotX(float botTheta) {
  return measure().transform([&](float d) -> float {
    const float sensorTheta = botTheta + m_offset.theta;
    const float cosTheta = cos(sensorTheta);
    const float distToWall =
        std::abs(d * cosTheta + m_offset.rotate(botTheta).x);
    if (cosTheta > 0) {
      // Facing towards wall at MAX_X
      return dimensions::field::MAX_X - distToWall;
    } else {
      // Facing towards wall at MIN_X
      return dimensions::field::MIN_X + distToWall;
    }
  });
}

std::optional<float> Laser::getBotY(float botTheta) {
  return measure().transform([&](float d) -> float {
    const float sensorTheta = botTheta + m_offset.theta;
    const float sinTheta = sin(sensorTheta);
    const float distToWall =
        std::abs(d * sinTheta + m_offset.rotate(botTheta).y);
    if (sinTheta > 0) {
      // Facing towards wall at MAX_Y
      return dimensions::field::MAX_Y - distToWall;
    } else {
      // Facing towards wall at MIN_Y
      return dimensions::field::MIN_Y + distToWall;
    }
  });
}

bool Laser::resetX() {
  auto botAsChassis = (lemlib::Chassis *)&bot;
  float botTheta = botAsChassis->getPose(true, true).theta;

  if (auto maybeX = getBotX(botTheta)) {
    log("Reset X to {} using laser. theta is {}deg", *maybeX,
        auton::util::trigAngleToHeading(botTheta));
    botAsChassis->setPose(botAsChassis->getPose().withX(*maybeX));
    return true;
  } else {
    log("Laser measurement failed. Cannot reset X.");
    return false;
  }
}

bool Laser::resetY() {
  auto botAsChassis = (lemlib::Chassis *)&bot;
  float botTheta = botAsChassis->getPose(true, true).theta;

  if (auto maybeY = getBotY(botTheta)) {
    botAsChassis->setPose(botAsChassis->getPose().withY(*maybeY));
    log("Reset Y to {} using laser. theta is {}deg", *maybeY,
        auton::util::trigAngleToHeading(botTheta));
    return true;
  } else {
    log("Laser measurement failed. Cannot reset Y.");
    return false;
  }
}