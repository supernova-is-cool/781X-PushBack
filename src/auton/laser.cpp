#include "auton/laser.h"
#include "dimensions.h"
#include "pros/error.h"
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