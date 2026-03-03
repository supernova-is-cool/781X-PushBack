#pragma once

#include <cmath>
#ifndef GTEST
#include "lemlib/pose.hpp"
#endif

/** Not lemlib pose. Lets us test on pc. */
struct NlPose {
  float x;
  float y;
  /** Should always be in standard radians. */
  float theta;

  float distance(const NlPose &other) const {
    return std::hypot(other.x - x, other.y - y);
  }
  float angle(const NlPose &other) const {
    return std::atan2(other.y - y, other.x - x) - theta;
  }
  NlPose rotate(float angle) const {
    float cosA = std::cos(angle);
    float sinA = std::sin(angle);
    return NlPose{x * cosA - y * sinA, x * sinA + y * cosA, theta + angle};
  }

  NlPose(float x, float y, float theta) : x(x), y(y), theta(theta) {}
  // Conversions between lemlib::Pose and NlPose
#ifndef GTEST
  operator lemlib::Pose() const { return lemlib::Pose(x, y, theta); }
  NlPose(lemlib::Pose pose) : x(pose.x), y(pose.y), theta(pose.theta) {}
#endif
};
