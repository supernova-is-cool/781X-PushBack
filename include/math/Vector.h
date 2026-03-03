#pragma once

#include "./NlPose.h"

struct Vector {
  float x;
  float y;

  static Vector between(const NlPose &from, const NlPose &to) {
    return Vector{to.x - from.x, to.y - from.y};
  }
  static Vector fromAngle(float angle) {
    return Vector{float(cos(angle)), float(sin(angle))};
  }

  float cross(const Vector &other) const { return x * other.y - y * other.x; }
};