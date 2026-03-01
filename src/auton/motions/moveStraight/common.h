/**
 * @file common.h
 *
 * Holds all the code for MoveStraight that is not specific to bot, enabling
 * testing it on pc.
 */

#include <cmath>
#include <format>
#include <functional>
#include <string>

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

  NlPose(float x, float y, float theta) : x(x), y(y), theta(theta) {}
  // Conversions between lemlib::Pose and NlPose
#ifndef GTEST
  operator lemlib::Pose() const { return lemlib::Pose(x, y, theta); }
  NlPose(lemlib::Pose pose) : x(pose.x), y(pose.y), theta(pose.theta) {}
#endif
};

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

class MoveToLineErrorFactory {
private:
  /** A point on the line */
  const NlPose m_point;
  /** Unit vector for direction of line */
  const Vector m_v;
  const std::function<NlPose()> m_getPose;
  const std::function<void(const std::string &)> m_errorSink;

public:
  /**
   * @brief Construct a new Move To Line Error Factory object
   *
   * @param line A vector representing the line to move to. Theta should be in
   * standard radians (ie. 0deg facing +y axis, 90deg facing +x axis)
   * @param getPose A function that returns the current pose of the robot. Theta
   * should be in standard radians.
   * @param errorSink Callback where errors messages can be sent.
   */
  MoveToLineErrorFactory(const NlPose &line, std::function<NlPose()> getPose,
                         std::function<void(const std::string &)> errorSink)
      : m_point(line), m_v(Vector::fromAngle(line.theta)), m_getPose(getPose),
        m_errorSink(errorSink) {};
  std::function<float()> operator()() {
    const float startTheta = m_getPose().theta;
    const Vector u = Vector::fromAngle(startTheta);
    const float u_cross_v = u.cross(m_v);
    if (std::abs(u_cross_v) < sin(1 * M_PI / 180)) {
      // If the robot is facing parallel to the line, return 0 for the error
      // function, since we won't be able to reach the line by driving straight.
      m_errorSink(std::format(
          "Since the robot is facing parallel to the line in moveToLine(),"
          "the bot cannot reach line, and thus therefore the motion will be "
          "skipped. Line theta: {}, Robot theta: {}",
          m_point.theta, m_getPose().theta));
      return []() { return 0; };
    }
    return [=]() -> float {
      const Vector PQ_vec = Vector::between(m_getPose(), m_point);
      return PQ_vec.cross(m_v) / u_cross_v;
    };
  };
};
