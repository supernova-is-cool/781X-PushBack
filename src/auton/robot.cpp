#include "robot.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"

// Handles transformations for auton to work for both red and blue alliance

void Robot::setTransform(std::shared_ptr<auton::Transformation> transform) {
  m_transform = transform;
}

constexpr auto ALLIANCE_DEFAULT = ALLIANCE::RED;

void Robot::setAlliance(ALLIANCE alliance) {
  m_alliance = alliance;
  setTransform(std::make_shared<auton::AllianceTransform>(alliance));
}

ALLIANCE Robot::getAlliance() const { return m_alliance; }

COLOR Robot::getColor() const { return getAlliance(); }

void Robot::setPose(lemlib::Pose newPose, bool radians) {
  lemlib::Chassis::setPose(m_transform->transformPose(newPose), radians);
}

lemlib::Pose Robot::getPose(bool radians, bool standardPosition) {
  const lemlib::Pose compassPose =
      m_transform->transformPose(lemlib::Chassis::getPose(radians));
  if (standardPosition)
    return compassPose.withTheta((radians ? M_PI_2 : 90) - compassPose.theta);
  return compassPose;
}

void Robot::turnToPoint(lemlib::Pose target, int timeout,
                        lemlib::TurnToPointParams params, bool async) {
  const auto transformed = m_transform->transformPose(target);
  lemlib::Chassis::turnToPoint(
      transformed.x, transformed.y, timeout,
      {
          .forwards = params.forwards,
          .direction = m_transform->transformAngDir(params.direction),
          .maxSpeed = params.maxSpeed,
          .minSpeed = params.minSpeed,
          .earlyExitRange = params.earlyExitRange,
      },
      async);
}

void Robot::turnToHeading(float theta, int timeout,
                          lemlib::TurnToHeadingParams params, bool async) {
  const auto transformed = m_transform->transformHeading(theta);
  lemlib::Chassis::turnToHeading(
      transformed, timeout,
      {
          .direction = m_transform->transformAngDir(params.direction),
          .maxSpeed = params.maxSpeed,
          .minSpeed = params.minSpeed,
          .earlyExitRange = params.earlyExitRange,
      },
      async);
}

void Robot::swingToPoint(lemlib::Pose target, lemlib::DriveSide lockedSide,
                         int timeout, lemlib::SwingToPointParams params,
                         bool async) {
  const auto transformed = m_transform->transformPose(target);
  lemlib::Chassis::swingToPoint(
      transformed.x, transformed.y, m_transform->transformDriveSide(lockedSide),
      timeout,
      {
          .forwards = params.forwards,
          .direction = m_transform->transformAngDir(params.direction),
          .maxSpeed = params.maxSpeed,
          .minSpeed = params.minSpeed,
          .earlyExitRange = params.earlyExitRange,
      },
      async);
}

void Robot::swingToHeading(float theta, lemlib::DriveSide lockedSide,
                           int timeout, lemlib::SwingToHeadingParams params,
                           bool async) {
  const auto transformed = m_transform->transformHeading(theta);
  lemlib::Chassis::swingToHeading(
      transformed, m_transform->transformDriveSide(lockedSide), timeout,
      {
          .direction = m_transform->transformAngDir(params.direction),
          .maxSpeed = params.maxSpeed,
          .minSpeed = params.minSpeed,
          .earlyExitRange = params.earlyExitRange,
      },
      async);
}

void Robot::moveToPose(lemlib::Pose target, int timeout,
                       lemlib::MoveToPoseParams params, bool async) {
  const auto transformed = m_transform->transformPose(target);
  lemlib::Chassis::moveToPose(transformed.x, transformed.y, transformed.theta,
                              timeout,
                              {
                                  .forwards = params.forwards,
                                  .horizontalDrift = params.horizontalDrift,
                                  .lead = params.lead,
                                  .maxSpeed = params.maxSpeed,
                                  .minSpeed = params.minSpeed,
                                  .earlyExitRange = params.earlyExitRange,
                              },
                              async);
}

void Robot::moveToPoint(lemlib::Pose target, int timeout,
                        lemlib::MoveToPointParams params, bool async) {
  const auto transformed = m_transform->transformPose(target);
  lemlib::Chassis::moveToPoint(transformed.x, transformed.y, timeout,
                               {
                                   .forwards = params.forwards,
                                   .maxSpeed = params.maxSpeed,
                                   .minSpeed = params.minSpeed,
                                   .earlyExitRange = params.earlyExitRange,
                               },
                               async);
}