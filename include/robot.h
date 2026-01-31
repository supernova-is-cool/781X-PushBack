#pragma once
#include "auton/transform.h"
#include "color.h"
#include "config.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "subsystems/intake.h"
#include <memory>

/**
 * @brief Provides an abstracted interface for controlling the robot and reading
 * from sensors. Follows the singleton pattern.
 */
class Robot : private lemlib::Chassis {
private:
  Robot(const robotConfig &config);

  /** @brief Should ever be one instance of Robot, and that's this one. */
  static Robot instance;

public:
  const robotConfig &m_config;

private:
  pros::Controller m_gamepad;
  Intake m_intake;

  ALLIANCE m_alliance;

public:
  std::shared_ptr<auton::Transformation> m_transform;

public:
  /**
   * @brief Gets the robot instance.
   * If it has not been previously constructed (instance == nullptr), then
   * this method will construct it
   */
  inline static Robot &get() { return instance; };

  Intake &intake;

  /** Extended is down/active. */
  pros::adi::Pneumatics matchLoader;
  /** Extended is up/inactive. */
  pros::adi::Pneumatics descore;

  pros::adi::Pneumatics park;

  pros::adi::Pneumatics clamp;

  pros::Controller &gamepad;

  pros::Distance &front;
  pros::Distance &back;
  pros::Distance &left;
  pros::Distance &right;

  void setTransform(std::shared_ptr<auton::Transformation> transform);
  /** @brief Sets the transformation to an AllianceTransform with the
   * specified alliance. */
  void setAlliance(ALLIANCE alliance);

  ALLIANCE getAlliance() const;
  COLOR getColor() const;

  using lemlib::Chassis::angularLargeExit;
  using lemlib::Chassis::angularPID;
  using lemlib::Chassis::angularSettings;
  using lemlib::Chassis::angularSmallExit;
  using lemlib::Chassis::arcade;
  using lemlib::Chassis::calibrate;
  using lemlib::Chassis::cancelAllMotions;
  using lemlib::Chassis::cancelMotion;
  using lemlib::Chassis::curvature;
  using lemlib::Chassis::isInMotion;
  using lemlib::Chassis::lateralLargeExit;
  using lemlib::Chassis::lateralPID;
  using lemlib::Chassis::lateralSettings;
  using lemlib::Chassis::lateralSmallExit;
  using lemlib::Chassis::setBrakeMode;
  using lemlib::Chassis::tank;
  using lemlib::Chassis::waitUntil;
  using lemlib::Chassis::waitUntilDone;

  void setPose(lemlib::Pose pose, bool radians = false);
  lemlib::Pose getPose(bool radians = false);

  void turnToPoint(lemlib::Pose target, int timeout,
                   lemlib::TurnToPointParams params = {}, bool async = true);
  void turnToHeading(float theta, int timeout,
                     lemlib::TurnToHeadingParams params = {},
                     bool async = true);
  void swingToPoint(lemlib::Pose target, lemlib::DriveSide lockedSide,
                    int timeout, lemlib::SwingToPointParams params = {},
                    bool async = true);
  void swingToHeading(float theta, lemlib::DriveSide lockedSide, int timeout,
                      lemlib::SwingToHeadingParams params = {},
                      bool async = true);
  void moveToPose(lemlib::Pose target, int timeout,
                  lemlib::MoveToPoseParams params = {}, bool async = true);
  void moveToPoint(lemlib::Pose target, int timeout,
                   lemlib::MoveToPointParams params = {}, bool async = true);
};

inline Robot &bot = Robot::get();