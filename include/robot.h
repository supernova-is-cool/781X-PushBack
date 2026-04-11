#pragma once
#include "auton/transform.h"
#include "color.h"
#include "config.h"
#include "subsystems/intake.h"
#include <functional>
#include <memory>

/**
 * @brief Parameters for Robot::moveStraight
 *
 * We use a struct to simplify customization. Robot::moveStraight has many
 * parameters and specifying them all just to set one optional param harms
 * readability. By passing a struct to the function, we can have named
 * parameters, overcoming the c/c++ limitation.
 */
struct MoveStraightParams {
  /** the maximum speed the robot can travel at. Value between 0-127. 127 by
   * default */
  float maxSpeed = 127;
  /** the minimum speed the robot can travel at. If set to a non-zero value, the
   * exit conditions will switch to less accurate but smoother ones. Value
   * between 0-127. 0 by default */
  float minSpeed = 0;
  /** distance between the robot and target point where the movement will exit.
   * Only has an effect if minSpeed is non-zero.*/
  float earlyExitRange = 0;

  std::optional<float> targetHeading;
};

typedef std::function<std::function<float()>(float targetHeading)>
    MoveStraightErrorFuncFactory;
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
  PneumaticGroup &lift;
  pros::Controller &gamepad;

  Laser &front;
  Laser &back;
  Laser &left;
  Laser &right;

  /**
   * Resets all PID controller settings and horizontal drift to config values.
   * Also resets the brake mode to coasting.
   */
  void resetControllerSettings();

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

  /**
   * @brief Drives straight, using the provided error function to determine when
   * to stop. The robot will attempt to maintain the same heading that it
   * started with.
   *
   * @param errorFuncFactory A factory function that returns an error function
   * to determine when to stop. This enables the error func to record the
   * initial state.
   */
  void moveStraight(MoveStraightErrorFuncFactory errorFuncFactory, int timeout,
                    MoveStraightParams params = {}, bool async = true);
  void moveToX(float targetX, int timeout, MoveStraightParams params = {},
               bool async = true);
  void moveToY(float targetY, int timeout, MoveStraightParams params = {},
               bool async = true);
  /**
   * @brief Moves straight to a line defined by the provided Pose. The robot
   * will attempt to maintain the same heading that it started with, and will
   * exit only once it reaches the line.
   *
   * @param line This pose is a point on the line with a heading parallel to the
   * line. Should be in compass degrees (0deg facing +y, 90deg facing +x).
   */
  void moveToLine(lemlib::Pose line, int timeout,
                  MoveStraightParams params = {}, bool async = true);
  void moveDistance(float distance, int timeout, MoveStraightParams params = {},
                    bool async = true);

  void setPose(lemlib::Pose pose, bool radians = false);
  lemlib::Pose getPose(bool radians = false, bool standardPosition = false);

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