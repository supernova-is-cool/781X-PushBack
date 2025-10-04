#pragma once
#include "config.h"
#include "pros/adi.hpp"
#include "subsystems/selector.h"
#include "lemlib/chassis/chassis.hpp"
#include "color.h"
#include "auton/transform.h"
#include <memory>
#include "subsystems/intake.h"

/**
 * @brief Provides an abstracted interface for controlling the robot and reading
 * from sensors. Follows the singleton pattern.
 */
class Robot : private lemlib::Chassis {
  private:
    Robot(const robotConfig& config);

    /** @brief Should ever be one instance of Robot, and that's this one. */
    static Robot instance;
  public:
    const robotConfig& m_config;
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
    inline static Robot& get() { return instance; };

    Intake& intake;


    // pros::adi::Pneumatics& kicker;
    pros::adi::Pneumatics& filter;
    /** Extended is down/active. */
    pros::adi::Pneumatics littleWill;
    /** Extended is down/active. */
    pros::adi::Pneumatics leftElevator;
    pros::adi::Pneumatics rightElevator;
    /** Extended is up/inactive. */
    pros::Controller& gamepad;

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
    using lemlib::Chassis::cancelAllMotions;
    using lemlib::Chassis::cancelMotion;
    using lemlib::Chassis::curvature;
    using lemlib::Chassis::drivetrain;
    using lemlib::Chassis::isInMotion;
    using lemlib::Chassis::lateralLargeExit;
    using lemlib::Chassis::lateralPID;
    using lemlib::Chassis::lateralSettings;
    using lemlib::Chassis::lateralSmallExit;
    using lemlib::Chassis::setBrakeMode;
    using lemlib::Chassis::tank;
    using lemlib::Chassis::waitUntil;
    using lemlib::Chassis::waitUntilDone;

    void calibrate();

    void setPose(lemlib::Pose pose, float stdDev = .5);
    lemlib::Pose getPose();

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

    /**
     * @warning Didn't work the single time I tested it.
     * @brief Move the a point on the bot to a desired point and heading.
     *
     * @param target The desired target for the point.
     * @param timeout How long to run the motion.
     * @param offset The offset of the point relative to the center of the
     * robot. X should be forwards/backwards, and Y, side to side.
     */
    void customMoveToPose(lemlib::Pose target, size_t timeout,
                          lemlib::Pose offset = {0, 0},
                          lemlib::MoveToPoseParams params = {},
                          bool async = true);
};

inline Robot& bot = Robot::get();