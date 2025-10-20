#include "robot.h"
#include "config.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/error.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include <cstdint>
#include <memory>
#include <vector>

lemlib::Drivetrain robotConfig::makeDrivetrain() const {
  return {&this->motors.left,
          &this->motors.right,
          this->dimensions.trackWidth,
          this->dimensions.driveWheelDiameter,
          this->dimensions.driveWheelRpm,
          this->tunables.horizontalDrift};
}

lemlib::OdomSensors robotConfig::makeSensors() const {
  return {new lemlib::TrackingWheel(&this->sensors.vert,
                                    this->dimensions.vertEncDiameter,
                                    this->dimensions.vertEncDistance,
                                    this->dimensions.vertEncGearRatio),
          nullptr,
          new lemlib::TrackingWheel(&this->sensors.hori,
                                    this->dimensions.horiEncDiameter,
                                    this->dimensions.horiEncDistance,
                                    this->dimensions.horiEncGearRatio),
          nullptr, &this->sensors.imu};
}
/*
static std::shared_ptr<Odom> makeRealOdom(const robotConfig& config) {
  return std::make_shared<LemLibOdom>(config.makeSensors(),
                                      config.makeDrivetrain());
}

static std::shared_ptr<SimOdom> makeSimOdom(const RobotConfig& config) {
  return std::make_shared<SimOdom>();
}
*/

Robot::Robot(const robotConfig &m_config)
    : lemlib::Chassis(m_config.makeDrivetrain(),
                      m_config.tunables.lateralController,
                      m_config.tunables.angularController,
                      robotConfig::config.makeSensors(),
                      &robotConfig::config.tunables.driveCurve),
      filter(robotConfig::config.pneumatics.filter),
      littleWill(robotConfig::config.pneumatics.littleWill),
      leftElevator(robotConfig::config.pneumatics.leftElevator),
      rightElevator(robotConfig::config.pneumatics.rightElevator),
      aligner(robotConfig::config.pneumatics.aligner),
      m_gamepad(pros::E_CONTROLLER_MASTER), gamepad(m_gamepad),
      m_transform(std::make_shared<auton::AllianceTransform>(COLOR::RED)),
      m_config(robotConfig::config),

      intakeSensor(robotConfig::config.sensors.intakeColor),

      m_intake(robotConfig::config.motors.topStage,
               robotConfig::config.motors.bottomStage,
               robotConfig::config.sensors.intakeDistance,
               robotConfig::config.pneumatics.filter, intakeSensor),

      intake(m_intake) {}

Robot Robot::instance{robotConfig::config};
