#include "robot.h"
#include "config.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/error.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include <cstdint>
#include <memory>
#include <vector>
#include "subsystems/imu.hpp"

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
          nullptr, new CustomIMU(this->sensors.imu.get_port(), this->tunables.imuGain)};
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
      matchLoader(robotConfig::config.pneumatics.matchLoader),
      descore(robotConfig::config.pneumatics.descore),
      park(robotConfig::config.pneumatics.park),
      clamp(robotConfig::config.pneumatics.clamp),
      front(robotConfig::config.sensors.front),
      back(robotConfig::config.sensors.back),
      left(robotConfig::config.sensors.left),
      right(robotConfig::config.sensors.right),
      m_gamepad(pros::E_CONTROLLER_MASTER), gamepad(m_gamepad),
      m_transform(std::make_shared<auton::AllianceTransform>(COLOR::RED)),
      m_config(robotConfig::config),

      m_intake(robotConfig::config.motors.topStage,
               robotConfig::config.motors.bottomStage,
               robotConfig::config.pneumatics.bottom_gate,
               robotConfig::config.pneumatics.top_gate),
      intake(m_intake) {}

Robot Robot::instance{robotConfig::config};
