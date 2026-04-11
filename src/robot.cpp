#include "robot.h"
#include "config.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/error.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include <cstdint>
#include <memory>
#include <vector>

class CustomIMU : public pros::IMU {
  public:
    CustomIMU(int port, double scalar)
      : pros::IMU(port),
        m_port(port),
        m_scalar(scalar) {}
    virtual double get_rotation() const override {
      return pros::c::imu_get_rotation(m_port) * m_scalar;
    }
  private:
    const int m_port;
    const double m_scalar;
};

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
      lift(robotConfig::config.pneumatics.lift),
      m_gamepad(pros::E_CONTROLLER_MASTER), gamepad(m_gamepad),
      m_transform(std::make_shared<auton::AllianceTransform>(COLOR::RED)),
      m_config(robotConfig::config),

      m_intake(robotConfig::config.motors.intake,
               robotConfig::config.pneumatics.bottom_gate,
               robotConfig::config.pneumatics.top_gate),
      intake(m_intake) {}

Robot Robot::instance{robotConfig::config};

/**  Creates a LemLib PID object from the provided controller settings. */
static lemlib::PID pidFromSettings(const lemlib::ControllerSettings &settings) {
  return lemlib::PID{settings.kP, settings.kI, settings.kD,
                     settings.windupRange, false};
}

void Robot::resetControllerSettings() {
  lemlib::Chassis::lateralSettings = m_config.tunables.lateralController;
  lemlib::Chassis::angularSettings = m_config.tunables.angularController;
  lemlib::Chassis::lateralPID =
      pidFromSettings(m_config.tunables.lateralController);
  lemlib::Chassis::lateralLargeExit = {
      m_config.tunables.lateralController.largeError,
      (int)m_config.tunables.lateralController.largeErrorTimeout};
  lemlib::Chassis::lateralSmallExit = {
      m_config.tunables.lateralController.smallError,
      (int)m_config.tunables.lateralController.smallErrorTimeout};
  lemlib::Chassis::angularPID =
      pidFromSettings(m_config.tunables.angularController);
  lemlib::Chassis::angularLargeExit = {
      m_config.tunables.angularController.largeError,
      (int)m_config.tunables.angularController.largeErrorTimeout};
  lemlib::Chassis::angularSmallExit = {
      m_config.tunables.angularController.smallError,
      (int)m_config.tunables.angularController.smallErrorTimeout};

  lemlib::Chassis::drivetrain.horizontalDrift =
      m_config.tunables.horizontalDrift;
  // Default to coasting
  lemlib::Chassis::setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}