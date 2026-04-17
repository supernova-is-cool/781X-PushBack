#include "robot.h"
#include "config.h"
#include "pros/motors.h"
#include <memory>

class CustomIMU : public pros::IMU {
public:
  CustomIMU(int port, double scalar)
      : pros::IMU(port), m_port(port), m_scalar(scalar) {}
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
          nullptr,
          new CustomIMU(this->sensors.imu.get_port(), this->tunables.imuGain)};
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

Robot::Robot(const robotConfig &conf)
    : lemlib::Chassis(conf.makeDrivetrain(), conf.tunables.lateralController,
                      conf.tunables.angularController, conf.makeSensors(),
                      &conf.tunables.driveCurve),
      matchLoader(conf.pneumatics.matchLoader),
      descore(conf.pneumatics.descore), park(conf.pneumatics.park),
      m_frontLaser(conf.sensors.front), m_backLaser(conf.sensors.back),
      m_leftLaser(conf.sensors.left), m_rightLaser(conf.sensors.right),
      lift(conf.pneumatics.lift),
      m_transform(std::make_shared<auton::AllianceTransform>(COLOR::RED)),
      m_config(conf), intake(conf.motors.intake),
      ballCounter(conf.sensors.ballCounter),
      intakeLift(conf.pneumatics.intakeLift),
      lever(conf.motors.lever, conf.pneumatics.gate, conf.sensors.tubeExit,
            conf.pneumatics.lift) {}

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