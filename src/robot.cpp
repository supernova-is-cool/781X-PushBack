#include "robot.h"
#include "config.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/error.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

lemlib::Drivetrain robotConfig::makeDrivetrain() const {
  return {
      &this->motors.left,
      &this->motors.right,
      this->dimensions.trackWidth,
      this->dimensions.driveWheelDiameter,
      this->dimensions.driveWheelRpm,
      this->tunables.horizontalDrift,
  };
}

static std::optional<std::unique_ptr<lemlib::TrackingWheel>>
makeTrackingWheel(pros::Rotation *encoder, float wheelDiameter, float distance,
                  float gearRatio = 1) {
  if (encoder->is_installed())
    return std::make_unique<lemlib::TrackingWheel>(encoder, wheelDiameter,
                                                   distance, gearRatio);
  else
    return std::nullopt;
}

/** @brief Provides an encoder-like interface for a motor group with redundancy.
 * If a motor disconnects, that motor will be discarded and only the working
 * motors will be used to calculate the motor position.  */
class MotorGroupEncoder {
  /** Working Ports. Any ports that fail, even once, will be forgotten. */
  std::vector<std::int8_t> m_ports;
  /** The motor cartridge of the motors */
  const pros::motor_gearset_e_t m_gearing;

public:
  MotorGroupEncoder(const std::vector<std::int8_t> &ports,
                    pros::motor_gearset_e_t gearing)
      : m_ports{ports}, m_gearing(gearing) {}

  /**
   * @returns The mean number of rotations for the functioning motors.
   * @returns PROS_ERR_F if all motors have failed.
   */
  float get_position() {
    /** How many rotations each motor has experienced. */
    std::vector<double> rotations{};
    std::vector<std::int8_t> portsToRemove{};
    for (const auto &port : m_ports) {
      pros::c::motor_set_encoder_units(
          port, pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_ROTATIONS);
      pros::c::motor_set_gearing(port, m_gearing);
      const auto rots = pros::c::motor_get_position(port);
      // printf("%2.4f, ", rots);
      // Remove any ports that have disconnected, as subsequent measurements
      // will not be accurate (the motor's position will be reset to zero)
      // even if the motor is reconnected. We could attempt to handle this,
      // but it should be a very small edge case that doesn't matter much.
      if (pros::c::motor_get_type(port) == pros::E_MOTOR_TYPE_INVALID)
        portsToRemove.push_back(port);
      else
        rotations.push_back(rots);
    }
    // Actually remove the ports
    for (const auto &port : portsToRemove) {
      // printf("Removing port %d\n", port);
      m_ports.erase(std::remove(m_ports.begin(), m_ports.end(), port),
                    m_ports.end());
    }

    if (rotations.size() == 0)
      return PROS_ERR_F;
    // Take the average of all the rotations
    double sum = 0;
    for (const auto &pos : rotations) {
      sum += pos;
    }
    double avg = sum / rotations.size();
    // printf("-> %2.4f\n", avg);
    return avg;
  }
};

/**
 * @brief Mimics a rotation sensor to be used as a tracking wheel for lemlib
 * odom. Takes the average of the left and right motors, to achieve a more
 * accurate reading. If a motor disconnects, even for a moment, it will be
 * removed from the average.
 */
class DriveEncoder : public pros::Rotation {
private:
  MotorGroupEncoder m_leftEnc;
  MotorGroupEncoder m_rightEnc;

public:
  /**
   * @brief Construct a new Drive Encoder object
   *
   * @param leftPorts Left motor ports
   * @param rightPorts Right motor ports
   * @param gearing The motor cartridge of the motors
   */
  DriveEncoder(const std::vector<std::int8_t> &leftPorts,
               const std::vector<std::int8_t> &rightPorts,
               pros::motor_gearset_e_t gearing)
      : pros::Rotation(30), m_leftEnc(leftPorts, gearing),
        m_rightEnc(rightPorts, gearing) {}

  virtual std::int32_t get_position() const override {
    // printf("Left: ");
    auto leftPos = const_cast<MotorGroupEncoder *>(&m_leftEnc)->get_position();
    // printf("Right: ");
    auto rightPos =
        const_cast<MotorGroupEncoder *>(&m_rightEnc)->get_position();
    if (leftPos == PROS_ERR_F || rightPos == PROS_ERR_F)
      return INT32_MAX;
    auto avgPos = (leftPos + rightPos) / 2.f;
    // Convert average to centidegrees
    return avgPos / 2.f * 360.f * 100.f;
  }
};

/**
 * Uses drivetrain motors to create a tracking wheel.
 * Differs from lemlib's default drive tracking wheel in that it takes the
 * average of all motors' positions, rather than just the left motors.
 *
 * Assumes motors have blue cartridges.
 */
static std::unique_ptr<lemlib::TrackingWheel>
makeDrivetrainTrackingWheel(const robotConfig::Motors &motors,
                            const robotConfig::Dimensions &dims) {
  std::vector<std::int8_t> ports{};

  return std::make_unique<lemlib::TrackingWheel>(
      new DriveEncoder(motors.left.get_port_all(), motors.right.get_port_all(),
                       pros::motor_gearset_e_t::E_MOTOR_GEAR_BLUE),
      dims.driveWheelDiameter, 0, dims.driveWheelRpm / 600.f);
}

lemlib::OdomSensors robotConfig::makeSensors() const {
  // Use vertical tracking wheel if installed, otherwise use drivetrain
  auto vertTrackingWheel =
      makeTrackingWheel(&this->sensors.vert, this->dimensions.vertEncDiameter,
                        this->dimensions.vertEncDistance,
                        this->dimensions.vertEncGearRatio)
          .transform([](auto tw) { return tw.release(); })
          .or_else([*this] -> std::optional<lemlib::TrackingWheel *> {
            return {makeDrivetrainTrackingWheel(this->motors, this->dimensions)
                        .release()};
          })
          .value();
  printf("vertTrackingWheel is nullptr?: %i\n", vertTrackingWheel == nullptr);
  auto horiTrackingWheel =
      makeTrackingWheel(&this->sensors.hori, this->dimensions.horiEncDiameter,
                        this->dimensions.horiEncDistance,
                        this->dimensions.horiEncGearRatio)
          .transform([](auto tw) { return tw.release(); })
          .value_or(nullptr);
  printf("horiTrackingWheel is nullptr?: %i\n", horiTrackingWheel == nullptr);
  return {vertTrackingWheel, nullptr, horiTrackingWheel, nullptr,
          &this->sensors.imu};
}

/*
static std::shared_ptr<Odom> makeRealOdom(const robotConfig& config) {
  return std::make_shared<LemLibOdom>(config.makeSensors(),
                                      config.makeDrivetrain());
}

static std::shared_ptr<SimOdom> makeSimOdom(const robotConfig& config) {
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
      m_gamepad(pros::E_CONTROLLER_MASTER), gamepad(m_gamepad),
      m_transform(std::make_shared<auton::AllianceTransform>(COLOR::RED)),
      m_config(robotConfig::config),
      m_intake(robotConfig::config.motors.topStage,
               robotConfig::config.motors.bottomStage),
      intake(m_intake) {}

Robot Robot::instance{robotConfig::config};