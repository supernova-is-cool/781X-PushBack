#pragma once

#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/optical.hpp"
#include <vector>

struct robotConfig {

  struct Pneumatics {
    pros::adi::Pneumatics filter;
    pros::adi::Pneumatics littleWill;
    pros::adi::Pneumatics leftElevator;
    pros::adi::Pneumatics rightElevator;
    pros::adi::Pneumatics aligner;

  private:
    friend struct robotConfig;
    static Pneumatics pneumatics;
  };

  struct Motors {
    pros::MotorGroup left;
    pros::MotorGroup right;
    pros::Motor topStage;
    pros::Motor bottomStage;

  private:
    friend struct robotConfig;
    static Motors motors;
  };

  struct Sensors {
    pros::Optical intakeColor;
    pros::Distance intakeDistance;
    pros::Rotation vert;
    pros::Rotation hori;
    pros::IMU imu;

  private:
    friend struct robotConfig;
    static Sensors sensors;
  };

  struct Dimensions {
    const float trackWidth;
    const float driveWheelDiameter;
    const float driveWheelRpm;

    const float vertEncDiameter;
    const float vertEncDistance;
    const float vertEncGearRatio;

    const float horiEncDiameter;
    const float horiEncDistance;
    const float horiEncGearRatio;

    const float drivetrainWidth;
    const float drivetrainLength;

    static Dimensions dimensions;
  };

  struct Tunables {
    const lemlib::ControllerSettings &lateralController;
    const lemlib::ControllerSettings &angularController;
    const float horizontalDrift;
    const float imuGain;

    lemlib::ExpoDriveCurve driveCurve;
    static Tunables tunables;

  private:
    friend struct RobotConfig;
  };

  Motors &motors;
  Pneumatics &pneumatics;
  Sensors &sensors;
  Dimensions &dimensions;
  Tunables &tunables;

  lemlib::OdomSensors makeSensors() const;
  lemlib::Drivetrain makeDrivetrain() const;

private:
  friend class Robot;
  static const robotConfig config;
};
