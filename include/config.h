#pragma once

#include "auton/laser.h"
#include "lemlib/chassis/chassis.hpp"

struct robotConfig {

  struct Pneumatics {
    pros::adi::Pneumatics matchLoader;
    pros::adi::Pneumatics descore;
    pros::adi::Pneumatics bottom_gate;
    pros::adi::Pneumatics top_gate;
    pros::adi::Pneumatics park;
    pros::adi::Pneumatics clamp;

  private:
    friend struct robotConfig;
    static Pneumatics pneumatics;
  };

  struct Motors {
    pros::MotorGroup left;
    pros::MotorGroup right;
    pros::MotorGroup intake;

  private:
    friend struct robotConfig;
    static Motors motors;
  };

  struct Sensors {
    pros::Distance intakeDistance;
    pros::Rotation vert;
    pros::Rotation hori;
    pros::IMU imu;
    Laser front;
    Laser back;
    Laser left;
    Laser right;

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
