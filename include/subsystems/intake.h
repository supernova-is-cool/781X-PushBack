#pragma once

#include "pros/motor_group.hpp"
#include "subsystems.h"

class Intake : public subsystem {
public:
  enum class SpinState {
    IDLE,
    INTAKING,
    OUTTAKING,
    SLOW_OUTTAKING,
  } m_spinState;
  pros::MotorGroup &m_motors;
  void runTask() override;

public:
  explicit Intake(pros::MotorGroup &motors);

  void stop();
  void intake();
  void outtake();
  void slowOuttake();
};
