#pragma once

#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "subsystems/subsystems.h"

class Lever : public subsystem {
public:
  enum class State { INTAKE_READY, SCORE_ONE, SCORE_ALL, COMPRESS };

private:
  State m_state;
  pros::MotorGroup &m_motors;
  pros::adi::Pneumatics m_gate;
  void runTask() override;

public:
  Lever(pros::MotorGroup &motors, pros::adi::Pneumatics gate);

  State getState() const;
  bool isScoring() const;

  void reset();
  /** Scores one or.a few balls. */
  void scoreOne();
  void scoreAll();
  /** Pushes lever upwards a wee bit to apply more compression to the balls.
   * Useful for scoring on lower goal. */
  void compress();
};