#pragma once

#include "pneumatics.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "subsystems/subsystems.h"
#include <optional>

class Lever : public subsystem {
public:
  enum class State { INTAKE_READY, SCORE_ONE, SCORE_ALL, COMPRESS };

private:
  State m_state;
  /** First sensed position for scoring one ball. */
  std::optional<float> m_scoreOneSensedPosition;
  /** Start time of the state. Used to implement a timeout. */
  std::optional<size_t> m_scoreOneStartTime;

  pros::MotorGroup &m_motors;
  pros::adi::Pneumatics m_gate;
  pros::Optical m_exitSensor;
  const PneumaticGroup &m_lift;

  /** Checks if a block is sensed by the exit sensor. */
  bool isBlockExiting();

  void runTask() override;

public:
  Lever(pros::MotorGroup &motors, pros::adi::Pneumatics gate,
        pros::Optical exitSensor, const PneumaticGroup &lift);

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