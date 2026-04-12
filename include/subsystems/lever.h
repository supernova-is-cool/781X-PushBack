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
  enum class ScoringSpeed { MAX, SLOW };

private:
  State m_state;
  /** Should only have a value when in a scoring state. */
  std::optional<ScoringSpeed> m_scoringSpeed;
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
  std::optional<ScoringSpeed> getScoringSpeed() const;
  bool isScoring() const;

  void reset();
  
  /** Scores one or a few balls at full speed. */
  void scoreOne();
  /** Scores all blocks in tube at full speed. */
  void scoreAll();
  /** Scores one or a few balls at slow speed. Ideal for center goal. */
  void scoreOneSlow();
  /** Scores all blocks in tube at slow speed. Ideal for center goal. */
  void scoreAllSlow();

  /** Pushes lever upwards a wee bit to apply more compression to the balls.
   * Useful for scoring on lower goal. */
  void compress();
};