#pragma once
#include "subsystems.h"

class Intake : public subsystem {

public:
  enum State {
    // State where the intake does not move
    IDLE,
    // State where the intake stores balls, but does not score them
    STORING,
    // State where the intake scores the balls
    SCORING,
    // State where the intake outakes the balls
    OUTAKE,
    // Emergency stop
    EMERGENCY_STOP
  };

private:
  State m_state;
  pros::Motor &m_top;
  pros::Motor &m_bottom;

public:
  void runTask() override;

  const State &getState();
  void setState(State state);

  void emergencyStop();
  void goToIdle();
  void goToStoring();
  void goToScoring();
  void goToOutaking();

  Intake(pros::Motor &top, pros::Motor &bottom);
};
