#pragma once

#include "color.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "subsystems.h"

class Intake : public subsystem {
public:
  enum class SpinState {
    IDLE,
    INTAKING,
    SLOW_INTAKING,
    OUTTAKING,
    SLOW_OUTTAKING,
    EVEN_SLOWER,
    EMERGENCY_STOP
  };
  enum class GateState { STORING, MIDDLE_GOAL, LONG_GOAL };

private:
  SpinState m_spinState;
  GateState m_gateState;
  pros::MotorGroup &m_motors;
  COLOR m_targetColor = COLOR::RED;
  bool enableFilter = false;
  pros::adi::Pneumatics &m_bottom_gate;
  pros::adi::Pneumatics &m_top_gate;

public:
  explicit Intake(pros::MotorGroup &motors, pros::adi::Pneumatics &bottom_gate,
                  pros::adi::Pneumatics &top_gate);

  void runTask() override;

  const SpinState &getSpinState() const;
  void setSpinState(SpinState state);

  const GateState &getGateState() const;
  void setGateState(GateState state);

  void emergencyStop();
  void goToIdle();
  void goToStoring();
  void goToTOP();
  void goToMIDDLE();
  void goToSlow();
  void goToOuttaking();

  void enableFiltering();
  void disableFiltering();

  std::optional<COLOR> getSensedRing();
  void setFilterColor();
  COLOR getFilterColor() const;
};
