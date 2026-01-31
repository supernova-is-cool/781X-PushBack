#pragma once
#include "color.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include "subsystems.h"
#include <optional>

class Intake : public subsystem {

public:
  enum State { IDLE, STORING, MIDDLE, TOP, OUTAKE, SLOW_OUTAKE, EMERGENCY_STOP };


private:
  State m_state;
  pros::Motor &m_top;
  pros::Motor &m_bottom;
  COLOR m_targetColor = COLOR::RED;
  bool enableFilter = false;
  pros::adi::Pneumatics &m_bottom_gate;
  pros::adi::Pneumatics &m_top_gate;

public:
  explicit Intake(pros::Motor &top, pros::Motor &bottom,
                  pros::adi::Pneumatics &bottom_gate, pros::adi::Pneumatics &top_gate);

  void runTask() override;

  const State &getState();
  void setState(State state);

  void emergencyStop();
  void goToIdle();
  void goToStoring();
  void goToTOP();
  void goToMIDDLE();
  void goToOutaking();

  void enableFiltering();
  void disableFiltering();

  std::optional<COLOR> getSensedRing();
  void setFilterColor();
  COLOR getFilterColor() const;
};
