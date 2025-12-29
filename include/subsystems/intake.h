#pragma once
#include "color.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include "subsystems.h"
#include <optional>

class Intake : public subsystem {

public:
  enum State { IDLE, STORING, SCORING, OUTAKE, SLOW_OUTAKE, SKILLS, EMERGENCY_STOP };


private:
  State m_state;
  pros::Motor &m_top;
  pros::Motor &m_bottom;
  pros::Distance &m_distance;
  bool isOpen = false;
  COLOR m_targetColor = COLOR::RED;
  bool enableFilter = false;

public:
  explicit Intake(pros::Motor &top, pros::Motor &bottom,
                  pros::Distance &distance);

  void runTask() override;

  const State &getState();
  void setState(State state);

  void emergencyStop();
  void goToIdle();
  void goToStoring();
  void goToScoring();
  void goToOutaking();
  void goToSlowOutake();
  void goToSkills();

  void enableFiltering();
  void disableFiltering();

  std::optional<COLOR> getSensedRing();
  void setFilterColor();
  COLOR getFilterColor() const;
};
