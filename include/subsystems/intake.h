#pragma once
#include "color.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include "subsystems.h"
#include <optional>

class Intake : public subsystem {

public:
  enum State { IDLE, STORING, SCORING, OUTAKE, EMERGENCY_STOP };

  class Sensor {
  private:
    pros::Optical &m_optical;

  public:
    explicit Sensor(pros::Optical &optical);
    std::optional<COLOR> getBlock() const;
  };

private:
  State m_state;
  pros::Motor &m_top;
  pros::Motor &m_bottom;
  pros::Distance &m_distance;
  pros::adi::Pneumatics &m_filter;
  Sensor &m_sensor;
  bool isOpen = false;
  COLOR m_targetColor = COLOR::RED;

public:
  explicit Intake(pros::Motor &top, pros::Motor &bottom,
                  pros::Distance &distance, pros::adi::Pneumatics &filter,
                  Sensor &sensor);

  void runTask() override;

  const State &getState();
  void setState(State state);

  void emergencyStop();
  void goToIdle();
  void goToStoring();
  void goToScoring();
  void goToOutaking();

  std::optional<COLOR> getSensedRing();
  void setFilterColor();
  COLOR getFilterColor() const;
};
