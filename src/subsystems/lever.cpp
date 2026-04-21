#include "subsystems/lever.h"
#include "robot.h"
#include <optional>

#include "log.h"

/** Max scoring voltage for the lever motor (in millivolts) */
constexpr int SCORE_MAX_VOLTAGE = 12000;
/** Slow scoring voltage for the lever motor (in millivolts) */
constexpr int SCORE_SLOW_VOLTAGE = SCORE_MAX_VOLTAGE * 0.5;
/** Max speed to reset the lever motor (in RPM) */
constexpr int RESET_MAX_SPEED = 100;
// TODO: Tune exit sensor threshold
/** Proximity threshold for the exit sensor to detect a block. */
constexpr int EXIT_SENSOR_THRESHOLD = 50;
// TODO: Tune angle, and maybe vary it based on starting position and lift state
/** Degrees to travel for the block to exit the lever once it is sensed, when
 * going at max speed. */
constexpr int SCORE_ONE_ANGLE_MAX = 20;
/** Degrees to travel for the block to exit the lever once it is sensed, when
 * going at slow speed. */
constexpr int SCORE_ONE_ANGLE_SLOW = 30;
/** Max time to wait for score one motion to complete (in milliseconds).
 * Timer begins when state is entered. */
constexpr int SCORE_ONE_TIMEOUT = 1000;
/** Threshold for determining if the lever is fully scored (in degrees). */
constexpr float FULLY_SCORED_THRESHOLD = 130;
/** Acceptable error for the lever position when in the intake ready state. */
constexpr float ACCEPTABLE_POSITION_ERROR = 5;
Lever::Lever(pros::MotorGroup &motors, pros::adi::Pneumatics gate,
             pros::Optical exitSensor, const PneumaticGroup &lift)
    : m_state(State::INTAKE_READY), m_scoreOneSensedPosition(std::nullopt),
      m_scoreOneStartTime(std::nullopt), m_motors(motors), m_gate(gate),
      m_exitSensor(exitSensor), m_lift(lift) {
  m_motors.tare_position_all();
  m_motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  m_exitSensor.set_led_pwm(100);
  m_exitSensor.set_integration_time(20);
}

void Lever::runTask() {
  // Wait until imu is done calibrating before moving lever as moving the lever
  // or the gate might mess up the imu calibration
  if (bot.m_config.sensors.imu.is_calibrating())
    return;
  switch (m_state) {
  case State::COMPRESS:
    // TODO: Implement Compress state. For now, just hold ready position.
  case State::INTAKE_READY:
    if (std::abs(m_motors.get_position() - 0) >= ACCEPTABLE_POSITION_ERROR)
      m_motors.move_absolute(0., 100);
    else
      m_motors.move_voltage(0);
    m_gate.retract();

    // Reset scoring state variables
    m_scoreOneSensedPosition = std::nullopt;
    m_scoreOneStartTime = std::nullopt;
    m_scoreOneSensedPosition = std::nullopt;
    break;

  case State::SCORE_ONE:
  case State::SCORE_ALL:
    switch (m_scoringSpeed.value_or(ScoringSpeed::MAX)) {
    case ScoringSpeed::MAX:
      m_motors.move_voltage(SCORE_MAX_VOLTAGE);
      break;
    case ScoringSpeed::SLOW:
      m_motors.move_voltage(SCORE_SLOW_VOLTAGE);
      break;
    }

    // Start timer even if we aren't in the SCORE_ONE state yet
    if (!m_scoreOneStartTime.has_value()) {
      m_scoreOneStartTime = pros::millis();
    }
    if (pros::millis() - m_scoreOneStartTime.value() > SCORE_ONE_TIMEOUT &&
        m_state == State::SCORE_ONE) {
      log("[{}] Score one timeout reached, resetting lever", pros::millis());
      // If jammed, reset to intake ready position.
      this->reset();
      runTask();
      return;
    }
    if (isFullyScored() && m_state == State::SCORE_ONE) {
      log("[{}] Fully scored, resetting lever", pros::millis());
      this->reset();
    }
    if (isBlockExiting()) {
      // Start distance thing even if we aren't in the SCORE_ONE state yet
      if (!m_scoreOneSensedPosition.has_value()) {
        m_scoreOneSensedPosition = m_motors.get_position();
        log("[{}] Block sensed, recording position {:.2f} deg", pros::millis(),
            m_scoreOneSensedPosition.value());
      }
      const float SCORE_ONE_ANGLE =
          (m_scoringSpeed.value_or(ScoringSpeed::MAX) == ScoringSpeed::MAX)
              ? SCORE_ONE_ANGLE_MAX
              : SCORE_ONE_ANGLE_SLOW;
      if (m_motors.get_position() - m_scoreOneSensedPosition.value() >
              SCORE_ONE_ANGLE &&
          m_state == State::SCORE_ONE) {
        log("[{}] Score distance reached, resetting lever", pros::millis());
        this->reset();
        runTask();
        return;
      }
    }

    // Only extend gate if scoring on middle goal.
    if (m_lift.isExtended()) {
      // Expand the gate when fully scored to prevent pulling the last block
      // when exiting
      if (isFullyScored())
        m_gate.extend();
      else
        m_gate.retract();
    } else
      m_gate.extend();
    break;
  }
}

bool Lever::isBlockExiting() { return m_exitSensor.get_proximity() > 50; }

Lever::State Lever::getState() const { return m_state; }

std::optional<Lever::ScoringSpeed> Lever::getScoringSpeed() const {
  return m_scoringSpeed;
}

bool Lever::isScoring() const {
  return m_state == State::SCORE_ONE || m_state == State::SCORE_ALL;
}

bool Lever::isFullyScored() const {
  return m_motors.get_position() >= FULLY_SCORED_THRESHOLD;
}

float Lever::blocksRemaining() const {
  const float angle = m_motors.get_position();
  // TODO: Tune this
  const float blockSpacing = FULLY_SCORED_THRESHOLD / 6;
  return std::max(0.f, (FULLY_SCORED_THRESHOLD - angle) / blockSpacing);
}

void Lever::reset() { m_state = State::INTAKE_READY; }

void Lever::scoreOne() {
  m_state = State::SCORE_ONE;
  m_scoringSpeed = ScoringSpeed::MAX;
}

void Lever::scoreAll() {
  m_state = State::SCORE_ALL;
  m_scoringSpeed = ScoringSpeed::MAX;
}

void Lever::scoreOneSlow() {
  m_state = State::SCORE_ONE;
  m_scoringSpeed = ScoringSpeed::SLOW;
}

void Lever::scoreAllSlow() {
  m_state = State::SCORE_ALL;
  m_scoringSpeed = ScoringSpeed::SLOW;
}

void Lever::compress() { m_state = State::COMPRESS; }