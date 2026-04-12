#include "subsystems/lever.h"

#include <optional>

/** Scoring voltage for the lever motor (in millivolts) */
constexpr int SCORE_VOLTAGE = 1200;
/** Max speed to reset the lever motor (in RPM) */
constexpr int RESET_MAX_SPEED = 100;
// TODO: Tune exit sensor threshold
/** Proximity threshold for the exit sensor to detect a block. */
constexpr int EXIT_SENSOR_THRESHOLD = 50;
// TODO: Tune angle, and maybe vary it based on starting position and lift state
/** Degrees to travel for the block to exit the lever once it is sensed. */
constexpr int SCORE_ONE_ANGLE = 10;
/** Max time to wait for score one motion to complete (in milliseconds).
 * Timer begins when state is entered. */
constexpr int SCORE_ONE_TIMEOUT = 1000;

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
  switch (m_state) {
  case State::COMPRESS:
    // TODO: Implement Compress state. For now, just hold ready position.
  case State::INTAKE_READY:
    m_motors.move_absolute(0., 100);
    m_gate.retract();
    m_scoreOneSensedPosition = std::nullopt;
    m_scoreOneStartTime = std::nullopt;
    break;

  case State::SCORE_ONE:
  case State::SCORE_ALL:
    m_motors.move_voltage(SCORE_VOLTAGE);
    m_gate.extend();

    // Start timer even if we aren't in the SCORE_ONE state yet
    if (!m_scoreOneStartTime.has_value()) {
      m_scoreOneStartTime = pros::millis();
    }
    if (pros::millis() - m_scoreOneStartTime.value() > SCORE_ONE_TIMEOUT &&
        m_state == State::SCORE_ONE) {
      // If jammed, reset to intake ready position.
      this->reset();
    }

    if (isBlockExiting()) {
      // Start distance thing even if we aren't in the SCORE_ONE state yet
      if (!m_scoreOneSensedPosition.has_value()) {
        m_scoreOneSensedPosition = m_motors.get_position();
      }
      if (m_motors.get_position() - m_scoreOneSensedPosition.value() >
              SCORE_ONE_ANGLE &&
          m_state == State::SCORE_ONE) {
        this->reset();
      }
    }
    break;
  }
}

bool Lever::isBlockExiting() { return m_exitSensor.get_proximity() > 50; }

Lever::State Lever::getState() const { return m_state; }

bool Lever::isScoring() const {
  return m_state == State::SCORE_ONE || m_state == State::SCORE_ALL;
}

void Lever::reset() { m_state = State::INTAKE_READY; }

void Lever::scoreOne() { m_state = State::SCORE_ONE; }

void Lever::scoreAll() { m_state = State::SCORE_ALL; }

void Lever::compress() { m_state = State::COMPRESS; }