#include "subsystems/intake.h"

#define POWER 127

void Intake::runTask() {
    switch (m_state) {
    case State::EMERGENCY_STOP: {
        m_top.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        m_top.brake();
        m_bottom.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        m_bottom.brake();
        return; 
    }
    case State::IDLE: {
        m_bottom.brake();
        m_top.brake();
        break;
    }
    case State::OUTAKE: {
        m_top.move(-POWER);
        m_bottom.move(-POWER);
        break;
    }
    case State::SCORING: {
        m_top.move(POWER);
        m_bottom.move(POWER);
        break;
    }
    case State::STORING: {
        m_top.move(-POWER * 0.25);
        m_bottom.move(POWER);
        break;
    }
    default:
        break;
    }

    m_top.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    m_bottom.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}


Intake::Intake(pros::Motor &top, pros::Motor &bottom)
    : m_state(State::IDLE), m_top(top), m_bottom(bottom) {}

const Intake::State &Intake::getState() { return m_state; }
void Intake::setState(State state) { m_state = state; }

void Intake::emergencyStop() { setState(State::EMERGENCY_STOP); }
void Intake::goToIdle() { setState(State::IDLE); }
void Intake::goToOutaking() { setState(State::OUTAKE); }
void Intake::goToScoring() { setState(State::SCORING); }
void Intake::goToStoring() { setState(State::STORING); }
