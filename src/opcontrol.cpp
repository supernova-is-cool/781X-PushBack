#include "dimensions.h"
#include "lemlib/pose.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <optional>
#include <string>
// #include "auton/util.h"
#include "main.h"
#include "robot.h"
#include "subsystems/intake.h"

namespace controller_mapping {
typedef pros::controller_analog_e_t axis_t;
typedef pros::controller_digital_e_t button_t;

constexpr axis_t LEFT_DRIVE = pros::E_CONTROLLER_ANALOG_LEFT_Y;
constexpr axis_t RIGHT_DRIVE = pros::E_CONTROLLER_ANALOG_RIGHT_Y;
constexpr button_t MATCH_LOADER = pros::E_CONTROLLER_DIGITAL_X;
constexpr button_t ELEVATOR = pros::E_CONTROLLER_DIGITAL_R2;
constexpr button_t OUTAKE = pros::E_CONTROLLER_DIGITAL_L2;
constexpr button_t STORE = pros::E_CONTROLLER_DIGITAL_L1;
constexpr button_t SCORE = pros::E_CONTROLLER_DIGITAL_R1;

}; // namespace controller_mapping
namespace map = controller_mapping;

// TODO: deduplicate this with the one in intake.cpp
template <class... Ts> struct overloads : Ts... {
  using Ts::operator()...;
};
template <class... Ts> overloads(Ts...) -> overloads<Ts...>;

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (true) {
    bot.tank(master.get_analog(map::LEFT_DRIVE),
             master.get_analog(map::RIGHT_DRIVE));

    if (master.get_digital_new_press(map::ELEVATOR)) {
      bot.leftElevator.toggle();
      bot.rightElevator.toggle();
    }

    if (master.get_digital_new_press(map::MATCH_LOADER)) {
      bot.littleWill.toggle();
    }

    if (master.get_digital(map::SCORE)){
      bot.intake.goToScoring();
    } else if (master.get_digital(map::OUTAKE)){
      bot.intake.goToOutaking();
    } else if (master.get_digital(map::STORE)){
      bot.intake.goToStoring();
    } else {
      bot.intake.goToIdle();
    }

    pros::delay(5); // Run every 20ms (refresh rate of the controller)
  }
}