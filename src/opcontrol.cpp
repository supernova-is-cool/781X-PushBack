#include "autonSelector/selector.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <string>
#include "main.h"
#include "robot.h"
#include "subsystems/intake.h"
#include <print>

namespace controller_mapping {
typedef pros::controller_analog_e_t axis_t;
typedef pros::controller_digital_e_t button_t;

constexpr axis_t LEFT_DRIVE = pros::E_CONTROLLER_ANALOG_LEFT_Y;
constexpr axis_t RIGHT_DRIVE = pros::E_CONTROLLER_ANALOG_RIGHT_Y;
constexpr button_t MIDDLE_GOAL = pros::E_CONTROLLER_DIGITAL_X;
constexpr button_t DESCORE = pros::E_CONTROLLER_DIGITAL_R2;
constexpr button_t OUTTAKE = pros::E_CONTROLLER_DIGITAL_L2;
constexpr button_t STORE = pros::E_CONTROLLER_DIGITAL_L1;
constexpr button_t HIGH_GOAL = pros::E_CONTROLLER_DIGITAL_R1;
constexpr button_t MATCH_LOADER = pros::E_CONTROLLER_DIGITAL_UP;
constexpr button_t PARK = pros::E_CONTROLLER_DIGITAL_DOWN;
constexpr button_t CLAMP = pros::E_CONTROLLER_DIGITAL_B;

constexpr button_t PREV_AUTON = pros::E_CONTROLLER_DIGITAL_LEFT;
constexpr button_t NEXT_AUTON = pros::E_CONTROLLER_DIGITAL_RIGHT;
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
  bot.resetControllerSettings();
  // bot.odomLift.retract();

  ts::selector *selector = ts::selector::get();
  while (true) {
    bot.tank(master.get_analog(map::LEFT_DRIVE),
             master.get_analog(map::RIGHT_DRIVE));

    if (master.get_digital_new_press(map::MATCH_LOADER)) {
      bot.matchLoader.toggle();
    }

    if (master.get_digital(map::HIGH_GOAL)) {
      bot.intake.goToTOP();
    } else if (master.get_digital(map::OUTTAKE)) {
      bot.intake.goToOuttaking();
    } else if (master.get_digital(map::STORE)) {
      bot.intake.goToStoring();
    } else if (master.get_digital(map::MIDDLE_GOAL)) {
      bot.intake.goToMIDDLE();
    } else {
      bot.intake.goToIdle();
    }

    if (master.get_digital_new_press(map::DESCORE)) {
      bot.descore.toggle();
    }

    if (master.get_digital_new_press(map::CLAMP)) {
      bot.clamp.toggle();
    }

    if (master.get_digital_new_press(map::PARK)) {
      bot.park.toggle();
    }

    // Use controller to select auton if we are not playing a match
    if (!pros::competition::is_connected()) {
      /** Desired change in index */
      int change = 0;
      // Left and right on the dpad cycle through autons
      if (master.get_digital_new_press(map::PREV_AUTON)) {
        change += -1;
      }
      if (master.get_digital_new_press(map::NEXT_AUTON)) {
        change += 1;
      }
      if (change != 0) {
        auto names = selector->get_auton_names();
        size_t len = names.size();
        auto it = std::find(names.begin(), names.end(),
                            selector->get_selected_auton_name());
        size_t old_i = it - names.begin();
        size_t new_i = ((old_i + change) % len + len) % len;
        selector->select_auton(names.at(new_i));
        println("old_i={},new_i={},selected={}", old_i, new_i,
                selector->get_selected_auton_name());
      }
    }

    pros::delay(5); // Run every 20ms (refresh rate of the controller)
  }
}