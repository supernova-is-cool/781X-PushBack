#include "auton/autons.h"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/pose.hpp"
#include "pros/motors.h"
#include "robot.h"
#include <cstdio>

#include "auton/autons.h"
#include "auton/util.h"
#include "color.h"
#include "dimensions.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "subsystems/intake.h"

using namespace dimensions::all;
using namespace DIR;
using namespace auton::util;

using lemlib::Pose;
using AngDir = lemlib::AngularDirection;

// Programmed from right blue perspective
void autons::leftMiddle() {
  // --- Starting Pose ---
  const Pose startingPosition = {2 * TILE + DRIVE_LENGTH / 2 + .5,
                                 -TILE + DRIVE_WIDTH / 2 - .5, RED_STATION};
  bot.setPose(startingPosition);

  bot.descore.extend();
  // --- Go to Balls ---
  const Pose leftCenterBalls = {
      1 * TILE,
      -1 * TILE,
  };
  bot.intake.goToStoring();
  bot.moveToPoint(leftCenterBalls, 1200, {.maxSpeed = 90});

  // Once within 6in of target, capture the balls with ML mech
  waitUntilDistToPose(leftCenterBalls, 14);
  bot.matchLoader.extend();
  bot.waitUntilDone();

  // --- Middle Goal Setup ---
  const Pose middleGoal = Pose::fromPolar(14, 135) + Pose{0, 0, 0};

  // Turn first for cleaner pathing
  bot.turnToPoint(middleGoal, 1200, {.forwards = false, .earlyExitRange = 15});
  bot.waitUntilDone();

  // Now drive backwards cleanly
  bot.moveToPoint(middleGoal, 1500, {.forwards = false, .maxSpeed = 60});
  bot.waitUntilDone();
  bot.turnToPoint({0, 0}, 500, {.forwards = false});
  bot.waitUntilDone();

  // Score + reset intake
  bot.tank(-10, -10);
  bot.intake.goToMIDDLE();
  // Intake quickly
  bot.intake.setSpinState(Intake::SpinState::INTAKING);
  pros::delay(500);

  // Hold onto any remaining balls
  bot.intake.goToOuttaking();
  pros::delay(200);
  bot.intake.goToStoring();
  // bot.tank(0, 0);

  const Pose matchLoader = {MAX_X - DRIVE_LENGTH / 2 - 2, -2 * TILE + 1,
                            BLUE_STATION};
  // Align with left matchloader
  bot.moveToY(matchLoader.y, 2000);

  bot.turnToHeading(BLUE_STATION, 1000);
  bot.waitUntilDone();
  bot.intake.goToStoring();

  // Go into match loader to pick up balls
  bot.moveToPoint(matchLoader, 2000, {.maxSpeed = 60});
  // Wait until bot accelerates
  pros::delay(200);
  // Wait until stopped because we hit match loader
  waitUntil(
      [] {
        const auto speed = lemlib::getSpeed(false);
        return !bot.isInMotion() || (speed.distance({0, 0}) < .015);
      },
      50, INT_MAX, true);
  bot.cancelMotion();
  // Give a little time to fully grab the balls before exiting
  bot.tank(15, 15);
  pros::delay(000);
  bot.tank(0, 0);

  const Pose longGoal = {TILE + DRIVE_LENGTH / 2, matchLoader.y, RED_STATION};

  bot.moveToPoint(longGoal, 1500,
                  {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
  // Score early
  waitUntilDistToPose(longGoal, 9, 0, true);
  bot.intake.goToTOP();
  bot.waitUntilDone();

  // Smooth scoring sequence
  bot.tank(-10, -10);
  bot.intake.goToTOP();
  pros::delay(1700);
  bot.tank(0, 0);
  bot.intake.goToIdle();

  // Don't extend descore, since retracted descore is at the correct height now
  bot.descore.retract();
  // Don't cross auton line when descoring
  bot.matchLoader.retract();

  const Pose scoringPose = longGoal;

  const float distanceFromGoalToDescore = 11.5;
  // Exit long goal and align with long goal
  const Pose descoreAlignTarget =
      (scoringPose + Pose{0, distanceFromGoalToDescore})
          .withX(TILE - DRIVE_LENGTH + 2);
  bot.swingToPoint(descoreAlignTarget, DriveSide::RIGHT, 500);
  bot.moveToPoint(descoreAlignTarget.withX(1.7 * TILE), 1000);
  bot.waitUntilDone();

  // Don't extend descore, since retracted descore is at the correct height now
  bot.descore.retract();

  /** If on left side, should face towards center. */
  const float descoreHeading = RED_STATION;
  bot.waitUntilDone();
  bot.turnToHeading(descoreHeading, 1000);
  bot.waitUntilDone();

  // When descoring, do not coast. This is especially important if we time out
  // while descoring, as we are at risk of crossing.
  bot.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

  const Pose pushBlocksTarget{DRIVE_LENGTH / 2 + 4, bot.getPose().y };
  bot.moveToX(pushBlocksTarget.x, 2500,
              {.maxSpeed = 67, .targetHeading = descoreHeading});
  bot.waitUntilDone();

  // Repeatedly run moveToPose to hold position against pushes
  // (When auton stage ends, this will also end)
  const Pose holdPose = [&] {
    const Pose currentPose = bot.getPose();
    return currentPose
        .withTheta(descoreHeading)
        // Do not go further than target and risk crossing auton line
        .withX(std::max(currentPose.x, pushBlocksTarget.x));
  }();
  while (true) {
    bot.moveToPose(holdPose, 1000, {}, false);
  }
}