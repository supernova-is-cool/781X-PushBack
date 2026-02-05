#include "auton/autons.h"
#include "lemlib/pose.hpp"
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

void autons::leftMiddle() {
  // --- Starting Pose ---
  const Pose startingPosition = {2 * TILE + DRIVE_LENGTH / 2, -0.7 * TILE + 1.5,
                                 270};
  bot.setPose(startingPosition);

  // --- Go to Balls ---
  const Pose theBalls = {
      1 * TILE,
      -1 * TILE,
  };

  bot.intake.goToStoring();
  bot.moveToPoint(theBalls, 1200, {.maxSpeed = 90});
  bot.waitUntil(20);
  bot.matchLoader.extend();
  bot.waitUntilDone();
  // pros::delay(750);

  // --- Middle Goal Setup ---
  const Pose middleGoal = Pose::fromPolar(18, 135) + Pose{0, 0, 0};

  // Turn first for cleaner pathing
  bot.turnToPoint(middleGoal, 1200, {.forwards = false});
  bot.waitUntilDone();

  // Now drive backwards cleanly
  bot.moveToPoint(middleGoal, 1500, {.forwards = false, .maxSpeed = 80});
  bot.waitUntilDone();
  bot.turnToPoint({0, 0}, 500, {.forwards = false});
  bot.waitUntilDone();

  // Score + reset intake
  bot.tank(-10, -10);
  bot.intake.goToTOP();
  pros::delay(100);
  bot.intake.goToOutaking();
  pros::delay(100);
  bot.tank(0, 0);
  bot.intake.goToTOP();
  pros::delay(1600);
  bot.intake.goToIdle();
  pros::delay(150);
  // bot.tank(0, 0);

  const Pose matchLoader = {MAX_X - DRIVE_LENGTH / 2 - 4, -2 * TILE - 1,
                            BLUE_STATION};

  bot.moveToPoint({2 * TILE + 2, -2 * TILE - 2}, 2500, {.maxSpeed = 90});
  bot.waitUntilDone();
  bot.turnToPoint(matchLoader, 1000);
  bot.waitUntilDone();
  bot.intake.goToStoring();
  bot.moveToPoint(matchLoader, 1100, {.maxSpeed = 45});
  bot.waitUntilDone();
  bot.tank(15, 15);
  pros::delay(400);
  bot.tank(0, 0);

  const Pose longGoal = {TILE + DRIVE_LENGTH / 2 - 4, -2 * TILE - 2,
                         RED_STATION};

  bot.moveToPoint(longGoal, 1500,
                  {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
  bot.waitUntilDone();

  // Smooth outaking sequence
  bot.tank(-10, -10);
  bot.intake.goToTOP();
  pros::delay(150);
  bot.intake.goToOutaking();
  pros::delay(150);
  bot.intake.goToTOP();
  pros::delay(1700);
  bot.tank(0, 0);
  bot.intake.goToIdle();
  pros::delay(100);

  // Don't extend descore, since retracted descore is at the correct height now
  bot.descore.retract();
  // Don't cross auton line when descoring
  bot.matchLoader.retract();

  // Exit long goal, and align descore mech
  bot.moveToPoint({1.7 * TILE, -36}, 1000);
  bot.waitUntilDone();

  // Rotate to face descore angle long goal
  bot.turnToHeading(RED_STATION, 1000);
  // Push blocks with descore
  bot.moveToPoint({DRIVE_LENGTH / 2 + 1, bot.getPose().y}, 2500,
                  {.maxSpeed = 67});
  bot.waitUntil(25);
  // Put descore down
  bot.descore.retract();
  bot.waitUntilDone();

  // Repeatedly run moveToPose to hold position against pushes
  // (When auton stage ends, this will also end)
  const Pose currentPose = bot.getPose().withTheta(RED_STATION);
  while (true) {
    bot.moveToPose(currentPose, 1000);
  }
}