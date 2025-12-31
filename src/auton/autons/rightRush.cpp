#include "auton/autons.h"
#include "lemlib/pose.hpp"
#include "pros/abstract_motor.hpp"
#include "robot.h"
#include <cstdio>
#include <print>

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


void autons::rightRush() {
  const Pose startingPosition = {-2 * TILE - DRIVE_LENGTH / 2 - .5,
                                 -0.7 * TILE + 2, 90};
  bot.setPose(startingPosition);

  const Pose theBalls = {
      -1 * TILE,
      -1 * TILE,
  };

  bot.intake.goToStoring();
  bot.moveToPoint(theBalls, 1200, {.minSpeed = 67, .earlyExitRange = 8});
  bot.waitUntil(20);
  bot.matchLoader.extend();
  bot.waitUntilDone();
  // pros::delay(750);
  bot.turnToPoint({-2 * TILE, -2 * TILE}, 1200,
                  {.maxSpeed = 95, .minSpeed = 80, .earlyExitRange = 17});
  bot.waitUntilDone();

  const Pose matchLoader = {MIN_X + DRIVE_LENGTH / 2 + 3.5, -2 * TILE - 1,
                            RED_STATION};
  const Pose alignmentPoint = matchLoader.withX(-2 * TILE + 1);

  bot.moveToPoint(alignmentPoint, 1000,
                  {.maxSpeed = 95, .minSpeed = 80, .earlyExitRange = 15});
  bot.moveToPoint(alignmentPoint, 1000, {.earlyExitRange = 2});

  bot.waitUntilDone();
  bot.turnToHeading(RED_STATION, 1000);
  bot.waitUntilDone();
  bot.moveToPoint(matchLoader.withY(bot.getPose().y), 1100,
                  {.maxSpeed = 72, .minSpeed = 16});
  bot.waitUntilDone();

  bot.tank(10, 10);
  pros::delay(850);
  bot.tank(0, 0);

  const Pose longGoal = {-TILE - DRIVE_LENGTH / 2 + 4, -2 * TILE - 2,
                         RED_STATION};

  bot.lift.extend();

  bot.moveToPoint(longGoal, 1500,
                  {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
  bot.waitUntilDone();

  // Smooth outaking sequence
  bot.tank(-10, -10);

  bot.intake.goToScoring();
  pros::delay(400);
  bot.intake.goToOutaking();
  pros::delay(200);
  bot.intake.goToScoring();
  pros::delay(2500);
  bot.intake.goToIdle();
  pros::delay(100);
  bot.tank(0, 0);

  const Pose scoringPose = longGoal;
  std::println("pose of scored: {}", bot.getPose());

  const Pose descoreTarget =
      (scoringPose + Pose{0, 10.5}).withX(-TILE + DRIVE_LENGTH - 2);

  bot.moveToPoint(descoreTarget.withX(-1.7 * TILE), 1000);
  bot.waitUntilDone();

  bot.descore.extend();

  // Do not cross auton line
  bot.matchLoader.retract();

  // bot.moveToPoint(descore, 2000, {.forwards = false});
  bot.waitUntilDone();
  bot.turnToHeading(RED_STATION, 1000);
  bot.waitUntilDone();
  bot.moveToPoint({-DRIVE_LENGTH / 2, bot.getPose().y}, 2500,
                  {.forwards = false, .maxSpeed = 67});
  bot.waitUntil(20);
  bot.descore.retract();
  bot.waitUntilDone();
  const Pose holdPose = bot.getPose().withTheta(RED_STATION);

  // Repeatedly run moveToPose to hold position against pushes
  // (When auton stage ends, this will also end)
  while (true) {
    bot.moveToPose(holdPose, 1000);
  }
}