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

void moreBalls() {


  // Starting pose (adjust heading based on alliance color)
  const Pose startingPosition{0, 0, 0};
  bot.setPose(startingPosition);

  // === FAR STACK COLLECTION ===
  const Pose farStack{-1 * TILE, 1 * TILE};

  // Turn toward far stack and start intake
  bot.swingToPoint(farStack, lemlib::DriveSide::RIGHT, 1000,
                   {.maxSpeed = 70, .minSpeed = 40});
  bot.waitUntilDone();

  // Move to collect far stack
  bot.intake.goToStoring();
  bot.moveToPoint(farStack, 1300,
                  {.maxSpeed = 60, .minSpeed = 25, .earlyExitRange = 3});
  bot.waitUntilDone();
  pros::delay(150);

  // === SCORE IN TOP CENTER GOAL ===
  const Pose topCenterGoal{-0.5 * TILE, 0.5 * TILE, 135};
  bot.moveToPose(topCenterGoal, 1300,
                 {.maxSpeed = 70, .minSpeed = 30, .earlyExitRange = 3});
  bot.waitUntilDone();

  // Score collected balls
  bot.intake.goToScoring();
  pros::delay(1200);
  bot.intake.goToIdle();
  pros::delay(200);

  // === MOVE TO MATCH LOADER / LONG GOAL AREA ===
  const Pose matchLoaderPos{-3 * TILE, 2 * TILE};
  const Pose underLongGoal{-5, 2 * TILE};

  // Move backwards toward match loader
  bot.moveToPoint(matchLoaderPos, 1500,
                  {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
  bot.waitUntilDone();

  // Pull blocks from underneath the long goal
  bot.littleWill.extend();
  bot.moveToPoint(underLongGoal, 1200, {.maxSpeed = 55, .minSpeed = 25});
  bot.waitUntilDone();
  bot.littleWill.retract();
  pros::delay(200);

  // === INTAKE BLOCKS FROM MATCH LOADER ===
  bot.intake.goToStoring();
  const Pose matchLoaderBlocks{-2.4 * TILE, 1.2 * TILE};
  bot.moveToPoint(matchLoaderBlocks, 1300,
                  {.maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 3});
  bot.waitUntilDone();
  pros::delay(150);

  // === SCORE IN LONG GOAL ===
  const Pose longGoal{-TILE - 5, 2 * TILE, BLUE_STATION};

  // Turn toward long goal
  bot.turnToPoint(longGoal, 1000, {.maxSpeed = 70, .minSpeed = 30});
  bot.waitUntilDone();

  // Extend elevators before scoring
  bot.leftElevator.extend();
  bot.rightElevator.extend();
  pros::delay(150);

  // Move into long goal and score
  bot.moveToPoint(longGoal, 1200, {.maxSpeed = 60, .minSpeed = 30});
  bot.waitUntilDone();
  bot.intake.goToScoring();
  pros::delay(1200);
  bot.intake.goToIdle();
  pros::delay(200);

  // === END AUTON ===
  bot.tank(0, 0);
  pros::delay(100);
}