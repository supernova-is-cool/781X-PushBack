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
  const Pose startingPosition{-2 * TILE - DRIVE_LENGTH + 2, 0.75 * TILE - 4,
                              BLUE_STATION};
  bot.setPose(startingPosition);

  // === FAR STACK COLLECTION ===
  const Pose farStack{-1 * TILE, 1 * TILE};

  bot.moveToPoint({-1.7 * TILE, 0.75 * TILE - 4}, 900,
                  {.maxSpeed = 110, .minSpeed = 90, .earlyExitRange = 2});
  bot.swingToPoint(farStack, lemlib::DriveSide::LEFT, 1000);

  bot.intake.goToStoring();
  bot.waitUntilDone();

  bot.moveToPoint(farStack, 1000, {.maxSpeed = 48});
  pros::delay(500);

  // center goal score
  const Pose centerGoal{-0.25 * TILE, 0.25 * TILE, 135};

  bot.turnToHeading(140, 1000);
  bot.waitUntilDone();
  bot.moveToPose(centerGoal, 1000);
  bot.waitUntilDone();
  bot.intake.goToScoring();

  pros::delay(1000);
  bot.moveToPoint({-TILE, TILE}, 750, {.forwards = false, .minSpeed = 90});
  bot.moveToPoint({-1.5*TILE, TILE}, 1000, {.forwards = false});

    // match loader closest to referee
  const Pose farMatchLoader{-(3 * TILE), 2 * TILE, RED_STATION};

  // moves bot towards the match loader in a curved motion and clears the match
  // loader
  // Move to first point smoothly
  bot.moveToPoint({-1.9 * TILE, 1.5 * TILE}, 1000,
                  {.maxSpeed = 80, .minSpeed = 30});
  bot.waitUntilDone();
  bot.littleWill.extend();

  bot.swingToHeading(RED_STATION, lemlib::DriveSide::LEFT, 1200,
                     {.maxSpeed = 70, .minSpeed = 40});
  bot.waitUntilDone();
  bot.moveToPoint(farMatchLoader, 1500,
                  {.maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 3});
  bot.waitUntilDone();
  bot.intake.goToStoring();
  bot.tank(-60, -60);
  pros::delay(150);
  bot.tank(0, 0);
  pros::delay(100);
  bot.tank(60, 60);
  pros::delay(250);
  bot.tank(0, 0);
  pros::delay(500);

  // bot.intake.goToIdle();

  // long goal closest to referee
  // Far long goal position (closest to referee) offset slightly rn
  const Pose farLongGoal{-TILE, 2 * TILE + 3, BLUE_STATION};

  bot.moveToPoint({-2 * TILE, 2 * TILE}, 1200,
                  {.forwards = false, .maxSpeed = 75, .minSpeed = 40});
  bot.waitUntil(10); // start next action slightly early
  bot.littleWill.retract();
  bot.waitUntilDone();
  pros::delay(100);
  bot.turnToPoint(farLongGoal, 1000, {.maxSpeed = 70, .minSpeed = 30});
  bot.waitUntilDone();
  bot.leftElevator.extend();
  bot.rightElevator.extend();
  pros::delay(150);
  bot.moveToPose(farLongGoal, 1200, {.maxSpeed = 70, .minSpeed = 30});
  bot.waitUntilDone();
  bot.tank(10, 10);
  bot.intake.goToScoring();
  pros::delay(2000);
  // bot.intake.goToIdle();
  pros::delay(200);
  bot.tank(0, 0);

  bot.intake.goToIdle();
  /*

  //undergoal shenanigans

  pros::delay(500);
  bot.intake.goToIdle();
  bot.moveToPoint({0.75 * TILE, 0.75 * TILE}, 750, {.forwards = false});
  bot.waitUntilDone();
  bot.moveToPose({-DRIVE_WIDTH - 1, 1.25 * TILE, 0}, 900, {.maxSpeed = 110, .minSpeed = 90});
  bot.moveToPoint({-DRIVE_WIDTH - 1, 1.75 * TILE}, 1000);
  bot.waitUntilDone();
  bot.littleWill.extend();
  pros::delay(100);
  bot.intake.goToStoring();
  bot.moveToPoint({-DRIVE_WIDTH - 1, 1.5 * TILE}, 750, {.forwards =false, .minSpeed = 90});
  bot.swingToHeading(BLUE_STATION, lemlib::DriveSide::LEFT, 500, {.minSpeed = 90});
  bot.moveToPoint({-1 * TILE, 1 * TILE}, 1000, {.forwards = false});
  bot.waitUntilDone();
  bot.turnToHeading(RED_STATION, 1000);

  //intake the undergoal shenanigans
  bot.moveToPoint({-1.5 * TILE, 1 * TILE}, 1000, {.maxSpeed = 30});
  bot.waitUntilDone();
  bot.littleWill.retract();



  

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
  */
}