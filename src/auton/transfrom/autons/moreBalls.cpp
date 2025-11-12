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

  // === STARTING POSE ===
  const Pose startingPosition{-2 * TILE - DRIVE_LENGTH + 2, 0.75 * TILE - 3,
                              BLUE_STATION};
  bot.setPose(startingPosition);
  

  // === FAR STACK COLLECTION ===
  const Pose farStack{-1 * TILE, 1 * TILE};

  bot.moveToPoint({-1.7 * TILE, 0.75 * TILE - 4}, 900,
                  {.maxSpeed = 110, .minSpeed = 90, .earlyExitRange = 2});
  bot.swingToPoint(farStack, lemlib::DriveSide::LEFT, 900,
                   {.maxSpeed = 90, .minSpeed = 60});
  bot.waitUntilDone();
  bot.intake.goToStoring();


  // Slowly move in to intake stack
  bot.moveToPoint(farStack, 900, {.maxSpeed = 30, .minSpeed = 20});
  pros::delay(500);

  // === CENTER GOAL SCORING ===
  const Pose centerGoal{-0.33 * TILE, 0.33 * TILE, 135};

  bot.turnToHeading(140, 900);
  bot.waitUntilDone();
  bot.moveToPose(centerGoal, 1100, {.maxSpeed = 70, .minSpeed = 40});
  bot.waitUntilDone();
  bot.intake.goToScoring();
  pros::delay(1700);

  // Reverse away from center goal
  bot.moveToPoint({-2 * TILE, 2 * TILE}, 1500,
                  {.forwards = false, .maxSpeed = 70, .minSpeed = 40});
  bot.waitUntilDone();
  pros::delay(100);

  // === MATCH LOADER INTAKE ===
  const Pose farMatchLoader{-(3 * TILE), 2 * TILE + 1, RED_STATION};

  // Move to match loader area and begin pulling cubes
  bot.littleWill.extend();
  bot.turnToPoint(farMatchLoader, 1200, {.maxSpeed = 70, .minSpeed = 40});
  bot.waitUntilDone();
  pros::delay(100);
  bot.moveToPoint(farMatchLoader, 1000, {.maxSpeed = 55, .minSpeed = 40});
  bot.intake.goToStoring();
  bot.waitUntilDone();
  // Jiggle to help cubes feed correctly
  bot.tank(-60, -60);
  pros::delay(100);
  bot.tank(0, 0);
  pros::delay(100);
  bot.tank(60, 60);
  pros::delay(200);
  bot.tank(0, 0);
  pros::delay(200);

  // === LONG GOAL SCORING ===
  const Pose farLongGoal{-TILE, 2 * TILE + 3, BLUE_STATION};

  bot.moveToPoint({-2 * TILE, 2 * TILE}, 1200,
                  {.forwards = false, .maxSpeed = 80, .minSpeed = 40});
  bot.waitUntilDone();

  bot.turnToPoint(farLongGoal, 900, {.maxSpeed = 70, .minSpeed = 40});
  bot.waitUntilDone();
  bot.littleWill.retract();


  // Extend elevators and align
  bot.leftElevator.extend();
  bot.rightElevator.extend();
  pros::delay(150);

  bot.moveToPose(farLongGoal, 1100, {.maxSpeed = 75, .minSpeed = 35});
  bot.waitUntilDone();

  // Final scoring
  bot.tank(20, 20);
  bot.intake.goToScoring();
  pros::delay(250);
  pros::delay(2000);
  bot.tank(0, 0);

  /*
  

    //undergoal shenanigans

    pros::delay(500);
    bot.intake.goToIdle();
    bot.moveToPoint({0.75 * TILE, 0.75 * TILE}, 750, {.forwards = false});
    bot.waitUntilDone();
    bot.moveToPose({-DRIVE_WIDTH - 1, 1.25 * TILE, 0}, 900, {.maxSpeed = 110,
    .minSpeed = 90}); bot.moveToPoint({-DRIVE_WIDTH - 1, 1.75 * TILE}, 1000);
    bot.waitUntilDone();
    bot.littleWill.extend();
    pros::delay(100);
    bot.intake.goToStoring();
    bot.moveToPoint({-DRIVE_WIDTH - 1, 1.5 * TILE}, 750, {.forwards =false,
    .minSpeed = 90}); bot.swingToHeading(BLUE_STATION, lemlib::DriveSide::LEFT,
    500, {.minSpeed = 90}); bot.moveToPoint({-1 * TILE, 1 * TILE}, 1000,
    {.forwards = false}); bot.waitUntilDone(); bot.turnToHeading(RED_STATION,
    1000);

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