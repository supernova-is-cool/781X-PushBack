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

void doNothing() { printf("yo what's up\n"); }

void redAWP() {
  bot.setAlliance(ALLIANCE::RED);
  bot.intake.disableFiltering();

  // place holder cuz idk were is gonna start
  const Pose startingPose{(-2 * TILE) - DRIVE_WIDTH + 5, 1.5 + DRIVE_LENGTH, 0};

  bot.setPose(startingPose);

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

  // swings to cluster of balls
  const Pose farBalls = {-TILE, TILE};

  bot.moveToPoint({-2 * TILE, -2 * TILE}, 1200,
                  {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
  bot.waitUntilDone();
  pros::delay(100);
  bot.swingToPoint(farBalls, lemlib::DriveSide::RIGHT, 1000,
                   {.maxSpeed = 60, .minSpeed = 30});
  bot.waitUntilDone();
  bot.intake.goToStoring();
  pros::delay(100);
  bot.moveToPoint(farBalls, 1500,
                  {.maxSpeed = 60, .minSpeed = 25, .earlyExitRange = 2});
  bot.waitUntilDone();
  bot.intake.goToIdle();
  pros::delay(200);

  /*

  // goes to center goal to score
  const Pose centerGoal = {-0.6 * TILE, 0.6 * TILE, 135};

  bot.moveToPose({centerGoal}, 1000);
  bot.waitUntilDone();
  bot.intake.goToScoring();
  pros::delay(750);
  bot.intake.goToIdle();

  // backs out and intakes the other stack
  const Pose closeBalls = {-TILE, -TILE};

  bot.moveToPoint({-TILE, TILE}, 1500, {.forwards = false});
  bot.waitUntilDone();
  bot.turnToPoint(closeBalls, 1000);
  bot.waitUntilDone();
  bot.intake.goToStoring();
  bot.moveToPoint({-TILE, 0}, 1000, {.minSpeed = 100});
  bot.waitUntilDone();
  bot.moveToPoint({closeBalls}, 1000, {.maxSpeed = 80});

  // swerve not and score on long goal
  const Pose closeLongGoal = {-TILE - DRIVE_LENGTH, -2 * TILE, BLUE_STATION};

  bot.moveToPoint({-2 * TILE, -2 * TILE}, 1000);
  bot.waitUntilDone();
  bot.turnToHeading(BLUE_STATION, 1000);
  bot.waitUntilDone();
  bot.moveToPoint(closeLongGoal, 1000);
  bot.waitUntilDone();
  bot.intake.goToScoring();
  */
}