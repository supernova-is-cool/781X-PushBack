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

  // place holder cuz idk were is gonna start
  const Pose startingPose{(-2 * TILE) - 5.7, 7 + DRIVE_LENGTH + 15, 180};
  
  bot.setPose(startingPose);
  

  // match loader closest to referee
  const Pose farMatchLoader{-(3 * TILE - DRIVE_LENGTH - 8), 2 * TILE,
                            RED_STATION};

  // moves bot towards the match loader in a curved motion and clears the match
  // loader
  bot.moveToPose({-1.5 * TILE, 1.5 * TILE, REFEREE}, 200, {.minSpeed = 72});
  bot.waitUntilDone();
  bot.littleWill.extend();
  bot.moveToPose(farMatchLoader, 1000, {.maxSpeed = 100});
  bot.intake.goToStoring();
  pros::delay(1500);
  bot.intake.goToIdle();

  /*
  // long goal closest to referee
  const Pose farLongGoal{-TILE - DRIVE_LENGTH, 2 * TILE, BLUE_STATION};

  // moves to long goal
  bot.moveToPoint({-2 * TILE, -2 * TILE}, 1000, {.forwards = false});
  bot.waitUntilDone();
  bot.littleWill.retract();
  bot.turnToHeading(BLUE_STATION, 1000);
  bot.leftElevator.extend();
  bot.rightElevator.extend();
  bot.aligner.extend();
  bot.moveToPoint(farLongGoal, 1000);

  // scores balls
  bot.intake.goToScoring();
  pros::delay(1500);
  bot.intake.goToIdle();

  // swings to cluster of balls
  const Pose farBalls = {-TILE, TILE};

  bot.moveToPoint({-2 * TILE, -2 * TILE}, 1000, {.forwards = false});
  bot.waitUntilDone();
  bot.swingToPoint({farBalls}, lemlib::DriveSide::RIGHT, 1000);
  bot.waitUntilDone();

  // slowerly move forward to intake the balls
  bot.intake.goToScoring();
  bot.moveToPoint({farBalls}, 1500, {.maxSpeed = 90});
  bot.waitUntilDone();
  bot.intake.goToIdle();

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