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
// using AngDir = lemlib::AngularDirection;
void wiggle() {
  for (int i = 0; i < 2; i++) {
    bot.tank(-25, 25);
    pros::delay(300);
    bot.tank(25, -25);
    pros::delay(300);
  }
  bot.tank(0, 0);
}

void autons::skills() {

  const Pose blueBall = {-18, 18};
  const Pose farLeftMatchLoader = {MIN_X, 2 * TILE - 1, RED_STATION};
  const Pose farRightMatchLoader = {MAX_X, 2 * TILE - 1, BLUE_STATION};
  const Pose closeLeftMatchLoader = {MIN_X, -2 * TILE + 1, RED_STATION};
  const Pose closeRightMatchLoader = {MAX_X, -2 * TILE + 1, BLUE_STATION};
  const Pose farLongGoalLeft = {-TILE - DRIVE_LENGTH, 2 * TILE, RED_STATION};
  const Pose farLongGoalRight = {TILE, 2 * TILE, BLUE_STATION};
  const Pose closeLongGoal = {-TILE, -2 * TILE, RED_STATION};
  const Pose centerGoal = {-7, 7};

  const Pose startingPosition = {-2 * TILE - 1.5, TILE - DRIVE_LENGTH + 8.5, 0};
  bot.setPose(startingPosition);

  // 1ST MATCHLOAD
  bot.descore.extend();

  bot.moveToPoint({-2 * TILE, 2 * TILE + 1}, 1500,
                  {.maxSpeed = 80, .minSpeed = 35, .earlyExitRange = 13});
  bot.waitUntilDone();
  pros::delay(40);

  bot.turnToPoint(farLeftMatchLoader, 1500, {.maxSpeed = 60, .minSpeed = 25});
  bot.waitUntilDone();
  pros::delay(40);

  bot.matchLoader.extend();
  bot.intake.goToStoring();

  bot.moveToPoint(farLeftMatchLoader, 2000, {.maxSpeed = 45, .minSpeed = 10});
  bot.waitUntilDone();

  bot.tank(10, 10);
  pros::delay(1300);
  bot.tank(0, 0);

  // BACK OUT + TRANSITION
  bot.moveToPose(
      {-1.25 * TILE, 2.5 * TILE + 1, RED_STATION}, 1500,
      {.forwards = false, .maxSpeed = 70, .minSpeed = 35, .earlyExitRange = 3});
  bot.waitUntilDone();

  bot.matchLoader.retract();

  bot.moveToPoint(
      {TILE + 12, bot.getPose().y}, 5000,
      {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 8});
  bot.waitUntilDone();

  bot.swingToHeading(BLUE_STATION, lemlib::DriveSide::LEFT, 2000,
                     {.direction = AngularDirection::CW_CLOCKWISE,
                      .maxSpeed = 55,
                      .minSpeed = 20});
  bot.waitUntilDone();

  // SCORE FAR LONG GOAL
  bot.moveToPoint(farLongGoalRight, 1700,
                  {.forwards = false, .maxSpeed = 55, .minSpeed = 20});
  bot.waitUntilDone();

  bot.intake.goToTOP();
  bot.tank(-20, -20);
  pros::delay(2000);
  bot.tank(0, 0);

  Pose temp = bot.getPose();
  bot.setPose({30, 47, temp.theta});
  pros::delay(100);
  bot.intake.goToIdle();

  // 2ND MATCHLOAD (RIGHT)
  bot.intake.goToStoring();
  bot.matchLoader.extend();

  bot.moveToPoint(farRightMatchLoader, 2000, {.maxSpeed = 45, .minSpeed = 10});
  bot.waitUntilDone();

  bot.tank(10, 10);
  pros::delay(1300);
  bot.tank(0, 0);

  bot.moveToPoint(farLongGoalRight, 1700,
                  {.forwards = false, .maxSpeed = 55, .minSpeed = 20});
  bot.waitUntilDone();

  bot.intake.goToTOP();
  bot.tank(-20, -20);
  pros::delay(2000);
  bot.tank(0, 0);

  // FIELD MIRROR RESET FOR RIGHT SIDE
  temp = bot.getPose();
  bot.setPose({30, 47, temp.theta});
  pros::delay(100);

  // TRANSITION
  bot.moveToPoint({1.5 * TILE, bot.getPose().y}, 1500,
                  {.maxSpeed = 70, .minSpeed = 40, .earlyExitRange = 5});
  bot.waitUntilDone();
  bot.intake.goToIdle();

  bot.swingToHeading(180, lemlib::DriveSide::RIGHT, 1200,
                     {.maxSpeed = 55, .minSpeed = 25});
  bot.waitUntilDone();

  bot.moveToPoint({bot.getPose().x, -2 * TILE + 1}, 5000,
                  {.maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 10});
  bot.waitUntilDone();

  // 3RD MATCHLOAD (RIGHT CLOSE)
  bot.turnToPoint(closeRightMatchLoader, 1000,
                  {.maxSpeed = 50, .minSpeed = 20});
  bot.waitUntilDone();

  bot.intake.goToStoring();
  bot.moveToPose(closeRightMatchLoader, 2000, {.maxSpeed = 45, .minSpeed = 15});
  bot.waitUntilDone();

  bot.tank(10, 10);
  pros::delay(1300);
  bot.tank(0, 0);

  // OUT → CLOSE LONG GOAL
  bot.moveToPose({1.25 * TILE, -2.5 * TILE - 1, BLUE_STATION}, 1500,
                 {.forwards = false, .minSpeed = 35, .earlyExitRange = 3});
  bot.waitUntilDone();

  bot.matchLoader.retract();

  bot.moveToPoint(
      {-TILE - 12, bot.getPose().y}, 5000,
      {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 8});
  bot.waitUntilDone();

  bot.swingToHeading(RED_STATION, lemlib::DriveSide::LEFT, 2000,
                     {.direction = AngularDirection::CW_CLOCKWISE,
                      .maxSpeed = 55,
                      .minSpeed = 20});
  bot.waitUntilDone();

  bot.moveToPoint(closeLongGoal, 1700, {.forwards = false, .maxSpeed = 55});
  bot.waitUntilDone();

  bot.intake.goToTOP();
  bot.tank(-20, -20);
  pros::delay(2000);
  bot.tank(0, 0);

  temp = bot.getPose();
  bot.setPose({-30, -47, temp.theta});
  pros::delay(100);

  // 4TH MATCHLOAD (LEFT CLOSE)
  bot.intake.goToStoring();
  bot.matchLoader.extend();

  bot.moveToPoint(closeLeftMatchLoader, 2000, {.maxSpeed = 45, .minSpeed = 10});
  bot.waitUntilDone();

  bot.tank(10, 10);
  pros::delay(1300);
  bot.tank(0, 0);

  bot.moveToPoint(closeLongGoal, 1700,
                  {.forwards = false, .maxSpeed = 55, .minSpeed = 20});
  bot.waitUntilDone();

  bot.intake.goToTOP();
  bot.tank(-20, -20);
  pros::delay(2000);
  bot.tank(0, 0);

  // FINAL RESET + PARK
  temp = bot.getPose();
  bot.setPose({-30, -47, temp.theta});
  pros::delay(100);

  bot.matchLoader.retract();

  bot.moveToPoint({-1.75 * TILE, bot.getPose().y}, 1000);
  bot.waitUntilDone();
  bot.moveToPose({-1.8 * TILE, 0, 0}, 4000, {.maxSpeed = 65});
  bot.waitUntilDone();
  bot.turnToHeading(RED_STATION, 1500);
  bot.waitUntilDone();

  bot.park.extend();
  bot.tank(70, 70);
  pros::delay(2400);
  wiggle();
  /*

  bot.moveToPose(
      {-2.8 * TILE, -TILE + 1, 0}, 2000,
      {.lead = 0.35, .maxSpeed = 90, .minSpeed = 60, .earlyExitRange = 7});
  bot.waitUntilDone();
  bot.matchLoader.extend();
  bot.clamp.extend();
  bot.intake.goToStoring();
  // bot.park.extend();
  // 1650
  /*
  bot.tank(80, 80);
  // waitUntil([] { return bot.front.get_distance() < 1670; });
  pros::delay(1000);
  bot.matchLoader.retract();
  pros::delay(1000);
  bot.tank(0, 0);

  /*
  // drive into park zone and intake the 6 balls
  bot.intake.goToStoring();
  bot.tank(50, 50);
  waitUntil([] { return bot.front.get_distance() < 5; });
  bot.tank(0, 0);
  pros::delay(200);

  // back out and reset odom
  bot.tank(50, 50);
  waitUntil([] { return bot.front.get_distance() > 30; });
  bot.tank(0, 0);
  pros::delay(200);
  bot.tank(20, 20);
  pros::delay(500);

  // reset here bot.setPose(lemlib::Pose pose);

  // intake the one blue ball and score on the middle goal
  bot.moveToPose({-14, 14, 45}, 1500, {.maxSpeed = 80});
  bot.waitUntilDone();
  bot.turnToPoint(blueBall, 1000);
  bot.waitUntilDone();
  bot.moveToPoint(blueBall, 1000, {.maxSpeed = 35});
  bot.waitUntilDone();
  bot.moveToPoint(centerGoal, 1000, {.forwards = false, .maxSpeed = 50});
  bot.waitUntilDone();
  bot.tank(-15, -15);
  bot.intake.goToMIDDLE();
  pros::delay(3000);
  bot.tank(0, 0);
  // odom reset here? bot.setPose();
  pros::delay(1000);
  bot.intake.goToStoring();

  // long goal score the remaining blocks & matchloader
  bot.moveToPose({-2 * TILE, 2 * TILE}, 1500);
  bot.waitUntilDone();
  bot.turnToPoint(farLongGoalLeft, 1500, {.forwards = false});
  bot.waitUntilDone();
  bot.moveToPoint(farLongGoalLeft, 1500, {.forwards = false});
  bot.waitUntilDone();
  bot.intake.goToTOP();
  bot.tank(-15, -15);
  pros::delay(200);
  bot.tank(0, 0);
  //Odom reset here bot.setPose
  pros::delay(200);
  bot.intake.goToStoring();

  bot.matchLoader.extend();
  bot.moveToPoint(farLeftMatchLoader, 1500, {.maxSpeed = 65});
  bot.waitUntilDone();
  bot.tank(15, 15);
  pros::delay(1200);
  bot.tank(0, 0);

  bot.moveToPose({-TILE, 2.5 * TILE}, BLUE_STATION);


*/
}