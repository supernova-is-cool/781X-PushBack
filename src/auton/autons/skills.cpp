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

void autons::skills() {

  const Pose blueBall = {-18, 18};
  const Pose farLeftMatchLoader = {MIN_X, 2 * TILE, RED_STATION};
  const Pose farRightMatchLoader = {MAX_X, 2 * TILE, BLUE_STATION};
  const Pose closeLeftMatchLoader = {MIN_X, -2 * TILE, RED_STATION};
  const Pose closeRightMatchLoader = {MAX_X, -2 * TILE, BLUE_STATION};
  const Pose farLongGoalLeft = {-TILE - DRIVE_LENGTH, 2 * TILE, RED_STATION};
  const Pose farLongGoalRight = {TILE + DRIVE_LENGTH, 2 * TILE, RED_STATION};
  const Pose closeLongGoal = {-TILE - DRIVE_LENGTH, -2 * TILE, BLUE_STATION};
  const Pose centerGoal = {-7, 7};

  const Pose startingPosition = {0, 0, 0};
  bot.setPose(startingPosition);

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

}