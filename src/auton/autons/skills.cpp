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
    bot.tank(-40, 40);
    pros::delay(300);
    bot.tank(40, -40);
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

  const Pose middleGoal = {-9, 9, 310};
  const Pose inFrontPark = {-47.5, 0, RED_STATION};

  const Pose startingPosition = {-2 * TILE - 1.5, TILE - DRIVE_LENGTH + 8.5,
  0}; bot.setPose(startingPosition);

  //const Pose startingPosition = {0, 0, RED_STATION};
  //bot.setPose(startingPosition);

  /*

  // === PARK → CENTER LANE ===
  bot.park.extend();
  bot.intake.goToStoring();

  

  // Smooth motor bias wiggle (reduced + symmetric)
  bot.tank(30, 30);
  pros::delay(225);
  bot.tank(0, 0);
  pros::delay(650);
  bot.tank(30, 30);
  pros::delay(650);
  // bot.tank(20, 20); pros::delay(400);
  wiggle();
  bot.tank(0, 0);

  // Clean reverse (straight + damped)
  bot.tank(-55, -55);
  pros::delay(1200);
  bot.tank(0, 0);

  // Aim robot for straight drive
  bot.turnToHeading(RED_STATION, 700);
  bot.waitUntilDone();

  // Small controlled push forward to clear bumper
  bot.moveToPoint({-12, 0}, 1100, {.maxSpeed = 25, .minSpeed = 20});
  bot.waitUntilDone();
  
  

  // Retract parking mech + reset pose to known clean spot
  bot.park.retract();
  bot.setPose({-47.5, 0, RED_STATION});

  // 1) Smooth reverse into center lane
  bot.moveToPoint(
      {-TILE - 3, 0}, 1300,
      {.forwards = false, .maxSpeed = 55, .minSpeed = 40, .earlyExitRange = 6});

  // 2) Smooth 225° swing into goal approach vector
  bot.swingToHeading(
      225, lemlib::DriveSide::RIGHT, 900,
      {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,
       .maxSpeed = 55,
       .minSpeed = 40,
       .earlyExitRange = 8});
  // bot.waitUntilDone();

  // 3) Main arc into the middle goal entrance

  bot.moveToPoint({-15, 5}, 1500,
                  {.forwards = false, .maxSpeed = 55, .minSpeed = 40});
  bot.waitUntilDone();

  bot.swingToHeading(310, lemlib::DriveSide::LEFT, 1500,
                     {.maxSpeed = 50, .minSpeed = 40});
  bot.waitUntilDone();
  pros::delay(100);
  // pros::delay(100);

  // =====================
  // === INTAKE BLOCK #2 ===
  // =====================

  // The second block is deeper inside.
  // Drive forward slowly to intake it.
  Pose intakeDeep = {middleGoal.x - 6, middleGoal.y + 6}; // 7–10 in deeper

  bot.moveToPoint(intakeDeep, 1500, {.maxSpeed = 30, .minSpeed = 25});
  bot.waitUntilDone();

  // Intake the block
  pros::delay(250);

  // Back out slightly to clear the wall + realign
  bot.moveToPoint(middleGoal, 1200,
                  {.forwards = false, .maxSpeed = 35, .minSpeed = 30});
  bot.waitUntilDone();

  // =====================
  // === SCORE IN MIDDLE ===
  // =====================

  bot.intake.goToMIDDLE();
  bot.tank(-10, -10);
  pros::delay(3000);
  bot.tank(0, 0);

  bot.setPose({middleGoal.x, middleGoal.y, bot.getPose().theta});
  pros::delay(100);
  bot.intake.goToStoring();
  */

  // 1ST MATCHLOAD
  bot.descore.extend();

  bot.moveToPoint({-2 * TILE, 2 * TILE + 1}, 1500,
                  {.maxSpeed = 80, .minSpeed = 35, .earlyExitRange = 15});
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

  // bot.matchLoader.retract();

  bot.moveToPoint({TILE + 12, bot.getPose().y}, 5000,
                  {.forwards = false,
                   .maxSpeed = 80,
                   .minSpeed = 50,
                   .earlyExitRange = 12});
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
  bot.moveToX(0, 1900, {.maxSpeed = 30, .targetHeading = BLUE_STATION});
  bot.waitUntilDone();
  pros::delay(100);

  Pose temp = bot.getPose();
  bot.setPose({29, 48, temp.theta});
  pros::delay(100);
  bot.intake.goToIdle();

  // 2ND MATCHLOAD (RIGHT)
  bot.intake.goToStoring();
  bot.matchLoader.extend();

  bot.moveToX(farRightMatchLoader.x, 2000,
              {.maxSpeed = 45, .minSpeed = 10, .targetHeading = BLUE_STATION});
  bot.waitUntilDone();

  bot.tank(10, 10);
  pros::delay(1300);
  bot.tank(0, 0);

  bot.moveToX(farLongGoalRight.x, 1700,
                  { .maxSpeed = 55, .minSpeed = 20, .targetHeading = BLUE_STATION});
  bot.waitUntilDone();

  bot.intake.goToTOP();
  bot.moveToX(0, 1900, {.maxSpeed = 30, .targetHeading = BLUE_STATION});
  bot.waitUntilDone();
  pros::delay(100);

  // FIELD MIRROR RESET FOR RIGHT SIDE
  temp = bot.getPose();
  bot.setPose({29, 48, temp.theta});
  pros::delay(100);

  // TRANSITION
  bot.moveToPoint({1.65 * TILE, bot.getPose().y}, 1500,
                  {.maxSpeed = 70, .minSpeed = 40, .earlyExitRange = 5});
  bot.waitUntilDone();
  bot.intake.goToIdle();

  bot.swingToHeading(180, lemlib::DriveSide::RIGHT, 1200, {.maxSpeed = 55});
  bot.waitUntilDone();

  bot.moveToPoint({bot.getPose().x, -2 * TILE + 1}, 5500,
                  {.maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 14});
  bot.waitUntilDone();

  // 3RD MATCHLOAD (RIGHT CLOSE)
  bot.turnToHeading(BLUE_STATION, 2000, {.maxSpeed = 50});
  bot.waitUntilDone();

  bot.intake.goToStoring();
  bot.moveToX(closeRightMatchLoader.x, 2000, {.maxSpeed = 45, .minSpeed = 15});
  bot.waitUntilDone();

  bot.tank(10, 10);
  pros::delay(1300);
  bot.tank(0, 0);

  // OUT → CLOSE LONG GOAL
  bot.moveToPose({1.25 * TILE, -2.5 * TILE - 1, BLUE_STATION}, 1500,
                 {.forwards = false, .minSpeed = 35, .earlyExitRange = 3});
  bot.waitUntilDone();

  bot.matchLoader.retract();

  bot.moveToPoint({-TILE - 12, bot.getPose().y}, 5000,
                  {.forwards = false,
                   .maxSpeed = 80,
                   .minSpeed = 50,
                   .earlyExitRange = 12});
  bot.waitUntilDone();

  bot.swingToHeading(RED_STATION, lemlib::DriveSide::LEFT, 2000,
                     {.direction = AngularDirection::CW_CLOCKWISE,
                      .maxSpeed = 55,
                      .minSpeed = 20});
  bot.waitUntilDone();

  bot.moveToPoint(closeLongGoal, 1700, {.forwards = false, .maxSpeed = 55});
  bot.waitUntilDone();

  bot.intake.goToTOP();
  bot.moveToX(0, 1900, {.maxSpeed = 30, .targetHeading = RED_STATION});
  bot.waitUntilDone();
  pros::delay(100);

  temp = bot.getPose();
  bot.setPose({-29, -48, temp.theta});
  pros::delay(100);

  // 4TH MATCHLOAD (LEFT CLOSE)
  bot.intake.goToStoring();
  bot.matchLoader.extend();

  bot.moveToX(closeLeftMatchLoader.x, 2000, {.maxSpeed = 45, .minSpeed = 10, .targetHeading = RED_STATION});
  bot.waitUntilDone();

  bot.tank(10, 10);
  pros::delay(1300);
  bot.tank(0, 0);

  bot.moveToX(closeLongGoal.x, 1700,
                  { .maxSpeed = 55, .minSpeed = 20, .targetHeading = RED_STATION});
  bot.waitUntilDone();

  bot.intake.goToTOP();
  bot.moveToX(0, 1900, {.maxSpeed = 30, .targetHeading = RED_STATION});
  bot.waitUntilDone();
  pros::delay(100);

  // FINAL RESET + PARK
  temp = bot.getPose();
  bot.setPose({-29, -48, temp.theta});
  pros::delay(100);

  bot.matchLoader.retract();

  /*

  bot.moveToPoint({-1.75 * TILE, bot.getPose().y}, 1000);
  bot.waitUntilDone();
  bot.moveToPose({-1.8 * TILE, 0, 0}, 4000, {.maxSpeed = 65});
  bot.waitUntilDone();
  bot.turnToHeading(RED_STATION, 1500, {.maxSpeed = 80});
  bot.waitUntilDone();

  bot.park.extend();
  bot.tank(80, 80);
  pros::delay(2400);
  wiggle();
  */

  bot.moveToPose(
      {-2.8 * TILE, -TILE + 1, 0}, 2000,
      {.lead = 0.35, .maxSpeed = 90, .minSpeed = 60, .earlyExitRange = 7});
  bot.waitUntilDone();
  // bot.matchLoader.extend();
  bot.park.extend();
  // bot.intake.goToStoring();
  //  bot.park.extend();
  //  1650

  bot.tank(80, 80);
  // waitUntil([] { return bot.front.get_distance() < 1670; });
  pros::delay(1000);
  bot.matchLoader.retract();
  pros::delay(300);
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