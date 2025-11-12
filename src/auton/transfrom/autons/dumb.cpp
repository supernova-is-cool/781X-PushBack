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

void dumbSkills() {
bot.intake.disableFiltering();

// === STARTING POSE ===
const Pose startingPose{(-2 * TILE) - DRIVE_WIDTH + 3.5, 0.5 + DRIVE_LENGTH, 0};
bot.setPose(startingPose);

// === MATCH LOADER INTAKE ===
const Pose farMatchLoader{-(3 * TILE), 2 * TILE, RED_STATION};

// Smooth approach toward long goal
bot.moveToPoint({-2 * TILE, 2 * TILE + 1}, 1600);
bot.waitUntilDone();
pros::delay(100);

// Deploy before turning
bot.littleWill.extend();
bot.turnToPoint(farMatchLoader, 1300, {.maxSpeed = 60, .minSpeed = 35});
bot.waitUntilDone();

// Intake cubes from match loader slowly
bot.moveToPoint(farMatchLoader, 1600, {.maxSpeed = 60, .minSpeed = 30});
bot.intake.goToScoring();
bot.waitUntilDone();

/*
// === JIGGLE SEQUENCE ===
for (int i = 0; i < 2; i++) {
  bot.tank(-55, -55);
  pros::delay(200);
  bot.tank(0, 0);
  pros::delay(150);
  bot.tank(55, 55);
  pros::delay(300);
  bot.tank(0, 0);
  //bot.intake.goToScoring();
  pros::delay(175);
}
*/

// Let cubes settle before next move
pros::delay(900);
bot.intake.goToScoring();
pros::delay(200);
bot.intake.goToIdle();

// === LONG GOAL SCORING ===
const Pose farLongGoal{-TILE, 2 * TILE, BLUE_STATION};

// Back up carefully
bot.moveToPoint({-2 * TILE, 2 * TILE}, 1400, {.forwards = false, .maxSpeed = 65, .minSpeed = 35});
bot.waitUntilDone();

// Turn precisely toward goal
bot.turnToPoint(farLongGoal, 1500, {.maxSpeed = 55});
bot.waitUntilDone();
bot.littleWill.retract();

// Elevators and fine approach
bot.leftElevator.extend();
bot.rightElevator.extend();
pros::delay(200);

bot.moveToPoint(farLongGoal, 1400, {.maxSpeed = 55, .minSpeed = 30});
bot.waitUntilDone();
bot.intake.goToScoring();
// Final scoring push
bot.tank(15, 15);
/*
for (int i = 0; i < 2; i++) {
  bot.intake.goToOutaking();
  pros::delay(250);
  bot.intake.goToScoring();
  pros::delay(250);
}
*/

pros::delay(6000);
bot.tank(0, 0);
bot.intake.goToIdle();

// ✅ Reset odometry since bot is stationary & aligned
Pose currentPose = bot.getPose();
bot.setPose({-30, 52, currentPose.theta});
pros::delay(100);

// === DRIVE DOWNFIELD ===
bot.moveToPoint({-1.5 * TILE, 2 * TILE}, 1200, {.forwards = false, .maxSpeed = 65, .minSpeed = 35});
bot.waitUntilDone();
pros::delay(100);

bot.turnToHeading(180, 1200, {.maxSpeed = 60, .minSpeed = 35});
bot.waitUntilDone();
pros::delay(100);

bot.leftElevator.retract();
bot.rightElevator.retract();

// Drive straight down to far field
bot.moveToPoint({-2 * TILE, -2 * TILE}, 4400, {.maxSpeed = 76});
bot.waitUntil(48);
bot.intake.goToOutaking();
bot.waitUntilDone();

// === SECOND MATCH LOADER INTAKE ===
const Pose closeMatchLoader{-(3 * TILE), -2 * TILE, RED_STATION};

bot.littleWill.extend();
bot.turnToPoint(closeMatchLoader, 1300);
bot.waitUntilDone();

bot.moveToPoint(closeMatchLoader, 1600, {.maxSpeed = 50, .minSpeed = 30});
bot.intake.goToScoring();
bot.waitUntilDone();

/*
// Jiggle cubes into place
for (int i = 0; i < 2; i++) {
  bot.tank(-55, -55);
  pros::delay(200);
  bot.tank(0, 0);
  pros::delay(150);
  bot.tank(55, 55);
  pros::delay(300);
  bot.tank(0, 0);
  //bot.intake.goToScoring();
  pros::delay(125);
}

  bot.tank(-55, -55);
  pros::delay(200);
  bot.tank(0, 0);
  pros::delay(150);
  bot.tank(55, 55);
  pros::delay(300);
  bot.tank(0, 0);
  pros::delay(125);
*/

// Final feed + pause
pros::delay(900);
bot.intake.goToScoring();
pros::delay(200);
bot.intake.goToIdle();

// === CLOSE GOAL SCORING ===
const Pose closeGoal = {-TILE, -2 * TILE, BLUE_STATION};

bot.moveToPoint({-2 * TILE, -2 * TILE}, 800, {.forwards = false});
bot.waitUntilDone();
pros::delay(100);

bot.turnToPoint(closeGoal, 1500, {.maxSpeed = 55, .minSpeed = 30});
bot.waitUntilDone();
bot.littleWill.retract();

bot.leftElevator.extend();
bot.rightElevator.extend();
pros::delay(200);

bot.moveToPoint(closeGoal, 1400, {.maxSpeed = 55, .minSpeed = 30});
bot.waitUntilDone();
bot.intake.goToScoring();

// Final scoring sequence
bot.tank(15, 15);

/*
for (int i = 0; i < 2; i++) {
  bot.intake.goToOutaking();
  pros::delay(250);
  bot.intake.goToScoring();
  pros::delay(250);
}
*/

pros::delay(6000);
bot.tank(0, 0);
bot.intake.goToIdle();

// === SAFETY POSE UPDATE ===
// === PARK SEQUENCE ===

bot.setPose(bot.getPose());
pros::delay(75);

bot.moveToPoint({-2 * TILE, -2 * TILE}, 1000, {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
bot.waitUntilDone();
pros::delay(100);

bot.moveToPoint({-2.67 * TILE, -1 * TILE}, 1300, {.maxSpeed = 80, .minSpeed = 40});
bot.waitUntilDone();
pros::delay(100);

bot.swingToHeading(0, lemlib::DriveSide::RIGHT, 1200, {.maxSpeed = 60, .minSpeed = 30});
bot.waitUntilDone();
pros::delay(150);
bot.intake.goToScoring();
//bot.littleWill.extend();
bot.leftElevator.retract();
bot.rightElevator.retract();
pros::delay(200);
bot.tank(-40, -40);
pros::delay(500);
bot.tank(0, 0);
pros::delay(300);

/*
bot.moveToPoint({MIN_X + 2, -0.5 * TILE}, 1200, {.maxSpeed = 70, .minSpeed = 35});
bot.waitUntilDone();
pros::delay(100);
*/

// 5️⃣ Final push — gentle to avoid overshooting
bot.tank(80, 80);
pros::delay(600);
bot.littleWill.extend();
pros::delay(775);
bot.tank(0, 0);
bot.littleWill.retract();
pros::delay(100);


// ✅ Stop everything cleanly
//bot.intake.goToIdle();



  /*
  bot.intake.goToScoring();
  bot.setPose({0, 0, 0});
  bot.moveToPoint({0, -15, 0}, 1000, {.forwards = false});
  bot.waitUntilDone();
  pros::delay(200);
  bot.tank(127, 127, false);
  pros::delay(1600);
  bot.tank(0, 0);
  */
}