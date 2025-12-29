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

void autons::center(){

// Define field positions
const Pose startingPosition{0, 0, 0};
const Pose farBalls{-1 * TILE, 1 * TILE};
const Pose bottomGoal{-0.5 * TILE - 5, 0.5 * TILE + 5, 135};
const Pose closeBalls{-1 * TILE, -1 * TILE};
const Pose centerGoal{-0.5 * TILE - 5, -0.5 * TILE + 5, 45};

// Initialize pose
bot.setPose(startingPosition);

// === INTAKE FAR BALLS ===
bot.swingToPoint(farBalls, lemlib::DriveSide::RIGHT, 1000, {
    .maxSpeed = 75,
    .minSpeed = 40
});
bot.intake.goToStoring();
bot.waitUntilDone();

// Smooth forward motion to collect
bot.moveToPoint(farBalls, 1200, {
    .maxSpeed = 65,
    .minSpeed = 25,
    .earlyExitRange = 2
});
bot.waitUntilDone();
pros::delay(150);

// === SCORE IN BOTTOM GOAL ===
bot.moveToPose(bottomGoal, 1300, {
    .maxSpeed = 70,
    .minSpeed = 30,
    .earlyExitRange = 3
});
bot.waitUntilDone();
bot.intake.goToScoring();
pros::delay(1000);
bot.intake.goToIdle();
pros::delay(200);

// === INTAKE CLOSE BALLS ===
// Move backward to clear goal
bot.moveToPoint({-1 * TILE, TILE}, 1200, {
    .forwards = false,
    .maxSpeed = 60,
    .minSpeed = 30
});
bot.waitUntilDone();

// Turn toward next stack
bot.swingToPoint(closeBalls, lemlib::DriveSide::LEFT, 1000, {
    .maxSpeed = 65,
    .minSpeed = 35
});
bot.waitUntilDone();

// Start intake slightly before moving
bot.intake.goToStoring();
pros::delay(100);

// Approach balls smoothly
bot.moveToPoint(closeBalls, 1200, {
    .maxSpeed = 60,
    .minSpeed = 25,
    .earlyExitRange = 2
});
bot.waitUntilDone();
pros::delay(150);

// === SCORE IN CENTER GOAL ===
bot.turnToPoint(centerGoal, 1000, {.maxSpeed = 70, .minSpeed = 30});
bot.waitUntilDone();
bot.moveToPose(centerGoal, 1300, {.maxSpeed = 70, .minSpeed = 30});
bot.waitUntilDone();

bot.intake.goToScoring();
pros::delay(1000);
bot.intake.goToIdle();


}