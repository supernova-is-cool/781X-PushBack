#include "auton/autons.h"
#include "auton/transform.h"
#include "lemlib/pose.hpp"
#include "pros/abstract_motor.hpp"
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
using namespace auton;

using lemlib::Pose;
using AngDir = lemlib::AngularDirection;

void autons::doNothing() { printf("yo what's up\n"); }

/**
 * @brief Rush auton implementation.
 * Grabs 3 center balls and 3 matchloader balls, scoring them all in long goal.
 * Then, uses descore to push blocks in long goal, securing control bonus.
 *
 * @param side Which side to run the auton on.
 *    - Left  == SignTransform::SIGN::REFEREE
 *    - Right == SignTransform::SIGN::AUDIENCE
 */
static void rush(SignTransform::SIGN side) {
  // Change coordinate system based on side being run.
  // Programmed relative to negative/audience side
  auto signTransform =
      std::make_shared<SignTransform>(side, SignTransform::AUDIENCE);
  bot.setTransform(signTransform);

  const Pose startingPosition = {-2 * TILE - DRIVE_LENGTH / 2 - .5,
                                 -0.7 * TILE + 2, 90};
  bot.setPose(startingPosition);

  const Pose theBalls = {
      -1 * TILE,
      -1 * TILE,
  };

  bot.intake.goToStoring();
  bot.moveToPoint(theBalls, 1200, {.minSpeed = 67, .earlyExitRange = 8});
  bot.waitUntil(20);
  bot.matchLoader.extend();
  bot.waitUntilDone();
  // pros::delay(750);
  bot.turnToPoint({-2 * TILE, -2 * TILE}, 1200,
                  {.maxSpeed = 95, .minSpeed = 80, .earlyExitRange = 17});
  bot.waitUntilDone();
  bot.moveToPoint({-2 * TILE + 1, -2 * TILE + 1}, 1000,
                  {.maxSpeed = 95, .minSpeed = 80, .earlyExitRange = 15});

  const Pose matchLoader = {MIN_X + DRIVE_LENGTH / 2 + 6, -2 * TILE - 1,
                            RED_STATION};
  bot.waitUntilDone();
  bot.turnToHeading(RED_STATION, 1000);
  bot.waitUntilDone();
  bot.moveToPoint(matchLoader, 1100, {.maxSpeed = 45});
  bot.waitUntilDone();

  bot.tank(10, 10);
  pros::delay(650);
  bot.tank(0, 0);

  /** Position of long goal inlet **SHOULD NOT BE TUNED** */
  const Pose longGoalInlet = {-TILE, -2 * TILE, RED_STATION};
  /** Position to go to in order to score in long goal */
  const Pose longGoalTarget = longGoalInlet + Pose{-DRIVE_LENGTH / 2 + 4, -2};

  bot.lift.extend();

  bot.moveToPoint(longGoalTarget, 1500,
                  {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
  bot.waitUntilDone();

  // Smooth outaking sequence
  bot.tank(-10, -10);

  bot.intake.goToScoring();
  pros::delay(400);
  bot.intake.goToOutaking();
  pros::delay(200);
  bot.intake.goToScoring();
  pros::delay(2500);
  bot.intake.goToIdle();
  pros::delay(100);
  bot.tank(0, 0);

  /** The side of the goal the bot should go to, in order to use descore.
   * -1 is towards AUDIENCE, +1 is towards REFEREE */
  const int descoreSideOfGoalSignum =
      side == SignTransform::SIGN::AUDIENCE ? 1 : -1;

  const Pose exitLongGoalTarget{-1.7 * TILE,
                                longGoalInlet.y + descoreSideOfGoalSignum * 10.25f};

  bot.moveToPoint(exitLongGoalTarget, 1000);
  bot.waitUntilDone();

  const Pose descore = {-TILE + DRIVE_LENGTH - 2,
                        longGoalInlet.y + descoreSideOfGoalSignum * 8.75f,
                        RED_STATION};
  bot.descore.extend();

  // Do not cross auton line
  bot.matchLoader.retract();

  // bot.moveToPoint(descore, 2000, {.forwards = false});
  bot.waitUntilDone();
  bot.turnToHeading(RED_STATION, 1000);
  bot.waitUntilDone();
  bot.moveToPoint({-5, bot.getPose().y}, 2500,
                  {.forwards = false, .maxSpeed = 67});
  bot.waitUntil(20);
  bot.descore.retract();
  bot.waitUntilDone();
  const Pose currentPose = bot.getPose();

  // Repeatedly run moveToPose to hold position against pushes
  // (When auton stage ends, this will also end)
  while (true) {
    bot.moveToPose(currentPose, 1000);
  }
}

void autons::leftRush() { rush(SignTransform::SIGN::REFEREE); }
void autons::rightRush() { rush(SignTransform::SIGN::AUDIENCE); }
