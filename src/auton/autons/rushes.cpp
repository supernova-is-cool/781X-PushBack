#include "auton/autons.h"
#include "auton/transform.h"
#include "lemlib/pose.hpp"
#include "pros/abstract_motor.hpp"
#include "robot.h"
#include <cstdio>
#include <memory>

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
using SIGN = auton::SignTransform::SIGN;

static void rush(auton::SignTransform::SIGN sign) {
  // Programmed from right red perspective
  bot.setTransform(std::make_shared<auton::SignTransform>(
      sign, auton::SignTransform::AUDIENCE));

  const Pose startingPosition = {-2 * TILE - DRIVE_LENGTH / 2 - .5,
                                 -TILE + DRIVE_WIDTH / 2 - .5, 90};
  bot.setPose(startingPosition);

  const Pose centerBallsTarget = [&] {
    /** Drop the the matchloader after the blocks in the x,
     * but centered in the y between the blocks  */
    const Pose matchloader_target{
        -1 * TILE + BALL_INNER_DIAM * 1,
        -1 * TILE + BALL_INNER_DIAM / 2,
    };
    /** Distance between robot target pose and matchloader target */
    const float offset = -((MATCHLOADER_DIST_TO_CENTER + DRIVE_LENGTH / 2) / 2);
    return matchloader_target +
           Pose::fromPolar(offset, trigAngleToHeading(bot.getPose().angle(
                                       matchloader_target)));
  }();

  bot.intake.goToStoring();
  bot.moveToPoint(centerBallsTarget, 1200, {.minSpeed = 32});
  // Once within 6in of target, capture the balls with ML mech
  waitUntilDistToPose(centerBallsTarget, 6);
  bot.matchLoader.extend();

  const Pose matchLoader = {MIN_X + DRIVE_LENGTH / 2 + 3.5, -2 * TILE ,
                            RED_STATION};
  const Pose alignmentPoint = matchLoader.withX(-2 * TILE + 1);
  bot.turnToPoint(alignmentPoint, 500,
                  {.maxSpeed = 95, .minSpeed = 80, .earlyExitRange = 15});
  bot.waitUntilDone();

  bot.moveToPoint(alignmentPoint, 1000,
                  {.maxSpeed = 95, .minSpeed = 80, .earlyExitRange = 18});
  bot.moveToPoint(alignmentPoint, 1000);

  bot.waitUntilDone();
  bot.turnToHeading(RED_STATION, 1000);
  bot.waitUntilDone();

  bot.moveToPoint(matchLoader, 1100,
                  {.maxSpeed = 60, .minSpeed = 16});
  bot.waitUntilDone();

  bot.tank(10, 10);
  pros::delay(550);
  bot.tank(0, 0);

  const Pose longGoal = {-TILE - DRIVE_LENGTH / 2 + 4, -2 * TILE - 2,
                         RED_STATION};
  bot.moveToPoint(longGoal, 1500,
                  {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
  bot.waitUntilDone();

  // Smooth outaking sequence
  bot.tank(-10, -10);

  //bot.intake.goToTOP();
  //pros::delay(400);
  //bot.intake.goToOutaking();
  //pros::delay(200);
  bot.intake.goToTOP();
  pros::delay(2500);
  //bot.intake.goToOutaking();
  //pros::delay(200);
  bot.intake.goToTOP();

  bot.intake.goToIdle();
  pros::delay(100);
  bot.tank(0, 0);

  const Pose scoringPose = longGoal;
  // std::println("pose of scored: {}", bot.getPose());

  // Do not cross auton line when descoring
  bot.matchLoader.retract();

  const float distanceFromGoalToDescore = 11;
  // Exit long goal and align with long goal
  const Pose descoreAlignTarget =
      (scoringPose + Pose{0, distanceFromGoalToDescore})
          .withX(-TILE + DRIVE_LENGTH - 2);
  bot.swingToPoint(descoreAlignTarget, DriveSide::RIGHT, 500);
  bot.moveToPoint(descoreAlignTarget.withX(-1.7 * TILE), 1000);
  bot.waitUntilDone();

  // Don't extend descore, since retracted descore is at the correct height now
  bot.descore.retract();

  /** If on left side, should face towards center. */
  const bool faceCenter = sign == SIGN::REFEREE;
  const float descoreHeading = faceCenter ? BLUE_STATION : RED_STATION;
  bot.waitUntilDone();
  bot.turnToHeading(descoreHeading, 1000);
  bot.waitUntilDone();

  const Pose pushBlocksTarget{-DRIVE_LENGTH / 2, bot.getPose().y};
  bot.moveToPoint({-DRIVE_LENGTH / 2 - 3, bot.getPose().y}, 2500,
                  {.forwards = faceCenter, .maxSpeed = 67});
  bot.waitUntilDone();

  // Repeatedly run moveToPose to hold position against pushes
  // (When auton stage ends, this will also end)
  const Pose holdPose = [&] {
    const Pose currentPose = bot.getPose();
    return currentPose
        .withTheta(descoreHeading)
        // Do not go further than target and risk crossing auton line
        .withX(std::max(currentPose.x, pushBlocksTarget.x));
  }();
  while (true) {
    bot.moveToPose(holdPose, 1000, {}, false);
  }
}

void autons::rightRush() { rush(SIGN::AUDIENCE); }

void autons::leftRush() { rush(SIGN::REFEREE); }