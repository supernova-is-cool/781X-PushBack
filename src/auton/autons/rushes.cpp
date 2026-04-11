#include "auton/autons.h"
#include "auton/transform.h"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/pose.hpp"
#include "robot.h"
#include <cstdio>
#include <memory>

#include "auton/autons.h"
#include "auton/util.h"
#include "dimensions.h"
#include "lemlib/chassis/chassis.hpp"
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
  auto transform = std::make_shared<auton::SignTransform>(
      sign, auton::SignTransform::AUDIENCE);
  bot.setTransform(transform);

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

  // Expand descore
  bot.descore.extend();
  bot.intake.intake();
  bot.moveToPoint(centerBallsTarget, 1200, {.minSpeed = 32});
  // Once near target, capture the balls with ML mech
  waitUntilDistToPose(centerBallsTarget, 9);
  bot.matchLoader.extend();

  const Pose matchLoader = {MIN_X + DRIVE_LENGTH / 2, -2 * TILE, RED_STATION};
  const Pose alignmentPoint = matchLoader.withX(-2 * TILE + 1);
  bot.turnToHeading(RED_STATION - 45, 500,
                    {.maxSpeed = 95, .minSpeed = 80, .earlyExitRange = 30});
  bot.waitUntilDone();

  // Align in front of matchloader to hit ML perfectly
  bot.lateralPID.kP *= .8;
  bot.lateralPID.kD *= 1.2;
  bot.moveToY(matchLoader.y + TRACK_WIDTH / 2, 1000,
              {.earlyExitRange = 3, .targetHeading = RED_STATION - 45});
  bot.waitUntilDone();
  bot.lateralPID.kP /= .8;
  bot.lateralPID.kD /= 1.2;

  bot.swingToHeading(RED_STATION, lemlib::DriveSide::RIGHT, 1000,
                     {.minSpeed = 20, .earlyExitRange = 10});
  bot.waitUntilDone();

  // Go into match loader to pick up balls
  bot.moveToPoint(matchLoader, 2000, {.maxSpeed = 60});
  // Wait until stopped because we hit match loader
  waitUntil(
      [] {
        const auto speed = lemlib::getSpeed(false);
        return !bot.isInMotion() || (speed.distance({0, 0}) < .02);
      },
      50, INT_MAX, true);
  bot.cancelMotion();
  // Give a little time to fully grab the balls before exiting
  bot.tank(15, 15);
  pros::delay(000);
  bot.tank(0, 0);

  // Reset y position using distance sensor that is hitting the closest wall
  bot.setTransform(std::make_shared<auton::SignTransform>(
      auton::SignTransform::AUDIENCE, auton::SignTransform::AUDIENCE));
  bot.setPose(bot.getPose().withY(
      (sign == auton::SignTransform::REFEREE ? bot.right : bot.left)
          .getBotY(bot.getPose(true, true).theta)
          .value_or(bot.getPose().y)));
  bot.setTransform(transform);

  const Pose longGoal = {-TILE - DRIVE_LENGTH / 2 + 4, -2 * TILE, RED_STATION};
  bot.moveToPoint(longGoal, 1500,
                  {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
  // Score on long goal once near
  waitUntilDistToPose(longGoal, 15, 0, true);
//   bot.intake.goToTOP();
  bot.waitUntilDone();
  // Smooth outtaking sequence
  bot.tank(-10, -10);
  pros::delay(1500);
  bot.intake.stop();

  pros::delay(100);
  bot.tank(0, 0);

  const Pose scoringPose = longGoal;
  // std::println("pose of scored: {}", bot.getPose());

  // Do not cross auton line when descoring
  bot.matchLoader.retract();

  const float distanceFromGoalToDescore = 11;
  bot.descore.retract();

  /** If on left side, should face towards center. */
  const bool faceCenter = sign == SIGN::REFEREE;
  const float descoreHeading = faceCenter ? BLUE_STATION : RED_STATION;
  // Align descore with long goal (use different methods for each side)
  switch (sign) {
  case auton::SignTransform::AUDIENCE: {
    // Exit long goal and align to descore
    const Pose descoreAlignTarget =
        (scoringPose + Pose{0, distanceFromGoalToDescore})
            .withX(-TILE + DRIVE_LENGTH - 2);
    // bot.swingToPoint(descoreAlignTarget, DriveSide::RIGHT, 500);
    bot.moveToPoint(descoreAlignTarget.withX(-1.7 * TILE), 1000);
    bot.waitUntilDone();
    bot.turnToHeading(descoreHeading, 1000);
    bot.waitUntilDone();
    break;
  }
  case auton::SignTransform::REFEREE: {
    bot.swingToHeading(descoreHeading, lemlib::DriveSide::RIGHT, 500,
                       {
                           .direction = AngDir::CW_CLOCKWISE,
                           .minSpeed = 20,
                           .earlyExitRange = 90,
                       });
    bot.turnToHeading(descoreHeading, 500,
                      {
                          .direction = AngDir::CW_CLOCKWISE,
                          .earlyExitRange = 20,
                      });
    bot.waitUntilDone();
    break;
  }
  }

  // When descoring, do not coast. This is especially important if we time out
  // while descoring, as we are at risk of crossing.
  bot.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

  const Pose pushBlocksTarget{-DRIVE_LENGTH / 2 -
                                  (sign == SIGN::AUDIENCE ? 10 : 6),
                              bot.getPose().y, descoreHeading};
  bot.moveToX(
      pushBlocksTarget.x, 3000,
      {.maxSpeed = 48,
       .targetHeading = descoreHeading + (sign == SIGN::AUDIENCE ? 0 : -30)});
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
  bot.lateralLargeExit.range = 0;
  bot.lateralSmallExit.range = 0;
  while (true) {
    bot.moveToX(holdPose.x, 1000, {.targetHeading = descoreHeading}, false);
  }
}

void autons::rightRush() { rush(SIGN::AUDIENCE); }

void autons::leftRush() { rush(SIGN::REFEREE); }