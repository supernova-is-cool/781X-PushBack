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

void autons::soloWinPoint() {
  // === STARTING POSE ===

  // TODO: Determine good sawp starting pose
  const Pose startingPosition = {-2 * TILE + 1.5, -2.5, REFEREE};
  bot.setPose(startingPosition);

  // Push Alliance partner and pick up their preload
  bot.intake.goToStoring();
  bot.moveToY(startingPosition.y + BALL_INNER_DIAM + 2, 2000, {.minSpeed = 48});
  bot.waitUntilDone();
  const Pose rightMatchloader = {MIN_X + DRIVE_LENGTH / 2 + 4.5, -2 * TILE,
                                 RED_STATION};
  bot.lateralPID.kP *= .75;

  // Align in front of matchloader
  bot.moveToY(rightMatchloader.y, 2000);
  bot.waitUntilDone();

  // Prepare matchloader mech and intake for storing
  bot.intake.goToStoring();
  bot.matchLoader.extend();

  // Face matchloader to prepare for storing
  bot.turnToHeading(RED_STATION, 1000);
  bot.waitUntilDone();
  // Move into matchloader to pick up blocks
  bot.moveToX(rightMatchloader.x, 1500, {.maxSpeed = 50});
  bot.waitUntilDone();
  const size_t matchloaderTimeToStore = 200;
  // Give time to yoink the blocks
  tank(10, 10, matchloaderTimeToStore, 0);

  const Pose rightLongGoal = {-TILE - DRIVE_LENGTH / 2 + 4, rightMatchloader.y,
                              RED_STATION};

  // Score 4 blocks on long goal
  bot.moveToPoint(rightLongGoal, 2000,
                  {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
  bot.waitUntilDone();

  bot.intake.goToTOP();
  tank(-10, -10, 1250, 0);
  bot.intake.goToIdle();

  pros::delay(100);
  bot.tank(0, 0);

  const Pose rightCenterBallsTarget{
      -TILE + 1.25 * BALL_INNER_DIAM - DRIVE_WIDTH / 2, -TILE};
  const Pose leftCenterBallsTarget{rightCenterBallsTarget.x,
                                   -rightCenterBallsTarget.y};
  // Exit long goal
  tank(48, 16, 100, 0);

  // Prepare to scoop up the center balls
  bot.matchLoader.retract();
  bot.intake.goToStoring();

  // Go to the right of the right center balls
  bot.swingToPoint(rightCenterBallsTarget, lemlib::DriveSide::RIGHT, 500,
                   {.minSpeed = 48, .earlyExitRange = 45});
  bot.turnToPoint(rightCenterBallsTarget, 500);
  {
    const float startTheta = bot.getPose().theta;
    const float finalCenterX = rightCenterBallsTarget.x;
    const float leftDriveX = finalCenterX - TRACK_WIDTH / 2;
    const float intermediateCenterX =
        leftDriveX + TRACK_WIDTH / 2 * cos(lemlib::degToRad(startTheta));
    bot.moveToX(intermediateCenterX, 2000,
                {.minSpeed = 32, .earlyExitRange = 5});
  }
  // Wait until near the right center balls to trap with matchloader
  waitUntil([&] {
    return !bot.isInMotion() ||
           bot.getPose().distance(rightCenterBallsTarget) <
               BALL_INNER_DIAM + MATCHLOADER_DIST_TO_CENTER - 2;
  });
  bot.matchLoader.extend();

  // Aim to scoop up left center balls
  bot.swingToPoint(leftCenterBallsTarget, lemlib::DriveSide::LEFT, 500);
  bot.moveToPoint(leftCenterBallsTarget, 2000,
                  {.maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 6});
  // Wait until facing and near left center balls
  waitUntil(
      [&] {
        return !bot.isInMotion() ||
               (robotAngDist(90.0 - lemlib::radToDeg(bot.getPose().angle(
                                        leftCenterBallsTarget))) < 10 &&
                leftCenterBallsTarget.y - bot.getPose().y < 1.5 * TILE);
      },
      100, 5000, true);
  bot.cancelMotion();
  bot.matchLoader.retract();

  const float middleGoalTargetTheta = -45;
  // Move to be directly in front of middle goal
  bot.moveToLine({0, 0, middleGoalTargetTheta}, 1500);
  // Wait until matchloader is in position to trap leftCenterBalls with ML mech
  waitUntil([&] {
    return !bot.isInMotion() || leftCenterBallsTarget.y + BALL_INNER_DIAM / 2 -
                                        MATCHLOADER_DIST_TO_CENTER + 2 <
                                    bot.getPose().y;
  });
  bot.matchLoader.extend();
  bot.waitUntilDone();

  // Go into middle goal
  bot.turnToHeading(middleGoalTargetTheta, 1000);
  const Pose middleGoalTarget = Pose::fromPolar(13, middleGoalTargetTheta);
  bot.moveToPoint(middleGoalTarget, 1000, {.forwards = false, .maxSpeed = 80});
  bot.waitUntilDone();

  // Score on middle goal
  bot.intake.goToMIDDLE();
  tank(-10, -10, 1250, 0);
  // Push blocks a bit maybe with trapdoor, and hold onto any remaining blocks
  bot.intake.goToOutaking();
  stop();
  pros::delay(100);
  // Recover any outtaked blocks
  bot.intake.goToStoring();

  // Adjust odom pose
  bot.setPose(bot.getPose() + Pose{0, 2, 0});

  const Pose leftMatchloader = {rightMatchloader.x, -rightMatchloader.y,
                                RED_STATION};
  const Pose leftMatchloaderAlignTarget = leftMatchloader.withX(-2 * TILE);

  // Align with left matchloader
  bot.moveToY(leftMatchloader.y, 2000);
  bot.waitUntilDone();

  // Prepare matchloader mech and intake for scoring
  bot.intake.goToStoring();
  bot.matchLoader.extend();

  bot.angularPID.kP *= 1.25;
  // Face matchloader to prepare for scoring
  bot.turnToHeading(RED_STATION, 1000);
  bot.waitUntilDone();
  bot.angularPID.kP /= 1.25;

  // Move into matchloader to pick up blocks
  bot.moveToX(leftMatchloader.x, 1000, {.maxSpeed = 50});
  bot.waitUntilDone();

  // Give time to yoink the blocks
  tank(10, 10, matchloaderTimeToStore, 0);

  // Score on left long goal
  const Pose leftLongGoal = {rightLongGoal.x, -rightLongGoal.y, RED_STATION};

  // Score 4 blocks on long goal
  bot.moveToPoint(leftLongGoal, 1500,
                  {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
  bot.waitUntilDone();

  bot.intake.goToTOP();
  tank(-10, -10, 1250, 0);
  bot.intake.goToIdle();

  // Leave Long goal to avoid touching our blocks
  tank(48, 48, 100, 0);
  stop();
}