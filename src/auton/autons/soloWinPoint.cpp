#include "auton/autons.h"
#include "lemlib/logger/logger.hpp"
#include "lemlib/logger/message.hpp"
#include "lemlib/pose.hpp"
#include "robot.h"
#include <cstdio>

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

void autons::soloWinPoint() {
  // === STARTING POSE ===

  // TODO: Determine good sawp starting pose
  const Pose startingPosition = {-2 * TILE + 1.5, -2.5, REFEREE};
  bot.setPose(startingPosition);

  // Push Alliance partner and pick up their preload
  bot.intake.intake();
  bot.moveToY(startingPosition.y + BALL_INNER_DIAM + 2, 2000, {.minSpeed = 48});
  bot.waitUntilDone();
  const Pose rightMatchloader = {MIN_X + DRIVE_LENGTH / 2 + 4, -2 * TILE + 1,
                                 RED_STATION};

  bot.lateralPID.kP *= .75;
  bot.lateralPID.kD *= 1.1;
  // Align in front of matchloader
  lemlib::infoSink()->setLowestLevel(lemlib::Level::INFO);
  bot.moveToY(rightMatchloader.y, 2000);
  bot.waitUntil(TILE);
  bot.matchLoader.extend();
  bot.waitUntilDone();
  bot.lateralPID.kD /= 1.1;
  lemlib::infoSink()->setLowestLevel(lemlib::Level::WARN);
  bot.lateralLargeExit.time = 200;
  bot.lateralSmallExit.time = 50;

  // Prepare matchloader mech and intake for storing
  bot.intake.intake();

  // Face matchloader to prepare for storing
  bot.turnToHeading(RED_STATION, 1000, {.earlyExitRange = 10});
  bot.waitUntilDone();
  // Move into matchloader to pick up blocks
  bot.moveToX(rightMatchloader.x, 1500,
              {.maxSpeed = 64, .targetHeading = RED_STATION});
  bot.waitUntilDone();
  const size_t matchloaderTimeToStore = 200;
  // Give time to yoink the blocks
  tank(10, 10, matchloaderTimeToStore, 0);

  const Pose rightLongGoal = {-TILE - 6, -2 * TILE, RED_STATION};

  // Score 4 blocks on long goal
  bot.moveToPoint(
      rightLongGoal, 2000,
      {.forwards = false, .maxSpeed = 96, .minSpeed = 45, .earlyExitRange = 3});
  waitUntilDistToPose(rightLongGoal, 9, 0, true);
  //   bot.intake.goToTOP();
  waitUntilDistToPose(rightLongGoal, 3, 0, true);
  bot.cancelMotion();

  // Push into long goal while scoring and try to maintain a heading of
  // RED_STATION
  bot.moveToX(0, 1250, {.maxSpeed = 20, .targetHeading = RED_STATION});
  bot.waitUntilDone();
  bot.intake.stop();
  // Retract matchloader to prep for center balls
  bot.matchLoader.retract();

  // Reset y position
  bot.leftLaser().resetY();

  pros::delay(100);
  bot.tank(0, 0);

  const Pose rightCenterBallsTarget{
      -TILE + DRIVE_WIDTH / 2 - 2 * BALL_INNER_DIAM, -TILE};
  const Pose leftCenterBallsTarget{rightCenterBallsTarget.x,
                                   -rightCenterBallsTarget.y};
  // Exit long goal
  bot.turnToPoint(rightCenterBallsTarget, 1000,
                  {.minSpeed = 32, .earlyExitRange = 20});
  bot.waitUntilDone();
  bot.intake.intake();
  bot.moveToPoint(rightCenterBallsTarget, 1500,
                  {.minSpeed = 32, .earlyExitRange = 9});
  bot.waitUntilDone();

  //   bot.moveToPoint(leftCenterBallsTarget, 2000,
  //                   {.maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 8});
  //   // Wait until facing and near left center balls
  //   waitUntil([&] { return !bot.isInMotion() || bot.getPose().y > -TILE *
  //   .25; },
  //             0, 5000);
  //   bot.cancelMotion();
  //   bot.matchLoader.retract();

  const float middleGoalTargetTheta = REFEREE - 45;

  //   bot.lateralPID.kP *= 1.25;
  bot.lateralPID.kD *= 1.25;
  // Move to be directly in front of middle goal
  // Adjust odom pose
  bot.setPose(bot.getPose() + Pose{0, 2, 0});
  bot.moveToLine({0, -2, middleGoalTargetTheta}, 2000,
                 {.targetHeading = REFEREE});
  // Wait until matchloader is in position to trap leftCenterBalls with ML
  // mech
  waitUntil([&] {
    return !bot.isInMotion() || leftCenterBallsTarget.y + BALL_INNER_DIAM / 2 -
                                        MATCHLOADER_DIST_TO_CENTER - 5 <
                                    bot.getPose().y;
  });
  bot.matchLoader.extend();
  bot.waitUntilDone();
  //   bot.lateralPID.kP /= 1.25;
  bot.lateralPID.kD /= 1.25;

  // Go into middle goal
  bot.turnToHeading(middleGoalTargetTheta, 1000, {.minSpeed = 40, .earlyExitRange = 8});
  const Pose middleGoalTarget = Pose::fromPolar(13, middleGoalTargetTheta);
  bot.moveToPoint(middleGoalTarget, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 30});
  bot.waitUntilDone();

  // Score on middle goal
  //   bot.intake.goToMIDDLE();
  bot.intake.intake();
  bot.lateralPID.kP *= .5;
  bot.moveToLine(
      {4, 0, middleGoalTargetTheta + 90}, 1250,
      {.maxSpeed = 40, .minSpeed = 12, .targetHeading = middleGoalTargetTheta});
  bot.waitUntilDone();
  bot.lateralPID.kP /= .5;

  const Pose leftMatchloader = {rightMatchloader.x, -rightMatchloader.y - 1,
                                RED_STATION};

  // Recover any outtaked blocks
  bot.intake.outtake();
  // Align with left matchloader
  bot.moveToY(leftMatchloader.y, 2000,
              {.targetHeading = middleGoalTargetTheta + 5});
  // Let outtake spin to hold onto blocjks
  pros::delay(300);
  bot.intake.intake();
  bot.waitUntilDone();

  // Prepare matchloader mech and intake for scoring
  bot.intake.intake();
  bot.matchLoader.extend();

  bot.angularPID.kP *= 1.25;
  // Face matchloader to prepare for scoring
  bot.turnToHeading(RED_STATION, 1000, {.earlyExitRange = 10});
  //bot.waitUntilDone();
  bot.angularPID.kP /= 1.25;

  // Move into matchloader to pick up blocks
  bot.moveToX(leftMatchloader.x, 1300,
              {.maxSpeed = 64, .targetHeading = RED_STATION});
  bot.waitUntilDone();

  // Reset y position
  bot.rightLaser().resetY();

  // Give time to yoink the blocks
  tank(10, 10, matchloaderTimeToStore, 0);

  // Score on left long goal
  const Pose leftLongGoal = {rightLongGoal.x, -rightLongGoal.y, RED_STATION};

  // Score 4 blocks on long goal
  bot.moveToPoint(
      leftLongGoal, 2000,
      {.forwards = false, .maxSpeed = 96, .minSpeed = 16, .earlyExitRange = 3});
  waitUntilDistToPose(leftLongGoal, 9, 0, true);
  //   bot.intake.goToTOP();
  waitUntilDistToPose(leftLongGoal, 3, 0, true);
  bot.cancelMotion();

  // Push into long goal while scoring and try to maintain a heading of
  // RED_STATION
  bot.moveToX(0, 1250, {.maxSpeed = 20, .targetHeading = RED_STATION});
  bot.waitUntilDone();
  bot.intake.stop();

  // Leave Long goal to avoid touching our blocks
  tank(64, 64, 200, 0);
  stop();
}