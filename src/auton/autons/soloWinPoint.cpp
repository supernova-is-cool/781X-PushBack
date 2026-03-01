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
  const Pose startingPosition = {-2 * TILE, -TILE + DRIVE_LENGTH / 2 + 1,
                                 AUDIENCE};
  bot.setPose(startingPosition);

  const Pose rightMatchloader = {MIN_X + DRIVE_LENGTH / 2 + 3, -2 * TILE + 2,
                                 RED_STATION};

  bot.lateralPID.kP *= .75;

  // Align in front of matchloader
  bot.moveToY(rightMatchloader.y, 2000);
  bot.waitUntilDone();

  // Prepare matchloader mech and intake for scoring
  bot.intake.goToStoring();
  bot.matchLoader.extend();

  // Face matchloader to prepare for scoring
  bot.turnToHeading(RED_STATION, 1000);
  bot.waitUntilDone();
  // Move into matchloader to pick up blocks
  bot.moveToX(rightMatchloader.x, 1500, {.maxSpeed = 50});
  bot.waitUntilDone();

  // Give time to yoink the blocks
  bot.tank(10, 10);
  pros::delay(200);
  bot.tank(0, 0);

  const Pose rightLongGoal = {-TILE - DRIVE_LENGTH / 2 + 4, rightMatchloader.y,
                              RED_STATION};

  // Score 4 blocks on long goal
  bot.moveToPoint(rightLongGoal, 2000,
                  {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
  bot.waitUntilDone();

  // Smooth scoring sequence
  bot.tank(-10, -10);

  bot.intake.goToTOP();
  // pros::delay(400);
  // bot.intake.goToOutaking();
  // pros::delay(200);
  // bot.intake.goToTOP();
  pros::delay(2500);
  // bot.intake.goToOutaking();
  // pros::delay(200);
  bot.intake.goToTOP();

  bot.intake.goToIdle();
  pros::delay(100);
  bot.tank(0, 0);

  const Pose rightCenterBallsTarget{
      -TILE + 1 * BALL_INNER_DIAM - DRIVE_WIDTH / 2, -TILE};
  const Pose leftCenterBallsTarget{rightCenterBallsTarget.x,
                                   -rightCenterBallsTarget.y};
  // Exit long goal
  bot.swingToPoint(rightCenterBallsTarget, lemlib::DriveSide::RIGHT, 1000,
                   {.minSpeed = 48, .earlyExitRange = 30});
  // Scoop up the center balls
  bot.matchLoader.retract();
  bot.intake.goToStoring();
  bot.moveToPoint(rightCenterBallsTarget, 3000,
                  {.maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 6});
  bot.moveToPoint(leftCenterBallsTarget, 2000,
                  {.maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 6});
  // Wait until facing left center balls
  waitUntil(
      [&] {
        return !bot.isInMotion() ||
               robotAngDist(90.0 - lemlib::radToDeg(bot.getPose().angle(
                                       leftCenterBallsTarget))) < 10;
      },
      100, 5000, true);
  bot.cancelMotion();

  const float middleGoalTargetTheta = -45;
  // Move to be directly in front of middle goal
  bot.moveToLine({0, 0, middleGoalTargetTheta}, 1500);
  bot.waitUntilDone();

  // Go into middle goal
  bot.turnToHeading(middleGoalTargetTheta, 1000);
  const Pose middleGoalTarget = Pose::fromPolar(4, middleGoalTargetTheta);
  bot.moveToPoint(middleGoalTarget, 1000, {.forwards = false, .maxSpeed = 80});
  bot.waitUntilDone();

  // Score on middle goal
  bot.tank(-10, -10);
  bot.intake.goToMIDDLE();
  pros::delay(100);
  bot.intake.goToOutaking();
  pros::delay(100);
  bot.tank(0, 0);
  bot.intake.goToMIDDLE();
  pros::delay(1600);
  bot.intake.goToIdle();
  pros::delay(150);

  const Pose leftMatchloader = {rightMatchloader.x, -rightMatchloader.y,
                                RED_STATION};
  const Pose leftMatchloaderAlignTarget = leftMatchloader.withX(-2 * TILE);

  // Exit middle goal
  tank(127, 127, 30, 0);
  // Aim to align with left matchloader
  bot.swingToPoint(leftMatchloaderAlignTarget, lemlib::DriveSide::LEFT, 500);
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
  bot.tank(10, 10);
  pros::delay(700);
  bot.tank(0, 0);

  // Score on left long goal
  const Pose leftLongGoal = {rightLongGoal.x, -rightLongGoal.y, RED_STATION};

  // Score 4 blocks on long goal
  bot.moveToPoint(leftLongGoal, 1500,
                  {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
  bot.waitUntilDone();

  // Smooth scoring sequence
  bot.tank(-10, -10);

  bot.intake.goToTOP();
  pros::delay(400);
  bot.intake.goToOutaking();
  pros::delay(200);
  bot.intake.goToTOP();
  pros::delay(2500);
  bot.intake.goToOutaking();
  pros::delay(200);
  bot.intake.goToTOP();

  bot.intake.goToIdle();
  pros::delay(100);
  bot.tank(0, 0);
}