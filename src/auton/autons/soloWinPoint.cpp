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

/** Pose thetas in standard radians */
std::optional<float> signed_ray_distance(const Pose ray, const Pose line) {
  // Based on:
  // https://github.com/darkMatter781x/HighStakes/blob/bb707b52cfde57a23899ca5c580d8f667b7515e9/src/pf/util/lines.cpp#L22
  const float q_x = ray.x;
  const float q_y = ray.y;
  const float s_x = cos(ray.theta);
  const float s_y = sin(ray.theta);
  const float p_x = line.x;
  const float p_y = line.y;
  const float r_x = cos(line.theta);
  const float r_y = sin(line.theta);

  const float r_cross_s = r_x * s_y - r_y * s_x;
  if (r_cross_s == 0)
    return std::nullopt;
  const auto q_minus_p_x = q_x - p_x;
  const auto q_minus_p_y = q_y - p_y;

  const float q_minus_p_cross_s = q_minus_p_x * s_y - q_minus_p_y * s_x;
  const float t = q_minus_p_cross_s / r_cross_s;

  return t;
}

void autons::soloWinPoint() {
  // === STARTING POSE ===

  // TODO: Determine good sawp starting pose
  const Pose startingPosition = {-2 * TILE - 1.5, -(TILE - DRIVE_LENGTH + 8),
                                 AUDIENCE};
  bot.setPose(startingPosition);

  const Pose rightMatchloader = {MIN_X + DRIVE_LENGTH / 2 + 3.5,
                                 -2 * TILE, RED_STATION};
  // Align in front of matchloader
  bot.moveToY(rightMatchloader.y, 5000);
  bot.waitUntilDone();

  // Prepare matchloader mech and intake for scoring
  bot.intake.goToStoring();
  bot.matchLoader.extend();

  // Face matchloader to prepare for scoring
  bot.turnToHeading(RED_STATION, 1000);
  bot.waitUntilDone();
  // Move into matchloader to pick up blocks
  bot.moveToX(rightMatchloader.x, 1000, {.maxSpeed = 50});
  bot.waitUntilDone();

  // Give time to yoink the blocks
  bot.tank(10, 10);
  pros::delay(700);
  bot.tank(0, 0);

  const Pose rightLongGoal = {-TILE - DRIVE_LENGTH / 2 + 4, -2 * TILE - 2,
                              RED_STATION};

  // Score 4 blocks on long goal
  bot.moveToPoint(rightLongGoal, 1500,
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

  const Pose rightCenterBallsTarget{
      -TILE - BALL_INNER_DIAM / 2 + DRIVE_WIDTH / 2 - 1, -TILE};
  const Pose leftCenterBallsTarget{rightCenterBallsTarget.x, TILE};
  // Exit long goal
  bot.swingToPoint(rightCenterBallsTarget, lemlib::DriveSide::RIGHT, 1000,
                   {.minSpeed = 48, .earlyExitRange = 30});
  // Scoop up the center balls
  bot.matchLoader.retract();
  bot.moveToPoint(rightCenterBallsTarget, 1000,
                  {.maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 6});
  bot.moveToPoint(leftCenterBallsTarget, 1000,
                  {.maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 6});
  // Wait until facing left center balls
  waitUntil(
      [&] {
        return !bot.isInMotion() ||
               std::abs(lemlib::radToDeg(lemlib::angleError(
                   bot.getPose().angle(leftCenterBallsTarget),
                   ((lemlib::Chassis *)(&bot))->getPose().theta))) < 10;
      },
      100, INT_MAX, true);
  bot.cancelMotion();

  const float middleGoalTargetTheta = (RED_STATION + REFEREE) / 2;
  // Move to be directly in front of middle goal
  bot.moveStraight(
      [&] {
        const float initialTheta = bot.getPose().theta;
        const Pose line{0, 0, middleGoalTargetTheta};
        return [&] {
          const Pose currentPose = bot.getPose();
          const std::optional<float> rayDist = signed_ray_distance(
              currentPose, {currentPose.x, currentPose.y, initialTheta});
          return rayDist.value_or(currentPose.distance(leftCenterBallsTarget));
        };
      },
      1500);
  bot.waitUntilDone();

  // Go into middle goal
  bot.turnToHeading(middleGoalTargetTheta, 1000);
  const Pose middleGoalTarget = Pose::fromPolar(15, middleGoalTargetTheta);
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

  // Face matchloader to prepare for scoring
  bot.turnToHeading(RED_STATION, 1000);
  bot.waitUntilDone();
  // Move into matchloader to pick up blocks
  bot.moveToX(leftMatchloader.x, 1000, {.maxSpeed = 50});
  bot.waitUntilDone();

  // Give time to yoink the blocks
  bot.tank(10, 10);
  pros::delay(700);
  bot.tank(0, 0);

  // Score on left long goal
  const Pose leftLongGoal = {-TILE - DRIVE_LENGTH / 2 + 4, -2 * TILE - 2,
                             RED_STATION};

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