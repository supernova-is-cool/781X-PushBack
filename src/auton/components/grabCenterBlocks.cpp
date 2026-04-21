#include "auton/components.h"
#include "auton/transform.h"
#include "auton/util.h"
#include "dimensions.h"
#include "log.h"
#include "robot.h"
#include <memory>

using namespace dimensions::all;
using namespace DIR;
using namespace auton::util;

using lemlib::Pose;
using AngDir = lemlib::AngularDirection;
using auton::Quadrant;

void auton::components::grabCenterBlocks(Quadrant quadrant, bool alignWithGoal,
                                         AUTON auton) {
  // Programmed from right red perspective
  auton::TransformLockGuard _transform{
      std::make_shared<auton::QuadrantTransform>(quadrant,
                                                 Quadrant::RED_RIGHT)};
  log("in grabCenterBlocks: bot.getPose() = {}", bot.getPose());
  const Pose offset = {quadrant.isRight() ? 0.f : -4.f, 0};
  const Pose centerBalls = Pose{-TILE, -TILE} + offset;
  const Pose centerGoalLine = (Pose{0, 0} + offset).withTheta(REFEREE + 45);

  // Turn towards the center balls
  bot.swingToPoint(centerBalls, lemlib::DriveSide::RIGHT, 500,
                   {.minSpeed = 32, .earlyExitRange = 20});
  bot.waitUntilDone();

  // Move towards the center balls and intake them
  if (alignWithGoal) {
    float targetHeading = trigAngleToHeading(bot.getPose().angle(centerBalls));
    // Align to the imaginary line emanating from the center goal
    bot.moveToLine(centerGoalLine, 2000, {.targetHeading = targetHeading});
  } else {
    // Just go to the center balls
    bot.moveToPoint(centerBalls, 2000, {.minSpeed = 32, .earlyExitRange = 3});
  }
  bot.intake.intake();
  // Make sure matchloader is retracted before we push them accidentally with
  // the matchloader mech
  waitUntil(
      [&] {
        return bot.getPose().distance(centerBalls) <
                   MATCHLOADER_DIST_TO_CENTER * 2 ||
               !bot.isInMotion();
      },
      0, 1000);
  bot.matchLoader.retract();
  // Wait until the perfect moment to use ML to capture the blocks
  waitUntil(
      [&] {
        return bot.getPose().distance(centerBalls) <
                   MATCHLOADER_DIST_TO_CENTER ||
               !bot.isInMotion();
      },
      0, 2000);
  bot.matchLoader.extend();

  if (alignWithGoal)
    // Take the time to fully align with the center goal
    bot.waitUntilDone();
  else
    // Rush onto the next step
    bot.cancelMotion();
}