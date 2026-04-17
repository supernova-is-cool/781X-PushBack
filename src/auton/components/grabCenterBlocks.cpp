#include "auton/components.h"
#include "auton/transform.h"
#include "auton/util.h"
#include "dimensions.h"
#include "robot.h"
#include <memory>

using namespace dimensions::all;
using namespace DIR;
using namespace auton::util;

using lemlib::Pose;
using AngDir = lemlib::AngularDirection;
using auton::Quadrant;

void auton::components::grabCenterBlocks(Quadrant quadrant) {
  // Programmed from right red perspective
  auton::TransformLockGuard _transform{
      std::make_shared<auton::QuadrantTransform>(quadrant,
                                                 Quadrant::RED_RIGHT)};
  const Pose centerBalls{-TILE, -TILE};
  const Pose centerGoalLine{0, 0, REFEREE + 45};

  // Turn towards the center balls
  bot.swingToPoint(centerBalls, lemlib::DriveSide::RIGHT, 500,
                   {.minSpeed = 32, .earlyExitRange = 20});
  bot.waitUntilDone();
  // Move towards the center balls and intake them
  bot.moveToLine(
      centerGoalLine, 2000,
      {.targetHeading = trigAngleToHeading(bot.getPose().angle(centerBalls))});
  bot.intake.intake();
  // Wait until the perfect moment to use ML to capture the blocks
  waitUntil(
      [&] {
        return bot.getPose().distance(centerBalls) <
                   MATCHLOADER_DIST_TO_CENTER ||
               !bot.isInMotion();
      },
      0, 2000);
  bot.matchLoader.extend();
  bot.waitUntilDone();

  // Next step
}