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

void auton::components::scoreLong(Quadrant quadrant, AUTON auton) {
  // Programmed from right red perspective
  auton::TransformLockGuard _transform{
      std::make_shared<auton::QuadrantTransform>(quadrant,
                                                 Quadrant::RED_RIGHT)};
  const Pose longGoalOpening{-TILE, -TILE * 2};
  const Pose longGoalTarget =
      longGoalOpening + Pose{-5 + (quadrant.isRight() ? 0.f : 1.f),
                             (quadrant.isRight() ? 1.f : 2.f)};

  // Move into long goal
  bot.moveToPoint(longGoalTarget, 2000,
                  {.forwards = false, .earlyExitRange = 1});
  // Prepare to score
  bot.lift.extend();
  // Wait until near long goal
  waitUntil([] { return bot.getPose().x > -2 * TILE || !bot.isInMotion(); });
  bot.cancelMotion();
  // Move strait towards long goal
  bot.moveToX(longGoalTarget.x, 1000,
              {.earlyExitRange = 1, .targetHeading = RED_STATION});
  bot.waitUntilDone();

  // Score the balls
  bot.lever.scoreAll();
  /** Push the bot into the long goal while scoring */
  const auto defaultLatPID = bot.lateralPID;
  const auto defaultAngPID = bot.angularPID;
  bot.lateralPID.kP *= .5;
  bot.angularPID.kP *= 3;
  bot.lateralPID.kD = 0;
  bot.moveToX(0, 3000, {.targetHeading = RED_STATION});
  // Wait until scoring is done or until timeout is reached
  waitUntil([] { return bot.lever.isFullyScored(); }, 20, 3000);
  bot.cancelMotion();

  bot.lateralPID = defaultLatPID;
  bot.angularPID = defaultAngPID;
  bot.lever.reset();
  bot.intake.stop();
}