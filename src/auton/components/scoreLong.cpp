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
                             (quadrant.isRight() ? 0.f : 2.f)};

  // Move into long goal
  bot.moveToPoint(longGoalTarget, 2000,
                  {.forwards = false, .earlyExitRange = 1});
  // Prepare to score
  bot.lift.extend();
  bot.waitUntilDone();

  // TODO: Consider pushing the bot into the goal while scoring, in order to
  // improve alignment

  // Score the balls
  bot.lever.scoreAll();
  /** Push the bot into the long goal while scoring */
  tank(-36, -36, 0, 0);
  // Wait until scoring is done or until timeout is reached
  waitUntil([] { return bot.lever.isFullyScored(); }, 20, 2000);
  bot.lever.reset();
  bot.intake.stop();
}