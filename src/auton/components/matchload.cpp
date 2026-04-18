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

void auton::components::matchload(Quadrant quadrant, bool onlyMyColor) {
  // Programmed from right red perspective
  auton::TransformLockGuard _transform{
      std::make_shared<auton::QuadrantTransform>(quadrant,
                                                 Quadrant::RED_RIGHT)};
  const Pose matchloader{MIN_X, -TILE * 2};
  const Pose matchloaderTarget =
      matchloader + Pose{MATCHLOADER_DIST_TO_CENTER + 1, 0};

  // TODO: Investigate using a swing turn for speed
  // Align in front of the matchloader
  bot.moveToY(matchloader.y, 2000);
  // Quickly turn to the matchloader
  bot.turnToHeading(RED_STATION,
                    1000 /* , {.minSpeed = 32, .earlyExitRange = 20} */);

  // Prepare to matchload
  bot.intake.intake();
  bot.matchLoader.extend();
  // Optimize intaking
  bot.intakeLift.retract();
  bot.lift.retract();

  bot.waitUntilDone();

  // Push matchloader mech into the matchloader
  bot.moveToX(matchloaderTarget.x, 2000,
              {/* .minSpeed = 32, */ .targetHeading = RED_STATION});

  constexpr float emptyDistance = 380;
  /** The minimum distance measurement that indicates 3 balls are present */
  constexpr float threeBallDistance = 130;
  constexpr float ballDistance = (threeBallDistance - emptyDistance) / 3;

  // Wait until the ball counter measures 3 balls or until 2 seconds have passed
  waitUntil([&] { return bot.ballCounter.get() < threeBallDistance; }, 20,
            2000);
  if (!onlyMyColor) {
    // If loading all 6 blocks, give a little more time to intake them
    pros::delay(1000);
  }
  bot.cancelMotion();
  bot.leftLaser().resetY();
}