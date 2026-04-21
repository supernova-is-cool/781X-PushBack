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

  /** The maximum distance measurement that indicates there are 4 or more balls
   */
  constexpr float maxBallDistance = 100;

  // Wait until balls in matchloader are detected
  waitUntil([&] { return bot.mlSensor.get() < maxBallDistance; }, 50, 2000,
            true);
  // Wait until 3 balls are taken from the matchloader, such that the sensor
  // does not see a ball
  waitUntil([&] { return bot.mlSensor.get() > maxBallDistance; }, 50, 2000,
            true);
  if (!onlyMyColor) {
    // If loading all 6 blocks, give a little more time to intake them
    pros::delay(1000);
  }
  bot.cancelMotion();
  bot.leftLaser().resetY();
}