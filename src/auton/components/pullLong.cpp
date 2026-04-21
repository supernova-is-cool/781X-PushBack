#include "auton/components.h"
#include "auton/quadrant.h"
#include "auton/transform.h"
#include "auton/util.h"
#include "dimensions.h"
#include "lemlib/chassis/chassis.hpp"
#include "robot.h"
#include <memory>

using namespace dimensions::all;
using namespace DIR;
using namespace auton::util;

using lemlib::Pose;
using AngDir = lemlib::AngularDirection;
using auton::Quadrant;
using SIDE = Quadrant::SIDE;
void auton::components::pullLong(Quadrant quadrant, AUTON auton) {
  // Programmed from right red perspective
  auton::TransformLockGuard _transform{
      std::make_shared<auton::AllianceTransform>(quadrant.color, COLOR::RED)};
  /** Which side of the goal the bot should be on, to descore */
  const SIDE side = Quadrant::oppositeSide(quadrant.side());
  const bool faceBlue = (side == SIDE::LEFT);

  const float descoreHeading = faceBlue ? BLUE_STATION : RED_STATION;

  const Pose descoreTarget =
      Pose{0, -TILE * 2} + Pose{-DRIVE_LENGTH / 2 - 3, DRIVE_WIDTH / 2 + 2};

  // TODO: Automatically determine lockedSide based on which side would make the
  // bot travel forwards.

  // Turn to face descore target
  bot.swingToPoint(descoreTarget, lemlib::DriveSide::LEFT, 500,
                   {
                       .direction = AngDir::CCW_COUNTERCLOCKWISE,
                       .minSpeed = 32,
                       .earlyExitRange = 15,
                   });
  bot.waitUntilDone();
  // Align beside long goal
  bot.moveToY(descoreTarget.y, 1000,
              {.targetHeading =
                   trigAngleToHeading(bot.getPose().angle(descoreTarget))});
  // Prevent the matchloader from crossing the auton line
  bot.matchLoader.retract();
  // Turn to face the descoring heading
  bot.turnToHeading(descoreHeading, 500,
                    {.minSpeed = 32, .earlyExitRange = 15});
  bot.waitUntilDone();
  // Place descore in tube
  bot.descore.retract();
  // Pull the blocks out of the tube
  bot.moveToX(-TILE * 1.5, 500, {.targetHeading = descoreHeading});
}