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
void auton::components::pushLong(Quadrant quadrant, LongDescoreSide oi_side, AUTON auton) {
  // Programmed from right red perspective
  auton::TransformLockGuard _transform{
      std::make_shared<auton::AllianceTransform>(quadrant.color, COLOR::RED)};
  /** Which side of the goal the bot should be on, to descore */
  const SIDE side = (oi_side == LongDescoreSide::OUTER)
                        ? quadrant.side()
                        : Quadrant::oppositeSide(quadrant.side());
  const bool faceBlue = (side == SIDE::LEFT);

  const float descoreHeading = faceBlue ? BLUE_STATION : RED_STATION;

  // Prepare to descore
  bot.descore.retract();
  // Prevent the matchloader from crossing the auton line
  bot.matchLoader.retract();
  switch (side) {
  case SIDE::LEFT: {
    // Exit long goal
    tank(32, 32, 100, 0);
    // Align descore with long goal
    bot.swingToHeading(descoreHeading, lemlib::DriveSide::RIGHT, 500,
                       {
                           .direction = AngDir::CW_CLOCKWISE,
                           .minSpeed = 32,
                           .earlyExitRange = 20,
                       });
    break;
  }
  case SIDE::RIGHT: {
    const Pose longGoalPose = bot.getPose();
    const float descoreY = longGoalPose.y - DRIVE_WIDTH / 2 - 2;
    const float alignHeading = descoreHeading + 30;
    bot.intake.outtake();
    // Align descore with long goal
    bot.swingToHeading(AUDIENCE + 45, lemlib::DriveSide::LEFT, 500,
                       {
                           .direction = AngDir::CCW_COUNTERCLOCKWISE,
                           .minSpeed = 32,
                           .earlyExitRange = 15,
                       });
    bot.swingToHeading(alignHeading, lemlib::DriveSide::RIGHT, 500,
                       {
                           .direction = AngDir::CW_CLOCKWISE,
                           .minSpeed = 32,
                           .earlyExitRange = 15,
                       });
    bot.moveToY(
        descoreY, 500,
        {.minSpeed = 32, .earlyExitRange = 1, .targetHeading = alignHeading});
    break;
  }
  }
  // Push blocks by moving in a straight line
  const float pushBlocksTargetX =
      0 - DRIVE_LENGTH / 2 + (side == SIDE::LEFT ? -2 : 0);
  bot.moveToX(pushBlocksTargetX, 2000,
              {.maxSpeed = 48, .targetHeading = descoreHeading});
  bot.waitUntilDone();

  // Repeatedly run moveToX to hold position against pushes
  // (When auton stage ends, this will also end)
  const Pose holdPose = [&] {
    const Pose currentPose = bot.getPose();
    return currentPose
        .withTheta(descoreHeading)
        // Do not go further than target and risk crossing auton line
        .withX(std::max(currentPose.x, pushBlocksTargetX));
  }();
  bot.lateralLargeExit.range = 0;
  bot.lateralSmallExit.range = 0;
  while (true) {
    bot.moveToX(holdPose.x, 1000, {.targetHeading = descoreHeading}, false);
  }
}