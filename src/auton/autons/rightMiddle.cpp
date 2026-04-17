#include "auton/autons.h"
#include "auton/components.h"
#include "auton/util.h"
#include "dimensions.h"
#include "robot.h"

using namespace dimensions::all;
using namespace DIR;
using namespace auton::util;
using namespace auton::components;

using lemlib::Pose;
using AngDir = lemlib::AngularDirection;
using auton::Quadrant;

// Programmed from right red perspective
void autons::rightMiddle() {
  // --- Starting Pose ---
  const Pose startPose = {-TILE * 2, -TILE + DRIVE_WIDTH / 2 + .5,
                          BLUE_STATION};
  bot.setPose(startPose);
  bot.descore.extend();

  grabCenterBlocks(Quadrant::RED_RIGHT);
  scoreCenter(Quadrant::RED_RIGHT, 1000);
  matchload(Quadrant::RED_RIGHT, true);
  scoreLong(Quadrant::RED_RIGHT);
  descoreLong(Quadrant::RED_RIGHT, LongDescoreSide::INNER);
}