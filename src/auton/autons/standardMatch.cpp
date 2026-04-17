#include "auton/autons.h"
#include "auton/components.h"
#include "auton/quadrant.h"
#include "auton/transform.h"
#include "auton/util.h"
#include "color.h"
#include "dimensions.h"
#include "robot.h"
#include <memory>

using namespace dimensions::all;
using namespace DIR;
using namespace auton::util;
using namespace auton::components;

using lemlib::Pose;
using AngDir = lemlib::AngularDirection;
using auton::Quadrant;

namespace {
struct StandardMatchAutonConfig {
  Quadrant::SIDE side;
  bool scoreCenter = true;
};
} // namespace

// Programmed from right red perspective
static void standardMatchAuto(StandardMatchAutonConfig config) {
  // Set starting pose
  {
    auton::TransformLockGuard _lockGuard{
        std::make_shared<auton::QuadrantTransform>(
            Quadrant::fromSide(COLOR::RED, config.side),
            Quadrant::fromSide(COLOR::RED, Quadrant::SIDE::RIGHT))};
    const Pose startPose = {-TILE * 2, -TILE + DRIVE_WIDTH / 2 + .5,
                            BLUE_STATION};
    bot.setPose(startPose);
  }
  bot.descore.extend();

  auto qd = Quadrant::fromSide(COLOR::RED, config.side);

  grabCenterBlocks(qd);
  if (config.scoreCenter)
    scoreCenter(qd, 1000);
  matchload(qd, true);
  scoreLong(qd);
  descoreLong(qd, LongDescoreSide::INNER);
}

void autons::rightMiddle() {
  standardMatchAuto({.side = Quadrant::SIDE::RIGHT, .scoreCenter = true});
}

void autons::leftMiddle() {
  standardMatchAuto({.side = Quadrant::SIDE::LEFT, .scoreCenter = true});
}

void autons::rightRush() {
  standardMatchAuto({.side = Quadrant::SIDE::RIGHT, .scoreCenter = false});
}

void autons::leftRush() {
  standardMatchAuto({.side = Quadrant::SIDE::LEFT, .scoreCenter = false});
}