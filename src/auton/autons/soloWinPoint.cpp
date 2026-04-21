#include "auton/autons.h"
#include "auton/components.h"
#include "auton/quadrant.h"
#include "auton/util.h"
#include "dimensions.h"
#include "robot.h"
#include "subsystems/intake.h"

using namespace dimensions::all;
using namespace DIR;
using namespace auton::util;
using namespace auton::components;

using lemlib::Pose;
using AngDir = lemlib::AngularDirection;
using auton::Quadrant;

void autons::soloWinPoint() {
  // === STARTING POSE ===

  // TODO: Determine good sawp starting pose
  const Pose startingPosition = {-2 * TILE + 1.5, -2.5, REFEREE};
  bot.setPose(startingPosition);

  // Push Alliance partner and pick up their preload
  bot.intake.intake();
  bot.moveToY(startingPosition.y + BALL_INNER_DIAM + 2, 2000, {.minSpeed = 48});
  bot.waitUntilDone();
  
  //   bot.lateralPID.kP *= .75;
  //   bot.lateralPID.kD *= 1.1;

  matchload(Quadrant::RED_RIGHT, true, AUTON::SAWP);
  scoreLong(Quadrant::RED_RIGHT, AUTON::SAWP);
  // Exit long goal and face center blocks
  bot.swingToPoint({-TILE, -TILE}, lemlib::DriveSide::RIGHT, 500,
                   {.minSpeed = 32, .earlyExitRange = 20});
  grabCenterBlocks(Quadrant::RED_RIGHT, false, AUTON::SAWP);
  grabCenterBlocks(Quadrant::RED_LEFT, true, AUTON::SAWP);
  scoreCenter(Quadrant::RED_LEFT, 1000, AUTON::SAWP);
  matchload(Quadrant::RED_LEFT, true, AUTON::SAWP);
  scoreLong(Quadrant::RED_LEFT, AUTON::SAWP);

  // Leave Long goal to avoid touching our blocks
  tank(64, 64, 200, 0);
  stop();
}