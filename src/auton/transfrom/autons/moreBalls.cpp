#include "auton/autons.h"
#include "lemlib/pose.hpp"
#include "robot.h"
#include <cstdio>

#include "auton/autons.h"
#include "auton/util.h"
#include "color.h"
#include "dimensions.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include "subsystems/intake.h"

using namespace dimensions::all;
using namespace DIR;
using namespace auton::util;

using lemlib::Pose;
using AngDir = lemlib::AngularDirection;

void moreBalls() {

  const Pose theBalls = {-TILE, -TILE};
  const Pose matchLoader = {MIN_X + DRIVE_LENGTH / 2 + 4, -2 * TILE,
                            RED_STATION};
  const Pose longGoal = {-TILE - DRIVE_LENGTH / 2 + 4, -2 * TILE - 1,
                         RED_STATION};
}