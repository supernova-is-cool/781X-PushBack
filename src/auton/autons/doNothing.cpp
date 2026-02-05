#include "auton/autons.h"
#include "lemlib/pose.hpp"
#include "pros/abstract_motor.hpp"
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

void autons::doNothing() {
  printf("yo what's up\n");
  tank(32, 32, 100, 0);
  stop();
}