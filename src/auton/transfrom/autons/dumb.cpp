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

void dumbSkills(){
    bot.intake.disableFiltering();
    bot.intake.goToScoring();
    bot.setPose({0, 0, 0});
    bot.moveToPoint({0, -15, 0}, 1000, {.forwards = false});
    bot.waitUntilDone();
    pros::delay(200);
    bot.tank(127, 127, false);
    pros::delay(1600);
    bot.tank(0, 0);
}