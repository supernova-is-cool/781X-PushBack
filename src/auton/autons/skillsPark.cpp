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

void autons::skillsPark() {
  bot.setAlliance(ALLIANCE::RED);

  /** 60 second timer */
  lemlib::Timer skillsTimer{60 * 1000};
  skillsTimer.resume();

  const Pose startPose{MIN_X + DRIVE_LENGTH / 2,
                       PARK_ZONE_WIDTH / 2 + DRIVE_LENGTH / 2, AUDIENCE};
  // === Clear and park in the park zone
  bot.setPose(startPose);

  // Back up bot
  bot.moveToPoint(startPose + Pose{0, 3}, 1000, {.forwards = false});
  bot.waitUntilDone();

  bot.matchLoader.extend();
  pros::delay(1000);

  // Retract odom to prevent impedance when parking
    //   bot.odomLift.retract();
  // Outtake to clear zone
  bot.intake.goToTOP();

  // Go into park zone
  tank(127, 127, 50, 0);

  // Wait until in park zone to stop
  // waitUntil([] { return bot.getPose().y < 0; }, 0, 1000);
  pros::delay(400);
  stop();
  pros::delay(3000);
  bot.matchLoader.retract();
  return;
  // Align next to park zone
  /** Second alignment point. Fully aligns the robot with park zone. */
  const Pose parkZoneAlignTarget2{MIN_X + (DRIVE_WIDTH / 2 + 2),
                                  PARK_ZONE_WIDTH / 2 + DRIVE_LENGTH / 2 + 8};
  /** First alignment point. Necessary to not hit wall */
  const Pose parkZoneAlignTarget1{MIN_X + (DRIVE_LENGTH / 2 + 2),
                                  parkZoneAlignTarget2.y + 10};
  bot.moveToPoint(parkZoneAlignTarget1, 2000);

  // Once away from long goal, prepare to park
  bot.waitUntil(6);
  bot.matchLoader.retract();
//   bot.lift.retract();
  bot.intake.goToIdle();
  bot.waitUntilDone();

  // Turn to face park zone
  bot.turnToHeading(AUDIENCE, 1000, {.minSpeed = 48, .earlyExitRange = 30});
  bot.waitUntilDone();

  // Fully align
  bot.moveToPoint(parkZoneAlignTarget2, 2000);
  bot.waitUntilDone();
  bot.turnToHeading(AUDIENCE, 1000);
  bot.waitUntilDone();

  // Park in zone
  const Pose parkTarget =
      parkZoneAlignTarget2.withY(-PARK_ZONE_WIDTH + DRIVE_LENGTH / 2 + 1);

  // Go into park zone
  // Wait a bit so that momentum from turn doesn't mess up pod
  tank(96, 96, 250, 0);
  // Retract horizontal pod to prevent impedance when parking
//   bot.odomLift.retract();

  /** Y distance of matchloader wheel to center of robot, when deployed */
  const float MATCHLOADER_WHEEL_OFFSET = 11.5; // in
  /** Y distance of back wheel to center of robot */
  const float BACK_WHEEL_OFFSET = 5.5; // in

  // Wait until matchloader wheel would clear park zone to deploy matchloader
  waitUntil(
      [&] {
        const float deployedMatchLoaderY =
            bot.getPose().y - MATCHLOADER_WHEEL_OFFSET;
        return deployedMatchLoaderY <
               PARK_ZONE_WIDTH / 2 +
                   2 /* Deploy a little early to give time for deployment */;
      },
      1000);
  // Use matchloader mech to clear blocks in park zone
  bot.matchLoader.extend();
  // Pickup any blocks not pushed out by matchloader
  bot.intake.goToStoring();
  // Wait until back wheel is in park zone to stop, or until skills time runs
  // out
  waitUntil(
      [&] {
        const float backWheelY = bot.getPose().y + BACK_WHEEL_OFFSET;
        return backWheelY < PARK_ZONE_WIDTH / 2 ||
               skillsTimer.getTimeLeft() < 100;
      },
      0, 4000);
  // Retract matchloader after parking
  bot.matchLoader.retract();
  stop();

  // YIPPEE!!
}