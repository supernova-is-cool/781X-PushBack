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

static void soloWinPointForSkills() {
  // === STARTING POSE ===
  const Pose startingPosition = {-2 * TILE - DRIVE_WIDTH + 5,
                                 -TILE + DRIVE_LENGTH / 2 + 1, 180};
  bot.setPose(startingPosition);
  // pros::delay(75);

  bot.intake.goToOutaking();

  // === MATCH LOADER INTAKE #1 ===
  const Pose matchLoader = {MIN_X + DRIVE_LENGTH / 2 + 4, -2 * TILE,
                            RED_STATION};

  // Drop slightly first (consistency)
  bot.moveToPoint({-2 * TILE - DRIVE_WIDTH + 8, -2 * TILE}, 1600);
  bot.waitUntil(2);
  bot.intake.goToStoring();
  bot.waitUntilDone();
  // pros::delay(100);
  bot.matchLoader.extend();

  // Turn into ML cleanly
  bot.turnToHeading(RED_STATION, 1500);
  bot.waitUntilDone();
  // pros::delay(80);

  // bot.intake.goToStoring();

  bot.moveToPoint(matchLoader, 1100, {.maxSpeed = 45});
  bot.waitUntilDone();

  // Feed cubes
  bot.tank(15, 15);
  pros::delay(500);
  bot.tank(0, 0);

  // === LONG GOAL SCORING ===
  const Pose longGoal = {-TILE - DRIVE_LENGTH / 2 + 4, -2 * TILE - 1,
                         RED_STATION};

  bot.lift.extend();

  bot.moveToPoint(longGoal, 1500,
                  {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
  bot.waitUntilDone();

  // Smooth outaking sequence
  bot.tank(-10, -10);
  /*
  bot.intake.goToScoring();
  pros::delay(200);
  bot.intake.goToOutaking();
  pros::delay(200);
  */
  bot.intake.goToScoring();
  pros::delay(1400);
  bot.tank(0, 0);

  // === ODOM RESET BEFORE CLOSE STACK ===

  // (Your robot was drifting badly here → keep θ, reset x/y only)
  Pose poseNow = bot.getPose();
  // -28 and -52 if goofy?
  bot.setPose({-26, -51, poseNow.theta});
  pros::delay(100);

  // === CLOSE STACK ===
  const Pose closeStack = {-TILE, -TILE};

  // right min = 60 , early exit = 10
  bot.turnToPoint(closeStack, 1000);
  bot.waitUntil(3); // early action window for lift unload

  bot.lift.retract();
  bot.matchLoader.retract();

  bot.waitUntilDone();
  bot.intake.goToStoring();

  // Controlled slow intake to stack
  bot.moveToPoint(closeStack, 2000, {.maxSpeed = 75, .minSpeed = 30});
  bot.waitUntilDone();

  // === FAR STACK ===
  const Pose farStack = {-TILE, TILE, 0};

  bot.moveToPose(farStack, 3000, {.maxSpeed = 90, .minSpeed = 40});
  bot.waitUntil(38);

  bot.matchLoader.extend();

  // === MIDDLE GOAL SCORING ===
  const Pose middleGoal = {-0.62 * TILE, 0.64 * TILE, -50};

  // bot.swingToHeading(-50, lemlib::DriveSide::LEFT ,1200);
  bot.waitUntilDone();
  // pros::delay(60);

  bot.moveToPose(middleGoal, 1200, {.forwards = false, .lead = 0.4});
  bot.waitUntilDone();

  // Score middle goal
  bot.intake.goToScoring();
  /*
  pros::delay(200);
  bot.intake.goToOutaking();
  pros::delay(200);
  bot.intake.goToScoring();
  */
  pros::delay(1200);
  bot.intake.goToIdle();
  pros::delay(60);

  const Pose farMatchLoader = {MIN_X + DRIVE_LENGTH / 2 + 3, 2 * TILE + 2,
                               RED_STATION};

  // Move near match loader staging point
  bot.moveToPoint({-2 * TILE, 2 * TILE + 4}, 1200, {.maxSpeed = 90});
  bot.waitUntilDone();
  // pros::delay(50);

  // Controlled swing into match loader line
  bot.turnToHeading(RED_STATION, 1000);
  bot.waitUntilDone();
  pros::delay(60);

  // Intake phase
  bot.intake.goToStoring();

  bot.moveToPoint(farMatchLoader, 1100, {.maxSpeed = 45, .minSpeed = 15});
  bot.waitUntilDone();

  bot.tank(15, 15);
  pros::delay(500);
  bot.tank(0, 0);

  const Pose farLongGoal = {-TILE - DRIVE_LENGTH / 2 + 4, 2 * TILE + 3,
                            RED_STATION};

  bot.lift.extend();
  bot.moveToPoint(farLongGoal, 1500, {.forwards = false, .maxSpeed = 70});
  bot.waitUntilDone();
  bot.tank(-10, -10);
  bot.intake.goToScoring();
  /*
  bot.intake.goToScoring();
  pros::delay(200);
  bot.intake.goToOutaking();
  pros::delay(200);
  bot.intake.goToScoring();
  */
  pros::delay(1400);
  bot.tank(0, 0);
}

void autons::skillsAWP() {
  bot.setAlliance(ALLIANCE::RED);

  /** 60 second timer */
  lemlib::Timer skillsTimer{60 * 1000};
  skillsTimer.resume();

  soloWinPointForSkills();

  // === Clear and park in the park zone

  // Align next to park zone
  const Pose parkZoneAlignTarget{MIN_X + (DRIVE_WIDTH / 2 + 2), 0};
  bot.moveToPoint(parkZoneAlignTarget, 2000);

  // Once away from long goal, prepare to park
  bot.waitUntil(6);
  bot.matchLoader.retract();
  bot.lift.retract();
  bot.intake.goToIdle();
  bot.waitUntilDone();

  // Turn to face park zone
  bot.turnToHeading(AUDIENCE, 1000);
  bot.waitUntilDone();

  // Park in zone
  const Pose parkTarget =
      parkZoneAlignTarget.withY(-PARK_ZONE_WIDTH + DRIVE_LENGTH / 2 + 1);

  // Go into park zone
  tank(96, 96, 0, 0);
  // Retract horizontal pod to prevent impedance when parking
  // Wait a bit so that momentum from turn doesn't mess up pod
  pros::delay(250);
  bot.odomLift.retract();

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
      2000);
  // Retract matchloader after parking
  bot.matchLoader.retract();
  stop();

  // YIPPEE!!
}