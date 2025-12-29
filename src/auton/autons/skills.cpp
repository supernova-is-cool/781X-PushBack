#include "auton/autons.h"
#include "auton/transform.h"
#include "color.h"
#include "lemlib/pose.hpp"
#include "lemlib/timer.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include <cstdio>
#include <print>

#include "auton/autons.h"
#include "auton/util.h"
#include "dimensions.h"
#include "lemlib/chassis/chassis.hpp"

using namespace dimensions::all;
using namespace DIR;
using namespace auton::util;
using namespace auton;

using lemlib::Pose;
using AngDir = lemlib::AngularDirection;

/** Same as solo awp */
const Pose startPose{-2 * TILE - DRIVE_WIDTH + 5, -TILE + DRIVE_LENGTH / 2 + 1,
                     AUDIENCE};

const Pose redAudienceMatchLoader = {MIN_X + DRIVE_LENGTH / 2 + 4, -2 * TILE,
                                     RED_STATION};
/** Target to align in front of matchloader */
const Pose alignWithRedAudienceMLTarget =
    startPose.withY(redAudienceMatchLoader.y);

/**
 * @brief Collect balls from the matchloader at the given target pose.
 * Assumes the robot starts at alignment target, facing matchloader.
 */
void collectMatchLoader(Pose matchLoaderTarget) {
  // Extend match loader
  bot.matchLoader.extend();

  // Go towards matchloader slowly
  bot.moveToPoint(matchLoaderTarget, 1100, {.maxSpeed = 45});
  bot.waitUntilDone();

  // Push robot into matchloader for a bit longer to pick up balls
  tank(15, 15, 500, 0);
}

/**
 * @brief Since the two long goal filling parts are symmetrical, this does one
 * of the long goals. Scores balls from both matchloaders into the red side of
 * the long goal.
 *
 * Assumes that the bot starts at red matchloader, whose sign matches the input
 * and that said matchloader is collected. Ends facing into long goal.
 *
 * @param sign  Positive will do long goal closest to referee
 *              Negative will do long goal closest to audience
 */
void fillLongGoal(SignTransform::SIGN sign) {
  // Change coordinate system based on sign of quadrant.
  // Programmed relative to negative/audience side
  auto signTransform =
      std::make_shared<SignTransform>(sign, SignTransform::AUDIENCE);
  bot.setTransform(signTransform);

  /** Red inlet of long goal */
  const Pose redLongGoalScoringTarget = {-TILE - DRIVE_LENGTH / 2 + 4,
                                         -2 * TILE - 1, RED_STATION};
  // Prepare to score
  bot.lift.extend();
  // Back into long goal
  bot.moveToPoint(redLongGoalScoringTarget, 1500,
                  {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
  bot.waitUntilDone();

  // Continuously push into long goal to ensure alignment while scoring
  tank(-10, -10, 0);
  bot.intake.goToScoring();
  // Give time to score
  pros::delay(1500);

  /** What to reset odom to after scoring in long goal. You must specify theta
   * yourself. */
  const Pose longGoalRedSideReset = {-26, -51};
  // Odom reset on goal
  bot.setPose(longGoalRedSideReset.withTheta(bot.getPose().theta));

  /** Signum of y offset from goal, for descore to be pointing away from goal.
   * Works for either sign. */
  const int awayFromGoalSignum = [&] {
    enum class BOT_SIDE {
      LEFT,
      RIGHT,
    };

    /** Descore is on the left side of the robot */
    constexpr BOT_SIDE DESCORE_SIDE = BOT_SIDE::LEFT;
    /** The side of the field the descore faces, when bot is facing towards blue
     * alliance station. */
    constexpr SignTransform::SIGN DESCORE_SIGN =
        (DESCORE_SIDE == BOT_SIDE::LEFT) ? SignTransform::SIGN::REFEREE
                                         : SignTransform::SIGN::AUDIENCE;

    /** Signum of y offset from goal, for descore to be pointing away from goal.
     * Works only when sign is audience. */
    constexpr int awayFromGoalSignumForAudience =
        (DESCORE_SIGN == SignTransform::SIGN::AUDIENCE) ? -1 : 1;
    return signTransform->transformPose({0, awayFromGoalSignumForAudience}).y;
  }();

  AllianceTransform redToBlueTransform{ALLIANCE::BLUE, ALLIANCE::RED};

  constexpr float longGoalWidth = 6 /* in */;

  const Pose blueMatchLoader =
      redToBlueTransform.transformPose(redAudienceMatchLoader);
  const Pose alignWithBlueMLTarget =
      redToBlueTransform.transformPose(alignWithRedAudienceMLTarget);
  /** Blue inlet of long goal */
  const Pose blueLongGoalScoringTarget =
      redToBlueTransform.transformPose(redLongGoalScoringTarget);

  /** Past long goal on wall side, faces towards alignWithBlueMLTarget.
   * Makes sure descore faces away from long goal */
  const Pose pastBlueLongGoalTarget = [&] -> Pose {
    const Pose pastLongGoalTargetPoint =
        blueLongGoalScoringTarget +
        Pose{DRIVE_LENGTH / 2,
             awayFromGoalSignum * (longGoalWidth / 2 + DRIVE_WIDTH / 2)};
    return pastLongGoalTargetPoint.withTheta(
        trigAngleToHeading(pastLongGoalTargetPoint.angle(blueMatchLoader)));
  }();

  // Get out of long goal quickly by swinging towards wall side
  bot.swingToPoint(
      pastBlueLongGoalTarget, lemlib::DriveSide::LEFT, 500,
      {.direction = pastBlueLongGoalTarget.y < redLongGoalScoringTarget.y
                        ? AngDir::CCW_COUNTERCLOCKWISE
                        : AngDir::CW_CLOCKWISE,
       .minSpeed = 96,
       .earlyExitRange = 45});
  bot.waitUntilDone();
  // Prevent descore from hitting wall
  bot.descore.extend();

  // Stop intake while going to next matchloader
  bot.intake.goToIdle();

  // Debug print
  std::print("pastLongGoalTarget: {}", pastBlueLongGoalTarget);

  // Move past long goal
  bot.moveToPose(pastBlueLongGoalTarget, 3000,
                 {
                     .lead = .3,
                     // TODO: increase
                     .maxSpeed = 96,
                     .minSpeed = 32,
                     .earlyExitRange = 6,
                 });

  // Wait until either past long goal x or motion done
  waitUntil(
      [&] {
        return bot.getPose().x < blueLongGoalScoringTarget.x ||
               !bot.isInMotion();
      },
      3000);
  bot.cancelMotion();

  // Prepare for match loading
  bot.matchLoader.extend();
  bot.intake.goToStoring();

  // Align with blue matchloader
  bot.moveToPoint(alignWithBlueMLTarget, 200);
  bot.waitUntilDone();

  // Turn towards matchloader
  bot.turnToHeading(BLUE_STATION, 1000);
  bot.waitUntilDone();

  // Collect from blue matchloader
  collectMatchLoader(blueMatchLoader);

  // Go around long goal, while keeping descore not facing long goal
  bot.moveToPoint(pastBlueLongGoalTarget, 500,
                  // TODO: increase speed
                  {
                      .forwards = false,
                      .minSpeed = 32,
                      .earlyExitRange = 6,
                  });

  // Position bot beside long goal to swing into scoring position
  const Pose besideRedLongGoalTarget =
      (redLongGoalScoringTarget + Pose{0, awayFromGoalSignum * DRIVE_WIDTH});
  bot.moveToPoint(besideRedLongGoalTarget, 3000,
                  // TODO: increase speed
                  {.forwards = false, .maxSpeed = 96});
  bot.waitUntilDone();

  // Swing into long goal
  bot.swingToHeading(
      RED_STATION, lemlib::DriveSide::LEFT, 500,
      {
          .direction = besideRedLongGoalTarget.y < redLongGoalScoringTarget.y
                           ? AngDir::CW_CLOCKWISE
                           : AngDir::CCW_COUNTERCLOCKWISE,
      });
  bot.waitUntilDone();

  // Continuously push into long goal to ensure alignment while scoring
  tank(-10, -10, 0);
  bot.intake.goToScoring();
  // Give time to score
  pros::delay(1500);

  // Odom reset on goal
  bot.setPose(longGoalRedSideReset.withTheta(bot.getPose().theta));
}

void autons::skills() {
  bot.setAlliance(ALLIANCE::RED);
  bot.setPose(startPose);

  /** 60 second timer */
  lemlib::Timer skillsTimer{60 * 1000};
  skillsTimer.resume();

  // Deploy hood
  bot.intake.goToOutaking();

  // Position in front of matchloader
  bot.moveToPoint(alignWithRedAudienceMLTarget, 1600);
  // Finish deploying hood, prepare to intake matchloader
  bot.waitUntil(2);
  bot.intake.goToStoring();
  bot.matchLoader.extend();
  bot.waitUntilDone();

  // Turn towards matchloader cleanly
  bot.turnToHeading(RED_STATION, 1500);
  bot.waitUntilDone();

  // Collect from red audience matchloader
  collectMatchLoader(redAudienceMatchLoader);

  // Fill audience side long goal
  fillLongGoal(SignTransform::SIGN::AUDIENCE);

  /** Transforms poses from audience to referee side */
  SignTransform negToPosTransform{SignTransform::SIGN::REFEREE,
                                  SignTransform::SIGN::AUDIENCE};

  const Pose redRefereeMatchLoader =
      negToPosTransform.transformPose(redAudienceMatchLoader);

  /** Distance between wall and barrier parallel to y axis */
  const float PARK_ZONE_DEPTH = 17 /* in */;

  /** Distance between parallel park barriers */
  const float PARK_ZONE_WIDTH = 18 /* in */;

  const Pose aroundParkZoneTarget{MIN_X + PARK_ZONE_DEPTH + DRIVE_WIDTH / 2 + 3,
                                  0};
  // Get out of long goal
  bot.swingToPoint(aroundParkZoneTarget, lemlib::DriveSide::RIGHT, 1000,
                   {
                       .direction = AngDir::CW_CLOCKWISE,
                       .minSpeed = 96,
                       .earlyExitRange = 30,
                   });
  // Go around park zone
  bot.moveToPoint(aroundParkZoneTarget, 2000,
                  // TODO: increase speed
                  {.minSpeed = 32, .earlyExitRange = 6});
  // Wait until past park zone y or motion done
  waitUntil([&] {
    return bot.getPose().y > aroundParkZoneTarget.y || !bot.isInMotion();
  });
  bot.cancelMotion();

  // Position in front of referee matchloader
  const Pose alignWithRedRefereeMLTarget =
      aroundParkZoneTarget.withY(redRefereeMatchLoader.y);
  bot.moveToPoint(alignWithRedRefereeMLTarget, 1600);
  bot.waitUntilDone();

  // Turn towards matchloader cleanly
  bot.turnToHeading(RED_STATION, 1500);
  bot.waitUntilDone();

  // Collect from red referee matchloader
  collectMatchLoader(redRefereeMatchLoader);

  // Fill referee side long goal
  fillLongGoal(SignTransform::SIGN::REFEREE);

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