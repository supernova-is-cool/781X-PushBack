#include "auton/components.h"
#include "auton/transform.h"
#include "auton/util.h"
#include "dimensions.h"
#include "robot.h"
#include <memory>

using namespace dimensions::all;
using namespace DIR;
using namespace auton::util;

using lemlib::Pose;
using AngDir = lemlib::AngularDirection;
using auton::Quadrant;

void auton::components::scoreCenter(Quadrant quadrant,
                                    std::size_t score_duration) {
  // Programmed from right red perspective
  auton::TransformLockGuard _transform{
      std::make_shared<auton::QuadrantTransform>(quadrant,
                                                 Quadrant::RED_RIGHT)};
  /** Whether the bot should score on the middle goal (middle as in height) */
  const bool isMiddle = quadrant.isLeft();
  const Pose offset = {isMiddle ? -4.f : 0.f, 0};
  const Pose center = Pose{0, 0} + offset;
  // Face the center goal
  bot.turnToPoint(center, 500, {.forwards = !isMiddle});
  bot.waitUntilDone();

  const float middleScoringDist = 4;
  const float lowerScoringDist = 5;
  /** Distance from center of bot to the lip of the goal. */
  const float scoringDist = (isMiddle) ? middleScoringDist : lowerScoringDist;

  const Pose scoreTargetLine =
      (center +
       Pose::fromPolar(-scoringDist - CENTER_GOAL_RADIUS, REFEREE + 45))
          .withTheta(REFEREE - 45);
  const float targetHeading =
      trigAngleToHeading(bot.getPose().angle(center)) + (isMiddle ? 180 : 0);

  // Move towards the center goal to score
  bot.moveToLine(scoreTargetLine, 1500, {.targetHeading = targetHeading});

  // Retract matchloader to prevent blocking the center goal
  bot.matchLoader.retract();
  if (isMiddle) {
    // Do not raise lift for middle goal
    bot.lift.retract();
  } else {
    // Raise lift to help with outtaking
    bot.lift.extend();
  }
  bot.waitUntilDone();

  if (isMiddle) {
    bot.intake.intake();
    // Just score a few, but slowly to prevent them from being launched out the
    // other side of the goal
    bot.lever.scoreAllSlow();
    lemlib::Timer timeout{score_duration};
    timeout.resume();
    waitUntil(
        [&] { return bot.lever.blocksRemaining() <= 1 || timeout.isDone(); });
    // Reset lever and outtake to prevent jamming
    bot.lever.reset();
    bot.intake.outtake();
  } else {
    // Slow outtake to score blocks without sending them out the other end
    bot.intake.slowOuttake();
    // Pull intake up to score on low goal
    bot.intakeLift.extend();
    pros::delay(score_duration);
    // Hold onto remaining blocks
    bot.intake.intake();
  }

  // Exit goal
  int directionSignum = (isMiddle) ? 1.f : -1.f;
  tank(directionSignum * 32, directionSignum * 32, 150, 0);
  bot.intake.intake();
}