#include "./common.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/util.hpp"
#include "robot.h"
#include <cfloat>

using namespace lemlib;

void Robot::moveToX(float targetX, int timeout, MoveStraightParams params,
                    bool async) {
  this->moveToLine({targetX, 0, 0}, timeout, params, async);
}
void Robot::moveToY(float targetY, int timeout, MoveStraightParams params,
                    bool async) {
  this->moveToLine(lemlib::Pose(0, targetY, 90), timeout, params, async);
}
static void errorSink(const std::string &msg) { infoSink()->error("{}", msg); }

void Robot::moveToLine(lemlib::Pose line, int timeout,
                       MoveStraightParams params, bool async) {
  const Pose realLine = m_transform->transformPose(line);
  /** Line pose with theta in standard radians */
  const Pose radianLine = realLine.withTheta(degToRad(90 - realLine.theta));
  auto errorFuncFactory = MoveToLineErrorFactory(
      NlPose(radianLine), [this]() { return Chassis::getPose(true, true); },
      errorSink);
  this->moveStraight(errorFuncFactory, timeout, params, async);
}

void Robot::moveDistance(float distance, int timeout, MoveStraightParams params,
                         bool async) {
  auto errorFuncFactory = [this, distance](float _targetHeading) {
    const Pose startPose = this->getPose();
    return [this, distance, startPose]() {
      const Pose currPose = this->getPose();
      const float distToCurr = startPose.distance(currPose);
      const float angleToCurr = startPose.angle(currPose);
      const float currDistance = distToCurr * std::cos(angleToCurr);
      return distance - currDistance;
    };
  };
  this->moveStraight(errorFuncFactory, timeout, params, async);
}

void Robot::moveStraight(MoveStraightErrorFuncFactory errorFuncFactory,
                         int timeout, MoveStraightParams params, bool async) {
  this->requestMotionStart();
  // were all motions cancelled?
  if (!this->motionRunning)
    return;
  // if the function is async, run it in a new task
  if (async) {
    pros::Task task([&]() {
      this->moveStraight(errorFuncFactory, timeout, params, false);
    });
    this->endMotion();
    pros::delay(10); // delay to give the task time to start
    return;
  }

  // reset PIDs and exit conditions
  lateralPID.reset();
  lateralLargeExit.reset();
  lateralSmallExit.reset();
  angularPID.reset();

  // initialize vars used between iterations
  Pose lastPose = Chassis::getPose(true, true);
  distTraveled = 0;
  Timer timer(timeout);
  bool close = false;
  float prevLateralOut = 0; // previous lateral power
  float prevAngularOut = 0; // previous angular power
  const int compState = pros::competition::get_status();
  std::optional<bool> prevSide = std::nullopt;

  // We will try to maintain this theta throughout the motion
  const float targetTheta = degToRad(
      90 - params.targetHeading.value_or(Chassis::getPose(false, false).theta));
  const auto errorFunc = errorFuncFactory(targetTheta);

  // main loop
  while (!timer.isDone() &&
         ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()) ||
          !close) &&
         this->motionRunning) {
    // update position
    const Pose currPose = Chassis::getPose(true, true);

    // update distance traveled
    distTraveled += currPose.distance(lastPose);
    lastPose = currPose;

    // calculate distance to the target point
    const float linearError = errorFunc();

    // check if the robot is close enough to the target to start settling
    if (linearError < 7.5 && close == false) {
      close = true;
      params.maxSpeed = fmax(fabs(prevLateralOut), 60);
    }

    // motion chaining
    const bool side = linearError > 0;
    if (prevSide == std::nullopt)
      prevSide = side;
    const bool sameSide = side == prevSide;
    // exit if close
    if (!sameSide && params.minSpeed != 0)
      break;
    prevSide = side;

    // calculate error
    const float angularError = angleError(currPose.theta, targetTheta);

    // update exit conditions
    lateralSmallExit.update(linearError);
    lateralLargeExit.update(linearError);

    // get output from PIDs
    float lateralOut = lateralPID.update(linearError);
    float angularOut = angularPID.update(radToDeg(angularError));
    // if (close)
    //   angularOut = 0;

    // apply restrictions on angular speed
    angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);
    angularOut = slew(angularOut, prevAngularOut, angularSettings.slew);

    // apply restrictions on lateral speed
    lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);
    // constrain lateral output by max accel
    // but not for decelerating, since that would interfere with settling
    if (!close)
      lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

    // constrain lateral output by the minimum speed
    lateralOut = sgn(lateralOut) *
                 std::max(std::fabs(lateralOut), std::fabs(params.minSpeed));

    // update previous output
    prevAngularOut = angularOut;
    prevLateralOut = lateralOut;

    infoSink()->debug("Angular Out: {}, Lateral Out: {}, Lin Err: {}, Ang Err: "
                      "{}, \n\t Pose: {}",
                      angularOut, lateralOut, linearError, angularError,
                      currPose);

    // ratio the speeds to respect the max speed
    float leftPower = lateralOut + angularOut;
    float rightPower = lateralOut - angularOut;
    const float ratio =
        std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
    if (ratio > 1) {
      leftPower /= ratio;
      rightPower /= ratio;
    }

    // move the drivetrain
    drivetrain.leftMotors->move(leftPower);
    drivetrain.rightMotors->move(rightPower);

    // delay to save resources
    pros::delay(10);
  }

  // stop the drivetrain
  drivetrain.leftMotors->move(0);
  drivetrain.rightMotors->move(0);
  // set distTraveled to -1 to indicate that the function has finished
  distTraveled = -1;
  this->endMotion();
}