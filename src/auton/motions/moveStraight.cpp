#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/util.hpp"
#include "robot.h"
#include <cfloat>

using namespace lemlib;

void Robot::moveToX(float targetX, int timeout, MoveStraightParams params,
                    bool async) {
  this->moveToLine({targetX, 0, 90}, timeout, params, async);
}
void Robot::moveToY(float targetY, int timeout, MoveStraightParams params,
                    bool async) {
  this->moveToLine(lemlib::Pose(0, targetY, 0), timeout, params, async);
}

struct Vector {
  float x;
  float y;

  static Vector between(const Pose &from, const Pose &to) {
    return Vector{to.x - from.x, to.y - from.y};
  }
  static Vector fromAngle(float angle) {
    return Vector(cos(angle), sin(angle));
  }

  float cross(const Vector &other) const { return x * other.y - y * other.x; }
};

void Robot::moveToLine(lemlib::Pose line, int timeout,
                       MoveStraightParams params, bool async) {
  const Pose realLine = m_transform->transformPose(line);
  /** Line pose with theta in standard radians */
  const Pose radianLine = realLine.withTheta(radToDeg(M_PI_2 - realLine.theta));
  const Vector v_vec = Vector::fromAngle(radianLine.theta);
  auto errorFuncFactory = [&]() -> std::function<float()> {
    const Pose startTheta = lemlib::Chassis::getPose(true, true);
    const Vector u_vec = Vector::fromAngle(startTheta.theta);
    const float u_cross_v = u_vec.cross(v_vec);
    if (std::abs(u_cross_v) < sin(degToRad(1))) {
      // If the robot is facing parallel to the line, return 0 for the error
      // function, since we won't be able to reach the line by driving straight.
      infoSink()->error(
          "Since the robot is facing parallel to the line in moveToLine(),"
          "the bot cannot reach line, and thus therefore motion will be "
          "skipped. Line theta: {}, Robot theta: {}",
          realLine.theta, bot.getPose().theta);
      return []() { return 0; };
    }
    return [this, radianLine, v_vec, u_cross_v]() -> float {
      const Vector PQ_vec =
          Vector::between(lemlib::Chassis::getPose(), radianLine);
      return PQ_vec.cross(v_vec) / u_cross_v;
    };
  };
  this->moveStraight(errorFuncFactory, timeout, params, async);
}
void Robot::moveDistance(float distance, int timeout, MoveStraightParams params,
                         bool async) {
  auto errorFuncFactory = [this, distance]() {
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
  const float targetTheta = lastPose.theta;
  const auto errorFunc = errorFuncFactory();

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

    infoSink()->debug("Angular Out: {}, Lateral Out: {}", angularOut,
                      lateralOut);

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