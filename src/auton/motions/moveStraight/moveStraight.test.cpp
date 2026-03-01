// If building for GTEST, do build test code.
// If viewing in IDE, enable for intellisense purposes.
// Otherwise, do not.
#include <cmath>
#if defined(GTEST) || defined(__clang__)
#include "./common.h"
#include "gmock/gmock.h"
#include <gtest/gtest.h>

NlPose getPoseFakeZero() { return NlPose(0, 0, 0); }
void errorSinkFake(const std::string &msg) { FAIL() << msg; }
TEST(MoveToErrorFactory, moveToXPerp) {
  const float targetX = 10;

  NlPose currentPose{0, 0, 0};
  const auto getPoseFake = [&currentPose]() { return currentPose; };
  MoveToLineErrorFactory factoryY0{
      {targetX, 0, M_PI_2}, getPoseFake, errorSinkFake};
  currentPose = NlPose(0, 0, 0);
  EXPECT_NEAR(factoryY0()(), 10, 1e-5);
  currentPose = NlPose(0, 31, 0);
  EXPECT_NEAR(factoryY0()(), 10, 1e-5);
  currentPose = NlPose(5, -41, 0);
  EXPECT_NEAR(factoryY0()(), 5, 1e-5);
  currentPose = NlPose(10, 85, 0);
  EXPECT_NEAR(factoryY0()(), 0, 1e-5);
  currentPose = NlPose(20, -.001, 0);
  EXPECT_NEAR(factoryY0()(), -10, 1e-5);
  currentPose = NlPose(15, -11.111, 0);
  EXPECT_NEAR(factoryY0()(), -5, 1e-5);

  MoveToLineErrorFactory factoryY10{
      {targetX, 10, M_PI_2}, getPoseFakeZero, errorSinkFake};
  MoveToLineErrorFactory factoryYNeg20{
      {targetX, -20, M_PI_2}, getPoseFakeZero, errorSinkFake};

  EXPECT_FLOAT_EQ(factoryY10()(), 10);
  EXPECT_FLOAT_EQ(factoryYNeg20()(), 10);
}

TEST(MoveToErrorFactory, moveToX60Deg) {
  const auto getPoseFake = []() -> NlPose { return {0, 0, 60 * M_PI / 180}; };
  MoveToLineErrorFactory factoryX10{
      {10, 0, M_PI_2}, getPoseFake, errorSinkFake};
  EXPECT_FLOAT_EQ(factoryX10()(), 20);
  MoveToLineErrorFactory factoryX31{
      {31, 0, M_PI_2}, getPoseFake, errorSinkFake};
  EXPECT_FLOAT_EQ(factoryX31()(), 62);
  MoveToLineErrorFactory factoryXNeg10{
      {-10, 0, M_PI_2}, getPoseFake, errorSinkFake};
  EXPECT_FLOAT_EQ(factoryXNeg10()(), -20);
}

TEST(MoveToLineErrorFactory, moveToDiag) {
  NlPose currentPose{0, 0, 0};
  const auto getPoseFake = [&currentPose]() { return currentPose; };
  MoveToLineErrorFactory factoryPos{
      {0, 0, M_PI / 4}, getPoseFake, errorSinkFake};
  MoveToLineErrorFactory factoryNeg{
      {0, 0, -M_PI / 4}, getPoseFake, errorSinkFake};

  currentPose = NlPose(-10, 0, 0);
  EXPECT_NEAR(factoryPos()(), 10, 1e-5);
  EXPECT_NEAR(factoryNeg()(), 10, 1e-5);
  currentPose = NlPose(-10, 10, 0);
  EXPECT_NEAR(factoryPos()(), 20, 1e-5);
  EXPECT_NEAR(factoryNeg()(), 0, 1e-5);
  currentPose = NlPose(10, 10, 0);
  EXPECT_NEAR(factoryPos()(), 0, 1e-5);
  EXPECT_NEAR(factoryNeg()(), -20, 1e-5);

  currentPose = NlPose(-10, 0, M_PI_4);
  EXPECT_NEAR(factoryNeg()(), 5 * sqrt(2), 1e-5);
  currentPose = NlPose(10, 0, -M_PI_4);
  EXPECT_NEAR(factoryPos()(), -5 * sqrt(2), 1e-5);
}

TEST(MoveToLineErrorFactory, moveToParallel) {
  testing::MockFunction<void(const std::string &)> mockErrorSink;
  EXPECT_CALL(mockErrorSink, Call(testing::HasSubstr("parallel to the line")))
      .Times(5);
  const auto evalFactory = [&](NlPose line, NlPose robotPose) {
    return MoveToLineErrorFactory(
        line, [robotPose]() { return robotPose; },
        mockErrorSink.AsStdFunction())()();
  };
  EXPECT_FLOAT_EQ(evalFactory({0, 0, 0}, {10, -10, 0}), 0);
  EXPECT_FLOAT_EQ(evalFactory({0, 0, 0}, {10, -10, M_PI}), 0);
  EXPECT_FLOAT_EQ(evalFactory({.04, 240, 13 * M_PI}, {0, -30, M_PI}), 0);
  EXPECT_FLOAT_EQ(evalFactory({.04, 240, 21}, {0, -30, 21 - M_PI}), 0);
  EXPECT_FLOAT_EQ(evalFactory({float(1. / 0.f), INFINITY, M_PI_4},
                              {-INFINITY, -NAN, -3 * M_PI_4}),
                  0);
}

#endif