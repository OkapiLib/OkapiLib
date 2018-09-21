/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncPosPidController.hpp"
#include "okapi/api/control/async/asyncVelPidController.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class AsyncWrapperTest : public ::testing::Test {
  protected:
  void SetUp() override {
    input = std::make_shared<MockContinuousRotarySensor>();
    output = std::make_shared<MockMotor>();
    posPIDController = new AsyncPosPIDController(input, output, createTimeUtil(), 0.1, 0, 0);
    velPIDController = new AsyncVelPIDController(
      input,
      output,
      createTimeUtil(),
      0.1,
      0,
      0,
      0,
      std::make_unique<VelMath>(
        imev5TPR, std::make_shared<PassthroughFilter>(), 0_ms, std::make_unique<MockTimer>()));
  }

  void TearDown() override {
    delete posPIDController;
    delete velPIDController;
  }

  std::shared_ptr<MockContinuousRotarySensor> input;
  std::shared_ptr<MockMotor> output;
  AsyncPosPIDController *posPIDController;
  AsyncVelPIDController *velPIDController;
};

TEST_F(AsyncWrapperTest, GetTargetTestPosPID) {
  posPIDController->setTarget(10);
  EXPECT_EQ(posPIDController->getTarget(), 10);
}

TEST_F(AsyncWrapperTest, GetTargetTestVelPID) {
  velPIDController->setTarget(10);
  EXPECT_EQ(velPIDController->getTarget(), 10);
}

TEST_F(AsyncWrapperTest, SettledWhenDisabledPosPID) {
  assertControllerIsSettledWhenDisabled(*posPIDController, 100.0);
}

TEST_F(AsyncWrapperTest, SettledWhenDisabledVelPID) {
  assertControllerIsSettledWhenDisabled(*velPIDController, 100.0);
}

TEST_F(AsyncWrapperTest, WaitUntilSettledWorksWhenDisabledPosPID) {
  assertWaitUntilSettledWorksWhenDisabled(*posPIDController);
}

TEST_F(AsyncWrapperTest, WaitUntilSettledWorksWhenDisabledVelPID) {
  assertWaitUntilSettledWorksWhenDisabled(*velPIDController);
}

TEST_F(AsyncWrapperTest, FollowsDisableLifecyclePosPID) {
  assertAsyncControllerFollowsDisableLifecycle(
    *posPIDController,
    output->lastPosition,
    output->lastVoltage,
    0); // Expected output is 0 since this controller requires stepping in another loop
}

TEST_F(AsyncWrapperTest, FollowsDisableLifecycleVelPID) {
  assertAsyncControllerFollowsDisableLifecycle(
    *velPIDController,
    output->lastPosition,
    output->lastVoltage,
    0); // Expected output is 0 since this controller requires stepping in another loop
}

TEST_F(AsyncWrapperTest, FollowsTargetLifecyclePosPID) {
  assertControllerFollowsTargetLifecycle(*posPIDController);
}

TEST_F(AsyncWrapperTest, FollowsTargetLifecycleVelPID) {
  assertControllerFollowsTargetLifecycle(*velPIDController);
}

TEST_F(AsyncWrapperTest, ScalesControllerSetTargetPosPID) {
  assertAsyncWrapperScalesControllerSetTargets(*posPIDController);
}

TEST_F(AsyncWrapperTest, ScalesControllerSetTargetVelPID) {
  assertAsyncWrapperScalesControllerSetTargets(*velPIDController);
}
