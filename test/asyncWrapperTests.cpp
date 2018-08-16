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
  }

  void TearDown() override {
    delete posPIDController;
  }

  std::shared_ptr<MockContinuousRotarySensor> input;
  std::shared_ptr<MockMotor> output;
  AsyncPosPIDController *posPIDController;
};

TEST_F(AsyncWrapperTest, PosPIDController) {
}

TEST_F(AsyncWrapperTest, GetTargetTest) {
  posPIDController->setTarget(10);
  EXPECT_EQ(posPIDController->getTarget(), 10);
}

TEST_F(AsyncWrapperTest, SettledWhenDisabled) {
  assertControllerIsSettledWhenDisabled(*posPIDController, 100.0);
}

TEST_F(AsyncWrapperTest, WaitUntilSettledWorksWhenDisabled) {
  assertWaitUntilSettledWorksWhenDisabled(*posPIDController);
}

TEST_F(AsyncWrapperTest, FollowsDisableLifecycle) {
  assertControllerFollowsDisableLifecycle(
    *posPIDController,
    output->lastPosition,
    output->lastVoltage,
    0); // Expected output is 0 since this controller requires stepping in another loop
}

TEST_F(AsyncWrapperTest, FollowsTargetLifecycle) {
  assertControllerFollowsTargetLifecycle(
    *posPIDController,
    0); // Expected output is 0 since this controller requires stepping in another loop
}
