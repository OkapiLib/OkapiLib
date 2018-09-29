/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncVelIntegratedController.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace okapi;

class AsyncVelIntegratedControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    motor = std::make_shared<MockMotor>();
    controller = new AsyncVelIntegratedController(motor, createTimeUtil());
  }

  void TearDown() override {
    delete controller;
  }

  std::shared_ptr<MockMotor> motor;
  AsyncVelIntegratedController *controller;
};

TEST_F(AsyncVelIntegratedControllerTest, GetTargetTest) {
  controller->setTarget(10);
  EXPECT_EQ(controller->getTarget(), 10);
}

TEST_F(AsyncVelIntegratedControllerTest, SettledWhenDisabled) {
  assertControllerIsSettledWhenDisabled(*controller, 100.0);
}

TEST_F(AsyncVelIntegratedControllerTest, WaitUntilSettledWorksWhenDisabled) {
  assertWaitUntilSettledWorksWhenDisabled(*controller);
}

TEST_F(AsyncVelIntegratedControllerTest, FollowsDisableLifecycle) {
  assertAsyncControllerFollowsDisableLifecycle(
    *controller, motor->lastVelocity, motor->lastVoltage, 100);
}

TEST_F(AsyncVelIntegratedControllerTest, FollowsTargetLifecycle) {
  assertControllerFollowsTargetLifecycle(*controller);
}

TEST_F(AsyncVelIntegratedControllerTest, ControllerSetScalesTarget) {
  controller->controllerSet(1);
  EXPECT_EQ(controller->getTarget(), toUnderlyingType(motor->getGearing()));
}
