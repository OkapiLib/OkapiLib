/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncPosIntegratedController.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class AsyncPosIntegratedControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    motor = new MockMotor();
    controller =
      new AsyncPosIntegratedController(std::shared_ptr<MockMotor>(motor), createTimeUtil());
  }

  void TearDown() override {
    delete controller;
  }

  MockMotor *motor;
  AsyncPosIntegratedController *controller;
};

TEST_F(AsyncPosIntegratedControllerTest, GetTargetTest) {
  controller->setTarget(10);
  EXPECT_EQ(controller->getTarget(), 10);
}

TEST_F(AsyncPosIntegratedControllerTest, SettledWhenDisabled) {
  assertControllerIsSettledWhenDisabled(*controller, 100.0);
}

TEST_F(AsyncPosIntegratedControllerTest, WaitUntilSettledWorksWhenDisabled) {
  assertWaitUntilSettledWorksWhenDisabled(*controller);
}

TEST_F(AsyncPosIntegratedControllerTest, FollowsDisableLifecycle) {
  assertControllerFollowsDisableLifecycle(
    *controller, motor->lastPosition, motor->lastVoltage, 100);
}

TEST_F(AsyncPosIntegratedControllerTest, FollowsTargetLifecycle) {
  assertControllerFollowsTargetLifecycle(*controller, 100);
}
