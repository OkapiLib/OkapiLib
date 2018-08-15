/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncVelIntegratedController.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace okapi;

class AsyncVelIntegratedControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    motor = new MockMotor();
    controller =
      new AsyncVelIntegratedController(std::shared_ptr<MockMotor>(motor), createTimeUtil());
  }

  void TearDown() override {
    delete controller;
  }

  MockMotor *motor;
  AsyncVelIntegratedController *controller;
};

TEST_F(AsyncVelIntegratedControllerTest, SettledWhenDisabled) {
  assertControllerIsSettledWhenDisabled(*controller, 100.0);
}

TEST_F(AsyncVelIntegratedControllerTest, WaitUntilSettledWorksWhenDisabled) {
  assertWaitUntilSettledWorksWhenDisabled(*controller);
}

TEST_F(AsyncVelIntegratedControllerTest, FollowsDisableLifecycle) {
  assertControllerFollowsDisableLifecycle(*controller, motor->lastVelocity, motor->lastVoltage);
}

TEST_F(AsyncVelIntegratedControllerTest, FollowsTargetLifecycle) {
  assertControllerFollowsTargetLifecycle(*controller);
}
