/*
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
    controller = new AsyncVelIntegratedController(
      motor, motor->gearset * 1.5, toUnderlyingType(motor->gearset), createTimeUtil());
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

TEST_F(AsyncVelIntegratedControllerTest, GetErrorTest) {
  controller->setTarget(10);
  EXPECT_EQ(controller->getError(), 10);
}

TEST_F(AsyncVelIntegratedControllerTest, SettledWhenDisabled) {
  assertControllerIsSettledWhenDisabled(*controller, 100.0);
}

TEST_F(AsyncVelIntegratedControllerTest, WaitUntilSettledWorksWhenDisabled) {
  assertWaitUntilSettledWorksWhenDisabled(*controller);
}

TEST_F(AsyncVelIntegratedControllerTest, FollowsDisableLifecycle) {
  assertAsyncControllerFollowsDisableLifecycle(
    *controller, motor->lastVelocity, motor->lastVoltage, 150);
}

TEST_F(AsyncVelIntegratedControllerTest, FollowsTargetLifecycle) {
  assertControllerFollowsTargetLifecycle(*controller);
}

TEST_F(AsyncVelIntegratedControllerTest, ControllerSetScalesTarget) {
  controller->controllerSet(1);
  EXPECT_EQ(controller->getTarget(), toUnderlyingType(motor->getGearing()));
}

TEST_F(AsyncVelIntegratedControllerTest, ExternalRatioWorksForPositiveInteger) {
  motor->setGearing(AbstractMotor::gearset::red);
  AsyncVelIntegratedController newController(
    motor, motor->gearset * 2, toUnderlyingType(motor->gearset), createTimeUtil());
  newController.setTarget(5);
  EXPECT_EQ(motor->lastVelocity, 10);
}

TEST_F(AsyncVelIntegratedControllerTest, ExternalRatioWorksForFraction) {
  motor->setGearing(AbstractMotor::gearset::red);
  AsyncVelIntegratedController newController(
    motor, motor->gearset * (2.0 / 3), toUnderlyingType(motor->gearset), createTimeUtil());
  newController.setTarget(5);
  EXPECT_EQ(motor->lastVelocity, static_cast<std::int16_t>(5 * (2.0 / 3)));
}

TEST_F(AsyncVelIntegratedControllerTest, GearRatioOfZeroThrowsException) {
  EXPECT_THROW(AsyncVelIntegratedController(motor, motor->gearset * 0, 100, createTimeUtil()),
               std::invalid_argument);
}
