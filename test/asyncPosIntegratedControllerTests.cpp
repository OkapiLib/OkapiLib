/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncPosIntegratedController.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class AsyncPosIntegratedControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    motor = std::make_shared<MockMotor>();
    controller = new AsyncPosIntegratedController(
      motor, motor->gearset * 1.5, toUnderlyingType(motor->gearset), createTimeUtil());
  }

  void TearDown() override {
    delete controller;
  }

  std::shared_ptr<MockMotor> motor;
  AsyncPosIntegratedController *controller;
};

TEST_F(AsyncPosIntegratedControllerTest, GetTargetTest) {
  controller->setTarget(10);
  EXPECT_EQ(controller->getTarget(), 10);
}

TEST_F(AsyncPosIntegratedControllerTest, GetErrorTest) {
  controller->setTarget(10);
  EXPECT_EQ(controller->getError(), 10);
}

TEST_F(AsyncPosIntegratedControllerTest, SettledWhenDisabled) {
  assertControllerIsSettledWhenDisabled(*controller, 100.0);
}

TEST_F(AsyncPosIntegratedControllerTest, WaitUntilSettledWorksWhenDisabled) {
  assertWaitUntilSettledWorksWhenDisabled(*controller);
}

TEST_F(AsyncPosIntegratedControllerTest, FollowsDisableLifecycle) {
  assertAsyncControllerFollowsDisableLifecycle(
    *controller, motor->lastPosition, motor->lastVoltage, 150);
}

TEST_F(AsyncPosIntegratedControllerTest, FollowsTargetLifecycle) {
  assertControllerFollowsTargetLifecycle(*controller);
}

TEST_F(AsyncPosIntegratedControllerTest, ControllerSetScalesTarget) {
  controller->controllerSet(1);
  EXPECT_EQ(controller->getTarget(), toUnderlyingType(motor->getGearing()));
}

TEST_F(AsyncPosIntegratedControllerTest, ProfiledMovementUsesMaxVelocityForRedGearset) {
  motor->setGearing(AbstractMotor::gearset::red);
  AsyncPosIntegratedController newController(
    motor, motor->gearset, toUnderlyingType(motor->gearset), createTimeUtil());
  newController.setTarget(5);
  EXPECT_EQ(motor->lastProfiledMaxVelocity, toUnderlyingType(AbstractMotor::gearset::red));
}

TEST_F(AsyncPosIntegratedControllerTest, ProfiledMovementUsesMaxVelocityForBlueGearset) {
  motor->setGearing(AbstractMotor::gearset::blue);
  AsyncPosIntegratedController newController(
    motor, motor->gearset, toUnderlyingType(motor->gearset), createTimeUtil());
  newController.setTarget(5);
  EXPECT_EQ(motor->lastProfiledMaxVelocity, toUnderlyingType(AbstractMotor::gearset::blue));
}

TEST_F(AsyncPosIntegratedControllerTest, ProfiledMovementUsesMaxVelocityForGreenGearset) {
  motor->setGearing(AbstractMotor::gearset::green);
  AsyncPosIntegratedController newController(
    motor, motor->gearset, toUnderlyingType(motor->gearset), createTimeUtil());
  newController.setTarget(5);
  EXPECT_EQ(motor->lastProfiledMaxVelocity, toUnderlyingType(AbstractMotor::gearset::green));
}

TEST_F(AsyncPosIntegratedControllerTest, TarePositionWorksWithSetTarget) {
  controller->setTarget(10);
  EXPECT_EQ(controller->getError(), 10);
  EXPECT_EQ(motor->lastPosition, 15);

  motor->encoder->value = 15;
  EXPECT_EQ(controller->getError(), 0);

  controller->tarePosition();
  EXPECT_EQ(controller->getError(), 10);

  motor->lastPosition = 7; // So we can detect if the controller wrote anything
  controller->setTarget(10);
  EXPECT_EQ(controller->getError(), 10);
  EXPECT_EQ(motor->lastPosition, 30);
}

TEST_F(AsyncPosIntegratedControllerTest, ExternalRatioWorksForPositiveInteger) {
  motor->setGearing(AbstractMotor::gearset::red);
  AsyncPosIntegratedController newController(
    motor, motor->gearset * 2, toUnderlyingType(motor->gearset), createTimeUtil());
  newController.setTarget(5);
  EXPECT_EQ(motor->lastPosition, 10);
  EXPECT_EQ(motor->lastProfiledMaxVelocity, toUnderlyingType(AbstractMotor::gearset::red));
}

TEST_F(AsyncPosIntegratedControllerTest, ExternalRatioWorksForFraction) {
  motor->setGearing(AbstractMotor::gearset::red);
  AsyncPosIntegratedController newController(
    motor, motor->gearset * (2.0 / 3), toUnderlyingType(motor->gearset), createTimeUtil());
  newController.setTarget(5);
  EXPECT_EQ(motor->lastPosition, static_cast<std::int16_t>(5 * (2.0 / 3)));
  EXPECT_EQ(motor->lastProfiledMaxVelocity, toUnderlyingType(AbstractMotor::gearset::red));
}

TEST_F(AsyncPosIntegratedControllerTest, GearRatioOfZeroThrowsException) {
  EXPECT_THROW(AsyncPosIntegratedController(motor, motor->gearset * 0, 100, createTimeUtil()),
               std::invalid_argument);
}
