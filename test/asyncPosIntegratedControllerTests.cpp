/**
 * @author Ryan Benasutti, WPI
 *
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
    controller = new AsyncPosIntegratedController(motor, createTimeUtil());
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

TEST_F(AsyncPosIntegratedControllerTest, SettledWhenDisabled) {
  assertControllerIsSettledWhenDisabled(*controller, 100.0);
}

TEST_F(AsyncPosIntegratedControllerTest, WaitUntilSettledWorksWhenDisabled) {
  assertWaitUntilSettledWorksWhenDisabled(*controller);
}

TEST_F(AsyncPosIntegratedControllerTest, FollowsDisableLifecycle) {
  assertAsyncControllerFollowsDisableLifecycle(
    *controller, motor->lastPosition, motor->lastVoltage, 100);
}

TEST_F(AsyncPosIntegratedControllerTest, FollowsTargetLifecycle) {
  assertControllerFollowsTargetLifecycle(*controller);
}

TEST_F(AsyncPosIntegratedControllerTest, ControllerSetScalesTarget) {
  controller->controllerSet(1);
  EXPECT_EQ(controller->getTarget(), toUnderlyingType(motor->getGearing()));
}

TEST_F(AsyncPosIntegratedControllerTest, ProfiledMovementUsesMaxVelocityForRedGearset) {
  auto motor = std::make_shared<MockMotor>();
  motor->setGearing(AbstractMotor::gearset::red);
  AsyncPosIntegratedController controller(motor, createTimeUtil());
  controller.setTarget(5);
  EXPECT_EQ(motor->lastProfiledMaxVelocity, toUnderlyingType(AbstractMotor::gearset::red));
}

TEST_F(AsyncPosIntegratedControllerTest, ProfiledMovementUsesMaxVelocityForBlueGearset) {
  auto motor = std::make_shared<MockMotor>();
  motor->setGearing(AbstractMotor::gearset::blue);
  AsyncPosIntegratedController controller(motor, createTimeUtil());
  controller.setTarget(5);
  EXPECT_EQ(motor->lastProfiledMaxVelocity, toUnderlyingType(AbstractMotor::gearset::blue));
}

TEST_F(AsyncPosIntegratedControllerTest, ProfiledMovementUsesMaxVelocityForGreenGearset) {
  auto motor = std::make_shared<MockMotor>();
  motor->setGearing(AbstractMotor::gearset::green);
  AsyncPosIntegratedController controller(motor, createTimeUtil());
  controller.setTarget(5);
  EXPECT_EQ(motor->lastProfiledMaxVelocity, toUnderlyingType(AbstractMotor::gearset::green));
}
