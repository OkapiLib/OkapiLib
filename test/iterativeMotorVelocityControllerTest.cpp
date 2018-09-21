/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/iterative/iterativeMotorVelocityController.hpp"
#include "okapi/api/control/iterative/iterativeVelPidController.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class MockIterativeVelPIDController : public IterativeVelPIDController {
  public:
  using IterativeVelPIDController::IterativeVelPIDController;
  using IterativeVelPIDController::outputMax;
  using IterativeVelPIDController::outputMin;
};

class MockIterativeMotorVelocityController : public IterativeMotorVelocityController {
  public:
  using IterativeMotorVelocityController::IterativeMotorVelocityController;
};

class IterativeMotorVelocityControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    motor = std::make_shared<MockMotor>();
    velController = std::make_shared<MockIterativeVelPIDController>(
      0,
      0,
      0,
      0,
      std::make_unique<VelMath>(1800,
                                std::make_shared<PassthroughFilter>(),
                                0_ms,
                                std::make_unique<ConstantMockTimer>(10_ms)),
      createTimeUtil(Supplier<std::unique_ptr<AbstractTimer>>(
        []() { return std::make_unique<ConstantMockTimer>(10_ms); })));
    controller = new MockIterativeMotorVelocityController(motor, velController);
  }

  void TearDown() override {
    delete controller;
  }

  std::shared_ptr<MockMotor> motor;
  std::shared_ptr<MockIterativeVelPIDController> velController;
  MockIterativeMotorVelocityController *controller;
};

TEST_F(IterativeMotorVelocityControllerTest, SettledWhenDisabled) {
  velController->setGains(0.1, 0.1, 0.1, 0.1);
  assertControllerIsSettledWhenDisabled(*controller, 100.0);
}

TEST_F(IterativeMotorVelocityControllerTest, DisabledLifecycle) {
  velController->setGains(0.1, 0, 0, 0);
  assertIterativeControllerFollowsDisableLifecycle(*controller);
}

TEST_F(IterativeMotorVelocityControllerTest, TargetLifecycle) {
  velController->setGains(0.1, 0, 0, 0);
  assertControllerFollowsTargetLifecycle(*controller);
}

TEST_F(IterativeMotorVelocityControllerTest, ScalesControllerSetTarget) {
  assertIterativeControllerScalesControllerSetTargets(*controller);
}

TEST_F(IterativeMotorVelocityControllerTest, StaticFrictionGainUsesTargetSign) {
  velController->setGains(0, 0, 0, 0.1);

  controller->setTarget(1);
  EXPECT_DOUBLE_EQ(controller->step(0), 1 * 0.1);

  // Use the same target but send the error to 0 to make sure the gain is applied to the target and
  // not the error
  EXPECT_DOUBLE_EQ(controller->step(1), 1 * 0.1);

  controller->setTarget(-1);
  EXPECT_DOUBLE_EQ(controller->step(0), -1 * 0.1);

  // Use the same target but send the error to 0 to make sure the gain is applied to the target and
  // not the error
  EXPECT_DOUBLE_EQ(controller->step(-1), -1 * 0.1);
}

TEST_F(IterativeMotorVelocityControllerTest, SetOutputLimitsTest) {
  controller->setOutputLimits(0.5, -0.5);
  EXPECT_DOUBLE_EQ(velController->outputMax, 0.5);
  EXPECT_DOUBLE_EQ(velController->getMaxOutput(), 0.5);
  EXPECT_DOUBLE_EQ(controller->getMaxOutput(), 0.5);
  EXPECT_DOUBLE_EQ(velController->outputMin, -0.5);
  EXPECT_DOUBLE_EQ(velController->getMinOutput(), -0.5);
  EXPECT_DOUBLE_EQ(controller->getMinOutput(), -0.5);
}

TEST_F(IterativeMotorVelocityControllerTest, SetOutputLimitsReversedTest) {
  controller->setOutputLimits(-0.5, 0.5);
  EXPECT_DOUBLE_EQ(velController->outputMax, 0.5);
  EXPECT_DOUBLE_EQ(velController->getMaxOutput(), 0.5);
  EXPECT_DOUBLE_EQ(controller->getMaxOutput(), 0.5);
  EXPECT_DOUBLE_EQ(velController->outputMin, -0.5);
  EXPECT_DOUBLE_EQ(velController->getMinOutput(), -0.5);
  EXPECT_DOUBLE_EQ(controller->getMinOutput(), -0.5);
}

TEST_F(IterativeMotorVelocityControllerTest, NoOutputWhenDisabled) {
  controller->setTarget(10);
  controller->step(0); // Generate some output
  controller->flipDisable(true);

  // Check output before and after since step writes to output
  EXPECT_EQ(controller->getOutput(), 0);
  EXPECT_EQ(controller->step(0), 0);
  EXPECT_EQ(controller->getOutput(), 0);
}

TEST_F(IterativeMotorVelocityControllerTest, SetTargetWorksWhenDisabled) {
  controller->setTarget(10);
  controller->flipDisable(true);

  EXPECT_EQ(controller->getTarget(), 10);
}

TEST_F(IterativeMotorVelocityControllerTest, SampleTime) {
  controller->setSampleTime(20_ms);
  EXPECT_EQ(controller->getSampleTime(), 20_ms);

  // Output will be zero because the mock timer always reads dt of 10_ms and the sample time is
  // 20_ms
  EXPECT_EQ(controller->step(-1), 0);
}
