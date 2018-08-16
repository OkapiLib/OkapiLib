/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class MockIterativePosPIDController : public IterativePosPIDController {
  public:
  using IterativePosPIDController::IterativePosPIDController;
  using IterativePosPIDController::integralMax;
  using IterativePosPIDController::integralMin;
};

class IterativePosPIDControllerTest : public ::testing::Test {
  protected:
  virtual void SetUp() {
    controller = new MockIterativePosPIDController(0.1, 0, 0, 0, createConstantTimeUtil(10_ms));
  }

  virtual void TearDown() {
    delete controller;
  }

  MockIterativePosPIDController *controller;
};

TEST_F(IterativePosPIDControllerTest, BasicKpTest) {
  EXPECT_DOUBLE_EQ(controller->step(1), 0.1 * -1);
}

TEST_F(IterativePosPIDControllerTest, DefaultErrorBounds_ErrorOfZero) {
  EXPECT_DOUBLE_EQ(controller->step(0), 0);
}

TEST_F(IterativePosPIDControllerTest, DefaultErrorBounds_ErrorOfOne) {
  EXPECT_DOUBLE_EQ(controller->step(1), 0.1 * -1);
}

TEST_F(IterativePosPIDControllerTest, DefaultErrorBounds_LargeError) {
  const double largeError = 10000000000000;
  EXPECT_DOUBLE_EQ(controller->step(largeError), -1);
}

TEST_F(IterativePosPIDControllerTest, SettledWhenDisabled) {
  assertControllerIsSettledWhenDisabled(*controller, 100.0);
}

TEST_F(IterativePosPIDControllerTest, SetIntegralLimitsTest) {
  MockIterativePosPIDController testController(0, 2, 0, 0, createTimeUtil());
  EXPECT_DOUBLE_EQ(testController.integralMin, -1 / 2.0);
  EXPECT_DOUBLE_EQ(testController.integralMax, 1 / 2.0);
}

TEST_F(IterativePosPIDControllerTest, NoOutputWhenDisabled) {
  controller->setTarget(10);
  controller->flipDisable(true);

  EXPECT_EQ(controller->step(0), 0);
  EXPECT_EQ(controller->getOutput(), 0);
}

TEST_F(IterativePosPIDControllerTest, SetTargetWorksWhenDisabled) {
  controller->setTarget(10);
  controller->flipDisable(true);

  EXPECT_EQ(controller->getTarget(), 10);
}
