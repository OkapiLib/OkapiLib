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

class IterativePosPIDControllerTest : public ::testing::Test {
  protected:
  virtual void SetUp() {
    controller = new IterativePosPIDController(0.1, 0, 0, 0, createConstantTimeUtil(10_ms));
  }

  virtual void TearDown() {
    delete controller;
  }

  IterativePosPIDController *controller;
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
