/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "okapi/api/control/iterative/iterativeVelPidController.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class IterativeVelPIDControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    controller = new IterativeVelPIDController(
      0,
      0,
      0,
      0,
      std::make_unique<VelMath>(
        1800, std::make_shared<PassthroughFilter>(), std::make_unique<ConstantMockTimer>(10_ms)),
      createTimeUtil(Supplier<std::unique_ptr<AbstractTimer>>(
        []() { return std::make_unique<ConstantMockTimer>(10_ms); })));
  }

  void TearDown() override {
    delete controller;
  }

  IterativeVelPIDController *controller;
};

TEST_F(IterativeVelPIDControllerTest, SettledWhenDisabled) {
  controller->setGains(0.1, 0.1, 0.1, 0.1);
  assertControllerIsSettledWhenDisabled(*controller, 100.0);
}

TEST_F(IterativeVelPIDControllerTest, StaticFrictionGainUsesTargetSign) {
  controller->setGains(0, 0, 0, 0.1);

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
