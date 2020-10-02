/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncVelPidController.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class AsyncVelPIDControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    input = std::make_shared<MockControllerInput>();
    output = std::make_shared<MockMotor>();
    controller = new AsyncVelPIDController(
      input,
      output,
      createTimeUtil(),
      0,
      0,
      0,
      0,
      std::make_unique<VelMath>(
        360, std::make_unique<PassthroughFilter>(), 10_ms, std::make_unique<MockTimer>()));
  }

  void TearDown() override {
    delete controller;
  }

  std::shared_ptr<MockControllerInput> input;
  std::shared_ptr<MockMotor> output;
  AsyncVelPIDController *controller;
};

TEST_F(AsyncVelPIDControllerTest, TestSetAndGetGains) {
  IterativeVelPIDController::Gains gains{1, 2, 3, 4};
  controller->setGains(gains);
  EXPECT_EQ(controller->getGains(), gains);
}
