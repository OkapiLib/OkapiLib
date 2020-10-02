/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncPosPidController.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class AsyncPosPIDControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    input = std::make_shared<MockControllerInput>();
    output = std::make_shared<MockMotor>();
    controller = new AsyncPosPIDController(input, output, createTimeUtil(), 0, 0, 0);
  }

  void TearDown() override {
    delete controller;
  }

  std::shared_ptr<MockControllerInput> input;
  std::shared_ptr<MockMotor> output;
  AsyncPosPIDController *controller;
};

TEST_F(AsyncPosPIDControllerTest, TestTarePosition) {
  controller->startThread();
  auto rate = createTimeUtil().getRate();

  input->reading = 0;
  controller->setTarget(100);
  rate->delayUntil(100_ms);
  EXPECT_EQ(controller->getError(), 100);

  input->reading = 100;
  rate->delayUntil(100_ms);
  EXPECT_EQ(controller->getError(), 0);

  controller->tarePosition();
  rate->delayUntil(100_ms);
  EXPECT_EQ(controller->getError(), 100);
}

TEST_F(AsyncPosPIDControllerTest, TestSetAndGetGains) {
  IterativePosPIDController::Gains gains{1, 2, 3, 4};
  controller->setGains(gains);
  EXPECT_EQ(controller->getGains(), gains);
}
