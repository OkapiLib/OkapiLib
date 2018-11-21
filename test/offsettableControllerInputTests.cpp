/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/offsettableControllerInput.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class OffsettableControllerInputTest : public ::testing::Test {
  protected:
  void SetUp() override {
    mockInput = new MockControllerInput();
    input = new OffsettableControllerInput(std::shared_ptr<MockControllerInput>(mockInput));
  }

  void TearDown() override {
    delete mockInput;
  }

  MockControllerInput *mockInput;
  OffsettableControllerInput *input;
};

TEST_F(OffsettableControllerInputTest, TestTarePositionNormally) {
  mockInput->reading = 100;
  EXPECT_EQ(input->controllerGet(), 100);

  input->tarePosition();

  mockInput->reading = 200;
  EXPECT_EQ(input->controllerGet(), 100);
}

TEST_F(OffsettableControllerInputTest, TestTarePositionDoesNothingWithError) {
  mockInput->reading = 100;
  EXPECT_EQ(input->controllerGet(), 100);

  mockInput->reading = OKAPI_PROS_ERR;
  input->tarePosition();

  mockInput->reading = 200;
  EXPECT_EQ(input->controllerGet(), 200);
}
