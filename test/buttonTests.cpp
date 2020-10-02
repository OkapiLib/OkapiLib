/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/device/button/buttonBase.hpp"
#include <gtest/gtest.h>
#include <utility>

using namespace okapi;

class MockButton : public ButtonBase {
  public:
  MockButton(std::initializer_list<bool> initializerList) : returnVals(initializerList) {
  }

  bool currentlyPressed() override {
    return returnVals.at(index);
  }

  std::vector<bool> returnVals{};
  size_t index = 0;
};

TEST(ButtonBaseTest, IsPressedShouldMirrorCurrentlyPressed) {
  MockButton btn({false, true, false});

  EXPECT_FALSE(btn.isPressed());
  btn.index++;
  EXPECT_TRUE(btn.isPressed());
  btn.index++;
  EXPECT_FALSE(btn.isPressed());
}

class ButtonBaseChangeTest : public ::testing::Test {
  protected:
  MockButton btn{false, true, true, false, false};
};

TEST_F(ButtonBaseChangeTest, ChangeShouldDetectBothEdges) {
  EXPECT_FALSE(btn.changed());
  btn.index++;
  EXPECT_TRUE(btn.changed());
  btn.index++;
  EXPECT_FALSE(btn.changed());
  btn.index++;
  EXPECT_TRUE(btn.changed());
  btn.index++;
  EXPECT_FALSE(btn.changed());
}

TEST_F(ButtonBaseChangeTest, ChangedToPressedShouldDetectRisingEdges) {
  EXPECT_FALSE(btn.changedToPressed());
  btn.index++;
  EXPECT_TRUE(btn.changedToPressed());
  btn.index++;
  EXPECT_FALSE(btn.changedToPressed());
  btn.index++;
  EXPECT_FALSE(btn.changedToPressed());
  btn.index++;
  EXPECT_FALSE(btn.changedToPressed());
}

TEST_F(ButtonBaseChangeTest, ChangedToReleasedShouldDetectFallingEdges) {
  EXPECT_FALSE(btn.changedToReleased());
  btn.index++;
  EXPECT_FALSE(btn.changedToReleased());
  btn.index++;
  EXPECT_FALSE(btn.changedToReleased());
  btn.index++;
  EXPECT_TRUE(btn.changedToReleased());
  btn.index++;
  EXPECT_FALSE(btn.changedToReleased());
}

TEST_F(ButtonBaseChangeTest, CallAllMethodsTogether) {
  EXPECT_FALSE(btn.isPressed());
  EXPECT_FALSE(btn.changed());
  EXPECT_FALSE(btn.changedToPressed());
  EXPECT_FALSE(btn.changedToReleased());
  btn.index++;
  EXPECT_TRUE(btn.isPressed());
  EXPECT_TRUE(btn.changed());
  EXPECT_TRUE(btn.changedToPressed());
  EXPECT_FALSE(btn.changedToReleased());
  btn.index++;
  EXPECT_TRUE(btn.isPressed());
  EXPECT_FALSE(btn.changed());
  EXPECT_FALSE(btn.changedToPressed());
  EXPECT_FALSE(btn.changedToReleased());
  btn.index++;
  EXPECT_FALSE(btn.isPressed());
  EXPECT_TRUE(btn.changed());
  EXPECT_FALSE(btn.changedToPressed());
  EXPECT_TRUE(btn.changedToReleased());
  btn.index++;
  EXPECT_FALSE(btn.isPressed());
  EXPECT_FALSE(btn.changed());
  EXPECT_FALSE(btn.changedToPressed());
  EXPECT_FALSE(btn.changedToReleased());
}
