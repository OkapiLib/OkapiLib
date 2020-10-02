/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class ChassisScalesTest : public ::testing::Test {};

TEST_F(ChassisScalesTest, RawScales) {
  ChassisScales scales({0.5, 0.3}, imev5GreenTPR);
  EXPECT_DOUBLE_EQ(scales.straight, 0.5);
  EXPECT_DOUBLE_EQ(scales.turn, 0.3);
}

TEST_F(ChassisScalesTest, ScalesFromWheelbase) {
  ChassisScales scales({4_in, 11.5_in}, 360);
  EXPECT_FLOAT_EQ(scales.straight, 1127.8696);
  EXPECT_FLOAT_EQ(scales.turn, 2.875);

  // Feed the raw scales back in to get the lengths out
  ChassisScales reverse({1127.8696, 2.875}, 360);
  EXPECT_NEAR(reverse.wheelDiameter.convert(meter), (4_in).convert(meter), 0.0001);
  EXPECT_NEAR(reverse.wheelTrack.convert(meter), (11.5_in).convert(meter), 0.0001);
}

TEST_F(ChassisScalesTest, TestMiddleScaleWithTwoMeasurements) {
  ChassisScales scales({4_in, 11.5_in}, imev5GreenTPR);
  EXPECT_FLOAT_EQ(scales.middleWheelDiameter.convert(inch), 4);
  EXPECT_FLOAT_EQ(scales.middle, scales.straight);
}

TEST_F(ChassisScalesTest, TestMiddleScaleWithThreeMeasurements) {
  ChassisScales scales({4_in, 11.5_in, 11.5_in / 2, 5_in}, imev5GreenTPR);
  EXPECT_FLOAT_EQ(scales.middleWheelDiameter.convert(inch), 5);
  EXPECT_NE(scales.middle, scales.straight);
}

TEST_F(ChassisScalesTest, TestDifferentTPR) {
  ChassisScales scalesGreen({4_in, 10_in}, imev5GreenTPR);
  ChassisScales scalesBlue({4_in, 10_in}, imev5BlueTPR);

  EXPECT_FLOAT_EQ(scalesGreen.straight / 3.0, scalesBlue.straight);
  EXPECT_FLOAT_EQ(scalesGreen.turn, scalesBlue.turn * imev5GreenTPR / imev5BlueTPR);
  EXPECT_FLOAT_EQ(scalesGreen.middle / 3.0, scalesBlue.middle);
}

TEST_F(ChassisScalesTest, TestLengthToMiddleWheel) {
  ChassisScales scales({1_in, 2_in, 1_in}, 360);
  EXPECT_FLOAT_EQ(scales.middleWheelDistance.convert(inch), 1);
  EXPECT_FLOAT_EQ(scales.straight, scales.middle);
}

TEST_F(ChassisScalesTest, TestMiddleWheelScaleWithLength) {
  ChassisScales scales({1_in, 2_in, 1_in, 2_in}, 360);
  EXPECT_FLOAT_EQ(scales.middleWheelDistance.convert(inch), 1);
  EXPECT_FLOAT_EQ(scales.middle, scales.straight / 2.0);
}
