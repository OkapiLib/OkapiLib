/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

TEST(ChassisScalesTest, RawScales) {
  ChassisScales scales({0.5, 0.3});
  EXPECT_DOUBLE_EQ(scales.straight, 0.5);
  EXPECT_DOUBLE_EQ(scales.turn, 0.3);
}

TEST(ChassisScalesTest, ScalesFromWheelbase) {
  ChassisScales scales({4_in, 11.5_in});
  EXPECT_FLOAT_EQ(scales.straight, 1127.8696);
  EXPECT_FLOAT_EQ(scales.turn, 2.875);

  // Feed the raw scales back in to get the lengths out
  ChassisScales reverse({1127.8696, 2.875});
  EXPECT_NEAR(reverse.wheelDiameter.convert(meter), (4_in).convert(meter), 0.0001);
  EXPECT_NEAR(reverse.wheelbaseWidth.convert(meter), (11.5_in).convert(meter), 0.0001);
}
