/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/api/chassisControllerTests.hpp"
#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "test/crossPlatformTestRunner.hpp"
#include <gtest/gtest.h>

using namespace okapi;
using namespace snowhouse;

TEST(ChassisScalesTest, RawScales) {
  ChassisScales scales({0.5, 0.3});
  EXPECT_FLOAT_EQ(scales.straight, 0.5);
  EXPECT_FLOAT_EQ(scales.turn, 0.3);
}

TEST(ChassisScalesTest, ScalesFromWheelbase) {
  ChassisScales scales({4_in, 11.5_in});
  EXPECT_FLOAT_EQ(scales.straight, 1127.8696);
  EXPECT_FLOAT_EQ(scales.turn, 2.875);
}
