/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/odometry/odomMath.hpp"
#include <gtest/gtest.h>

using namespace okapi;

TEST(OdomMathTests, ComputeDistanceToPoint) {
  EXPECT_FLOAT_EQ(
    OdomMath::computeDistanceToPoint({2_m, 3_m}, OdomState{1_m, -2_m, 75_deg}).convert(meter),
    sqrt(26));
}

TEST(OdomMathTests, ComputeAngleToPoint) {
  EXPECT_FLOAT_EQ(
    OdomMath::computeAngleToPoint({2_m, 3_m}, OdomState{1_m, -2_m, 75_deg}).convert(degree),
    atan2(5, 1) * radianToDegree - 75);
}

TEST(OdomMathTests, ComputeDistanceAndAngleToPoint) {
  auto [dist, angle] =
    OdomMath::computeDistanceAndAngleToPoint({2_m, 3_m}, OdomState{1_m, -2_m, 75_deg});
  EXPECT_FLOAT_EQ(sqrt(26), dist.convert(meter));
  EXPECT_FLOAT_EQ(atan2(5, 1) * radianToDegree - 75, angle.convert(degree));
}
