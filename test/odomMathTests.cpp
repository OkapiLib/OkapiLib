/*
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

TEST(OdomMathTests, ConstrainAngle360) {
  QAngle angle = OdomMath::constrainAngle360(75.0_deg);
  EXPECT_FLOAT_EQ(75.0, angle.convert(degree));
  angle = OdomMath::constrainAngle360(0.0_deg);
  EXPECT_FLOAT_EQ(0.0, angle.convert(degree));
  angle = OdomMath::constrainAngle360(360.0_deg);
  EXPECT_FLOAT_EQ(0.0, angle.convert(degree));
  angle = OdomMath::constrainAngle360(720.0_deg);
  EXPECT_FLOAT_EQ(0.0, angle.convert(degree));
  angle = OdomMath::constrainAngle360(-90.0_deg);
  EXPECT_FLOAT_EQ(270.0, angle.convert(degree));
}

TEST(OdomMathTests, ConstrainAngle180) {
  QAngle angle = OdomMath::constrainAngle180(75.0_deg);
  EXPECT_FLOAT_EQ(75.0, angle.convert(degree));
  angle = OdomMath::constrainAngle180(-75.0_deg);
  EXPECT_FLOAT_EQ(-75.0, angle.convert(degree));
  angle = OdomMath::constrainAngle180(270.0_deg);
  EXPECT_FLOAT_EQ(-90.0, angle.convert(degree));
  angle = OdomMath::constrainAngle180(270.0_deg + 360.0_deg);
  EXPECT_FLOAT_EQ(-90.0, angle.convert(degree));
  angle = OdomMath::constrainAngle180(180.0_deg);
  EXPECT_FLOAT_EQ(-180.0, angle.convert(degree));
  angle = OdomMath::constrainAngle180(181.0_deg);
  EXPECT_FLOAT_EQ(-179.0, angle.convert(degree));
}
