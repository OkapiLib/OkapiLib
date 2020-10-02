/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QArea.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QTime.hpp"
#include "okapi/api/units/QVolume.hpp"
#include <gtest/gtest.h>

using namespace okapi;

TEST(UnitTests, TimeAddition) {
  QTime start = 0_ms;

  EXPECT_DOUBLE_EQ((start + 1_ms).convert(millisecond), (1_ms).convert(millisecond));
}

TEST(UnitTests, TimeAssignmentAddition) {
  QTime start = 0_ms;
  start += 1_ms;

  EXPECT_DOUBLE_EQ(start.convert(millisecond), (1_ms).convert(millisecond));
}

TEST(UnitTests, AbsTest) {
  EXPECT_DOUBLE_EQ(QLength(-3.0).abs().getValue(), 3.0);
  EXPECT_DOUBLE_EQ((-3.0 * inch).abs().convert(meter), (3.0_in).convert(meter));
}

TEST(UnitTests, UnaryMinusTest) {
  EXPECT_EQ((1_ft - -4_ft).convert(foot), 5);
  EXPECT_EQ((1_ft - 4_ft).convert(foot), -3);
  EXPECT_EQ((1_s + -500_ms).convert(millisecond), 500);

  // Make sure there are no side-effects
  auto test = 5_in;
  -test;
  EXPECT_NE(test.convert(inch), -5);
}

TEST(UnitTests, SqrtTest) {
  EXPECT_DOUBLE_EQ(std::sqrt(2), (2 * meter2).sqrt().convert(meter));
}

TEST(UnitTests, AbsFreeFunction) {
  EXPECT_EQ(abs(3.0_m), 3.0_m);
  EXPECT_EQ(abs(-3.0_m), 3.0_m);
}

TEST(UnitTests, PowRootTest) {
  EXPECT_DOUBLE_EQ((pow<2>(3_in)).convert(inch2), 9.0);
  EXPECT_DOUBLE_EQ(root<3>(3_in * 3_in * 3_in).convert(inch), 3.0);

  RQuantity three_halves_power = pow<std::ratio<3, 2>>(3_in);
  RQuantity cube_of_sqrt = pow<3>(root<2>(3_in));
  EXPECT_DOUBLE_EQ(three_halves_power.getValue(), cube_of_sqrt.getValue());
}

TEST(UnitTests, SquareCubeTest) {
  EXPECT_EQ(square(QLength(3.0)), QArea(9.0));
  EXPECT_EQ(cube(QLength(4.0)), QVolume(64.0));
  EXPECT_EQ(sqrt(QArea(25.0)), QLength(5.0));
  EXPECT_EQ(cbrt(QVolume(8.0)), QLength(2.0));
  EXPECT_EQ(hypot(QLength(3.0), QLength(4.0)), QLength(5.0));
}

TEST(UnitTests, ModTest) {
  EXPECT_DOUBLE_EQ(mod(QLength(4.0), QLength(2.0)).convert(meter), 0.0);
  EXPECT_DOUBLE_EQ(mod(10_in, 3_in).convert(inch), 1.0);
}

TEST(UnitTests, CopysignTest) {
  EXPECT_EQ(copysign(3_m, -4_in), -3_m);
  EXPECT_EQ(copysign(1_s, 5_m), 1_s);
}

TEST(UnitTests, RoundTest) {
  EXPECT_DOUBLE_EQ(ceil(4.1_in, inch).convert(inch), 5.0);
  EXPECT_DOUBLE_EQ(ceil(-4.1_in, inch).convert(inch), -4.0);

  EXPECT_DOUBLE_EQ(floor(1.17_s, 100_ms).convert(second), 1.1);
  EXPECT_DOUBLE_EQ(floor(-1.17_s, 100_ms).convert(second), -1.2);

  EXPECT_DOUBLE_EQ(trunc(5.8_cm, centimeter).convert(centimeter), 5.0);
  EXPECT_DOUBLE_EQ(trunc(-5.8_cm, centimeter).convert(centimeter), -5.0);

  EXPECT_DOUBLE_EQ(round(2.4_in, inch).convert(inch), 2.0);
  EXPECT_DOUBLE_EQ(round(2.6_in, inch).convert(inch), 3.0);
  EXPECT_DOUBLE_EQ(round(-2.4_in, inch).convert(inch), -2.0);
  EXPECT_DOUBLE_EQ(round(-2.6_in, inch).convert(inch), -3.0);
}

TEST(UnitTests, TrigTest) {
  EXPECT_DOUBLE_EQ(sin(30_deg).convert(number), 0.5);
  EXPECT_DOUBLE_EQ(cos(60_deg).convert(number), 0.5);
  EXPECT_DOUBLE_EQ(tan(45_deg).convert(number), 1.0);
}

TEST(UnitTests, InverseTrigTest) {
  EXPECT_DOUBLE_EQ(asin(Number(0.5)).convert(degree), 30.0);
  EXPECT_DOUBLE_EQ(acos(Number(0.5)).convert(degree), 60.0);
  EXPECT_DOUBLE_EQ(atan(Number(1.0)).convert(degree), 45.0);
}

TEST(UnitTests, HyperbTest) {
  EXPECT_DOUBLE_EQ(sinh(37_deg).convert(number), 0.6916004653045855);
  EXPECT_DOUBLE_EQ(cosh(37_deg).convert(number), 1.2158582169025791);
  EXPECT_DOUBLE_EQ(tanh(37_deg).convert(number), 0.5688167055090109);
}

TEST(UnitTests, InverseHyperbTest) {
  EXPECT_DOUBLE_EQ(asinh(Number(0.6916004653045855)).convert(degree), 37.0);
  EXPECT_DOUBLE_EQ(acosh(Number(1.2158582169025791)).convert(degree), 37.0);
  EXPECT_DOUBLE_EQ(atanh(Number(0.5688167055090109)).convert(degree), 37.0);
}

TEST(UnitTests, Atan2Test) {
  EXPECT_DOUBLE_EQ(atan2(1_ft, 2_ft).convert(radian), 0.4636476090008061);
  EXPECT_DOUBLE_EQ(atan2(1_ft, -2_ft).convert(radian), 2.677945044588987);
  EXPECT_DOUBLE_EQ(atan2(-1_ft, -2_ft).convert(radian), -2.677945044588987);
  EXPECT_DOUBLE_EQ(atan2(-1_ft, 2_ft).convert(radian), -0.4636476090008061);
}
