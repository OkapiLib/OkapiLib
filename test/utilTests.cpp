/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/util/mathUtil.hpp"
#include <gtest/gtest.h>

using namespace okapi;

TEST(IpowTest, IntegerTests) {
  EXPECT_EQ(ipow(0, 0), 1);
  EXPECT_EQ(ipow(0, 1), 0);
  EXPECT_EQ(ipow(1, 0), 1);
  EXPECT_EQ(ipow(1, 1), 1);
  EXPECT_EQ(ipow(2, 1), 2);
  EXPECT_EQ(ipow(2, 2), 4);
}

TEST(IpowTest, FloatingPointTests) {
  EXPECT_FLOAT_EQ(ipow(0.5, 1), 0.5);
  EXPECT_FLOAT_EQ(ipow(2.5, 2), 6.25);
}

TEST(CutRangeTest, Tests) {
  EXPECT_DOUBLE_EQ(cutRange(1, -2, 2), 2) << "1 : [-2, 2] -> 2";
  EXPECT_DOUBLE_EQ(cutRange(2, -2, 2), 2) << "2 : [-2, 2] -> 2";
  EXPECT_DOUBLE_EQ(cutRange(0, -2, 2), 2) << "0 : [-2, 2] -> 2";
  EXPECT_DOUBLE_EQ(cutRange(-2, -2, 2), -2) << "-2 : [-2, 2] -> -2";
  EXPECT_DOUBLE_EQ(cutRange(-3, -2, 2), -3) << "-3 : [-2, 2] -> -3";
  EXPECT_DOUBLE_EQ(cutRange(3, -2, 2), 3) << "3 : [-2, 2] -> 3";
}

TEST(DeadbandTest, Tests) {
  EXPECT_DOUBLE_EQ(deadband(0, -2, 2), 0) << "0 : [-2, 2] -> 0";
  EXPECT_DOUBLE_EQ(deadband(1, -2, 2), 0) << "1 : [-2, 2] -> 0";
  EXPECT_DOUBLE_EQ(deadband(2, -2, 2), 0) << "2 : [-2, 2] -> 0";
  EXPECT_DOUBLE_EQ(deadband(-2, -2, 2), 0) << "-2 : [-2, 2] -> 0";
  EXPECT_DOUBLE_EQ(deadband(3, -2, 2), 3) << "3 : [-2, 2] -> 3";
  EXPECT_DOUBLE_EQ(deadband(-3, -2, 2), -3) << "-3 : [-2, 2] -> -3";
}

TEST(RemapRangeTest, Tests) {
  EXPECT_FLOAT_EQ(remapRange(0, -1, 1, -2, 2), 0) << "0 : [-1, 1] -> [-2, 2] -> 0";
  EXPECT_FLOAT_EQ(remapRange(0.1, -1, 1, -2, 2), 0.2) << "0.1 : [-1, 1] -> [-2, 2] -> 0.2";
  EXPECT_FLOAT_EQ(remapRange(-0.1, -1, 1, 2, -2), 0.2) << "-0.1 : [-1, 1] -> [2, -2] -> 0.2";
  EXPECT_FLOAT_EQ(remapRange(0, -1, 1, -5, 2), -1.5) << "0 : [-1, 1] -> [-5, 2] -> -1.5";
}

TEST(TrueModTest, Tests) {
  EXPECT_EQ(modulus(0, 1), 0);
  EXPECT_EQ(modulus(1, 2), 1);
  EXPECT_EQ(modulus(-2, 5), 3);
  EXPECT_EQ(modulus(-1800, 3600), 1800);
  EXPECT_EQ(modulus(1, -3), -2);
}
