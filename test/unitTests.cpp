/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/units/QTime.hpp"
#include "test/crossPlatformTestRunner.hpp"
#include <gtest/gtest.h>

using namespace okapi;
using namespace snowhouse;

TEST(UnitTests, TimeAddition) {
  QTime start = 0_ms;

  EXPECT_FLOAT_EQ((start + 1_ms).convert(millisecond), (1_ms).convert(millisecond));
}

TEST(UnitTests, TimeAssignmentAddition) {
  QTime start = 0_ms;
  start += 1_ms;

  EXPECT_FLOAT_EQ(start.convert(millisecond), (1_ms).convert(millisecond));
}
