/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/filter/averageFilter.hpp"
#include "okapi/api/filter/composableFilter.hpp"
#include "okapi/api/filter/demaFilter.hpp"
#include "okapi/api/filter/ekfFilter.hpp"
#include "okapi/api/filter/emaFilter.hpp"
#include "okapi/api/filter/medianFilter.hpp"
#include "okapi/api/filter/passthroughFilter.hpp"
#include "okapi/api/filter/velMath.hpp"
#include "okapi/api/util/abstractTimer.hpp"
#include "test/crossPlatformTestRunner.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;
using namespace snowhouse;

void assertThatFilterAndFilterOutputAreEqual(Filter &filter, double input, double value) {
  EXPECT_NEAR(filter.filter(input), value, 0.0001);
  EXPECT_NEAR(filter.getOutput(), value, 0.0001);
}

TEST(AverageFilterTest, OutputTest) {
  AverageFilter<5> filter;

  for (int i = 0; i < 10; i++) {
    switch (i) {
    case 0: {
      assertThatFilterAndFilterOutputAreEqual(filter, i, 0);
      break;
    }

    case 1: {
      assertThatFilterAndFilterOutputAreEqual(filter, i, 0.2);
      break;
    }

    case 2: {
      assertThatFilterAndFilterOutputAreEqual(filter, i, 0.6);
      break;
    }

    case 3: {
      assertThatFilterAndFilterOutputAreEqual(filter, i, 1.2);
      break;
    }

    default: {
      assertThatFilterAndFilterOutputAreEqual(filter, i, i - 2);
      break;
    }
    }
  }
}

TEST(MedianFilterTest, OutputTest) {
  MedianFilter<5> filter;

  for (int i = 0; i < 10; i++) {
    if (i < 3) {
      assertThatFilterAndFilterOutputAreEqual(filter, i, 0);
    } else {
      assertThatFilterAndFilterOutputAreEqual(filter, i, i - 2);
    }
  }
}

TEST(EmaFilterTest, FloatingPointGainOutputTest) {
  EmaFilter filter(0.5);

  assertThatFilterAndFilterOutputAreEqual(filter, 0, 0);
  assertThatFilterAndFilterOutputAreEqual(filter, 1, 0.5);
  assertThatFilterAndFilterOutputAreEqual(filter, 2, 1.25);
  assertThatFilterAndFilterOutputAreEqual(filter, -3, -0.875);
}

TEST(EmaFilterTest, IntegerGainOutputTest) {
  EmaFilter filter(1);

  assertThatFilterAndFilterOutputAreEqual(filter, 5, 5);
  assertThatFilterAndFilterOutputAreEqual(filter, 6, 6);
  assertThatFilterAndFilterOutputAreEqual(filter, 7, 7);
}

TEST(EmaFilterTest, SetGainsTest) {
  EmaFilter filter(0);
  filter.setGains(1);

  assertThatFilterAndFilterOutputAreEqual(filter, 5, 5);
  assertThatFilterAndFilterOutputAreEqual(filter, 6, 6);
  assertThatFilterAndFilterOutputAreEqual(filter, 7, 7);
}

TEST(DemaFilterTest, FloatingPointGainOutputTest) {
  DemaFilter filter(0.5, 0.05);

  assertThatFilterAndFilterOutputAreEqual(filter, 0, 0);
  assertThatFilterAndFilterOutputAreEqual(filter, 1, 0.525);
  assertThatFilterAndFilterOutputAreEqual(filter, 2, 1.3244);
  assertThatFilterAndFilterOutputAreEqual(filter, 2, 1.7409);
  assertThatFilterAndFilterOutputAreEqual(filter, 2, 1.9557);
}

TEST(DemaFilterTest, IntegerGainOutputTest) {
  DemaFilter filter(1, 0);

  assertThatFilterAndFilterOutputAreEqual(filter, 5, 5);
  assertThatFilterAndFilterOutputAreEqual(filter, 6, 6);
  assertThatFilterAndFilterOutputAreEqual(filter, 7, 7);
}

TEST(DemaFilterTest, SetGainsTest) {
  DemaFilter filter(0, 1);
  filter.setGains(1, 0);

  assertThatFilterAndFilterOutputAreEqual(filter, 5, 5);
  assertThatFilterAndFilterOutputAreEqual(filter, 6, 6);
  assertThatFilterAndFilterOutputAreEqual(filter, 7, 7);
}

TEST(EKFFilterTest, OutputTest) {
  EKFFilter filter(0.0001, ipow(0.2, 2));

  assertThatFilterAndFilterOutputAreEqual(filter, 0, 0);
  assertThatFilterAndFilterOutputAreEqual(filter, 0.5, 0.2454);
  assertThatFilterAndFilterOutputAreEqual(filter, -0.5, -0.0008);
  assertThatFilterAndFilterOutputAreEqual(filter, 0.5, 0.1242);
  assertThatFilterAndFilterOutputAreEqual(filter, 0, 0.0992);
}

void testComposableFilterFunctionality(ComposableFilter &filter) {
  assertThatFilterAndFilterOutputAreEqual(filter, 1, 0.1111);
  assertThatFilterAndFilterOutputAreEqual(filter, 2, 0.4444);
  assertThatFilterAndFilterOutputAreEqual(filter, 3, 1.1111);

  for (int i = 4; i < 10; i++) {
    assertThatFilterAndFilterOutputAreEqual(filter, i, i - 2);
  }
}

TEST(ComposableFilterTest, OutputTest) {
  ComposableFilter filter(
    {std::make_shared<AverageFilter<3>>(), std::make_shared<AverageFilter<3>>()});

  testComposableFilterFunctionality(filter);
}

TEST(ComposableFilterTest, OutputWithNoFiltersTest) {
  ComposableFilter filter({});
  EXPECT_DOUBLE_EQ(filter.filter(1), 0);
  EXPECT_DOUBLE_EQ(filter.getOutput(), 0);
}

TEST(ComposableFilterTest, AddingAFilterIsEquivalentToCtorParam) {
  ComposableFilter filter(
    {std::make_shared<AverageFilter<3>>(), std::make_shared<AverageFilter<3>>()});

  ComposableFilter filterWithAdd({std::make_shared<AverageFilter<3>>()});
  filterWithAdd.addFilter(std::make_shared<AverageFilter<3>>());

  testComposableFilterFunctionality(filter);
  testComposableFilterFunctionality(filterWithAdd);
}

TEST(PassthroughFilterTest, OutputTest) {
  PassthroughFilter filter;

  for (int i = 0; i < 5; i++) {
    assertThatFilterAndFilterOutputAreEqual(filter, i, i);
  }
}

void testVelMathFunctionality(VelMath &velMath) {
  for (int i = 0; i < 10; i++) {
    if (i == 0) {
      EXPECT_NEAR(velMath.step(i * 10).convert(rpm), 0, 0.01);
      EXPECT_NEAR(velMath.getVelocity().convert(rpm), 0, 0.01);
    } else {
      // 10 ticks per 10 ms should be ~166.67 rpm
      EXPECT_NEAR(velMath.step(i * 10).convert(rpm), 166.67, 0.01);
      EXPECT_NEAR(velMath.getVelocity().convert(rpm), 166.67, 0.01);
    }
  }
}

TEST(VelMathTest, DtLessThanSampleTime) {
  VelMath velMath(
    360, std::make_shared<PassthroughFilter>(), 0_ms, std::make_unique<ConstantMockTimer>(10_ms));

  testVelMathFunctionality(velMath);
}

TEST(VelMathTest, DtEqualToSampleTime) {
  VelMath velMath(
    360, std::make_shared<PassthroughFilter>(), 10_ms, std::make_unique<ConstantMockTimer>(10_ms));

  testVelMathFunctionality(velMath);
}

TEST(VelMathTest, DtGreaterThanToSampleTime) {
  VelMath velMath(
    360, std::make_shared<PassthroughFilter>(), 11_ms, std::make_unique<ConstantMockTimer>(10_ms));

  EXPECT_EQ(velMath.step(10).convert(rpm), 0);
  EXPECT_EQ(velMath.getVelocity().convert(rpm), 0);

  EXPECT_EQ(velMath.step(20).convert(rpm), 0);
  EXPECT_EQ(velMath.getVelocity().convert(rpm), 0);
}

TEST(VelMathTest, SetTPRTest) {
  VelMath velMath(
    1, std::make_shared<PassthroughFilter>(), 0_ms, std::make_unique<ConstantMockTimer>(10_ms));
  velMath.setTicksPerRev(360);

  testVelMathFunctionality(velMath);
}

TEST(VelMathTest, ZeroTPRThrowsException) {
  EXPECT_THROW(
    VelMath(
      0, std::make_shared<PassthroughFilter>(), 0_ms, std::make_unique<ConstantMockTimer>(10_ms)),
    std::invalid_argument);
}
