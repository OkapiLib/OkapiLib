/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_FILTERTESTS_HPP_
#define _OKAPI_FILTERTESTS_HPP_

#include "okapi/api.hpp"
#include "test/testRunner.hpp"

void testFilters() {
  using namespace okapi;
  using namespace snowhouse;

  {
    test_printf("Testing AverageFilter");

    AverageFilter<5> filt;

    for (int i = 0; i < 10; i++) {
      auto testName = "AverageFilter i = " + std::to_string(i);
      switch (i) {
      case 0: {
        test(testName, TEST_BODY(AssertThat, filt.filter(i), Equals(0)));
        break;
      }

      case 1: {
        test(testName, TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(0.2, 0.01)));
        break;
      }

      case 2: {
        test(testName, TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(0.6, 0.01)));
        break;
      }

      case 3: {
        test(testName, TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(1.2, 0.01)));
        break;
      }

      default: {
        test(testName, TEST_BODY(AssertThat, filt.filter(i), Equals(i - 2)));
        break;
      }
      }
    }
  }

  {
    test_printf("Testing MedianFilter");

    MedianFilter<5> filt;

    for (int i = 0; i < 10; i++) {
      auto testName = "MedianFilter i = " + std::to_string(i);
      if (i < 3) {
        test(testName, TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(0, 0.0001)));
      } else {
        test(testName, TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(i - 2, 0.0001)));
      }
    }
  }

  {
    test_printf("Testing EmaFilter");

    EmaFilter filt(0.5);

    test("EmaFilter i = 0", TEST_BODY(AssertThat, filt.filter(0), EqualsWithDelta(0, 0.0001)));
    test("EmaFilter i = 1", TEST_BODY(AssertThat, filt.filter(1), EqualsWithDelta(0.5, 0.0001)));
    test("EmaFilter i = 2", TEST_BODY(AssertThat, filt.filter(2), EqualsWithDelta(1.25, 0.0001)));
    test("EmaFilter i = -3",
         TEST_BODY(AssertThat, filt.filter(-3), EqualsWithDelta(-0.875, 0.0001)));

    EmaFilter filt2(1);
    test("EmaFilter with alpha = 1 should return input signal 1",
         TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
    test("EmaFilter with alpha = 1 should return input signal 2",
         TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
    test("EmaFilter with alpha = 1 should return input signal 3",
         TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
  }

  {
    test_printf("Testing DemaFilter");

    DemaFilter filt(0.5, 0.05);

    test("DemaFilter i = 0", TEST_BODY(AssertThat, filt.filter(0), EqualsWithDelta(0, 0.0001)));
    test("DemaFilter i = 1", TEST_BODY(AssertThat, filt.filter(1), EqualsWithDelta(0.525, 0.0001)));
    test("DemaFilter i = 2",
         TEST_BODY(AssertThat, filt.filter(2), EqualsWithDelta(1.3244, 0.0001)));
    test("DemaFilter i = 2",
         TEST_BODY(AssertThat, filt.filter(2), EqualsWithDelta(1.7410, 0.0001)));
    test("DemaFilter i = 2",
         TEST_BODY(AssertThat, filt.filter(2), EqualsWithDelta(1.9557, 0.0001)));

    DemaFilter filt2(1, 0);
    test("DemaFilter with alpha = 1 and beta = 0 should return input signal 1",
         TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
    test("DemaFilter with alpha = 1 and beta = 0 should return input signal 2",
         TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
    test("DemaFilter with alpha = 1 and beta = 0 should return input signal 3",
         TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
  }

  {
    test_printf("Testing EKFFilter");

    EKFFilter filt(0.0001, ipow(0.2, 2));

    test("EKFFilter i = 0", TEST_BODY(AssertThat, filt.filter(0), EqualsWithDelta(0, 0.0001)));
    test("EKFFilter i = 0.5",
         TEST_BODY(AssertThat, filt.filter(0.5), EqualsWithDelta(0.2454, 0.0001)));
    test("EKFFilter i = -0.5",
         TEST_BODY(AssertThat, filt.filter(-0.5), EqualsWithDelta(-0.0008, 0.0001)));
    test("EKFFilter i = 0.5",
         TEST_BODY(AssertThat, filt.filter(0.5), EqualsWithDelta(0.1242, 0.0001)));
    test("EKFFilter i = 0", TEST_BODY(AssertThat, filt.filter(0), EqualsWithDelta(0.0992, 0.0001)));
  }

  {
    test_printf("Testing ComposableFilter");

    ComposableFilter filt(
      {std::make_shared<AverageFilter<3>>(), std::make_shared<AverageFilter<3>>()});

    test("ComposableFilter i = 1",
         TEST_BODY(AssertThat, filt.filter(1), EqualsWithDelta(0.1111, 0.0001)));
    test("ComposableFilter i = 2",
         TEST_BODY(AssertThat, filt.filter(2), EqualsWithDelta(0.4444, 0.0001)));
    test("ComposableFilter i = 3",
         TEST_BODY(AssertThat, filt.filter(3), EqualsWithDelta(1.1111, 0.0001)));

    for (int i = 4; i < 10; i++) {
      test("ComposableFilter i = " + std::to_string(i),
           TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(i - 2, 0.0001)));
    }
  }

  {
    test_printf("Testing PassthroughFilter");

    PassthroughFilter filt;

    for (int i = 0; i < 5; i++) {
      test("PassthroughFilter i = " + std::to_string(i),
           TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(i, 0.0001)));
    }
  }

  {
    test_printf("Testing VelMath");

    class MockTimer : public Timer {
      public:
      using Timer::Timer;
      virtual QTime getDt() override {
        return 10_ms;
      }
    };

    VelMath velMath(360, std::make_shared<PassthroughFilter>(), std::make_unique<MockTimer>());

    for (int i = 0; i < 10; i++) {
      if (i == 0) {
        test("VelMath " + std::to_string(i),
             TEST_BODY(AssertThat, velMath.step(i * 10), EqualsWithDelta(0, 0.01)));
      } else {
        // 10 ticks per 100 ms should be ~16.67 rpm
        test("VelMath " + std::to_string(i),
             TEST_BODY(AssertThat, velMath.step(i * 10), EqualsWithDelta(166.67, 0.01)));
      }
    }
  }
}

void runHeadlessFilterTests() {
  testFilters();
}

#endif
