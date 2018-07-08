/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/api/filterTests.hpp"
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

void testFilters() {
  using namespace okapi;
  using namespace snowhouse;

  auto assertThatFilterAndFilterOutputAreEqual = [](okapi::Filter *filt, double input, double value,
                                                    double delta) {
    auto text = "Assert that filter and filter output are equal and correct for input = " +
                std::to_string(input);
    test(text, [&]() {
      AssertThat(filt->filter(input), EqualsWithDelta(value, delta));
      AssertThat(filt->getOutput(), EqualsWithDelta(value, delta));
    });
  };

  {
    test_printf("Testing AverageFilter");

    AverageFilter<5> filt;

    for (int i = 0; i < 10; i++) {
      auto testName = "AverageFilter i = " + std::to_string(i);
      switch (i) {
      case 0: {
        assertThatFilterAndFilterOutputAreEqual(&filt, i, 0, 0.0001);
        break;
      }

      case 1: {
        assertThatFilterAndFilterOutputAreEqual(&filt, i, 0.2, 0.0001);
        break;
      }

      case 2: {
        assertThatFilterAndFilterOutputAreEqual(&filt, i, 0.6, 0.0001);
        break;
      }

      case 3: {
        assertThatFilterAndFilterOutputAreEqual(&filt, i, 1.2, 0.0001);
        break;
      }

      default: {
        assertThatFilterAndFilterOutputAreEqual(&filt, i, i - 2, 0.0001);
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
        assertThatFilterAndFilterOutputAreEqual(&filt, i, 0, 0.0001);
      } else {
        assertThatFilterAndFilterOutputAreEqual(&filt, i, i - 2, 0.0001);
      }
    }
  }

  {
    test_printf("Testing EmaFilter");

    EmaFilter filt(0.5);
    assertThatFilterAndFilterOutputAreEqual(&filt, 0, 0, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 1, 0.5, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 2, 1.25, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, -3, -0.875, 0.0001);

    EmaFilter filt2(1);
    assertThatFilterAndFilterOutputAreEqual(&filt2, 5, 5, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt2, 6, 6, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt2, 7, 7, 0.0001);
  }

  {
    test_printf("Testing DemaFilter");

    DemaFilter filt(0.5, 0.05);

    assertThatFilterAndFilterOutputAreEqual(&filt, 0, 0, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 1, 0.525, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 2, 1.3244, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 2, 1.7410, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 2, 1.9557, 0.0001);

    DemaFilter filt2(1, 0);
    assertThatFilterAndFilterOutputAreEqual(&filt2, 5, 5, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt2, 6, 6, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt2, 7, 7, 0.0001);
  }

  {
    test_printf("Testing EKFFilter");

    EKFFilter filt(0.0001, ipow(0.2, 2));
    assertThatFilterAndFilterOutputAreEqual(&filt, 0, 0, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 0.5, 0.2454, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, -0.5, -0.0008, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 0.5, 0.1242, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 0, 0.0992, 0.0001);
  }

  {
    test_printf("Testing ComposableFilter");

    ComposableFilter filt(
      {std::make_shared<AverageFilter<3>>(), std::make_shared<AverageFilter<3>>()});
    assertThatFilterAndFilterOutputAreEqual(&filt, 1, 0.1111, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 2, 0.4444, 0.0001);
    assertThatFilterAndFilterOutputAreEqual(&filt, 3, 1.1111, 0.0001);

    for (int i = 4; i < 10; i++) {
      assertThatFilterAndFilterOutputAreEqual(&filt, i, i - 2, 0.0001);
    }
  }

  {
    test_printf("Testing PassthroughFilter");

    PassthroughFilter filt;
    for (int i = 0; i < 5; i++) {
      assertThatFilterAndFilterOutputAreEqual(&filt, i, i, 0.0001);
    }
  }

  {
    test_printf("Testing VelMath");

    {
      class MockTimer : public AbstractTimer {
        public:
        QTime millis() const override {
          return QTime();
        }

        QTime getStartingTime() const override {
          return QTime();
        }

        QTime getDtFromStart() const override {
          return QTime();
        }

        void placeMark() override {
        }

        void placeHardMark() override {
        }

        QTime clearHardMark() override {
          return second;
        }

        QTime getDtFromMark() const override {
          return QTime();
        }

        QTime getDtFromHardMark() const override {
          return QTime();
        }

        bool repeat(const QTime time) override {
          return false;
        }

        bool repeat(const QFrequency frequency) override {
          return false;
        }

        QTime getDt() override {
          return 10_ms;
        }
      };

      VelMath velMath(360, std::make_shared<PassthroughFilter>(), std::make_unique<MockTimer>());

      for (int i = 0; i < 10; i++) {
        if (i == 0) {
          test("VelMath " + std::to_string(i), [&]() {
            AssertThat(velMath.step(i * 10).convert(rpm), EqualsWithDelta(0, 0.01));
            AssertThat(velMath.getVelocity().convert(rpm), EqualsWithDelta(0, 0.01));
          });
        } else {
          // 10 ticks per 100 ms should be ~16.67 rpm
          test("VelMath " + std::to_string(i), [&]() {
            AssertThat(velMath.step(i * 10).convert(rpm), EqualsWithDelta(166.67, 0.01));
            AssertThat(velMath.getVelocity().convert(rpm), EqualsWithDelta(166.67, 0.01));
          });
        }
      }
    }
  }
}