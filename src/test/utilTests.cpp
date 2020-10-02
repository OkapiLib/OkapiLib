/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/impl/utilTests.hpp"
#include "test/testRunner.hpp"

using namespace okapi;
using namespace snowhouse;

static void testUtils() {
  {
    test_printf("Testing Rate");

    // Test the overload that takes QFrequency
    {
      printf("Testing QFrequency delay overload\n");

      Rate rate;
      uint32_t lastTime = pros::millis();

      for (int i = 0; i < 10; i++) {
        rate.delay(10_Hz);

        // Static cast so the compiler doesn't complain about comparing signed and unsigned values
        test("Rate " + std::to_string(i),
             TEST_BODY(AssertThat,
                       static_cast<double>(pros::millis() - lastTime),
                       EqualsWithDelta(100, 10)));

        lastTime = pros::millis();
        pros::Task::delay(50); // Emulate some computation
      }
    }

    // Test the overload that takes raw ms
    {
      printf("Testing raw ms delay overload\n");

      Rate rate;
      uint32_t lastTime = pros::millis();

      for (int i = 0; i < 10; i++) {
        rate.delayUntil(10_ms);

        // Static cast so the compiler doesn't complain about comparing signed and unsigned values
        test("Rate " + std::to_string(i),
             TEST_BODY(AssertThat,
                       static_cast<double>(pros::millis() - lastTime),
                       EqualsWithDelta(100, 10)));

        lastTime = pros::millis();
        pros::Task::delay(50); // Emulate some computation
      }
    }
  }

  {
    test_printf("Testing Timer");

    // Test the overload that takes QTime
    {
      printf("Testing QTime repeat overload\n");

      Timer timer;
      uint32_t lastTime = pros::millis();
      int successCount = 0;

      for (int i = 0; i < 10000 && successCount < 10; i++) {
        if (timer.repeat(100_ms)) {
          successCount++;

          // Static cast so the compiler doesn't complain about comparing signed and unsigned values
          test("Timer " + std::to_string(i),
               TEST_BODY(AssertThat,
                         static_cast<double>(pros::millis() - lastTime),
                         EqualsWithDelta(100, 10)));

          lastTime = pros::millis();
        }
        pros::Task::delay(1); // Emulate some computation
      }
    }

    // Test the overload that takes QFrequency
    {
      printf("Testing QFrequency repeat overload\n");

      Timer timer;
      uint32_t lastTime = pros::millis();
      int successCount = 0;

      for (int i = 0; i < 10000 && successCount < 10; i++) {
        if (timer.repeat(10_Hz)) {
          successCount++;

          // Static cast so the compiler doesn't complain about comparing signed and unsigned values
          test("Timer " + std::to_string(i),
               TEST_BODY(AssertThat,
                         static_cast<double>(pros::millis() - lastTime),
                         EqualsWithDelta(100, 10)));

          lastTime = pros::millis();
        }
        pros::Task::delay(1); // Emulate some computation
      }
    }

    {
      printf("Testing getDt and readDt\n");

      Timer timer;

      test("getDt should read zero the first time",
           TEST_BODY(AssertThat, timer.getDt().convert(millisecond), Equals(0)));

      pros::Task::delay(1000);

      test("readDt should read the same as getDt but not affect getDt",
           TEST_BODY(AssertThat, timer.readDt().convert(millisecond), EqualsWithDelta(1000, 5)));
      test("getDt should read the dt the second time",
           TEST_BODY(AssertThat, timer.getDt().convert(millisecond), EqualsWithDelta(1000, 5)));
    }
  }
}

void runUtilTests() {
  test_printf("Testing Util");
  testUtils();
}
