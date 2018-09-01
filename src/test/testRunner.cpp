/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/testRunner.hpp"
#include "okapi/impl/util/timer.hpp"

namespace okapi {
static size_t testPassCount = 0;
static size_t testFailCount = 0;
static std::vector<std::string> testFailLog; // Names of tests that have failed
static Timer testLengthTimer;                // Time since the first test

void test_printf(const std::string &istring) {
  printf("\n%s\n%s\n", istring.c_str(), std::string(istring.length(), '-').c_str());
}

void test(const std::string &iname, std::function<void()> ifunc) {
  testLengthTimer.placeHardMark(); // Hard mark for the first test

  try {
    ifunc();
    testPassCount++;
    printf(TEST_PRINT_GRN "Test passed:" TEST_PRINT_WHT " %s\n\n" TEST_PRINT_RESET, iname.c_str());
  } catch (const snowhouse::AssertionException &e) {
    testFailCount++;
    testFailLog.push_back(iname);
    printf(TEST_PRINT_RED "Test failed:" TEST_PRINT_WHT " %s" TEST_PRINT_RESET "\n%s\n",
           iname.c_str(),
           e.GetMessage().c_str());
  }

  testLengthTimer.placeMark(); // Mark for the end of the last test
}

void test_print_report() {
  printf(TEST_PRINT_GRN "%d tests finished," TEST_PRINT_RESET " took: %1.2f seconds\n",
         testPassCount + testFailCount,
         (testLengthTimer.getDtFromHardMark() - testLengthTimer.getDtFromMark()).convert(second));
  printf(TEST_PRINT_GRN "%d tests passed" TEST_PRINT_RESET "\n", testPassCount);
  printf(TEST_PRINT_RED "%d tests failed" TEST_PRINT_RESET "\n", testFailCount);
  if (testFailLog.size() > 0) {
    printf(TEST_PRINT_RED "Failed tests:\n" TEST_PRINT_RESET);
    for (auto &&elem : testFailLog) {
      printf("%s\n", elem.c_str());
    }
  }
}
} // namespace okapi
