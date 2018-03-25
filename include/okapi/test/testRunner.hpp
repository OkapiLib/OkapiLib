/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_TESTRUNNER_HPP_
#define _OKAPI_TESTRUNNER_HPP_

#include "api.h"
#include "okapi/snowhouse/snowhouse.h"
#include <functional>
#include <string>

#define TEST_PRINT_RED "\x1B[31m"
#define TEST_PRINT_GRN "\x1B[32m"
#define TEST_PRINT_YEL "\x1B[33m"
#define TEST_PRINT_BLU "\x1B[34m"
#define TEST_PRINT_MAG "\x1B[35m"
#define TEST_PRINT_CYN "\x1B[36m"
#define TEST_PRINT_WHT "\x1B[37m"
#define TEST_PRINT_RESET "\x1B[0m"

#define TEST_BODY(FUNCTION, ...) [&]() { FUNCTION(__VA_ARGS__); }

namespace okapi {
static void test_printf(const std::string &istring) {
  printf("\n%s\n%s\n", istring.c_str(), std::string(istring.length(), '-').c_str());
}

static void test(const std::string &iname, std::function<void()> ifunc) {
  try {
    ifunc();
    printf(TEST_PRINT_GRN "Test passed:" TEST_PRINT_WHT " %s\n\n" TEST_PRINT_RESET, iname.c_str());
  } catch (const snowhouse::AssertionException &e) {
    printf(TEST_PRINT_RED "Test failed:" TEST_PRINT_WHT " %s" TEST_PRINT_RESET "\n%s\n",
           iname.c_str(), e.GetMessage().c_str());
  }
}
} // namespace okapi

#endif
