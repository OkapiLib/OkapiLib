/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CROSSPLATFORMTESTRUNNER_HPP_
#define _OKAPI_CROSSPLATFORMTESTRUNNER_HPP_

#include "test/FakeIt/single_header/standalone/fakeit.hpp"
#include "test/snowhouse/snowhouse.h"
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

enum class TestSuiteResult { SUCCESS = 0, FAIL = 1 };

/**
 * Print the input string with an underline made from hypens ("-").
 *
 * @param istring string to print
 */
void test_printf(const std::string &istring);

/**
 * Run a test. The lambda can have any body, or it can be a sinlge function call. In that case, use
 * TEST_BODY for a more succinct way of writing the test.
 *
 * @param iname test name (describe what it does)
 * @param ifunc test body
 */
void test(const std::string &iname, std::function<void()> ifunc);

/**
 * Print out a test report detailing how long the tests took to run; how many tests passed; how many
 * tests failed; and if any failed, which ones.
 */
void test_print_report();

/**
 * Get the test report for the suite.
 *
 * @return the test suite result
 */
TestSuiteResult test_query_report();
} // namespace okapi

#endif
