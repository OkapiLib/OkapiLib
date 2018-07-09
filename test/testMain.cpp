/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/api/allApiTests.hpp"

int main() {
  using namespace okapi;

  testChassisScales();
  testChassisModels();
  testControlUtils();
  testButtons();
  testFilters();
  testUtil();
  testControllers();
  testOdometry();

  test_print_report();

  return static_cast<int>(test_query_report());
}
