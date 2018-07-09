/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api.hpp"
#include "test/tests/api/allApiTests.hpp"
#include "test/tests/api/implMocks.hpp"

int main() {
  using namespace okapi;

  testChassisScales();
  testChassisModels();
  testControlUtils();
  testButtons();
  testFilters();
  testUtil();
  testControllers();

  test_print_report();

  auto motor = std::make_shared<MockMotor>();

  PIDTuner pidTuner(motor, std::make_unique<MockTimer>(), createSettledUtilPtr(),
                    std::make_unique<MockRate>(), 5_s, 1000, 0.1, 2.0, 0.0001, 0.01, 20.0, 40.0);

  auto out = pidTuner.autotune();
  printf("kP: %1.2f, kI: %1.2f, kD: %1.2f\n", out.kP, out.kI, out.kD);

  return static_cast<int>(test_query_report());
}
