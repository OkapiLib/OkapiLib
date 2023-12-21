/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "main.h"
#include "pros/apix.h"
#include "unity/unity.h"

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void test_function_should_doBlahAndBlah(void) {
  // test stuff
}

void test_function_should_doAlsoDoBlah(void) {
  // more test stuff
}

// not needed when using generate_test_runner.rb
void opcontrol() {
  pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
  pros::delay(10000);
  UNITY_BEGIN();
  RUN_TEST(test_function_should_doBlahAndBlah);
  RUN_TEST(test_function_should_doAlsoDoBlah);
  UNITY_END();
}

void initialize() {
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
}