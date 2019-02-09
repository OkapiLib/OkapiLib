#include "main.h"
#include "pros/apix.h"
#include "test/tests/impl/allImplTests.hpp"

using namespace okapi;

void task1(void *) {
  auto drive = ChassisControllerBuilder().withMotors(-18, 19).withGains({}, {}).build();
  pros::delay(100);
}

void opcontrol() {
  pros::delay(100);
  pros::Task t1(task1);

  pros::delay(50);
}
