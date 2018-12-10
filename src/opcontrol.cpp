#include "main.h"
#include "test/tests/impl/allImplTests.hpp"

using namespace okapi;

void fooFunc(void *) {
  auto drive = ChassisControllerBuilder().withMotors(1, 1).withGains({}, {}).build();
  pros::delay(1000);
}

void opcontrol() {
  pros::delay(100);
  CrossplatformThread fooTask(fooFunc, nullptr);
  pros::delay(300);
  pros::c::task_delete(fooTask.thread);
  pros::delay(10000);
}
