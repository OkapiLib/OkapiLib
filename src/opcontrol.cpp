#include "main.h"
#include "test/tests/impl/allImplTests.hpp"

using namespace okapi;
void opcontrol() {
  pros::delay(100);
  runAllImplTests();
}
