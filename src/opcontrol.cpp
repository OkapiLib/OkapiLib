#include "main.h"
#include "test/tests/impl/allImplTests.hpp"

using namespace okapi;
void opcontrol() {
  auto drive = ChassisControllerBuilder()
                 .withMotors(1, -2) // Left motor is 1, right motor is 2 (reversed)
                 .withLogger(std::make_shared<Logger>(
                   TimeUtilFactory::create().getTimer(), "/ser/sout", Logger::LogLevel::debug))
                 .build();
}
