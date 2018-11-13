#include "main.h"
#include "okapi/impl/chassis/controller/chassisControllerBuilder.hpp"
#include "test/tests/impl/allImplTests.hpp"

using namespace okapi;
void opcontrol() {
  runAllImplTests();

  auto drive = ChassisControllerBuilder()
                 .withMotors({1, 1}, {1, 1})
                 .withSensors({'A', 'A'}, {'A', 'A'})
                 .withGains({0, 0, 0}, {0, 0, 0}, {0, 0, 0})
                 .withGearset(AbstractMotor::gearset::blue * 1.5)
                 .withScales({4_in, 10_in})
                 .build();
}
