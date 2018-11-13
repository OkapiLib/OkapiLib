#include "main.h"
#include "test/tests/impl/allImplTests.hpp"

using namespace okapi;
void opcontrol() {
  runAllImplTests();

  ChassisControllerBuilder()
    .withMotors({1, 1}, {1, 1})
    .withSensors({'A', 'A'}, {'A', 'A'})
    .withGains({0, 0, 0}, {0, 0, 0}, {0, 0, 0})
    .withGearset(AbstractMotor::gearset::red * 1)
    .withDimensions({4_in, 10_in})
    .withMaxVelocity(600)
    .withMaxVoltage(12000)
    .build();
}
