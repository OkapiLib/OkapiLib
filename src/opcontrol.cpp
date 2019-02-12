#include "main.h"
#include "test/tests/impl/allImplTests.hpp"

using namespace okapi;
void opcontrol() {
  pros::delay(100);

  auto drive = ChassisControllerBuilder()
                 .withMotors(1, 2)
                 .withSensors({'A', 'B'}, {'E', 'F'}, {'C', 'D'})
                 .withDimensions({{4.125_in, 6_in}, quadEncoderTPR})
                 .withOdometry()
                 .buildOdometry();

  while (true) {
    auto state = drive->getState();
    printf("x: %1.2f, y: %1.2f, theta: %1.2f\n",
           state.x.convert(inch),
           state.y.convert(inch),
           state.theta.convert(degree));
    pros::delay(50);
  }
}
