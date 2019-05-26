#include "main.h"
#include "test/tests/impl/allImplTests.hpp"

using namespace okapi;

std::shared_ptr<ChassisController> drive;

void printSensorVals(void *) {
  while (true) {
    auto state = drive->getSensorVals();
    printf("left: %ld, right: %ld\n", state[0], state[1]);
    printf("printSensorVals %d\n", errno);
    pros::delay(50);
  }
}

void opcontrol() {
  pros::delay(100);

  drive = ChassisControllerBuilder()
    .withMotors({16, -17, -14}, {-18, 19, 20})
    .withDimensions({{4.1_in, 11.375_in}, imev5GreenTPR})
    .withGains({0.01}, {})
    .withSensors({'D', 'E'}, {'G', 'H'})
      //    .withLogger(
      //      std::make_shared<Logger>(std::make_unique<Timer>(), "/ser/sout",
      //      Logger::LogLevel::debug))
    .build();

  pros::Task printSensorValsTask(printSensorVals);

  while (true) {
    printf("opcontrol %d\n", errno);
    pros::delay(50);
  }

  //  drive->setMaxVelocity(40);
  //  drive->moveDistance(-6_in);
  //  ADIEncoder enc('E', 'F');
  //  while (true) {
  //    printf("%f\n", enc.get());
  //    pros::delay(50);
  //  }
}
