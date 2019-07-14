#include "main.h"
#include "test/tests/impl/allImplTests.hpp"

using namespace okapi;

std::shared_ptr<OdomChassisController> drive;

void printSensorVals(void *) {
  while (true) {
    //    auto state = drive->model().getSensorVals();
    //    printf("left: %ld, right: %ld\n", state[0], state[1]);
    auto state = drive->getState(StateMode::FRAME_TRANSFORMATION);
    printf("x=%f, y=%f, theta=%f\n",
           state.x.convert(inch),
           state.y.convert(inch),
           state.theta.convert(degree));
    pros::delay(50);
  }
}

void opcontrol() {
  pros::delay(100);

  drive = ChassisControllerBuilder()
            .withMotors({-18, 19, 20}, {16, -17, -14})
            .withDimensions({{4.1_in, 11.375_in}, imev5GreenTPR})
            .withGains({0.006, 0, 0.0001}, {0.006, 0, 0.0001})
            // .withSensors({'E', 'F'}, {'G', 'H'})
            .withLogger(std::make_shared<Logger>(
              std::make_unique<Timer>(), "/ser/sout", Logger::LogLevel::debug))
            .withMaxVelocity(100)
            .withOdometry()
            .buildOdometry();

  pros::Task printSensorValsTask(printSensorVals);

  drive->moveDistance(6_in);
  drive->turnAngle(90_deg);
  drive->moveDistance(6_in);

  while (true) {
    pros::delay(50);
  }
}
