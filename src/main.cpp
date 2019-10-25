#include "main.h"
#include "test/tests/impl/allImplTests.hpp"

using namespace okapi;

std::shared_ptr<OdomChassisController> drive;

void printSensorVals(void *) {
  while (true) {
    // auto state = drive->model().getSensorVals();
    // printf("left: %ld, right: %ld\n", state[0], state[1]);
    auto state = drive->getState();
    printf("x=%f, y=%f, theta=%f\n",
           state.x.convert(inch),
           state.y.convert(inch),
           state.theta.convert(degree));
    pros::delay(50);
  }
}

void opcontrol() {
  pros::delay(100);

  Logger::setDefaultLogger(std::make_shared<Logger>(
    TimeUtilFactory::createDefault().getTimer(), "/ser/sout", Logger::LogLevel::debug));

  drive = ChassisControllerBuilder()
            .withMotors({-18, 19, 20}, {16, -17, -14})
            .withDimensions({{4.1_in, 11.375_in}, imev5GreenTPR})
            // .withDimensions({{3.125_in, 11.375_in}, 4096})
            // .withGains({0.006, 0, 0.0001}, {0.006, 0, 0.0001})
            // .withSensors({'G', 'H'}, {'E', 'F'})
            //            .withLogger(std::make_shared<Logger>(
            //              std::make_unique<Timer>(), "/ser/sout", Logger::LogLevel::debug))
            .withMaxVelocity(60)
            .withOdometry(StateMode::FRAME_TRANSFORMATION) // StateMode::CARTESIAN
            .buildOdometry();

  pros::Task printSensorValsTask(printSensorVals, NULL, "");

  drive->driveToPoint({6_in, 2_in});
  drive->driveToPoint({6_in, 6_in});
  drive->driveToPoint({0_in, 0_in}, true);
  // drive->turnToPoint({6_in, 2_in});
  // drive->turnToAngle(90_deg);
  // drive->moveDistance(6_in);
  // drive->turnAngle(90_deg);
  // drive->moveDistance(6_in);

  while (true) {
    pros::delay(50);
  }
}

void initialize() {
}

void disabled() {
}

void competition_initialize() {
}

void autonomous() {
}
