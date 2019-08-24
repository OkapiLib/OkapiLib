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

const int DRIVE_MOTOR_LEFT = 1;
const int DRIVE_MOTOR_RIGHT = 2;

const double liftkP = 0.001;
const double liftkI = 0.0001;
const double liftkD = 0.0001;
const int LIFT_MOTOR = 2;

void opcontrol() {
  pros::delay(100);

  auto driveController =
    ChassisControllerBuilder().withMotors(DRIVE_MOTOR_LEFT, -DRIVE_MOTOR_RIGHT).build();

  auto liftController =
    AsyncPosControllerBuilder().withMotor(LIFT_MOTOR).withGains({liftkP, liftkI, liftkD}).build();

  driveController->moveDistanceAsync(1000); // Move 1000 motor degrees forward
  liftController->setTarget(200); // Move 200 motor degrees upward
  driveController->waitUntilSettled();
  liftController->waitUntilSettled();

  Logger::setDefaultLogger(
    std::make_shared<Logger>(std::make_unique<Timer>(), "/ser/sout", Logger::LogLevel::debug));

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

  pros::Task printSensorValsTask(printSensorVals);

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
