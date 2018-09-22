#include "api.h"
#include "okapi/api.hpp"

#include "test/testRunner.hpp"
#include "test/tests/impl/utilTests.hpp"

void runHeadlessTests() {
  using namespace okapi;

  runHeadlessUtilTests();

  test_print_report();
}

void opcontrol() {
  using namespace okapi;
  pros::Task::delay(100);

  //  Logger::initialize(std::make_unique<Timer>(), "/ser/sout", Logger::LogLevel::debug);
  //  auto logger = Logger::instance();

  auto drive = ChassisControllerFactory::createOdom(
    -1, 2, AbstractMotor::gearset::red, {2.5_in, 10.5_in}, 0_mm);
  drive.driveToPoint(4_in, 0_in);

  auto state = drive.getState();
  printf("x: %1.2f, y: %1.2f, theta: %1.2f\n",
         state.x.convert(inch),
         state.y.convert(inch),
         state.theta.convert(degree));

  drive.driveToPoint(4_in, 4_in);
  state = drive.getState();
  printf("x: %1.2f, y: %1.2f, theta: %1.2f\n",
         state.x.convert(inch),
         state.y.convert(inch),
         state.theta.convert(degree));

  drive.driveToPoint(0_in, 4_in);
  state = drive.getState();
  printf("x: %1.2f, y: %1.2f, theta: %1.2f\n",
         state.x.convert(inch),
         state.y.convert(inch),
         state.theta.convert(degree));

  drive.driveToPoint(0_in, 0_in);
  state = drive.getState();
  printf("x: %1.2f, y: %1.2f, theta: %1.2f\n",
         state.x.convert(inch),
         state.y.convert(inch),
         state.theta.convert(degree));

  pros::Task::delay(500);
}
