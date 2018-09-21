#include "main.h"

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  using namespace okapi;

  pros::delay(100);

  auto velCnt =
    std::make_shared<AsyncVelIntegratedController>(AsyncControllerFactory::velIntegrated(1));

  auto mpCnt =
    AsyncLinearMotionProfileController(TimeUtilFactory::create(), 1.0, 2.0, 10.0, velCnt);
  mpCnt.startThread();

  Logger::initialize(TimeUtilFactory::create().getTimer(), "/ser/sout", Logger::LogLevel::debug);
  mpCnt.generatePath({0, 12}, "A");
  mpCnt.setTarget("A");
  mpCnt.waitUntilSettled();
}
