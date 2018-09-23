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

  auto drive = ChassisControllerFactory::create(-18, 19, AbstractMotor::gearset::green, {1, 1});
  drive.forward(0.1);
  Motor mtr(-18);
  while (true) {
    printf("%1.2f\n", mtr.getActualVelocity());
    pros::delay(10);
  }
}
