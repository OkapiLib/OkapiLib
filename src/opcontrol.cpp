#include "main.h"
#include "okapi/api.hpp"

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
  auto drive = ChassisControllerFactory::create(-18, 19);
  Controller master;
  pros::c::lcd_initialize();
  while (true) {
    drive.tank(0.3, 0.3);
    // drive.forward(100);
    pros::c::lcd_print(0, "cnt %f", master.getAnalog(ControllerAnalog::rightY));
    pros::delay(50);
  }
}
