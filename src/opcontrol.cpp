/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "api.h"

//#include "okapi/api.hpp"
//#include "test/testRunner.hpp"
//#include "test/tests/impl/allImplTests.hpp"

#include "ros_lib/rosserial_vex_v5/examples/helloworld.h"

void runHeadlessTests() {
  // using namespace okapi;

  // runHeadlessUtilTests();

  // test_print_report();
}

void opcontrol() {
  setup();
  // while(1) {
  //   pros::c::delay(200);
  //   printf("hi\r\n");
  // }

  /* 
   *
   * test code
   * 
   *
   * using namespace okapi;
   *   pros::Task::delay(100);
   * 
   *   runHeadlessTests();
   *   return;
   * 
   *   MotorGroup leftMotors({19_mtr, 20_mtr});
   *   MotorGroup rightMotors({13_rmtr, 14_rmtr});
   *   Motor armMotor = 15_mtr;
   *   armMotor.move(10);
   * 
   *   auto chassis =
   *     ChassisControllerFactory::create({19, 20}, {-14}, AbstractMotor::gearset::red, {4_in, 11.5_in});
   * 
   *   Controller controller;
   *   ControllerButton btn1(E_CONTROLLER_DIGITAL_A);
   *   ControllerButton btn2(E_CONTROLLER_DIGITAL_B);
   *   ControllerButton btn3(E_CONTROLLER_DIGITAL_Y);
   *   ControllerButton btn4(E_CONTROLLER_DIGITAL_X);
   * 
   *   while (true) {
   *     chassis.arcade(controller.getAnalog(E_CONTROLLER_ANALOG_LEFT_Y),
   *                    controller.getAnalog(E_CONTROLLER_ANALOG_LEFT_X));
   * 
   *     if (btn1.changedToPressed()) {
   *       printf("move distance\n");
   *       chassis.moveDistance(12_in);
   *     }
   * 
   *     if (btn2.changedToPressed()) {
   *       printf("turn angle\n");
   *       chassis.turnAngle(90_deg);
   *     }
   * 
   *     if (btn3.changedToPressed()) {
   *       printf("move arm\n");
   *       armMotor.moveRelative(-10, 127);
   *     }
   * 
   *     if (btn4.changedToPressed()) {
   *       printf("autonomous routine\n");
   *       for (int i = 0; i < 4; i++) {
   *         chassis.moveDistance(12_in);
   *         chassis.turnAngle(90_deg);
   *       }
   *     }
   * 
   *     pros::Task::delay(10);
   *   }
  */
}
