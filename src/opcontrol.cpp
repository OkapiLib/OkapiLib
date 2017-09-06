/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"
#include "chassis/odomChassisController.h"

using namespace okapi;

void operatorControl() {
	Encoder leftEnc = encoderInit(3, 4, true);
	Encoder rightEnc = encoderInit(1, 2, false);
	OdomChassisControllerPid chassis(OdomParams(
    SkidSteerModelParams<3>({4,5,6, 7,8,9}, //The six motor ports
                            leftEnc,   //Left encoder
                            rightEnc), //Right encoder
    0.716457354, //Distance scale (encoder ticks to mm)
    0.0270938),  //Turn scale (encoder ticks to deg)
  PidParams(0,0,0),  //Distance PID controller
  PidParams(0,0,0)); //Angle PID controller

	while (1) {
		// volatile int x = *((int*)0xFFFFFFFF) = 69;
		// printf("%d", x);
		// const auto state = chassis.getState();
		// printf("%1.2f, %1.2f, %1.2f\n", state.x, state.y, state.theta);
		taskDelay(100);
	}
}
