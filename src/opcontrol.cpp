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

#include <cmath>
#include "main.h"
#include "chassis/basicChassisController.h"
#include "util/timer.h"
#include "util/mathUtil.h"
#include "control/pid.h"
#include "control/nsPid.h"
#include "control/velPid.h"

using namespace okapi;

float measureRPM(const unsigned char motorPort, const Encoder enc1, const Encoder enc2) {
	using namespace std;

	encoderReset(enc1);
	encoderReset(enc2);

	Timer timer;
	timer.placeMark();

	while (timer.getDtFromMark() < 5000) {
		motorSet(motorPort, 127);
		printf("%d,%d\n",encoderGet(enc1),encoderGet(enc2));
		taskDelay(15);
	}

	motorSet(motorPort, 0);

	if (abs(encoderGet(enc1)) > abs(encoderGet(enc2)))
		return fabs(((float)encoderGet(enc1) / 360.0) * 12.0);

	return fabs(((float)encoderGet(enc2) / 360.0) * 12.0);
}

void operatorControl() {
	using namespace std; //Needed to get round to compile

	Encoder leftEnc = encoderInit(1, 2, false);
	Encoder rightEnc = encoderInit(3, 4, false);
	ChassisControllerPid controller(SkidSteerModelParams<3>({2_m,3_m,4_m, 5_m,6_m,7_m}, leftEnc, rightEnc), PidParams(0.15, 0.05, 0.07), PidParams(0.02, 0.01, 0));

	const unsigned char liftLeft = 8, liftRight = 9, liftPot = 1;

  NsPid liftPid(PidParams(0.2, 0.1, 0.1), VelMathParams(360), 0.5);

	constexpr int liftUpTarget = 2570, lift34 = 300, liftDownTarget = 10;
	int target = liftUpTarget;

	// constexpr unsigned char motor1 = 2, motor2 = 3;
	// lcdSetBacklight(uart1, true);

	int power = 0;
	VelMath vm(360);

	while (1) {
	  // controller.driveForward(30);
		// controller.turnClockwise(30);
		// printf("%d,%d\n",encoderGet(leftEnc), encoderGet(rightEnc));

		// volatile int x = *((int*)0xFFFFFFFF) = 69;
		// printf("%d", x);

		if (joystickGetDigital(1, 6, JOY_UP))
			target = liftUpTarget;
		else if (joystickGetDigital(1, 6, JOY_DOWN))
			target = liftDownTarget;
		else if (joystickGetDigital(1, 5, JOY_UP))
			target = lift34;
		else if (joystickGetDigital(1, 8, JOY_LEFT)) {
			liftPid.flipDisable();
			while (joystickGetDigital(1, 8, JOY_LEFT));
		}

		liftPid.setTarget(target);
		liftPid.loop(analogRead(liftPot));
		motorSet(liftLeft, liftPid.getOutput());
		motorSet(liftRight, liftPid.getOutput());

		controller.arcade(joystickGetAnalog(1, 3), joystickGetAnalog(1, 4));

		// controller.driveForward(power);
		if (joystickGetDigital(1, 8, JOY_RIGHT)) {
			encoderReset(leftEnc);
			encoderReset(rightEnc);
			VelPid left(0.05, 0), right(0.05, 0);
			left.setTarget(50);
			right.setTarget(25);
			while (true) {
				left.loop(encoderGet(leftEnc));
				right.loop(encoderGet(rightEnc));
				controller.tank(left.getOutput(), right.getOutput());
				// printf("%1.2f,%1.2f\n", left.getVel(), right.getVel());
				if (joystickGetDigital(1, 7, JOY_RIGHT)) break;
				taskDelay(15);
			}
		}

		// vm.loop((encoderGet(leftEnc) + encoderGet(rightEnc)) / 2.0);
		// printf("%d,%1.2f\n", power, vm.getOutput());
		// power++;
		// if (power > 127) break;
		// if (joystickGetDigital(1, 8, JOY_LEFT)) break;
		// taskDelay(50);

		// while (lcdReadButtons(uart1) != LCD_BTN_CENTER)
		// 	taskDelay(15);
		//
		// const float rpm1 = measureRPM(motor1, leftEnc, rightEnc);
		// const float rpm2 = measureRPM(motor2, leftEnc, rightEnc);
		//
		// char line1[16], line2[16];
		// line1[15] = '\0';
		// line2[15] = '\0';
		// sprintf(line1, "1:%d,2:%d", (int)round(rpm1), (int)round(rpm2));
		// sprintf(line2, "Voltage: %1.2f", powerLevelMain() / 1000.0);
		// lcdSetText(uart1, 1, line1);
		// lcdSetText(uart1, 2, line2);

		// taskDelay(15);
	}
}
