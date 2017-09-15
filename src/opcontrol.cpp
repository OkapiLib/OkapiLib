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
#include "chassis/basicChassisController.h"
#include "util/timer.h"
#include "util/mathUtil.h"
#include "control/pid.h"
#include <cmath>

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

	Encoder leftEnc = encoderInit(12, 11, true);
	Encoder rightEnc = encoderInit(1, 2, false);
	SkidSteerModel<3> model({2,3,4, 5,6,7}, leftEnc, rightEnc);
	ChassisControllerPid controller(std::make_shared<SkidSteerModel<3>>(model), PidParams(0.35, 0.22, 0.18), PidParams(0.5, 0.5, 0));

	const unsigned char liftLeft = 8, liftRight = 9, liftPot = 1;

  Pid liftPid(0.2, 0.1, 0.1);

	constexpr int liftUpTarget = 2570, lift34 = 800, liftDownTarget = 10;
	int target = liftUpTarget;

	// constexpr unsigned char motor1 = 2, motor2 = 3;
	// lcdSetBacklight(uart1, true);

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

		// printf("%d\n", analogRead(liftPot));

		liftPid.setTarget(target);
		liftPid.loop(analogRead(liftPot));
		motorSet(liftLeft, liftPid.getOutput());
		motorSet(liftRight, liftPid.getOutput());

		controller.arcade(joystickGetAnalog(1, 3), joystickGetAnalog(1, 4));

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

		taskDelay(15);
	}
}
