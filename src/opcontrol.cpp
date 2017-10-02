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
#include "control/genericController.h"

using namespace okapi;

void operatorControl() {
	using namespace std; //Needed to get round to compile

	Encoder leftEnc = encoderInit(1, 2, false);
	Encoder rightEnc = encoderInit(3, 4, false);
	ChassisControllerPid controller(SkidSteerModelParams<3>({2_m,3_m,4_m, 5_m,6_m,7_m}, leftEnc, rightEnc), PidParams(0.15, 0.05, 0.07), PidParams(0.02, 0.01, 0));

	const unsigned char liftPot = 1;

  GenericController<2> liftController({8_m, 9_m}, std::make_shared<NsPid>(NsPid(PidParams(0.2, 0.1, 0.1), VelMathParams(360), 0.5)));

	constexpr int liftUpTarget = 2570, lift34 = 300, liftDownTarget = 10;
	int target = liftUpTarget;

	while (1) {
		if (joystickGetDigital(1, 6, JOY_UP))
			target = liftUpTarget;
		else if (joystickGetDigital(1, 6, JOY_DOWN))
			target = liftDownTarget;
		else if (joystickGetDigital(1, 5, JOY_UP))
			target = lift34;
		else if (joystickGetDigital(1, 8, JOY_LEFT)) {
			liftController.flipDisable();
			while (joystickGetDigital(1, 8, JOY_LEFT));
		}

    liftController.setTarget(target);
    liftController.loop(analogRead(liftPot));

    controller.arcade(joystickGetAnalog(1, 3), joystickGetAnalog(1, 4));
    
    if (joystickGetDigital(1, 8, JOY_RIGHT)) {
      while (true) {
        VelMath vm1(360), vm2(360, 1, 0);
        float avg = (encoderGet(leftEnc) + encoderGet(rightEnc)) / 2.0;
        vm1.loop(avg);
        vm2.loop(avg);
        printf("%1.2f,%1.2f\n", vm1.getOutput(), vm2.getOutput());

        if (joystickGetDigital(1, 8, JOY_LEFT)) {
          while (joystickGetDigital(1, 8, JOY_LEFT)) { taskDelay(15); }
          break;
        }

        taskDelay(15);
      }
    }
	}
}
