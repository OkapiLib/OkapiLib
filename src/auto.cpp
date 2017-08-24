/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
 *
 * Any copyright is dedicated to the Public Domain.
 * http://creativecommons.org/publicdomain/zero/1.0/
 *
 * PROS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"
#include "chassis/basicChassisController.h"
#include "filter/avgFilter.h"
#include "chassis/chassisModel.h"
#include "filter/demaFilter.h"
#include "filter/emaFilter.h"
#include "filter/filter.h"
#include "util/mathUtil.h"
#include "motionProfile/motionProfile.h"
#include "control/mpConsumer.h"
#include "motionProfile/mpGenerator.h"
#include "chassis/odomChassisController.h"
#include "odometry/odometry.h"
#include "odometry/odomMath.h"
#include "control/pid.h"
#include "util/timer.h"
#include "control/velPid.h"
#include "control/mpController.h"
#include <API.h>
#include <functional>

using namespace okapi;

void autonomous() {
  // OdomChassisControllerMP foo(
  //   OdomParams(
  //     SkidSteerModelParams<2>({1,3,2,4}, //The four motor ports
  //                             encoderInit(1,2,false), //Left encoder
  //                             encoderInit(3,4,true)), //Right encoder
  //     1.345,     //Distance scale (encoder ticks to mm)
  //     12.88361), //Turn scale (encoder ticks to deg)
  //   MPControllerParams(
  //     MPGenParams(1, 15, 1000),       //MPGenerator params; max & min accel: 1, max vel: 15, target pos: 1000
  //     MPConsumerParams(6, 1.2, 0.5))); //MPConsumer params; kV: 6, kA: 1.2, kP: 0.5
  // // EmaFilter bar(0,0);
  // // DemaFilter baz(0,0);
  // // AvgFilter<5> avg;
  // Timer t;

  ChassisControllerPid controller(SkidSteerModelParams<3>({4,5,6, 7,8,9}, //Left motors, right motors
                                                          encoderInit(3, 4, false), //Left encoder
                                                          encoderInit(1, 2, true)), //Right encoder
                                  PidParams(0.35, 0.22, 0.18), //Distance PID
                                  PidParams(0.5, 0.5, 0));     //Angle PID

  //Test slow drive forward
  controller.driveForward(30);
}
