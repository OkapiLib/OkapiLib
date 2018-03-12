/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/chassis/odomChassisController.hpp"
#include "okapi/odometry/odomMath.hpp"
#include <cmath>

namespace okapi {
  void OdomChassisControllerPid::driveToPoint(const float ix, const float iy, const bool ibackwards, const float ioffset) {
    DistanceAndAngle daa = OdomMath::computeDistanceAndAngleToPoint(ix, iy, odom.getState());

    if (ibackwards) {
      daa.theta += 180;
      daa.length *= -1;
    }

    if (std::abs(daa.theta) > 1) {
      ChassisControllerPid::pointTurn(daa.theta);
    }

    if (std::abs(daa.length - ioffset) > moveThreshold) {
      ChassisControllerPid::driveStraight(static_cast<int>(daa.length - ioffset));
    }
  }

  void OdomChassisControllerPid::turnToAngle(const float iangle) {
    ChassisControllerPid::pointTurn(iangle - odom.getState().theta);
  }
}
