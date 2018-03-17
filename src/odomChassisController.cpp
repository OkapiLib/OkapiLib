/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/chassis/controller/odomChassisController.hpp"

namespace okapi {
OdomChassisController::OdomChassisController(const OdometryParams &iparams,
                                             const float imoveThreshold)
  : ChassisController(iparams.model),
    moveThreshold(imoveThreshold),
    odom(iparams),
    task((task_fn_t)Odometry::trampoline, &odom, TASK_PRIORITY_DEFAULT + 1) {
}

OdomChassisController::~OdomChassisController() = default;

OdomState OdomChassisController::getState() const {
  return odom.getState();
}

void OdomChassisController::setState(const OdomState &istate) {
  odom.setState(istate);
}

void OdomChassisController::setMoveThreshold(const float imoveThreshold) {
  moveThreshold = imoveThreshold;
}
} // namespace okapi
