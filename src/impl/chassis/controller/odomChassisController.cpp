/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/chassis/controller/odomChassisController.hpp"

namespace okapi {
OdomChassisController::OdomChassisController(std::shared_ptr<SkidSteerModel> imodel,
                                             std::unique_ptr<Odometry> iodometry,
                                             const float imoveThreshold)
  : ChassisController(imodel),
    moveThreshold(imoveThreshold),
    odom(std::move(iodometry)),
    task((task_fn_t)Odometry::trampoline, &odom, TASK_PRIORITY_DEFAULT + 1) {
}

OdomState OdomChassisController::getState() const {
  return odom->getState();
}

void OdomChassisController::setState(const OdomState &istate) {
  odom->setState(istate);
}

void OdomChassisController::setMoveThreshold(const float imoveThreshold) {
  moveThreshold = imoveThreshold;
}
} // namespace okapi
