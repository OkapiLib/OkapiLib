/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/odomChassisController.hpp"

namespace okapi {
OdomChassisController::OdomChassisController(std::shared_ptr<SkidSteerModel> imodel,
                                             std::unique_ptr<Odometry> iodometry,
                                             const QLength imoveThreshold,
                                             const QAngle iturnThreshold)
  : ChassisController(imodel),
    moveThreshold(imoveThreshold),
    turnThreshold(iturnThreshold),
    odom(std::move(iodometry)),
    task(Odometry::trampoline, odom.get()) {
}

OdomChassisController::~OdomChassisController() {
  odom->stopLooping();
}

OdomState OdomChassisController::getState() const {
  return odom->getState();
}

void OdomChassisController::setState(const OdomState &istate) {
  odom->setState(istate);
}

void OdomChassisController::setMoveThreshold(const QLength imoveThreshold) {
  moveThreshold = imoveThreshold;
}

void OdomChassisController::setTurnThreshold(QAngle iturnTreshold) {
  turnThreshold = iturnTreshold;
}
} // namespace okapi
