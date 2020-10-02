/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/odomChassisController.hpp"

namespace okapi {
OdomChassisController::OdomChassisController(TimeUtil itimeUtil,
                                             std::shared_ptr<Odometry> iodometry,
                                             const StateMode &imode,
                                             const QLength &imoveThreshold,
                                             const QAngle &iturnThreshold,
                                             std::shared_ptr<Logger> ilogger)
  : logger(std::move(ilogger)),
    timeUtil(std::move(itimeUtil)),
    moveThreshold(imoveThreshold),
    turnThreshold(iturnThreshold),
    odom(std::move(iodometry)),
    defaultStateMode(imode) {
}

OdomChassisController::~OdomChassisController() {
  dtorCalled.store(true, std::memory_order_release);
  delete odomTask;
}

OdomState OdomChassisController::getState() const {
  return odom->getState(defaultStateMode);
}

void OdomChassisController::setState(const OdomState &istate) {
  odom->setState(istate, defaultStateMode);
}

void OdomChassisController::setDefaultStateMode(const StateMode &imode) {
  defaultStateMode = imode;
}

void OdomChassisController::setMoveThreshold(const QLength &imoveThreshold) {
  moveThreshold = imoveThreshold;
}

void OdomChassisController::setTurnThreshold(const QAngle &iturnTreshold) {
  turnThreshold = iturnTreshold;
}

QLength OdomChassisController::getMoveThreshold() const {
  return moveThreshold;
}

QAngle OdomChassisController::getTurnThreshold() const {
  return turnThreshold;
}

void OdomChassisController::startOdomThread() {
  if (!odomTask) {
    odomTask = new CrossplatformThread(trampoline, this, "OdomChassisController");
  }
}

void OdomChassisController::trampoline(void *context) {
  if (context) {
    static_cast<OdomChassisController *>(context)->loop();
  }
}

void OdomChassisController::loop() {
  odomTaskRunning = true;
  LOG_INFO_S("Started OdomChassisController task.");

  auto rate = timeUtil.getRate();
  while (!dtorCalled.load(std::memory_order_acquire) && !odomTask->notifyTake(0)) {
    odom->step();
    rate->delayUntil(10_ms);
  }

  odomTaskRunning = false;
  LOG_INFO_S("Stopped OdomChassisController task.");
}

CrossplatformThread *OdomChassisController::getOdomThread() const {
  return odomTask;
}

std::shared_ptr<Odometry> OdomChassisController::getOdometry() {
  return odom;
}
} // namespace okapi
