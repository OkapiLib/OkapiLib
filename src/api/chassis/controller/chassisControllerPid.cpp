/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisControllerPid.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <cmath>

namespace okapi {
ChassisControllerPID::ChassisControllerPID(
  const TimeUtil &itimeUtil,
  const std::shared_ptr<ChassisModel> &imodel,
  std::unique_ptr<IterativePosPIDController> idistanceController,
  std::unique_ptr<IterativePosPIDController> iangleController,
  std::unique_ptr<IterativePosPIDController> iturnController,
  const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales)
  : ChassisController(imodel, toUnderlyingType(igearset.internalGearset)),
    timeUtil(itimeUtil),
    distancePid(std::move(idistanceController)),
    anglePid(std::move(iangleController)),
    turnPid(std::move(iturnController)),
    scales(iscales),
    gearsetRatioPair(igearset) {
  if (igearset.ratio == 0) {
    logger->error("ChassisControllerPID: The gear ratio cannot be zero! Check if you are using "
                  "integer division.");
    throw std::invalid_argument("ChassisControllerPID: The gear ratio cannot be zero! Check if you "
                                "are using integer division.");
  }

  setGearing(igearset.internalGearset);
  setEncoderUnits(AbstractMotor::encoderUnits::degrees);
}

ChassisControllerPID::ChassisControllerPID(ChassisControllerPID &&other) noexcept
  : ChassisController(other.model, other.maxVelocity, other.maxVoltage),
    logger(other.logger),
    timeUtil(other.timeUtil),
    distancePid(std::move(other.distancePid)),
    anglePid(std::move(other.anglePid)),
    turnPid(std::move(other.turnPid)),
    scales(other.scales),
    gearsetRatioPair(other.gearsetRatioPair),
    doneLooping(other.doneLooping.load(std::memory_order_acquire)),
    newMovement(other.newMovement.load(std::memory_order_acquire)),
    dtorCalled(other.dtorCalled.load(std::memory_order_acquire)),
    mode(other.mode),
    task(other.task) {
  other.task = nullptr;
}

ChassisControllerPID::~ChassisControllerPID() {
  dtorCalled.store(true, std::memory_order_release);
  delete task;
}

void ChassisControllerPID::loop() {
  auto encStartVals = model->getSensorVals();
  std::valarray<std::int32_t> encVals;
  double distanceElapsed = 0, angleChange = 0;
  modeType pastMode = none;
  auto rate = timeUtil.getRate();

  while (!dtorCalled.load(std::memory_order_acquire)) {
    /**
     * doneLooping is set to false by moveDistanceAsync and turnAngleAsync and then set to true by
     * waitUntilSettled
     */
    if (doneLooping.load(std::memory_order_acquire)) {
      doneLoopingSeen.store(true, std::memory_order_release);
    } else {
      if (mode != pastMode || newMovement.load(std::memory_order_acquire)) {
        encStartVals = model->getSensorVals();
        newMovement.store(false, std::memory_order_release);
      }

      switch (mode) {
      case distance:
        encVals = model->getSensorVals() - encStartVals;
        distanceElapsed = static_cast<double>((encVals[0] + encVals[1])) / 2.0;
        angleChange = static_cast<double>(encVals[0] - encVals[1]);
        model->driveVector(distancePid->step(distanceElapsed), anglePid->step(angleChange));
        break;

      case angle:
        encVals = model->getSensorVals() - encStartVals;
        angleChange = (encVals[0] - encVals[1]) / 2.0;
        model->rotate(turnPid->step(angleChange));
        break;

      default:
        break;
      }

      pastMode = mode;
    }

    rate->delayUntil(threadSleepTime);
  }
}

void ChassisControllerPID::trampoline(void *context) {
  if (context) {
    static_cast<ChassisControllerPID *>(context)->loop();
  }
}

void ChassisControllerPID::moveDistanceAsync(const QLength itarget) {
  logger->info("ChassisControllerPID: moving " + std::to_string(itarget.convert(meter)) +
               " meters");

  distancePid->reset();
  anglePid->reset();
  distancePid->flipDisable(false);
  anglePid->flipDisable(false);
  turnPid->flipDisable(true);
  mode = distance;

  const double newTarget = itarget.convert(meter) * scales.straight * gearsetRatioPair.ratio;

  logger->info("ChassisControllerPID: moving " + std::to_string(newTarget) + " motor degrees");

  distancePid->setTarget(newTarget);
  anglePid->setTarget(0);

  doneLooping.store(false, std::memory_order_release);
  newMovement.store(true, std::memory_order_release);
}

void ChassisControllerPID::moveDistanceAsync(const double itarget) {
  // Divide by straightScale so the final result turns back into motor degrees
  moveDistanceAsync((itarget / scales.straight) * meter);
}

void ChassisControllerPID::moveDistance(const QLength itarget) {
  moveDistanceAsync(itarget);
  waitUntilSettled();
}

void ChassisControllerPID::moveDistance(const double itarget) {
  // Divide by straightScale so the final result turns back into motor degrees
  moveDistance((itarget / scales.straight) * meter);
}

void ChassisControllerPID::turnAngleAsync(const QAngle idegTarget) {
  logger->info("ChassisControllerPID: turning " + std::to_string(idegTarget.convert(degree)) +
               " degrees");

  turnPid->reset();
  turnPid->flipDisable(false);
  distancePid->flipDisable(true);
  anglePid->flipDisable(true);
  mode = angle;

  const double newTarget =
    idegTarget.convert(degree) * scales.turn * gearsetRatioPair.ratio * boolToSign(normalTurns);

  logger->info("ChassisControllerPID: turning " + std::to_string(newTarget) + " motor degrees");

  turnPid->setTarget(newTarget);

  doneLooping.store(false, std::memory_order_release);
  newMovement.store(true, std::memory_order_release);
}

void ChassisControllerPID::turnAngleAsync(const double idegTarget) {
  // Divide by turnScale so the final result turns back into motor degrees
  turnAngleAsync((idegTarget / scales.turn) * degree);
}

void ChassisControllerPID::turnAngle(const QAngle idegTarget) {
  turnAngleAsync(idegTarget);
  waitUntilSettled();
}

void ChassisControllerPID::turnAngle(const double idegTarget) {
  // Divide by turnScale so the final result turns back into motor degrees
  turnAngle((idegTarget / scales.turn) * degree);
}

void ChassisControllerPID::waitUntilSettled() {
  logger->info("ChassisControllerPID: Waiting to settle");
  bool completelySettled = false;

  while (!completelySettled) {
    switch (mode) {
    case distance:
      completelySettled = waitForDistanceSettled();
      break;

    case angle:
      completelySettled = waitForAngleSettled();
      break;

    default:
      completelySettled = true;
      break;
    }
  }

  // Order here is important
  mode = none;
  doneLooping.store(true, std::memory_order_release);
  doneLoopingSeen.store(false, std::memory_order_release);

  // Wait for the thread to finish if it happens to be writing to motors
  auto rate = timeUtil.getRate();
  while (!doneLoopingSeen.load(std::memory_order_acquire)) {
    rate->delayUntil(threadSleepTime);
  }

  // Stop after the thread has run at least once
  stopAfterSettled();
  logger->info("ChassisControllerPID: Done waiting to settle");
}

/**
 * Wait for the distance setup (distancePid and anglePid) to settle.
 *
 * @return true if done settling; false if settling should be tried again
 */
bool ChassisControllerPID::waitForDistanceSettled() {
  logger->info("ChassisControllerPID: Waiting to settle in distance mode");

  auto rate = timeUtil.getRate();
  while (!(distancePid->isSettled() && anglePid->isSettled())) {
    if (mode == angle) {
      // False will cause the loop to re-enter the switch
      logger->warn("ChassisControllerPID: Mode changed to angle while waiting in distance!");
      return false;
    }

    rate->delayUntil(10_ms);
  }

  // True will cause the loop to exit
  return true;
}

/**
 * Wait for the angle setup (anglePid) to settle.
 *
 * @return true if done settling; false if settling should be tried again
 */
bool ChassisControllerPID::waitForAngleSettled() {
  logger->info("ChassisControllerPID: Waiting to settle in angle mode");

  auto rate = timeUtil.getRate();
  while (!turnPid->isSettled()) {
    if (mode == distance) {
      // False will cause the loop to re-enter the switch
      logger->warn("ChassisControllerPID: Mode changed to distance while waiting in angle!");
      return false;
    }

    rate->delayUntil(10_ms);
  }

  // True will cause the loop to exit
  return true;
}

void ChassisControllerPID::stopAfterSettled() {
  distancePid->flipDisable(true);
  anglePid->flipDisable(true);
  turnPid->flipDisable(true);
  model->stop();
}

void ChassisControllerPID::stop() {
  mode = none;
  doneLooping.store(true, std::memory_order_release);
  stopAfterSettled();
  ChassisController::stop();
}

void ChassisControllerPID::startThread() {
  if (!task) {
    task = new CrossplatformThread(trampoline, this);
  }
}

ChassisScales ChassisControllerPID::getChassisScales() const {
  return scales;
}

AbstractMotor::GearsetRatioPair ChassisControllerPID::getGearsetRatioPair() const {
  return gearsetRatioPair;
}
} // namespace okapi
