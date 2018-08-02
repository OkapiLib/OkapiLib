/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisControllerPid.hpp"
#include <cmath>

namespace okapi {
ChassisControllerPID::ChassisControllerPID(const TimeUtil &itimeUtil,
                                           std::unique_ptr<ChassisModel> imodel,
                                           const IterativePosPIDControllerArgs &idistanceArgs,
                                           const IterativePosPIDControllerArgs &iangleArgs,
                                           const AbstractMotor::GearsetRatioPair igearset,
                                           const ChassisScales &iscales)
  : ChassisControllerPID(itimeUtil, std::move(imodel),
                         std::make_unique<IterativePosPIDController>(idistanceArgs, itimeUtil),
                         std::make_unique<IterativePosPIDController>(iangleArgs, itimeUtil),
                         igearset, iscales) {
}

ChassisControllerPID::ChassisControllerPID(
  const TimeUtil &itimeUtil, std::unique_ptr<ChassisModel> imodel,
  std::unique_ptr<IterativePosPIDController> idistanceController,
  std::unique_ptr<IterativePosPIDController> iangleController,
  const AbstractMotor::GearsetRatioPair igearset, const ChassisScales &iscales)
  : ChassisController(std::move(imodel)),
    rate(std::move(itimeUtil.getRate())),
    distancePid(std::move(idistanceController)),
    anglePid(std::move(iangleController)),
    gearRatio(igearset.ratio),
    straightScale(iscales.straight),
    turnScale(iscales.turn),
    task(trampoline, this) {
  if (igearset.ratio == 0) {
    throw std::invalid_argument("ChassisControllerPID: The gear ratio cannot be zero! Check if you "
                                "are using integer division.");
  }

  setGearing(igearset.internalGearset);
  setEncoderUnits(AbstractMotor::encoderUnits::degrees);
}

ChassisControllerPID::~ChassisControllerPID() {
  dtorCalled = true;
}

void ChassisControllerPID::loop() {
  auto encStartVals = model->getSensorVals();
  std::valarray<std::int32_t> encVals;
  double distanceElapsed = 0, angleChange = 0;
  modeType pastMode = distance;

  while (!dtorCalled) {
    /**
     * doneLooping is set to false by moveDistanceAsync and turnAngleAsync and then set to true by
     * waitUntilSettled
     */
    if (!doneLooping) {
      if (mode != pastMode) {
        encStartVals = model->getSensorVals();

        distancePid->reset();
        anglePid->reset();
        model->stop();
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
        angleChange = static_cast<double>(encVals[0] - encVals[1]);
        model->rotate(anglePid->step(angleChange));

        break;
      default:
        break;
      }

      pastMode = mode;
    }

    rate->delayUntil(10_ms);
  }
}

void ChassisControllerPID::trampoline(void *context) {
  static_cast<ChassisControllerPID *>(context)->loop();
}

void ChassisControllerPID::moveDistanceAsync(const QLength itarget) {
  distancePid->reset();
  anglePid->reset();
  distancePid->flipDisable(false);
  anglePid->flipDisable(false);
  mode = distance;

  const double newTarget = itarget.convert(meter) * straightScale * gearRatio;
  distancePid->setTarget(newTarget);
  anglePid->setTarget(0);

  doneLooping = false;
}

void ChassisControllerPID::moveDistanceAsync(const double itarget) {
  // Divide by straightScale so the final result turns back into motor degrees
  moveDistanceAsync((itarget / straightScale) * meter);
}

void ChassisControllerPID::moveDistance(const QLength itarget) {
  moveDistanceAsync(itarget);
  waitUntilSettled();
}

void ChassisControllerPID::moveDistance(const double itarget) {
  // Divide by straightScale so the final result turns back into motor degrees
  moveDistance((itarget / straightScale) * meter);
}

void ChassisControllerPID::turnAngleAsync(const QAngle idegTarget) {
  anglePid->reset();
  anglePid->flipDisable(false);
  distancePid->flipDisable(true);
  mode = angle;

  const double newTarget = idegTarget.convert(degree) * turnScale * gearRatio;
  anglePid->setTarget(newTarget);

  doneLooping = false;
}

void ChassisControllerPID::turnAngleAsync(const double idegTarget) {
  // Divide by turnScale so the final result turns back into motor degrees
  turnAngleAsync((idegTarget / turnScale) * degree);
}

void ChassisControllerPID::turnAngle(const QAngle idegTarget) {
  turnAngleAsync(idegTarget);
  waitUntilSettled();
}

void ChassisControllerPID::turnAngle(const double idegTarget) {
  // Divide by turnScale so the final result turns back into motor degrees
  turnAngle((idegTarget / turnScale) * degree);
}

void ChassisControllerPID::waitUntilSettled() {
  bool completelySettled = false;

  while (!completelySettled) {
    switch (mode) {
    case distance:
      completelySettled = waitForDistanceSettled();

      // Only disable the controllers and stop if we are totally settled and won't try again
      if (completelySettled) {
        stopAfterSettled();
      }

      break;

    case angle:
      completelySettled = waitForAngleSettled();

      // Only disable the controllers and stop if we are totally settled and won't try again
      if (completelySettled) {
        stopAfterSettled();
      }

      break;

    default:
      break;
    }
  }
}

/**
 * Wait for the distance setup (distancePid and anglePid) to settle.
 *
 * @return true if done settling; false if settling should be tried again
 */
bool ChassisControllerPID::waitForDistanceSettled() {
  while (!(distancePid->isSettled() && anglePid->isSettled())) {
    if (mode == angle) {
      // False will cause the loop to re-enter the switch
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
  while (!anglePid->isSettled()) {
    if (mode == distance) {
      // False will cause the loop to re-enter the switch
      return false;
    }

    rate->delayUntil(10_ms);
  }

  // True will cause the loop to exit
  return true;
}

void ChassisControllerPID::stopAfterSettled() {
  switch (mode) {
  case distance:
    doneLooping = true;
    distancePid->flipDisable(true);
    anglePid->flipDisable(true);
    model->stop();
    break;

  case angle:
    doneLooping = true;
    anglePid->flipDisable(true);
    model->stop();
    break;

  default:
    break;
  }
}

void ChassisControllerPID::stop() {
  stopAfterSettled();
  ChassisController::stop();
}
} // namespace okapi
