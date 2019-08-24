/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/defaultOdomChassisController.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include <cmath>

namespace okapi {
DefaultOdomChassisController::DefaultOdomChassisController(
  const TimeUtil &itimeUtil,
  std::unique_ptr<Odometry> iodometry,
  std::shared_ptr<ChassisController> icontroller,
  const QLength imoveThreshold,
  const QAngle iturnThreshold,
  std::shared_ptr<Logger> ilogger)
  : OdomChassisController(itimeUtil, std::move(iodometry), imoveThreshold, iturnThreshold),
    logger(std::move(ilogger)),
    controller(std::move(icontroller)) {
}

void DefaultOdomChassisController::driveToPoint(const Point &ipoint,
                                                const StateMode &imode,
                                                const bool ibackwards,
                                                const QLength &ioffset) {
  auto [length, angle] = OdomMath::computeDistanceAndAngleToPoint(
    ipoint, odom->getState(StateMode::FRAME_TRANSFORMATION), imode);

  if (ibackwards) {
    length *= -1;
    angle += 180_deg;
  }

  LOG_INFO("DefaultOdomChassisController: Computed length of " +
           std::to_string(length.convert(meter)) + " meters and angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

  if (angle.abs() > turnThreshold) {
    LOG_INFO("DefaultOdomChassisController: Turning " + std::to_string(angle.convert(degree)) +
             " degrees");
    controller->turnAngle(angle);
  }

  if ((length - ioffset).abs() > moveThreshold) {
    LOG_INFO("DefaultOdomChassisController: Driving " +
             std::to_string((length - ioffset).convert(meter)) + " meters");
    controller->moveDistance(length - ioffset);
  }
}

void DefaultOdomChassisController::turnToPoint(const Point &ipoint, const StateMode &imode) {
  const auto angle =
    OdomMath::computeAngleToPoint(ipoint, odom->getState(StateMode::FRAME_TRANSFORMATION), imode);

  LOG_INFO("DefaultOdomChassisController: Computed angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

  if (angle.abs() > turnThreshold) {
    LOG_INFO("DefaultOdomChassisController: Turning " + std::to_string(angle.convert(degree)) +
             " degrees");
    controller->turnAngle(angle);
  }
}

void DefaultOdomChassisController::turnToAngle(const QAngle &iangle) {
  const auto angle = iangle - odom->getState(StateMode::FRAME_TRANSFORMATION).theta;

  LOG_INFO("DefaultOdomChassisController: Computed angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

  if (angle.abs() > turnThreshold) {
    LOG_INFO("DefaultOdomChassisController: Turning " + std::to_string(angle.convert(degree)) +
             " degrees");
    controller->turnAngle(angle);
  }
}
void DefaultOdomChassisController::moveDistance(QLength itarget) {
  controller->moveDistance(itarget);
}
void DefaultOdomChassisController::moveDistance(double itarget) {
  controller->moveDistance(itarget);
}
void DefaultOdomChassisController::moveDistanceAsync(QLength itarget) {
  controller->moveDistanceAsync(itarget);
}
void DefaultOdomChassisController::moveDistanceAsync(double itarget) {
  controller->moveDistanceAsync(itarget);
}
void DefaultOdomChassisController::turnAngle(QAngle idegTarget) {
  controller->turnAngle(idegTarget);
}
void DefaultOdomChassisController::turnAngle(double idegTarget) {
  controller->turnAngle(idegTarget);
}
void DefaultOdomChassisController::turnAngleAsync(QAngle idegTarget) {
  controller->turnAngleAsync(idegTarget);
}
void DefaultOdomChassisController::turnAngleAsync(double idegTarget) {
  controller->turnAngleAsync(idegTarget);
}
void DefaultOdomChassisController::setTurnsMirrored(bool ishouldMirror) {
  controller->setTurnsMirrored(ishouldMirror);
}
void DefaultOdomChassisController::waitUntilSettled() {
  controller->waitUntilSettled();
}
void DefaultOdomChassisController::stop() {
  controller->stop();
}
ChassisScales DefaultOdomChassisController::getChassisScales() const {
  return controller->getChassisScales();
}
AbstractMotor::GearsetRatioPair DefaultOdomChassisController::getGearsetRatioPair() const {
  return controller->getGearsetRatioPair();
}
std::shared_ptr<ChassisModel> DefaultOdomChassisController::getModel() {
  return controller->getModel();
}
ChassisModel &DefaultOdomChassisController::model() {
  return controller->model();
}
std::shared_ptr<ChassisController> DefaultOdomChassisController::getChassisController() {
  return controller;
}
ChassisController &DefaultOdomChassisController::chassisController() {
  return *controller;
}
} // namespace okapi
