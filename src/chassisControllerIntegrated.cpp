/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#include "okapi/chassis/controller/chassisControllerIntegrated.hpp"

namespace okapi {
ChassisControllerIntegrated::ChassisControllerIntegrated(
  const ChassisModelParams &imodelParams,
  const PosIntegratedControllerParams &ileftControllerParams,
  const PosIntegratedControllerParams &irightControllerParams)
  : ChassisController(imodelParams),
    leftController(ileftControllerParams),
    rightController(irightControllerParams),
    lastTarget(0) {
}

ChassisControllerIntegrated::ChassisControllerIntegrated(
  std::shared_ptr<const ChassisModel> imodel,
  const PosIntegratedControllerParams &ileftControllerParams,
  const PosIntegratedControllerParams &irightControllerParams)
  : ChassisController(imodel),
    leftController(ileftControllerParams),
    rightController(irightControllerParams),
    lastTarget(0) {
}

/**
 * ChassisController using the V5 motor's integrated control. This constructor assumes a skid
 * steer layout.
 *
 * @param ileftSideMotor left side motor
 * @param irightSideMotor right side motor
 */
ChassisControllerIntegrated::ChassisControllerIntegrated(const AbstractMotor &ileftSideMotor,
                                                         const AbstractMotor &irightSideMotor)
  : ChassisController(SkidSteerModelParams(ileftSideMotor, irightSideMotor)),
    leftController(ileftSideMotor),
    rightController(irightSideMotor),
    lastTarget(0) {
}

/**
 * ChassisController using V5 motor's integrated control. This constructor assumes an x-drive
 * layout.
 *
 * @param itopLeftMotor top left motor
 * @param itopRightMotor top right motor
 * @param ibottomRightMotor bottom right motor
 * @param ibottomLeftMotor bottom left motor
 */
ChassisControllerIntegrated::ChassisControllerIntegrated(const AbstractMotor &itopLeftMotor,
                                                         const AbstractMotor &itopRightMotor,
                                                         const AbstractMotor &ibottomRightMotor,
                                                         const AbstractMotor &ibottomLeftMotor)
  : ChassisController(
      XDriveModelParams(itopLeftMotor, itopRightMotor, ibottomRightMotor, ibottomLeftMotor)),
    leftController(itopLeftMotor),
    rightController(itopRightMotor),
    lastTarget(0) {
}

ChassisControllerIntegrated::~ChassisControllerIntegrated() = default;

/**
 * Drives the robot straight.
 *
 * @param itarget Distance to travel
 */
void ChassisControllerIntegrated::driveStraight(const int itarget) {
  const int newTarget = itarget + lastTarget;
  leftController.setTarget(newTarget);
  rightController.setTarget(newTarget);
}

/**
 * Turns the robot clockwise in place.
 *
 * @param idegTarget Degrees to turn for
 */
void ChassisControllerIntegrated::pointTurn(float idegTarget) {
  lastTarget = 0;
  leftController.reset();
  rightController.reset();

  leftController.setTarget(idegTarget);
  rightController.setTarget(-1 * idegTarget);
}
} // namespace okapi
