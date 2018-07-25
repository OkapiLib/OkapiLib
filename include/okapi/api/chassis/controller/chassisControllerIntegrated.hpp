/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISCONTROLLERINTEGRATED_HPP_
#define _OKAPI_CHASSISCONTROLLERINTEGRATED_HPP_

#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/control/async/asyncPosIntegratedController.hpp"
#include "okapi/api/util/timeUtil.hpp"

namespace okapi {
class ChassisControllerIntegrated : public virtual ChassisController {
  public:
  /**
   * ChassisController using the V5 motor's integrated control. Puts the motors into degree units.
   * Throws a std::invalid_argument exception if the gear ratio is zero.
   *
   * @param imodelArgs ChassisModelArgs
   * @param ileftControllerArgs left side controller params
   * @param irightControllerArgs right side controller params
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  ChassisControllerIntegrated(
    const TimeUtil &itimeUtil, std::unique_ptr<ChassisModel> imodel,
    const AsyncPosIntegratedControllerArgs &ileftControllerArgs,
    const AsyncPosIntegratedControllerArgs &irightControllerArgs,
    AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
    const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using the V5 motor's integrated control. Puts the motors into degree units.
   * Throws a std::invalid_argument exception if the gear ratio is zero.
   *
   * @param imodelArgs ChassisModelArgs
   * @param ileftControllerArgs left side controller params
   * @param irightControllerArgs right side controller params
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  ChassisControllerIntegrated(
    const TimeUtil &itimeUtil, std::unique_ptr<ChassisModel> imodel,
    std::unique_ptr<AsyncPosIntegratedController> ileftController,
    std::unique_ptr<AsyncPosIntegratedController> irightController,
    AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
    const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * Drives the robot straight for a distance (using closed-loop control).
   *
   * @param itarget distance to travel
   */
  void moveDistance(QLength itarget) override;

  /**
   * Drives the robot straight for a distance (using closed-loop control).
   *
   * @param itarget distance to travel in motor degrees
   */
  void moveDistance(double itarget) override;

  /**
   * Sets the target distance for the robot to drive straight (using closed-loop control).
   *
   * @param itarget distance to travel
   */
  void moveDistanceAsync(QLength itarget);

  /**
   * Sets the target distance for the robot to drive straight (using closed-loop control).
   *
   * @param itarget distance to travel in motor degrees
   */
  void moveDistanceAsync(double itarget);

  /**
   * Turns the robot clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for
   */
  void turnAngle(QAngle idegTarget) override;

  /**
   * Turns the robot clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for in motor degrees
   */
  void turnAngle(double idegTarget) override;

  /**
   * Sets the target angle for the robot to turn clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for
   */
  void turnAngleAsync(QAngle idegTarget);

  /**
   * Sets the target angle for the robot to turn clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for in motor degrees
   */
  void turnAngleAsync(double idegTarget);

  /**
   * Delays until the currently executing movement completes.
   */
  void waitUntilSettled();

  protected:
  std::unique_ptr<AbstractRate> rate;
  std::unique_ptr<AsyncPosIntegratedController> leftController;
  std::unique_ptr<AsyncPosIntegratedController> rightController;
  int lastTarget;
  const double gearRatio;
  const double straightScale;
  const double turnScale;
};
} // namespace okapi

#endif
