/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISCONTROLLERPID_HPP_
#define _OKAPI_CHASSISCONTROLLERPID_HPP_

#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/util/abstractRate.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include <memory>

namespace okapi {
class ChassisControllerPID : public virtual ChassisController {
  public:
  /**
   * ChassisController using PID control. Puts the motors into encoder degree units. Throws a
   * std::invalid_argument exception if the gear ratio is zero.
   *
   * @param imodelArgs ChassisModelArgs
   * @param idistanceArgs distance PID controller params
   * @param iangleArgs angle PID controller params (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  ChassisControllerPID(const TimeUtil &itimeUtil, std::unique_ptr<ChassisModel> imodel,
                       const IterativePosPIDControllerArgs &idistanceArgs,
                       const IterativePosPIDControllerArgs &iangleArgs,
                       AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
                       const ChassisScales &iscales = ChassisScales({1, 1}));

  /**
   * ChassisController using PID control. Puts the motors into encoder degree units. Throws a
   * std::invalid_argument exception if the gear ratio is zero.
   *
   * @param imodelArgs ChassisModelArgs
   * @param idistanceController distance PID controller
   * @param iangleController angle PID controller (keeps the robot straight)
   * @param igearset motor internal gearset and gear ratio
   * @param iscales see ChassisScales docs
   */
  ChassisControllerPID(const TimeUtil &itimeUtil, std::unique_ptr<ChassisModel> imodel,
                       std::unique_ptr<IterativePosPIDController> idistanceController,
                       std::unique_ptr<IterativePosPIDController> iangleController,
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
  void moveDistanceAsync(QLength itarget) override;

  /**
   * Sets the target distance for the robot to drive straight (using closed-loop control).
   *
   * @param itarget distance to travel in motor degrees
   */
  void moveDistanceAsync(double itarget) override;

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
  void turnAngleAsync(QAngle idegTarget) override;

  /**
   * Sets the target angle for the robot to turn clockwise in place (using closed-loop control).
   *
   * @param idegTarget angle to turn for in motor degrees
   */
  void turnAngleAsync(double idegTarget) override;

  /**
   * Delays until the currently executing movement completes.
   */
  void waitUntilSettled() override;

  protected:
  std::unique_ptr<AbstractRate> rate;
  std::unique_ptr<IterativePosPIDController> distancePid;
  std::unique_ptr<IterativePosPIDController> anglePid;
  const double gearRatio;
  const double straightScale;
  const double turnScale;
  CROSSPLATFORM_THREAD task;

  static void trampoline(void *context);
  void loop();

  typedef enum { distance, angle } modeType;
  modeType mode;
};
} // namespace okapi

#endif
