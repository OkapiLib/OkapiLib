/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/util/abstractRate.hpp"
#include "okapi/api/util/logging.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include <atomic>
#include <memory>

namespace okapi {
class ChassisControllerPID : public virtual ChassisController {
  public:
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
  ChassisControllerPID(const TimeUtil &itimeUtil,
                       const std::shared_ptr<ChassisModel> &imodel,
                       std::unique_ptr<IterativePosPIDController> idistanceController,
                       std::unique_ptr<IterativePosPIDController> iangleController,
                       std::unique_ptr<IterativePosPIDController> iturnController,
                       AbstractMotor::GearsetRatioPair igearset = AbstractMotor::gearset::red,
                       const ChassisScales &iscales = ChassisScales({1, 1}));

  ChassisControllerPID(ChassisControllerPID &&other) noexcept;

  ~ChassisControllerPID() override;

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

  /**
   * Stop the robot (set all the motors to 0).
   */
  void stop() override;

  /**
   * Starts the internal thread. This should not be called by normal users. This method is called
   * by the ChassisControllerFactory when making a new instance of this class.
   */
  void startThread();

  /**
   * Get the ChassisScales.
   */
  ChassisScales getChassisScales() const override;

  /**
   * Get the GearsetRatioPair.
   */
  AbstractMotor::GearsetRatioPair getGearsetRatioPair() const override;

  protected:
  Logger *logger;
  TimeUtil timeUtil;
  std::unique_ptr<IterativePosPIDController> distancePid;
  std::unique_ptr<IterativePosPIDController> anglePid;
  std::unique_ptr<IterativePosPIDController> turnPid;
  ChassisScales scales;
  AbstractMotor::GearsetRatioPair gearsetRatioPair;
  std::atomic_bool doneLooping{true};
  std::atomic_bool doneLoopingSeen{true};
  std::atomic_bool newMovement{false};
  std::atomic_bool dtorCalled{false};
  QTime threadSleepTime{10_ms};

  static void trampoline(void *context);
  void loop();

  bool waitForDistanceSettled();
  bool waitForAngleSettled();
  void stopAfterSettled();

  typedef enum { distance, angle, none } modeType;
  modeType mode{none};

  CrossplatformThread *task{nullptr};
};
} // namespace okapi
