/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ASYNCCONTROLLERFACTORY_HPP_
#define _OKAPI_ASYNCCONTROLLERFACTORY_HPP_

#include "okapi/api/chassis/controller/chassisController.hpp"
#include "okapi/api/control/async/asyncLinearMotionProfileController.hpp"
#include "okapi/api/control/async/asyncMotionProfileController.hpp"
#include "okapi/api/control/async/asyncPosIntegratedController.hpp"
#include "okapi/api/control/async/asyncPosPidController.hpp"
#include "okapi/api/control/async/asyncVelIntegratedController.hpp"
#include "okapi/api/control/async/asyncVelPidController.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/impl/device/rotarysensor/adiEncoder.hpp"
#include "okapi/impl/device/rotarysensor/potentiometer.hpp"

namespace okapi {
class AsyncControllerFactory {
  public:
  /**
   * A position controller that uses the V5 motor's onboard control.
   *
   * @param imotor controller input (from the integrated encoder) and output
   * @param imaxVelocity the maximum velocity during a profiled movement
   */
  static AsyncPosIntegratedController posIntegrated(Motor imotor, std::int32_t imaxVelocity = 600);

  /**
   * A position controller that uses the V5 motor's onboard control.
   *
   * @param imotor controller input (from the integrated encoder) and output
   * @param imaxVelocity the maximum velocity during a profiled movement
   */
  static AsyncPosIntegratedController posIntegrated(MotorGroup imotor,
                                                    std::int32_t imaxVelocity = 600);

  /**
   * A velocity controller that uses the V5 motor's onboard control.
   *
   * @param imotor controller input (from the integrated encoder) and output
   */
  static AsyncVelIntegratedController velIntegrated(Motor imotor);

  /**
   * A velocity controller that uses the V5 motor's onboard control.
   *
   * @param imotor controller input (from the integrated encoder) and output
   */
  static AsyncVelIntegratedController velIntegrated(MotorGroup imotor);

  /**
   * A position controller that uses the PID algorithm.
   *
   * @param imotor controller input (from the integrated encoder) and output
   * @param ikP proportional gain
   * @param ikI integration gain
   * @param ikD derivative gain
   * @param ikBias output bias (a constant added to the output)
   */
  static AsyncPosPIDController
  posPID(Motor imotor,
         double ikP,
         double ikI,
         double ikD,
         double ikBias = 0,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());

  /**
   * A position controller that uses the PID algorithm.
   *
   * @param imotor controller output
   * @param ienc controller input
   * @param ikP proportional gain
   * @param ikI integration gain
   * @param ikD derivative gain
   * @param ikBias output bias (a constant added to the output)
   */
  static AsyncPosPIDController
  posPID(Motor imotor,
         ADIEncoder ienc,
         double ikP,
         double ikI,
         double ikD,
         double ikBias = 0,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());

  /**
   * A position controller that uses the PID algorithm.
   *
   * @param imotor controller output
   * @param ipot controller input
   * @param ikP proportional gain
   * @param ikI integration gain
   * @param ikD derivative gain
   * @param ikBias output bias (a constant added to the output)
   */
  static AsyncPosPIDController
  posPID(Motor imotor,
         Potentiometer ipot,
         double ikP,
         double ikI,
         double ikD,
         double ikBias = 0,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());

  /**
   * A position controller that uses the PID algorithm.
   *
   * @param imotor controller input (from the integrated encoder) and output
   * @param ikP proportional gain
   * @param ikI integration gain
   * @param ikD derivative gain
   * @param ikBias output bias (a constant added to the output)
   */
  static AsyncPosPIDController
  posPID(MotorGroup imotor,
         double ikP,
         double ikI,
         double ikD,
         double ikBias = 0,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());

  /**
   * A position controller that uses the PID algorithm.
   *
   * @param imotor controller output
   * @param ienc controller input
   * @param ikP proportional gain
   * @param ikI integration gain
   * @param ikD derivative gain
   * @param ikBias output bias (a constant added to the output)
   */
  static AsyncPosPIDController
  posPID(MotorGroup imotor,
         ADIEncoder ienc,
         double ikP,
         double ikI,
         double ikD,
         double ikBias = 0,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());

  /**
   * A position controller that uses the PID algorithm.
   *
   * @param imotor controller output
   * @param ipot controller input
   * @param ikP proportional gain
   * @param ikI integration gain
   * @param ikD derivative gain
   * @param ikBias output bias (a constant added to the output)
   */
  static AsyncPosPIDController
  posPID(MotorGroup imotor,
         Potentiometer ipot,
         double ikP,
         double ikI,
         double ikD,
         double ikBias = 0,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());

  /**
   * A position controller that uses the PID algorithm.
   *
   * @param iinput controller input
   * @param ioutput controller output
   * @param ikP proportional gain
   * @param ikI integration gain
   * @param ikD derivative gain
   * @param ikBias output bias (a constant added to the output)
   */
  static AsyncPosPIDController
  posPID(std::shared_ptr<ControllerInput<double>> iinput,
         std::shared_ptr<ControllerOutput<double>> ioutput,
         double ikP,
         double ikI,
         double ikD,
         double ikBias = 0,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());

  /**
   * A velocity controller that uses the PD algorithm.
   *
   * @param imotor controller input (from the integrated encoder) and output
   * @param ikP proportional gain
   * @param ikD derivative gain
   * @param ikF feed-forward gain
   * @param ikSF a feed-forward gain to counteract static friction
   */
  static AsyncVelPIDController
  velPID(Motor imotor,
         double ikP,
         double ikD,
         double ikF = 0,
         double ikSF = 0,
         double iTPR = imev5TPR,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());

  /**
   * A velocity controller that uses the PD algorithm.
   *
   * @param imotor controller output
   * @param ienc controller input
   * @param ikP proportional gain
   * @param ikD derivative gain
   * @param ikF feed-forward gain
   * @param ikSF a feed-forward gain to counteract static friction
   */
  static AsyncVelPIDController
  velPID(Motor imotor,
         ADIEncoder ienc,
         double ikP,
         double ikD,
         double ikF = 0,
         double ikSF = 0,
         double iTPR = imev5TPR,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());

  /**
   * A velocity controller that uses the PD algorithm.
   *
   * @param imotor controller output
   * @param ipot controller input
   * @param ikP proportional gain
   * @param ikD derivative gain
   * @param ikF feed-forward gain
   * @param ikSF a feed-forward gain to counteract static friction
   */
  static AsyncVelPIDController
  velPID(Motor imotor,
         Potentiometer ipot,
         double ikP,
         double ikD,
         double ikF = 0,
         double ikSF = 0,
         double iTPR = imev5TPR,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());

  /**
   * A velocity controller that uses the PD algorithm.
   *
   * @param imotor controller input (from the integrated encoder) and output
   * @param ikP proportional gain
   * @param ikD derivative gain
   * @param ikF feed-forward gain
   * @param ikSF a feed-forward gain to counteract static friction
   */
  static AsyncVelPIDController
  velPID(MotorGroup imotor,
         double ikP,
         double ikD,
         double ikF = 0,
         double ikSF = 0,
         double iTPR = imev5TPR,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());

  /**
   * A velocity controller that uses the PD algorithm.
   *
   * @param imotor controller output
   * @param ienc controller input
   * @param ikP proportional gain
   * @param ikD derivative gain
   * @param ikF feed-forward gain
   * @param ikSF a feed-forward gain to counteract static friction
   */
  static AsyncVelPIDController
  velPID(MotorGroup imotor,
         ADIEncoder ienc,
         double ikP,
         double ikD,
         double ikF = 0,
         double ikSF = 0,
         double iTPR = imev5TPR,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());

  /**
   * A velocity controller that uses the PD algorithm.
   *
   * @param imotor controller output
   * @param ipot controller input
   * @param ikP proportional gain
   * @param ikD derivative gain
   * @param ikF feed-forward gain
   * @param ikSF a feed-forward gain to counteract static friction
   */
  static AsyncVelPIDController
  velPID(MotorGroup imotor,
         Potentiometer ipot,
         double ikP,
         double ikD,
         double ikF = 0,
         double ikSF = 0,
         double iTPR = imev5TPR,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());

  /**
   * A velocity controller that uses the PD algorithm.
   *
   * @param iinput controller input
   * @param ioutput controller output
   * @param ikP proportional gain
   * @param ikD derivative gain
   * @param ikF feed-forward gain
   * @param ikSF a feed-forward gain to counteract static friction
   */
  static AsyncVelPIDController
  velPID(std::shared_ptr<ControllerInput<double>> iinput,
         std::shared_ptr<ControllerOutput<double>> ioutput,
         double ikP,
         double ikD,
         double ikF = 0,
         double ikSF = 0,
         double iTPR = imev5TPR,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>());

  /**
   * A controller which generates and follows 2D motion profiles.
   *
   * @param imaxVel The maximum possible velocity in m/s.
   * @param imaxAccel The maximum possible acceleration in m/s/s.
   * @param imaxJerk The maximum possible jerk in m/s/s/s.
   * @param ichassis The chassis to control.
   */
  static AsyncMotionProfileController motionProfile(double imaxVel,
                                                    double imaxAccel,
                                                    double imaxJerk,
                                                    const ChassisController &ichassis);

  /**
   * A controller which generates and follows 2D motion profiles.
   *
   * @param imaxVel The maximum possible velocity in m/s.
   * @param imaxAccel The maximum possible acceleration in m/s/s.
   * @param imaxJerk The maximum possible jerk in m/s/s/s.
   * @param imodel The chassis model to control.
   * @param iwidth The chassis wheelbase width.
   */
  static AsyncMotionProfileController motionProfile(double imaxVel,
                                                    double imaxAccel,
                                                    double imaxJerk,
                                                    std::shared_ptr<ChassisModel> imodel,
                                                    const ChassisScales &iscales,
                                                    AbstractMotor::GearsetRatioPair ipair);

  /**
   * A controller which generates and follows 1D motion profiles.
   *
   * @param imaxVel The maximum possible velocity in m/s.
   * @param imaxAccel The maximum possible acceleration in m/s/s.
   * @param imaxJerk The maximum possible jerk in m/s/s/s.
   * @param ioutput The output to write velocity targets to.
   */
  static AsyncLinearMotionProfileController
  linearMotionProfile(double imaxVel,
                      double imaxAccel,
                      double imaxJerk,
                      std::shared_ptr<ControllerOutput<double>> ioutput);
};
} // namespace okapi

#endif
