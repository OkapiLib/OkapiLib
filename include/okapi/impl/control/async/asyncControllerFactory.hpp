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
#include "okapi/impl/device/rotarysensor/integratedEncoder.hpp"
#include "okapi/impl/device/rotarysensor/potentiometer.hpp"

#define makePosPID(MotorType)                                                                      \
  static AsyncPosPIDController posPID(MotorType imotor,                                            \
                                      double ikP,                                                  \
                                      double ikI,                                                  \
                                      double ikD,                                                  \
                                      double ikBias = 0,                                           \
                                      std::unique_ptr<Filter> iderivativeFilter =                  \
                                        std::make_unique<PassthroughFilter>()) {                   \
    return posPID(imotor.getEncoder(),                                                             \
                  std::make_shared<MotorType>(imotor),                                             \
                  ikP,                                                                             \
                  ikI,                                                                             \
                  ikD,                                                                             \
                  ikBias,                                                                          \
                  std::move(iderivativeFilter));                                                   \
  }

#define makePosPIDWithSensor(MotorType, SensorType)                                                \
  static AsyncPosPIDController posPID(MotorType imotor,                                            \
                                      SensorType isensor,                                          \
                                      double ikP,                                                  \
                                      double ikI,                                                  \
                                      double ikD,                                                  \
                                      double ikBias = 0,                                           \
                                      std::unique_ptr<Filter> iderivativeFilter =                  \
                                        std::make_unique<PassthroughFilter>()) {                   \
    return posPID(std::make_shared<SensorType>(isensor),                                           \
                  std::make_shared<MotorType>(imotor),                                             \
                  ikP,                                                                             \
                  ikI,                                                                             \
                  ikD,                                                                             \
                  ikBias,                                                                          \
                  std::move(iderivativeFilter));                                                   \
  }

#define makeVelPID(MotorType)                                                                      \
  static AsyncVelPIDController velPID(MotorType imotor,                                            \
                                      double ikP,                                                  \
                                      double ikD,                                                  \
                                      double ikF = 0,                                              \
                                      double ikSF = 0,                                             \
                                      double iTPR = imev5TPR,                                      \
                                      std::unique_ptr<Filter> iderivativeFilter =                  \
                                        std::make_unique<PassthroughFilter>()) {                   \
    return velPID(imotor.getEncoder(),                                                             \
                  std::make_shared<MotorType>(imotor),                                             \
                  ikP,                                                                             \
                  ikD,                                                                             \
                  ikF,                                                                             \
                  ikSF,                                                                            \
                  iTPR,                                                                            \
                  std::move(iderivativeFilter));                                                   \
  }

#define makeVelPIDWithSensor(MotorType, SensorType)                                                \
  static AsyncVelPIDController velPID(MotorType imotor,                                            \
                                      SensorType ienc,                                             \
                                      double ikP,                                                  \
                                      double ikD,                                                  \
                                      double ikF = 0,                                              \
                                      double ikSF = 0,                                             \
                                      double iTPR = imev5TPR,                                      \
                                      std::unique_ptr<Filter> iderivativeFilter =                  \
                                        std::make_unique<PassthroughFilter>()) {                   \
    return velPID(std::make_shared<SensorType>(ienc),                                              \
                  std::make_shared<MotorType>(imotor),                                             \
                  ikP,                                                                             \
                  ikD,                                                                             \
                  ikF,                                                                             \
                  ikSF,                                                                            \
                  iTPR,                                                                            \
                  std::move(iderivativeFilter));                                                   \
  }

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

  makePosPID(Motor);
  makePosPID(MotorGroup);
  makePosPIDWithSensor(Motor, ADIEncoder);
  makePosPIDWithSensor(Motor, Potentiometer);
  makePosPIDWithSensor(Motor, IntegratedEncoder);
  makePosPIDWithSensor(MotorGroup, ADIEncoder);
  makePosPIDWithSensor(MotorGroup, Potentiometer);
  makePosPIDWithSensor(MotorGroup, IntegratedEncoder);

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

  makeVelPID(Motor);
  makeVelPID(MotorGroup);
  makeVelPIDWithSensor(Motor, ADIEncoder);
  makeVelPIDWithSensor(Motor, Potentiometer);
  makeVelPIDWithSensor(Motor, IntegratedEncoder);
  makeVelPIDWithSensor(MotorGroup, ADIEncoder);
  makeVelPIDWithSensor(MotorGroup, Potentiometer);
  makeVelPIDWithSensor(MotorGroup, IntegratedEncoder);

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
