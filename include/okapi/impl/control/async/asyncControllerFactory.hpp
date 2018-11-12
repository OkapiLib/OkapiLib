/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

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
#include "okapi/impl/device/rotarysensor/adiGyro.hpp"
#include "okapi/impl/device/rotarysensor/integratedEncoder.hpp"
#include "okapi/impl/device/rotarysensor/potentiometer.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

#define okapi_makePosPID(MotorType)                                                                \
  static AsyncPosPIDController posPID(MotorType imotor,                                            \
                                      double ikP,                                                  \
                                      double ikI,                                                  \
                                      double ikD,                                                  \
                                      double ikBias = 0,                                           \
                                      std::unique_ptr<Filter> iderivativeFilter =                  \
                                        std::make_unique<PassthroughFilter>(),                     \
                                      const TimeUtil &itimeUtil = TimeUtilFactory::create()) {     \
    return posPID(imotor.getEncoder(),                                                             \
                  std::make_shared<MotorType>(imotor),                                             \
                  ikP,                                                                             \
                  ikI,                                                                             \
                  ikD,                                                                             \
                  ikBias,                                                                          \
                  std::move(iderivativeFilter),                                                    \
                  itimeUtil);                                                                      \
  }

#define okapi_makePosPIDWithSensor(MotorType, SensorType)                                          \
  static AsyncPosPIDController posPID(MotorType imotor,                                            \
                                      SensorType isensor,                                          \
                                      double ikP,                                                  \
                                      double ikI,                                                  \
                                      double ikD,                                                  \
                                      double ikBias = 0,                                           \
                                      std::unique_ptr<Filter> iderivativeFilter =                  \
                                        std::make_unique<PassthroughFilter>(),                     \
                                      const TimeUtil &itimeUtil = TimeUtilFactory::create()) {     \
    return posPID(std::make_shared<SensorType>(isensor),                                           \
                  std::make_shared<MotorType>(imotor),                                             \
                  ikP,                                                                             \
                  ikI,                                                                             \
                  ikD,                                                                             \
                  ikBias,                                                                          \
                  std::move(iderivativeFilter),                                                    \
                  itimeUtil);                                                                      \
  }

#define okapi_makeVelPID(MotorType)                                                                \
  static AsyncVelPIDController velPID(MotorType imotor,                                            \
                                      double ikP,                                                  \
                                      double ikD,                                                  \
                                      double ikF = 0,                                              \
                                      double ikSF = 0,                                             \
                                      double iTPR = imev5TPR,                                      \
                                      std::unique_ptr<Filter> iderivativeFilter =                  \
                                        std::make_unique<PassthroughFilter>(),                     \
                                      const TimeUtil &itimeUtil = TimeUtilFactory::create()) {     \
    return velPID(imotor.getEncoder(),                                                             \
                  std::make_shared<MotorType>(imotor),                                             \
                  ikP,                                                                             \
                  ikD,                                                                             \
                  ikF,                                                                             \
                  ikSF,                                                                            \
                  iTPR,                                                                            \
                  std::move(iderivativeFilter),                                                    \
                  itimeUtil);                                                                      \
  }

#define okapi_makeVelPIDWithSensor(MotorType, SensorType)                                          \
  static AsyncVelPIDController velPID(MotorType imotor,                                            \
                                      SensorType ienc,                                             \
                                      double ikP,                                                  \
                                      double ikD,                                                  \
                                      double ikF = 0,                                              \
                                      double ikSF = 0,                                             \
                                      double iTPR = imev5TPR,                                      \
                                      std::unique_ptr<Filter> iderivativeFilter =                  \
                                        std::make_unique<PassthroughFilter>(),                     \
                                      const TimeUtil &itimeUtil = TimeUtilFactory::create()) {     \
    return velPID(std::make_shared<SensorType>(ienc),                                              \
                  std::make_shared<MotorType>(imotor),                                             \
                  ikP,                                                                             \
                  ikD,                                                                             \
                  ikF,                                                                             \
                  ikSF,                                                                            \
                  iTPR,                                                                            \
                  std::move(iderivativeFilter),                                                    \
                  itimeUtil);                                                                      \
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
  static AsyncPosIntegratedController
  posIntegrated(Motor imotor,
                std::int32_t imaxVelocity = 600,
                const TimeUtil &itimeUtil = TimeUtilFactory::create());

  /**
   * A position controller that uses the V5 motor's onboard control.
   *
   * @param imotor controller input (from the integrated encoder) and output
   * @param imaxVelocity the maximum velocity during a profiled movement
   */
  static AsyncPosIntegratedController
  posIntegrated(MotorGroup imotor,
                std::int32_t imaxVelocity = 600,
                const TimeUtil &itimeUtil = TimeUtilFactory::create());

  /**
   * A velocity controller that uses the V5 motor's onboard control.
   *
   * @param imotor controller input (from the integrated encoder) and output
   */
  static AsyncVelIntegratedController
  velIntegrated(Motor imotor, const TimeUtil &itimeUtil = TimeUtilFactory::create());

  /**
   * A velocity controller that uses the V5 motor's onboard control.
   *
   * @param imotor controller input (from the integrated encoder) and output
   */
  static AsyncVelIntegratedController
  velIntegrated(MotorGroup imotor, const TimeUtil &itimeUtil = TimeUtilFactory::create());

  okapi_makePosPID(Motor);
  okapi_makePosPID(MotorGroup);
  okapi_makePosPIDWithSensor(Motor, ADIEncoder);
  okapi_makePosPIDWithSensor(Motor, ADIGyro);
  okapi_makePosPIDWithSensor(Motor, Potentiometer);
  okapi_makePosPIDWithSensor(Motor, IntegratedEncoder);
  okapi_makePosPIDWithSensor(MotorGroup, ADIEncoder);
  okapi_makePosPIDWithSensor(MotorGroup, ADIGyro);
  okapi_makePosPIDWithSensor(MotorGroup, Potentiometer);
  okapi_makePosPIDWithSensor(MotorGroup, IntegratedEncoder);

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
  posPID(const std::shared_ptr<ControllerInput<double>> &iinput,
         const std::shared_ptr<ControllerOutput<double>> &ioutput,
         double ikP,
         double ikI,
         double ikD,
         double ikBias = 0,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>(),
         const TimeUtil &itimeUtil = TimeUtilFactory::create());

  okapi_makeVelPID(Motor);
  okapi_makeVelPID(MotorGroup);
  okapi_makeVelPIDWithSensor(Motor, ADIEncoder);
  okapi_makeVelPIDWithSensor(Motor, ADIGyro);
  okapi_makeVelPIDWithSensor(Motor, Potentiometer);
  okapi_makeVelPIDWithSensor(Motor, IntegratedEncoder);
  okapi_makeVelPIDWithSensor(MotorGroup, ADIEncoder);
  okapi_makeVelPIDWithSensor(MotorGroup, ADIGyro);
  okapi_makeVelPIDWithSensor(MotorGroup, Potentiometer);
  okapi_makeVelPIDWithSensor(MotorGroup, IntegratedEncoder);

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
  velPID(const std::shared_ptr<ControllerInput<double>> &iinput,
         const std::shared_ptr<ControllerOutput<double>> &ioutput,
         double ikP,
         double ikD,
         double ikF = 0,
         double ikSF = 0,
         double iTPR = imev5TPR,
         std::unique_ptr<Filter> iderivativeFilter = std::make_unique<PassthroughFilter>(),
         const TimeUtil &itimeUtil = TimeUtilFactory::create());

  /**
   * A controller which generates and follows 2D motion profiles.
   *
   * @param imaxVel The maximum possible velocity in m/s.
   * @param imaxAccel The maximum possible acceleration in m/s/s.
   * @param imaxJerk The maximum possible jerk in m/s/s/s.
   * @param ichassis The chassis to control.
   */
  static AsyncMotionProfileController
  motionProfile(double imaxVel,
                double imaxAccel,
                double imaxJerk,
                const ChassisController &ichassis,
                const TimeUtil &itimeUtil = TimeUtilFactory::create());

  /**
   * A controller which generates and follows 2D motion profiles.
   *
   * @param imaxVel The maximum possible velocity in m/s.
   * @param imaxAccel The maximum possible acceleration in m/s/s.
   * @param imaxJerk The maximum possible jerk in m/s/s/s.
   * @param imodel The chassis model to control.
   * @param iwidth The chassis wheelbase width.
   */
  static AsyncMotionProfileController
  motionProfile(double imaxVel,
                double imaxAccel,
                double imaxJerk,
                const std::shared_ptr<ChassisModel> &imodel,
                const ChassisScales &iscales,
                AbstractMotor::GearsetRatioPair ipair,
                const TimeUtil &itimeUtil = TimeUtilFactory::create());

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
                      const std::shared_ptr<ControllerOutput<double>> &ioutput,
                      const TimeUtil &itimeUtil = TimeUtilFactory::create());
};
} // namespace okapi
