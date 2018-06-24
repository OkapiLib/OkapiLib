/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ASYNCCONTROLLERFACTORY_HPP_
#define _OKAPI_ASYNCCONTROLLERFACTORY_HPP_

#include "okapi/api/control/async/asyncPosIntegratedController.hpp"
#include "okapi/api/control/async/asyncVelIntegratedController.hpp"
#include "okapi/impl/control/async/asyncPosPidController.hpp"
#include "okapi/impl/control/async/asyncVelPidController.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"

namespace okapi {
class AsyncControllerFactory {
  public:
  static AsyncPosIntegratedController posIntegrated(Motor imotor);
  static AsyncPosIntegratedController posIntegrated(MotorGroup imotor);

  static AsyncVelIntegratedController velIntegrated(Motor imotor);
  static AsyncVelIntegratedController velIntegrated(MotorGroup imotor);

  static AsyncPosPIDController posPID(Motor imotor, const double ikP, const double ikI,
                                      const double ikD, const double ikBias = 0);
  static AsyncPosPIDController posPID(MotorGroup imotor, const double ikP, const double ikI,
                                      const double ikD, const double ikBias = 0);
  static AsyncPosPIDController posPID(std::shared_ptr<ControllerInput> iinput,
                                      std::shared_ptr<ControllerOutput> ioutput, const double ikP,
                                      const double ikI, const double ikD, const double ikBias = 0);

  static AsyncVelPIDController velPID(Motor imotor, const double ikP, const double ikD,
                                      const double ikF = 0);
  static AsyncVelPIDController velPID(MotorGroup imotor, const double ikP, const double ikD,
                                      const double ikF = 0);
  static AsyncVelPIDController velPID(std::shared_ptr<ControllerInput> iinput,
                                      std::shared_ptr<ControllerOutput> ioutput, const double ikP,
                                      const double ikD, const double ikF = 0);
};
} // namespace okapi

#endif
