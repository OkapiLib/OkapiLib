/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ASYNCCONTROLLERBUILDER_HPP_
#define _OKAPI_ASYNCCONTROLLERBUILDER_HPP_

#include "okapi/api/control/async/asyncController.hpp"
#include "okapi/api/control/iterative/iterativeController.hpp"
#include "okapi/api/device/adiUltrasonic.hpp"
#include "okapi/api/device/motor/motor.hpp"
#include "okapi/api/device/motor/motorGroup.hpp"
#include "okapi/api/device/rotarysensor/adiEncoder.hpp"
#include "okapi/api/device/rotarysensor/integratedEncoder.hpp"
#include "okapi/api/device/rotarysensor/potentiometer.hpp"
#include "okapi/api/filter/composableFilter.hpp"
#include "okapi/api/filter/demaFilter.hpp"
#include "okapi/api/filter/emaFilter.hpp"
#include "okapi/api/filter/velMath.hpp"
#include <vector>

namespace okapi {
class AsyncControllerBuilder {
  public:
  AsyncControllerBuilder();

  virtual ~AsyncControllerBuilder();

  AsyncControllerBuilder &input(ADIEncoder iencoder);
  AsyncControllerBuilder &input(IntegratedEncoder iencoder);
  AsyncControllerBuilder &input(MotorGroup imotor);
  AsyncControllerBuilder &input(Potentiometer ipotentiometer);
  AsyncControllerBuilder &input(ADIUltrasonic iultrasonic);

  AsyncControllerBuilder &filter(EmaFilter ifilter);
  AsyncControllerBuilder &filter(DemaFilter ifilter);
  AsyncControllerBuilder &filter(ComposableFilter ifilter);
  AsyncControllerBuilder &filter(std::shared_ptr<Filter> ifilter);

  AsyncControllerBuilder &posPid(const double ikP, const double ikI, const double ikD,
                                 const double ikBias = 0);

  AsyncControllerBuilder &velPid(const double ikP, const double ikD, const double ikF,
                                 const VelMathArgs &iparams);

  AsyncControllerBuilder &output(Motor imotor);
  AsyncControllerBuilder &output(MotorGroup imotor);
  AsyncControllerBuilder &output(std::shared_ptr<AbstractMotor> imotor);

  std::shared_ptr<AsyncController> build() const;

  private:
  std::shared_ptr<ControllerInput> m_input;
  std::vector<std::shared_ptr<Filter>> m_filters{};
  std::vector<std::shared_ptr<IterativeController>> m_controllers{};
  std::shared_ptr<ControllerOutput> m_output;
};
} // namespace okapi

#endif
