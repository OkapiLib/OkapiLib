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
#include "okapi/api/control/iterative/iterativeLambdaBasedController.hpp"
#include "okapi/api/filter/composableFilter.hpp"
#include "okapi/api/filter/demaFilter.hpp"
#include "okapi/api/filter/emaFilter.hpp"
#include "okapi/api/filter/velMath.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include "okapi/impl/device/adiUltrasonic.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "okapi/impl/device/rotarysensor/adiEncoder.hpp"
#include "okapi/impl/device/rotarysensor/integratedEncoder.hpp"
#include "okapi/impl/device/rotarysensor/potentiometer.hpp"
#include <vector>

namespace okapi {
class AsyncControllerBuilder {
  public:
  AsyncControllerBuilder(const TimeUtil &itimeUtil);

  virtual ~AsyncControllerBuilder();

  AsyncControllerBuilder &input(ADIEncoder iencoder);
  AsyncControllerBuilder &input(IntegratedEncoder iencoder);
  //  AsyncControllerBuilder &input(Motor imotor);
  AsyncControllerBuilder &input(MotorGroup imotor);
  AsyncControllerBuilder &input(Potentiometer ipotentiometer);
  AsyncControllerBuilder &input(ADIUltrasonic iultrasonic);

  AsyncControllerBuilder &filter(EmaFilter ifilter);
  AsyncControllerBuilder &filter(DemaFilter ifilter);
  AsyncControllerBuilder &filter(ComposableFilter ifilter);
  AsyncControllerBuilder &filter(std::shared_ptr<Filter> ifilter);

  AsyncControllerBuilder &posPid(double ikP, double ikI, double ikD, double ikBias = 0);

  AsyncControllerBuilder &velPid(double ikP, double ikD, double ikF,
                                 std::unique_ptr<VelMath> ivelMath);

  AsyncControllerBuilder &lambda(std::function<double(double)> istepFunction);

  AsyncControllerBuilder &output(Motor imotor);
  AsyncControllerBuilder &output(MotorGroup imotor);
  AsyncControllerBuilder &output(std::shared_ptr<AbstractMotor> imotor);

  std::unique_ptr<AsyncController<double, double>> build() const;

  private:
  TimeUtil timeUtil;

  std::shared_ptr<ControllerInput<double>> m_input;
  std::vector<std::shared_ptr<Filter>> m_filters{};
  std::vector<std::unique_ptr<IterativeController<double, double>>> m_controllers{};
  std::shared_ptr<ControllerOutput<double>> m_output;
};
} // namespace okapi

#endif
