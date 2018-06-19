/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncControllerBuilder.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/control/iterative/iterativeVelPidController.hpp"

namespace okapi {
AsyncControllerBuilder::AsyncControllerBuilder() = default;

AsyncControllerBuilder::~AsyncControllerBuilder() = default;

AsyncControllerBuilder AsyncControllerBuilder::input(ADIEncoder iencoder) {
  m_input = std::make_shared<ADIEncoder>(iencoder);
}
AsyncControllerBuilder AsyncControllerBuilder::input(IntegratedEncoder iencoder) {
  m_input = std::make_shared<IntegratedEncoder>(iencoder);
}
AsyncControllerBuilder AsyncControllerBuilder::input(MotorGroup imotor) {
  m_input = std::make_shared<IntegratedEncoder>(imotor);
}
AsyncControllerBuilder AsyncControllerBuilder::input(Potentiometer ipotentiometer) {
  m_input = std::make_shared<Potentiometer>(ipotentiometer);
}
AsyncControllerBuilder AsyncControllerBuilder::input(ADIUltrasonic iultrasonic) {
  m_input = std::make_shared<ADIUltrasonic>(iultrasonic);
}

AsyncControllerBuilder AsyncControllerBuilder::filter(EmaFilter ifilter) {
  m_filters.push_back(std::make_shared<EmaFilter>(ifilter));
}
AsyncControllerBuilder AsyncControllerBuilder::filter(DemaFilter ifilter) {
  m_filters.push_back(std::make_shared<DemaFilter>(ifilter));
}
AsyncControllerBuilder AsyncControllerBuilder::filter(ComposableFilter ifilter) {
  m_filters.push_back(std::make_shared<ComposableFilter>(ifilter));
}
AsyncControllerBuilder AsyncControllerBuilder::filter(std::shared_ptr<Filter> ifilter) {
  m_filters.push_back(ifilter);
}

AsyncControllerBuilder AsyncControllerBuilder::posPid(const double ikP, const double ikI,
                                                      const double ikD, const double ikBias = 0) {
  m_controllers.push_back < std::make_shared<IterativePosPIDController>(ikP, ikI, ikD, ikBias);
}

AsyncControllerBuilder AsyncControllerBuilder::velPid(const double ikP, const double ikD,
                                                      const double ikF,
                                                      const VelMathArgs &iparams) {
  m_controllers.push_back < std::make_shared<IterativeVelPIDController>(ikP, ikD, ikF, iparams);
}

AsyncControllerBuilder AsyncControllerBuilder::output(Motor imotor) {
  m_output = std::make_shared<Motor>(imotor);
}
AsyncControllerBuilder AsyncControllerBuilder::output(MotorGroup imotor) {
  m_output = std::make_shared<MotorGroup>(imotor);
}
AsyncControllerBuilder AsyncControllerBuilder::output(std::shared_ptr<AbstractMotor> imotor) {
  m_output = imotor;
}

std::shared_ptr<AsyncController> AsyncControllerBuilder::build() const {
  auto outFilter = std::make_shared<ComposableFilter>(m_filters);

  return NULL;
}
} // namespace okapi
