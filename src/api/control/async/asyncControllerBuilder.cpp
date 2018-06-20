/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/control/async/asyncControllerBuilder.hpp"
#include "okapi/api/control/async/asyncWrapper.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/control/iterative/iterativeVelPidController.hpp"
#include "okapi/api/filter/filteredControllerInput.hpp"

namespace okapi {
AsyncControllerBuilder::AsyncControllerBuilder() = default;

AsyncControllerBuilder::~AsyncControllerBuilder() = default;

AsyncControllerBuilder &AsyncControllerBuilder::input(ADIEncoder iencoder) {
  m_input = std::make_shared<ADIEncoder>(iencoder);
  return *this;
}
AsyncControllerBuilder &AsyncControllerBuilder::input(IntegratedEncoder iencoder) {
  m_input = std::make_shared<IntegratedEncoder>(iencoder);
  return *this;
}
AsyncControllerBuilder &AsyncControllerBuilder::input(MotorGroup imotor) {
  m_input = std::make_shared<IntegratedEncoder>(imotor.getEncoder());
  return *this;
}
AsyncControllerBuilder &AsyncControllerBuilder::input(Potentiometer ipotentiometer) {
  m_input = std::make_shared<Potentiometer>(ipotentiometer);
  return *this;
}
AsyncControllerBuilder &AsyncControllerBuilder::input(ADIUltrasonic iultrasonic) {
  m_input = std::make_shared<ADIUltrasonic>(iultrasonic);
  return *this;
}

AsyncControllerBuilder &AsyncControllerBuilder::filter(EmaFilter ifilter) {
  m_filters.push_back(std::make_shared<EmaFilter>(ifilter));
  return *this;
}
AsyncControllerBuilder &AsyncControllerBuilder::filter(DemaFilter ifilter) {
  m_filters.push_back(std::make_shared<DemaFilter>(ifilter));
  return *this;
}
AsyncControllerBuilder &AsyncControllerBuilder::filter(ComposableFilter ifilter) {
  m_filters.push_back(std::make_shared<ComposableFilter>(ifilter));
  return *this;
}
AsyncControllerBuilder &AsyncControllerBuilder::filter(std::shared_ptr<Filter> ifilter) {
  m_filters.push_back(ifilter);
  return *this;
}

AsyncControllerBuilder &AsyncControllerBuilder::posPid(const double ikP, const double ikI,
                                                       const double ikD, const double ikBias) {
  m_controllers.push_back(
    std::move(std::make_unique<IterativePosPIDController>(ikP, ikI, ikD, ikBias)));
  return *this;
}

AsyncControllerBuilder &AsyncControllerBuilder::velPid(const double ikP, const double ikD,
                                                       const double ikF,
                                                       const VelMathArgs &iparams) {
  m_controllers.push_back(
    std::move(std::make_unique<IterativeVelPIDController>(ikP, ikD, ikF, iparams)));
  return *this;
}

AsyncControllerBuilder &
AsyncControllerBuilder::lambda(std::function<double(double)> istepFunction) {
  m_controllers.push_back(
    std::move(std::make_unique<IterativeLambdaBasedController>(istepFunction)));
  return *this;
}

AsyncControllerBuilder &AsyncControllerBuilder::output(Motor imotor) {
  m_output = std::make_shared<Motor>(imotor);
  return *this;
}
AsyncControllerBuilder &AsyncControllerBuilder::output(MotorGroup imotor) {
  m_output = std::make_shared<MotorGroup>(imotor);
  return *this;
}
AsyncControllerBuilder &AsyncControllerBuilder::output(std::shared_ptr<AbstractMotor> imotor) {
  m_output = imotor;
  return *this;
}

std::unique_ptr<AsyncController> AsyncControllerBuilder::build() const {
  auto outFilter = std::make_shared<ComposableFilter>(m_filters);
  return std::make_unique<AsyncWrapper>(m_input, m_output, m_controllers[0]);
}
} // namespace okapi
