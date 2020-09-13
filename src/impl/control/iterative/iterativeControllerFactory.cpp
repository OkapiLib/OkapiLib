/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/iterative/iterativeControllerFactory.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

namespace okapi {
IterativePosPIDController
IterativeControllerFactory::posPID(const double ikP,
                                   const double ikI,
                                   const double ikD,
                                   const double ikBias,
                                   std::unique_ptr<Filter> iderivativeFilter,
                                   const std::shared_ptr<Logger> &ilogger) {
  return IterativePosPIDController(
    ikP, ikI, ikD, ikBias, TimeUtilFactory::createDefault(), std::move(iderivativeFilter), ilogger);
}

IterativeVelPIDController
IterativeControllerFactory::velPID(const double ikP,
                                   const double ikD,
                                   const double ikF,
                                   const double ikSF,
                                   std::unique_ptr<VelMath> ivelMath,
                                   std::unique_ptr<Filter> iderivativeFilter,
                                   const std::shared_ptr<Logger> &ilogger) {
  return IterativeVelPIDController(ikP,
                                   ikD,
                                   ikF,
                                   ikSF,
                                   std::move(ivelMath),
                                   TimeUtilFactory::createDefault(),
                                   std::move(iderivativeFilter),
                                   ilogger);
}

IterativeMotorVelocityController
IterativeControllerFactory::motorVelocity(Motor imotor,
                                          const double ikP,
                                          const double ikD,
                                          const double ikF,
                                          const double ikSF,
                                          std::unique_ptr<VelMath> ivelMath,
                                          std::unique_ptr<Filter> iderivativeFilter,
                                          const std::shared_ptr<Logger> &ilogger) {
  return IterativeMotorVelocityController(
    std::make_shared<Motor>(imotor),
    std::make_shared<IterativeVelPIDController>(
      velPID(ikP, ikD, ikF, ikSF, std::move(ivelMath), std::move(iderivativeFilter), ilogger)));
}

IterativeMotorVelocityController
IterativeControllerFactory::motorVelocity(MotorGroup imotor,
                                          const double ikP,
                                          const double ikD,
                                          const double ikF,
                                          const double ikSF,
                                          std::unique_ptr<VelMath> ivelMath,
                                          std::unique_ptr<Filter> iderivativeFilter,
                                          const std::shared_ptr<Logger> &ilogger) {
  return IterativeMotorVelocityController(
    std::make_shared<MotorGroup>(imotor),
    std::make_shared<IterativeVelPIDController>(
      velPID(ikP, ikD, ikF, ikSF, std::move(ivelMath), std::move(iderivativeFilter), ilogger)));
}

IterativeMotorVelocityController IterativeControllerFactory::motorVelocity(
  Motor imotor,
  std::shared_ptr<IterativeVelocityController<double, double>> icontroller) {
  return IterativeMotorVelocityController(std::make_shared<Motor>(imotor), icontroller);
}

IterativeMotorVelocityController IterativeControllerFactory::motorVelocity(
  MotorGroup imotor,
  std::shared_ptr<IterativeVelocityController<double, double>> icontroller) {
  return IterativeMotorVelocityController(std::make_shared<MotorGroup>(imotor), icontroller);
}
} // namespace okapi
