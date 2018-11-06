/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/control/async/asyncControllerFactory.hpp"
#include "okapi/impl/filter/velMathFactory.hpp"

namespace okapi {
AsyncPosIntegratedController AsyncControllerFactory::posIntegrated(Motor imotor,
                                                                   std::int32_t imaxVelocity,
                                                                   const TimeUtil &itimeUtil) {
  return AsyncPosIntegratedController(std::make_shared<Motor>(imotor), imaxVelocity, itimeUtil);
}

AsyncPosIntegratedController AsyncControllerFactory::posIntegrated(MotorGroup imotor,
                                                                   std::int32_t imaxVelocity,
                                                                   const TimeUtil &itimeUtil) {
  return AsyncPosIntegratedController(
    std::make_shared<MotorGroup>(imotor), imaxVelocity, itimeUtil);
}

AsyncVelIntegratedController AsyncControllerFactory::velIntegrated(Motor imotor,
                                                                   const TimeUtil &itimeUtil) {
  return AsyncVelIntegratedController(std::make_shared<Motor>(imotor), itimeUtil);
}

AsyncVelIntegratedController AsyncControllerFactory::velIntegrated(MotorGroup imotor,
                                                                   const TimeUtil &itimeUtil) {
  return AsyncVelIntegratedController(std::make_shared<MotorGroup>(imotor), itimeUtil);
}

AsyncPosPIDController
AsyncControllerFactory::posPID(const std::shared_ptr<ControllerInput<double>> &iinput,
                               const std::shared_ptr<ControllerOutput<double>> &ioutput,
                               const double ikP,
                               const double ikI,
                               const double ikD,
                               const double ikBias,
                               std::unique_ptr<Filter> iderivativeFilter,
                               const TimeUtil &itimeUtil) {
  AsyncPosPIDController out(
    iinput, ioutput, itimeUtil, ikP, ikI, ikD, ikBias, std::move(iderivativeFilter));
  out.startThread();
  return out;
}

AsyncVelPIDController
AsyncControllerFactory::velPID(const std::shared_ptr<ControllerInput<double>> &iinput,
                               const std::shared_ptr<ControllerOutput<double>> &ioutput,
                               const double ikP,
                               const double ikD,
                               const double ikF,
                               const double ikSF,
                               const double iTPR,
                               std::unique_ptr<Filter> iderivativeFilter,
                               const TimeUtil &itimeUtil) {
  AsyncVelPIDController out(iinput,
                            ioutput,
                            itimeUtil,
                            ikP,
                            ikD,
                            ikF,
                            ikSF,
                            VelMathFactory::createPtr(iTPR),
                            std::move(iderivativeFilter));
  out.startThread();
  return out;
}

AsyncMotionProfileController
AsyncControllerFactory::motionProfile(double imaxVel,
                                      double imaxAccel,
                                      double imaxJerk,
                                      const ChassisController &ichassis,
                                      const TimeUtil &itimeUtil) {
  return motionProfile(imaxVel,
                       imaxAccel,
                       imaxJerk,
                       ichassis.getChassisModel(),
                       ichassis.getChassisScales(),
                       ichassis.getGearsetRatioPair());
}

AsyncMotionProfileController
AsyncControllerFactory::motionProfile(double imaxVel,
                                      double imaxAccel,
                                      double imaxJerk,
                                      const std::shared_ptr<ChassisModel> &imodel,
                                      const ChassisScales &iscales,
                                      AbstractMotor::GearsetRatioPair ipair,
                                      const TimeUtil &itimeUtil) {
  AsyncMotionProfileController out(itimeUtil, imaxVel, imaxAccel, imaxJerk, imodel, iscales, ipair);
  out.startThread();
  return out;
}

AsyncLinearMotionProfileController
linearMotionProfile(double imaxVel,
                    double imaxAccel,
                    double imaxJerk,
                    const std::shared_ptr<ControllerOutput<double>> &ioutput,
                    const TimeUtil &itimeUtil) {
  AsyncLinearMotionProfileController out(itimeUtil, imaxVel, imaxAccel, imaxJerk, ioutput);
  out.startThread();
  return out;
}
} // namespace okapi
