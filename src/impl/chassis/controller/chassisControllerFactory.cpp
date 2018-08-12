/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/chassis/controller/chassisControllerFactory.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/chassis/model/threeEncoderSkidSteerModel.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

namespace okapi {
ChassisControllerIntegrated
ChassisControllerFactory::create(Motor ileftSideMotor, Motor irightSideMotor,
                                 const AbstractMotor::GearsetRatioPair igearset,
                                 const ChassisScales &iscales) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  return ChassisControllerIntegrated(
    TimeUtilFactory::create(), std::make_shared<SkidSteerModel>(leftMtr, rightMtr),
    std::make_unique<AsyncPosIntegratedController>(leftMtr, TimeUtilFactory::create()),
    std::make_unique<AsyncPosIntegratedController>(rightMtr, TimeUtilFactory::create()), igearset,
    iscales);
}

ChassisControllerIntegrated
ChassisControllerFactory::create(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                                 const AbstractMotor::GearsetRatioPair igearset,
                                 const ChassisScales &iscales) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  return ChassisControllerIntegrated(
    TimeUtilFactory::create(), std::make_shared<SkidSteerModel>(leftMtr, rightMtr),
    std::make_unique<AsyncPosIntegratedController>(leftMtr, TimeUtilFactory::create()),
    std::make_unique<AsyncPosIntegratedController>(rightMtr, TimeUtilFactory::create()), igearset,
    iscales);
}

ChassisControllerIntegrated ChassisControllerFactory::create(
  Motor itopLeftMotor, Motor itopRightMotor, Motor ibottomRightMotor, Motor ibottomLeftMotor,
  const AbstractMotor::GearsetRatioPair igearset, const ChassisScales &iscales) {
  auto topLeftMtr = std::make_shared<Motor>(itopLeftMotor);
  auto topRightMtr = std::make_shared<Motor>(itopRightMotor);
  auto bottomRightMtr = std::make_shared<Motor>(ibottomRightMotor);
  auto bottomLeftMtr = std::make_shared<Motor>(ibottomLeftMotor);
  return ChassisControllerIntegrated(
    TimeUtilFactory::create(),
    std::make_shared<XDriveModel>(topLeftMtr, topRightMtr, bottomRightMtr, bottomLeftMtr),
    std::make_unique<AsyncPosIntegratedController>(topLeftMtr, TimeUtilFactory::create()),
    std::make_unique<AsyncPosIntegratedController>(topRightMtr, TimeUtilFactory::create()),
    igearset, iscales);
}

ChassisControllerPID
ChassisControllerFactory::create(Motor ileftSideMotor, Motor irightSideMotor,
                                 const IterativePosPIDController::Gains &idistanceArgs,
                                 const IterativePosPIDController::Gains &iangleArgs,
                                 const AbstractMotor::GearsetRatioPair igearset,
                                 const ChassisScales &iscales) {
  return create(ileftSideMotor, irightSideMotor, idistanceArgs, iangleArgs, iangleArgs, igearset,
                iscales);
}

ChassisControllerPID
ChassisControllerFactory::create(Motor ileftSideMotor, Motor irightSideMotor,
                                 const IterativePosPIDController::Gains &idistanceArgs,
                                 const IterativePosPIDController::Gains &iangleArgs,
                                 const IterativePosPIDController::Gains &iturnArgs,
                                 const AbstractMotor::GearsetRatioPair igearset,
                                 const ChassisScales &iscales) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  return ChassisControllerPID(
    TimeUtilFactory::create(), std::make_shared<SkidSteerModel>(leftMtr, rightMtr),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()), igearset,
    iscales);
}

ChassisControllerPID
ChassisControllerFactory::create(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                                 const IterativePosPIDController::Gains &idistanceArgs,
                                 const IterativePosPIDController::Gains &iangleArgs,
                                 const AbstractMotor::GearsetRatioPair igearset,
                                 const ChassisScales &iscales) {
  return create(ileftSideMotor, irightSideMotor, idistanceArgs, iangleArgs, iangleArgs, igearset,
                iscales);
}

ChassisControllerPID
ChassisControllerFactory::create(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                                 const IterativePosPIDController::Gains &idistanceArgs,
                                 const IterativePosPIDController::Gains &iangleArgs,
                                 const IterativePosPIDController::Gains &iturnArgs,
                                 const AbstractMotor::GearsetRatioPair igearset,
                                 const ChassisScales &iscales) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  return ChassisControllerPID(
    TimeUtilFactory::create(), std::make_shared<SkidSteerModel>(leftMtr, rightMtr),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()), igearset,
    iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  MotorGroup ileftSideMotor, MotorGroup irightSideMotor, ADIEncoder ileftEnc, ADIEncoder irightEnc,
  const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const AbstractMotor::GearsetRatioPair igearset, const ChassisScales &iscales) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  return create(ileftSideMotor, irightSideMotor, ileftEnc, irightEnc, idistanceArgs, iangleArgs,
                iangleArgs, igearset, iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  MotorGroup ileftSideMotor, MotorGroup irightSideMotor, ADIEncoder ileftEnc, ADIEncoder irightEnc,
  const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const IterativePosPIDController::Gains &iturnArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  return ChassisControllerPID(
    TimeUtilFactory::create(),
    std::make_shared<SkidSteerModel>(leftMtr, rightMtr, std::make_shared<ADIEncoder>(ileftEnc),
                                     std::make_shared<ADIEncoder>(irightEnc)),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()), igearset,
    iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  std::shared_ptr<AbstractMotor> ileftSideMotor, std::shared_ptr<AbstractMotor> irightSideMotor,
  std::shared_ptr<ContinuousRotarySensor> ileftEnc,
  std::shared_ptr<ContinuousRotarySensor> irightEnc,
  const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const AbstractMotor::GearsetRatioPair igearset, const ChassisScales &iscales) {
  return create(ileftSideMotor, irightSideMotor, ileftEnc, irightEnc, idistanceArgs, iangleArgs,
                iangleArgs, igearset, iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  std::shared_ptr<AbstractMotor> ileftSideMotor, std::shared_ptr<AbstractMotor> irightSideMotor,
  std::shared_ptr<ContinuousRotarySensor> ileftEnc,
  std::shared_ptr<ContinuousRotarySensor> irightEnc,
  const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const IterativePosPIDController::Gains &iturnArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales) {
  return ChassisControllerPID(
    TimeUtilFactory::create(),
    std::make_shared<SkidSteerModel>(ileftSideMotor, irightSideMotor, ileftEnc, irightEnc),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()), igearset,
    iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  Motor itopLeftMotor, Motor itopRightMotor, Motor ibottomRightMotor, Motor ibottomLeftMotor,
  const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const AbstractMotor::GearsetRatioPair igearset, const ChassisScales &iscales) {
  return create(itopLeftMotor, itopRightMotor, ibottomRightMotor, ibottomLeftMotor, idistanceArgs,
                iangleArgs, iangleArgs, igearset, iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  Motor itopLeftMotor, Motor itopRightMotor, Motor ibottomRightMotor, Motor ibottomLeftMotor,
  const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const IterativePosPIDController::Gains &iturnArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales) {
  auto topLeftMtr = std::make_shared<Motor>(itopLeftMotor);
  auto topRightMtr = std::make_shared<Motor>(itopRightMotor);
  auto bottomRightMtr = std::make_shared<Motor>(ibottomRightMotor);
  auto bottomLeftMtr = std::make_shared<Motor>(ibottomLeftMotor);
  return ChassisControllerPID(
    TimeUtilFactory::create(),
    std::make_shared<XDriveModel>(topLeftMtr, topRightMtr, bottomRightMtr, bottomLeftMtr),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()), igearset,
    iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  Motor itopLeftMotor, Motor itopRightMotor, Motor ibottomRightMotor, Motor ibottomLeftMotor,
  ADIEncoder itopLeftEnc, ADIEncoder itopRightEnc,
  const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const AbstractMotor::GearsetRatioPair igearset, const ChassisScales &iscales) {
  return create(itopLeftMotor, itopRightMotor, ibottomRightMotor, ibottomLeftMotor, itopLeftEnc,
                itopRightEnc, idistanceArgs, iangleArgs, iangleArgs, igearset, iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  Motor itopLeftMotor, Motor itopRightMotor, Motor ibottomRightMotor, Motor ibottomLeftMotor,
  ADIEncoder itopLeftEnc, ADIEncoder itopRightEnc,
  const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const IterativePosPIDController::Gains &iturnArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales) {
  auto topLeftMtr = std::make_shared<Motor>(itopLeftMotor);
  auto topRightMtr = std::make_shared<Motor>(itopRightMotor);
  auto bottomRightMtr = std::make_shared<Motor>(ibottomRightMotor);
  auto bottomLeftMtr = std::make_shared<Motor>(ibottomLeftMotor);
  return ChassisControllerPID(
    TimeUtilFactory::create(),
    std::make_shared<XDriveModel>(topLeftMtr, topRightMtr, bottomRightMtr, bottomLeftMtr,
                                  std::make_shared<ADIEncoder>(itopLeftEnc),
                                  std::make_shared<ADIEncoder>(itopRightEnc)),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()), igearset,
    iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  std::shared_ptr<AbstractMotor> itopLeftMotor, std::shared_ptr<AbstractMotor> itopRightMotor,
  std::shared_ptr<AbstractMotor> ibottomRightMotor, std::shared_ptr<AbstractMotor> ibottomLeftMotor,
  std::shared_ptr<ContinuousRotarySensor> itopLeftEnc,
  std::shared_ptr<ContinuousRotarySensor> itopRightEnc,
  const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const AbstractMotor::GearsetRatioPair igearset, const ChassisScales &iscales) {
  return create(itopLeftMotor, itopRightMotor, ibottomRightMotor, ibottomLeftMotor, itopLeftEnc,
                itopRightEnc, idistanceArgs, iangleArgs, iangleArgs, igearset, iscales);
}

ChassisControllerPID ChassisControllerFactory::create(
  std::shared_ptr<AbstractMotor> itopLeftMotor, std::shared_ptr<AbstractMotor> itopRightMotor,
  std::shared_ptr<AbstractMotor> ibottomRightMotor, std::shared_ptr<AbstractMotor> ibottomLeftMotor,
  std::shared_ptr<ContinuousRotarySensor> itopLeftEnc,
  std::shared_ptr<ContinuousRotarySensor> itopRightEnc,
  const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const IterativePosPIDController::Gains &iturnArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales) {
  return ChassisControllerPID(
    TimeUtilFactory::create(),
    std::make_shared<XDriveModel>(itopLeftMotor, itopRightMotor, ibottomRightMotor,
                                  ibottomLeftMotor, itopLeftEnc, itopRightEnc),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()), igearset,
    iscales);
}

OdomChassisControllerIntegrated
ChassisControllerFactory::createOdom(Motor ileftSideMotor, Motor irightSideMotor,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model = std::make_shared<SkidSteerModel>(leftMtr, rightMtr);
  return OdomChassisControllerIntegrated(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<AsyncPosIntegratedController>(leftMtr, TimeUtilFactory::create()),
    std::make_unique<AsyncPosIntegratedController>(rightMtr, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerIntegrated
ChassisControllerFactory::createOdom(Motor ileftSideMotor, Motor irightSideMotor,
                                     ADIEncoder ileftEnc, ADIEncoder irightEnc,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model =
    std::make_shared<SkidSteerModel>(leftMtr, rightMtr, std::make_shared<ADIEncoder>(ileftEnc),
                                     std::make_shared<ADIEncoder>(irightEnc));
  return OdomChassisControllerIntegrated(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<AsyncPosIntegratedController>(leftMtr, TimeUtilFactory::create()),
    std::make_unique<AsyncPosIntegratedController>(rightMtr, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerIntegrated ChassisControllerFactory::createOdom(
  Motor ileftSideMotor, Motor irightSideMotor, ADIEncoder ileftEnc, ADIEncoder irightEnc,
  ADIEncoder imiddleEnc, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model = std::make_shared<ThreeEncoderSkidSteerModel>(
    leftMtr, rightMtr, std::make_shared<ADIEncoder>(ileftEnc),
    std::make_shared<ADIEncoder>(irightEnc), std::make_shared<ADIEncoder>(imiddleEnc));
  return OdomChassisControllerIntegrated(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<AsyncPosIntegratedController>(leftMtr, TimeUtilFactory::create()),
    std::make_unique<AsyncPosIntegratedController>(rightMtr, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerIntegrated
ChassisControllerFactory::createOdom(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model = std::make_shared<SkidSteerModel>(leftMtr, rightMtr);
  return OdomChassisControllerIntegrated(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<AsyncPosIntegratedController>(leftMtr, TimeUtilFactory::create()),
    std::make_unique<AsyncPosIntegratedController>(rightMtr, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerIntegrated
ChassisControllerFactory::createOdom(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                                     ADIEncoder ileftEnc, ADIEncoder irightEnc,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model =
    std::make_shared<SkidSteerModel>(leftMtr, rightMtr, std::make_shared<ADIEncoder>(ileftEnc),
                                     std::make_shared<ADIEncoder>(irightEnc));
  return OdomChassisControllerIntegrated(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<AsyncPosIntegratedController>(leftMtr, TimeUtilFactory::create()),
    std::make_unique<AsyncPosIntegratedController>(rightMtr, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerIntegrated ChassisControllerFactory::createOdom(
  MotorGroup ileftSideMotor, MotorGroup irightSideMotor, ADIEncoder ileftEnc, ADIEncoder irightEnc,
  ADIEncoder imiddleEnc, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model = std::make_shared<ThreeEncoderSkidSteerModel>(
    leftMtr, rightMtr, std::make_shared<ADIEncoder>(ileftEnc),
    std::make_shared<ADIEncoder>(irightEnc), std::make_shared<ADIEncoder>(imiddleEnc));
  return OdomChassisControllerIntegrated(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<AsyncPosIntegratedController>(leftMtr, TimeUtilFactory::create()),
    std::make_unique<AsyncPosIntegratedController>(rightMtr, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(Motor ileftSideMotor, Motor irightSideMotor,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model = std::make_shared<SkidSteerModel>(leftMtr, rightMtr);
  return OdomChassisControllerPID(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(Motor ileftSideMotor, Motor irightSideMotor,
                                     ADIEncoder ileftEnc, ADIEncoder irightEnc,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model =
    std::make_shared<SkidSteerModel>(leftMtr, rightMtr, std::make_shared<ADIEncoder>(ileftEnc),
                                     std::make_shared<ADIEncoder>(irightEnc));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerPID ChassisControllerFactory::createOdom(
  Motor ileftSideMotor, Motor irightSideMotor, ADIEncoder ileftEnc, ADIEncoder irightEnc,
  ADIEncoder imiddleEnc, const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const AbstractMotor::GearsetRatioPair igearset, const ChassisScales &iscales,
  const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model = std::make_shared<ThreeEncoderSkidSteerModel>(
    leftMtr, rightMtr, std::make_shared<ADIEncoder>(ileftEnc),
    std::make_shared<ADIEncoder>(irightEnc), std::make_shared<ADIEncoder>(imiddleEnc));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model = std::make_shared<SkidSteerModel>(leftMtr, rightMtr);
  return OdomChassisControllerPID(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                                     ADIEncoder ileftEnc, ADIEncoder irightEnc,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model =
    std::make_shared<SkidSteerModel>(leftMtr, rightMtr, std::make_shared<ADIEncoder>(ileftEnc),
                                     std::make_shared<ADIEncoder>(irightEnc));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerPID ChassisControllerFactory::createOdom(
  MotorGroup ileftSideMotor, MotorGroup irightSideMotor, ADIEncoder ileftEnc, ADIEncoder irightEnc,
  ADIEncoder imiddleEnc, const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const AbstractMotor::GearsetRatioPair igearset, const ChassisScales &iscales,
  const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model = std::make_shared<ThreeEncoderSkidSteerModel>(
    leftMtr, rightMtr, std::make_shared<ADIEncoder>(ileftEnc),
    std::make_shared<ADIEncoder>(irightEnc), std::make_shared<ADIEncoder>(imiddleEnc));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(Motor ileftSideMotor, Motor irightSideMotor,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const IterativePosPIDController::Gains &iturnArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model = std::make_shared<SkidSteerModel>(leftMtr, rightMtr);
  return OdomChassisControllerPID(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerPID ChassisControllerFactory::createOdom(
  Motor ileftSideMotor, Motor irightSideMotor, ADIEncoder ileftEnc, ADIEncoder irightEnc,
  const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const IterativePosPIDController::Gains &iturnArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model =
    std::make_shared<SkidSteerModel>(leftMtr, rightMtr, std::make_shared<ADIEncoder>(ileftEnc),
                                     std::make_shared<ADIEncoder>(irightEnc));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerPID ChassisControllerFactory::createOdom(
  Motor ileftSideMotor, Motor irightSideMotor, ADIEncoder ileftEnc, ADIEncoder irightEnc,
  ADIEncoder imiddleEnc, const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const IterativePosPIDController::Gains &iturnArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model = std::make_shared<ThreeEncoderSkidSteerModel>(
    leftMtr, rightMtr, std::make_shared<ADIEncoder>(ileftEnc),
    std::make_shared<ADIEncoder>(irightEnc), std::make_shared<ADIEncoder>(imiddleEnc));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(MotorGroup ileftSideMotor, MotorGroup irightSideMotor,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const IterativePosPIDController::Gains &iturnArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model = std::make_shared<SkidSteerModel>(leftMtr, rightMtr);
  return OdomChassisControllerPID(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerPID ChassisControllerFactory::createOdom(
  MotorGroup ileftSideMotor, MotorGroup irightSideMotor, ADIEncoder ileftEnc, ADIEncoder irightEnc,
  const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const IterativePosPIDController::Gains &iturnArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model =
    std::make_shared<SkidSteerModel>(leftMtr, rightMtr, std::make_shared<ADIEncoder>(ileftEnc),
                                     std::make_shared<ADIEncoder>(irightEnc));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}

OdomChassisControllerPID ChassisControllerFactory::createOdom(
  MotorGroup ileftSideMotor, MotorGroup irightSideMotor, ADIEncoder ileftEnc, ADIEncoder irightEnc,
  ADIEncoder imiddleEnc, const IterativePosPIDController::Gains &idistanceArgs,
  const IterativePosPIDController::Gains &iangleArgs,
  const IterativePosPIDController::Gains &iturnArgs, const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales, const QLength imoveThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model = std::make_shared<ThreeEncoderSkidSteerModel>(
    leftMtr, rightMtr, std::make_shared<ADIEncoder>(ileftEnc),
    std::make_shared<ADIEncoder>(irightEnc), std::make_shared<ADIEncoder>(imiddleEnc));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(), model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()), igearset,
    iscales, imoveThreshold);
}
} // namespace okapi
