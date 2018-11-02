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
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"

namespace okapi {
OdomChassisControllerIntegrated
ChassisControllerFactory::createOdom(Motor ileftSideMotor,
                                     Motor irightSideMotor,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model =
    std::make_shared<SkidSteerModel>(leftMtr, rightMtr, toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerIntegrated(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<AsyncPosIntegratedController>(leftMtr, TimeUtilFactory::create()),
    std::make_unique<AsyncPosIntegratedController>(rightMtr, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerIntegrated
ChassisControllerFactory::createOdom(Motor ileftSideMotor,
                                     Motor irightSideMotor,
                                     ADIEncoder ileftEnc,
                                     ADIEncoder irightEnc,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model = std::make_shared<SkidSteerModel>(leftMtr,
                                                rightMtr,
                                                std::make_shared<ADIEncoder>(ileftEnc),
                                                std::make_shared<ADIEncoder>(irightEnc),
                                                toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerIntegrated(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<AsyncPosIntegratedController>(leftMtr, TimeUtilFactory::create()),
    std::make_unique<AsyncPosIntegratedController>(rightMtr, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerIntegrated
ChassisControllerFactory::createOdom(Motor ileftSideMotor,
                                     Motor irightSideMotor,
                                     ADIEncoder ileftEnc,
                                     ADIEncoder irightEnc,
                                     ADIEncoder imiddleEnc,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model =
    std::make_shared<ThreeEncoderSkidSteerModel>(leftMtr,
                                                 rightMtr,
                                                 std::make_shared<ADIEncoder>(ileftEnc),
                                                 std::make_shared<ADIEncoder>(irightEnc),
                                                 std::make_shared<ADIEncoder>(imiddleEnc),
                                                 toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerIntegrated(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<AsyncPosIntegratedController>(leftMtr, TimeUtilFactory::create()),
    std::make_unique<AsyncPosIntegratedController>(rightMtr, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerIntegrated
ChassisControllerFactory::createOdom(MotorGroup ileftSideMotor,
                                     MotorGroup irightSideMotor,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model =
    std::make_shared<SkidSteerModel>(leftMtr, rightMtr, toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerIntegrated(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<AsyncPosIntegratedController>(leftMtr, TimeUtilFactory::create()),
    std::make_unique<AsyncPosIntegratedController>(rightMtr, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerIntegrated
ChassisControllerFactory::createOdom(MotorGroup ileftSideMotor,
                                     MotorGroup irightSideMotor,
                                     ADIEncoder ileftEnc,
                                     ADIEncoder irightEnc,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model = std::make_shared<SkidSteerModel>(leftMtr,
                                                rightMtr,
                                                std::make_shared<ADIEncoder>(ileftEnc),
                                                std::make_shared<ADIEncoder>(irightEnc),
                                                toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerIntegrated(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<AsyncPosIntegratedController>(leftMtr, TimeUtilFactory::create()),
    std::make_unique<AsyncPosIntegratedController>(rightMtr, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerIntegrated
ChassisControllerFactory::createOdom(MotorGroup ileftSideMotor,
                                     MotorGroup irightSideMotor,
                                     ADIEncoder ileftEnc,
                                     ADIEncoder irightEnc,
                                     ADIEncoder imiddleEnc,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model =
    std::make_shared<ThreeEncoderSkidSteerModel>(leftMtr,
                                                 rightMtr,
                                                 std::make_shared<ADIEncoder>(ileftEnc),
                                                 std::make_shared<ADIEncoder>(irightEnc),
                                                 std::make_shared<ADIEncoder>(imiddleEnc),
                                                 toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerIntegrated(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<AsyncPosIntegratedController>(leftMtr, TimeUtilFactory::create()),
    std::make_unique<AsyncPosIntegratedController>(rightMtr, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(Motor ileftSideMotor,
                                     Motor irightSideMotor,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model =
    std::make_shared<SkidSteerModel>(leftMtr, rightMtr, toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(Motor ileftSideMotor,
                                     Motor irightSideMotor,
                                     ADIEncoder ileftEnc,
                                     ADIEncoder irightEnc,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model = std::make_shared<SkidSteerModel>(leftMtr,
                                                rightMtr,
                                                std::make_shared<ADIEncoder>(ileftEnc),
                                                std::make_shared<ADIEncoder>(irightEnc),
                                                toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(Motor ileftSideMotor,
                                     Motor irightSideMotor,
                                     ADIEncoder ileftEnc,
                                     ADIEncoder irightEnc,
                                     ADIEncoder imiddleEnc,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model =
    std::make_shared<ThreeEncoderSkidSteerModel>(leftMtr,
                                                 rightMtr,
                                                 std::make_shared<ADIEncoder>(ileftEnc),
                                                 std::make_shared<ADIEncoder>(irightEnc),
                                                 std::make_shared<ADIEncoder>(imiddleEnc),
                                                 toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(MotorGroup ileftSideMotor,
                                     MotorGroup irightSideMotor,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model =
    std::make_shared<SkidSteerModel>(leftMtr, rightMtr, toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(MotorGroup ileftSideMotor,
                                     MotorGroup irightSideMotor,
                                     ADIEncoder ileftEnc,
                                     ADIEncoder irightEnc,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model = std::make_shared<SkidSteerModel>(leftMtr,
                                                rightMtr,
                                                std::make_shared<ADIEncoder>(ileftEnc),
                                                std::make_shared<ADIEncoder>(irightEnc),
                                                toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(MotorGroup ileftSideMotor,
                                     MotorGroup irightSideMotor,
                                     ADIEncoder ileftEnc,
                                     ADIEncoder irightEnc,
                                     ADIEncoder imiddleEnc,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model =
    std::make_shared<ThreeEncoderSkidSteerModel>(leftMtr,
                                                 rightMtr,
                                                 std::make_shared<ADIEncoder>(ileftEnc),
                                                 std::make_shared<ADIEncoder>(irightEnc),
                                                 std::make_shared<ADIEncoder>(imiddleEnc),
                                                 toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(Motor ileftSideMotor,
                                     Motor irightSideMotor,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const IterativePosPIDController::Gains &iturnArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model =
    std::make_shared<SkidSteerModel>(leftMtr, rightMtr, toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(Motor ileftSideMotor,
                                     Motor irightSideMotor,
                                     ADIEncoder ileftEnc,
                                     ADIEncoder irightEnc,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const IterativePosPIDController::Gains &iturnArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model = std::make_shared<SkidSteerModel>(leftMtr,
                                                rightMtr,
                                                std::make_shared<ADIEncoder>(ileftEnc),
                                                std::make_shared<ADIEncoder>(irightEnc),
                                                toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(Motor ileftSideMotor,
                                     Motor irightSideMotor,
                                     ADIEncoder ileftEnc,
                                     ADIEncoder irightEnc,
                                     ADIEncoder imiddleEnc,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const IterativePosPIDController::Gains &iturnArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<Motor>(ileftSideMotor);
  auto rightMtr = std::make_shared<Motor>(irightSideMotor);
  auto model =
    std::make_shared<ThreeEncoderSkidSteerModel>(leftMtr,
                                                 rightMtr,
                                                 std::make_shared<ADIEncoder>(ileftEnc),
                                                 std::make_shared<ADIEncoder>(irightEnc),
                                                 std::make_shared<ADIEncoder>(imiddleEnc),
                                                 toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(MotorGroup ileftSideMotor,
                                     MotorGroup irightSideMotor,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const IterativePosPIDController::Gains &iturnArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model =
    std::make_shared<SkidSteerModel>(leftMtr, rightMtr, toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(MotorGroup ileftSideMotor,
                                     MotorGroup irightSideMotor,
                                     ADIEncoder ileftEnc,
                                     ADIEncoder irightEnc,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const IterativePosPIDController::Gains &iturnArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model = std::make_shared<SkidSteerModel>(leftMtr,
                                                rightMtr,
                                                std::make_shared<ADIEncoder>(ileftEnc),
                                                std::make_shared<ADIEncoder>(irightEnc),
                                                toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}

OdomChassisControllerPID
ChassisControllerFactory::createOdom(MotorGroup ileftSideMotor,
                                     MotorGroup irightSideMotor,
                                     ADIEncoder ileftEnc,
                                     ADIEncoder irightEnc,
                                     ADIEncoder imiddleEnc,
                                     const IterativePosPIDController::Gains &idistanceArgs,
                                     const IterativePosPIDController::Gains &iangleArgs,
                                     const IterativePosPIDController::Gains &iturnArgs,
                                     const AbstractMotor::GearsetRatioPair igearset,
                                     const ChassisScales &iscales,
                                     const QLength imoveThreshold,
                                     const QAngle iturnThreshold) {
  auto leftMtr = std::make_shared<MotorGroup>(ileftSideMotor);
  auto rightMtr = std::make_shared<MotorGroup>(irightSideMotor);
  auto model =
    std::make_shared<ThreeEncoderSkidSteerModel>(leftMtr,
                                                 rightMtr,
                                                 std::make_shared<ADIEncoder>(ileftEnc),
                                                 std::make_shared<ADIEncoder>(irightEnc),
                                                 std::make_shared<ADIEncoder>(imiddleEnc),
                                                 toUnderlyingType(igearset.internalGearset));
  return OdomChassisControllerPID(
    TimeUtilFactory::create(),
    model,
    std::make_unique<Odometry>(model, iscales, TimeUtilFactory::create().getRate()),
    std::make_unique<IterativePosPIDController>(idistanceArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iangleArgs, TimeUtilFactory::create()),
    std::make_unique<IterativePosPIDController>(iturnArgs, TimeUtilFactory::create()),
    igearset,
    iscales,
    imoveThreshold,
    iturnThreshold);
}
} // namespace okapi
