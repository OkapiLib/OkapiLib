/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/chassis/controller/chassisControllerBuilder.hpp"
#include "okapi/api/chassis/model/threeEncoderSkidSteerModel.hpp"
#include "okapi/api/chassis/model/threeEncoderXDriveModel.hpp"
#include "okapi/api/odometry/threeEncoderOdometry.hpp"
#include "okapi/impl/util/configurableTimeUtilFactory.hpp"
#include "okapi/impl/util/rate.hpp"
#include "okapi/impl/util/timer.hpp"
#include <stdexcept>

namespace okapi {
ChassisControllerBuilder::ChassisControllerBuilder(const std::shared_ptr<Logger> &ilogger)
  : logger(ilogger) {
}

ChassisControllerBuilder &ChassisControllerBuilder::withMotors(const Motor &ileft,
                                                               const Motor &iright) {
  return withMotors(std::make_shared<Motor>(ileft), std::make_shared<Motor>(iright));
}

ChassisControllerBuilder &ChassisControllerBuilder::withMotors(const MotorGroup &ileft,
                                                               const MotorGroup &iright) {
  return withMotors(std::make_shared<MotorGroup>(ileft), std::make_shared<MotorGroup>(iright));
}

ChassisControllerBuilder &
ChassisControllerBuilder::withMotors(const std::shared_ptr<AbstractMotor> &ileft,
                                     const std::shared_ptr<AbstractMotor> &iright) {
  hasMotors = true;
  driveMode = DriveMode::SkidSteer;
  skidSteerMotors = {ileft, iright};

  if (!sensorsSetByUser) {
    leftSensor = ileft->getEncoder();
    rightSensor = iright->getEncoder();
  }

  if (!maxVelSetByUser) {
    maxVelocity = toUnderlyingType(ileft->getGearing());
  }

  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withMotors(const Motor &itopLeft,
                                                               const Motor &itopRight,
                                                               const Motor &ibottomRight,
                                                               const Motor &ibottomLeft) {
  return withMotors(std::make_shared<Motor>(itopLeft),
                    std::make_shared<Motor>(itopRight),
                    std::make_shared<Motor>(ibottomRight),
                    std::make_shared<Motor>(ibottomLeft));
}

ChassisControllerBuilder &ChassisControllerBuilder::withMotors(const MotorGroup &itopLeft,
                                                               const MotorGroup &itopRight,
                                                               const MotorGroup &ibottomRight,
                                                               const MotorGroup &ibottomLeft) {
  return withMotors(std::make_shared<MotorGroup>(itopLeft),
                    std::make_shared<MotorGroup>(itopRight),
                    std::make_shared<MotorGroup>(ibottomRight),
                    std::make_shared<MotorGroup>(ibottomLeft));
}

ChassisControllerBuilder &
ChassisControllerBuilder::withMotors(const std::shared_ptr<AbstractMotor> &itopLeft,
                                     const std::shared_ptr<AbstractMotor> &itopRight,
                                     const std::shared_ptr<AbstractMotor> &ibottomRight,
                                     const std::shared_ptr<AbstractMotor> &ibottomLeft) {
  hasMotors = true;
  driveMode = DriveMode::XDrive;
  xDriveMotors = {itopLeft, itopRight, ibottomRight, ibottomLeft};

  if (!sensorsSetByUser) {
    leftSensor = itopLeft->getEncoder();
    rightSensor = itopRight->getEncoder();
  }

  if (!maxVelSetByUser) {
    maxVelocity = toUnderlyingType(itopLeft->getGearing());
  }

  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withMotors(const Motor &ileft,
                                                               const Motor &iright,
                                                               const Motor &imiddle) {
  return withMotors(std::make_shared<Motor>(ileft),
                    std::make_shared<Motor>(iright),
                    std::make_shared<Motor>(imiddle));
}

ChassisControllerBuilder &ChassisControllerBuilder::withMotors(const MotorGroup &ileft,
                                                               const MotorGroup &iright,
                                                               const MotorGroup &imiddle) {
  return withMotors(std::make_shared<MotorGroup>(ileft),
                    std::make_shared<MotorGroup>(iright),
                    std::make_shared<MotorGroup>(imiddle));
}

ChassisControllerBuilder &ChassisControllerBuilder::withMotors(const MotorGroup &ileft,
                                                               const MotorGroup &iright,
                                                               const Motor &imiddle) {
  return withMotors(std::make_shared<MotorGroup>(ileft),
                    std::make_shared<MotorGroup>(iright),
                    std::make_shared<Motor>(imiddle));
}

ChassisControllerBuilder &
ChassisControllerBuilder::withMotors(const std::shared_ptr<AbstractMotor> &ileft,
                                     const std::shared_ptr<AbstractMotor> &iright,
                                     const std::shared_ptr<AbstractMotor> &imiddle) {
  hasMotors = true;
  driveMode = DriveMode::HDrive;
  hDriveMotors = {ileft, iright, imiddle};

  if (!sensorsSetByUser) {
    leftSensor = ileft->getEncoder();
    rightSensor = iright->getEncoder();
    middleSensor = imiddle->getEncoder();
  }

  if (!maxVelSetByUser) {
    maxVelocity = toUnderlyingType(ileft->getGearing());
  }

  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withSensors(const ADIEncoder &ileft,
                                                                const ADIEncoder &iright) {
  return withSensors(std::make_shared<ADIEncoder>(ileft), std::make_shared<ADIEncoder>(iright));
}

ChassisControllerBuilder &ChassisControllerBuilder::withSensors(const okapi::ADIEncoder &ileft,
                                                                const okapi::ADIEncoder &iright,
                                                                const okapi::ADIEncoder &imiddle) {
  return withSensors(std::make_shared<ADIEncoder>(ileft),
                     std::make_shared<ADIEncoder>(iright),
                     std::make_shared<ADIEncoder>(imiddle));
}

ChassisControllerBuilder &ChassisControllerBuilder::withSensors(const RotationSensor &ileft,
                                                                const RotationSensor &iright) {
  return withSensors(std::make_shared<RotationSensor>(ileft),
                     std::make_shared<RotationSensor>(iright));
}

ChassisControllerBuilder &
ChassisControllerBuilder::withSensors(const okapi::RotationSensor &ileft,
                                      const okapi::RotationSensor &iright,
                                      const okapi::RotationSensor &imiddle) {
  return withSensors(std::make_shared<RotationSensor>(ileft),
                     std::make_shared<RotationSensor>(iright),
                     std::make_shared<RotationSensor>(imiddle));
}

ChassisControllerBuilder &ChassisControllerBuilder::withSensors(const IntegratedEncoder &ileft,
                                                                const IntegratedEncoder &iright) {
  return withSensors(std::make_shared<IntegratedEncoder>(ileft),
                     std::make_shared<IntegratedEncoder>(iright));
}

ChassisControllerBuilder &
ChassisControllerBuilder::withSensors(const okapi::IntegratedEncoder &ileft,
                                      const okapi::IntegratedEncoder &iright,
                                      const okapi::ADIEncoder &imiddle) {
  return withSensors(std::make_shared<IntegratedEncoder>(ileft),
                     std::make_shared<IntegratedEncoder>(iright),
                     std::make_shared<ADIEncoder>(imiddle));
}

ChassisControllerBuilder &
ChassisControllerBuilder::withSensors(const std::shared_ptr<ContinuousRotarySensor> &ileft,
                                      const std::shared_ptr<ContinuousRotarySensor> &iright) {
  sensorsSetByUser = true;
  leftSensor = ileft;
  rightSensor = iright;
  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withSensors(
  const std::shared_ptr<okapi::ContinuousRotarySensor> &ileft,
  const std::shared_ptr<okapi::ContinuousRotarySensor> &iright,
  const std::shared_ptr<okapi::ContinuousRotarySensor> &imiddle) {
  sensorsSetByUser = true;
  leftSensor = ileft;
  rightSensor = iright;
  middleSensor = imiddle;
  return *this;
}

ChassisControllerBuilder &
ChassisControllerBuilder::withGains(const IterativePosPIDController::Gains &idistanceGains,
                                    const IterativePosPIDController::Gains &iturnGains) {
  return withGains(idistanceGains, iturnGains, iturnGains);
}

ChassisControllerBuilder &
ChassisControllerBuilder::withGains(const IterativePosPIDController::Gains &idistanceGains,
                                    const IterativePosPIDController::Gains &iturnGains,
                                    const IterativePosPIDController::Gains &iangleGains) {
  hasGains = true;
  distanceGains = idistanceGains;
  turnGains = iturnGains;
  angleGains = iangleGains;
  return *this;
}

ChassisControllerBuilder &
ChassisControllerBuilder::withDerivativeFilters(std::unique_ptr<Filter> idistanceFilter,
                                                std::unique_ptr<Filter> iturnFilter,
                                                std::unique_ptr<Filter> iangleFilter) {
  distanceFilter = std::move(idistanceFilter);
  turnFilter = std::move(iturnFilter);
  angleFilter = std::move(iangleFilter);
  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withOdometry(const StateMode &imode,
                                                                 const QLength &imoveThreshold,
                                                                 const QAngle &iturnThreshold) {
  hasOdom = true;
  odometry = nullptr;
  stateMode = imode;
  moveThreshold = imoveThreshold;
  turnThreshold = iturnThreshold;
  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withOdometry(const ChassisScales &iodomScales,
                                                                 const StateMode &imode,
                                                                 const QLength &imoveThreshold,
                                                                 const QAngle &iturnThreshold) {
  hasOdom = true;
  differentOdomScales = true;
  odomScales = iodomScales;
  odometry = nullptr;
  stateMode = imode;
  moveThreshold = imoveThreshold;
  turnThreshold = iturnThreshold;
  return *this;
}

ChassisControllerBuilder &
ChassisControllerBuilder::withOdometry(std::shared_ptr<Odometry> iodometry,
                                       const StateMode &imode,
                                       const QLength &imoveThreshold,
                                       const QAngle &iturnThreshold) {
  if (iodometry == nullptr) {
    std::string msg = "ChassisControllerBuilder: Odometry cannot be null.";
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  hasOdom = true;
  odometry = std::move(iodometry);
  stateMode = imode;
  moveThreshold = imoveThreshold;
  turnThreshold = iturnThreshold;
  return *this;
}

ChassisControllerBuilder &
ChassisControllerBuilder::withDimensions(const AbstractMotor::gearset &igearset,
                                         const ChassisScales &iscales) {
  gearset = igearset;

  if (!maxVelSetByUser) {
    maxVelocity = toUnderlyingType(igearset);
  }

  driveScales = iscales;
  if (!differentOdomScales) {
    odomScales = driveScales;
  }
  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withMaxVelocity(const double imaxVelocity) {
  maxVelSetByUser = true;
  maxVelocity = imaxVelocity;
  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withMaxVoltage(const double imaxVoltage) {
  maxVoltage = imaxVoltage;
  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withChassisControllerTimeUtilFactory(
  const TimeUtilFactory &itimeUtilFactory) {
  chassisControllerTimeUtilFactory = itimeUtilFactory;
  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::withClosedLoopControllerTimeUtilFactory(
  const TimeUtilFactory &itimeUtilFactory) {
  closedLoopControllerTimeUtilFactory = itimeUtilFactory;
  return *this;
}

ChassisControllerBuilder &
ChassisControllerBuilder::withClosedLoopControllerTimeUtil(const double iatTargetError,
                                                           const double iatTargetDerivative,
                                                           const QTime &iatTargetTime) {
  closedLoopControllerTimeUtilFactory =
    ConfigurableTimeUtilFactory(iatTargetError, iatTargetDerivative, iatTargetTime);
  return *this;
}

ChassisControllerBuilder &
ChassisControllerBuilder::withOdometryTimeUtilFactory(const TimeUtilFactory &itimeUtilFactory) {
  odometryTimeUtilFactory = itimeUtilFactory;
  return *this;
}

ChassisControllerBuilder &
ChassisControllerBuilder::withLogger(const std::shared_ptr<Logger> &ilogger) {
  controllerLogger = ilogger;
  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::parentedToCurrentTask() {
  isParentedToCurrentTask = true;
  return *this;
}

ChassisControllerBuilder &ChassisControllerBuilder::notParentedToCurrentTask() {
  isParentedToCurrentTask = false;
  return *this;
}

std::shared_ptr<ChassisController> ChassisControllerBuilder::build() {
  if (!hasMotors) {
    std::string msg("ChassisControllerBuilder: No motors given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  if (gearset.internalGearset == AbstractMotor::gearset::invalid) {
    // Invalid by default. The user must provide one.
    std::string msg("ChassisControllerBuilder: A gearset was not provided.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  if (maxVelSetByUser && maxVelocity > toUnderlyingType(gearset.internalGearset)) {
    LOG_WARN(
      "ChassisControllerBuilder: The custom maximum velocity (" + std::to_string(maxVelocity) +
      ") is greater than the maximum velocity of the selected gearset (" +
      std::to_string(toUnderlyingType(gearset.internalGearset)) + "). This is probably a bug.");
  }

  std::shared_ptr<ChassisController> out;

  if (hasGains) {
    out = buildCCPID();
  } else {
    out = buildCCI();
  }

  return out;
}

std::shared_ptr<OdomChassisController> ChassisControllerBuilder::buildOdometry() {
  if (!hasOdom) {
    std::string msg("ChassisControllerBuilder: No odometry information given.");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }

  return buildDOCC(build());
}

std::shared_ptr<DefaultOdomChassisController>
ChassisControllerBuilder::buildDOCC(std::shared_ptr<ChassisController> chassisController) {
  if (odometry == nullptr) {
    if (middleSensor == nullptr) {
      odometry = std::make_shared<TwoEncoderOdometry>(odometryTimeUtilFactory.create(),
                                                      chassisController->getModel(),
                                                      odomScales,
                                                      controllerLogger);
    } else {
      odometry = std::make_shared<ThreeEncoderOdometry>(odometryTimeUtilFactory.create(),
                                                        chassisController->getModel(),
                                                        odomScales,
                                                        controllerLogger);
    }
  }

  auto out =
    std::make_shared<DefaultOdomChassisController>(chassisControllerTimeUtilFactory.create(),
                                                   std::move(odometry),
                                                   chassisController,
                                                   stateMode,
                                                   moveThreshold,
                                                   turnThreshold,
                                                   controllerLogger);

  out->startOdomThread();

  if (isParentedToCurrentTask && NOT_INITIALIZE_TASK && NOT_COMP_INITIALIZE_TASK) {
    out->getOdomThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
  }

  return out;
}

std::shared_ptr<ChassisControllerPID> ChassisControllerBuilder::buildCCPID() {
  auto out = std::make_shared<ChassisControllerPID>(
    chassisControllerTimeUtilFactory.create(),
    makeChassisModel(),
    std::make_unique<IterativePosPIDController>(distanceGains,
                                                closedLoopControllerTimeUtilFactory.create(),
                                                std::move(distanceFilter),
                                                controllerLogger),
    std::make_unique<IterativePosPIDController>(turnGains,
                                                closedLoopControllerTimeUtilFactory.create(),
                                                std::move(turnFilter),
                                                controllerLogger),
    std::make_unique<IterativePosPIDController>(angleGains,
                                                closedLoopControllerTimeUtilFactory.create(),
                                                std::move(angleFilter),
                                                controllerLogger),
    gearset,
    driveScales,
    controllerLogger);

  out->startThread();

  if (isParentedToCurrentTask && NOT_INITIALIZE_TASK && NOT_COMP_INITIALIZE_TASK) {
    out->getThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
  }

  return out;
}

std::shared_ptr<ChassisControllerIntegrated> ChassisControllerBuilder::buildCCI() {
  std::shared_ptr<AbstractMotor> leftMotorGroup;
  std::shared_ptr<AbstractMotor> rightMotorGroup;

  switch (driveMode) {
  case DriveMode::SkidSteer:
    leftMotorGroup = skidSteerMotors.left;
    rightMotorGroup = skidSteerMotors.right;
    break;

  case DriveMode::XDrive:
    leftMotorGroup = std::make_shared<MotorGroup>(
      std::initializer_list({xDriveMotors.topLeft, xDriveMotors.bottomLeft}), controllerLogger);
    rightMotorGroup = std::make_shared<MotorGroup>(
      std::initializer_list({xDriveMotors.topRight, xDriveMotors.bottomRight}), controllerLogger);
    break;

  case DriveMode::HDrive:
    leftMotorGroup = hDriveMotors.left;
    rightMotorGroup = hDriveMotors.right;
    break;
  }

  return std::make_shared<ChassisControllerIntegrated>(
    chassisControllerTimeUtilFactory.create(),
    makeChassisModel(),
    std::make_unique<AsyncPosIntegratedController>(leftMotorGroup,
                                                   gearset,
                                                   maxVelocity,
                                                   closedLoopControllerTimeUtilFactory.create(),
                                                   controllerLogger),
    std::make_unique<AsyncPosIntegratedController>(rightMotorGroup,
                                                   gearset,
                                                   maxVelocity,
                                                   closedLoopControllerTimeUtilFactory.create(),
                                                   controllerLogger),
    gearset,
    driveScales,
    controllerLogger);
}

std::shared_ptr<ChassisModel> ChassisControllerBuilder::makeChassisModel() {
  // These implementations should handle a null middleSensor
  switch (driveMode) {
  case DriveMode::SkidSteer:
    return makeSkidSteerModel();

  case DriveMode::XDrive:
    return makeXDriveModel();

  case DriveMode::HDrive:
    return makeHDriveModel();

  default:
    std::string msg =
      "ChassisControllerBuilder: BUG: Unhandled DriveMode case in makeChassisModel.";
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
  }
}

std::shared_ptr<SkidSteerModel> ChassisControllerBuilder::makeSkidSteerModel() {
  if (middleSensor != nullptr) {
    return std::make_shared<ThreeEncoderSkidSteerModel>(skidSteerMotors.left,
                                                        skidSteerMotors.right,
                                                        leftSensor,
                                                        rightSensor,
                                                        middleSensor,
                                                        maxVelocity,
                                                        maxVoltage);
  } else {
    return std::make_shared<SkidSteerModel>(skidSteerMotors.left,
                                            skidSteerMotors.right,
                                            leftSensor,
                                            rightSensor,
                                            maxVelocity,
                                            maxVoltage);
  }
}

std::shared_ptr<XDriveModel> ChassisControllerBuilder::makeXDriveModel() {
  if (middleSensor != nullptr) {
    return std::make_shared<ThreeEncoderXDriveModel>(xDriveMotors.topLeft,
                                                     xDriveMotors.topRight,
                                                     xDriveMotors.bottomRight,
                                                     xDriveMotors.bottomLeft,
                                                     leftSensor,
                                                     rightSensor,
                                                     middleSensor,
                                                     maxVelocity,
                                                     maxVoltage);
  } else {
    return std::make_shared<XDriveModel>(xDriveMotors.topLeft,
                                         xDriveMotors.topRight,
                                         xDriveMotors.bottomRight,
                                         xDriveMotors.bottomLeft,
                                         leftSensor,
                                         rightSensor,
                                         maxVelocity,
                                         maxVoltage);
  }
}

std::shared_ptr<HDriveModel> ChassisControllerBuilder::makeHDriveModel() {
  // HDriveModel already has three encoders
  return std::make_shared<HDriveModel>(hDriveMotors.left,
                                       hDriveMotors.right,
                                       hDriveMotors.middle,
                                       leftSensor,
                                       rightSensor,
                                       middleSensor,
                                       maxVelocity,
                                       maxVoltage);
}
} // namespace okapi
