/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/impl/chassis/model/chassisModelFactory.hpp"

namespace okapi {
SkidSteerModel ChassisModelFactory::create(Motor ileftSideMotor,
                                           Motor irightSideMotor,
                                           const double imaxVelocity,
                                           const double imaxVoltage) {
  return SkidSteerModel(std::make_shared<Motor>(ileftSideMotor),
                        std::make_shared<Motor>(irightSideMotor),
                        imaxVelocity,
                        imaxVoltage);
}

SkidSteerModel ChassisModelFactory::create(MotorGroup ileftSideMotor,
                                           MotorGroup irightSideMotor,
                                           const double imaxVelocity,
                                           const double imaxVoltage) {
  return SkidSteerModel(std::make_shared<MotorGroup>(ileftSideMotor),
                        std::make_shared<MotorGroup>(irightSideMotor),
                        imaxVelocity,
                        imaxVoltage);
}

SkidSteerModel ChassisModelFactory::create(MotorGroup ileftSideMotor,
                                           MotorGroup irightSideMotor,
                                           ADIEncoder ileftEnc,
                                           ADIEncoder irightEnc,
                                           const double imaxVelocity,
                                           const double imaxVoltage) {
  return SkidSteerModel(std::make_shared<MotorGroup>(ileftSideMotor),
                        std::make_shared<MotorGroup>(irightSideMotor),
                        std::make_shared<ADIEncoder>(ileftEnc),
                        std::make_shared<ADIEncoder>(irightEnc),
                        imaxVelocity,
                        imaxVoltage);
}

XDriveModel ChassisModelFactory::create(Motor itopLeftMotor,
                                        Motor itopRightMotor,
                                        Motor ibottomRightMotor,
                                        Motor ibottomLeftMotor,
                                        const double imaxVelocity,
                                        const double imaxVoltage) {
  return XDriveModel(std::make_shared<Motor>(itopLeftMotor),
                     std::make_shared<Motor>(itopRightMotor),
                     std::make_shared<Motor>(ibottomRightMotor),
                     std::make_shared<Motor>(ibottomLeftMotor),
                     imaxVelocity,
                     imaxVoltage);
}

XDriveModel ChassisModelFactory::create(Motor itopLeftMotor,
                                        Motor itopRightMotor,
                                        Motor ibottomRightMotor,
                                        Motor ibottomLeftMotor,
                                        ADIEncoder ileftEnc,
                                        ADIEncoder irightEnc,
                                        const double imaxVelocity,
                                        const double imaxVoltage) {
  return XDriveModel(std::make_shared<Motor>(itopLeftMotor),
                     std::make_shared<Motor>(itopRightMotor),
                     std::make_shared<Motor>(ibottomRightMotor),
                     std::make_shared<Motor>(ibottomLeftMotor),
                     std::make_shared<ADIEncoder>(ileftEnc),
                     std::make_shared<ADIEncoder>(irightEnc),
                     imaxVelocity,
                     imaxVoltage);
}

ThreeEncoderSkidSteerModel ChassisModelFactory::create(Motor ileftSideMotor,
                                                       Motor irightSideMotor,
                                                       ADIEncoder ileftEnc,
                                                       ADIEncoder imiddleEnc,
                                                       ADIEncoder irightEnc,
                                                       const double imaxVelocity,
                                                       const double imaxVoltage) {
  return ThreeEncoderSkidSteerModel(std::make_shared<Motor>(ileftSideMotor),
                                    std::make_shared<Motor>(irightSideMotor),
                                    std::make_shared<ADIEncoder>(ileftEnc),
                                    std::make_shared<ADIEncoder>(imiddleEnc),
                                    std::make_shared<ADIEncoder>(irightEnc),
                                    imaxVelocity,
                                    imaxVoltage);
}

ThreeEncoderSkidSteerModel ChassisModelFactory::create(MotorGroup ileftSideMotor,
                                                       MotorGroup irightSideMotor,
                                                       ADIEncoder ileftEnc,
                                                       ADIEncoder imiddleEnc,
                                                       ADIEncoder irightEnc,
                                                       const double imaxVelocity,
                                                       const double imaxVoltage) {
  return ThreeEncoderSkidSteerModel(std::make_shared<MotorGroup>(ileftSideMotor),
                                    std::make_shared<MotorGroup>(irightSideMotor),
                                    std::make_shared<ADIEncoder>(ileftEnc),
                                    std::make_shared<ADIEncoder>(imiddleEnc),
                                    std::make_shared<ADIEncoder>(irightEnc),
                                    imaxVelocity,
                                    imaxVoltage);
}
} // namespace okapi
