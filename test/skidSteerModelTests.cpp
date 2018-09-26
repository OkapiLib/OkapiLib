/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class SkidSteerModelTest : public ::testing::Test {
  public:
  SkidSteerModelTest() : model(leftMotor, rightMotor, 127) {
  }

  void assertAllMotorsLastVelocity(const std::int16_t expectedLastVelocity) const {
    EXPECT_EQ(leftMotor->lastVelocity, expectedLastVelocity);
    EXPECT_EQ(rightMotor->lastVelocity, expectedLastVelocity);
  }

  void assertAllMotorsLastVoltage(const std::int16_t expectedLastVoltage) const {
    EXPECT_EQ(leftMotor->lastVoltage, expectedLastVoltage);
    EXPECT_EQ(rightMotor->lastVoltage, expectedLastVoltage);
  }

  void assertLeftAndRightMotorsLastVelocity(const std::int16_t expectedLeftLastVelocity,
                                            const std::int16_t expectedRightLastVelocity) const {
    EXPECT_EQ(leftMotor->lastVelocity, expectedLeftLastVelocity);
    EXPECT_EQ(rightMotor->lastVelocity, expectedRightLastVelocity);
  }

  void assertLeftAndRightMotorsLastVoltage(const std::int16_t expectedLeftLastVoltage,
                                           const std::int16_t expectedRightLastVoltage) const {
    EXPECT_EQ(leftMotor->lastVoltage, expectedLeftLastVoltage);
    EXPECT_EQ(rightMotor->lastVoltage, expectedRightLastVoltage);
  }

  std::shared_ptr<MockMotor> leftMotor = std::make_shared<MockMotor>();
  std::shared_ptr<MockMotor> rightMotor = std::make_shared<MockMotor>();
  SkidSteerModel model;
};

TEST_F(SkidSteerModelTest, ForwardHalfPower) {
  model.forward(0.5);
  assertAllMotorsLastVelocity(63);
}

TEST_F(SkidSteerModelTest, ForwardBoundsInput) {
  model.forward(10);
  assertAllMotorsLastVelocity(127);
}

TEST_F(SkidSteerModelTest, RotateHalfPower) {
  model.rotate(0.5);
  assertLeftAndRightMotorsLastVelocity(63, -63);
}

TEST_F(SkidSteerModelTest, RotateBoundsInput) {
  model.rotate(10);
  assertLeftAndRightMotorsLastVelocity(127, -127);
}

TEST_F(SkidSteerModelTest, DriveVectorHalfPower) {
  model.driveVector(0.25, 0.25);
  assertLeftAndRightMotorsLastVelocity(63, 0);
}

TEST_F(SkidSteerModelTest, DriveVectorBoundsInput) {
  model.driveVector(0.9, 0.25);
  assertLeftAndRightMotorsLastVelocity(127, 71);
}

TEST_F(SkidSteerModelTest, StopTest) {
  leftMotor->lastVelocity = 100;
  rightMotor->lastVelocity = 100;

  model.stop();

  assertAllMotorsLastVelocity(0);
}

TEST_F(SkidSteerModelTest, LeftHalfPower) {
  model.left(0.5);
  assertLeftAndRightMotorsLastVelocity(63, 0);
}

TEST_F(SkidSteerModelTest, LeftBoundsInput) {
  model.left(10);
  assertLeftAndRightMotorsLastVelocity(127, 0);
}

TEST_F(SkidSteerModelTest, RightHalfPower) {
  model.right(0.5);
  assertLeftAndRightMotorsLastVelocity(0, 63);
}

TEST_F(SkidSteerModelTest, RightBoundsInput) {
  model.right(10);
  assertLeftAndRightMotorsLastVelocity(0, 127);
}

TEST_F(SkidSteerModelTest, TankHalfPower) {
  model.tank(0.5, 0.5);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(6000);
}

TEST_F(SkidSteerModelTest, TankBoundsInput) {
  model.tank(10, 10);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(12000);
}

TEST_F(SkidSteerModelTest, TankThresholds) {
  model.tank(0.1, 0.1, 0.3);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(0);
}

TEST_F(SkidSteerModelTest, ArcadeHalfPower) {
  model.arcade(0.5, 0);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(6000);
}

TEST_F(SkidSteerModelTest, ArcadeHalfPowerTurn) {
  model.arcade(0, 0.5);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(6000, -6000);
}

TEST_F(SkidSteerModelTest, ArcadeBoundsInput) {
  model.arcade(10, 0);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(12000);
}

TEST_F(SkidSteerModelTest, ArcadeThresholds) {
  model.arcade(0.2, 0, 0.5);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(0);
}

TEST_F(SkidSteerModelTest, SetMaxVelocity) {
  model.setMaxVelocity(2);
  model.forward(0.5);
  assertAllMotorsLastVelocity(1);
}

TEST_F(SkidSteerModelTest, SetMaxVoltage) {
  model.setMaxVoltage(2);
  model.tank(0.5, 0.5);
  assertAllMotorsLastVoltage(1);
}

TEST_F(SkidSteerModelTest, SetGearsetTest) {
  model.setGearing(AbstractMotor::gearset::green);
  assertMotorsGearsetEquals(AbstractMotor::gearset::green, {*leftMotor, *rightMotor});
}

TEST_F(SkidSteerModelTest, SetBrakeModeTest) {
  model.setBrakeMode(AbstractMotor::brakeMode::hold);
  assertMotorsBrakeModeEquals(AbstractMotor::brakeMode::hold, {*leftMotor, *rightMotor});
}

TEST_F(SkidSteerModelTest, SetEncoderUnitsTest) {
  model.setEncoderUnits(AbstractMotor::encoderUnits::counts);
  assertMotorsEncoderUnitsEquals(AbstractMotor::encoderUnits::counts, {*leftMotor, *rightMotor});
}

TEST_F(SkidSteerModelTest, GetLeftSideMotor) {
  EXPECT_EQ(model.getLeftSideMotor().get(), leftMotor.get());
}

TEST_F(SkidSteerModelTest, GetRightSideMotor) {
  EXPECT_EQ(model.getRightSideMotor().get(), rightMotor.get());
}
