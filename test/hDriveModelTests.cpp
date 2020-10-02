/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/hDriveModel.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class HDriveModelTest : public ::testing::Test {
  public:
  HDriveModelTest()
    : model(leftMotor,
            rightMotor,
            middleMotor,
            leftSensor,
            rightSensor,
            middleSensor,
            127,
            v5MotorMaxVoltage) {
  }

  void assertAllMotorsLastVelocity(const std::int16_t expectedLastVelocity) const {
    EXPECT_EQ(leftMotor->lastVelocity, expectedLastVelocity);
    EXPECT_EQ(rightMotor->lastVelocity, expectedLastVelocity);
    EXPECT_EQ(middleMotor->lastVelocity, expectedLastVelocity);
  }

  void assertAllMotorsLastVoltage(const std::int16_t expectedLastVoltage) const {
    EXPECT_EQ(leftMotor->lastVoltage, expectedLastVoltage);
    EXPECT_EQ(rightMotor->lastVoltage, expectedLastVoltage);
    EXPECT_EQ(middleMotor->lastVoltage, expectedLastVoltage);
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
  std::shared_ptr<MockMotor> middleMotor = std::make_shared<MockMotor>();
  std::shared_ptr<MockContinuousRotarySensor> leftSensor =
    std::make_shared<MockContinuousRotarySensor>();
  std::shared_ptr<MockContinuousRotarySensor> rightSensor =
    std::make_shared<MockContinuousRotarySensor>();
  std::shared_ptr<MockContinuousRotarySensor> middleSensor =
    std::make_shared<MockContinuousRotarySensor>();
  HDriveModel model;
};

TEST_F(HDriveModelTest, ForwardHalfPower) {
  model.forward(0.5);
  assertLeftAndRightMotorsLastVelocity(63, 63);
}

TEST_F(HDriveModelTest, ForwardBoundsInput) {
  model.forward(10);
  assertLeftAndRightMotorsLastVelocity(127, 127);
}

TEST_F(HDriveModelTest, RotateHalfPower) {
  model.rotate(0.5);
  assertLeftAndRightMotorsLastVelocity(63, -63);
}

TEST_F(HDriveModelTest, RotateBoundsInput) {
  model.rotate(10);
  assertLeftAndRightMotorsLastVelocity(127, -127);
}

TEST_F(HDriveModelTest, DriveVectorHalfPower) {
  model.driveVector(0.25, 0.25);
  assertLeftAndRightMotorsLastVelocity(63, 0);
}

TEST_F(HDriveModelTest, DriveVectorBoundsInput) {
  model.driveVector(0.9, 0.25);
  assertLeftAndRightMotorsLastVelocity(127, 71);
}

TEST_F(HDriveModelTest, DriveVectorAndRotateAreEquivalent) {
  for (double i = -1; i < 1;) {
    model.driveVector(0, i);
    auto lastLeft = leftMotor->lastVelocity;
    auto lastRight = rightMotor->lastVelocity;
    model.rotate(i);
    EXPECT_FLOAT_EQ(leftMotor->lastVelocity, lastLeft);
    EXPECT_FLOAT_EQ(rightMotor->lastVelocity, lastRight);
    i += 0.001;
  }
}

TEST_F(HDriveModelTest, DriveVectorVoltageHalfPower) {
  model.driveVectorVoltage(0.25, 0.25);
  assertLeftAndRightMotorsLastVoltage(6000, 0);
}

TEST_F(HDriveModelTest, DriveVectorVoltageBoundsInput) {
  model.driveVectorVoltage(0.9, 0.25);
  assertLeftAndRightMotorsLastVoltage(12000, 6782);
}

TEST_F(HDriveModelTest, StopTest) {
  leftMotor->lastVelocity = 100;
  rightMotor->lastVelocity = 100;

  model.stop();

  assertAllMotorsLastVelocity(0);
}

TEST_F(HDriveModelTest, LeftHalfPower) {
  model.left(0.5);
  assertLeftAndRightMotorsLastVelocity(63, 0);
}

TEST_F(HDriveModelTest, LeftBoundsInput) {
  model.left(10);
  assertLeftAndRightMotorsLastVelocity(127, 0);
}

TEST_F(HDriveModelTest, RightHalfPower) {
  model.right(0.5);
  assertLeftAndRightMotorsLastVelocity(0, 63);
}

TEST_F(HDriveModelTest, RightBoundsInput) {
  model.right(10);
  assertLeftAndRightMotorsLastVelocity(0, 127);
}

TEST_F(HDriveModelTest, TankHalfPower) {
  model.tank(0.5, 0.5);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(6000, 6000);
}

TEST_F(HDriveModelTest, TankBoundsInput) {
  model.tank(10, 10);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(12000, 12000);
}

TEST_F(HDriveModelTest, TankThresholds) {
  model.tank(0.1, 0.1, 0.3);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(0, 0);
}

TEST_F(HDriveModelTest, ArcadeHalfPower) {
  model.arcade(0.5, 0);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(6000, 6000);
}

TEST_F(HDriveModelTest, ArcadeHalfPowerTurn) {
  model.arcade(0, 0.5);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(6000, -6000);
}

TEST_F(HDriveModelTest, ArcadeBoundsInput) {
  model.arcade(10, 0);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(12000, 12000);
}

TEST_F(HDriveModelTest, ArcadeThresholds) {
  model.arcade(0.2, 0, 0.5);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(0);
}

TEST_F(HDriveModelTest, ArcadeNegativeZero) {
  model.arcade(-0.0, -1.0);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(-12000, 12000);
}
TEST_F(HDriveModelTest, HArcadeHalfPowerForward) {
  model.hArcade(0, 0.5, 0);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(6000, 6000);
}

TEST_F(HDriveModelTest, HArcadeForwardBoundsInput) {
  model.hArcade(0, 10, 0);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(12000, 12000);
}

TEST_F(HDriveModelTest, HArcadeForwardThresholds) {
  model.hArcade(0, 0.4, 0, 0.5);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(0);
}

TEST_F(HDriveModelTest, HArcadeHalfPowerStrafe) {
  model.hArcade(0.5, 0, 0);

  assertAllMotorsLastVelocity(0);
  EXPECT_EQ(middleMotor->lastVoltage, 6000);
}

TEST_F(HDriveModelTest, HArcadeStrafeBoundsInput) {
  model.hArcade(10, 0, 0);

  assertAllMotorsLastVelocity(0);
  EXPECT_EQ(middleMotor->lastVoltage, 12000);
}

TEST_F(HDriveModelTest, HArcadeStrafeThresholds) {
  model.hArcade(0.4, 0, 0, 0.5);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(0);
}

TEST_F(HDriveModelTest, HArcadeTurnBoundsInput) {
  model.hArcade(0, 0, 10);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(12000, -12000);
}

TEST_F(HDriveModelTest, HArcadeTurnHalfPower) {
  model.hArcade(0, 0, 0.5);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(6000, -6000);
}

TEST_F(HDriveModelTest, HArcadeTurnThresholds) {
  model.hArcade(0, 0, 0.4, 0.5);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(0);
}

TEST_F(HDriveModelTest, HArcadeBoundsInputAllPoweredFull) {
  model.hArcade(10, 10, 10);

  assertAllMotorsLastVelocity(0);
  EXPECT_EQ(leftMotor->lastVoltage, 12000);
  EXPECT_EQ(rightMotor->lastVoltage, 0);
  EXPECT_EQ(middleMotor->lastVoltage, 12000);
}

TEST_F(HDriveModelTest, HArcadeBoundsInputAllNoTurn) {
  model.hArcade(10, 10, 0);

  assertAllMotorsLastVelocity(0);
  EXPECT_EQ(leftMotor->lastVoltage, 12000);
  EXPECT_EQ(rightMotor->lastVoltage, 12000);
  EXPECT_EQ(middleMotor->lastVoltage, 12000);
}

TEST_F(HDriveModelTest, HArcadeBoundsInputAllNoForward) {
  model.hArcade(10, 0, 10);

  assertAllMotorsLastVelocity(0);
  EXPECT_EQ(leftMotor->lastVoltage, 12000);
  EXPECT_EQ(rightMotor->lastVoltage, -12000);
  EXPECT_EQ(middleMotor->lastVoltage, 12000);
}

TEST_F(HDriveModelTest, SetMaxVelocity) {
  model.setMaxVelocity(2);
  model.forward(0.5);
  assertLeftAndRightMotorsLastVelocity(1, 1);
}

TEST_F(HDriveModelTest, SetMaxVoltage) {
  model.setMaxVoltage(2);
  model.tank(0.5, 0.5);
  assertLeftAndRightMotorsLastVoltage(1, 1);
}

TEST_F(HDriveModelTest, SetGearsetTest) {
  model.setGearing(AbstractMotor::gearset::green);
  assertMotorsGearsetEquals(AbstractMotor::gearset::green, {*leftMotor, *rightMotor, *middleMotor});
}

TEST_F(HDriveModelTest, SetBrakeModeTest) {
  model.setBrakeMode(AbstractMotor::brakeMode::hold);
  assertMotorsBrakeModeEquals(AbstractMotor::brakeMode::hold,
                              {*leftMotor, *rightMotor, *middleMotor});
}

TEST_F(HDriveModelTest, SetEncoderUnitsTest) {
  model.setEncoderUnits(AbstractMotor::encoderUnits::counts);
  assertMotorsEncoderUnitsEquals(AbstractMotor::encoderUnits::counts,
                                 {*leftMotor, *rightMotor, *middleMotor});
}

TEST_F(HDriveModelTest, GetLeftSideMotor) {
  EXPECT_EQ(model.getLeftSideMotor().get(), leftMotor.get());
}

TEST_F(HDriveModelTest, GetRightSideMotor) {
  EXPECT_EQ(model.getRightSideMotor().get(), rightMotor.get());
}

TEST_F(HDriveModelTest, GetMiddleMotor) {
  EXPECT_EQ(model.getMiddleMotor().get(), middleMotor.get());
}

TEST_F(HDriveModelTest, Reset) {
  leftSensor->value = 1;
  rightSensor->value = 1;
  middleSensor->value = 1;

  model.resetSensors();

  EXPECT_EQ(leftSensor->get(), 0);
  EXPECT_EQ(rightSensor->get(), 0);
  EXPECT_EQ(middleSensor->get(), 0);
}

TEST_F(HDriveModelTest, SetMaxVoltageGreaterThan12000) {
  model.setMaxVoltage(12001);
  EXPECT_EQ(model.getMaxVoltage(), 12000);
}

TEST_F(HDriveModelTest, SetMaxVoltageLessThan0) {
  model.setMaxVoltage(-1);
  EXPECT_EQ(model.getMaxVoltage(), 0);
}

TEST_F(HDriveModelTest, SetMaxVelocityLessThan0) {
  model.setMaxVelocity(-1);
  EXPECT_EQ(model.getMaxVelocity(), 0);
}
