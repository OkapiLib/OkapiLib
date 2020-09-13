/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class XDriveModelTest : public ::testing::Test {
  public:
  XDriveModelTest()
    : model(topLeftMotor,
            topRightMotor,
            bottomRightMotor,
            bottomLeftMotor,
            leftSensor,
            rightSensor,
            127,
            v5MotorMaxVoltage) {
  }

  void assertAllMotorsLastVelocity(const std::int16_t expectedLastVelocity) const {
    EXPECT_EQ(topLeftMotor->lastVelocity, expectedLastVelocity);
    EXPECT_EQ(topRightMotor->lastVelocity, expectedLastVelocity);
    EXPECT_EQ(bottomRightMotor->lastVelocity, expectedLastVelocity);
    EXPECT_EQ(bottomLeftMotor->lastVelocity, expectedLastVelocity);
  }

  void assertAllMotorsLastVoltage(const std::int16_t expectedLastVoltage) const {
    EXPECT_EQ(topLeftMotor->lastVoltage, expectedLastVoltage);
    EXPECT_EQ(topRightMotor->lastVoltage, expectedLastVoltage);
    EXPECT_EQ(bottomRightMotor->lastVoltage, expectedLastVoltage);
    EXPECT_EQ(bottomLeftMotor->lastVoltage, expectedLastVoltage);
  }

  void assertLeftAndRightMotorsLastVelocity(const std::int16_t expectedLeftLastVelocity,
                                            const std::int16_t expectedRightLastVelocity) const {
    EXPECT_EQ(topLeftMotor->lastVelocity, expectedLeftLastVelocity);
    EXPECT_EQ(topRightMotor->lastVelocity, expectedRightLastVelocity);
    EXPECT_EQ(bottomRightMotor->lastVelocity, expectedRightLastVelocity);
    EXPECT_EQ(bottomLeftMotor->lastVelocity, expectedLeftLastVelocity);
  }

  void assertTLBRAndTRBLMotorsLastVelocity(const std::int16_t expectedTopLeftLastVelocity,
                                           const std::int16_t expectedTopRightLastVelocity) const {
    EXPECT_EQ(topLeftMotor->lastVelocity, expectedTopLeftLastVelocity);
    EXPECT_EQ(topRightMotor->lastVelocity, expectedTopRightLastVelocity);
    EXPECT_EQ(bottomRightMotor->lastVelocity, expectedTopLeftLastVelocity);
    EXPECT_EQ(bottomLeftMotor->lastVelocity, expectedTopRightLastVelocity);
  }

  void assertLeftAndRightMotorsLastVoltage(const std::int16_t expectedLeftLastVoltage,
                                           const std::int16_t expectedRightLastVoltage) const {
    EXPECT_EQ(topLeftMotor->lastVoltage, expectedLeftLastVoltage);
    EXPECT_EQ(topRightMotor->lastVoltage, expectedRightLastVoltage);
    EXPECT_EQ(bottomRightMotor->lastVoltage, expectedRightLastVoltage);
    EXPECT_EQ(bottomLeftMotor->lastVoltage, expectedLeftLastVoltage);
  }

  std::shared_ptr<MockMotor> topLeftMotor = std::make_shared<MockMotor>();
  std::shared_ptr<MockMotor> topRightMotor = std::make_shared<MockMotor>();
  std::shared_ptr<MockMotor> bottomRightMotor = std::make_shared<MockMotor>();
  std::shared_ptr<MockMotor> bottomLeftMotor = std::make_shared<MockMotor>();
  std::shared_ptr<MockContinuousRotarySensor> leftSensor =
    std::make_shared<MockContinuousRotarySensor>();
  std::shared_ptr<MockContinuousRotarySensor> rightSensor =
    std::make_shared<MockContinuousRotarySensor>();
  XDriveModel model;
};

TEST_F(XDriveModelTest, ForwardHalfPower) {
  model.forward(0.5);
  assertAllMotorsLastVelocity(63);
}

TEST_F(XDriveModelTest, ForwardBoundsInput) {
  model.forward(10);
  assertAllMotorsLastVelocity(127);
}

TEST_F(XDriveModelTest, RotateHalfPower) {
  model.rotate(0.5);
  assertLeftAndRightMotorsLastVelocity(63, -63);
}

TEST_F(XDriveModelTest, RotateBoundsInput) {
  model.rotate(10);
  assertLeftAndRightMotorsLastVelocity(127, -127);
}

TEST_F(XDriveModelTest, StrafeHalfPower) {
  model.strafe(0.5);
  assertTLBRAndTRBLMotorsLastVelocity(-63, 63);
}

TEST_F(XDriveModelTest, StrafeBoundsInput) {
  model.strafe(10);
  assertTLBRAndTRBLMotorsLastVelocity(-127, 127);
}

TEST_F(XDriveModelTest, DriveVectorHalfPower) {
  model.driveVector(0.25, 0.25);
  assertLeftAndRightMotorsLastVelocity(63, 0);
}

TEST_F(XDriveModelTest, DriveVectorBoundsInput) {
  model.driveVector(0.9, 0.25);
  assertLeftAndRightMotorsLastVelocity(127, 71);
}

TEST_F(XDriveModelTest, DriveVectorAndRotateAreEquivalent) {
  for (double i = -1; i < 1;) {
    model.driveVector(0, i);
    auto lastTopLeft = topLeftMotor->lastVelocity;
    auto lastTopRight = topRightMotor->lastVelocity;
    auto lastBottomRight = bottomRightMotor->lastVelocity;
    auto lastBottomLeft = bottomLeftMotor->lastVelocity;
    model.rotate(i);
    EXPECT_FLOAT_EQ(topLeftMotor->lastVelocity, lastTopLeft);
    EXPECT_FLOAT_EQ(topRightMotor->lastVelocity, lastTopRight);
    EXPECT_FLOAT_EQ(bottomRightMotor->lastVelocity, lastBottomRight);
    EXPECT_FLOAT_EQ(bottomLeftMotor->lastVelocity, lastBottomLeft);
    i += 0.001;
  }
}

TEST_F(XDriveModelTest, DriveVectorVoltageHalfPower) {
  model.driveVectorVoltage(0.25, 0.25);
  assertLeftAndRightMotorsLastVoltage(6000, 0);
}

TEST_F(XDriveModelTest, DriveVectorVoltageBoundsInput) {
  model.driveVectorVoltage(0.9, 0.25);
  assertLeftAndRightMotorsLastVoltage(12000, 6782);
}

TEST_F(XDriveModelTest, StrafeVectorHalfPower) {
  model.strafeVector(0.25, 0.25);
  EXPECT_FLOAT_EQ(topLeftMotor->lastVelocity, 0);
  EXPECT_FLOAT_EQ(topRightMotor->lastVelocity, 0);
  EXPECT_FLOAT_EQ(bottomRightMotor->lastVelocity, -63);
  EXPECT_FLOAT_EQ(bottomLeftMotor->lastVelocity, 63);
}

TEST_F(XDriveModelTest, StrafeVectorBoundsInput) {
  model.strafeVector(0.9, 0.25);
  EXPECT_FLOAT_EQ(topLeftMotor->lastVelocity, -71);
  EXPECT_FLOAT_EQ(topRightMotor->lastVelocity, 71);
  EXPECT_FLOAT_EQ(bottomRightMotor->lastVelocity, -127);
  EXPECT_FLOAT_EQ(bottomLeftMotor->lastVelocity, 127);
}

TEST_F(XDriveModelTest, StrafeVectorAndRotateAreEquivalent) {
  for (double i = -1; i < 1;) {
    model.strafeVector(0, i);
    auto lastTopLeft = topLeftMotor->lastVelocity;
    auto lastTopRight = topRightMotor->lastVelocity;
    auto lastBottomRight = bottomRightMotor->lastVelocity;
    auto lastBottomLeft = bottomLeftMotor->lastVelocity;
    model.rotate(i);
    EXPECT_FLOAT_EQ(topLeftMotor->lastVelocity, lastTopLeft);
    EXPECT_FLOAT_EQ(topRightMotor->lastVelocity, lastTopRight);
    EXPECT_FLOAT_EQ(bottomRightMotor->lastVelocity, lastBottomRight);
    EXPECT_FLOAT_EQ(bottomLeftMotor->lastVelocity, lastBottomLeft);
    i += 0.001;
  }
}

TEST_F(XDriveModelTest, StrafeVectorAndStrafeAreEquivalent) {
  for (double i = -1; i < 1;) {
    model.strafeVector(i, 0);
    auto lastTopLeft = topLeftMotor->lastVelocity;
    auto lastTopRight = topRightMotor->lastVelocity;
    auto lastBottomRight = bottomRightMotor->lastVelocity;
    auto lastBottomLeft = bottomLeftMotor->lastVelocity;
    model.strafe(i);
    EXPECT_FLOAT_EQ(topLeftMotor->lastVelocity, lastTopLeft);
    EXPECT_FLOAT_EQ(topRightMotor->lastVelocity, lastTopRight);
    EXPECT_FLOAT_EQ(bottomRightMotor->lastVelocity, lastBottomRight);
    EXPECT_FLOAT_EQ(bottomLeftMotor->lastVelocity, lastBottomLeft);
    i += 0.001;
  }
}

TEST_F(XDriveModelTest, StopTest) {
  topLeftMotor->lastVelocity = 100;
  topRightMotor->lastVelocity = 100;
  bottomRightMotor->lastVelocity = 100;
  bottomLeftMotor->lastVelocity = 100;

  model.stop();

  assertAllMotorsLastVelocity(0);
}

TEST_F(XDriveModelTest, LeftHalfPower) {
  model.left(0.5);
  assertLeftAndRightMotorsLastVelocity(63, 0);
}

TEST_F(XDriveModelTest, LeftBoundsInput) {
  model.left(10);
  assertLeftAndRightMotorsLastVelocity(127, 0);
}

TEST_F(XDriveModelTest, RightHalfPower) {
  model.right(0.5);
  assertLeftAndRightMotorsLastVelocity(0, 63);
}

TEST_F(XDriveModelTest, RightBoundsInput) {
  model.right(10);
  assertLeftAndRightMotorsLastVelocity(0, 127);
}

TEST_F(XDriveModelTest, TankHalfPower) {
  model.tank(0.5, 0.5);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(6000);
}

TEST_F(XDriveModelTest, TankBoundsInput) {
  model.tank(10, 10);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(12000);
}

TEST_F(XDriveModelTest, TankThresholds) {
  model.tank(0.1, 0.1, 0.3);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(0);
}

TEST_F(XDriveModelTest, ArcadeHalfPower) {
  model.arcade(0.5, 0);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(6000);
}

TEST_F(XDriveModelTest, ArcadeHalfPowerTurn) {
  model.arcade(0, 0.5);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(6000, -6000);
}

TEST_F(XDriveModelTest, ArcadeBoundsInput) {
  model.arcade(10, 0);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(12000);
}

TEST_F(XDriveModelTest, ArcadeThresholds) {
  model.arcade(0.2, 0, 0.5);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(0);
}

TEST_F(XDriveModelTest, ArcadeNegativeZero) {
  model.arcade(-0.0, -1.0);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(-12000, 12000);
}

TEST_F(XDriveModelTest, XArcadeHalfPowerForward) {
  model.xArcade(0, 0.5, 0);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(6000);
}

TEST_F(XDriveModelTest, XArcadeForwardBoundsInput) {
  model.xArcade(0, 10, 0);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(12000);
}

TEST_F(XDriveModelTest, XArcadeForwardThresholds) {
  model.xArcade(0, 0.4, 0, 0.5);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(0);
}

TEST_F(XDriveModelTest, XArcadeHalfPowerStrafe) {
  model.xArcade(0.5, 0, 0);

  assertAllMotorsLastVelocity(0);
  EXPECT_EQ(topLeftMotor->lastVoltage, 6000);
  EXPECT_EQ(topRightMotor->lastVoltage, -6000);
  EXPECT_EQ(bottomRightMotor->lastVoltage, 6000);
  EXPECT_EQ(bottomLeftMotor->lastVoltage, -6000);
}

TEST_F(XDriveModelTest, XArcadeStrafeBoundsInput) {
  model.xArcade(10, 0, 0);

  assertAllMotorsLastVelocity(0);
  EXPECT_EQ(topLeftMotor->lastVoltage, 12000);
  EXPECT_EQ(topRightMotor->lastVoltage, -12000);
  EXPECT_EQ(bottomRightMotor->lastVoltage, 12000);
  EXPECT_EQ(bottomLeftMotor->lastVoltage, -12000);
}

TEST_F(XDriveModelTest, XArcadeStrafeThresholds) {
  model.xArcade(0.4, 0, 0, 0.5);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(0);
}

TEST_F(XDriveModelTest, XArcadeTurnBoundsInput) {
  model.xArcade(0, 0, 10);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(12000, -12000);
}

TEST_F(XDriveModelTest, XArcadeTurnHalfPower) {
  model.xArcade(0, 0, 0.5);

  assertAllMotorsLastVelocity(0);
  assertLeftAndRightMotorsLastVoltage(6000, -6000);
}

TEST_F(XDriveModelTest, XArcadeTurnThresholds) {
  model.xArcade(0, 0, 0.4, 0.5);

  assertAllMotorsLastVelocity(0);
  assertAllMotorsLastVoltage(0);
}

TEST_F(XDriveModelTest, XArcadeBoundsInputAllPoweredFull) {
  model.xArcade(10, 10, 10);

  assertAllMotorsLastVelocity(0);
  EXPECT_EQ(topLeftMotor->lastVoltage, 12000);
  EXPECT_EQ(topRightMotor->lastVoltage, -12000);
  EXPECT_EQ(bottomRightMotor->lastVoltage, 12000);
  EXPECT_EQ(bottomLeftMotor->lastVoltage, 12000);
}

TEST_F(XDriveModelTest, XArcadeBoundsInputAllNoTurn) {
  model.xArcade(10, 10, 0);

  assertAllMotorsLastVelocity(0);
  EXPECT_EQ(topLeftMotor->lastVoltage, 12000);
  EXPECT_EQ(topRightMotor->lastVoltage, 0);
  EXPECT_EQ(bottomRightMotor->lastVoltage, 12000);
  EXPECT_EQ(bottomLeftMotor->lastVoltage, 0);
}

TEST_F(XDriveModelTest, XArcadeBoundsInputAllNoForward) {
  model.xArcade(10, 0, 10);

  assertAllMotorsLastVelocity(0);
  EXPECT_EQ(topLeftMotor->lastVoltage, 12000);
  EXPECT_EQ(topRightMotor->lastVoltage, -12000);
  EXPECT_EQ(bottomRightMotor->lastVoltage, 0);
  EXPECT_EQ(bottomLeftMotor->lastVoltage, 0);
}

TEST_F(XDriveModelTest, SetMaxVelocity) {
  model.setMaxVelocity(2);
  model.forward(0.5);
  assertAllMotorsLastVelocity(1);
}

TEST_F(XDriveModelTest, SetMaxVoltage) {
  model.setMaxVoltage(2);
  model.tank(0.5, 0.5);
  assertAllMotorsLastVoltage(1);
}

TEST_F(XDriveModelTest, SetGearsetTest) {
  model.setGearing(AbstractMotor::gearset::green);
  assertMotorsGearsetEquals(AbstractMotor::gearset::green,
                            {*topLeftMotor, *topRightMotor, *bottomRightMotor, *bottomLeftMotor});
}

TEST_F(XDriveModelTest, SetBrakeModeTest) {
  model.setBrakeMode(AbstractMotor::brakeMode::hold);
  assertMotorsBrakeModeEquals(AbstractMotor::brakeMode::hold,
                              {*topLeftMotor, *topRightMotor, *bottomRightMotor, *bottomLeftMotor});
}

TEST_F(XDriveModelTest, SetEncoderUnitsTest) {
  model.setEncoderUnits(AbstractMotor::encoderUnits::counts);
  assertMotorsEncoderUnitsEquals(
    AbstractMotor::encoderUnits::counts,
    {*topLeftMotor, *topRightMotor, *bottomRightMotor, *bottomLeftMotor});
}

TEST_F(XDriveModelTest, GetTopLeftMotor) {
  EXPECT_EQ(model.getTopLeftMotor().get(), topLeftMotor.get());
}

TEST_F(XDriveModelTest, GetTopRightMotor) {
  EXPECT_EQ(model.getTopRightMotor().get(), topRightMotor.get());
}

TEST_F(XDriveModelTest, GetBottomLeftMotor) {
  EXPECT_EQ(model.getBottomLeftMotor().get(), bottomLeftMotor.get());
}

TEST_F(XDriveModelTest, GetBottomRightMotor) {
  EXPECT_EQ(model.getBottomRightMotor().get(), bottomRightMotor.get());
}

TEST_F(XDriveModelTest, Reset) {
  leftSensor->value = 1;
  rightSensor->value = 1;

  model.resetSensors();

  EXPECT_EQ(leftSensor->get(), 0);
  EXPECT_EQ(rightSensor->get(), 0);
}

TEST_F(XDriveModelTest, SetMaxVoltageGreaterThan12000) {
  model.setMaxVoltage(12001);
  EXPECT_EQ(model.getMaxVoltage(), 12000);
}

TEST_F(XDriveModelTest, SetMaxVoltageLessThan0) {
  model.setMaxVoltage(-1);
  EXPECT_EQ(model.getMaxVoltage(), 0);
}

TEST_F(XDriveModelTest, SetMaxVelocityLessThan0) {
  model.setMaxVelocity(-1);
  EXPECT_EQ(model.getMaxVelocity(), 0);
}
