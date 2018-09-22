/**
 * @author Ryan Benasutti, WPI
 *
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
  XDriveModelTest() : model(topLeftMotor, topRightMotor, bottomRightMotor, bottomLeftMotor, 127) {
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

TEST_F(XDriveModelTest, DriveVectorHalfPower) {
  model.driveVector(0.25, 0.25);
  assertLeftAndRightMotorsLastVelocity(63, 0);
}

TEST_F(XDriveModelTest, DriveVectorBoundsInput) {
  model.driveVector(0.9, 0.25);
  assertLeftAndRightMotorsLastVelocity(127, 71);
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
