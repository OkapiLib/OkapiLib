/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/chassisController.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class MockChassisModel : public ChassisModel {
  public:
  MockChassisModel() : ChassisModel(100) {
  }

  void forward(double ispeed) const override {
    lastForward = ispeed;
  }

  void driveVector(double iySpeed, double izRotation) const override {
    lastVectorY = iySpeed;
    lastVectorZ = izRotation;
  }

  void rotate(double ispeed) const override {
    lastRotate = ispeed;
  }

  void stop() override {
    stopWasCalled = true;
  }

  void tank(double ileftSpeed, double irightSpeed, double) const override {
    lastTankLeft = ileftSpeed;
    lastTankRight = irightSpeed;
  }

  void arcade(double iySpeed, double izRotation, double) const override {
    lastArcadeY = iySpeed;
    lastArcadeZ = izRotation;
  }

  void left(double ispeed) const override {
    lastLeft = ispeed;
  }

  void right(double ispeed) const override {
    lastRight = ispeed;
  }

  void resetSensors() const override {
    resetSensorsWasCalled = true;
  }

  void setBrakeMode(AbstractMotor::brakeMode mode) const override {
    lastBrakeMode = mode;
  }

  void setEncoderUnits(AbstractMotor::encoderUnits units) const override {
    lastEncoderUnits = units;
  }

  void setGearing(AbstractMotor::gearset gearset) const override {
    lastGearset = gearset;
  }

  std::valarray<int32_t> getSensorVals() const override {
    return {0, 0};
  }

  void setPosPID(double, double, double, double) const override {
  }

  void
  setPosPIDFull(double, double, double, double, double, double, double, double) const override {
  }

  void setVelPID(double, double, double, double) const override {
  }

  void
  setVelPIDFull(double, double, double, double, double, double, double, double) const override {
  }

  mutable double lastForward{0};
  mutable double lastVectorY{0};
  mutable double lastVectorZ{0};
  mutable double lastRotate{0};
  mutable bool stopWasCalled{false};
  mutable double lastTankLeft{0};
  mutable double lastTankRight{0};
  mutable double lastArcadeY{0};
  mutable double lastArcadeZ{0};
  mutable double lastLeft{0};
  mutable double lastRight{0};
  mutable bool resetSensorsWasCalled{false};
  mutable AbstractMotor::brakeMode lastBrakeMode{AbstractMotor::brakeMode::invalid};
  mutable AbstractMotor::encoderUnits lastEncoderUnits{AbstractMotor::encoderUnits::invalid};
  mutable AbstractMotor::gearset lastGearset{AbstractMotor::gearset::invalid};
};

class MockChassisController : public ChassisController {
  public:
  using ChassisController::ChassisController;

  void moveDistance(QLength) override {
  }

  void moveDistance(double) override {
  }

  void moveDistanceAsync(QLength) override {
  }

  void moveDistanceAsync(double) override {
  }

  void turnAngle(QAngle) override {
  }

  void turnAngle(double) override {
  }

  void turnAngleAsync(QAngle) override {
  }

  void turnAngleAsync(double) override {
  }

  void waitUntilSettled() override {
  }

  ChassisScales getChassisScales() const override {
    return ChassisScales({1, 1});
  }

  AbstractMotor::GearsetRatioPair getGearsetRatioPair() const override {
    return AbstractMotor::gearset::green;
  }
};

class ChassisControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    model = std::make_shared<MockChassisModel>();
    controller = new MockChassisController(model, 100);
  }

  void TearDown() override {
    delete controller;
  }

  std::shared_ptr<MockChassisModel> model;
  ChassisController *controller;
};

TEST_F(ChassisControllerTest, Forward) {
  controller->forward(0.5);
  EXPECT_EQ(model->lastForward, 0.5);
}

TEST_F(ChassisControllerTest, DriveVector) {
  controller->driveVector(0.4, 0.5);
  EXPECT_EQ(model->lastVectorY, 0.4);
  EXPECT_EQ(model->lastVectorZ, 0.5);
}

TEST_F(ChassisControllerTest, Rotate) {
  controller->rotate(0.5);
  EXPECT_EQ(model->lastRotate, 0.5);
}

TEST_F(ChassisControllerTest, Stop) {
  controller->stop();
  EXPECT_TRUE(model->stopWasCalled);
}

TEST_F(ChassisControllerTest, Tank) {
  controller->tank(0.4, 0.5);
  EXPECT_EQ(model->lastTankLeft, 0.4);
  EXPECT_EQ(model->lastTankRight, 0.5);
}

TEST_F(ChassisControllerTest, Arcade) {
  controller->arcade(0.4, 0.5);
  EXPECT_EQ(model->lastArcadeY, 0.4);
  EXPECT_EQ(model->lastArcadeZ, 0.5);
}

TEST_F(ChassisControllerTest, Left) {
  controller->left(0.5);
  EXPECT_EQ(model->lastLeft, 0.5);
}

TEST_F(ChassisControllerTest, Right) {
  controller->right(0.5);
  EXPECT_EQ(model->lastRight, 0.5);
}

TEST_F(ChassisControllerTest, ResetSensors) {
  controller->resetSensors();
  EXPECT_TRUE(model->resetSensorsWasCalled);
}

TEST_F(ChassisControllerTest, SetBrakeMode) {
  controller->setBrakeMode(AbstractMotor::brakeMode::hold);
  EXPECT_EQ(model->lastBrakeMode, AbstractMotor::brakeMode::hold);
}

TEST_F(ChassisControllerTest, SetEncoderUnits) {
  controller->setEncoderUnits(AbstractMotor::encoderUnits::counts);
  EXPECT_EQ(model->lastEncoderUnits, AbstractMotor::encoderUnits::counts);
}

TEST_F(ChassisControllerTest, SetGearing) {
  controller->setGearing(AbstractMotor::gearset::green);
  EXPECT_EQ(model->lastGearset, AbstractMotor::gearset::green);
}
