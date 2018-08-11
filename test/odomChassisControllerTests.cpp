/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/odomChassisControllerIntegrated.hpp"
#include "okapi/api/chassis/controller/odomChassisControllerPid.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

void assertOdomStateEquals(double x, double y, double theta, const OdomState &actual) {
  EXPECT_DOUBLE_EQ(actual.x, x);
  EXPECT_DOUBLE_EQ(actual.y, y);
  EXPECT_DOUBLE_EQ(actual.theta, theta);
}

void assertOdomStateEquals(const OdomState &expected, const OdomState &actual) {
  assertOdomStateEquals(expected.x, expected.y, expected.theta, actual);
}

class OdomChassisControllerIntegratedTest : public ::testing::Test {
  protected:
  void SetUp() override {
    scales = new ChassisScales({2, 2});
    leftMotor = new MockMotor();
    rightMotor = new MockMotor();

    leftController = new MockAsyncController();
    rightController = new MockAsyncController();

    model = new SkidSteerModel(std::unique_ptr<AbstractMotor>(leftMotor),
                               std::unique_ptr<AbstractMotor>(rightMotor));

    std::shared_ptr<SkidSteerModel> modelPtr = std::shared_ptr<SkidSteerModel>(model);

    odom = new Odometry(modelPtr, *scales, createTimeUtil().getRate());

    drive = new OdomChassisControllerIntegrated(
      createTimeUtil(), modelPtr, std::unique_ptr<Odometry>(odom),
      std::unique_ptr<AsyncPosIntegratedController>(leftController),
      std::unique_ptr<AsyncPosIntegratedController>(rightController));
  }

  void TearDown() override {
    delete scales;
    delete drive;
  }

  ChassisScales *scales;
  MockMotor *leftMotor;
  MockMotor *rightMotor;
  MockAsyncController *leftController;
  MockAsyncController *rightController;
  SkidSteerModel *model;
  Odometry *odom;
  OdomChassisControllerIntegrated *drive;
};

TEST_F(OdomChassisControllerIntegratedTest, MoveBelowThreshold) {
  assertMotorsHaveBeenStopped(leftMotor, rightMotor);

  drive->setMoveThreshold(5);
  drive->driveToPoint(4, 0);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 0);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), 0);
}

TEST_F(OdomChassisControllerIntegratedTest, MoveAboveThreshold) {
  assertMotorsHaveBeenStopped(leftMotor, rightMotor);

  drive->setMoveThreshold(5);
  drive->driveToPoint(6, 0);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 6);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), 6);
}

TEST_F(OdomChassisControllerIntegratedTest, SetStateTest) {
  auto stateBefore = drive->getState();
  assertOdomStateEquals(0, 0, 0, stateBefore);

  OdomState newState(1, 2, 3);
  drive->setState(newState);

  auto stateAfter = drive->getState();
  assertOdomStateEquals(stateAfter, newState);
}

class OdomChassisControllerPIDTest : public ::testing::Test {
  protected:
  void SetUp() override {
    scales = new ChassisScales({2, 2});
    leftMotor = new MockMotor();
    rightMotor = new MockMotor();

    distanceController = new MockIterativeController();
    angleController = new MockIterativeController();
    turnController = new MockIterativeController();

    model = new SkidSteerModel(std::unique_ptr<AbstractMotor>(leftMotor),
                               std::unique_ptr<AbstractMotor>(rightMotor));

    std::shared_ptr<SkidSteerModel> modelPtr = std::shared_ptr<SkidSteerModel>(model);

    odom = new Odometry(modelPtr, *scales, createTimeUtil().getRate());

    drive =
      new OdomChassisControllerPID(createTimeUtil(), modelPtr, std::unique_ptr<Odometry>(odom),
                                   std::unique_ptr<IterativePosPIDController>(distanceController),
                                   std::unique_ptr<IterativePosPIDController>(angleController),
                                   std::unique_ptr<IterativePosPIDController>(turnController));
  }

  void TearDown() override {
    delete scales;
    delete drive;
  }

  ChassisScales *scales;
  MockMotor *leftMotor;
  MockMotor *rightMotor;
  MockIterativeController *distanceController;
  MockIterativeController *angleController;
  MockIterativeController *turnController;
  SkidSteerModel *model;
  Odometry *odom;
  OdomChassisControllerPID *drive;
};

TEST_F(OdomChassisControllerPIDTest, MoveBelowThreshold) {
  assertMotorsHaveBeenStopped(leftMotor, rightMotor);

  drive->setMoveThreshold(5);
  drive->driveToPoint(4, 0);

  EXPECT_DOUBLE_EQ(distanceController->getTarget(), 0);
  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
  EXPECT_DOUBLE_EQ(turnController->getTarget(), 0);
}

TEST_F(OdomChassisControllerPIDTest, MoveAboveThreshold) {
  assertMotorsHaveBeenStopped(leftMotor, rightMotor);

  drive->setMoveThreshold(5);
  drive->driveToPoint(6, 0);

  EXPECT_DOUBLE_EQ(distanceController->getTarget(), 6);
  EXPECT_DOUBLE_EQ(angleController->getTarget(), 0);
  EXPECT_DOUBLE_EQ(turnController->getTarget(), 0);
}

TEST_F(OdomChassisControllerPIDTest, SetStateTest) {
  auto stateBefore = drive->getState();
  assertOdomStateEquals(0, 0, 0, stateBefore);

  OdomState newState(1, 2, 3);
  drive->setState(newState);

  auto stateAfter = drive->getState();
  assertOdomStateEquals(stateAfter, newState);
}
