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

class OdomChassisControllerIntegratedTest : public ::testing::Test {
  protected:
  void SetUp() override {
    scales = new ChassisScales({2, 2}, imev5GreenTPR);
    leftMotor = new MockMotor();
    rightMotor = new MockMotor();

    leftController = new MockAsyncPosIntegratedController();
    rightController = new MockAsyncPosIntegratedController();

    model = new SkidSteerModel(
      std::unique_ptr<AbstractMotor>(leftMotor), std::unique_ptr<AbstractMotor>(rightMotor), 600);

    std::shared_ptr<SkidSteerModel> modelPtr = std::shared_ptr<SkidSteerModel>(model);

    odom = new Odometry(modelPtr, *scales, createTimeUtil().getRate());

    drive = new OdomChassisControllerIntegrated(
      createTimeUtil(),
      modelPtr,
      std::unique_ptr<Odometry>(odom),
      std::unique_ptr<AsyncPosIntegratedController>(leftController),
      std::unique_ptr<AsyncPosIntegratedController>(rightController),
      AbstractMotor::gearset::red,
      {{1, 1}, imev5GreenTPR});
  }

  void TearDown() override {
    delete scales;
    delete drive;
  }

  ChassisScales *scales;
  MockMotor *leftMotor;
  MockMotor *rightMotor;
  MockAsyncPosIntegratedController *leftController;
  MockAsyncPosIntegratedController *rightController;
  SkidSteerModel *model;
  Odometry *odom;
  OdomChassisControllerIntegrated *drive;
};

TEST_F(OdomChassisControllerIntegratedTest, MoveBelowThreshold) {
  assertMotorsHaveBeenStopped(leftMotor, rightMotor);

  drive->setMoveThreshold(5_m);
  drive->driveToPoint(4_m, 0_m);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 0);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), 0);
}

TEST_F(OdomChassisControllerIntegratedTest, MoveAboveThreshold) {
  assertMotorsHaveBeenStopped(leftMotor, rightMotor);

  drive->setMoveThreshold(5_m);
  drive->driveToPoint(6_m, 0_m);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 6);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), 6);
}

TEST_F(OdomChassisControllerIntegratedTest, TurnBelowThreshold) {
  assertMotorsHaveBeenStopped(leftMotor, rightMotor);

  drive->setTurnThreshold(5_deg);
  drive->turnToAngle(4_deg);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 0);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), 0);
}

TEST_F(OdomChassisControllerIntegratedTest, TurnAboveThreshold) {
  assertMotorsHaveBeenStopped(leftMotor, rightMotor);

  drive->setTurnThreshold(5_deg);
  drive->turnToAngle(6_deg);

  EXPECT_DOUBLE_EQ(leftController->getTarget(), 6);
  EXPECT_DOUBLE_EQ(rightController->getTarget(), -6);
}

TEST_F(OdomChassisControllerIntegratedTest, SetStateTest) {
  auto stateBefore = drive->getState();
  assertOdomStateEquals(0, 0, 0, stateBefore);

  OdomState newState{1_m, 2_m, 3_deg};
  drive->setState(newState);

  auto stateAfter = drive->getState();
  assertOdomStateEquals(stateAfter, newState);
}
