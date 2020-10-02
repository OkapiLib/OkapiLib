/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/defaultOdomChassisController.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include "okapi/api/odometry/twoEncoderOdometry.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class MockDefaultOdomChassisController : public DefaultOdomChassisController {
  public:
  using DefaultOdomChassisController::DefaultOdomChassisController;
  using DefaultOdomChassisController::odomTaskRunning;
};

class DefaultOdomChassisControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    odom = new TwoEncoderOdometry(
      createTimeUtil(), std::make_shared<MockReadOnlyChassisModel>(), scales);
    controller = std::make_shared<MockChassisController>();

    drive = new MockDefaultOdomChassisController(
      createTimeUtil(), std::shared_ptr<Odometry>(odom), controller);
    drive->odomTaskRunning = true;
  }

  void TearDown() override {
    delete drive;
  }

  ChassisScales scales{{4.125_in, 10_in}, imev5GreenTPR};
  SkidSteerModel *model;
  TwoEncoderOdometry *odom;
  MockDefaultOdomChassisController *drive;
  std::shared_ptr<MockChassisController> controller;
};

TEST_F(DefaultOdomChassisControllerTest, MoveBelowThreshold) {
  drive->setMoveThreshold(5_m);
  EXPECT_EQ(drive->getMoveThreshold(), 5_m);

  drive->driveToPoint({4_m, 0_m});
  EXPECT_EQ(controller->lastMoveDistanceTargetQLength, 0_m);
}

TEST_F(DefaultOdomChassisControllerTest, MoveAboveThreshold) {
  drive->setMoveThreshold(5_m);
  EXPECT_EQ(drive->getMoveThreshold(), 5_m);

  drive->driveToPoint({6_m, 0_m});
  EXPECT_EQ(controller->lastMoveDistanceTargetQLength, 6_m);
}

TEST_F(DefaultOdomChassisControllerTest, MoveAboveThresholdInCartesianModeWithNonzeroOdomState) {
  drive->setMoveThreshold(5_m);
  EXPECT_EQ(drive->getMoveThreshold(), 5_m);

  drive->setDefaultStateMode(StateMode::CARTESIAN);
  drive->setState({0_m, 0_m, 0_deg});

  drive->driveToPoint({2_m, 6_m});
  EXPECT_FLOAT_EQ(controller->lastMoveDistanceTargetQLength.convert(meter), sqrt(2 * 2 + 6 * 6));
  EXPECT_FLOAT_EQ(controller->lastTurnAngleTargetQAngle.convert(degree),
                  atan2(2, 6) * radianToDegree);
}

TEST_F(DefaultOdomChassisControllerTest, TurnToPointBelowThreshold) {
  drive->setTurnThreshold(5_deg);
  EXPECT_EQ(drive->getTurnThreshold(), 5_deg);

  drive->turnToPoint({1_m, 0.01_m});
  EXPECT_FLOAT_EQ(controller->lastTurnAngleTargetQAngle.convert(degree), 0);
}

TEST_F(DefaultOdomChassisControllerTest, TurnToPointBelowThresholdInCartesianMode) {
  drive->setTurnThreshold(5_deg);
  EXPECT_EQ(drive->getTurnThreshold(), 5_deg);

  drive->setDefaultStateMode(StateMode::CARTESIAN);
  drive->setState({0_m, 0_m, 0_deg});
  drive->turnToPoint({0.01_m, 1_m});
  EXPECT_FLOAT_EQ(controller->lastTurnAngleTargetQAngle.convert(degree), 0);
}

TEST_F(DefaultOdomChassisControllerTest, TurnToPointAboveThreshold) {
  drive->setTurnThreshold(5_deg);
  EXPECT_EQ(drive->getTurnThreshold(), 5_deg);

  drive->turnToPoint({1_m, 2_m});
  EXPECT_FLOAT_EQ(controller->lastTurnAngleTargetQAngle.convert(degree),
                  atan2(2, 1) * radianToDegree);
}

TEST_F(DefaultOdomChassisControllerTest, NonZeroHeadingTurnToPointAboveThreshold) {
  drive->setTurnThreshold(5_deg);
  EXPECT_EQ(drive->getTurnThreshold(), 5_deg);

  drive->setState({1_ft, 0_ft, 177_deg});

  drive->turnToPoint({0.7_ft, -0.4_ft});
  auto desiredAngle = atan2(-0.4, 0.7 - 1.0) * radian - 177_deg;
  EXPECT_FLOAT_EQ(controller->lastTurnAngleTargetQAngle.convert(degree),
                  OdomMath::constrainAngle180(desiredAngle).convert(degree));
}

TEST_F(DefaultOdomChassisControllerTest, TurnToPointAboveThresholdInCartesianMode) {
  drive->setTurnThreshold(5_deg);
  EXPECT_EQ(drive->getTurnThreshold(), 5_deg);

  drive->setDefaultStateMode(StateMode::CARTESIAN);
  drive->turnToPoint({2_m, 1_m});
  EXPECT_FLOAT_EQ(controller->lastTurnAngleTargetQAngle.convert(degree),
                  atan2(2, 1) * radianToDegree);
}

TEST_F(DefaultOdomChassisControllerTest,
       TurnToPointAboveThresholdInCartesianModeWithNonzeroOdomState) {
  drive->setTurnThreshold(5_deg);
  EXPECT_EQ(drive->getTurnThreshold(), 5_deg);

  drive->setDefaultStateMode(StateMode::CARTESIAN);
  drive->setState({5_m, 7_m, 10_deg});

  drive->turnToPoint({2_m, 1_m});
  EXPECT_FLOAT_EQ(controller->lastTurnAngleTargetQAngle.convert(degree),
                  atan2(-3, -6) * radianToDegree - 10);
}

TEST_F(DefaultOdomChassisControllerTest, TurnToPointAboveThresholdModeWithNonzeroOdomState) {
  drive->setTurnThreshold(5_deg);
  EXPECT_EQ(drive->getTurnThreshold(), 5_deg);

  drive->setState({7_m, 5_m, 10_deg});

  drive->turnToPoint({1_m, 2_m});
  EXPECT_FLOAT_EQ(controller->lastTurnAngleTargetQAngle.convert(degree),
                  atan2(-3, -6) * radianToDegree - 10);
}

TEST_F(DefaultOdomChassisControllerTest, TurnBelowThreshold) {
  drive->setTurnThreshold(5_deg);
  EXPECT_EQ(drive->getTurnThreshold(), 5_deg);

  drive->turnToAngle(4_deg);
  EXPECT_EQ(controller->lastTurnAngleTargetQAngle, 0_deg);
}

TEST_F(DefaultOdomChassisControllerTest, TurnAboveThreshold) {
  drive->setTurnThreshold(5_deg);
  EXPECT_EQ(drive->getTurnThreshold(), 5_deg);

  drive->turnToAngle(6_deg);
  EXPECT_EQ(controller->lastTurnAngleTargetQAngle, 6_deg);
}

TEST_F(DefaultOdomChassisControllerTest, SetStateTest) {
  auto stateBefore = drive->getState();
  assertOdomStateEquals(0, 0, 0, stateBefore);

  OdomState newState{1_m, 2_m, 3_deg};
  drive->setState(newState);

  auto stateAfter = drive->getState();
  EXPECT_EQ(stateAfter, newState);
}

TEST_F(DefaultOdomChassisControllerTest, SetStateInCartesianTest) {
  auto stateBefore = drive->getState();
  assertOdomStateEquals(0, 0, 0, stateBefore);

  drive->setDefaultStateMode(StateMode::CARTESIAN);
  OdomState newState{2_m, 1_m, 3_deg};
  drive->setState(newState);

  auto stateAfter = drive->getState();
  EXPECT_EQ(stateAfter, newState);
}
