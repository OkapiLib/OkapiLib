/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/controller/defaultOdomChassisController.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>

using namespace okapi;

class DefaultOdomChassisControllerTest : public ::testing::Test {
  protected:
  void SetUp() override {
    odom = new Odometry(createTimeUtil(), std::make_shared<MockReadOnlyChassisModel>(), scales);
    controller = std::make_shared<MockChassisController>();

    drive = new DefaultOdomChassisController(
      createTimeUtil(), std::unique_ptr<Odometry>(odom), controller);
  }

  void TearDown() override {
    delete drive;
  }

  ChassisScales scales{{4.125_in, 10_in}, imev5GreenTPR};
  SkidSteerModel *model;
  Odometry *odom;
  DefaultOdomChassisController *drive;
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

  OdomState newState{2_m, 1_m, 3_deg};
  drive->setState(newState, StateMode::CARTESIAN);

  auto stateAfter = drive->getState(StateMode::CARTESIAN);
  EXPECT_EQ(stateAfter, newState);
}
