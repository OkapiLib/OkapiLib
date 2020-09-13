/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/odometry/threeEncoderOdometry.hpp"
#include "okapi/api/odometry/twoEncoderOdometry.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>
#include <memory>

using namespace okapi;

class OdometryTest : public ::testing::Test {
  protected:
  void SetUp() override {
    model = new MockSkidSteerModel();
    odom = new TwoEncoderOdometry(createConstantTimeUtil(10_ms),
                                  std::shared_ptr<MockSkidSteerModel>(model),
                                  ChassisScales({{wheelDiam, wheelbaseWidth}, 360}));
  }

  void TearDown() override {
    delete odom;
  }

  QLength calculateDistanceTraveled(int ticks) {
    return (ticks / 360.0) * 1_pi * wheelDiam;
  }

  QLength wheelDiam = 4_in;
  QLength wheelbaseWidth = 10_in;
  MockSkidSteerModel *model;
  TwoEncoderOdometry *odom;
};

TEST_F(OdometryTest, NoSensorMovementDoesNotAffectState) {
  assertOdomStateEquals(odom, 0_m, 0_m, 0_deg);
  odom->step();
  assertOdomStateEquals(odom, 0_m, 0_m, 0_deg);
}

TEST_F(OdometryTest, MoveForwardTest) {
  model->setSensorVals(10, 10);
  odom->step();
  assertOdomStateEquals(odom, calculateDistanceTraveled(10), 0_m, 0_deg);

  model->setSensorVals(20, 20);
  odom->step();
  assertOdomStateEquals(odom, calculateDistanceTraveled(20), 0_m, 0_deg);

  model->setSensorVals(10, 10);
  odom->step();
  assertOdomStateEquals(odom, calculateDistanceTraveled(10), 0_m, 0_deg);
}

TEST_F(OdometryTest, TurnInPlaceTest) {
  model->setSensorVals(10, -10);
  odom->step();
  assertOdomStateEquals(odom, 0_m, 0_m, 4_deg);

  model->setSensorVals(0, 0);
  odom->step();
  assertOdomStateEquals(odom, 0_m, 0_m, 0_deg);

  model->setSensorVals(-10, 10);
  odom->step();
  assertOdomStateEquals(odom, 0_m, 0_m, -4_deg);
}

TEST_F(OdometryTest, TurnAndDriveTest) {
  model->setSensorVals(90, -90);
  odom->step();
  assertOdomStateEquals(odom, 0_m, 0_m, 36_deg);

  model->setSensorVals(180, 0);
  odom->step();
  assertOdomStateEquals(odom,
                        calculateDistanceTraveled(90) * std::cos((36_deg).convert(radian)),
                        calculateDistanceTraveled(90) * std::sin((36_deg).convert(radian)),
                        36_deg);
}

TEST_F(OdometryTest, GetStateInFrameTransformationTest) {
  OdomState inputState{1_in, 2_in, 1_deg};
  odom->setState(inputState);
  EXPECT_EQ(odom->getState(StateMode::FRAME_TRANSFORMATION), inputState);
}

TEST_F(OdometryTest, GetStateInCartesianTest) {
  OdomState inputState{1_in, 2_in, 1_deg};
  OdomState expected{inputState.y, inputState.x, 1_deg};
  odom->setState(inputState);
  EXPECT_EQ(odom->getState(StateMode::CARTESIAN), expected);
}

TEST_F(OdometryTest, SetStateInFrameTransformationTest) {
  OdomState inputState{1_in, 2_in, 1_deg};
  odom->setState(inputState, StateMode::FRAME_TRANSFORMATION);
  EXPECT_EQ(odom->getState(StateMode::FRAME_TRANSFORMATION), inputState);
}

TEST_F(OdometryTest, SetStateInCartesianTest) {
  OdomState expected{1_in, 2_in, 1_deg};
  OdomState inputState{expected.y, expected.x, 1_deg};
  odom->setState(inputState, StateMode::CARTESIAN);
  EXPECT_EQ(odom->getState(StateMode::FRAME_TRANSFORMATION), expected);
}

TEST_F(OdometryTest, TickDiffGreaterThanMax) {
  odom->setState(OdomState{1_in, 2_in, 45_deg});
  model->setSensorVals(1e+9, 1e+9);
  odom->step();
  assertOdomStateEquals(odom, 1_in, 2_in, 45_deg);
}

TEST_F(OdometryTest, TickDiffLessThanMax) {
  odom->setState(OdomState{1_in, 2_in, 45_deg});
  model->setSensorVals(-1e+9, -1e+9);
  odom->step();
  assertOdomStateEquals(odom, 1_in, 2_in, 45_deg);
}
