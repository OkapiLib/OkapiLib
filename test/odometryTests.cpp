/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/api/odometryTests.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/odometry/odometry.hpp"
#include "okapi/api/odometry/threeEncoderOdometry.hpp"
#include "test/tests/api/implMocks.hpp"
#include <gtest/gtest.h>
#include <memory>

using namespace okapi;

class MockModel : public SkidSteerModel {
  public:
  MockModel() : SkidSteerModel(std::make_shared<MockMotor>(), std::make_shared<MockMotor>()) {
  }

  std::valarray<std::int32_t> getSensorVals() const override {
    return std::valarray<std::int32_t>{leftEnc, rightEnc};
  }

  void setSensorVals(std::int32_t left, std::int32_t right) {
    leftEnc = left;
    rightEnc = right;
  }

  std::int32_t leftEnc{0};
  std::int32_t rightEnc{0};
};

void assertOdomStateEquals(Odometry *odom, double x, double y, double theta) {
  EXPECT_NEAR(odom->getState().x.convert(meter), x, 1e-8);
  EXPECT_NEAR(odom->getState().y.convert(meter), y, 1e-8);
  EXPECT_NEAR(odom->getState().theta.convert(degree), theta, 1e-8);
}

class OdometryTest : public ::testing::Test {
  protected:
  void SetUp() override {
    model = new MockModel();
    odom = new Odometry(std::shared_ptr<MockModel>(model), ChassisScales({1, 1}),
                        std::make_unique<MockRate>());
  }

  void TearDown() override {
    delete odom;
  }

  MockModel *model;
  Odometry *odom;
};

TEST_F(OdometryTest, NoSensorMovementDoesNotAffectState) {
  assertOdomStateEquals(odom, 0, 0, 0);
  odom->step();
  assertOdomStateEquals(odom, 0, 0, 0);
}

TEST_F(OdometryTest, MoveForwardTest) {
  model->setSensorVals(10, 10);
  odom->step();
  assertOdomStateEquals(odom, 10, 0, 0);

  model->setSensorVals(20, 20);
  odom->step();
  assertOdomStateEquals(odom, 20, 0, 0);

  model->setSensorVals(10, 10);
  odom->step();
  assertOdomStateEquals(odom, 10, 0, 0);
}

TEST_F(OdometryTest, TurnInPlaceTest) {
  model->setSensorVals(10, -10);
  odom->step();
  assertOdomStateEquals(odom, 0, 0, 10);

  model->setSensorVals(0, 0);
  odom->step();
  assertOdomStateEquals(odom, 0, 0, 0);

  model->setSensorVals(-10, 10);
  odom->step();
  assertOdomStateEquals(odom, 0, 0, -10);
}

TEST_F(OdometryTest, TurnAndDriveTest) {
  model->setSensorVals(90, -90);
  odom->step();
  assertOdomStateEquals(odom, 0, 0, 90);

  model->setSensorVals(180, 0);
  odom->step();
  assertOdomStateEquals(odom, 0, 90, 90);
}

class MockThreeEncoderModel : public ThreeEncoderSkidSteerModel {
  public:
  MockThreeEncoderModel()
    : ThreeEncoderSkidSteerModel(std::make_shared<MockMotor>(), std::make_shared<MockMotor>(),
                                 std::make_shared<MockMotor>()->getEncoder()) {
  }

  std::valarray<std::int32_t> getSensorVals() const override {
    return std::valarray<std::int32_t>{leftEnc, rightEnc, middleEnc};
  }

  void setSensorVals(std::int32_t left, std::int32_t right, std::int32_t middle) {
    leftEnc = left;
    rightEnc = right;
    middleEnc = middle;
  }

  std::int32_t leftEnc{0};
  std::int32_t rightEnc{0};
  std::int32_t middleEnc{0};
};

class ThreeEncoderOdometryTest : public ::testing::Test {
  protected:
  void SetUp() override {
    model = new MockThreeEncoderModel();
    odom = new ThreeEncoderOdometry(std::shared_ptr<MockThreeEncoderModel>(model),
                                    ChassisScales({1, 1, 1}), createTimeUtil());
  }

  void TearDown() override {
    delete odom;
  }

  MockThreeEncoderModel *model;
  ThreeEncoderOdometry *odom;
};

TEST_F(ThreeEncoderOdometryTest, NoSensorMovementDoesNotAffectState) {
  assertOdomStateEquals(odom, 0, 0, 0);
  odom->step();
  assertOdomStateEquals(odom, 0, 0, 0);
}

TEST_F(ThreeEncoderOdometryTest, MoveForwardTest) {
  model->setSensorVals(10, 10, 0);
  odom->step();
  assertOdomStateEquals(odom, 10, 0, 0);

  model->setSensorVals(20, 20, 0);
  odom->step();
  assertOdomStateEquals(odom, 20, 0, 0);

  model->setSensorVals(10, 10, 0);
  odom->step();
  assertOdomStateEquals(odom, 10, 0, 0);
}

TEST_F(ThreeEncoderOdometryTest, TurnInPlaceTest) {
  model->setSensorVals(10, -10, 0);
  odom->step();
  assertOdomStateEquals(odom, 0, 0, 10);

  model->setSensorVals(0, 0, 0);
  odom->step();
  assertOdomStateEquals(odom, 0, 0, 0);

  model->setSensorVals(-10, 10, 0);
  odom->step();
  assertOdomStateEquals(odom, 0, 0, -10);
}

TEST_F(ThreeEncoderOdometryTest, TurnAndDriveTest) {
  model->setSensorVals(90, -90, 0);
  odom->step();
  assertOdomStateEquals(odom, 0, 0, 90);

  model->setSensorVals(180, 0, 0);
  odom->step();
  assertOdomStateEquals(odom, 0, 90, 90);
}

TEST_F(ThreeEncoderOdometryTest, StrafeTest) {
  model->setSensorVals(0, 0, 10);
  odom->step();
  assertOdomStateEquals(odom, 0, 10, 0);
}
