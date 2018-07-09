/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/api/chassisModelTests.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "test/crossPlatformTestRunner.hpp"
#include "test/tests/api/implMocks.hpp"

void testXDriveModel() {
  using namespace okapi;
  using namespace snowhouse;

  test_printf("Testing XDriveModel");

  auto topLeftMotor = std::make_shared<MockMotor>();
  auto topRightMotor = std::make_shared<MockMotor>();
  auto bottomRightMotor = std::make_shared<MockMotor>();
  auto bottomLeftMotor = std::make_shared<MockMotor>();
  XDriveModel model(topLeftMotor, topRightMotor, bottomRightMotor, bottomLeftMotor, 127);

  model.forward(0.5);
  test("XDriveModel forward should power all motors forward", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(63));
    AssertThat(topRightMotor->lastVelocity, Equals(63));
    AssertThat(bottomRightMotor->lastVelocity, Equals(63));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(63));
  });

  model.forward(10);
  test("XDriveModel forward should bound its input", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(127));
    AssertThat(topRightMotor->lastVelocity, Equals(127));
    AssertThat(bottomRightMotor->lastVelocity, Equals(127));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(127));
  });

  model.rotate(0.5);
  test("XDriveModel rotate should power left motors positive and right motors negative", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(63));
    AssertThat(topRightMotor->lastVelocity, Equals(-63));
    AssertThat(bottomRightMotor->lastVelocity, Equals(-63));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(63));
  });

  model.rotate(10);
  test("XDriveModel rotate should bound its input", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(127));
    AssertThat(topRightMotor->lastVelocity, Equals(-127));
    AssertThat(bottomRightMotor->lastVelocity, Equals(-127));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(127));
  });

  model.driveVector(0.25, 0.25);
  test("XDriveModel driveVector should make a swing turn", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(63));
    AssertThat(topRightMotor->lastVelocity, Equals(0));
    AssertThat(bottomRightMotor->lastVelocity, Equals(0));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(63));
  });

  model.driveVector(0.9, 0.25);
  test("XDriveModel driveVector should make a bounded swing turn", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(127));
    AssertThat(topRightMotor->lastVelocity, Equals(71));
    AssertThat(bottomRightMotor->lastVelocity, Equals(71));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(127));
  });

  topLeftMotor->lastVelocity = 100;
  topRightMotor->lastVelocity = 100;
  bottomRightMotor->lastVelocity = 100;
  bottomLeftMotor->lastVelocity = 100;
  model.stop();
  test("XDriveModel stop should set the motors to 0", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(0));
    AssertThat(topRightMotor->lastVelocity, Equals(0));
    AssertThat(bottomRightMotor->lastVelocity, Equals(0));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(0));
  });

  model.left(0.5);
  test("XDriveModel left should set the left motors", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(63));
    AssertThat(bottomLeftMotor->lastVelocity, Equals(63));
  });

  model.right(0.5);
  test("XDriveModel right should set the right motors", [&]() {
    AssertThat(topRightMotor->lastVelocity, Equals(63));
    AssertThat(bottomRightMotor->lastVelocity, Equals(63));
  });

  model.tank(0.5, 0.5);
  test("XDriveModel tank should set the left and right voltages", [&]() {
    AssertThat(topLeftMotor->lastVelocity, Equals(63));
    AssertThat(topRightMotor->lastVoltage, Equals(63));
    AssertThat(bottomRightMotor->lastVoltage, Equals(63));
    AssertThat(bottomLeftMotor->lastVoltage, Equals(63));
  });

  model.tank(10, 10);
  test("XDriveModel tank should bound its inputs", [&]() {
    AssertThat(topLeftMotor->lastVoltage, Equals(127));
    AssertThat(topRightMotor->lastVoltage, Equals(127));
    AssertThat(bottomRightMotor->lastVoltage, Equals(127));
    AssertThat(bottomLeftMotor->lastVoltage, Equals(127));
  });

  model.tank(0.2, 0.2, 0.5);
  test("XDriveModel tank should apply threshold", [&]() {
    AssertThat(topLeftMotor->lastVoltage, Equals(0));
    AssertThat(topRightMotor->lastVoltage, Equals(0));
    AssertThat(bottomRightMotor->lastVoltage, Equals(0));
    AssertThat(bottomLeftMotor->lastVoltage, Equals(0));
  });

  model.arcade(0.5, 0);
  test("XDriveModel arcade should move the robot forward", [&]() {
    AssertThat(topLeftMotor->lastVoltage, Equals(63));
    AssertThat(topRightMotor->lastVoltage, Equals(63));
    AssertThat(bottomRightMotor->lastVoltage, Equals(63));
    AssertThat(bottomLeftMotor->lastVoltage, Equals(63));
  });

  model.arcade(0, 0.5);
  test("XDriveModel arcade should turn the robot", [&]() {
    AssertThat(topLeftMotor->lastVoltage, Equals(63));
    AssertThat(topRightMotor->lastVoltage, Equals(-63));
    AssertThat(bottomRightMotor->lastVoltage, Equals(-63));
    AssertThat(bottomLeftMotor->lastVoltage, Equals(63));
  });

  model.arcade(10, 0);
  test("XDriveModel arcade should bound its inputs", [&]() {
    AssertThat(topLeftMotor->lastVoltage, Equals(127));
    AssertThat(topRightMotor->lastVoltage, Equals(127));
    AssertThat(bottomRightMotor->lastVoltage, Equals(127));
    AssertThat(bottomLeftMotor->lastVoltage, Equals(127));
  });

  model.arcade(0.2, 0, 0.5);
  test("XDriveModel arcade should apply threshold", [&]() {
    AssertThat(topLeftMotor->lastVoltage, Equals(0));
    AssertThat(topRightMotor->lastVoltage, Equals(0));
    AssertThat(bottomRightMotor->lastVoltage, Equals(0));
    AssertThat(bottomLeftMotor->lastVoltage, Equals(0));
  });
}

void testSkidSteerModel() {
  using namespace okapi;
  using namespace snowhouse;

  test_printf("Testing SkidSteerModel");

  auto leftMotor = std::make_shared<MockMotor>();
  auto rightMotor = std::make_shared<MockMotor>();
  SkidSteerModel model(leftMotor, rightMotor, 127);

  model.forward(0.5);
  test("SkidSteerModel forward should power all motors forward", [&]() {
    AssertThat(leftMotor->lastVelocity, Equals(63));
    AssertThat(rightMotor->lastVelocity, Equals(63));
  });

  model.forward(10);
  test("SkidSteerModel forward should bound its input", [&]() {
    AssertThat(leftMotor->lastVelocity, Equals(127));
    AssertThat(rightMotor->lastVelocity, Equals(127));
  });

  model.rotate(0.5);
  test("SkidSteerModel rotate should power left motors positive and right motors negative", [&]() {
    AssertThat(leftMotor->lastVelocity, Equals(63));
    AssertThat(rightMotor->lastVelocity, Equals(-63));
  });

  model.rotate(10);
  test("SkidSteerModel rotate should bound its input", [&]() {
    AssertThat(leftMotor->lastVelocity, Equals(127));
    AssertThat(rightMotor->lastVelocity, Equals(-127));
  });

  model.driveVector(0.25, 0.25);
  test("SkidSteerModel driveVector should make a swing turn", [&]() {
    AssertThat(leftMotor->lastVelocity, Equals(63));
    AssertThat(rightMotor->lastVelocity, Equals(0));
  });

  model.driveVector(0.9, 0.25);
  test("SkidSteerModel driveVector should make a bounded swing turn", [&]() {
    AssertThat(leftMotor->lastVelocity, Equals(127));
    AssertThat(rightMotor->lastVelocity, Equals(71));
  });

  leftMotor->lastVelocity = 100;
  rightMotor->lastVelocity = 100;
  model.stop();
  test("SkidSteerModel stop should set the motors to 0", [&]() {
    AssertThat(leftMotor->lastVelocity, Equals(0));
    AssertThat(rightMotor->lastVelocity, Equals(0));
  });

  model.left(0.5);
  test("SkidSteerModel left should set the left motors",
       [&]() { AssertThat(leftMotor->lastVelocity, Equals(63)); });

  model.right(0.5);
  test("SkidSteerModel right should set the right motors",
       [&]() { AssertThat(rightMotor->lastVelocity, Equals(63)); });

  model.tank(0.5, 0.5);
  test("SkidSteerModel tank should set the left and right voltages", [&]() {
    AssertThat(leftMotor->lastVoltage, Equals(63));
    AssertThat(rightMotor->lastVoltage, Equals(63));
  });

  model.tank(10, 10);
  test("SkidSteerModel tank should bound its inputs", [&]() {
    AssertThat(leftMotor->lastVoltage, Equals(127));
    AssertThat(rightMotor->lastVoltage, Equals(127));
  });

  model.tank(0.2, 0.2, 0.5);
  test("SkidSteerModel tank should apply threshold", [&]() {
    AssertThat(leftMotor->lastVoltage, Equals(0));
    AssertThat(rightMotor->lastVoltage, Equals(0));
  });

  model.arcade(0.5, 0);
  test("SkidSteerModel arcade should move the robot forward", [&]() {
    AssertThat(leftMotor->lastVoltage, Equals(63));
    AssertThat(rightMotor->lastVoltage, Equals(63));
  });

  model.arcade(0, 0.5);
  test("SkidSteerModel arcade should turn the robot", [&]() {
    AssertThat(leftMotor->lastVoltage, Equals(63));
    AssertThat(rightMotor->lastVoltage, Equals(-63));
  });

  model.arcade(10, 0);
  test("SkidSteerModel arcade should bound its inputs", [&]() {
    AssertThat(leftMotor->lastVoltage, Equals(127));
    AssertThat(rightMotor->lastVoltage, Equals(127));
  });

  model.arcade(0.2, 0, 0.5);
  test("SkidSteerModel arcade should apply threshold", [&]() {
    AssertThat(leftMotor->lastVoltage, Equals(0));
    AssertThat(rightMotor->lastVoltage, Equals(0));
  });
}

void testChassisModels() {
  testSkidSteerModel();
  testXDriveModel();
}
