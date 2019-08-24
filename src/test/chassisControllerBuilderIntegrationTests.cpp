/*
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "test/tests/impl/chassisControllerBuilderIntegrationTests.hpp"
#include "test/testRunner.hpp"

using namespace okapi;
using namespace snowhouse;

static void testForwardUsesCorrectMaximumVelocityForAGearset() {
  printf("Testing forward uses the correct max vel for a gearset\n");
  resetHardware();

  const double power = 0.3;
  auto drive = ChassisControllerBuilder()
                 .withMotors(MOTOR_1_PORT, MOTOR_2_PORT)
                 .withGearset(MOTOR_GEARSET)
                 .build();

  drive->model().forward(power);
  pros::delay(500);

  auto sampleRPM = Motor(MOTOR_1_PORT).getActualVelocity();
  test(
    "Sample RPM should be approximately equal to the gearset rpm multiplied by the power",
    TEST_BODY(AssertThat, sampleRPM, EqualsWithDelta(toUnderlyingType(MOTOR_GEARSET) * power, 3)));

  drive->stop();
  resetHardware();
  pros::delay(500);
}

static void testMaxVelWorksOutOfOrder() {
  printf("Testing withMaxVelocity works out of order\n");
  resetHardware();

  const double maxVel = 20;
  auto drive = ChassisControllerBuilder()
                 .withMaxVelocity(maxVel)
                 .withMotors(MOTOR_1_PORT, MOTOR_2_PORT)
                 .withGearset(MOTOR_GEARSET)
                 .build();

  drive->model().forward(1);
  pros::delay(500);

  auto sampleRPM = Motor(MOTOR_1_PORT).getActualVelocity();
  test("Sample RPM should be approximately 20",
       TEST_BODY(AssertThat, sampleRPM, EqualsWithDelta(maxVel, 2)));

  drive->stop();
  resetHardware();
  pros::delay(500);
}

static void testSensorsWork() {
  printf("Testing sensor configuration works\n");
  resetHardware();

  const double power = 0.2;
  auto drive = ChassisControllerBuilder()
                 .withSensors({MOTOR_1_PORT}, {MOTOR_2_PORT})
                 .withMotors(MOTOR_1_PORT, MOTOR_2_PORT)
                 .withGearset(MOTOR_GEARSET)
                 .build();

  Motor leftMtr(MOTOR_1_PORT);
  Motor rightMtr(MOTOR_2_PORT);

  auto sensorValsBefore = drive->model().getSensorVals();
  auto leftBefore = leftMtr.getPosition();
  auto rightBefore = rightMtr.getPosition();

  drive->model().forward(power);
  pros::delay(250);

  drive->stop();
  auto sensorValsAfter = drive->model().getSensorVals();
  auto sensorDiff = sensorValsAfter - sensorValsBefore;
  auto leftAfter = leftMtr.getPosition();
  auto rightAfter = rightMtr.getPosition();
  auto leftDiff = leftAfter - leftBefore;
  auto rightDiff = rightAfter - rightBefore;

  test("Left sensor diff should be positive",
       TEST_BODY(AssertThat, sensorDiff[0], IsGreaterThan(0)));
  test("Left sensor diff should be equal to the diff from the motor",
       TEST_BODY(AssertThat, sensorDiff[0], EqualsWithDelta(leftDiff, 5)));
  test("Right sensor diff should be positive",
       TEST_BODY(AssertThat, sensorDiff[1], IsGreaterThan(0)));
  test("Right sensor diff should be equal to the diff from the motor",
       TEST_BODY(AssertThat, sensorDiff[1], EqualsWithDelta(rightDiff, 5)));

  drive->stop();
  resetHardware();
  pros::delay(500);
}

static void testMotorGearsetsAreNotOverwritten() {
  printf("Testing motor gearsets are not overwritten\n");
  resetHardware();

  // blue is not the correct gearset on purpose
  auto drive = ChassisControllerBuilder()
                 .withSensors({MOTOR_1_PORT}, {MOTOR_2_PORT})
                 .withMotors(MOTOR_1_PORT, MOTOR_2_PORT)
                 .withGearset(AbstractMotor::gearset::blue)
                 .build();

  test("Left motor gearset should equal blue",
       TEST_BODY(
         AssertThat, pros::c::motor_get_gearing(MOTOR_1_PORT), Equals(pros::E_MOTOR_GEARSET_06)));
  test("Right motor gearset should equal blue",
       TEST_BODY(
         AssertThat, pros::c::motor_get_gearing(MOTOR_2_PORT), Equals(pros::E_MOTOR_GEARSET_06)));

  resetHardware();
  pros::delay(500);
}

static void testOCCI() {
  printf("Testing OCCI\n");
  resetHardware();

  auto drive = ChassisControllerBuilder()
                 .withMotors(MOTOR_1_PORT, MOTOR_2_PORT)
                 .withGearset(MOTOR_GEARSET)
                 .withOdometry()
                 .withDimensions({{4_in, 10_in}, toUnderlyingType(MOTOR_GEARSET)})
                 .buildOdometry();

  const auto stateBefore = drive->getState();
  printf("x:%1.2f, y:%1.2f, theta:%1.2f\n",
         stateBefore.x.convert(inch),
         stateBefore.y.convert(inch),
         stateBefore.theta.convert(degree));
  drive->moveDistance(12_in);
  const auto stateAfter = drive->getState();
  printf("x:%1.2f, y:%1.2f, theta:%1.2f\n",
         stateAfter.x.convert(inch),
         stateAfter.y.convert(inch),
         stateAfter.theta.convert(degree));

  test("State diff x should be 12 inches",
       TEST_BODY(AssertThat, (stateAfter.x - stateBefore.x).convert(inch), EqualsWithDelta(12, 1)));
  test("State diff y should be 0 inches",
       TEST_BODY(AssertThat, (stateAfter.y - stateBefore.y).convert(inch), EqualsWithDelta(0, 1)));
  test("State diff theta should be 0 degrees",
       TEST_BODY(AssertThat,
                 (stateAfter.theta - stateBefore.theta).convert(degree),
                 EqualsWithDelta(0, 1)));

  drive->stop();
  resetHardware();
  pros::delay(500);
}

static void testOCCPID() {
  printf("Testing OCCPID\n");
  resetHardware();

  auto drive = ChassisControllerBuilder()
                 .withMotors(MOTOR_1_PORT, MOTOR_2_PORT)
                 .withGains({0.01, 0, 0}, {0, 0, 0})
                 .withGearset(MOTOR_GEARSET)
                 .withOdometry()
                 .withDimensions({{4_in, 10_in}, toUnderlyingType(MOTOR_GEARSET)})
                 .buildOdometry();

  const auto stateBefore = drive->getState();
  printf("x:%1.2f, y:%1.2f, theta:%1.2f\n",
         stateBefore.x.convert(inch),
         stateBefore.y.convert(inch),
         stateBefore.theta.convert(degree));
  drive->moveDistance(12_in);
  const auto stateAfter = drive->getState();
  printf("x:%1.2f, y:%1.2f, theta:%1.2f\n",
         stateAfter.x.convert(inch),
         stateAfter.y.convert(inch),
         stateAfter.theta.convert(degree));

  test("State diff x should be 12 inches",
       TEST_BODY(AssertThat, (stateAfter.x - stateBefore.x).convert(inch), EqualsWithDelta(12, 1)));
  test("State diff y should be 0 inches",
       TEST_BODY(AssertThat, (stateAfter.y - stateBefore.y).convert(inch), EqualsWithDelta(0, 1)));
  test("State diff theta should be 0 degrees",
       TEST_BODY(AssertThat,
                 (stateAfter.theta - stateBefore.theta).convert(degree),
                 EqualsWithDelta(0, 1)));

  drive->stop();
  resetHardware();
  pros::delay(500);
}

void runChassisControllerBuilderIntegrationTests() {
  test_printf("Testing ChassisControllerBuilder");
  testForwardUsesCorrectMaximumVelocityForAGearset();
  testMaxVelWorksOutOfOrder();
  testSensorsWork();
  testMotorGearsetsAreNotOverwritten();
  testOCCI();
  testOCCPID();
}
