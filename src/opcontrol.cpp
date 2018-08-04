/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "api.h"

#include "okapi/api.hpp"
#include "test/testRunner.hpp"
#include "test/tests/impl/allImplTests.hpp"

extern "C" {
#include "pathfinder.h"
}

void runHeadlessTests() {
  using namespace okapi;

  runHeadlessUtilTests();

  test_print_report();
}

void opcontrol() {
  using namespace okapi;
  pros::Task::delay(100);

  Logger::initialize(std::make_unique<Timer>(), "/ser/sout", Logger::LogLevel::info);

  {
    auto logger = Logger::instance();
    logger->info("Started pathfinder.");

    int POINT_LENGTH = 2;

    auto *points = (Waypoint *)malloc(sizeof(Waypoint) * POINT_LENGTH);

    Waypoint p1 = {0, 0, 0};
    Waypoint p2 = {0.9144, 0, d2r(45)};
    points[0] = p1;
    points[1] = p2;

    TrajectoryCandidate candidate;
    pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_FAST, 0.001, 1.0,
                       2.0, 10.0, &candidate);

    logger->info("Done preparing points.");

    int length = candidate.length;
    auto *trajectory = static_cast<Segment *>(malloc(length * sizeof(Segment)));

    pathfinder_generate(&candidate, trajectory);

    logger->info("Done generating.");

    auto *leftTrajectory = (Segment *)malloc(sizeof(Segment) * length);
    auto *rightTrajectory = (Segment *)malloc(sizeof(Segment) * length);

    double wheelbase_width = (10.5_in).convert(meter);

    pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheelbase_width);

    logger->info("Done modifying for tank.");

    auto drive =
      ChassisControllerFactory::create(-1, 2, AbstractMotor::gearset::red, {2.5_in, 10.5_in});

    Rate rate;
    for (int i = 0; i < length; i += 1) {
      logger->info("loop");
      drive.left(leftTrajectory[i].velocity);
      drive.right(rightTrajectory[i].velocity);
      rate.delayUntil(1_ms);
    }

    drive.stop();

    free(trajectory);
  }

  //  auto drive =
  //    ChassisControllerFactory::create(-1, 2, AbstractMotor::gearset::red, {2.5_in, 10.5_in});
  //  drive.moveDistanceAsync(2_in);
  //  drive.waitUntilSettled();

  //  runHeadlessTests();
  return;

  MotorGroup leftMotors({19_mtr, 20_mtr});
  MotorGroup rightMotors({13_rmtr, 14_rmtr});
  Motor armMotor = 15_mtr;
  armMotor.move(10);

  auto chassis =
    ChassisControllerFactory::create({19, 20}, {-14}, AbstractMotor::gearset::red, {4_in, 11.5_in});

  Controller controller;
  ControllerButton btn1(E_CONTROLLER_DIGITAL_A);
  ControllerButton btn2(E_CONTROLLER_DIGITAL_B);
  ControllerButton btn3(E_CONTROLLER_DIGITAL_Y);
  ControllerButton btn4(E_CONTROLLER_DIGITAL_X);

  while (true) {
    chassis.arcade(controller.getAnalog(E_CONTROLLER_ANALOG_LEFT_Y),
                   controller.getAnalog(E_CONTROLLER_ANALOG_LEFT_X));

    if (btn1.changedToPressed()) {
      printf("move distance\n");
      chassis.moveDistance(12_in);
    }

    if (btn2.changedToPressed()) {
      printf("turn angle\n");
      chassis.turnAngle(90_deg);
    }

    if (btn3.changedToPressed()) {
      printf("move arm\n");
      armMotor.moveRelative(-10, 127);
    }

    if (btn4.changedToPressed()) {
      printf("autonomous routine\n");
      for (int i = 0; i < 4; i++) {
        chassis.moveDistance(12_in);
        chassis.turnAngle(90_deg);
      }
    }

    pros::Task::delay(10);
  }
}
