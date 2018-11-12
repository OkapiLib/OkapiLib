#include "main.h"
#include "test/tests/impl/allImplTests.hpp"

using namespace okapi;
void opcontrol() {
  runAllImplTests();

  ChassisControllerFactory::create(1, 2);
  ChassisControllerFactory::create({1, 1}, {2, 2});
  ChassisControllerFactory::createPtr(1, 2);
  ChassisControllerFactory::createPtr({1, 1}, {2, 2});

  ChassisControllerFactory::create(
    1, 2, IterativePosPIDController::Gains(), IterativePosPIDController::Gains());
  ChassisControllerFactory::create(
    {1, 1}, {2, 2}, IterativePosPIDController::Gains(), IterativePosPIDController::Gains());
  ChassisControllerFactory::createPtr(
    1, 2, IterativePosPIDController::Gains(), IterativePosPIDController::Gains());
  ChassisControllerFactory::createPtr(
    {1, 1}, {2, 2}, IterativePosPIDController::Gains(), IterativePosPIDController::Gains());

  ChassisControllerFactory::create(1,
                                   2,
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains());
  ChassisControllerFactory::create({1, 1},
                                   {2, 2},
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains());
  ChassisControllerFactory::createPtr(1,
                                      2,
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains());
  ChassisControllerFactory::createPtr({1, 1},
                                      {2, 2},
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains());

  ChassisControllerFactory::create(1,
                                   2,
                                   ADIEncoder('A', 'B'),
                                   ADIEncoder('A', 'B'),
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains());
  ChassisControllerFactory::create({1, 1},
                                   {2, 2},
                                   ADIEncoder('A', 'B'),
                                   ADIEncoder('A', 'B'),
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains());
  ChassisControllerFactory::createPtr(1,
                                      2,
                                      ADIEncoder('A', 'B'),
                                      ADIEncoder('A', 'B'),
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains());
  ChassisControllerFactory::createPtr({1, 1},
                                      {2, 2},
                                      ADIEncoder('A', 'B'),
                                      ADIEncoder('A', 'B'),
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains());

  ChassisControllerFactory::create(1,
                                   2,
                                   ADIEncoder('A', 'B'),
                                   ADIEncoder('A', 'B'),
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains());
  ChassisControllerFactory::create({1, 1},
                                   {2, 2},
                                   ADIEncoder('A', 'B'),
                                   ADIEncoder('A', 'B'),
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains());
  ChassisControllerFactory::createPtr(1,
                                      2,
                                      ADIEncoder('A', 'B'),
                                      ADIEncoder('A', 'B'),
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains());
  ChassisControllerFactory::createPtr({1, 1},
                                      {2, 2},
                                      ADIEncoder('A', 'B'),
                                      ADIEncoder('A', 'B'),
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains());

  ChassisControllerFactory::create(1,
                                   2,
                                   IntegratedEncoder(pros::Motor(1)),
                                   IntegratedEncoder(pros::Motor(1)),
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains());
  ChassisControllerFactory::create({1, 1},
                                   {2, 2},
                                   IntegratedEncoder(pros::Motor(1)),
                                   IntegratedEncoder(pros::Motor(1)),
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains());
  ChassisControllerFactory::createPtr(1,
                                      2,
                                      IntegratedEncoder(pros::Motor(1)),
                                      IntegratedEncoder(pros::Motor(1)),
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains());
  ChassisControllerFactory::createPtr({1, 1},
                                      {2, 2},
                                      IntegratedEncoder(pros::Motor(1)),
                                      IntegratedEncoder(pros::Motor(1)),
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains());

  ChassisControllerFactory::create(1,
                                   2,
                                   IntegratedEncoder(pros::Motor(1)),
                                   IntegratedEncoder(pros::Motor(1)),
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains());
  ChassisControllerFactory::create({1, 1},
                                   {2, 2},
                                   IntegratedEncoder(pros::Motor(1)),
                                   IntegratedEncoder(pros::Motor(1)),
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains(),
                                   IterativePosPIDController::Gains());





  ChassisControllerFactory::createPtr(1,
                                      2,
                                      IntegratedEncoder(pros::Motor(1)),
                                      IntegratedEncoder(pros::Motor(1)),
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains());
  ChassisControllerFactory::createPtr({1, 1},
                                      {2, 2},
                                      IntegratedEncoder(pros::Motor(1)),
                                      IntegratedEncoder(pros::Motor(1)),
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains(),
                                      IterativePosPIDController::Gains());
}
