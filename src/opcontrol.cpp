#include "main.h"
#include "test/tests/impl/allImplTests.hpp"

using namespace okapi;
void opcontrol() {
  runAllImplTests();

  ChassisControllerIntegrated f1 = ChassisControllerFactory::create(1, 2);
  ChassisControllerIntegrated f2 = ChassisControllerFactory::create({1, 1}, {2, 2});
  std::shared_ptr<ChassisControllerIntegrated> f3 = ChassisControllerFactory::createPtr(1, 2);
  std::shared_ptr<ChassisControllerIntegrated> f4 =
    ChassisControllerFactory::createPtr({1, 1}, {2, 2});

  ChassisControllerPID f5 = ChassisControllerFactory::create(
    1, 2, IterativePosPIDController::Gains(), IterativePosPIDController::Gains());
  ChassisControllerPID f6 = ChassisControllerFactory::create(
    {1, 1}, {2, 2}, IterativePosPIDController::Gains(), IterativePosPIDController::Gains());
  std::shared_ptr<ChassisControllerPID> f7 = ChassisControllerFactory::createPtr(
    1, 2, IterativePosPIDController::Gains(), IterativePosPIDController::Gains());
  std::shared_ptr<ChassisControllerPID> f8 = ChassisControllerFactory::createPtr(
    {1, 1}, {2, 2}, IterativePosPIDController::Gains(), IterativePosPIDController::Gains());

  ChassisControllerPID f9 = ChassisControllerFactory::create(1,
                                                             2,
                                                             IterativePosPIDController::Gains(),
                                                             IterativePosPIDController::Gains(),
                                                             IterativePosPIDController::Gains());
  ChassisControllerPID f10 = ChassisControllerFactory::create({1, 1},
                                                              {2, 2},
                                                              IterativePosPIDController::Gains(),
                                                              IterativePosPIDController::Gains(),
                                                              IterativePosPIDController::Gains());
  std::shared_ptr<ChassisControllerPID> f19 =
    ChassisControllerFactory::createPtr(1,
                                        2,
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains());
  std::shared_ptr<ChassisControllerPID> f20 =
    ChassisControllerFactory::createPtr({1, 1},
                                        {2, 2},
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains());

  ChassisControllerPID f11 = ChassisControllerFactory::create(1,
                                                              2,
                                                              ADIEncoder('A', 'B'),
                                                              ADIEncoder('A', 'B'),
                                                              IterativePosPIDController::Gains(),
                                                              IterativePosPIDController::Gains());
  ChassisControllerPID f12 = ChassisControllerFactory::create({1, 1},
                                                              {2, 2},
                                                              ADIEncoder('A', 'B'),
                                                              ADIEncoder('A', 'B'),
                                                              IterativePosPIDController::Gains(),
                                                              IterativePosPIDController::Gains());
  std::shared_ptr<ChassisControllerPID> f21 =
    ChassisControllerFactory::createPtr(1,
                                        2,
                                        ADIEncoder('A', 'B'),
                                        ADIEncoder('A', 'B'),
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains());
  std::shared_ptr<ChassisControllerPID> f22 =
    ChassisControllerFactory::createPtr({1, 1},
                                        {2, 2},
                                        ADIEncoder('A', 'B'),
                                        ADIEncoder('A', 'B'),
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains());

  ChassisControllerPID f13 = ChassisControllerFactory::create(1,
                                                              2,
                                                              ADIEncoder('A', 'B'),
                                                              ADIEncoder('A', 'B'),
                                                              IterativePosPIDController::Gains(),
                                                              IterativePosPIDController::Gains(),
                                                              IterativePosPIDController::Gains());
  ChassisControllerPID f14 = ChassisControllerFactory::create({1, 1},
                                                              {2, 2},
                                                              ADIEncoder('A', 'B'),
                                                              ADIEncoder('A', 'B'),
                                                              IterativePosPIDController::Gains(),
                                                              IterativePosPIDController::Gains(),
                                                              IterativePosPIDController::Gains());
  std::shared_ptr<ChassisControllerPID> f23 =
    ChassisControllerFactory::createPtr(1,
                                        2,
                                        ADIEncoder('A', 'B'),
                                        ADIEncoder('A', 'B'),
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains());
  std::shared_ptr<ChassisControllerPID> f24 =
    ChassisControllerFactory::createPtr({1, 1},
                                        {2, 2},
                                        ADIEncoder('A', 'B'),
                                        ADIEncoder('A', 'B'),
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains());

  ChassisControllerPID f15 = ChassisControllerFactory::create(1,
                                                              2,
                                                              IntegratedEncoder(pros::Motor(1)),
                                                              IntegratedEncoder(pros::Motor(1)),
                                                              IterativePosPIDController::Gains(),
                                                              IterativePosPIDController::Gains());
  ChassisControllerPID f16 = ChassisControllerFactory::create({1, 1},
                                                              {2, 2},
                                                              IntegratedEncoder(pros::Motor(1)),
                                                              IntegratedEncoder(pros::Motor(1)),
                                                              IterativePosPIDController::Gains(),
                                                              IterativePosPIDController::Gains());
  std::shared_ptr<ChassisControllerPID> f25 =
    ChassisControllerFactory::createPtr(1,
                                        2,
                                        IntegratedEncoder(pros::Motor(1)),
                                        IntegratedEncoder(pros::Motor(1)),
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains());
  std::shared_ptr<ChassisControllerPID> f26 =
    ChassisControllerFactory::createPtr({1, 1},
                                        {2, 2},
                                        IntegratedEncoder(pros::Motor(1)),
                                        IntegratedEncoder(pros::Motor(1)),
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains());
  ChassisControllerPID f17 = ChassisControllerFactory::create(1,
                                                              2,
                                                              IntegratedEncoder(pros::Motor(1)),
                                                              IntegratedEncoder(pros::Motor(1)),
                                                              IterativePosPIDController::Gains(),
                                                              IterativePosPIDController::Gains(),
                                                              IterativePosPIDController::Gains());
  ChassisControllerPID f18 = ChassisControllerFactory::create({1, 1},
                                                              {2, 2},
                                                              IntegratedEncoder(pros::Motor(1)),
                                                              IntegratedEncoder(pros::Motor(1)),
                                                              IterativePosPIDController::Gains(),
                                                              IterativePosPIDController::Gains(),
                                                              IterativePosPIDController::Gains());
  std::shared_ptr<ChassisControllerPID> f27 =
    ChassisControllerFactory::createPtr(1,
                                        2,
                                        IntegratedEncoder(pros::Motor(1)),
                                        IntegratedEncoder(pros::Motor(1)),
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains());
  std::shared_ptr<ChassisControllerPID> f28 =
    ChassisControllerFactory::createPtr({1, 1},
                                        {2, 2},
                                        IntegratedEncoder(pros::Motor(1)),
                                        IntegratedEncoder(pros::Motor(1)),
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains(),
                                        IterativePosPIDController::Gains());
}
