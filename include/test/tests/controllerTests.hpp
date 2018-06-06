/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CONTROLLERTESTS_HPP_
#define _OKAPI_CONTROLLERTESTS_HPP_

#include "okapi/api.hpp"
#include "test/testRunner.hpp"

void testIterativeControllers() {
  using namespace okapi;
  using namespace snowhouse;

  {
    test_printf("Testing IterativePosPIDController");

    class MockTimer : public Timer {
      public:
      using Timer::Timer;
      virtual QTime getDtFromHardMark() const override {
        return 10_ms;
      }
    };

    FlywheelSimulator sim;
    sim.setExternalTorqueFunction([](double, double, double) { return 0; });

    IterativePosPIDController controller(0.004, 0, 0, 0, std::make_unique<MockTimer>(),
                                         std::make_unique<SettledUtil>());

    const double target = 45;
    controller.setTarget(target);
    for (size_t i = 0; i < 2000; i++) {
      controller.step(sim.getAngle() * radianToDegree);
      sim.step(controller.getOutput() * sim.getMaxTorque());
    }

    test("IterativePosPIDController should settle after 2000 iterations (simulator angle is "
         "correct)",
         TEST_BODY(AssertThat, sim.getAngle(), EqualsWithDelta(target * degreeToRadian, 0.01)));
    test("IterativePosPIDController should settle after 2000 iterations (controller error is "
         "correct)",
         TEST_BODY(AssertThat, controller.getError(), EqualsWithDelta(0, 0.01)));
  }

  {
    test_printf("Testing IterativeVelPIDController");

    class MockTimer : public Timer {
      public:
      using Timer::Timer;
      virtual QTime getDtFromHardMark() const override {
        return 10_ms;
      }
      virtual QTime getDt() override {
        return 10_ms;
      }
    };

    FlywheelSimulator sim;
    sim.setExternalTorqueFunction([](double, double, double) { return 0; });

    IterativeVelPIDController controller(
      0.000015, 0, 0,
      std::make_unique<VelMath>(1800, std::make_shared<PassthroughFilter>(),
                                std::make_unique<MockTimer>()),
      std::make_unique<MockTimer>(), std::make_unique<SettledUtil>());

    const double target = 10;
    controller.setTarget(target);
    for (size_t i = 0; i < 2000; i++) {
      controller.step(sim.getAngle() * radianToDegree);
      sim.step(controller.getOutput() * sim.getMaxTorque());
    }

    test("IterativeVelPIDController should settle after 2000 iterations (simulator omega is "
         "correct)",
         TEST_BODY(AssertThat, sim.getOmega(), EqualsWithDelta(1.04719755, 0.01)));
    test("IterativeVelPIDController should settle after 2000 iterations (controller error is "
         "correct)",
         TEST_BODY(AssertThat, controller.getError(), EqualsWithDelta(0, 0.01)));
  }

  {
    test_printf("Testing IterativeMotorVelocityController");

    class MockMotor : public Motor {
      public:
      MockMotor() : Motor(1) {
      }
      virtual std::int32_t moveVelocity(const std::int16_t ivelocity) const override {
        lastVelocity = ivelocity;
        return 1;
      }
      mutable std::int16_t lastVelocity;
    };

    class MockIterativeVelPIDController : public IterativeVelPIDController {
      public:
      MockIterativeVelPIDController() : IterativeVelPIDController(0, 0, 0) {
      }
      virtual double step(const double inewReading) override {
        return inewReading;
      }
    };

    auto motor = std::make_shared<MockMotor>();

    IterativeMotorVelocityController controller(motor,
                                                std::make_shared<MockIterativeVelPIDController>());

    controller.step(0);
    test("IterativeMotorVelocityController should set the motor velocity to the controller output "
         "* 127 1",
         TEST_BODY(AssertThat, motor->lastVelocity, EqualsWithDelta(0 * 127, 0.01)));

    controller.step(0.5);
    test("IterativeMotorVelocityController should set the motor velocity to the controller output "
         "* 127 2",
         TEST_BODY(AssertThat, motor->lastVelocity, EqualsWithDelta(63, 0.01)));

    controller.step(1);
    test("IterativeMotorVelocityController should set the motor velocity to the controller output "
         "* 127 3",
         TEST_BODY(AssertThat, motor->lastVelocity, EqualsWithDelta(1 * 127, 0.01)));

    controller.step(-0.5);
    test("IterativeMotorVelocityController should set the motor velocity to the controller output "
         "* 127 4",
         TEST_BODY(AssertThat, motor->lastVelocity, EqualsWithDelta(-63, 0.01)));
  }
}

void testAsyncControllers() {
  using namespace okapi;
  using namespace snowhouse;

  {
    test_printf("Testing AsyncPosIntegratedController");

    class MockMotor : public Motor {
      public:
      MockMotor() : Motor(1) {
      }

      virtual std::int32_t moveAbsolute(const double iposition, const std::int32_t) const override {
        lastPosition = iposition;
        return 1;
      }

      virtual std::int32_t moveVoltage(const std::int16_t ivoltage) const override {
        lastVoltage = ivoltage;
        return 1;
      }

      virtual double getPosition() const override {
        return 0;
      }

      mutable std::int16_t lastVoltage;
      mutable std::int16_t lastPosition;
    };

    auto motor = std::make_shared<MockMotor>();

    AsyncPosIntegratedController controller(motor);

    controller.setTarget(100);
    test("Should be on by default", TEST_BODY(AssertThat, motor->lastPosition, Equals(100)));

    controller.flipDisable();
    test("Disabling the controller should turn the motor off",
         TEST_BODY(AssertThat, motor->lastVoltage, Equals(0)));

    controller.flipDisable();
    test("Re-enabling the controller should move the motor to the previous target",
         TEST_BODY(AssertThat, motor->lastPosition, Equals(100)));

    controller.flipDisable();
    controller.reset();
    test("Resetting the controller should not change the current target",
         TEST_BODY(AssertThat, motor->lastVoltage, Equals(0)));
    controller.flipDisable();
    motor->lastPosition = 1337; // Sample value to check it doesn't change
    test("Re-enabling the controller after a reset should not move the motor",
         TEST_BODY(AssertThat, motor->lastPosition, Equals(1337)));
  }

  {
    test_printf("Testing AsyncVelIntegratedController");

    class MockMotor : public Motor {
      public:
      MockMotor() : Motor(1) {
      }

      virtual std::int32_t moveVelocity(const std::int16_t ivelocity) const override {
        lastVelocity = ivelocity;
        return 1;
      }

      virtual std::int32_t moveVoltage(const std::int16_t ivoltage) const override {
        lastVoltage = ivoltage;
        return 1;
      }

      virtual double getActualVelocity() const override {
        return 0;
      }

      mutable std::int16_t lastVoltage;
      mutable std::int16_t lastVelocity;
    };

    auto motor = std::make_shared<MockMotor>();

    AsyncVelIntegratedController controller(motor);

    controller.setTarget(100);
    test("Should be on by default", TEST_BODY(AssertThat, motor->lastVelocity, Equals(100)));

    controller.flipDisable();
    test("Disabling the controller should turn the motor off",
         TEST_BODY(AssertThat, motor->lastVoltage, Equals(0)));

    controller.flipDisable();
    test("Re-enabling the controller should move the motor to the previous target",
         TEST_BODY(AssertThat, motor->lastVelocity, Equals(100)));

    controller.flipDisable();
    controller.reset();
    test("Resetting the controller should not change the current target",
         TEST_BODY(AssertThat, motor->lastVoltage, Equals(0)));
    controller.flipDisable();
    motor->lastVelocity = 1337; // Sample value to check it doesn't change
    test("Re-enabling the controller after a reset should not move the motor",
         TEST_BODY(AssertThat, motor->lastVelocity, Equals(1337)));
  }
}

void testControlUtils() {
  using namespace okapi;
  using namespace snowhouse;

  {
    test_printf("Testing FlywheelSimulator");

    FlywheelSimulator sim;

    sim.setTorque(0.3);
    sim.step();

    test("FlywheelSimulator i = 0 angle",
         TEST_BODY(AssertThat, sim.getAngle(), EqualsWithDelta(0.000020193, 0.00000005)));
    test("FlywheelSimulator i = 0 omega",
         TEST_BODY(AssertThat, sim.getOmega(), EqualsWithDelta(0.0020193, 0.000005)));
    test("FlywheelSimulator i = 0 accel",
         TEST_BODY(AssertThat, sim.getAcceleration(), EqualsWithDelta(20.193, 0.0005)));
  }
}

void testFilteredControllerInput() {
  using namespace okapi;
  using namespace snowhouse;

  {
    test_printf("Testing FilteredControllerInput");

    class MockControllerInput : public ControllerInput {
      public:
      MockControllerInput() = default;
      double controllerGet() override {
        return 1;
      }
    };

    MockControllerInput mockInput;
    PassthroughFilter filter;
    FilteredControllerInput<MockControllerInput, PassthroughFilter> input(mockInput, filter);

    for (int i = 0; i < 10; i++) {
      auto testName = "FilteredControllerInput i = " + std::to_string(i);
      test(testName, TEST_BODY(AssertThat, input.controllerGet(), EqualsWithDelta(1, 0.0001)));
    }
  }
}

void runHeadlessControllerTests() {
  testControlUtils();
  testIterativeControllers();
  testAsyncControllers();
  testFilteredControllerInput();
}

#endif
