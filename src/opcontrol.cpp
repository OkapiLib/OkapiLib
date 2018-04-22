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

void testIterativeControllers() {
  using namespace okapi;
  using namespace snowhouse;

  {
    test_printf("Testing IterativePosPIDController");

    class MockTimer : public Timer {
      public:
      using Timer::Timer;
      virtual std::uint32_t getDtFromHardMark() const override {
        return 10;
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
      virtual std::uint32_t getDtFromHardMark() const override {
        return 10;
      }
      virtual std::uint32_t getDt() override {
        return 10;
      }
    };

    FlywheelSimulator sim;
    sim.setExternalTorqueFunction([](double, double, double) { return 0; });

    IterativeVelPIDController controller(
      0.000015, 0, std::make_unique<VelMath>(1800, std::make_shared<PassthroughFilter>(),
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
      MockIterativeVelPIDController() : IterativeVelPIDController(0, 0) {
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
         TEST_BODY(AssertThat, sim.getAngle(), EqualsWithDelta(0.0001237742, 0.00000001)));
    test("FlywheelSimulator i = 0 omega",
         TEST_BODY(AssertThat, sim.getOmega(), EqualsWithDelta(0.0247548337, 0.00000001)));
    test("FlywheelSimulator i = 0 accel",
         TEST_BODY(AssertThat, sim.getAcceleration(), EqualsWithDelta(990.19335, 0.0001)));
  }
}

void runHeadlessControllerTests() {
  testControlUtils();
  testIterativeControllers();
  testAsyncControllers();
}

void testUtils() {
  using namespace okapi;
  using namespace snowhouse;

  {
    test_printf("Testing ipow");

    test_printf("Integer tests");
    test("0^0 == 1", TEST_BODY(AssertThat, ipow(0, 0), Equals(1)));
    test("0^1 == 0", TEST_BODY(AssertThat, ipow(0, 1), Equals(0)));
    test("1^0 == 1", TEST_BODY(AssertThat, ipow(1, 0), Equals(1)));
    test("1^1 == 1", TEST_BODY(AssertThat, ipow(1, 1), Equals(1)));
    test("2^1 == 2", TEST_BODY(AssertThat, ipow(2, 1), Equals(2)));
    test("2^2 == 4", TEST_BODY(AssertThat, ipow(2, 2), Equals(4)));

    test_printf("Floating point tests");
    test("0.5^1 == 0.5", TEST_BODY(AssertThat, ipow(0.5, 1), EqualsWithDelta(0.5, 0.0001)));
    test("2.5^2 == 6.25", TEST_BODY(AssertThat, ipow(2.5, 2), EqualsWithDelta(6.25, 0.0001)));
  }

  {
    test_printf("Testing cutRange");

    test("1 : [-2, 2] -> 0", TEST_BODY(AssertThat, cutRange(1, -2, 2), EqualsWithDelta(2, 0.0001)));
    test("2 : [-2, 2] -> 0", TEST_BODY(AssertThat, cutRange(2, -2, 2), EqualsWithDelta(2, 0.0001)));
    test("0 : [-2, 2] -> 0", TEST_BODY(AssertThat, cutRange(0, -2, 2), EqualsWithDelta(2, 0.0001)));
    test("-2 : [-2, 2] -> 0",
         TEST_BODY(AssertThat, cutRange(-2, -2, 2), EqualsWithDelta(-2, 0.0001)));
    test("-3 : [-2, 2] -> -3",
         TEST_BODY(AssertThat, cutRange(-3, -2, 2), EqualsWithDelta(-3, 0.0001)));
    test("3 : [-2, 2] -> -3",
         TEST_BODY(AssertThat, cutRange(3, -2, 2), EqualsWithDelta(3, 0.0001)));
  }

  {
    test_printf("Testing deadband");

    test("0 : [-2, 2] -> 0", TEST_BODY(AssertThat, deadband(0, -2, 2), EqualsWithDelta(0, 0.0001)));
    test("1 : [-2, 2] -> 0", TEST_BODY(AssertThat, deadband(1, -2, 2), EqualsWithDelta(0, 0.0001)));
    test("2 : [-2, 2] -> 0", TEST_BODY(AssertThat, deadband(2, -2, 2), EqualsWithDelta(0, 0.0001)));
    test("-2 : [-2, 2] -> 0",
         TEST_BODY(AssertThat, deadband(-2, -2, 2), EqualsWithDelta(0, 0.0001)));
    test("3 : [-2, 2] -> 3", TEST_BODY(AssertThat, deadband(3, -2, 2), EqualsWithDelta(3, 0.0001)));
    test("-3 : [-2, 2] -> -3",
         TEST_BODY(AssertThat, deadband(-3, -2, 2), EqualsWithDelta(-3, 0.0001)));
  }

  {
    test_printf("Testing remapRange");

    test("0 : [-1, 1] -> [-2, 2]",
         TEST_BODY(AssertThat, remapRange(0, -1, 1, -2, 2), EqualsWithDelta(0, 0.0001)));
    test("0.1 : [-1, 1] -> [-2, 2]",
         TEST_BODY(AssertThat, remapRange(0.1, -1, 1, -2, 2), EqualsWithDelta(0.2, 0.0001)));
    test("-0.1 : [-1, 1] -> [2, -2]",
         TEST_BODY(AssertThat, remapRange(-0.1, -1, 1, 2, -2), EqualsWithDelta(0.2, 0.0001)));
    test("0 : [-1, 1] -> [-5, 2]",
         TEST_BODY(AssertThat, remapRange(0, -1, 1, -5, 2), EqualsWithDelta(-1.5, 0.0001)));
  }

  {
    test_printf("Testing Rate");

    Rate rate;
    uint32_t lastTime = pros::millis();

    for (int i = 0; i < 10; i++) {
      rate.delayHz(10);

      // Static cast so the compiler doesn't complain about comparing signed and unsigned values
      test("Rate " + std::to_string(i),
           TEST_BODY(AssertThat, static_cast<double>(pros::millis() - lastTime),
                     EqualsWithDelta(100, 10)));

      lastTime = pros::millis();
      pros::Task::delay(50); // Emulate some computation
    }
  }
}

void runHeadlessUtilTests() {
  testUtils();
}

void testFilters() {
  using namespace okapi;
  using namespace snowhouse;

  {
    test_printf("Testing AverageFilter");

    AverageFilter<5> filt;

    for (int i = 0; i < 10; i++) {
      auto testName = "AverageFilter i = " + std::to_string(i);
      switch (i) {
      case 0: {
        test(testName, TEST_BODY(AssertThat, filt.filter(i), Equals(0)));
        break;
      }

      case 1: {
        test(testName, TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(0.2, 0.01)));
        break;
      }

      case 2: {
        test(testName, TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(0.6, 0.01)));
        break;
      }

      case 3: {
        test(testName, TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(1.2, 0.01)));
        break;
      }

      default: {
        test(testName, TEST_BODY(AssertThat, filt.filter(i), Equals(i - 2)));
        break;
      }
      }
    }
  }

  {
    test_printf("Testing MedianFilter");

    MedianFilter<5> filt;

    for (int i = 0; i < 10; i++) {
      auto testName = "MedianFilter i = " + std::to_string(i);
      if (i < 3) {
        test(testName, TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(0, 0.0001)));
      } else {
        test(testName, TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(i - 2, 0.0001)));
      }
    }
  }

  {
    test_printf("Testing EmaFilter");

    EmaFilter filt(0.5);

    test("EmaFilter i = 0", TEST_BODY(AssertThat, filt.filter(0), EqualsWithDelta(0, 0.0001)));
    test("EmaFilter i = 1", TEST_BODY(AssertThat, filt.filter(1), EqualsWithDelta(0.5, 0.0001)));
    test("EmaFilter i = 2", TEST_BODY(AssertThat, filt.filter(2), EqualsWithDelta(1.25, 0.0001)));
    test("EmaFilter i = -3",
         TEST_BODY(AssertThat, filt.filter(-3), EqualsWithDelta(-0.875, 0.0001)));

    EmaFilter filt2(1);
    test("EmaFilter with alpha = 1 should return input signal 1",
         TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
    test("EmaFilter with alpha = 1 should return input signal 2",
         TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
    test("EmaFilter with alpha = 1 should return input signal 3",
         TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
  }

  {
    test_printf("Testing DemaFilter");

    DemaFilter filt(0.5, 0.05);

    test("DemaFilter i = 0", TEST_BODY(AssertThat, filt.filter(0), EqualsWithDelta(0, 0.0001)));
    test("DemaFilter i = 1", TEST_BODY(AssertThat, filt.filter(1), EqualsWithDelta(0.525, 0.0001)));
    test("DemaFilter i = 2",
         TEST_BODY(AssertThat, filt.filter(2), EqualsWithDelta(1.3244, 0.0001)));
    test("DemaFilter i = 2",
         TEST_BODY(AssertThat, filt.filter(2), EqualsWithDelta(1.7410, 0.0001)));
    test("DemaFilter i = 2",
         TEST_BODY(AssertThat, filt.filter(2), EqualsWithDelta(1.9557, 0.0001)));

    DemaFilter filt2(1, 0);
    test("DemaFilter with alpha = 1 and beta = 0 should return input signal 1",
         TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
    test("DemaFilter with alpha = 1 and beta = 0 should return input signal 2",
         TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
    test("DemaFilter with alpha = 1 and beta = 0 should return input signal 3",
         TEST_BODY(AssertThat, filt2.filter(5), EqualsWithDelta(5, 0.0001)));
  }

  {
    test_printf("Testing EKFFilter");

    EKFFilter filt(0.0001, ipow(0.2, 2));

    test("EKFFilter i = 0", TEST_BODY(AssertThat, filt.filter(0), EqualsWithDelta(0, 0.0001)));
    test("EKFFilter i = 0.5",
         TEST_BODY(AssertThat, filt.filter(0.5), EqualsWithDelta(0.2454, 0.0001)));
    test("EKFFilter i = -0.5",
         TEST_BODY(AssertThat, filt.filter(-0.5), EqualsWithDelta(-0.0008, 0.0001)));
    test("EKFFilter i = 0.5",
         TEST_BODY(AssertThat, filt.filter(0.5), EqualsWithDelta(0.1242, 0.0001)));
    test("EKFFilter i = 0", TEST_BODY(AssertThat, filt.filter(0), EqualsWithDelta(0.0992, 0.0001)));
  }

  {
    test_printf("Testing ComposableFilter");

    ComposableFilter filt(
      {std::make_shared<AverageFilter<3>>(), std::make_shared<AverageFilter<3>>()});

    test("ComposableFilter i = 1",
         TEST_BODY(AssertThat, filt.filter(1), EqualsWithDelta(0.1111, 0.0001)));
    test("ComposableFilter i = 2",
         TEST_BODY(AssertThat, filt.filter(2), EqualsWithDelta(0.4444, 0.0001)));
    test("ComposableFilter i = 3",
         TEST_BODY(AssertThat, filt.filter(3), EqualsWithDelta(1.1111, 0.0001)));

    for (int i = 4; i < 10; i++) {
      test("ComposableFilter i = " + std::to_string(i),
           TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(i - 2, 0.0001)));
    }
  }

  {
    test_printf("Testing PassthroughFilter");

    PassthroughFilter filt;

    for (int i = 0; i < 5; i++) {
      test("PassthroughFilter i = " + std::to_string(i),
           TEST_BODY(AssertThat, filt.filter(i), EqualsWithDelta(i, 0.0001)));
    }
  }

  {
    test_printf("Testing VelMath");

    class MockTimer : public Timer {
      public:
      using Timer::Timer;
      virtual std::uint32_t getDt() override {
        return 10;
      }
    };

    VelMath velMath(360, std::make_shared<PassthroughFilter>(), std::make_unique<MockTimer>());

    for (int i = 0; i < 10; i++) {
      if (i == 0) {
        test("VelMath " + std::to_string(i),
             TEST_BODY(AssertThat, velMath.step(i * 10), EqualsWithDelta(0, 0.01)));
      } else {
        // 10 ticks per 100 ms should be ~16.67 rpm
        test("VelMath " + std::to_string(i),
             TEST_BODY(AssertThat, velMath.step(i * 10), EqualsWithDelta(166.67, 0.01)));
      }
    }
  }
}

void runHeadlessFilterTests() {
  testFilters();
}

void testSkidSteerModel() {
  using namespace okapi;
  using namespace snowhouse;

  test_printf("Testing SkidSteerModel");

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
    mutable std::int16_t lastVelocity;
    mutable std::int16_t lastVoltage;
  };

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

void testXDriveModel() {
  using namespace okapi;
  using namespace snowhouse;

  test_printf("Testing XDriveModel");

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
    mutable std::int16_t lastVelocity;
    mutable std::int16_t lastVoltage;
  };

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

void testChassisModels() {
  testSkidSteerModel();
  testXDriveModel();
}

void runHeadlessChassisModelTests() {
  testChassisModels();
}

void testButtons() {
  using namespace okapi;
  using namespace snowhouse;
  using namespace fakeit;

  class MockButton : public Button {
    public:
    bool currentlyPressed() override {
      printf("???\n");
      return false;
    }
  };

  {
    test_printf("Testing Button");

    {
      MockButton spyMe;
      Mock<MockButton> mockFactory(spyMe);
      When(Method(mockFactory, currentlyPressed)).Return(false).Return(true).Return(false);
      Spy(Method(mockFactory, isPressed));
      MockButton &btn = mockFactory.get();

      test("Button isPressed should be false",
           TEST_BODY(AssertThat, btn.isPressed(), Equals(false)));
      test("Button isPressed should be true", TEST_BODY(AssertThat, btn.isPressed(), Equals(true)));
      test("Button isPressed should be false",
           TEST_BODY(AssertThat, btn.isPressed(), Equals(false)));
    }

    {
      MockButton spyMe;
      Mock<MockButton> mockFactory(spyMe);
      When(Method(mockFactory, currentlyPressed))
        .Return(false)
        .Return(true)
        .Return(true)
        .Return(false)
        .Return(false);
      Spy(Method(mockFactory, changed));
      MockButton &btn = mockFactory.get();

      test("Button changed should be false", TEST_BODY(AssertThat, btn.changed(), Equals(false)));
      test("Button changed should be true", TEST_BODY(AssertThat, btn.changed(), Equals(true)));
      test("Button changed should be false", TEST_BODY(AssertThat, btn.changed(), Equals(false)));
      test("Button changed should be true", TEST_BODY(AssertThat, btn.changed(), Equals(true)));
      test("Button changed should be false", TEST_BODY(AssertThat, btn.changed(), Equals(false)));
    }

    {
      MockButton spyMe;
      Mock<MockButton> mockFactory(spyMe);
      When(Method(mockFactory, currentlyPressed))
        .Return(false)
        .Return(true)
        .Return(true)
        .Return(false)
        .Return(false);
      Spy(Method(mockFactory, changed));
      Spy(Method(mockFactory, changedToPressed));
      MockButton &btn = mockFactory.get();

      test("Button changedToPressed should be false",
           TEST_BODY(AssertThat, btn.changedToPressed(), Equals(false)));
      test("Button changedToPressed should be true",
           TEST_BODY(AssertThat, btn.changedToPressed(), Equals(true)));
      test("Button changedToPressed should be false",
           TEST_BODY(AssertThat, btn.changedToPressed(), Equals(false)));
      test("Button changedToPressed should be true",
           TEST_BODY(AssertThat, btn.changedToPressed(), Equals(false)));
      test("Button changedToPressed should be false",
           TEST_BODY(AssertThat, btn.changedToPressed(), Equals(false)));
    }

    {
      MockButton spyMe;
      Mock<MockButton> mockFactory(spyMe);
      When(Method(mockFactory, currentlyPressed))
        .Return(false)
        .Return(true)
        .Return(true)
        .Return(false)
        .Return(false);
      Spy(Method(mockFactory, changed));
      Spy(Method(mockFactory, changedToReleased));
      MockButton &btn = mockFactory.get();

      test("Button changedToReleased should be false",
           TEST_BODY(AssertThat, btn.changedToReleased(), Equals(false)));
      test("Button changedToReleased should be true",
           TEST_BODY(AssertThat, btn.changedToReleased(), Equals(false)));
      test("Button changedToReleased should be false",
           TEST_BODY(AssertThat, btn.changedToReleased(), Equals(false)));
      test("Button changedToReleased should be true",
           TEST_BODY(AssertThat, btn.changedToReleased(), Equals(true)));
      test("Button changedToReleased should be false",
           TEST_BODY(AssertThat, btn.changedToReleased(), Equals(false)));
    }
  }
}

void runHeadlessDeviceTests() {
  testButtons();
}

void runHeadlessTests() {
  using namespace okapi;

  runHeadlessDeviceTests();
  runHeadlessUtilTests();
  runHeadlessFilterTests();
  runHeadlessControllerTests();
  runHeadlessChassisModelTests();

  test_print_report();
}

void constructorTests() {
  using namespace okapi;

  {
    ADIButton btn(2);
    ControllerButton btn2(E_CONTROLLER_DIGITAL_A);
    btn.isPressed();
    btn.changed();
    btn.changedToPressed();
    btn.changedToReleased();
  }

  {
    ADIEncoder leftEncoder(1, 2, true);
    ADIEncoder rightEncoder(3, 4);
    ADIEncoder test('A', 'B');
    ADIEncoder test2('a', 'b');
    leftEncoder.get();
  }

  {
    ADIUltrasonic ultra1(1, 2);
    ultra1.get();
  }

  {
    Motor mtr = 1_mtr;
    Motor r_mtr = 2_rmtr;
  }

  {
    ADIEncoder leftEncoder(1, 2, true);
    ADIEncoder rightEncoder(3, 4);
    ChassisControllerIntegrated int1(1_mtr,  // One motor on left side
                                     2_mtr); // One motor on right side

    ChassisControllerIntegrated int2(MotorGroup({1_mtr, 2_mtr, 3_mtr}), // Three motors on left side
                                     MotorGroup({4_mtr, 5_mtr}));       // Two motors on right side

    int1.moveDistance(0); // Closed-loop control
    int1.turnAngle(0);    // Closed-loop control

    int1.forward(0);                  // Open-loop control
    int1.rotate(0);                   // Open-loop control
    int1.driveVector(0, 0);           // Open-loop control
    int1.tank(0, 0);                  // Tank drive
    int1.arcade(0, 0);                // Arcade drive
    int1.left(0);                     // Left drive side
    int1.right(0);                    // Right drive side
    int1.stop();                      // Stop motors
    auto vals = int1.getSensorVals(); // Read left and right sensors
    int1.resetSensors();              // Set sensors to 0

    ChassisControllerPID controller1(
      std::make_shared<SkidSteerModel>(MotorGroup({1_mtr, 2_mtr}), MotorGroup({3_mtr, 4_mtr}),
                                       leftEncoder, rightEncoder),
      IterativePosPIDControllerArgs(0, 0, 0), IterativePosPIDControllerArgs(0, 0, 0));

    ChassisControllerPID controller2(
      std::make_shared<XDriveModel>(1_mtr, 2_mtr, 3_mtr, 4_mtr, leftEncoder, rightEncoder),
      IterativePosPIDControllerArgs(0, 0, 0), IterativePosPIDControllerArgs(0, 0, 0));
  }

  {
    IterativePosPIDController pid1(0, 0, 0); // PID controller
    IterativeMotorVelocityController mc1(1_mtr, std::make_shared<IterativeVelPIDController>(0, 0));
    IterativeMotorVelocityController mc2(MotorGroup({1_mtr, 2_mtr}),
                                         std::make_shared<IterativeVelPIDController>(0, 0));
  }

  { AsyncPosIntegratedController posI1(1_mtr); }

  {
    AsyncPosPIDController apospid1(std::make_shared<ADIEncoder>(1, 2, true),
                                   std::make_shared<Motor>(1_mtr),
                                   IterativePosPIDControllerArgs(0, 0, 0));

    AsyncPosPIDController apospid2(std::make_shared<ADIEncoder>(1, 2, true),
                                   std::make_shared<Motor>(1_mtr), 0, 0, 0);
  }

  {
    IterativePosPIDController pid2(0, 0, 0);
    IterativePosPIDController pid3(0, 0, 0, 0);
    IterativePosPIDController pid4(IterativePosPIDControllerArgs(0, 0, 0));
    IterativePosPIDController pid5(IterativePosPIDControllerArgs(0, 0, 0, 0));
  }

  {
    VelMath velMath1(0);
    VelMath velMath2(0, 0);
    VelMath velMath3(0, std::make_shared<DemaFilter>(0.0, 0.0));
  }

  {
    IterativeVelPIDController velPid1(0, 0);
    IterativeVelPIDController velPid2(IterativeVelPIDControllerArgs(0, 0));
  }

  {
    ADIEncoder quad1(0, 0);
    ADIEncoder quad2(0, 0, true);
  }

  { MotorGroup mg1({1_mtr, 2_mtr}); }

  {
    AverageFilter<2> avgFilt1;
    avgFilt1.filter(0);
    avgFilt1.getOutput();
  }

  { DemaFilter demaFilt1(0, 0); }

  {
    EKFFilter ekfFilter1;
    EKFFilter ekfFilter2(0);
    EKFFilter ekfFilter3(0, 0);
  }

  { EmaFilter emaFilt1(0); }

  { MedianFilter<5> medianFilt1; }

  { Timer timer1(); }

  {
    ControllerRunner controllerRunner;
    AsyncPosIntegratedController testControllerRunnerController1(1_mtr);
    IterativePosPIDController testControllerRunnerController2(0, 0, 0);
    Motor controllerRunnerMotor = 1_mtr;
    controllerRunner.runUntilSettled(0, testControllerRunnerController1);
    controllerRunner.runUntilSettled(0, testControllerRunnerController2, controllerRunnerMotor);
    controllerRunner.runUntilAtTarget(0, testControllerRunnerController1);
    controllerRunner.runUntilAtTarget(0, testControllerRunnerController2, controllerRunnerMotor);
  }

  {
    SettledUtil settledUtil1;
    settledUtil1.isSettled(0);
  }

  {
    auto mtr = 1_mtr;
    AsyncVelPIDController con(std::make_shared<IntegratedEncoder>(mtr),
                              std::make_shared<Motor>(mtr), 0, 0);
  }

  {
    auto mtr = 1_mtr;
    AsyncWrapper wrapper(std::make_shared<IntegratedEncoder>(mtr), std::make_shared<Motor>(mtr),
                         std::make_unique<IterativePosPIDController>(0, 0, 0));
  }
}

void opcontrol() {
  using namespace okapi;
  pros::Task::delay(100);

  runHeadlessTests();
  return;

  MotorGroup leftMotors({19_mtr, 20_mtr});
  MotorGroup rightMotors({13_rmtr, 14_rmtr});
  Motor armMotor = 15_mtr;

  ChassisControllerIntegrated robotChassisController({19, 20}, {-13, -14}, E_MOTOR_GEARSET_36,
                                                     1127.86968, 2.8745);

  Controller controller;
  ControllerButton btn1(E_CONTROLLER_DIGITAL_A);
  ControllerButton btn2(E_CONTROLLER_DIGITAL_B);
  ControllerButton btn3(E_CONTROLLER_DIGITAL_Y);
  ControllerButton btn4(E_CONTROLLER_DIGITAL_X);

  while (true) {
    // printf("loop\n");
    robotChassisController.arcade(controller.getAnalog(E_CONTROLLER_ANALOG_LEFT_Y),
                                  controller.getAnalog(E_CONTROLLER_ANALOG_LEFT_X));

    if (btn1.changedToPressed()) {
      printf("move distance\n");
      robotChassisController.moveDistance(12.0_in);
    }

    if (btn2.changedToPressed()) {
      printf("turn angle\n");
      robotChassisController.turnAngle(90.0_deg);
    }

    if (btn3.changedToPressed()) {
      printf("move arm\n");
      armMotor.moveRelative(-10, 127);
    }

    if (btn4.changedToPressed()) {
      printf("autonomous routine\n");
      for (int i = 0; i < 4; i++) {
        robotChassisController.moveDistance(12);
        robotChassisController.turnAngle(90);
      }
    }

    pros::Task::delay(10);
  }
}
