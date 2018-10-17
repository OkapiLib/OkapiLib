/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_IMPLMOCKS_HPP_
#define _OKAPI_IMPLMOCKS_HPP_

#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/control/async/asyncPosIntegratedController.hpp"
#include "okapi/api/control/async/asyncVelIntegratedController.hpp"
#include "okapi/api/control/async/asyncWrapper.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/control/util/flywheelSimulator.hpp"
#include "okapi/api/control/util/settledUtil.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"
#include "okapi/api/util/abstractRate.hpp"
#include "okapi/api/util/abstractTimer.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include <atomic>
#include <chrono>
#include <gtest/gtest.h>

namespace okapi {

class MockContinuousRotarySensor : public ContinuousRotarySensor {
  public:
  double controllerGet() override;

  int32_t reset() override;

  double get() const override;

  mutable std::int32_t value{0};
};

/**
 * A motor mock that saves the last set position, velocity, and voltage.
 */
class MockMotor : public AbstractMotor {
  public:
  MockMotor();

  void controllerSet(double ivalue) override;

  int32_t moveAbsolute(double iposition, std::int32_t ivelocity) const override;

  int32_t moveRelative(double iposition, std::int32_t ivelocity) const override;

  double getTargetPosition() const override;

  double getPosition() const override;

  int32_t getTargetVelocity() const override;

  double getActualVelocity() const override;

  int32_t tarePosition() const override;

  int32_t setBrakeMode(brakeMode imode) override;

  int32_t setCurrentLimit(std::int32_t ilimit) const override;

  int32_t setEncoderUnits(encoderUnits iunits) override;

  int32_t setGearing(gearset igearset) override;

  int32_t setReversed(bool ireverse) const override;

  int32_t setVoltageLimit(std::int32_t ilimit) const override;

  std::shared_ptr<ContinuousRotarySensor> getEncoder() const override;

  std::int32_t moveVelocity(std::int16_t ivelocity) const override;

  std::int32_t moveVoltage(std::int16_t ivoltage) const override;

  int32_t getCurrentDraw() const override;

  int32_t getDirection() const override;

  double getEfficiency() const override;

  int32_t isOverCurrent() const override;

  int32_t isOverTemp() const override;

  int32_t isStopped() const override;

  int32_t getZeroPositionFlag() const override;

  uint32_t getFaults() const override;

  uint32_t getFlags() const override;

  int32_t getRawPosition(std::uint32_t *timestamp) const override;

  double getPower() const override;

  double getTemperature() const override;

  double getTorque() const override;

  int32_t getVoltage() const override;

  int32_t modifyProfiledVelocity(std::int32_t ivelocity) const override;

  int32_t setPosPID(double ikF, double ikP, double ikI, double ikD) const override;

  int32_t setPosPIDFull(double ikF,
                        double ikP,
                        double ikI,
                        double ikD,
                        double ifilter,
                        double ilimit,
                        double ithreshold,
                        double iloopSpeed) const override;

  int32_t setVelPID(double ikF, double ikP, double ikI, double ikD) const override;

  int32_t setVelPIDFull(double ikF,
                        double ikP,
                        double ikI,
                        double ikD,
                        double ifilter,
                        double ilimit,
                        double ithreshold,
                        double iloopSpeed) const override;

  AbstractMotor::brakeMode getBrakeMode() const override;

  int32_t getCurrentLimit() const override;

  AbstractMotor::encoderUnits getEncoderUnits() const override;

  AbstractMotor::gearset getGearing() const override;

  std::shared_ptr<MockContinuousRotarySensor> encoder;
  mutable std::int16_t lastVelocity{0};
  mutable std::int16_t maxVelocity{0};
  mutable std::int16_t lastVoltage{0};
  mutable std::int16_t lastPosition{0};
  mutable std::int32_t lastProfiledMaxVelocity{0};
  AbstractMotor::gearset gearset{AbstractMotor::gearset::green};
  AbstractMotor::encoderUnits encoderUnits{AbstractMotor::encoderUnits::counts};
  AbstractMotor::brakeMode brakeMode{AbstractMotor::brakeMode::coast};
};

/**
 * A timer mock that implements all features using the system timer.
 */
class MockTimer : public AbstractTimer {
  public:
  MockTimer();

  QTime millis() const override;

  std::chrono::system_clock::time_point epoch = std::chrono::high_resolution_clock::from_time_t(0);
};

/**
 * A timer mock that always returns a constant dt and 0 for other methods.
 */
class ConstantMockTimer : public AbstractTimer {
  public:
  explicit ConstantMockTimer(QTime idt);

  QTime millis() const override;

  QTime getDt() override;

  QTime readDt() const override;

  QTime getStartingTime() const override;

  QTime getDtFromStart() const override;

  void placeMark() override;

  QTime clearMark() override;

  void placeHardMark() override;

  QTime clearHardMark() override;

  QTime getDtFromMark() const override;

  QTime getDtFromHardMark() const override;

  bool repeat(QTime time) override;

  bool repeat(QFrequency frequency) override;

  QTime dtToReturn;
};

class MockRate : public AbstractRate {
  public:
  MockRate();

  void delay(QFrequency ihz) override;

  void delay(int ihz) override;

  void delayUntil(QTime itime) override;

  void delayUntil(uint32_t ims) override;
};

std::unique_ptr<SettledUtil> createSettledUtilPtr(double iatTargetError = 50,
                                                  double iatTargetDerivative = 5,
                                                  QTime iatTargetTime = 250_ms);

TimeUtil createTimeUtil();

TimeUtil createConstantTimeUtil(QTime idt);

TimeUtil createTimeUtil(const Supplier<std::unique_ptr<AbstractTimer>> &itimerSupplier);

TimeUtil createTimeUtil(const Supplier<std::unique_ptr<SettledUtil>> &isettledUtilSupplier);

class SimulatedSystem : public ControllerInput<double>, public ControllerOutput<double> {
  public:
  explicit SimulatedSystem(FlywheelSimulator &simulator);

  virtual ~SimulatedSystem();

  double controllerGet() override;

  void controllerSet(double ivalue) override;

  void step();

  static void trampoline(void *system);

  void startThread();

  void join();

  FlywheelSimulator &simulator;
  MockRate rate{};
  std::atomic_bool dtorCalled{false};
  std::thread thread;
};

class MockAsyncPosIntegratedController : public AsyncPosIntegratedController {
  public:
  MockAsyncPosIntegratedController();

  explicit MockAsyncPosIntegratedController(const TimeUtil &itimeUtil);

  bool isSettled() override;

  bool isSettledOverride{true};
  using AsyncPosIntegratedController::maxVelocity;
};

class MockAsyncVelIntegratedController : public AsyncVelIntegratedController {
  public:
  MockAsyncVelIntegratedController();

  void setTarget(double itarget) override;

  bool isSettled() override;

  void controllerSet(double ivalue) override;

  bool isSettledOverride{true};

  double lastTarget{0};
  double maxTarget{0};

  double lastControllerOutputSet{0};
  double maxControllerOutputSet{0};
};

class MockIterativeController : public IterativePosPIDController {
  public:
  MockIterativeController();

  explicit MockIterativeController(double ikP);

  bool isSettled() override;

  bool isSettledOverride{true};
};

void assertMotorsHaveBeenStopped(MockMotor *leftMotor, MockMotor *rightMotor);

void assertMotorsGearsetEquals(AbstractMotor::gearset expected,
                               const std::initializer_list<MockMotor> &motors);

void assertMotorsBrakeModeEquals(AbstractMotor::brakeMode expected,
                                 const std::initializer_list<MockMotor> &motors);

void assertMotorsEncoderUnitsEquals(AbstractMotor::encoderUnits expected,
                                    const std::initializer_list<MockMotor> &motors);

template <typename I, typename O>
void assertControllerIsSettledWhenDisabled(ClosedLoopController<I, O> &controller, I target) {
  controller.flipDisable(false);
  EXPECT_FALSE(controller.isDisabled());
  controller.setTarget(target);
  EXPECT_EQ(controller.getTarget(), target);
  EXPECT_FALSE(controller.isSettled());

  controller.flipDisable();
  EXPECT_TRUE(controller.isDisabled());
  EXPECT_TRUE(controller.isSettled());
}

template <typename I, typename O>
void assertWaitUntilSettledWorksWhenDisabled(AsyncController<I, O> &controller) {
  controller.flipDisable(true);
  EXPECT_TRUE(controller.isDisabled());
  controller.waitUntilSettled();
}

void assertAsyncControllerFollowsDisableLifecycle(AsyncController<double, double> &controller,
                                                  std::int16_t &domainValue,
                                                  std::int16_t &voltageValue,
                                                  int expectedOutput);

void assertIterativeControllerFollowsDisableLifecycle(
  IterativeController<double, double> &controller);

void assertControllerFollowsTargetLifecycle(ClosedLoopController<double, double> &controller);

void assertIterativeControllerScalesControllerSetTargets(
  IterativeController<double, double> &controller);

void assertAsyncWrapperScalesControllerSetTargets(AsyncWrapper<double, double> &controller);

} // namespace okapi

#endif
