/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_IMPLMOCKS_HPP_
#define _OKAPI_IMPLMOCKS_HPP_

#include "okapi/api/control/util/settledUtil.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/api/util/abstractTimer.hpp"
#include <chrono>

namespace okapi {
/**
 * A motor mock that saves the last set position, velocity, and voltage.
 */
class MockMotor : public AbstractMotor {
  public:
  void controllerSet(const double ivalue) override {
  }

  int32_t moveAbsolute(const double iposition, const std::int32_t ivelocity) const override {
    lastPosition = (int16_t)iposition;
    return 0;
  }

  int32_t moveRelative(const double iposition, const std::int32_t ivelocity) const override {
    lastPosition += iposition;
    return 0;
  }

  double getTargetPosition() const override {
    return 0;
  }

  double getPosition() const override {
    return 0;
  }

  int32_t getTargetVelocity() const override {
    return 0;
  }

  double getActualVelocity() const override {
    return 0;
  }

  int32_t tarePosition() const override {
    return 0;
  }

  int32_t setBrakeMode(const brakeMode imode) const override {
    return 0;
  }

  int32_t setCurrentLimit(const std::int32_t ilimit) const override {
    return 0;
  }

  int32_t setEncoderUnits(const encoderUnits iunits) const override {
    return 0;
  }

  int32_t setGearing(const gearset igearset) const override {
    return 0;
  }

  int32_t setReversed(const bool ireverse) const override {
    return 0;
  }

  int32_t setVoltageLimit(const std::int32_t ilimit) const override {
    return 0;
  }

  std::shared_ptr<ContinuousRotarySensor> getEncoder() const override {
    return std::shared_ptr<ContinuousRotarySensor>();
  }

  std::int32_t moveVelocity(const std::int16_t ivelocity) const override {
    lastVelocity = ivelocity;
    return 1;
  }

  std::int32_t moveVoltage(const std::int16_t ivoltage) const override {
    lastVoltage = ivoltage;
    return 1;
  }

  mutable std::int16_t lastVelocity{};
  mutable std::int16_t lastVoltage{};
  mutable std::int16_t lastPosition{};
};

/**
 * A timer mock that implements all features using the system timer.
 */
class MockTimer : public AbstractTimer {
  public:
  MockTimer() : firstCalled(millis()), lastCalled(firstCalled), mark(firstCalled) {
  }

  ~MockTimer() override = default;

  QTime millis() const override {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::high_resolution_clock::now() - epoch)
             .count() *
           millisecond;
  }

  QTime getDt() override {
    const QTime currTime = millis();
    const QTime dt = currTime - lastCalled;
    lastCalled = currTime;
    return dt;
  }

  QTime getStartingTime() const override {
    return firstCalled;
  }

  QTime getDtFromStart() const override {
    return millis() - firstCalled;
  }

  void placeMark() override {
    mark = millis();
  }

  void placeHardMark() override {
    if (hardMark == 0_ms)
      hardMark = millis();
  }

  QTime clearHardMark() override {
    const QTime old = hardMark;
    hardMark = 0_ms;
    return old;
  }

  QTime getDtFromMark() const override {
    return millis() - mark;
  }

  QTime getDtFromHardMark() const override {
    return hardMark == 0_ms ? 0_ms : millis() - hardMark;
  }

  bool repeat(const QTime time) override {
    if (repeatMark == 0_ms) {
      repeatMark = millis();
      return false;
    }

    if (millis() - repeatMark >= time) {
      repeatMark = 0_ms;
      return true;
    }

    return false;
  }

  bool repeat(const QFrequency frequency) override {
    return repeat(QTime(1 / frequency.convert(Hz)));
  }

  std::chrono::system_clock::time_point epoch = std::chrono::high_resolution_clock::from_time_t(0);
  QTime firstCalled;
  QTime lastCalled;
  QTime mark;
  QTime hardMark;
  QTime repeatMark;
};

/**
 * A timer mock that always returns a constant dt and 0 for other methods.
 */
class ConstantMockTimer : public AbstractTimer {
  public:
  explicit ConstantMockTimer(const QTime idt) : dtToReturn(idt) {
  }

  ~ConstantMockTimer() override = default;

  QTime millis() const override {
    return 0_ms;
  }

  QTime getDt() override {
    return dtToReturn;
  }

  QTime getStartingTime() const override {
    return 0_ms;
  }

  QTime getDtFromStart() const override {
    return dtToReturn;
  }

  void placeMark() override {
  }

  void placeHardMark() override {
  }

  QTime clearHardMark() override {
    return 0_ms;
  }

  QTime getDtFromMark() const override {
    return dtToReturn;
  }

  QTime getDtFromHardMark() const override {
    return dtToReturn;
  }

  bool repeat(QTime time) override {
    return false;
  }

  bool repeat(QFrequency frequency) override {
    return false;
  }

  QTime dtToReturn;
};

std::unique_ptr<SettledUtil> createSettledUtilPtr(const double iatTargetError = 50,
                                                  const double iatTargetDerivative = 5,
                                                  const QTime iatTargetTime = 250_ms) {
  return std::make_unique<SettledUtil>(std::make_unique<MockTimer>(), iatTargetError,
                                       iatTargetDerivative, iatTargetTime);
}
} // namespace okapi

#endif
