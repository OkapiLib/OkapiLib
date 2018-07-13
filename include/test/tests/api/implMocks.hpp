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
#include "okapi/api/util/abstractRate.hpp"
#include "okapi/api/util/abstractTimer.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include <chrono>

namespace okapi {
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

  int32_t setBrakeMode(brakeMode imode) const override;

  int32_t setCurrentLimit(std::int32_t ilimit) const override;

  int32_t setEncoderUnits(encoderUnits iunits) const override;

  int32_t setGearing(gearset igearset) const override;

  int32_t setReversed(bool ireverse) const override;

  int32_t setVoltageLimit(std::int32_t ilimit) const override;

  std::shared_ptr<ContinuousRotarySensor> getEncoder() const override;

  std::int32_t moveVelocity(std::int16_t ivelocity) const override;

  std::int32_t moveVoltage(std::int16_t ivoltage) const override;

  mutable std::int16_t lastVelocity{};
  mutable std::int16_t lastVoltage{};
  mutable std::int16_t lastPosition{};
};

/**
 * A timer mock that implements all features using the system timer.
 */
class MockTimer : public AbstractTimer {
  public:
  MockTimer();

  QTime millis() const override;

  QTime getDt() override;

  QTime getStartingTime() const override;

  QTime getDtFromStart() const override;

  void placeMark() override;

  void placeHardMark() override;

  QTime clearHardMark() override;

  QTime getDtFromMark() const override;

  QTime getDtFromHardMark() const override;

  bool repeat(QTime time) override;

  bool repeat(QFrequency frequency) override;

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
  explicit ConstantMockTimer(QTime idt);

  QTime millis() const override;

  QTime getDt() override;

  QTime getStartingTime() const override;

  QTime getDtFromStart() const override;

  void placeMark() override;

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

TimeUtil createTimeUtil(const Supplier<std::unique_ptr<AbstractTimer>> &itimerSupplier);
} // namespace okapi

#endif
