/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef OKAPI_CHASSISMODEL_HPP_
#define OKAPI_CHASSISMODEL_HPP_

#include <array>
#include <initializer_list>
#include <valarray>
#include <memory>
#include "okapi/device/quadEncoder.hpp"

namespace okapi {
  class ChassisModel {
  public:
    ChassisModel() {}
    virtual ~ChassisModel() = default;
    virtual void driveForward(const int power) = 0;
    virtual void driveVector(const int distPower, const int anglePower) = 0;
    virtual void turnClockwise(const int power) = 0;
    virtual void stop() = 0;
    virtual void tank(const int leftVal, const int rightVal, const int threshold = 0) = 0;
    virtual void arcade(int verticalVal, int horizontalVal, const int threshold = 0) = 0;
    virtual void left(const int val) = 0;
    virtual void leftTS(const int val) = 0;
    virtual void right(const int val) = 0;
    virtual void rightTS(const int val) = 0;
    virtual std::valarray<int> getSensorVals() = 0;
    virtual void resetSensors() = 0;
  };

  class ChassisModelParams {
  public:
    ChassisModelParams() {}
    virtual ~ChassisModelParams() = default;

    /**
     * Allocates a new ChassisModel
     * @return shared_ptr pointing to new ChassisModel
     */
    virtual std::shared_ptr<ChassisModel> make() const = 0;
  };

  template<size_t motorsPerSide>
  class SkidSteerModel;

  template<size_t motorsPerSide>
  class SkidSteerModelParams : public ChassisModelParams {
  public:
    SkidSteerModelParams(const std::array<pros::Motor, motorsPerSide * 2>& imotorList, const QuadEncoder& ileftEnc, const QuadEncoder& irightEnc):
      motorList(imotorList),
      leftSensor(std::make_shared<QuadEncoder>(ileftEnc)),
      rightSensor(std::make_shared<QuadEncoder>(irightEnc)) {}

    virtual ~SkidSteerModelParams() = default;

    std::shared_ptr<ChassisModel> make() const override {
      return std::make_shared<SkidSteerModel<motorsPerSide>>(*this);
    }

    const std::array<pros::Motor, motorsPerSide * 2>& motorList;
    std::shared_ptr<RotarySensor> leftSensor, rightSensor;
  };

  template<size_t motorsPerSide>
  class SkidSteerModel : public ChassisModel {
  public:
    /**
     * Model for a skid steer drive (wheels parallel with robot's direction of
     * motion). When all motors are powered +127, the robot should move forward
     * in a straight line at full speed.
     * @param imotors   Motors in the format: {{left side motors}, {right side motors}}
     * @param ileftEnc  Left side encoder
     * @param irightEnc Right side encoder
     */
    SkidSteerModel(const std::array<pros::Motor, motorsPerSide * 2>& imotorList, const QuadEncoder& ileftEnc, const QuadEncoder& irightEnc):
      motors(imotorList),
      leftSensor(std::make_shared<QuadEncoder>(ileftEnc)),
      rightSensor(std::make_shared<QuadEncoder>(irightEnc)) {}

    SkidSteerModel(const SkidSteerModelParams<motorsPerSide>& iparams):
      motors(iparams.motorList),
      leftSensor(iparams.leftSensor),
      rightSensor(iparams.rightSensor) {}

    SkidSteerModel(const SkidSteerModel<motorsPerSide>& other):
      motors(other.motors),
      leftSensor(other.leftSensor),
      rightSensor(other.rightSensor) {}

    virtual ~SkidSteerModel() { delete &motors; }

    void driveForward(const int power) override {
      for (size_t i = 0; i < motorsPerSide * 2; i++)
        motors[i].set_velocity(power);
    }

    void driveVector(const int distPower, const int anglePower) override {
      for (size_t i = 0; i < motorsPerSide; i++)
        motors[i].set_velocity(distPower + anglePower);
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        motors[i].set_velocity(distPower - anglePower);
    }

    void turnClockwise(const int power) override {
      for (size_t i = 0; i < motorsPerSide; i++)
        motors[i].set_velocity(power);
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        motors[i].set_velocity(-1 * power);
    }

    void stop() override {
      for (size_t i = 0; i < motorsPerSide * 2; i++)
        motors[i].set_velocity(0);
    }

    void tank(const int leftVal, const int rightVal, const int threshold = 0) override {
      using namespace std;

      if (fabs(leftVal) < threshold) {
        for (size_t i = 0; i < motorsPerSide; i++)
          motors[i].set_velocity(0);
      } else {
        for (size_t i = 0; i < motorsPerSide; i++)
          motors[i].set_velocity(leftVal);
      }
      
      if (fabs(rightVal) < threshold) {
        for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
          motors[i].set_velocity(0);
      } else {
        for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
          motors[i].set_velocity(rightVal);
      }
    }

    void arcade(int verticalVal, int horizontalVal, const int threshold = 0) override {
      using namespace std;
      
      if (fabs(verticalVal) < threshold)
        verticalVal = 0;
      if (fabs(horizontalVal) < threshold)
        horizontalVal = 0;
      
      for (size_t i = 0; i < motorsPerSide; i++)
        motors[i].set_velocity(verticalVal + horizontalVal);
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        motors[i].set_velocity(verticalVal - horizontalVal);
    }

    void left(const int val) override {
      for (size_t i = 0; i < motorsPerSide; i++)
        motors[i].set_velocity(val);
    }
    
    void leftTS(const int val) override {
      for (size_t i = 0; i < motorsPerSide; i++)
        motors[i].set_velocity(val);
    }

    void right(const int val) override {
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        motors[i].set_velocity(val);
    }
    
    void rightTS(const int val) override {
      for (size_t i = motorsPerSide; i < motorsPerSide * 2; i++)
        motors[i].set_velocity(val);
    }

    std::valarray<int> getSensorVals() override {
      return std::valarray<int>{leftSensor->get(), rightSensor->get()};
    }

    void resetSensors() override {
      leftSensor->reset();
      rightSensor->reset();
    }

  private:
    std::array<pros::Motor, motorsPerSide * 2> motors;
    std::shared_ptr<RotarySensor> leftSensor, rightSensor;
  };

  template<size_t motorsPerCorner>
  class XDriveModel;

  template<size_t motorsPerCorner>
  class XDriveModelParams : public ChassisModelParams {
  public:
    XDriveModelParams(const std::array<pros::Motor, motorsPerCorner * 4>& imotorList, const QuadEncoder& ileftEnc, const QuadEncoder& irightEnc):
      motorList(imotorList),
      leftSensor(std::make_shared<QuadEncoder>(ileftEnc)),
      rightSensor(std::make_shared<QuadEncoder>(irightEnc)) {}

    virtual ~XDriveModelParams() {}

    std::shared_ptr<ChassisModel> make() const override {
      return std::make_shared<XDriveModel<motorsPerCorner>>(*this);
    }

    const std::array<pros::Motor, motorsPerCorner * 4>& motorList;
    std::shared_ptr<RotarySensor> leftSensor, rightSensor;
  };

  template<size_t motorsPerCorner>
  class XDriveModel : public ChassisModel {
  public:
    /**
     * Model for an x drive (wheels at 45 deg from a skid steer drive). When all
     * motors are powered +127, the robot should move forward in a straight line
     * at full speed.
     * @param imotors Motors in the format: {{top left motors}, {top right motors}, {bottom right motors}, {bottom left motors}}
     * @param ileftEnc Left side encoder
     * @param irightEnc Right side encoder
     */
    XDriveModel(const std::array<pros::Motor, motorsPerCorner * 4>& imotorList, const QuadEncoder& ileftEnc, const QuadEncoder& irightEnc):
      motors(imotorList),
      leftSensor(std::make_shared<QuadEncoder>(ileftEnc)),
      rightSensor(std::make_shared<QuadEncoder>(irightEnc)) {}

    XDriveModel(const XDriveModelParams<motorsPerCorner>& iparams):
      motors(iparams.motorList),
      leftSensor(iparams.leftSensor),
      rightSensor(iparams.rightSensor) {}

    XDriveModel(const XDriveModel<motorsPerCorner>& other):
      motors(other.motors),
      leftSensor(other.leftSensor),
      rightSensor(other.rightSensor) {}

    virtual ~XDriveModel() { delete &motors; }

    void driveForward(const int power) override {
      for (size_t i = 0; i < motorsPerCorner * 4; i++)
        motors[i].set_velocity(power);
    }

    void driveVector(const int distPower, const int anglePower) override {
      for (size_t i = 0; i < motorsPerCorner; i++)
        motors[i].set_velocity(distPower + anglePower);
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
        motors[i].set_velocity(distPower - anglePower);
      for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
        motors[i].set_velocity(distPower - anglePower);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motors[i].set_velocity(distPower + anglePower);
    }

    void turnClockwise(const int power) override {
      for (size_t i = 0; i < motorsPerCorner; i++)
        motors[i].set_velocity(power);
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
        motors[i].set_velocity(-1 * power);
      for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
        motors[i].set_velocity(-1 * power);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motors[i].set_velocity(power);
    }

    void stop() override {
      for (size_t i = 0; i < motorsPerCorner * 4; i++)
        motors[i].set_velocity(0);
    }

    void tank(const int leftVal, const int rightVal, const int threshold = 0) override {
      using namespace std;

      if (fabs(leftVal) < threshold) {
        for (size_t i = 0; i < motorsPerCorner; i++)
          motors[i].set_velocity(0);
        for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
          motors[i].set_velocity(0);
      } else {
        for (size_t i = 0; i < motorsPerCorner; i++)
          motors[i].set_velocity(leftVal);
        for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
          motors[i].set_velocity(leftVal);
      }
      
      if (fabs(rightVal) < threshold) {
        for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
          motors[i].set_velocity(0);
        for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
          motors[i].set_velocity(0);
      } else {
        for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
          motors[i].set_velocity(rightVal);
        for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
          motors[i].set_velocity(rightVal);
      }
    }

    void arcade(int verticalVal, int horizontalVal, const int threshold = 0) override {
      using namespace std;
      
      if (fabs(verticalVal) < threshold)
        verticalVal = 0;
      if (fabs(horizontalVal) < threshold)
        horizontalVal = 0;
      
      for (size_t i = 0; i < motorsPerCorner; i++)
        motors[i].set_velocity(verticalVal + horizontalVal);
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
        motors[i].set_velocity(verticalVal - horizontalVal);
      for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
        motors[i].set_velocity(verticalVal - horizontalVal);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motors[i].set_velocity(verticalVal + horizontalVal);
    }

    void xArcade(int verticalVal, int horizontalVal, int rotateVal, const int threshold = 0) {
      using namespace std;
      
      if (fabs(verticalVal) < threshold)
        verticalVal = 0;
      if (fabs(horizontalVal) < threshold)
        horizontalVal = 0;
      if (fabs(rotateVal) < threshold)
        rotateVal = 0;
      
      for (size_t i = 0; i < motorsPerCorner; i++)
        motors[i].set_velocity(verticalVal + horizontalVal + rotateVal);
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 2; i++)
        motors[i].set_velocity(verticalVal - horizontalVal - rotateVal);
      for (size_t i = motorsPerCorner * 2; i < motorsPerCorner * 3; i++)
        motors[i].set_velocity(verticalVal + horizontalVal - rotateVal);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motors[i].set_velocity(verticalVal - horizontalVal + rotateVal);
    }
    
    void left(const int val) override {
      for (size_t i = 0; i < motorsPerCorner; i++)
        motors[i].set_velocity(val);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motors[i].set_velocity(val);
    }
    
    void leftTS(const int val) override {
      for (size_t i = 0; i < motorsPerCorner; i++)
        motors[i].set_velocity(val);
      for (size_t i = motorsPerCorner * 3; i < motorsPerCorner * 4; i++)
        motors[i].set_velocity(val);
    }

    void right(const int val) override {
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 3; i++)
        motors[i].set_velocity(val);
    }
    
    void rightTS(const int val) override {
      for (size_t i = motorsPerCorner; i < motorsPerCorner * 3; i++)
        motors[i].set_velocity(val);
    }

    std::valarray<int> getSensorVals() override {
      return std::valarray<int>{leftSensor->get(), rightSensor->get()};
    }

    void resetSensors() override {
      leftSensor->reset();
      rightSensor->reset();
    }

  private:
    std::array<pros::Motor, motorsPerCorner * 4> motors;
    std::shared_ptr<RotarySensor> leftSensor, rightSensor;
  };
}

#endif
