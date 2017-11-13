#ifndef OKAPI_GENERICCONTROLLER
#define OKAPI_GENERICCONTROLLER

#include <memory>
#include <array>
#include "control/controlObject.h"
#include "device/motor.h"

namespace okapi {
  template<size_t motorNum>
  class GenericController {
  public:
    GenericController(const std::array<Motor, motorNum> &imotorList, const std::shared_ptr<ControlObject> &iptr):
      motors(imotorList),
      controller(iptr) {}

    void step(const float ireading) {
      controller->step(ireading);
      for (size_t i = 0; i < motors.size(); i++)
        motors[i].setTS(static_cast<int>(controller->getOutput()));
    }

    void setTarget(const float itarget) { controller->setTarget(itarget); }

    void getOutput() const { return controller->getOutput(); }

    void setSampleTime(const int isampleTime) { controller->setSampleTime(isampleTime); }

    void setOutputLimits(float imax, float imin) { controller->setOutputLimits(imax, imin); }

    void reset() { controller->reset(); }

    void flipDisable() { controller->flipDisable(); }
  private:
    std::array<Motor, motorNum> motors;
    std::shared_ptr<ControlObject> controller;
  };
}

#endif /* OKAPI_GENERICCONTROLLER */
