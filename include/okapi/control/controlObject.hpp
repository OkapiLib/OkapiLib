/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_CONTORLOBJECT_HPP_
#define _OKAPI_CONTORLOBJECT_HPP_

namespace okapi {
  class ControlObjectParams {};

  class ControlObject {
  public:
    /**
     * Do one iteration of the controller.
     *
     * @param inewReading new measurement
     * @return controller output
     */
    virtual float step(const float ireading) = 0;

    /**
     * Sets the target for the controller.
     */
    virtual void setTarget(const float itarget) = 0;

    /**
     * Returns the last calculated output of the controller.
     */
    virtual float getOutput() const = 0;

    /**
     * Returns the last error of the controller.
     */
    virtual float getError() const = 0;

    /**
     * Set time between loops in ms.
     *
     * @param isampleTime time between loops in ms
     */
    virtual void setSampleTime(const int isampleTime) {}

    /**
     * Set controller output bounds.
     *
     * @param imax max output
     * @param imin min output
     */
    virtual void setOutputLimits(float imax, float imin) {}

    /**
     * Resets the controller so it can start from 0 again properly. Keeps configuration from
     * before.
     */
    virtual void reset() {}

    /**
     * Change whether the controll is off or on.
     */
    virtual void flipDisable() {}
  };
}

#endif
