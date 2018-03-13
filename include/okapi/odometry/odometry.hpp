/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_ODOMETRY_HPP_
#define _OKAPI_ODOMETRY_HPP_

#include "okapi/chassis/chassisModel.hpp"
#include <valarray>
#include <memory>

namespace okapi {
  class OdomState {
  public:
    OdomState(const float ix, const float iy, const float itheta):
      x(ix),
      y(iy),
      theta(itheta) {}

    OdomState():
      x(0),
      y(0),
      theta(0) {}

    virtual ~OdomState() = default;

    float x, y, theta;
  };

  class OdometryParams {
  public:
    OdometryParams(const ChassisModelParams& iparams, const float iscale, const float iturnScale):
      model(iparams.make()),
      scale(iscale),
      turnScale(iturnScale) {}

    virtual ~OdometryParams() = default;

    std::shared_ptr<ChassisModel> model;
    float scale, turnScale;
  };

  class Odometry {
  public:
    Odometry(const ChassisModelParams& imodelParams, const float iscale, const float iturnScale):
      model(imodelParams.make()),
      scale(iscale),
      turnScale(iturnScale),
      lastTicks{0, 0},
      mm(0) {}

    Odometry(const OdometryParams& iparams):
      model(iparams.model),
      scale(iparams.scale),
      turnScale(iparams.turnScale),
      lastTicks{0, 0},
      mm(0) {}

    /**
     * Sets the parameters for Odometry math.
     * 
     * @param iparams Odometry parameters
     */
    void setParams(OdometryParams& iparams) {
      model = iparams.model;
      scale = iparams.scale;
      turnScale = iparams.turnScale;
    }

    /**
     * Set the drive and turn scales.
     * 
     * @param iscale     Scale converting encoder ticks to mm
     * @param iturnScale Scale converting encoder ticks to radians
     */
    void setScales(const float iscale, const float iturnScale) {
      scale = iscale;
      turnScale = iturnScale;
    }

    /**
     * Do odom math in an infinite loop.
     */
    void loop();

    static void trampoline(void *context) { static_cast<Odometry*>(context)->loop(); }

    OdomState getState() { return state; }
  private:
    std::shared_ptr<ChassisModel> model;
    OdomState state;
    float scale, turnScale;
    std::valarray<int> lastTicks;
    float mm;
  };
}

#endif
