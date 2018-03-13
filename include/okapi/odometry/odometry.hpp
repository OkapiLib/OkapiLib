/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_ODOMETRY_HPP_
#define _OKAPI_ODOMETRY_HPP_

#include "okapi/chassis/chassisModel.hpp"
#include <memory>
#include <valarray>

namespace okapi {
class OdomState {
  public:
  OdomState(const float ix, const float iy, const float itheta) : x(ix), y(iy), theta(itheta) {
  }

  OdomState() : x(0), y(0), theta(0) {
  }

  virtual ~OdomState() = default;

  float x, y, theta;
};

class OdometryParams {
  public:
  OdometryParams(const ChassisModelParams &iparams, const float iscale, const float iturnScale)
    : model(iparams.make()), scale(iscale), turnScale(iturnScale) {
  }

  virtual ~OdometryParams() = default;

  const ChassisModel &model;
  float scale, turnScale;
};

class Odometry {
  public:
  /**
   * Odometry. Tracks the movement of the robot and estimates its position in coordinates
   * relative to the start (assumed to be (0, 0)).
   *
   * @param imodelParams ChassisModel for reading sensors
   * @param iscale straight scale
   * @param iturnScale turn scale
   */
  Odometry(const ChassisModelParams &imodelParams, const float iscale, const float iturnScale);

  /**
   * Odometry. Tracks the movement of the robot and estimates its position in coordinates
   * relative to the start (assumed to be (0, 0)).
   *
   * @param iparams OdometryParams
   */
  Odometry(const OdometryParams &iparams);

  virtual ~Odometry();

  /**
   * Set the drive and turn scales.
   *
   * @param iscale straight scale converting encoder ticks to mm
   * @param iturnScale turn scale converting encoder ticks to radians
   */
  void setScales(const float iscale, const float iturnScale);

  /**
   * Do odometry math in an infinite loop.
   */
  void loop();

  /**
   * Tread the input as an Odometry pointer and call loop. Meant to be used to bounce into a
   * thread because loop runs forever.
   *
   * @param context pointer to an Odometry object
   */
  static void trampoline(void *context);

  /**
   * Get the current state.
   *
   * @return current state
   */
  OdomState getState();

  private:
  const ChassisModel &model;
  OdomState state;
  float scale, turnScale;
  std::valarray<int> lastTicks;
  float mm;
};
} // namespace okapi

#endif
