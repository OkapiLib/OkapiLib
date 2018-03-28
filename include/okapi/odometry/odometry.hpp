/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ODOMETRY_HPP_
#define _OKAPI_ODOMETRY_HPP_

#include "okapi/chassis/model/skidSteerModel.hpp"
#include <memory>
#include <valarray>

namespace okapi {
class OdomState {
  public:
  OdomState();
  OdomState(const double ix, const double iy, const double itheta);

  virtual ~OdomState();

  double x = 0;
  double y = 0;
  double theta = 0;
};

class OdometryParams {
  public:
  OdometryParams(const SkidSteerModel &iparams, const double iscale, const double iturnScale);

  virtual ~OdometryParams();

  const SkidSteerModel &model;
  const double scale, turnScale;
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
  Odometry(const SkidSteerModel &imodel, const double iscale, const double iturnScale);

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
  virtual void setScales(const double iscale, const double iturnScale);

  /**
   * Do odometry math in an infinite loop.
   */
  virtual void loop();

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
  virtual OdomState getState() const;

  /**
   * Set a new state to be the current state.
   *
   * @param istate new state
   */
  virtual void setState(const OdomState &istate);

  protected:
  OdomState state;
  double scale, turnScale;
  std::valarray<int> lastTicks{0, 0};
  double mm = 0;

  private:
  const SkidSteerModel &model;
};
} // namespace okapi

#endif
