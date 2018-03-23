/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ODOMETRY_HPP_
#define _OKAPI_ODOMETRY_HPP_

#include "okapi/chassis/model/chassisModel.hpp"
#include <memory>
#include <valarray>

namespace okapi {
class OdomState {
  public:
  OdomState(const double ix, const double iy, const double itheta);

  OdomState();

  virtual ~OdomState();

  double x, y, theta;
};

class OdometryParams {
  public:
  OdometryParams(const ChassisModelParams &iparams, const double iscale, const double iturnScale);

  virtual ~OdometryParams();

  std::shared_ptr<const ChassisModel> model;
  double scale, turnScale;
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
  Odometry(const ChassisModelParams &imodelParams, const double iscale, const double iturnScale);

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
  void setScales(const double iscale, const double iturnScale);

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
  OdomState getState() const;

  /**
   * Set a new state to be the current state.
   *
   * @param istate new state
   */
  void setState(const OdomState &istate);

  private:
  std::shared_ptr<const ChassisModel> model;
  OdomState state;
  double scale, turnScale;
  std::valarray<int> lastTicks;
  double mm;
};
} // namespace okapi

#endif
