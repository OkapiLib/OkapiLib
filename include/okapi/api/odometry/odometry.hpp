/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ODOMETRY_HPP_
#define _OKAPI_ODOMETRY_HPP_

#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/chassis/model/readOnlyChassisModel.hpp"
#include "okapi/api/util/abstractRate.hpp"
#include <memory>
#include <valarray>

namespace okapi {
class OdomState {
  public:
  OdomState();
  OdomState(double ix, double iy, double itheta);

  virtual ~OdomState();

  double x = 0;
  double y = 0;
  double theta = 0;
};

class OdometryArgs {
  public:
  OdometryArgs(std::shared_ptr<ReadOnlyChassisModel> imodel, const ChassisScales &ichassisScales);

  virtual ~OdometryArgs();

  std::shared_ptr<ReadOnlyChassisModel> model;
  ChassisScales chassisScales;
};

class Odometry {
  public:
  /**
   * Odometry. Tracks the movement of the robot and estimates its position in coordinates
   * relative to the start (assumed to be (0, 0)).
   *
   * @param imodel SkidSteerModel for reading sensors
   * @param iscale straight scale
   * @param iturnScale turn scale
   */
  Odometry(std::shared_ptr<ReadOnlyChassisModel> imodel, const ChassisScales &ichassisScales,
           std::unique_ptr<AbstractRate> irate);

  /**
   * Odometry. Tracks the movement of the robot and estimates its position in coordinates
   * relative to the start (assumed to be (0, 0)).
   *
   * @param iparams OdometryArgs
   */
  explicit Odometry(const OdometryArgs &iparams);

  virtual ~Odometry();

  /**
   * Sets the drive and turn scales.
   */
  virtual void setScales(const ChassisScales &ichassisScales);

  /**
   * Do odometry math in an infinite loop.
   */
  virtual void loop();

  /**
   * Do one odometry step. Do not use this and loop in combination, only one may be used per
   * Odometry object. It is recommended to use loop.
   */
  virtual void step();

  /**
   * Treat the input as an Odometry pointer and call loop. Meant to be used to bounce into a
   * thread because loop runs forever.
   *
   * @param context pointer to an Odometry object
   */
  static void trampoline(void *context);

  /**
   * Returns the current state.
   *
   * @return current state
   */
  virtual OdomState getState() const;

  /**
   * Sets a new state to be the current state.
   *
   * @param istate new state
   */
  virtual void setState(const OdomState &istate);

  protected:
  std::shared_ptr<ReadOnlyChassisModel> model;
  std::unique_ptr<AbstractRate> rate;
  OdomState state;
  ChassisScales chassisScales;
  std::valarray<std::int32_t> newTicks{0, 0}, tickDiff{0, 0}, lastTicks{0, 0};
  double mm = 0;
};
} // namespace okapi

#endif
