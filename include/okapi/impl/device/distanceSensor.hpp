/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "api.h"
#include "okapi/api/control/controllerInput.hpp"
#include "okapi/api/filter/passthroughFilter.hpp"
#include <memory>

namespace okapi {
class DistanceSensor : public ControllerInput<double> {
  public:
  /**
   * A distance sensor on a V5 port.
   *
   * ```cpp
   * auto ds = DistanceSensor(1);
   * auto filteredDistSensor = DistanceSensor(1, std::make_unique<MedianFilter<5>>());
   * ```
   *
   * @param iport The V5 port the device uses.
   * @param ifilter The filter to use for filtering the distance measurements.
   */
  DistanceSensor(std::uint8_t iport,
                 std::unique_ptr<Filter> ifilter = std::make_unique<PassthroughFilter>());

  virtual ~DistanceSensor();

  /**
   * Returns the current filtered sensor value in mm.
   *
   * @return current value.
   */
  virtual double get();

  /**
   * Get the sensor value for use in a control loop. This method might be automatically called in
   * another thread by the controller. Calls get().
   */
  virtual double controllerGet() override;

  /**
   * Get the confidence in the distance reading.
   *
   * This is a value that has a range of 0 to 63. 63 means high confidence, lower values imply less
   * confidence. Confidence is only available when distance is > 200mm.
   *   *
   * @return The confidence value.
   */
  std::int32_t getConfidence() const;

  /**
   * Get the current guess at relative object size.
   *
   * This is a value that has a range of 0 to 400. A 18" x 30" grey card will return a 
   * value of approximately 75 in typical room lighting.
   *
   * @return The size value or PROS_ERR if the operation failed, setting
   * errno.
   */
  std::int32_t getObjectSize();

  /**
   * Get the object velocity in m/s.
   *
   * @return The velocity value.
   */
  double getObjectVelocity();

  protected:
  std::uint8_t port;
  std::unique_ptr<Filter> filter;
};
} // namespace okapi
