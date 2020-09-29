/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "api.h"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"
#include "okapi/api/util/logging.hpp"

namespace okapi {
class ADIEncoder : public ContinuousRotarySensor {
  public:
  /**
   * An encoder in an ADI port.
   *
   * @param iportTop The "top" wire from the encoder with the removable cover side up. This must be
   * in port ``1``, ``3``, ``5``, or ``7`` (``A``, ``C``, ``E``, or ``G``).
   * @param iportBottom The "bottom" wire from the encoder. This must be in port ``2``, ``4``,
   * ``6``, or ``8`` (``B``, ``D``, ``F``, or ``H``).
   * @param ireversed Whether the encoder is reversed.
   */
  ADIEncoder(std::uint8_t iportTop, std::uint8_t iportBottom, bool ireversed);

  /**
   * An encoder in an ADI port.
   *
   * @param ismartPort The smart port the ADI Expander is in.
   * @param iportTop The "top" wire from the encoder with the removable cover side up. This must be
   * in port ``1``, ``3``, ``5``, or ``7`` (``A``, ``C``, ``E``, or ``G``).
   * @param iportBottom The "bottom" wire from the encoder. This must be in port ``2``, ``4``,
   * ``6``, or ``8`` (``B``, ``D``, ``F``, or ``H``).
   * @param ireversed Whether the encoder is reversed.
   * @param logger The logger that initialization warnings will be logged to.
   */
  ADIEncoder(std::uint8_t ismartPort,
             std::uint8_t iportTop,
             std::uint8_t iportBottom,
             bool ireversed,
             const std::shared_ptr<Logger> &logger = Logger::getDefaultLogger());

  /**
   * Get the current sensor value.
   *
   * @return the current sensor value, or `PROS_ERR` on a failure.
   */
  virtual double get() const override;

  /**
   * Reset the sensor to zero.
   *
   * @return `1` on success, `PROS_ERR` on fail
   */
  virtual std::int32_t reset() override;

  /**
   * Get the sensor value for use in a control loop. This method might be automatically called in
   * another thread by the controller.
   *
   * @return the current sensor value, or `PROS_ERR` on a failure.
   */
  virtual double controllerGet() override;

  protected:
  pros::c::ext_adi_encoder_t enc;
};
} // namespace okapi
