/*
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "api.h"
#include "okapi/api/device/button/buttonBase.hpp"

namespace okapi {
class ADIButton : public ButtonBase {
  public:
  /**
   * A button in an ADI port.
   *
   * @param iport The ADI port number.
   * @param iinverted Whether the button is inverted (`true` meaning default pressed and `false`
   * meaning default not pressed).
   */
  ADIButton(std::uint8_t iport, bool iinverted = false);

  protected:
  std::uint8_t port;

  virtual bool currentlyPressed() override;
};
} // namespace okapi
