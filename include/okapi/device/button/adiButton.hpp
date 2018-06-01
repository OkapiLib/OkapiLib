/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_ADIBUTTON_HPP_
#define _OKAPI_ADIBUTTON_HPP_

#include "okapi/device/button/button.hpp"

namespace okapi {
class ADIButton : public AbstractButton {
  public:
  ADIButton(const std::uint8_t iport, const bool iinverted = false);

  protected:
  pros::ADIButton btn;
  std::uint8_t port;

  virtual bool currentlyPressed() override;
};
} // namespace okapi

#endif
