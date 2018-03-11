/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
#ifndef _OKAPI_QUADENCODER_HPP_
#define _OKAPI_QUADENCODER_HPP_

#include "api.h"
#include "okapi/device/rotarySensor.hpp"

namespace okapi {
  class QuadEncoder : public RotarySensor {
  public:
    QuadEncoder(const uint8_t iportTop, const uint8_t iportBottom, const bool ireversed = false);

    int32_t get() override;

    int32_t reset() override;

  private:
    pros::ADIEncoder enc;
  };
}

#endif
