/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */
 #include "okapi/device/quadEncoder.hpp"

namespace okapi {
  QuadEncoder::QuadEncoder(const uint8_t iportTop, const uint8_t iportBottom,
    const bool ireversed):
    enc(iportBottom, iportTop, ireversed) {}

  int32_t QuadEncoder::get() {
    return enc.value_get();
  }

  int32_t QuadEncoder::reset() {
    return enc.reset();
  }
}