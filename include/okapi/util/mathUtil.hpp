/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_MATHUTIL_HPP_
#define _OKAPI_MATHUTIL_HPP_

namespace okapi {
static constexpr double analogInToV = 286.0;
static constexpr double inchToMM = 25.4;
static constexpr double degreeToRadian = 0.01745;
static constexpr double radianToDegree = 57.2958;
static constexpr double imeTorqueTPR = 627.2;
static constexpr double imeSpeedTPR = 392.0;
static constexpr double imeTurboTPR = 261.333;
static constexpr double ime269TPR = 240.448;
static constexpr double imev5TPR = 1800.0;
static constexpr double quadEncoderTPR = 360.0;
static constexpr double pi = 3.14159265358979323846;

/**
 * Integer power function. Computes base^expo.
 *
 * @param base base
 * @param expo exponent
 * @return base^expo
 */
constexpr double ipow(const double base, const int expo) {
  return (expo == 0)
           ? 1
           : expo == 1 ? base
                       : expo > 1 ? ((expo & 1) ? base * ipow(base, expo - 1)
                                                : ipow(base, expo / 2) * ipow(base, expo / 2))
                                  : 1 / ipow(base, -expo);
}
} // namespace okapi

#endif
