/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include <algorithm>
#include <cstdint>
#include <type_traits>

namespace okapi {
static constexpr double analogInToV = 286.0;
static constexpr double inchToMM = 25.4;
static constexpr double mmToInch = 0.0393700787;
static constexpr double degreeToRadian = 0.01745;
static constexpr double radianToDegree = 57.2957795;
static constexpr double imeTorqueTPR = 627.2;
static constexpr double imeSpeedTPR = 392.0;
static constexpr double imeTurboTPR = 261.333;
static constexpr double ime269TPR = 240.448;
static constexpr double imev5TPR = 1800.0;
static constexpr double quadEncoderTPR = 360.0;
static constexpr double pi = 3.1415926535897932;
static constexpr double pi2 = 1.5707963267948966;
static constexpr double gravity = 9.80665;

static constexpr std::int8_t motorUpdateRate = 10;
static constexpr std::int8_t adiUpdateRate = 50;

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

/**
 * Cuts out a range from the number. The new range of the input number will be
 * (-inf, min]U[max, +inf). If value sits equally between min and max, max will be returned.
 *
 * @param value number to bound
 * @param min lower bound of range
 * @param max upper bound of range
 * @return the remapped value
 */
constexpr double cutRange(const double value, const double min, const double max) {
  const double middle = max - ((max - min) / 2);

  if (value > min && value < middle) {
    return min;
  } else if (value <= max && value >= middle) {
    return max;
  }

  return value;
}

/**
 * Deadbands a range of the number. Returns the input value, or 0 if it is in the range [min, max].
 *
 * @param value number to deadband
 * @param min lower bound of deadband
 * @param max upper bound of deadband
 * @return value, or 0 if it is in the range [min, max]
 */
constexpr double deadband(const double value, const double min, const double max) {
  return std::clamp(value, min, max) == value ? 0 : value;
}

/**
 * Remap a value in the range [oldMin, oldMax] to the range [newMin, newMax].
 *
 * @param value value in the old range
 * @param oldMin old range lower bound
 * @param oldMax old range upper bound
 * @param newMin new range lower bound
 * @param newMax new range upper bound
 * @return remapped value in the new range [newMin, newMax]
 */
constexpr double remapRange(const double value,
                            const double oldMin,
                            const double oldMax,
                            const double newMin,
                            const double newMax) {
  return (value - oldMin) * ((newMax - newMin) / (oldMax - oldMin)) + newMin;
}

/**
 * Converts an enum to its value type.
 */
template <typename E> constexpr auto toUnderlyingType(const E e) noexcept {
  return static_cast<std::underlying_type_t<E>>(e);
}

/**
 * Converts a bool to a sign. True corresponds to 1 and false corresponds to -1.
 */
constexpr auto boolToSign(const bool b) noexcept {
  return b ? 1 : -1;
}

/**
 * Computes lhs mod rhs using Euclidean division. C's % symbol computes the remainder, not modulus.
 *
 * @param lhs the left-hand side
 * @param rhs the right-hand side
 * @return lhs mod rhs
 */
constexpr long modulus(const long lhs, const long rhs) noexcept {
  return ((lhs % rhs) + rhs) % rhs;
}
} // namespace okapi
