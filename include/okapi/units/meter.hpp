/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_METER_HPP_
#define _OKAPI_METER_HPP_

namespace okapi {
inline namespace units {
class Meter {
  public:
  constexpr Meter(const long double imeters) : value(imeters) {
  }

  constexpr Meter(const Meter &other) : value(other.value) {
  }

  ~Meter() = default;

  static constexpr Meter fromMeters(const long double imeters) {
    return Meter(imeters);
  }

  static constexpr Meter fromCentimeters(const long double icentimeters) {
    return Meter(icentimeters * 0.01);
  }

  static constexpr Meter fromMillimeters(const long double imillimeters) {
    return Meter(imillimeters * 0.001);
  }

  static constexpr Meter fromFeet(const long double ifeet) {
    return Meter(ifeet * 0.3048);
  }

  static constexpr Meter fromInches(const long double iinches) {
    return Meter(iinches * 0.0254);
  }

  constexpr long double toMeters() const {
    return value;
  }

  constexpr long double toCentimeters() const {
    return value * 100;
  }

  constexpr long double toMillimeters() const {
    return value * 1000;
  }

  constexpr long double toFeet() const {
    return value * 3.28084;
  }

  constexpr long double toInches() const {
    return value * 39.37007874;
  }

  constexpr Meter operator+(const Meter &rhs) const {
    return Meter(value + rhs.value);
  }

  constexpr Meter operator+(const long double rhs) const {
    return Meter(value + rhs);
  }

  constexpr Meter operator-(const Meter &rhs) const {
    return Meter(value - rhs.value);
  }

  constexpr Meter operator-(const long double rhs) const {
    return Meter(value - rhs);
  }

  constexpr Meter operator*(const long double rhs) const {
    return Meter(value * rhs);
  }

  constexpr Meter operator/(const long double rhs) const {
    return Meter(value / rhs);
  }

  protected:
  const long double value;
};

constexpr Meter operator"" _m(const long double imeters) {
  return Meter(imeters);
}

constexpr Meter operator"" _cm(const long double icentimeters) {
  return Meter(icentimeters * 0.01);
}

constexpr Meter operator"" _mm(const long double imillimeters) {
  return Meter(imillimeters * 0.001);
}

constexpr Meter operator"" _ft(const long double ifeet) {
  return Meter(ifeet * 0.3048);
}

constexpr Meter operator"" _in(const long double iinches) {
  return Meter(iinches * 0.0254);
}
} // namespace units
} // namespace okapi

#endif
