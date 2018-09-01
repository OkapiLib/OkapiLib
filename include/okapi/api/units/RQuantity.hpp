/**
 * @author Mikhail Semenov
 * @author Benjamin Jurke
 * @author Ryan Benasutti, WPI
 *
 * This code is a modified version of Benjamin Jurke's work in 2015. You can read his blog post
 * here:
 * https://benjaminjurke.com/content/articles/2015/compile-time-numerical-unit-dimension-checking/
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_RQUANTITY_HPP_
#define _OKAPI_RQUANTITY_HPP_

#include <ratio>

namespace okapi {
template <typename MassDim, typename LengthDim, typename TimeDim, typename AngleDim>
class RQuantity {
  public:
  explicit constexpr RQuantity() : value(0.0) {
  }

  explicit constexpr RQuantity(double val) : value(val) {
  }

  explicit constexpr RQuantity(long double val) : value(static_cast<double>(val)) {
  }

  // The intrinsic operations for a quantity with a unit is addition and subtraction
  constexpr RQuantity const &operator+=(const RQuantity &rhs) {
    value += rhs.value;
    return *this;
  }

  constexpr RQuantity const &operator-=(const RQuantity &rhs) {
    value -= rhs.value;
    return *this;
  }

  // Returns the value of the quantity in multiples of the specified unit
  constexpr double convert(const RQuantity &rhs) const {
    return value / rhs.value;
  }

  // returns the raw value of the quantity (should not be used)
  constexpr double getValue() const {
    return value;
  }

  private:
  double value;
};

// Predefined (physical unit) quantity types:
// ------------------------------------------
#define QUANTITY_TYPE(_Mdim, _Ldim, _Tdim, _Adim, name)                                            \
  typedef RQuantity<std::ratio<_Mdim>, std::ratio<_Ldim>, std::ratio<_Tdim>, std::ratio<_Adim>>    \
    name;

// Unitless
QUANTITY_TYPE(0, 0, 0, 0, Number)

// Standard arithmetic operators:
// ------------------------------
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> operator+(const RQuantity<M, L, T, A> &lhs,
                                          const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(lhs.getValue() + rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> operator-(const RQuantity<M, L, T, A> &lhs,
                                          const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(lhs.getValue() - rhs.getValue());
}
template <typename M1,
          typename L1,
          typename T1,
          typename A1,
          typename M2,
          typename L2,
          typename T2,
          typename A2>
constexpr RQuantity<std::ratio_add<M1, M2>,
                    std::ratio_add<L1, L2>,
                    std::ratio_add<T1, T2>,
                    std::ratio_add<A1, A2>>
operator*(const RQuantity<M1, L1, T1, A1> &lhs, const RQuantity<M2, L2, T2, A2> &rhs) {
  return RQuantity<std::ratio_add<M1, M2>,
                   std::ratio_add<L1, L2>,
                   std::ratio_add<T1, T2>,
                   std::ratio_add<A1, A2>>(lhs.getValue() * rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> operator*(const double &lhs, const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(lhs * rhs.getValue());
}
template <typename M1,
          typename L1,
          typename T1,
          typename A1,
          typename M2,
          typename L2,
          typename T2,
          typename A2>
constexpr RQuantity<std::ratio_subtract<M1, M2>,
                    std::ratio_subtract<L1, L2>,
                    std::ratio_subtract<T1, T2>,
                    std::ratio_subtract<A1, A2>>
operator/(const RQuantity<M1, L1, T1, A1> &lhs, const RQuantity<M2, L2, T2, A2> &rhs) {
  return RQuantity<std::ratio_subtract<M1, M2>,
                   std::ratio_subtract<L1, L2>,
                   std::ratio_subtract<T1, T2>,
                   std::ratio_subtract<A1, A2>>(lhs.getValue() / rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<std::ratio_subtract<std::ratio<0>, M>,
                    std::ratio_subtract<std::ratio<0>, L>,
                    std::ratio_subtract<std::ratio<0>, T>,
                    std::ratio_subtract<std::ratio<0>, A>>
operator/(double x, const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<std::ratio_subtract<std::ratio<0>, M>,
                   std::ratio_subtract<std::ratio<0>, L>,
                   std::ratio_subtract<std::ratio<0>, T>,
                   std::ratio_subtract<std::ratio<0>, A>>(x / rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> operator/(const RQuantity<M, L, T, A> &rhs, double x) {
  return RQuantity<M, L, T, A>(rhs.getValue() / x);
}

// Comparison operators for quantities:
// ------------------------------------
template <typename M, typename L, typename T, typename A>
constexpr bool operator==(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() == rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator!=(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() != rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator<=(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() <= rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator>=(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() >= rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator<(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() < rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator>(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() > rhs.getValue());
}

inline namespace literals {
constexpr long double operator"" _pi(long double x) {
  return static_cast<double>(x) * 3.1415926535897932384626433832795;
}
constexpr long double operator"" _pi(unsigned long long int x) {
  return static_cast<double>(x) * 3.1415926535897932384626433832795;
}
} // namespace literals
} // namespace okapi

// Conversion macro, which utilizes the string literals
#define ConvertTo(_x, _y) (_x).convert(1.0_##_y)

#endif
