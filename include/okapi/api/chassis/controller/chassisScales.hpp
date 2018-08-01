/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CHASSISSCALES_HPP_
#define _OKAPI_CHASSISSCALES_HPP_

#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/RQuantity.hpp"
#include <initializer_list>
#include <stdexcept>
#include <vector>

namespace okapi {
class ChassisScales {
  public:
  /**
   * The two scales a Chassis Controller needs to do all of its closed-loop control. First index is
   * the wheel diameter, second index is the wheelbase width. Read the clawbot programming tutorial
   * for more information behind the meaning of these two numbers.
   *
   * The middle scale can be set directly after calling the constructor if it is needed.
   *
   * The wheelbase diameter is the center-to-center distance between the wheels (center-to-center
   * meaning the width between the centers of both wheels). For example, if you are using four inch
   * omni wheels and there are 11.5 inches between the centers of each wheel, you would call the
   * constructor like so:
   *   ChassisScales scales({4_in, 11.5_in});
   *
   *                             Wheel diameter
   *
   *                              +-+
   *                              | |
   *                              v v
   *
   *                     +--->    ===             ===
   *                     |         +               +
   *                     |        ++---------------++
   *                     |        |                 |
   *    Wheelbase Width  |        |                 |
   *                     |        |                 |
   *                     |        |                 |
   *                     |        ++---------------++
   *                     |         +               +
   *                     +--->    ===             ===
   *
   *
   * @param  iwheelbase {wheel diameter, wheelbase width}
   */
  ChassisScales(const std::initializer_list<QLength> &iwheelbase);

  QLength wheelDiameter{0_m};
  QLength wheelbaseWidth{0_m};
  double straight{0};
  double turn{0};
  double middle{0};

  protected:
  void validateInput(std::size_t inputSize);
};
} // namespace okapi

#endif
