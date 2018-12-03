/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/RQuantity.hpp"
#include "okapi/api/util/logging.hpp"
#include <initializer_list>
#include <stdexcept>
#include <vector>

namespace okapi {
class ChassisScales {
  public:
  /**
   * The scales a Chassis Controller needs to do all of its closed-loop control. First index is
   * the wheel diameter, second index is the wheelbase width. An optional third index is the middle
   * wheel diameter if you are using a 3-encoder setup. Read the clawbot programming tutorial for
   * more information behind the meaning of these two numbers.
   *
   * The wheelbase diameter is the center-to-center distance between the wheels (center-to-center
   * meaning the width between the centers of both wheels). For example, if you are using four inch
   * omni wheels and there are 11.5 inches between the centers of each wheel, you would call the
   * constructor like so:
   *   ChassisScales scales({4_in, 11.5_in});
   *
   *                             Wheel diameter
   *
   *                              +-+      Center of rotation
   *                              | |      |
   *                              v v      +----------+ Length to middle wheel
   *                                       |          | from center of rotation
   *                     +--->    ===      |      === |
   *                     |         +       v       +  |
   *                     |        ++---------------++ |
   *                     |        |                 | v
   *    Wheelbase Width  |        |                 |
   *                     |        |        x        |+|  <-- Middle wheel
   *                     |        |                 |
   *                     |        |                 |
   *                     |        ++---------------++
   *                     |         +               +
   *                     +--->    ===             ===
   *
   *
   * @param  iwheelbase {wheel diameter, wheelbase width} or {wheel diameter, wheelbase width,
   * length to middle wheel, middle wheel diameter}
   * @param itpr The ticks per revolution of the encoders.
   * @param ilogger The logger this instance will log to.
   */
  ChassisScales(const std::initializer_list<QLength> &iwheelbase,
                std::int32_t itpr,
                const std::shared_ptr<Logger> &ilogger = std::make_shared<Logger>());

  /**
   * The scales a Chassis Controller needs to do all of its closed-loop control. First index is
   * the straight scale, second index is the turn scale. An optional third index is the middle
   * scale. The straight scale converts motor degrees to meters, the turn scale converts motor
   * degrees to robot turn degrees, and the middle scale converts middle wheel degrees to meters.
   * Read the clawbot programming tutorial for more information behind the meaning of these two
   * numbers.
   *
   * @param  iscales {straight scale, turn scale} or {straight scale, turn scale, length to middle
   * wheel in meters, middle scale}
   * @param itpr The ticks per revolution of the encoders.
   * @param ilogger The logger this instance will log to.
   */
  ChassisScales(const std::initializer_list<double> &iscales,
                std::int32_t itpr,
                const std::shared_ptr<Logger> &ilogger = std::make_shared<Logger>());

  QLength wheelDiameter;
  QLength wheelbaseWidth;
  QLength middleWheelDistance;
  QLength middleWheelDiameter;
  double straight;
  double turn;
  double middle;
  std::int32_t tpr;

  protected:
  void validateInput(std::size_t inputSize, const std::shared_ptr<Logger> &logger);
};
} // namespace okapi
