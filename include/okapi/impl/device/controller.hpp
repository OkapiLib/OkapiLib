/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_CONTROLLER_HPP_
#define _OKAPI_CONTROLLER_HPP_

#include "api.h"
#include "okapi/impl/device/button/controllerButton.hpp"
#include "okapi/impl/device/controllerUtil.hpp"

namespace okapi {
class Controller {
  public:
  Controller(ControllerId iid = ControllerId::master);

  virtual ~Controller();

  /**
   * Returns whether the controller is connected.
   *
   * @return true if the controller is connected
   */
  virtual bool isConnected();

  /**
   * Returns the full connection state of the controller.
   *  0 = disconnected
   *  1 = tethered
   *  2 = VEXnet
   *
   * @return the connection state of the controller
   */
  virtual std::int32_t getConnectionState();

  /**
   * Returns the current analog reading for the channel in the range [-1, 1]. Returns 0 if the
   * controller is not connected.
   *
   * @param ichannel the channel to read
   * @return the value of that channel in the range [-1, 1]
   */
  virtual float getAnalog(ControllerAnalog ichannel);

  /**
   * Returns whether the digital button is currently pressed. Returns false if the controller is
   * not connected.
   *
   * @param ibutton the button to check
   * @return true if the button is pressed, false if the controller is not connected
   */
  virtual bool getDigital(ControllerDigital ibutton);

  /**
   * Returns a ControllerButton for the given button on this controller.
   *
   * @param ibtn the button
   * @return a ControllerButton on this controller
   */
  virtual ControllerButton operator[](ControllerDigital ibtn);

  /**
   * Sets text to the controller LCD screen.
   *
   * @param iline the line number at which the text will be displayed [0-2]
   * @param icol the column number at which the text will be displayed [0-14]
   * @param itext the string to display
   * @return 1 if the operation was successful, PROS_ERR otherwise
   */
  virtual std::int32_t setText(std::uint8_t iline, std::uint8_t icol, std::string itext);

  /**
   * Clears all of the lines of the controller screen.
   *
   * @return 1 if the operation was successful, PROS_ERR otherwise
   */
  virtual std::int32_t clear();

  /**
   * Clears an individual line of the controller screen.
   *
   * @param iline the line number to clear
   * @return 1 if the operation was successful, PROS_ERR otherwise
   */
  virtual std::int32_t clearLine(std::uint8_t iline);

  /**
   * Rumble the controller.
   *
   * Controller rumble activation is currently in beta, so continuous, fast
   * updates will not work well.
   *
   * @param irumblePattern A string consisting of the characters '.', '-', and ' ', where dots are
   * short rumbles, dashes are long rumbles, and spaces are pauses. Maximum supported length is 8
   * characters.
   *
   * @return 1 if the operation was successful or PROS_ERR if the operation failed, setting errno.
   */
  virtual std::int32_t rumble(std::string irumblePattern);

  /**
   * Gets the battery capacity of the given controller.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the controller port.
   *
   * @return the controller's battery capacity
   */
  virtual std::int32_t getBatteryCapacity();

  /**
   * Gets the battery level of the given controller.
   *
   * This function uses the following values of errno when an error state is reached:
   * EACCES - Another resource is currently trying to access the controller port.
   *
   * @return the controller's battery level
   */
  virtual std::int32_t getBatteryLevel();

  protected:
  const ControllerId m_id;
  pros::Controller controller;
};
} // namespace okapi

#endif
