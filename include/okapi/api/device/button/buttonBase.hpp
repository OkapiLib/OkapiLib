#ifndef _OKAPI_BUTTONBASE_HPP_
#define _OKAPI_BUTTONBASE_HPP_

#include "okapi/api/device/button/abstractButton.hpp"

namespace okapi {
class ButtonBase : public AbstractButton {
  public:
  ButtonBase(const bool iinverted = false);

  /**
   * Return whether the button is currently pressed.
   **/
  virtual bool isPressed() override;

  /**
   * Return whether the state of the button changed since the last time this method was
   * called.
   **/
  virtual bool changed() override;

  /**
   * Return whether the state of the button changed to being pressed since the last time this method
   * was called.
   **/
  virtual bool changedToPressed() override;

  /**
   * Return whether the state of the button to being not pressed changed since the last time this
   * method was called.
   **/
  virtual bool changedToReleased() override;

  protected:
  const bool inverted;
  bool wasPressedLast = false;

  virtual bool currentlyPressed() = 0;
};
} // namespace okapi

#endif
