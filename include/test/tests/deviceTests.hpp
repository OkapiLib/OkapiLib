/**
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#ifndef _OKAPI_DEVICETESTS_HPP_
#define _OKAPI_DEVICETESTS_HPP_

#include "okapi/api.hpp"
#include "test/testRunner.hpp"

void testButtons() {
  using namespace okapi;
  using namespace snowhouse;
  using namespace fakeit;

  class MockButton : public ButtonBase {
    public:
    bool currentlyPressed() override {
      printf("???\n");
      return false;
    }
  };

  {
    test_printf("Testing Button");

    {
      MockButton spyMe;
      Mock<MockButton> mockFactory(spyMe);
      When(Method(mockFactory, currentlyPressed)).Return(false).Return(true).Return(false);
      Spy(Method(mockFactory, isPressed));
      MockButton &btn = mockFactory.get();

      test("Button isPressed should be false",
           TEST_BODY(AssertThat, btn.isPressed(), Equals(false)));
      test("Button isPressed should be true", TEST_BODY(AssertThat, btn.isPressed(), Equals(true)));
      test("Button isPressed should be false",
           TEST_BODY(AssertThat, btn.isPressed(), Equals(false)));
    }

    {
      MockButton spyMe;
      Mock<MockButton> mockFactory(spyMe);
      When(Method(mockFactory, currentlyPressed))
        .Return(false)
        .Return(true)
        .Return(true)
        .Return(false)
        .Return(false);
      Spy(Method(mockFactory, changed));
      MockButton &btn = mockFactory.get();

      test("Button changed should be false", TEST_BODY(AssertThat, btn.changed(), Equals(false)));
      test("Button changed should be true", TEST_BODY(AssertThat, btn.changed(), Equals(true)));
      test("Button changed should be false", TEST_BODY(AssertThat, btn.changed(), Equals(false)));
      test("Button changed should be true", TEST_BODY(AssertThat, btn.changed(), Equals(true)));
      test("Button changed should be false", TEST_BODY(AssertThat, btn.changed(), Equals(false)));
    }

    {
      MockButton spyMe;
      Mock<MockButton> mockFactory(spyMe);
      When(Method(mockFactory, currentlyPressed))
        .Return(false)
        .Return(true)
        .Return(true)
        .Return(false)
        .Return(false);
      Spy(Method(mockFactory, changed));
      Spy(Method(mockFactory, changedToPressed));
      MockButton &btn = mockFactory.get();

      test("Button changedToPressed should be false",
           TEST_BODY(AssertThat, btn.changedToPressed(), Equals(false)));
      test("Button changedToPressed should be true",
           TEST_BODY(AssertThat, btn.changedToPressed(), Equals(true)));
      test("Button changedToPressed should be false",
           TEST_BODY(AssertThat, btn.changedToPressed(), Equals(false)));
      test("Button changedToPressed should be true",
           TEST_BODY(AssertThat, btn.changedToPressed(), Equals(false)));
      test("Button changedToPressed should be false",
           TEST_BODY(AssertThat, btn.changedToPressed(), Equals(false)));
    }

    {
      MockButton spyMe;
      Mock<MockButton> mockFactory(spyMe);
      When(Method(mockFactory, currentlyPressed))
        .Return(false)
        .Return(true)
        .Return(true)
        .Return(false)
        .Return(false);
      Spy(Method(mockFactory, changed));
      Spy(Method(mockFactory, changedToReleased));
      MockButton &btn = mockFactory.get();

      test("Button changedToReleased should be false",
           TEST_BODY(AssertThat, btn.changedToReleased(), Equals(false)));
      test("Button changedToReleased should be true",
           TEST_BODY(AssertThat, btn.changedToReleased(), Equals(false)));
      test("Button changedToReleased should be false",
           TEST_BODY(AssertThat, btn.changedToReleased(), Equals(false)));
      test("Button changedToReleased should be true",
           TEST_BODY(AssertThat, btn.changedToReleased(), Equals(true)));
      test("Button changedToReleased should be false",
           TEST_BODY(AssertThat, btn.changedToReleased(), Equals(false)));
    }
  }
}

void runHeadlessDeviceTests() {
  testButtons();
}

#endif
