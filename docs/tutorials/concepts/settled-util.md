SettledUtils
============

One of the most important factors in utilizing feedback controllers is
establishing proper exit conditions. If a feedback controller decides
that it has reached its target and exits too early, then the target will
never be met. Alternatively, if the controller takes too long to realize
that it's at its goal, the best case scenario is that the movement is
just a bit slower, but in the worst case the movement may never be
considered complete, leaving the motors humming indefinitely.

OkapiLib's [SettledUtil](../../api/control/util/settled-util.html)
provides three different parameters for tuning this exit condition:
`atTargetError`, `atTargetDerivative`, and `atTargetTime`. We'll take a
further look at tuning these three parameters.

atTargetError
-------------

`atTargetError` is the maximum error value for the controller to
consider itself \"settled\", or conclusively at its target. This value
can also be thought of as the precision for the controller's movement;
if a controller with an `atTargetError` value of 10 is moving towards a
target of 100, then the movement could be considered done at any value
between 90 and 110.

It would be ideal to be able to set 0 for this value and theoretically
ensure that the controller ends up exactly at its target with every
movement, but this is a very good way to end up in the indefinite motor
humming scenario detailed at the top of the tutorial. At the very least
this value needs to be greater than the mechanical slop in the system
(the amount that the sensor can move without encountering resistance
from the motors), but typically this value will be closer to the default
of 50.

To disable this check, set the maximum value for a double.

atTargetDerivative
------------------

`atTargetDerivative` is the maximum *change in error* that the
controller can be observing and still consider itself settled. This
value helps prevent scenarios where the controller considers itself
settled while quickly passing by the target and oscillating.

To disable this check, set the maximum value for a double.

atTargetTime
------------

`atTargetTime` is the minimum amount of time that the controller must
stay in the range of `Target +- atTargetError` to be considered settled.
This also helps prevent a false settled scenario when oscillating around
the target.

To disable this check, set 0.
