---
title: API Reference
type: index
---

This section is meant to be a quick reference for Okapi's entire API, including methods the user may not normally interact with. This reference is broken into sections, covering one class per section. Subclasses are placed below the base class, but in different sections. If a class is marked (abstract) then it contains one or more pure virtual functions, and cannot be instantiated (it is designed only to be an interface).

{{< note title="Note" >}}
Remember that derived classes inherit the interface of their base class; therefore, derived classes will not have their base class' functions documented (you can safely assume that all functions from the base class are implemented).
{{< /note >}}

{{< readfile file="content/api/filter/avgFilter.md" markdown="true" >}}
{{< readfile file="content/api/device/button.md" markdown="true" >}}
{{< readfile file="content/api/chassisController/chassisController.md" markdown="true" >}}
{{< readfile file="content/api/chassisController/chassisControllerMP.md" markdown="true" >}}
{{< warning title="Careful" >}}
Most users should not call this constructor with a std::shared_ptr&lt;ChassisModel&gt;. Instead, pass a ChassisModelParams and Okapi will figure out what to do.
{{< /warning >}}
{{< readfile file="content/api/chassisController/chassisControllerPID.md" markdown="true" >}}
{{< warning title="Careful" >}}
Most users should not call this constructor with a std::shared_ptr&lt;ChassisModel&gt;. Instead, pass a ChassisModelParams and Okapi will figure out what to do.
{{< /warning >}}
{{< readfile file="content/api/chassisModel/chassisModel.md" markdown="true" >}}
{{< readfile file="content/api/chassisModel/chassisModelParams.md" markdown="true" >}}
{{< readfile file="content/api/control/controlObject.md" markdown="true" >}}
{{< readfile file="content/api/device/cubicMotor.md" markdown="true" >}}
{{< readfile file="content/api/filter/demaFilter.md" markdown="true" >}}
{{< readfile file="content/api/odometry/distanceAndAngle.md" markdown="true" >}}
{{< readfile file="content/api/filter/emaFilter.md" markdown="true" >}}
{{< readfile file="content/api/filter/filter.md" markdown="true" >}}
{{< readfile file="content/api/control/genericController.md" markdown="true" >}}
{{< readfile file="content/api/util/mathUtil.md" markdown="true" >}}
{{< readfile file="content/api/motionProfile/motionProfile.md" markdown="true" >}}
{{< readfile file="content/api/device/motor.md" markdown="true" >}}
{{< readfile file="content/api/motionProfile/mpConsumer.md" markdown="true" >}}
{{< readfile file="content/api/motionProfile/mpConsumerParams.md" markdown="true" >}}
{{< readfile file="content/api/control/mpController.md" markdown="true" >}}
{{< readfile file="content/api/motionProfile/mpGenerator.md" markdown="true" >}}
{{< readfile file="content/api/motionProfile/mpGeneratorParams.md" markdown="true" >}}
{{< readfile file="content/api/control/pid/nsPid.md" markdown="true" >}}
{{< readfile file="content/api/chassisController/odomChassisController/odomChassisController.md" markdown="true" >}}
{{< readfile file="content/api/chassisController/odomChassisController/odomChassisControllerMP.md" markdown="true" >}}
{{< readfile file="content/api/chassisController/odomChassisController/odomChassisControllerPID.md" markdown="true" >}}
{{< readfile file="content/api/odometry/odometry.md" markdown="true" >}}
{{< readfile file="content/api/odometry/odomMath.md" markdown="true" >}}
{{< readfile file="content/api/control/pid/pid.md" markdown="true" >}}
{{< readfile file="content/api/control/pid/pidParams.md" markdown="true" >}}
{{< readfile file="content/api/device/potentiometer.md" markdown="true" >}}
{{< readfile file="content/api/device/quadEncoder.md" markdown="true" >}}
{{< readfile file="content/api/device/rangeFinder.md" markdown="true" >}}
{{< readfile file="content/api/util/timer.md" markdown="true" >}}
{{< readfile file="content/api/control/velMath.md" markdown="true" >}}
{{< readfile file="content/api/control/pid/velPid.md" markdown="true" >}}
{{< readfile file="content/api/control/pid/velPidParams.md" markdown="true" >}}
