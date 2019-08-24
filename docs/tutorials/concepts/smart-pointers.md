Smart Pointers
==============

Overview
--------

**Smart Pointers** are a C++ concept that extend the functionality of C
pointers. A knowledge of Smart Pointers is not necessary for simple to
intermediate OkapiLib programs, but extending OkapiLib's functionality
(i.e. adding your own control algorithm) will require that you know how
to use smart pointers. Smart Pointers may seem complicated at first,
given their more complex syntax compared to the simple '\*' and '&'
operations with traditional pointers, but they make it much safer and
easier to handle memory.

Given that Smart Pointers are a standard C++ feature and not specific to
OkapiLib, a plethora of guides on the subject can be found elsewhere on
the Internet. We recommend the following tutorial:
<https://msdn.microsoft.com/en-us/library/hh279674.aspx>

The above tutorial will give you the majority of the information you
will need to use Smart Pointers. That said, it is worth noting some
conventions with Smart Pointer use in OkapiLib.

There are three types of Smart Pointers: `unique_ptr`, `shared_ptr`, and
`weak_ptr`. As of the time of writing, there are no uses of `weak_ptr`
in OkapiLib.

`unique_ptr` is used for parameters to an object that will take
exclusive ownership of the object that is pointed to by the
`unique_ptr`. A good example of this is the `IterativeController`
parameter to an `AsyncWrapper`. The `AsyncWrapper` takes control of the
`IterativeController` and handles all future movements executed by the
controller because you wouldn't want another object trying to interact
with the same `IterativeController`.

`shared_ptr` is used for parameters to an object that should be able to
be used by more than one other object. The `ControllerInput` and
`ControllerOutput` parameters to the same `AsyncWrapper` class are good
examples of this. While it is typical that your `AsyncWrapper`
controller will be the only object using the `ControllerInput` data and
the only object writing to the `ControllerOutput` (typically a motor or
motor group) object, it is bad practice to prevent other objects from
accessing that data if, for instance, you also wanted a class to monitor
and log the `ControllerInput` data. `shared_ptr` is a good choice if
there is a possibility that the user will want to use the pointer's
object for more applications than the class you're designing.

Dynamic-casting Smart Pointers
------------------------------

A smart pointer cannot be used with `dynamic_cast` like a normal object
could. Instead, you need to use `dynamic_pointer_cast`. For example, if
you want to cast the output of a `ChassisControllerBuilder` into a more
specific type because you need features of that specific implementation,
you could do this:
