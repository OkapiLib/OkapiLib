# A note about where to use builders

OkapiLib's builders wire together any tasks internal to the object being built such that they are
stopped when the calling task (the task the builder is run from) is deleted (to prevent runaway
tasks). This means that if you run the builder's `build()` method in the `autonomous` task, the
built object's internal tasks will be stopped once `autonomous` ends. If you put the built object in
a variable in global scope and then you try to use it in `opcontrol`, **it will not work** because
the object's internal tasks will be stopped when `autonomous` ends.

This is the case for every task except for `initialize`. To work around this limitation, you can:
- Use the builder in global scope and save the built object to a variable also in global scope
- Use the builder in `initialize` and save the built object to a variable in global scope
- Use the builder in a local scope and save the built object to a variable _also in the same local
scope_ 
