#!/bin/bash
cppcheck --enable=all -I include/okapi include/test/tests src test
