#!/bin/bash
find include/okapi/ -iname *.hpp | xargs clang-format -i -style=file
find include/test/tests/ -iname *.hpp | xargs clang-format -i -style=file
find src/ -iname *.cpp | xargs clang-format -i -style=file
find test/ -iname *.cpp | xargs clang-format -i -style=file
clang-format -i -style=file include/test/crossPlatformTestRunner.hpp
clang-format -i -style=file include/test/testRunner.hpp
