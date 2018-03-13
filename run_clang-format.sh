#!/bin/bash
find include/okapi/ -iname *.hpp | xargs clang-format-5.0 -i -style=file
find src/ -iname *.cpp | xargs clang-format-5.0 -i -style=file
