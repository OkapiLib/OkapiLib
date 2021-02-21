################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=-Wall -Wextra -Wno-implicit-fallthrough -Wno-psabi -pedantic -Werror=return-type # -Wconversion #-Wmissing-include-dirs
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=0

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=1
LIBNAME:=okapilib
VERSION:=4.2.0
EXCLUDE_SRC_FROM_LIB=$(call rwildcard,$(SRCDIR)/test,*.*)
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+= $(foreach file, $(SRCDIR)/opcontrol $(SRCDIR)/initialize $(SRCDIR)/autonomous $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# Added to the local makefile to ensure that we don't try to download the 
# Squiggles source files for projects including Okapi as a template
DOWNLOAD_SQUIGGLES:=$(shell mkdir -p cmake-build-debug && cd cmake-build-debug && cmake -DCMAKE_BUILD_TYPE=Debug -G "CodeBlocks - Unix Makefiles" .. && mkdir -p ../src/squiggles && find ./squiggles-src/main/src -type f -name '*.cpp' ! -name 'main.cpp' | xargs cp -t ../src/squiggles && cp -r ./squiggles-src/main/include ../include/okapi/squiggles && cd ..)

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the the include directory get exported
TEMPLATE_FILES=$(INCDIR)/okapi/**/*.h $(INCDIR)/okapi/**/*.hpp $(FWDIR)/squiggles.mk

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk
