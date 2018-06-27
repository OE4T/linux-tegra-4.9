#!/bin/bash

#
# Execute the unit test. Args to this script are passed on to the unit test
# core. This just serves to set the LD_LIBRARY_PATH environment variable such
# that unit tests are found and nvgpu-drv is found.
#

LD_LIBRARY_PATH="build:build/units"
NVGPU_UNIT=build/nvgpu_unit

export LD_LIBRARY_PATH

echo "$ $NVGPU_UNIT $*"

$NVGPU_UNIT $*
