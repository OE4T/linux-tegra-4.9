# Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

# Turn off suffix rules. They are deprecated.
.SUFFIXES:

# Full paths. Makes submakefiles easier. That said this make file _must_ be run
# from <NVGPU>/userspace/.
TWD=$(CURDIR)

# Top level out dir.
OUT=$(TWD)/build

# Nvgpu driver code.
NVGPU_SRC=$(TWD)/../drivers/gpu/nvgpu
NVGPU_OUT=$(OUT)/libnvgpu

INCLUDES= 			\
	-I$(NVGPU_SRC)		\
	-I$(NVGPU_SRC)/include	\
	-I../include		\
	-I../include/uapi

CONFIGS=			\
	-D__NVGPU_POSIX__

# Compiler, c-flags, etc.

# CC    = clang
CC      = gcc
CFLAGS  = -Wall -ggdb  -Werror -fPIC $(INCLUDES) $(CONFIGS)
LIBS    = -lpthread -pthread

# Source files. We expect $(OBJS) and $(HEADERS) to get filled in here.
include Makefile.sources

# Linuxy configs. We want these so that we can mirror builds from the actual
# Linux kernel.
include Makefile.configs

all: $(OUT)/libnvgpu-drv.so

$(OUT)/libnvgpu-drv.so: $(OBJS)
	$(CC) -shared -o $(OUT)/libnvgpu-drv.so $(OBJS) -lgcov

# Default build target for all the object files we want to build in userspace.
$(NVGPU_OUT)/%.o : $(NVGPU_SRC)/%.c $(HEADERS)
	@if [ ! -d $(dir $@) ] ; then \
		mkdir -p $(dir $@) ; \
	fi
	$(CC) $(CFLAGS) $(configs) -c -o $@ $<

.PHONY: clean

clean:
	rm -rf $(OUT)
