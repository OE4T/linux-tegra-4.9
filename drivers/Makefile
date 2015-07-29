# Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.

KBUILD_CFLAGS += -I$(srctree)/../kernel-t18x/include
obj-y += platform/
obj-y += video/
obj-y += clk/
obj-$(CONFIG_CRYPTO) += crypto/
obj-y += dma/
obj-y += spi/
obj-$(CONFIG_TEGRA_BPMP) += firmware/tegra/
obj-$(CONFIG_CPU_FREQ)   += cpufreq/
obj-y += pinctrl/
obj-y += gpio/
obj-y += reset/
obj-y += thermal/
obj-$(CONFIG_STAGING) += staging/
obj-y += i2c/
obj-y += net/
obj-$(CONFIG_CPU_IDLE) += cpuidle/
