/*
 * tegra_asoc_xbar_virt_alt.h - Tegra xbar dai link for machine drivers
 *
 * Copyright (c) 2017 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TEGRA_VIRT_ALT_XBAR_H__
#define __TEGRA_VIRT_ALT_XBAR_H__

#include <sound/soc.h>

#define TEGRA_XBAR_RX_STRIDE	0x4
#define TEGRA_T186_SRC_NUM_MUX	83
#define TEGRA_T210_SRC_NUM_MUX	55

#define MUX_REG(id) (TEGRA_XBAR_RX_STRIDE * (id))
#define SOC_ENUM_EXT_REG(xname, xcount, xenum, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_enum_double, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)xenum,	\
	.tlv.p = (unsigned int *) xcount,	\
}

#define MUX_ENUM_CTRL_DECL(ename, reg, src) \
	SOC_ENUM_EXT_REG(ename, MUX_REG(reg),	\
	src,	\
	tegra_virt_get_route,	\
	tegra_virt_put_route)

#define MUX_VALUE(npart, nbit) (1 + nbit + npart * 32)
extern const int tegra_virt_t210ref_source_value[];
extern const char * const tegra_virt_t210ref_source_text[];
extern const int tegra_virt_t186ref_source_value[];
extern const char * const tegra_virt_t186ref_source_text[];

int tegra_virt_get_route(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_put_route(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
void tegra_virt_set_enum_source(const struct soc_enum *enum_virt);
#endif
