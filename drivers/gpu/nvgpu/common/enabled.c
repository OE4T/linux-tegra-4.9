/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/enabled.h>
#include <nvgpu/bitops.h>

#include "gk20a/gk20a.h"

int nvgpu_init_enabled_flags(struct gk20a *g)
{
	/*
	 * Zero all flags initially. Flags that should be set to non-zero states
	 * can be done so during driver init.
	 */
	g->enabled_flags = nvgpu_kzalloc(g,
					 BITS_TO_LONGS(NVGPU_MAX_ENABLED_BITS) *
					 sizeof(unsigned long));
	if (!g->enabled_flags)
		return -ENOMEM;

	return 0;
}

bool nvgpu_is_enabled(struct gk20a *g, int flag)
{
	return test_bit(flag, g->enabled_flags);
}

bool __nvgpu_set_enabled(struct gk20a *g, int flag, bool state)
{
	if (state)
		return test_and_set_bit(flag, g->enabled_flags);
	else
		return test_and_clear_bit(flag, g->enabled_flags);
}
