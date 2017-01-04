/* Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/nvs.h>


struct nvs_fn_if *nvs_auto(void)
{
#ifdef NVS_KIF_RELAY
	return nvs_relay();
#endif
#ifdef NVS_KIF_IIO
	return nvs_iio();
#endif
#ifdef NVS_KIF_INPUT
	return nvs_input();
#endif
	return NULL;
}

