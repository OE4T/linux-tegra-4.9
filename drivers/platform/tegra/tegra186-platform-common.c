/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_fdt.h>

static __init int display_tegra_dt_info(void)
{
	int ret_d;
	int ret_t;
	unsigned long dt_root;
	const char *dts_fname;
	const char *dtb_bdate;
	const char *dtb_btime;

	dt_root = of_get_flat_dt_root();

	dts_fname = of_get_flat_dt_prop(dt_root, "nvidia,dtsfilename", NULL);
	if (dts_fname)
		pr_info("DTS File Name: %s\n", dts_fname);
	else
		pr_info("DTS File Name: <unknown>\n");

	ret_d = of_property_read_string_index(of_find_node_by_path("/"),
			"nvidia,dtbbuildtime", 0, &dtb_bdate);
	ret_t = of_property_read_string_index(of_find_node_by_path("/"),
			"nvidia,dtbbuildtime", 1, &dtb_btime);
	if (!ret_d && !ret_t)
		pr_info("DTB Build time: %s %s\n", dtb_bdate, dtb_btime);
	else
		pr_info("DTB Build time: <unknown>\n");

	return 0;
}
arch_initcall(display_tegra_dt_info);

