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

struct ari_mca_error {
	char *name;
	u16 error_code;
};

struct ari_mca_bank {
	struct list_head node;
	char *name;
	u64 bank;
	struct ari_mca_error *errors;
};

void register_ari_mca_bank(struct ari_mca_bank *bank);
void unregister_ari_mca_bank(struct ari_mca_bank *bank);
