/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _NVGPU_FIRMWARE_H_
#define _NVGPU_FIRMWARE_H_

struct gk20a;

#define NVGPU_REQUEST_FIRMWARE_NO_WARN		(1UL << 0)
#define NVGPU_REQUEST_FIRMWARE_NO_SOC		(1UL << 1)

struct nvgpu_firmware {
	u8 *data;
	size_t size;
};

/**
 * nvgpu_request_firmware - load a firmware blob from filesystem.
 *
 * @g		The GPU driver struct for device to load firmware for
 * @fw_name	The base name of the firmware file.
 * @flags	Flags for loading;
 *
 * 		NVGPU_REQUEST_FIRMWARE_NO_WARN: Do not display warning on
 * 		failed load.
 *
 * 		NVGPU_REQUEST_FIRMWARE_NO_SOC: Do not attempt loading from
 * 		path <SOC_NAME>.
 *
 * nvgpu_request_firmware() will load firmware from:
 *
 * <system firmware load path>/<GPU name>/<fw_name>
 *
 * If that fails and NO_SOC is not enabled, it'll try next from:
 *
 * <system firmware load path>/<SOC name>/<fw_name>
 *
 * It'll allocate a nvgpu_firmware structure and initializes it and returns
 * it to caller.
 */
struct nvgpu_firmware *nvgpu_request_firmware(struct gk20a *g,
					      const char *fw_name,
					      int flags);

/**
 * nvgpu_release_firmware - free firmware and associated nvgpu_firmware blob
 *
 * @g		The GPU driver struct for device to free firmware for
 * @fw		The firmware to free. fw blob will also be freed.
 */
void nvgpu_release_firmware(struct gk20a *g, struct nvgpu_firmware *fw);

#endif
