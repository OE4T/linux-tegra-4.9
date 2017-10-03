/*
 * This header provides macros for different DT binding version used
 * by different OS/kernel versions.
 */

#ifndef _DT_BINDINGS_VERSION_H_
#define _DT_BINDINGS_VERSION_H_

#define DT_VERSION_1		1
#define DT_VERSION_2		2

/**
 * TEGRA_AUDIO_BUS_DT_VERSION: Audio node DT version.
 *		V1: All audio bus are in root node.
 *		V2: All audio bus are in the under aconnect node.
 *
 * TEGRA_POWER_DOMAIN_DT_VERSION: Power domain DT versions.
 *		V1: Use legacy power gating API.
 *		V2: Use BPMP power domain provider.
 *
 * TEGRA_XUSB_PADCONTROL_VERSION
 *		V1: Nv Version of DT bidning
 *		V2: Mainline compatible DT binding.
 *
 * TEGRA_XUSB_DT_VERSION: XUSB DT binding version
 *		V1: Nv Version of DT bidning
 *		V2: Mainline compatible DT binding.
 *
 * TEGRA_XUDC_DT_VERSION: XUDC DT binding version
 *		V1: Nv Version of DT bidning
 *		V2: Mainline compatible DT binding.
 *
 * TEGRA_GENERIC_CARVEOUT_SUPPORT_ENABLE
 *		Enable generic carveout.
 */

/* OS Linux */
#if defined(LINUX_VERSION)
#define _OS_FOUND_
#if LINUX_VERSION >= 409
#define TEGRA_AUDIO_BUS_DT_VERSION		DT_VERSION_2
#define TEGRA_POWER_DOMAIN_DT_VERSION		DT_VERSION_2
#define TEGRA_XUSB_PADCONTROL_VERSION		DT_VERSION_2
#define TEGRA_XUSB_DT_VERSION			DT_VERSION_2
#define TEGRA_XUDC_DT_VERSION			DT_VERSION_2
#define TEGRA_GENERIC_CARVEOUT_SUPPORT_ENABLE	1
#else
#define TEGRA_AUDIO_BUS_DT_VERSION		DT_VERSION_1
#define TEGRA_POWER_DOMAIN_DT_VERSION		DT_VERSION_1
#define TEGRA_XUSB_PADCONTROL_VERSION		DT_VERSION_1
#define TEGRA_XUSB_DT_VERSION			DT_VERSION_1
#define TEGRA_XUDC_DT_VERSION			DT_VERSION_1
#define TEGRA_GENERIC_CARVEOUT_SUPPORT_ENABLE	0
#endif
#endif

/* OS QNX */
#if defined (__QNX__)
#define _OS_FOUND_
#define TEGRA_AUDIO_BUS_DT_VERSION		DT_VERSION_1
#define TEGRA_POWER_DOMAIN_DT_VERSION		DT_VERSION_1
#define TEGRA_XUSB_PADCONTROL_VERSION		DT_VERSION_1
#define TEGRA_XUSB_DT_VERSION			DT_VERSION_1
#define TEGRA_XUDC_DT_VERSION			DT_VERSION_1
#define TEGRA_GENERIC_CARVEOUT_SUPPORT_ENABLE	0
#endif

/* OS Integrity */
#if defined( __INTEGRITY)
#define _OS_FOUND_
#define TEGRA_AUDIO_BUS_DT_VERSION		DT_VERSION_1
#define TEGRA_POWER_DOMAIN_DT_VERSION		DT_VERSION_1
#define TEGRA_XUSB_PADCONTROL_VERSION		DT_VERSION_1
#define TEGRA_XUSB_DT_VERSION			DT_VERSION_1
#define TEGRA_XUDC_DT_VERSION			DT_VERSION_1
#define TEGRA_GENERIC_CARVEOUT_SUPPORT_ENABLE	0
#endif

/**
 *  If no OS found then set as default.
 * This will be converted to error once all OS define OS
 * macro
 */
#if !defined(_OS_FOUND_)
#define TEGRA_AUDIO_BUS_DT_VERSION		DT_VERSION_1
#define TEGRA_POWER_DOMAIN_DT_VERSION		DT_VERSION_1
#define TEGRA_XUSB_PADCONTROL_VERSION		DT_VERSION_1
#define TEGRA_XUSB_DT_VERSION			DT_VERSION_1
#define TEGRA_XUDC_DT_VERSION			DT_VERSION_1
#define TEGRA_GENERIC_CARVEOUT_SUPPORT_ENABLE	0
#endif

#endif /* _DT_BINDINGS_VERSION_H_ */
