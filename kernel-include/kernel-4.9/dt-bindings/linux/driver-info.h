/*
 * This header provides constants for driver specific info which is applicable
 * in that linux version.
 *
 * Most PWM bindings can include a flags cell as part of the PWM specifier.
 * In most cases, the format of the flags cell uses the standard values
 * defined in this header.
 */

#ifndef _DT_BINDINGS_DRIVER_INFO_H
#define _DT_BINDINGS_DRIVER_INFO_H

/* For the time being, declare us also as kernel 4.4 */
#define KERNEL_VERSION_4_9

/*
 * SDMMC core in kernel 4.4 and presumably in kernel 4.9 handles the
 * chip specific regulator and hence provide all regulator with core
 * name supply.
 */
#define SDMMC_USE_CORE_REGULATOR_HANDLER

#endif
