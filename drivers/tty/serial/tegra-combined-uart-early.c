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

#include <asm/io.h>
#include <linux/console.h>
#include <linux/serial_core.h>

#define NUM_BYTES_FIELD_BIT	24
#define FLUSH_BIT		26
#define INTR_TRIGGER_BIT	31

/*
 * This function splits the string to be printed (const char *s) into multiple
 * packets. Each packet contains a max of 3 characters. Packets are sent to the
 * SPE-based combined UART server for printing. Communication with SPE is done
 * through mailbox registers which can generate interrupts for SPE.
 */
static void __init early_tcu_write(struct console *console,
			const char *s, unsigned int count)
{
	struct earlycon_device *device = console->data;
	u8 __iomem *addr = device->port.membase;
	int num_packets, curr_packet_bytes, last_packet_bytes;
	u32 reg_val;
	int i, j;

	num_packets = count / 3;
	if (count % 3 != 0) {
		last_packet_bytes = count % 3;
		num_packets += 1;
	} else {
		last_packet_bytes = 3;
	}

	/* Loop for processing each 3 char packet */
	for (i = 0; i < num_packets; i++) {
		reg_val = BIT(INTR_TRIGGER_BIT);

		if (i == num_packets - 1) {
			reg_val |= BIT(FLUSH_BIT);
			curr_packet_bytes = last_packet_bytes;
		} else {
			curr_packet_bytes = 3;
		}

		/*
		 * Extract the current 3 chars from the
		 * string buffer (const char *s) and store them in the mailbox
		 * register value.
		 */
		reg_val |= curr_packet_bytes << NUM_BYTES_FIELD_BIT;
		for (j = 0; j < curr_packet_bytes; j++)
			reg_val |= s[i*3 + j] << (j * 8);

		/* Send current packet to SPE */
		while (readl(addr) & BIT(INTR_TRIGGER_BIT))
			cpu_relax();
		writel(reg_val, addr);
	}
}

int __init early_tegra_combined_uart_setup(struct earlycon_device *device,
						const char *options)
{
	if (!(device->port.membase))
		return -ENODEV;

	device->con->write = early_tcu_write;

	return 0;
}

EARLYCON_DECLARE(tegra_comb_uart, early_tegra_combined_uart_setup);
