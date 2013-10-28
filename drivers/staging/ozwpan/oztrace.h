/* -----------------------------------------------------------------------------
 * Copyright (c) 2011 Ozmo Inc
 * Released under the GNU General Public License Version 2 (GPLv2).
 * -----------------------------------------------------------------------------
 */
#ifndef _OZTRACE_H_
#define _OZTRACE_H_
#include <linux/usb.h>
#include <linux/netdevice.h>
#include "ozeventtrace.h"

extern struct device *g_oz_wpan_dev;

#define oz_trace(fmt, ...) \
	do { dev_dbg(g_oz_wpan_dev, fmt, ##__VA_ARGS__); } while (0)

void oz_trace_f_urb_out(struct urb *urb, int status);
void oz_trace_f_urb_in(struct urb *urb);
void oz_trace_f_skb(struct sk_buff *skb, char dir);
void oz_trace_f_dbg(void);
void trace_dbg_msg(int c, char *fmt, ...);
void trace_debug_log(char *log_type, ...);

extern u32 g_debug;

#define TRC_A 0x00000001
#define TRC_B 0x00000002
#define TRC_C 0x00000004	/* urb Completion */
#define TRC_D 0x00000008	/* Debug */
#define TRC_E 0x00000010	/* urb Error */
#define TRC_F 0x00000020
#define TRC_G 0x00000040
#define TRC_H 0x00000080	/* Hcd message */
#define TRC_I 0x00000100	/* Isoc buffer depth */
#define TRC_J 0x00000200
#define TRC_K 0x00000400
#define TRC_L 0x00000800
#define TRC_M 0x00001000	/* Message */
#define TRC_N 0x00002000
#define TRC_O 0x00004000
#define TRC_P 0x00008000
#define TRC_Q 0x00010000
#define TRC_R 0x00020000	/* Rx Ozmo frame */
#define TRC_S 0x00040000	/* urb Submission */
#define TRC_T 0x00080000	/* Tx ozmo frame */
#define TRC_U 0x00100000
#define TRC_V 0x00200000
#define TRC_W 0x00400000
#define TRC_X 0x00800000
#define TRC_Y 0x01000000
#define TRC_Z 0x02000000


#define oz_trace_urb_out(u, s) \
	do { if (!g_debug) \
		trace_urb_out(u, s); \
	else if ((g_debug & TRC_C) || ((g_debug & TRC_E) && (u->status != 0))) \
		oz_trace_f_urb_out(u, s); } while (0)

#define oz_trace_urb_in(u) \
	do { if (!g_debug) \
		trace_urb_in(u); \
	else if (g_debug & TRC_S) \
		oz_trace_f_urb_in(u); } while (0)

#define oz_trace_skb(u, d) \
	do { if ((!g_debug) && ('T' == d)) \
		trace_tx_frame(u); \
	else if ((!g_debug) && ('R' == d)) \
		trace_rx_frame(u); \
	else if ((('T' == d) && (g_debug & TRC_T)) || \
					(('R' == d) && (g_debug & TRC_R))) \
		oz_trace_f_skb(u, d); } while(0)

#define oz_trace_msg(f, ...) \
	do { if (!g_debug) \
		trace_debug_log(#f, __VA_ARGS__); \
	else if (g_debug & TRC_##f) \
		printk("OZ " #f " " __VA_ARGS__); } while(0)

enum {
	TRACE_HCD_MSG,
	TRACE_ISOC_MSG,
	TRACE_INFO_MSG
};

#define trace_hcd_msg(fmt, ...)\
	trace_dbg_msg(TRACE_HCD_MSG, fmt, ##__VA_ARGS__)

#define trace_isoc_msg(fmt, ...)\
	trace_dbg_msg(TRACE_ISOC_MSG, fmt, ##__VA_ARGS__)

#define trace_info_msg(fmt, ...)\
	trace_dbg_msg(TRACE_INFO_MSG, fmt, ##__VA_ARGS__)

#endif /* Sentry */

