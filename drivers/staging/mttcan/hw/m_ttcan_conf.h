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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TTCAN_CONF_INCLUDE
#define __TTCAN_CONF_INCLUDE
#define CAN_NUMBER_OF_PRESENT_NODES   2

#define MTTCAN_RAM_BASE                0x20000
#define MTTCAN_RAM_SIZE_IN_WORDS      ((4096) >> 2)
#define RAM_WORD_WIDTH_IN_BYTES		4

/* Message RAM - Partitioning */
#define RAM_WORDS_PER_MTT_CAN  1024
#define RAM_BYTES_PER_MTT_CAN  (RAM_WORDS_PER_MTT_CAN * RAM_WORD_WIDTH_IN_BYTES)

#define MAX_11_BIT_FILTER_ELEMS 128
#define MAX_29_BIT_FILTER_ELEMS  64
#define MAX_RX_FIFO_0_ELEMS      32
#define MAX_RX_FIFO_1_ELEMS      32
#define MAX_RX_BUFFER_ELEMS      64
#define MAX_TX_EVENT_FIFO_ELEMS  32
#define MAX_TX_BUFFER_ELEMS      32
#define MAX_TRIG_ELEMS           64

/* Element Size in Words */
#define MAX_RX_BUF_ELEM_SIZE_WORD     18
#define MAX_TX_BUF_ELEM_SIZE_WORD     18
#define TX_EVENT_FIFO_ELEM_SIZE_WORD   2
#define STD_ID_FILTER_ELEM_SIZE_WORD   1
#define EXT_ID_FILTER_ELEM_SIZE_WORD   2

#define TX_BUF_ELEM_HEADER_WORD        2
#define RX_BUF_ELEM_HEADER_WORD        2

#define TRIG_ELEM_SIZE_WORD            2

/* Message RAM config (in Byte) */
/*	+-------------------+
 *	|   11 bit filter   | 16 * 01 * 4
 *	+-------------------+ = 64
 *	|   29 bit filter   | 16 * 02 * 4
 *	+-------------------+ = 128 + 64
 *	|   RX FIFO 0       | 8 * 18 * 4
 *	+-------------------+ = 576 + 192
 *	|   RX FIFO 1       | 8 * 18 * 4
 *	+-------------------+ = 576 + 768
 *	|   RX BUFFERS      | 8 * 18 * 4
 *	+-------------------+ = 576 + 1344
 *	|   TX EVENT FIFO   | 16 * 02 * 4
 *	+-------------------+ = 128 + 1920
 *	|   TX BUFFERS      | 16 * 18 * 4
 *	+-------------------+ = 1152 + 2048
 *	|   MEM TRIGG ELMTS | 16 * 02 * 4
 *	+-------------------+ = 128 + 3232 = 3360
 */

#define CONF_11_BIT_FILTER_ELEMS 16
#define CONF_29_BIT_FILTER_ELEMS 16
#define CONF_RX_FIFO_0_ELEMS     8
#define CONF_RX_FIFO_1_ELEMS     8
#define CONF_RX_BUFFER_ELEMS     8
#define CONF_TX_EVENT_FIFO_ELEMS 16
#define CONF_TX_BUFFER_ELEMS     8
#define CONF_TX_FIFO_ELEMS       8
#define CONF_TRIG_ELEMS          16

#define REL_SIDFC_FLSSA  0
#define REL_XIDFC_FLESA (REL_SIDFC_FLSSA + (CONF_11_BIT_FILTER_ELEMS * STD_ID_FILTER_ELEM_SIZE_WORD * RAM_WORD_WIDTH_IN_BYTES))
#define REL_RXF0C_F0SA  (REL_XIDFC_FLESA + (CONF_29_BIT_FILTER_ELEMS * EXT_ID_FILTER_ELEM_SIZE_WORD * RAM_WORD_WIDTH_IN_BYTES))
#define REL_RXF1C_F1SA  (REL_RXF0C_F0SA  + (CONF_RX_FIFO_0_ELEMS     * MAX_RX_BUF_ELEM_SIZE_WORD    * RAM_WORD_WIDTH_IN_BYTES))
#define REL_RXBC_RBSA   (REL_RXF1C_F1SA  + (CONF_RX_FIFO_1_ELEMS     * MAX_RX_BUF_ELEM_SIZE_WORD    * RAM_WORD_WIDTH_IN_BYTES))
#define REL_TXEFC_EFSA  (REL_RXBC_RBSA   + (CONF_RX_BUFFER_ELEMS     * MAX_RX_BUF_ELEM_SIZE_WORD    * RAM_WORD_WIDTH_IN_BYTES))
#define REL_TXBC_TBSA   (REL_TXEFC_EFSA  + (CONF_TX_EVENT_FIFO_ELEMS * TX_EVENT_FIFO_ELEM_SIZE_WORD * RAM_WORD_WIDTH_IN_BYTES))
#define REL_TMC_TMSA    (REL_TXBC_TBSA   + (CONF_TX_BUFFER_ELEMS     * MAX_TX_BUF_ELEM_SIZE_WORD    * RAM_WORD_WIDTH_IN_BYTES))
#define REL_TMC_END     (REL_TMC_TMSA    + (CONF_TRIG_ELEMS          * TRIG_ELEM_SIZE_WORD          * RAM_WORD_WIDTH_IN_BYTES) - 1)
#endif /*__TTCAN_CONF_INCLUDE */
