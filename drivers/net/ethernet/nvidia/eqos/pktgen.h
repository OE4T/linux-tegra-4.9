/* =========================================================================
 * The Synopsys DWC ETHER QOS Software Driver and documentation (hereinafter
 * "Software") is an unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto.  Permission is hereby granted,
 * free of charge, to any person obtaining a copy of this software annotated
 * with this license and the Software, to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * ========================================================================= */
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
#ifndef __DWC_ETH_QOS_YPG_H__

#define __DWC_ETH_QOS_YPG_H__

static void DWC_ETH_QOS_tx_interrupt_pg(struct DWC_ETH_QOS_prv_data *pdata,
				     UINT qInx);

static void DWC_ETH_QOS_prepare_desc(struct DWC_ETH_QOS_prv_data *pdata,
				struct s_TX_NORMAL_DESC *txptr,
				struct DWC_ETH_QOS_tx_buffer *buffer,
				int i,
				unsigned int qInx);

static int DWC_ETH_QOS_poll_pg_sq(struct DWC_ETH_QOS_prv_data *pdata,
					unsigned int qInx);

static void DWC_ETH_QOS_poll_pg(struct DWC_ETH_QOS_prv_data *pdata);

static void DWC_ETH_QOS_prepare_hw_for_pg_test(struct DWC_ETH_QOS_prv_data *pdata);

static void DWC_ETH_QOS_pg_timer_fun(unsigned long data);

static void DWC_ETH_QOS_pg_run(struct DWC_ETH_QOS_prv_data *pdata);

static void DWC_ETH_QOS_setup_timer(struct DWC_ETH_QOS_prv_data *pdata);

static void DWC_ETH_QOS_pg_run_test(struct DWC_ETH_QOS_prv_data *pdata);

static void DWC_ETH_QOS_print_pg_struct(struct DWC_ETH_QOS_prv_data *pdata);

static void DWC_ETH_QOS_pg_set_config(struct DWC_ETH_QOS_prv_data *pdata,
					struct ifr_data_struct *req);

static void DWC_ETH_QOS_pg_get_result(struct DWC_ETH_QOS_prv_data *pdata,
					struct ifr_data_struct *req);
#endif
