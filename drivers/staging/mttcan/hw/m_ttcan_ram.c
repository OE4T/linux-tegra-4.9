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
#include "m_ttcan.h"

int ttcan_read_txevt_ram(struct ttcan_controller *ttcan, u32 read_addr,
				struct mttcan_tx_evt_element *txevt)
{
	void __iomem *msg_addr = read_addr + ttcan->mram_sa.virt_base -
		ttcan->mram_sa.base;
	if (!txevt)
		return -1;

	txevt->f0 = readl(msg_addr);
	txevt->f1 = readl(msg_addr + CAN_WORD_IN_BYTES);

	return 0;
}

int ttcan_read_rx_msg_ram(struct ttcan_controller *ttcan, u32 read_addrs,
			  struct ttcanfd_frame *ttcanfd)
{
	int i = 0, byte_index;
	u32 msg_data;
	void __iomem *addr_in_msg_ram = read_addrs + ttcan->mram_sa.virt_base -
	    ttcan->mram_sa.base;
	int bytes_to_read = CANFD_MAX_DLEN;

	if (!ttcanfd)
		return -1;

	ttcanfd->flags |= CAN_DIR_RX;

	while (bytes_to_read) {
		msg_data =
		    readl(addr_in_msg_ram + (i * CAN_WORD_IN_BYTES));

		switch (i) {
		case 0:
			ttcanfd->can_id = 0;
			ttcanfd->flags = 0;

			if (msg_data & RX_BUF_XTD) {
				/* Extended Frame */
				ttcanfd->can_id =  CAN_FMT |
				    ((msg_data & RX_BUF_EXTID_MASK) &
				   CAN_EXT_ID_MASK);
			} else {
				ttcanfd->can_id =
					((msg_data & RX_BUF_STDID_MASK) >>
					RX_BUF_STDID_SHIFT) & CAN_STD_ID_MASK;
			}

			if (msg_data & RX_BUF_RTR)
				ttcanfd->can_id |= CAN_RTR;

			if (msg_data & RX_BUF_ESI)
				ttcanfd->flags |= CAN_ESI_FLAG;
			break;
		case 1:
			ttcanfd->d_len =
			    ttcan_dlc2len((msg_data & RX_BUF_DLC_MASK)
					  >> RX_BUF_DLC_SHIFT);
			bytes_to_read = ttcanfd->d_len;

			if (msg_data & RX_BUF_FDF)
				ttcanfd->flags |= CAN_FD_FLAG;

			if (msg_data & RX_BUF_BRS)
				ttcanfd->flags |= CAN_BRS_FLAG;

			break;
		default:
			byte_index = (i - 2) * CAN_WORD_IN_BYTES;
			switch	(bytes_to_read) {
			default:
			case 4:
				ttcanfd->data[byte_index + 3] =
					(msg_data >> 24) & 0xFF;
			case 3:
				ttcanfd->data[byte_index + 2] =
					(msg_data >> 16) & 0xFF;
			case 2:
				ttcanfd->data[byte_index + 1] =
					(msg_data >> 8) & 0xFF;
			case 1:
				ttcanfd->data[byte_index + 0] =
					 msg_data & 0xFF;
			}

			if (bytes_to_read >= 4)
				bytes_to_read -= 4;
			else
				bytes_to_read = 0;
			pr_debug("%s addr %p received 0x%x\n", __func__,
				addr_in_msg_ram +
				(i * CAN_WORD_IN_BYTES), msg_data);
			break;
		}
		i++;
	}
	pr_info("%s:received ID(0x%x) %s %s(%s)\n", __func__,
		(ttcanfd->can_id & CAN_FMT) ?
			(ttcanfd->can_id & CAN_EXT_ID_MASK) :
			(ttcanfd->can_id & CAN_STD_ID_MASK),
		(ttcanfd->can_id & CAN_FMT) ? "XTD" : "STD",
		(ttcanfd->flags & CAN_FD_FLAG) ? "FD" : "NON-FD",
		(ttcanfd->flags & CAN_BRS_FLAG) ? "BRS" : "NOBRS");

	return i;
}

int ttcan_write_tx_msg_ram(struct ttcan_controller *ttcan, u32 write_addrs,
			   struct ttcanfd_frame *ttcanfd, u8 index)
{
	u32 msg_data, idx;
	int bytes_to_write;
	void __iomem *addr_in_msg_ram = write_addrs + ttcan->mram_sa.virt_base -
	    ttcan->mram_sa.base;

	/* T0 */
	if (ttcanfd->can_id & CAN_FMT)
		msg_data = (ttcanfd->can_id & CAN_EXT_ID_MASK) | TX_BUF_XTD;
	else
		msg_data =
		    (ttcanfd->can_id & CAN_STD_ID_MASK) << TX_BUF_STDID_SHIFT;

	if (ttcanfd->can_id & CAN_RTR)
		msg_data |= TX_BUF_RTR;

	/*  This flag is ORed with error passive flag while sending */
	if (ttcanfd->flags & CAN_ESI_FLAG)
		msg_data |= TX_BUF_ESI;

	pr_info("T0: addr %p msg %x\n", addr_in_msg_ram, msg_data);
	writel(msg_data, addr_in_msg_ram);

	/* T1 */
	msg_data =
	    (ttcan_len2dlc(ttcanfd->d_len) << TX_BUF_DLC_SHIFT) &
	    TX_BUF_DLC_MASK;

	if (ttcan->tx_config.evt_q_size)
		msg_data |= TX_BUF_EFC;

	if (ttcanfd->flags & CAN_FD_FLAG)
		msg_data |= TX_BUF_FDF;

	if (ttcanfd->flags & CAN_BRS_FLAG)
		msg_data |= TX_BUF_BRS;

	msg_data |= index <<  TX_BUF_MM_SHIFT;

	pr_info("%s:buf_id(%d):- %s(%s)\n", __func__, index,
		(ttcanfd->flags & CAN_FD_FLAG) ? "FD" : "NON-FD",
		(ttcanfd->flags & CAN_BRS_FLAG) ? "BRS" : "NOBRS");

	pr_debug("T1: addr %p msg %x\n", (addr_in_msg_ram+CAN_WORD_IN_BYTES),
		msg_data);
	writel(msg_data, (addr_in_msg_ram + CAN_WORD_IN_BYTES));

	bytes_to_write = ttcanfd->d_len;

	idx = 0;

	while (bytes_to_write > 0) {
		msg_data = 0;
		switch (bytes_to_write) {
		default:
		case 4:
			msg_data = ttcanfd->data[idx + 3] << 24;
		case 3:
			msg_data += ttcanfd->data[idx + 2] << 16;
		case 2:
			msg_data += ttcanfd->data[idx + 1] << 8;
		case 1:
			msg_data += ttcanfd->data[idx + 0] << 0;
		}

		pr_debug("T2: addr %p msg %x\n", (addr_in_msg_ram +
				(((idx >> 2) + 2) * CAN_WORD_IN_BYTES)),
				msg_data);
		writel(msg_data, (addr_in_msg_ram +
		       (((idx >> 2) + 2) * CAN_WORD_IN_BYTES)));
		idx += 4;
		bytes_to_write -= 4;
	}

	return idx;
}

void ttcan_set_std_id_filter(struct ttcan_controller *ttcan, int filter_index,
			     u8 sft, u8 sfec, u32 sfid1, u32 sfid2)
{
	u32 filter_elem = 0;
	void __iomem *filter_addr =
	    ttcan->mram_sa.virt_base + ttcan->mram_sa.sidfc_flssa -
	    ttcan->mram_sa.base;
	u32 filter_offset =
	    (filter_index * STD_ID_FILTER_ELEM_SIZE_WORD * CAN_WORD_IN_BYTES);

	filter_elem = (sfid2 << MTT_STD_FLTR_SFID2_SHIFT) &
		MTT_STD_FLTR_SFID2_MASK;
	filter_elem |= (sfid1 << MTT_STD_FLTR_SFID1_SHIFT) &
		MTT_STD_FLTR_SFID1_MASK;
	filter_elem |= (sfec <<  MTT_STD_FLTR_SFEC_SHIFT) &
		MTT_STD_FLTR_SFEC_MASK;
	filter_elem |= (sft << MTT_STD_FLTR_SFT_SHIFT) & MTT_STD_FLTR_SFT_MASK;

	pr_debug("%s %p\n", __func__, (filter_addr + filter_offset));
	writel(filter_elem, (void __iomem *)(filter_addr + filter_offset));
}

void ttcan_reset_std_id_filter(struct ttcan_controller *ttcan, u32 list_size)
{
	int idx;
	void __iomem *filter_addr = ttcan->mram_sa.virt_base +
					ttcan->mram_sa.sidfc_flssa -
					ttcan->mram_sa.base;
	for (idx = 0; idx < list_size; idx++) {
		u32 offset =
			(idx * STD_ID_FILTER_ELEM_SIZE_WORD) << 2;
		writel(0, (void __iomem *)(filter_addr + offset));
	}
}

u32 ttcan_get_std_id_filter(struct ttcan_controller *ttcan, int idx)
{
	void __iomem *filter_addr = ttcan->mram_sa.virt_base +
					ttcan->mram_sa.sidfc_flssa -
					ttcan->mram_sa.base;

	u32 filter_offset = idx * STD_ID_FILTER_ELEM_SIZE_WORD << 2;
	return readl(filter_addr + filter_offset);
}

void ttcan_reset_xtd_id_filter(struct ttcan_controller *ttcan, u32 list_size)
{
	int idx;
	void __iomem *filter_addr = ttcan->mram_sa.virt_base +
					ttcan->mram_sa.xidfc_flesa -
					ttcan->mram_sa.base;

	for (idx = 0; idx < list_size; idx++) {
		u32 offset = (idx * EXT_ID_FILTER_ELEM_SIZE_WORD) << 2;

		writel(0, (void __iomem *)(filter_addr + offset));
		writel(0, (void __iomem *)(filter_addr + offset +
			CAN_WORD_IN_BYTES));
	}
}

void ttcan_reset_trigger_mem(struct ttcan_controller *ttcan, u32 list_size)
{
	int idx;
	void __iomem *filter_addr = ttcan->mram_sa.virt_base +
					ttcan->mram_sa.tmc_tmsa -
					ttcan->mram_sa.base;

	for (idx = 0; idx < list_size; idx++) {
		u32 offset = (idx * TRIG_ELEM_SIZE_WORD) << 2;

		writel(0, (void __iomem *)(filter_addr + offset));
		writel(0, (void __iomem *)(filter_addr + offset +
					CAN_WORD_IN_BYTES));
	}
}

void ttcan_set_xtd_id_filter(struct ttcan_controller *ttcan, int filter_index,
			     u8 eft, u8 efec, u32 efid1, u32 efid2)
{
	struct mttcan_xtd_id_filt_element xfilter_elem;
	void __iomem *filter_addr =
	    ttcan->mram_sa.virt_base + ttcan->mram_sa.xidfc_flesa -
	    ttcan->mram_sa.base;
	u32 filter_offset =
	    (filter_index * EXT_ID_FILTER_ELEM_SIZE_WORD * CAN_WORD_IN_BYTES);

	xfilter_elem.f0 = (efec << MTT_XTD_FLTR_F0_EFEC_SHIFT) &
		MTT_XTD_FLTR_F0_EFEC_MASK;
	xfilter_elem.f0 |= (efid1 << MTT_XTD_FLTR_F0_EFID1_SHIFT) &
		MTT_XTD_FLTR_F0_EFID1_MASK;
	xfilter_elem.f1 = (eft << MTT_XTD_FLTR_F1_EFT_SHIFT) &
		MTT_XTD_FLTR_F1_EFT_MASK;
	xfilter_elem.f1 = (efid2 << MTT_XTD_FLTR_F1_EFID2_SHIFT) &
		MTT_XTD_FLTR_F1_EFID2_MASK;

	writel(xfilter_elem.f0,
	       (void __iomem *)(filter_addr + filter_offset));
	writel(xfilter_elem.f1,
	       (void __iomem *)(filter_addr + filter_offset +
				CAN_WORD_IN_BYTES));
	pr_debug("%s %x %p\n", __func__, xfilter_elem.f0,
			(filter_addr + filter_offset));
	pr_debug("%s %x %p\n", __func__, xfilter_elem.f1,
			(filter_addr + filter_offset + CAN_WORD_IN_BYTES));
}

u64 ttcan_get_xtd_id_filter(struct ttcan_controller *ttcan, int idx)
{
	u64 xtd;
	void __iomem *filter_addr =
	    ttcan->mram_sa.virt_base + ttcan->mram_sa.xidfc_flesa -
	    ttcan->mram_sa.base;
	u32 offset = idx * EXT_ID_FILTER_ELEM_SIZE_WORD << 2;

	xtd = ((u64) readl(filter_addr + offset + CAN_WORD_IN_BYTES)) << 32;
	xtd |= readl(filter_addr + offset);
	return xtd;
}

u64 ttcan_get_trigger_mem(struct ttcan_controller *ttcan, int idx)
{
	u64 trig;
	void __iomem *addr =
	    ttcan->mram_sa.virt_base + ttcan->mram_sa.tmc_tmsa -
	    ttcan->mram_sa.base;
	u32 offset = idx * TRIG_ELEM_SIZE_WORD << 2;

	trig = ((u64) readl(addr + offset + CAN_WORD_IN_BYTES)) << 32;
	trig |= readl(addr + offset);
	return trig;
}

int ttcan_set_trigger_mem(struct ttcan_controller *ttcan, int trig_index,
			  u16 time_mark, u16 cycle_code, u8 tmin, u8 tmex,
			  u16 trig_type, u8 filter_type,
			  u8 mesg_num)
{
	struct mttcan_trig_mem_element trig_elem;
	void __iomem *trig_mem_addr =
	    ttcan->mram_sa.virt_base + ttcan->mram_sa.tmc_tmsa -
	    ttcan->mram_sa.base;
	u32 trig_mem_offset =
	    (trig_index * TRIG_ELEM_SIZE_WORD * CAN_WORD_IN_BYTES);

	if (trig_index > 63) {
		pr_err("%s: Incorrect Index\n", __func__);
		return -1;
	}

	if (cycle_code > 127) {
		pr_err("%s: Invalid cycle code\n", __func__);
		return -1;
	}

	/* TBD: ASC is disabled - Hardcoded */
	/* Mesg. Status Count is set 0 */
	trig_elem.f0 = (time_mark << MTT_TRIG_ELE_F0_TM_SHIFT) &
		MTT_TRIG_ELE_F0_TM_MASK;
	trig_elem.f0 |= cycle_code << MTT_TRIG_ELE_F0_CC_SHIFT &
		MTT_TRIG_ELE_F0_CC_SHIFT;
	/* ASC = 0 */;
	trig_elem.f0 |= (tmin <<  MTT_TRIG_ELE_F0_TMIN_SHIFT) &
		MTT_TRIG_ELE_F0_TMIN_MASK;
	trig_elem.f0 |= (tmex << MTT_TRIG_ELE_F0_TMEX_SHIFT) &
		MTT_TRIG_ELE_F0_TMEX_MASK;
	trig_elem.f0 |= (trig_type << MTT_TRIG_ELE_F0_TYPE_SHIFT) &
		MTT_TRIG_ELE_F0_TYPE_MASK;
	trig_elem.f1 = (filter_type << MTT_TRIG_ELE_F1_FTYPE_SHIFT) &
		MTT_TRIG_ELE_F1_FTYPE_MASK;
	trig_elem.f1 |= (mesg_num << MTT_TRIG_ELE_F1_MNR_SHIFT) &
		MTT_TRIG_ELE_F1_MNR_MASK;

	writel(trig_elem.f0,
	       (void __iomem *)(trig_mem_addr + trig_mem_offset));
	writel(trig_elem.f1,
	       (void __iomem *)(trig_mem_addr + trig_mem_offset +
				CAN_WORD_IN_BYTES));

	return 0;
}

int ttcan_mesg_ram_config(struct ttcan_controller *ttcan)
{
	u32 base_addr = ttcan->mram_sa.base;


	if ((REL_TMC_END >= RAM_BYTES_PER_MTT_CAN + base_addr)) {
		pr_err("%s: Incorrect config for Message RAM\n", __func__);
		return -EINVAL;
	}

	ttcan->mram_sa.sidfc_flssa = base_addr + REL_SIDFC_FLSSA;
	ttcan->mram_sa.xidfc_flesa = base_addr + REL_XIDFC_FLESA;
	ttcan->mram_sa.rxf0c_f0sa = base_addr + REL_RXF0C_F0SA;
	ttcan->mram_sa.rxf1c_f1sa = base_addr + REL_RXF1C_F1SA;
	ttcan->mram_sa.rxbc_rbsa = base_addr + REL_RXBC_RBSA;
	ttcan->mram_sa.txefc_efsa = base_addr + REL_TXEFC_EFSA;
	ttcan->mram_sa.txbc_tbsa = base_addr + REL_TXBC_TBSA;
	ttcan->mram_sa.tmc_tmsa = base_addr + REL_TMC_TMSA;

	pr_info(" Message RAM Configuration\n"
		"\t| base addr   |%x|\n\t| sidfc_flssa |%x|\n\t| xidfc_flesa |%x|\n"
		"\t| rxf0c_f0sa  |%x|\n\t| rxf1c_f1sa  |%x|\n\t| rxbc_rbsa   |%x|\n"
		"\t| txefc_efsa  |%x|\n\t| txbc_tbsa   |%x|\n\t| tmc_tmsa    |%x|\n",
		base_addr, ttcan->mram_sa.sidfc_flssa,
		ttcan->mram_sa.xidfc_flesa, ttcan->mram_sa.rxf0c_f0sa,
		ttcan->mram_sa.rxf1c_f1sa, ttcan->mram_sa.rxbc_rbsa,
		ttcan->mram_sa.txefc_efsa, ttcan->mram_sa.txbc_tbsa,
		ttcan->mram_sa.tmc_tmsa);
	return 0;
}
