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
 *
 * ========================================================================= */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
/* for socket programming */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/if.h>		/* for struct ifreq */
#include <linux/sockios.h>	/* for IOCTL no */
#include <linux/ethtool.h>


#include "DWC_ETH_QOS_yapphdr.h"

static unsigned int tx_queue_count = 1;
static unsigned int rx_queue_count = 1;
static unsigned int connected_speed = SPEED_100;

static char *exe_name;
static char *if_name;
static int print_all;

static int open_socket(char *ifname)
{
	int sockfd;

	sockfd = socket(PF_INET, SOCK_STREAM, 0);
	return sockfd;
}

static void close_socket(int sockfd)
{
	close(sockfd);
}

unsigned int wakeup_filter_config[] = {
	/* for filter 0 CRC is computed on 0 - 7 bytes from offset */
	0x000000ff,

	/* for filter 1 CRC is computed on 0 - 7 bytes from offset */
	0x000000ff,

	/* for filter 2 CRC is computed on 0 - 7 bytes from offset */
	0x000000ff,

	/* for filter 3 CRC is computed on 0 - 31 bytes from offset */
	0x000000ff,

	/* filter 0, 1 independently enabled and would apply for
	 * unicast packet only filter 3, 2 combined as,
	 * "Filter-3 pattern AND NOT Filter-2 pattern" */
	0x03050101,

	/* filter 3, 2, 1 and 0 offset is 50, 58, 66, 74 bytes
	 * from start */
	0x4a423a32,

	/* pattern for filter 1 and 0, "0x55", "11", repeated 8 times */
	0xe7b77eed,

	/* pattern for filter 3 and 4, "0x44", "33", repeated 8 times */
	0x9b8a5506,
};

int populate_filter_frames(struct ifr_data_struct *data)
{
	int i;

	for (i = 0; i < DWC_ETH_QOS_RWK_FILTER_LENGTH; i++)
		data->rwk_filter_values[i] = wakeup_filter_config[i];

	data->rwk_filter_length = DWC_ETH_QOS_RWK_FILTER_LENGTH;

	return 0;
}

static int DWC_ETH_QOS_get_rx_qcnt(int sockfd, char *ifname)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int ret = 0;

	data.cmd = DWC_ETH_QOS_GET_RX_QCNT;
	data.qInx = 0;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Rx queue count is %d\n", data.qInx);

	return data.qInx;
}

static int DWC_ETH_QOS_get_tx_qcnt(int sockfd, char *ifname)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int ret = 0;

	data.cmd = DWC_ETH_QOS_GET_TX_QCNT;
	data.qInx = 0;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Tx queue count is %d\n", data.qInx);

	return data.qInx;
}

static int DWC_ETH_QOS_get_connected_speed(int sockfd, char *ifname)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int ret = 0;

	data.cmd = DWC_ETH_QOS_GET_CONNECTED_SPEED;
	data.qInx = 0;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Connected speed is %dMBps\n", data.connected_speed);

	return data.connected_speed;
}

static unsigned int DWC_ETH_QOS_get_dcb_queue_weights(unsigned int dcb_algorithm,
		unsigned int percent_weight, unsigned int max_frame_size)
{
	unsigned int weight = 0;

	switch (dcb_algorithm) {
	case eDWC_ETH_QOS_DCB_WRR:
		weight = percent_weight;
		break;

	case eDWC_ETH_QOS_DCB_DWRR:
		weight = (percent_weight * max_frame_size);
		break;

	case eDWC_ETH_QOS_DCB_WFQ:
		weight = (DWC_ETH_QOS_MAX_WFQ_WEIGHT/(percent_weight+1));
		break;

	case eDWC_ETH_QOS_DCB_SP:
		/* queue weights are ignored with this algorithm */
		break;
	}

	return weight;
}

static unsigned int get_idle_slope(unsigned char bw)
{
	unsigned int multiplier = 1;
	unsigned int idle_slope = 0;

	if (connected_speed == SPEED_1000)
		multiplier = 2;

	/*
	 * idleslope = ((bandwidth/100) * (4 [for MII] or 8 [for GMII])) * 1024
	 * 1024 is multiplied for normalizing the calculated value
	 */
	idle_slope = (((4 * multiplier * bw) * 1024)/100);

	return idle_slope;
}

static unsigned int get_send_slope(unsigned char bw)
{
	unsigned int multiplier = 1;
	unsigned int idle_slope = 0;
	unsigned int send_slope = 0;

	if (connected_speed == SPEED_1000)
		multiplier = 2;

	/*
	 * sendslope = (((100 - bandwidth)/100) * (4 [for MII] or 8 [for GMII])) * 1024
	 * 1024 is multiplied for normalizing the calculated value
	 * OR
	 * sendslope = (1024 * (4 [for MII] or 8 [for GMII]) - idle_slope)
	 */
	//send_slope = (((4 * multiplier * (100 - bw)) * 1024)/100);
	idle_slope = (((4 * multiplier * bw) * 1024)/100);
	send_slope = ((4 * multiplier * 1024)- idle_slope);

	return send_slope;
}

static unsigned int get_hi_credit(unsigned char bw)
{
	unsigned int hi_credit = 0;
	typedef unsigned long long u64;
	hi_credit = (u64)(((u64)DWC_ETH_QOS_MAX_INT_FRAME_SIZE * (u64)bw * 1024 * 8))/100;
	return hi_credit;
}

static unsigned int get_low_credit(unsigned char bw)
{
	unsigned int hi_credit = 0;
	int low_credit = 0;
    typedef unsigned long long u64;
	hi_credit = (u64)(((u64)DWC_ETH_QOS_MAX_INT_FRAME_SIZE * (u64)bw * 1024 * 8))/100;
	low_credit =  -((u64)(((u64)DWC_ETH_QOS_MAX_INT_FRAME_SIZE * (100 - bw)) * 1024 * 8)/100);

	return low_credit;
}

static int powerup_device(int sockfd, char *ifname, char *parameter)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int ret = 0;

	if (strcmp("magic", parameter) == 0)
		data.cmd = DWC_ETH_QOS_POWERUP_MAGIC_CMD;
	else if (strcmp("remote_wakeup", parameter) == 0)
		data.cmd = DWC_ETH_QOS_POWERUP_REMOTE_WAKEUP_CMD;

	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;
	data.qInx = 0; /* Not used */

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0) {
		printf("IOCTL Error\n");
		switch (data.command_error) {
		case DWC_ETH_QOS_NO_HW_SUPPORT:
			printf("No hardware support present for %s\n",
			    parameter);
			break;
		case DWC_ETH_QOS_CONFIG_FAIL:
			printf("Powerup already done\n");
			break;
		}
	} else
		printf("Got the device out of power down mode for '%s'\n",
		       parameter);

	return ret;
}

static int powerdown_device(int sockfd, char *ifname, char *parameter)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int ret = 0;
	char *str = NULL;

	if (strcmp("magic", parameter) == 0) {
		str = "magic";
		data.cmd = DWC_ETH_QOS_POWERDOWN_MAGIC_CMD;
	} else if (strcmp("remote_wakeup", parameter) == 0) {
		str = "remote wakeup";
		data.cmd = DWC_ETH_QOS_POWERDOWN_REMOTE_WAKEUP_CMD;
		populate_filter_frames(&data);
	}

	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;
	data.qInx = 0; /* Not used */

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0) {
		printf("IOCTL Error\n");
		switch (data.command_error) {
		case DWC_ETH_QOS_NO_HW_SUPPORT:
			printf("No hardware support present for %s\n", str);
			break;
		case DWC_ETH_QOS_CONFIG_FAIL:
			printf("Powerdown already done\n");
			break;
		}
	} else {
		printf("Put the device in power down mode\n");
		printf("Device will now power up on manual 'powerup' with"\
		    " '%s' or on receiving a %s packet\n",
		     parameter, str);
	}

	return ret;
}

static int config_rx_threshold(int sockfd, char *ifname, char *argv1,
				char *argv2)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	unsigned int qInx = atoi(argv1);
	int option = atoi(argv2);
	int ret = 0;

	if ((qInx > DWC_ETH_QOS_MAX_RX_QUEUE_CNT) || (qInx < 0)) {
		printf("Invalid channel no %d\n", qInx);
		return -1;
	}

	switch (option) {
	case 32:
		data.flags = DWC_ETH_QOS_RX_THRESHOLD_32;
		break;
	case 64:
		data.flags = DWC_ETH_QOS_RX_THRESHOLD_64;
		break;
	case 96:
		data.flags = DWC_ETH_QOS_RX_THRESHOLD_96;
		break;
	case 128:
		data.flags = DWC_ETH_QOS_RX_THRESHOLD_128;
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_RX_THRESHOLD_CMD;
	data.qInx = qInx;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Successfully configured RX threshold to %d bytes\n",
		       option);

	return ret;
}

static int config_tx_threshold(int sockfd, char *ifname, char *argv1,
				char *argv2)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	unsigned int qInx = atoi(argv1);
	int option = atoi(argv2);
	int ret = 0;

	if ((qInx > DWC_ETH_QOS_MAX_TX_QUEUE_CNT) || (qInx < 0)) {
		printf("Invalid channel no %d\n", qInx);
		return -1;
	}

	switch (option) {
	case 32:
		data.flags = DWC_ETH_QOS_TX_THRESHOLD_32;
		break;
	case 64:
		data.flags = DWC_ETH_QOS_TX_THRESHOLD_64;
		break;
	case 96:
		data.flags = DWC_ETH_QOS_TX_THRESHOLD_96;
		break;
	case 128:
		data.flags = DWC_ETH_QOS_TX_THRESHOLD_128;
		break;
	case 192:
		data.flags = DWC_ETH_QOS_TX_THRESHOLD_192;
		break;
	case 256:
		data.flags = DWC_ETH_QOS_TX_THRESHOLD_256;
		break;
	case 384:
		data.flags = DWC_ETH_QOS_TX_THRESHOLD_384;
		break;
	case 512:
		data.flags = DWC_ETH_QOS_TX_THRESHOLD_512;
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_TX_THRESHOLD_CMD;
	data.qInx = qInx;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Successfully configured TX threshold to %d bytes\n",
		       option);

	return ret;
}

static int config_rsf(int sockfd, char *ifname, char *argv1, char *argv2)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	unsigned int qInx = atoi(argv1);
	int option = atoi(argv2);
	int ret = 0;

	if ((qInx > DWC_ETH_QOS_MAX_RX_QUEUE_CNT) || (qInx < 0)) {
		printf("Invalid channel no %d\n", qInx);
		return -1;
	}

	switch (option) {
	case 0:
		data.flags = DWC_ETH_QOS_RSF_DISABLE;
		break;
	case 1:
		data.flags = DWC_ETH_QOS_RSF_ENABLE;
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_RSF_CMD;
	data.qInx = qInx;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Successfully %s Receive Store and Forward Mode\n",
		       (option ? "ENABLED" : "DISABLED"));

	return ret;
}

static int config_tsf(int sockfd, char *ifname, char *argv1, char *argv2)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	unsigned int qInx = atoi(argv1);
	int option = atoi(argv2);
	int ret = 0;

	if ((qInx > DWC_ETH_QOS_MAX_TX_QUEUE_CNT) || (qInx < 0)) {
		printf("Invalid channel no %d\n", qInx);
		return -1;
	}

	switch (option) {
	case 0:
		data.flags = DWC_ETH_QOS_TSF_DISABLE;
		break;
	case 1:
		data.flags = DWC_ETH_QOS_TSF_ENABLE;
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_TSF_CMD;
	data.qInx = qInx;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Successfully %s Transmit Store and Forward Mode\n",
		       (option ? "ENABLED" : "DISABLED"));

	return ret;
}

static int config_osf(int sockfd, char *ifname, char *argv1, char *argv2)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	unsigned int qInx = atoi(argv1);
	int option = atoi(argv2);
	int ret = 0;

	if ((qInx > DWC_ETH_QOS_MAX_TX_QUEUE_CNT) || (qInx < 0)) {
		printf("Invalid channel no %d\n", qInx);
		return -1;
	}

	switch (option) {
	case 0:
		data.flags = DWC_ETH_QOS_OSF_DISABLE;
		break;
	case 1:
		data.flags = DWC_ETH_QOS_OSF_ENABLE;
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_OSF_CMD;
	data.qInx = qInx;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf
		    ("Successfully %s Transmit Operate on Second Frame Mode\n",
		     (option ? "ENABLED" : "DISABLED"));

	return ret;
}

static int config_incrx_incr(int sockfd, char *ifname, char *argv)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int option = atoi(argv);
	int ret = 0;

	switch (option) {
	case 0:
		data.flags = DWC_ETH_QOS_INCRX_ENABLE;
		break;
	case 1:
		data.flags = DWC_ETH_QOS_INCR_ENABLE;
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_INCR_INCRX_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Successfully configured device in %s mode\n",
		       option ? "INCRX" : "INCR");

	return ret;
}

static int config_rx_pbl(int sockfd, char *ifname, char *argv1, char *argv2)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	unsigned int qInx = atoi(argv1);
	int option = atoi(argv2);
	int ret = 0;

	if ((qInx > DWC_ETH_QOS_MAX_RX_QUEUE_CNT) || (qInx < 0)) {
		printf("Invalid channel no %d\n", qInx);
		return -1;
	}

	switch (option) {
	case DWC_ETH_QOS_PBL_1:
	case DWC_ETH_QOS_PBL_2:
	case DWC_ETH_QOS_PBL_4:
	case DWC_ETH_QOS_PBL_8:
	case DWC_ETH_QOS_PBL_16:
	case DWC_ETH_QOS_PBL_32:
	case DWC_ETH_QOS_PBL_64:
	case DWC_ETH_QOS_PBL_128:
	case DWC_ETH_QOS_PBL_256:
		data.flags = option;
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_RX_PBL_CMD;
	data.qInx = qInx;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Successfully configured RX PBL to %d\n", option);

	return ret;
}

static int config_tx_pbl(int sockfd, char *ifname, char *argv1, char *argv2)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	unsigned int qInx = atoi(argv1);
	int option = atoi(argv2);
	int ret = 0;

	if ((qInx > DWC_ETH_QOS_MAX_TX_QUEUE_CNT) || (qInx < 0)) {
		printf("Invalid channel no %d\n", qInx);
		return -1;
	}

	switch (option) {
	case DWC_ETH_QOS_PBL_1:
	case DWC_ETH_QOS_PBL_2:
	case DWC_ETH_QOS_PBL_4:
	case DWC_ETH_QOS_PBL_8:
	case DWC_ETH_QOS_PBL_16:
	case DWC_ETH_QOS_PBL_32:
	case DWC_ETH_QOS_PBL_64:
	case DWC_ETH_QOS_PBL_128:
	case DWC_ETH_QOS_PBL_256:
		data.flags = option;
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_TX_PBL_CMD;
	data.qInx = qInx;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Successfully configured TX PBL to %d\n", option);

	return ret;
}

/* return 0 on success and -ve on failure */
#if 0
static int rx_outer_vlan_strip(int sockfd, char *ifname, char *argv1)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int option = atoi(argv1);
	int ret = 0;
	char *string = NULL;

	switch (option) {
	case 0:
		data.flags = DWC_ETH_QOS_RX_NO_VLAN_STRIP;
		string = "not to strip outer RX VLAN Tag";
		break;
	case 1:
		data.flags = DWC_ETH_QOS_RX_VLAN_STRIP_IF_FILTER_PASS;
		string = "strip outer RX VLAN Tag if it passes VLAN filter";
		break;
	case 2:
		data.flags = DWC_ETH_QOS_RX_VLAN_STRIP_IF_FILTER_FAIL;
		string = "strip outer RX VLAN Tag if it fail VLAN filter";
		break;
	case 3:
		data.flags = DWC_ETH_QOS_RX_VLAN_STRIP_ALWAYS;
		string = "strip outer RX VLAN Tag always";
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_RX_OUTER_VLAN_STRIPPING_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Successfully configured device to %s\n", string);

	return ret;
}

/* return 0 on success and -ve on failure */
static int rx_inner_vlan_strip(int sockfd, char *ifname, char *argv1)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int option = atoi(argv1);
	int ret = 0;
	char *string = NULL;

	switch (option) {
	case 0:
		data.flags = DWC_ETH_QOS_RX_NO_VLAN_STRIP;
		string = "not to strip inner RX VLAN Tag";
		break;
	case 1:
		data.flags = DWC_ETH_QOS_RX_VLAN_STRIP_IF_FILTER_PASS;
		string = "strip inner RX VLAN Tag if it passes VLAN filter";
		break;
	case 2:
		data.flags = DWC_ETH_QOS_RX_VLAN_STRIP_IF_FILTER_FAIL;
		string = "strip inner RX VLAN Tag if it fail VLAN filter";
		break;
	case 3:
		data.flags = DWC_ETH_QOS_RX_VLAN_STRIP_ALWAYS;
		string = "strip inner RX VLAN Tag always";
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_RX_INNER_VLAN_STRIPPING_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Successfully configured device to %s\n", string);

	return ret;
}
/* return 0 on success and -ve on failure */
static int tx_vlan_ctrl_via_desc(int sockfd, char *ifname, char *argv1, char *argv2)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int qInx = atoi(argv1);
	int option = atoi(argv2);
	int ret = 0;
	char *string = NULL;

	switch (option) {
	case 0:
		data.flags = DWC_ETH_QOS_TX_VLAN_TAG_NONE;
		string = "not TX VLAN Tag deletion, insertion or replacement";
		break;
	case 1:
		data.flags = DWC_ETH_QOS_TX_VLAN_TAG_DELETE;
		string = "TX VLAN Tag deletion";
		break;
	case 2:
		data.flags = DWC_ETH_QOS_TX_VLAN_TAG_INSERT;
		string = "TX VLAN Tag insertion";
		break;
	case 3:
		data.flags = DWC_ETH_QOS_TX_VLAN_TAG_REPLACE;
		string = "TX VLAN Tag replacement";
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_TX_VLAN_DESC_CMD;
	data.qInx = qInx;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Successfully configured device to"\
			" (per packet based) %s\n", string);

	return ret;
}

/* return 0 on success and -ve on failure */
static int tx_vlan_ctrl_via_reg(int sockfd, char *ifname, char *argv1, char *argv2)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int qInx = atoi(argv1);
	int option = atoi(argv2);
	int ret = 0;
	char *string = NULL;

	switch (option) {
	case 0:
		data.flags = DWC_ETH_QOS_TX_VLAN_TAG_NONE;
		string = "not TX VLAN Tag deletion, insertion or replacement";
		break;
	case 1:
		data.flags = DWC_ETH_QOS_TX_VLAN_TAG_DELETE;
		string = "TX VLAN Tag deletion";
		break;
	case 2:
		data.flags = DWC_ETH_QOS_TX_VLAN_TAG_INSERT;
		string = "TX VLAN Tag insertion";
		break;
	case 3:
		data.flags = DWC_ETH_QOS_TX_VLAN_TAG_REPLACE;
		string = "TX VLAN Tag replacement";
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_TX_VLAN_REG_CMD;
	data.qInx = qInx;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf
		    ("Successfully configured device to (register based) %s\n",
		     string);

	return ret;
}

#endif
/* return 0 on success and -ve on failure */
static int sa0_ctrl_via_desc(int sockfd, char *ifname, char *argv)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int option = atoi(argv);
	int ret = 0;
	char *string = NULL;

	switch (option) {
	case 0:
		data.flags = DWC_ETH_QOS_SA0_NONE;
		string = "Do not include the source address with value given"\
			" in the MAC Address 0 reg";
		break;
	case 1:
		data.flags = DWC_ETH_QOS_SA0_DESC_INSERT;
		string = "Include/Insert the source address with the value"\
			" given in the MAC Address 0 reg";
		break;
	case 2:
		data.flags = DWC_ETH_QOS_SA0_DESC_REPLACE;
		string = "Replace the source address with the value given in"\
			" the MAC Address 0 reg";
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_SA0_DESC_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Successfully configured device to"\
			" (per packet based) %s\n", string);

	return ret;
}

static int sa1_ctrl_via_desc(int sockfd, char *ifname, char *argv)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int option = atoi(argv);
	int ret = 0;
	char *string = NULL;

	switch (option) {
	case 0:
		data.flags = DWC_ETH_QOS_SA1_NONE;
		string = "Do not include the source address with value given"\
			" in the MAC Address 1 reg";
		break;
	case 1:
		data.flags = DWC_ETH_QOS_SA1_DESC_INSERT;
		string = "Include/Insert the source address with value given"\
			" in the MAC Address 1 reg";
		break;
	case 2:
		data.flags = DWC_ETH_QOS_SA1_DESC_REPLACE;
		string = "Replace the source address with value given in the"\
			" MAC Address 1 reg";
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_SA1_DESC_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Successfully configured device to"\
			" (per packet based) %s\n", string);

	return ret;
}

/* return 0 on success and -ve on failure */
static int sa0_ctrl_via_reg(int sockfd, char *ifname, char *argv)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int option = atoi(argv);
	int ret = 0;
	char *string = NULL;

	switch (option) {
	case 0:
		data.flags = DWC_ETH_QOS_SA0_NONE;
		string = "Do not include the source address with value given"\
			" in the MAC Address 0 reg";
		break;
	case 1:
		data.flags = DWC_ETH_QOS_SA0_REG_INSERT;
		string = "Include/Insert the source address with the value"\
			" given in the MAC Address 0 reg";
		break;
	case 2:
		data.flags = DWC_ETH_QOS_SA0_REG_REPLACE;
		string = "Replace the source address with the value given in"\
			" the MAC Address 0 reg";
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_SA0_REG_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf
		    ("Successfully configured device to (register based) %s\n",
		     string);

	return ret;
}

static int sa1_ctrl_via_reg(int sockfd, char *ifname, char *argv)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int option = atoi(argv);
	int ret = 0;
	char *string = NULL;

	switch (option) {
	case 0:
		data.flags = DWC_ETH_QOS_SA1_NONE;
		string = "Do not include the source address with value given"\
			" in the MAC Address 1 reg";
		break;
	case 1:
		data.flags = DWC_ETH_QOS_SA1_REG_INSERT;
		string = "Include/Insert the source address with value given"\
			" in the MAC Address 1 reg";
		break;
	case 2:
		data.flags = DWC_ETH_QOS_SA1_REG_REPLACE;
		string = "Replace the source address with value given in the"\
			" MAC Address 1 reg";
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_SA1_REG_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf
		    ("Successfully configured device to (register based) %s\n",
		     string);

	return ret;
}

static int program_dcb_algorithm(int sockfd, char *ifname, char *argv1,
		char *argv2, char *argv3, char *argv4)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	struct DWC_ETH_QOS_dcb_algorithm dcb_struct;
	int qInx = atoi(argv1);
	int algo = atoi(argv2);
	int percent_weight = atoi(argv3);
	int max_frame_size = atoi(argv4);
	int ret = 0;

	data.cmd = DWC_ETH_QOS_DCB_ALGORITHM;
	data.qInx = qInx;
	dcb_struct.qInx = qInx;
	dcb_struct.algorithm = algo;
	dcb_struct.weight = DWC_ETH_QOS_get_dcb_queue_weights(algo, percent_weight, max_frame_size);
	dcb_struct.op_mode = eDWC_ETH_QOS_QDCB;
	data.ptr = &dcb_struct;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Configured DCB Algorithm parameters successfully\n");

	return ret;
}

static int program_avb_algorithm(int sockfd, char *ifname, char *argv1,
		char *argv2, char *argv3, char *argv4)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	struct DWC_ETH_QOS_avb_algorithm avb_struct;
	int qInx = atoi(argv1);
	int algo = atoi(argv2);
	int bw = atoi(argv3);
	int cc = atoi(argv4);
	int ret = 0;

	data.cmd = DWC_ETH_QOS_AVB_ALGORITHM;
	data.qInx = qInx;
	avb_struct.qInx = qInx;
	avb_struct.algorithm = algo;
	avb_struct.cc = cc;
	avb_struct.idle_slope = get_idle_slope(bw);
	avb_struct.send_slope = get_send_slope(bw);
	avb_struct.hi_credit = get_hi_credit(bw);
	avb_struct.low_credit = get_low_credit(bw);
	avb_struct.op_mode = eDWC_ETH_QOS_QAVB;
	data.ptr = &avb_struct;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Configured AVB Algorithm parameters successfully\n");

	return ret;
}

static int setup_context_descriptor(int sockfd, char *ifname, char *argv1, char *argv2)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int qInx = atoi(argv1);
	int setup = atoi(argv2);
	int ret = 0;

	data.cmd = DWC_ETH_QOS_SETUP_CONTEXT_DESCRIPTOR;
	data.context_setup = setup;
	data.qInx = qInx;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Context descriptor configuration changed\n");

	return ret;
}

static int config_rx_split_hdr(int sockfd, char *ifname, char *argv1)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int option = atoi(argv1);
	int ret = 0;

	if (option)
		data.flags = DWC_ETH_QOS_RX_SPLIT_HDR_ENABLE;
	else
		data.flags = DWC_ETH_QOS_RX_SPLIT_HDR_DISABLE;

	data.cmd = DWC_ETH_QOS_RX_SPLIT_HDR_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0) {
		printf("IOCTL Error\n");
		switch (data.command_error) {
		case DWC_ETH_QOS_NO_HW_SUPPORT:
			printf("No hardware support present for Split header\n");
			break;
		case DWC_ETH_QOS_CONFIG_FAIL:
			printf("Rx Split header mode is already %s\n",
				(option ? "enabled" : "disable"));
			break;
		}
	}
	else
		printf("Successfully %s Rx Split header mode\n",
			(option ? "enabled" : "disabled"));

	return ret;
}

static int config_l3_l4_filter(int sockfd, char *ifname,
			char *enable_disable)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int option = atoi(enable_disable);
	int ret = 0;

	if (option)
		data.flags = DWC_ETH_QOS_L3_L4_FILTER_ENABLE;
	else
		data.flags = DWC_ETH_QOS_L3_L4_FILTER_DISABLE;

	data.cmd = DWC_ETH_QOS_L3_L4_FILTER_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0) {
		printf("IOCTL Error\n");
		switch (data.command_error) {
		case DWC_ETH_QOS_NO_HW_SUPPORT:
			printf("No hardware support present for L3/L4 filtering\n");
			break;
		case DWC_ETH_QOS_CONFIG_FAIL:
			printf("L3/L4 filtering is already %s\n",
				(option ? "ENABLED" : "DISABLED"));
			break;
		}
	}
	else
		printf("Successfully %s L3/L4 filtering\n",
			(option ? "ENABLED" : "DISABLED"));

	return ret;
}

static int check_for_valid_ipv4_addr(char *ip)
{
	int i = 0, ret = 0;
	char *token = NULL;

	token = strtok(ip, ".");
	if (token) {
		while(1) {
			token = strtok(NULL, ".");
			if (!token) {
				break;
			}
			i++;
		}
	}

	if (i != 3)
		ret = -1;

	return ret;
}

static int check_for_valid_ipv6_addr(char *ip)
{
	int i = 0, ret = 0;
	char *token = NULL;

	token = strtok(ip, ":");
	if (token) {
		while(1) {
			token = strtok(NULL, ":");
			if (!token) {
				break;
			}
			i++;
		}
	}

	if (i != 7)
		ret = -1;

	return ret;
}

static int config_ipv4_filters(int sockfd, char *ifname,
				char *filter_no,
				char *src_dst_addr_match,
				char *enable_disable,
				char *perfect_inverse_match,
				char *ip)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	struct DWC_ETH_QOS_l3_l4_filter l3_filter;
	char *tmp;
	int ret = 0;

	data.flags = atoi(enable_disable);
	data.cmd = DWC_ETH_QOS_IPV4_FILTERING_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	l3_filter.filter_no = atoi(filter_no);
	if (l3_filter.filter_no < 0 || l3_filter.filter_no > 7) {
		printf("Filter number should be between 0 to 7\n");
		return -1;
	}

	l3_filter.src_dst_addr_match = atoi(src_dst_addr_match);
	l3_filter.filter_enb_dis = atoi(enable_disable);
	if (l3_filter.filter_enb_dis) {
		l3_filter.perfect_inverse_match = atoi(perfect_inverse_match);

		if (!strcmp(ip, "0")) {
			memset(l3_filter.ip4_addr, 0, sizeof(l3_filter.ip4_addr));
		} else {
			tmp = strdup(ip);
			ret = check_for_valid_ipv4_addr(tmp);
			if (!ret) {
			  l3_filter.ip4_addr[0] = strtol(strtok(ip, "."), NULL, 10);
			  l3_filter.ip4_addr[1] = strtol(strtok(NULL, "."), NULL, 10);
			  l3_filter.ip4_addr[2] = strtol(strtok(NULL, "."), NULL, 10);
			  l3_filter.ip4_addr[3] = strtol(strtok(NULL, "."), NULL, 10);
			} else {
				printf("ERROR: Specify correct IPv4 addr(eq - 10.144.134.108)\n");
				return ret;
			}
		}
	} else {
		l3_filter.perfect_inverse_match = 0;
		memset(l3_filter.ip4_addr, 0, sizeof(l3_filter.ip4_addr));
	}
	data.ptr = &l3_filter;

	printf("%d.%d.%d.%d\n", l3_filter.ip4_addr[0], l3_filter.ip4_addr[1],
		l3_filter.ip4_addr[2], l3_filter.ip4_addr[3]);

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0) {
		printf("IOCTL Error\n");
		switch (data.command_error) {
		case DWC_ETH_QOS_NO_HW_SUPPORT:
			printf("No hardware support present for IPv4 addr filtering\n");
			break;
		default:
			printf("IPv4(L3) filtering is failed\n");
			break;
		}
	} else
		printf("Successfully %s IPv4 %s %s addressing filtering on %d filter\n",
			(l3_filter.filter_enb_dis ? "ENABLED" : "DISABLED"),
			(l3_filter.perfect_inverse_match ? "INVERSE" : "PERFECT"),
			(l3_filter.src_dst_addr_match ? "DESTINATION" : "SOURCE"),
			l3_filter.filter_no);

	return ret;
}

static int config_ipv6_filters(int sockfd, char *ifname,
				char *filter_no,
				char *src_dst_addr_match,
				char *enable_disable,
				char *perfect_inverse_match,
				char *ip)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	struct DWC_ETH_QOS_l3_l4_filter l3_filter;
	char *tmp;
	int ret = 0;

	data.flags = atoi(enable_disable);
	data.cmd = DWC_ETH_QOS_IPV6_FILTERING_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	l3_filter.filter_no = atoi(filter_no);
	if (l3_filter.filter_no < 0 || l3_filter.filter_no > 7) {
		printf("Filter number should be between 0 to 7\n");
		return -1;
	}

	l3_filter.src_dst_addr_match = atoi(src_dst_addr_match);
	l3_filter.filter_enb_dis = atoi(enable_disable);
	if (l3_filter.filter_enb_dis) {
		l3_filter.perfect_inverse_match = atoi(perfect_inverse_match);

		if (!strcmp(ip, "0")) {
			memset(l3_filter.ip6_addr, 0, sizeof(l3_filter.ip6_addr));
		} else {
			tmp = strdup(ip);
			ret = check_for_valid_ipv6_addr(tmp);
			if (!ret) {
			  l3_filter.ip6_addr[0] = strtol(strtok(ip, ":"), NULL, 16);
			  l3_filter.ip6_addr[1] = strtol(strtok(NULL, ":"), NULL, 16);
			  l3_filter.ip6_addr[2] = strtol(strtok(NULL, ":"), NULL, 16);
			  l3_filter.ip6_addr[3] = strtol(strtok(NULL, ":"), NULL, 16);
			  l3_filter.ip6_addr[4] = strtol(strtok(NULL, ":"), NULL, 16);
			  l3_filter.ip6_addr[5] = strtol(strtok(NULL, ":"), NULL, 16);
			  l3_filter.ip6_addr[6] = strtol(strtok(NULL, ":"), NULL, 16);
			  l3_filter.ip6_addr[7] = strtol(strtok(NULL, ":"), NULL, 16);
			} else {
				printf("ERROR: Specify correct IPv6 addr(eq - fe80:0:0:0:f2de:f1ff:fe58:de16)\n");
				return ret;
			}
		}
	} else {
		l3_filter.perfect_inverse_match = 0;
		memset(l3_filter.ip6_addr, 0, sizeof(l3_filter.ip6_addr));
	}
	data.ptr = &l3_filter;

	printf("%#x:%#x:%#x:%#x:%#x:%#x:%#x:%#x\n",
		l3_filter.ip6_addr[0], l3_filter.ip6_addr[1],
		l3_filter.ip6_addr[2], l3_filter.ip6_addr[3],
		l3_filter.ip6_addr[4], l3_filter.ip6_addr[5],
		l3_filter.ip6_addr[6], l3_filter.ip6_addr[7]);

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0) {
		printf("IOCTL Error\n");
		switch (data.command_error) {
		case DWC_ETH_QOS_NO_HW_SUPPORT:
			printf("No hardware support present for IPv6 addr filtering\n");
			break;
		default:
			printf("IPv6(L3) filtering is failed\n");
			break;
		}
	} else
		printf("Successfully %s IPv6 %s %s addressing filtering on %d filter\n",
			(l3_filter.filter_enb_dis ? "ENABLED" : "DISABLED"),
			(l3_filter.perfect_inverse_match ? "INVERSE" : "PERFECT"),
			(l3_filter.src_dst_addr_match ? "DESTINATION" : "SOURCE"),
			l3_filter.filter_no);

	return ret;
}

static int config_l4_filters(int sockfd, char *ifname,
				char *udp_tcp,
				char *filter_no,
				char *src_dst_addr_match,
				char *enable_disable,
				char *perfect_inverse_match,
				char *port)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	struct DWC_ETH_QOS_l3_l4_filter l4_filter;
	int ret = 0;

	data.flags = atoi(enable_disable);
	if (!strcmp(udp_tcp, "udp_filter"))
		data.cmd = DWC_ETH_QOS_UDP_FILTERING_CMD;
	else
		data.cmd = DWC_ETH_QOS_TCP_FILTERING_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	l4_filter.filter_no = atoi(filter_no);
	l4_filter.src_dst_addr_match = atoi(src_dst_addr_match);
	l4_filter.filter_enb_dis = atoi(enable_disable);
	if (l4_filter.filter_enb_dis) {
		l4_filter.perfect_inverse_match = atoi(perfect_inverse_match);

		if (!strcmp(port, "0")) {
			l4_filter.port_no = 0;
		} else {
			l4_filter.port_no = atoi(port);
		}
	} else {
		l4_filter.perfect_inverse_match = 0;
		l4_filter.port_no = 0;
	}
	data.ptr = &l4_filter;

	printf("%d\n", l4_filter.port_no);

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0) {
		printf("IOCTL Error\n");
		switch (data.command_error) {
		case DWC_ETH_QOS_NO_HW_SUPPORT:
			printf("No hardware support present for UDP port filtering\n");
			break;
		default:
			printf("UPD(L4) filtering is failed\n");
			break;
		}
	} else
		printf("Successfully %s %s %s %s port filtering on %d filter\n",
			(l4_filter.filter_enb_dis ? "ENABLED" : "DISABLED"),
			(data.cmd == DWC_ETH_QOS_UDP_FILTERING_CMD ? "UDP" : "TCP"),
			(l4_filter.perfect_inverse_match ? "INVERSE" : "PERFECT"),
			(l4_filter.src_dst_addr_match ? "DESTINATION" : "SOURCE"),
			l4_filter.filter_no);

	return ret;
}

static int config_vlan_filters(int sockfd, char *ifname,
				char *enable_disable,
				char *perfect_hash_filtering,
				char *perfect_inverse_match)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	struct DWC_ETH_QOS_vlan_filter vlan_filter;
	int ret = 0;

	data.flags = atoi(enable_disable);
	data.cmd = DWC_ETH_QOS_VLAN_FILTERING_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	vlan_filter.filter_enb_dis = atoi(enable_disable);
	if (vlan_filter.filter_enb_dis) {
		vlan_filter.perfect_hash = atoi(perfect_hash_filtering);
		vlan_filter.perfect_inverse_match =
				atoi(perfect_inverse_match);
	} else {
		vlan_filter.perfect_hash = 0;
		vlan_filter.perfect_inverse_match = 0;
	}
	data.ptr = &vlan_filter;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0) {
		printf("IOCTL Error\n");
		switch (data.command_error) {
		case DWC_ETH_QOS_NO_HW_SUPPORT:
			printf("No hardware support present for VLAN filtering\n");
			break;
		default:
			printf("VLAN filtering is failed\n");
			break;
		}
	} else
		printf("Successfully %s VLAN %s filtering and %s matching\n",
			(vlan_filter.filter_enb_dis ? "ENABLED" : "DISABLED"),
			(vlan_filter.perfect_hash ? "HASH" : "PERFECT"),
			(vlan_filter.perfect_inverse_match ? "INVERSE" : "PERFECT"));
	return ret;
}


static int config_arp_offload(int sockfd, char *ifname, char *enable_disable)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int option = atoi(enable_disable);
	struct DWC_ETH_QOS_arp_offload arp_offload;
	char *ip_addr;
	struct sockaddr_in *addr = NULL;
	int ret = 0;

	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	/* Get IP addr of our interface */
	if (ioctl(sockfd, SIOCGIFADDR, &ifr) < 0) {
	  printf("failed to get IP address of %s\n", ifname);
	  ret = -1;
	  goto get_ip_addr_failed;
	}
	addr = ((struct sockaddr_in *)&ifr.ifr_addr);
	ip_addr = inet_ntoa(addr->sin_addr);
	printf("IP addr to be programmed in MAC_ARP_Addr reg is %s\n",
	    ip_addr);

	arp_offload.ip_addr[0] = strtol(strtok(ip_addr, "."), NULL, 10);
	arp_offload.ip_addr[1] = strtol(strtok(NULL, "."), NULL, 10);
	arp_offload.ip_addr[2] = strtol(strtok(NULL, "."), NULL, 10);
	arp_offload.ip_addr[3] = strtol(strtok(NULL, "."), NULL, 10);

	data.flags = option;
	data.cmd = DWC_ETH_QOS_ARP_OFFLOAD_CMD;
	data.qInx = 0; /* Not used */
	ifr.ifr_ifru.ifru_data = &data;
	data.ptr = &arp_offload;

	printf("arp_offload.ip_addr = %d.%d.%d.%d\n", arp_offload.ip_addr[0],
	    arp_offload.ip_addr[1], arp_offload.ip_addr[2],
	    arp_offload.ip_addr[3]);

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0) {
		printf("IOCTL Error\n");
		switch (data.command_error) {
		case DWC_ETH_QOS_NO_HW_SUPPORT:
			printf("No hardware support present for ARP Offloading\n");
			break;
		default:
			printf("ARP Offload configuration failed\n");
			break;
		}
	} else
		printf("Successfully %s ARP Offload\n",
		    (option ? "ENABLED" : "DISABLED"));

get_ip_addr_failed:
	return ret;
}


static int config_l2_da_filtering(int sockfd, char *ifname,
				  char *perfect_hash,
				  char *perfect_inverse_match)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	struct DWC_ETH_QOS_l2_da_filter l2_da_filter;
	int ret = 0;

	data.cmd = DWC_ETH_QOS_L2_DA_FILTERING_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	l2_da_filter.perfect_hash = atoi(perfect_hash);
	l2_da_filter.perfect_inverse_match = atoi(perfect_inverse_match);
	data.ptr = &l2_da_filter;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0) {
		printf("IOCTL Error\n");
		switch (data.command_error) {
		case DWC_ETH_QOS_NO_HW_SUPPORT:
			printf("No hardware support present for L2 %s filtering\n",
				(l2_da_filter.perfect_hash ? "HASH" : "PERFECT"));
			break;
		default:
			printf("L2 filtering mode selection failed\n");
			break;
		}
	} else
		printf("Successfully selected L2 %s filtering and %s DA matching\n",
			(l2_da_filter.perfect_hash ? "HASH" : "PERFECT"),
			(l2_da_filter.perfect_inverse_match ? "INVERSE" : "PERFECT"));

get_ip_addr_failed:
	return ret;
}

/* return 0 on success and -ve on failure */
static int config_axi_pbl(int sockfd, char *ifname, char *argv1)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int option = atoi(argv1);
	int ret = 0;

	switch (option) {
	case 4:
		data.flags = DWC_ETH_QOS_AXI_PBL_4;
		break;
	case 8:
		data.flags = DWC_ETH_QOS_AXI_PBL_8;
		break;
	case 16:
		data.flags = DWC_ETH_QOS_AXI_PBL_16;
		break;
	case 32:
		data.flags = DWC_ETH_QOS_AXI_PBL_32;
		break;
	case 64:
		data.flags = DWC_ETH_QOS_AXI_PBL_64;
		break;
	case 128:
		data.flags = DWC_ETH_QOS_AXI_PBL_128;
		break;
	case 256:
		data.flags = DWC_ETH_QOS_AXI_PBL_256;
		break;
	default:
		printf("Invalid option %d\n", option);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_AXI_PBL_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Successfully configured AXI PBL to %d\n", option);

	return ret;
}

/* return 0 on success and -ve on failure */
static int config_axi_worl(int sockfd, char *ifname, char *argv1)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int option = atoi(argv1);
	int ret = 0;

	if ((option > DWC_ETH_QOS_MAX_AXI_WORL) || (option < 0)) {
		printf("Invalid axi_worl value %d\n", option);
		return -1;
	}

	data.flags = (option -1);
	data.cmd = DWC_ETH_QOS_AXI_WORL_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Successfully configured AXI WORL to %d\n", option);

	return ret;
}

/* return 0 on success and -ve on failure */
static int config_axi_rorl(int sockfd, char *ifname, char *argv1)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int option = atoi(argv1);
	int ret = 0;

	if ((option > DWC_ETH_QOS_MAX_AXI_RORL) || (option < 0)) {
		printf("Invalid axi_rorl value %d\n", option);
		return -1;
	}

	data.flags = (option -1);
	data.cmd = DWC_ETH_QOS_AXI_RORL_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf("Successfully configured AXI RORL to %d\n", option);

	return ret;
}

/* return 0 on success and -ve on failure */
static int config_loopback_mode(int sockfd, char *ifname, char *argv1)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int option = atoi(argv1);
	int ret = 0;

	if (option)
		data.flags = DWC_ETH_QOS_MAC_LOOPBACK_ENABLE;
	else
		data.flags = DWC_ETH_QOS_MAC_LOOPBACK_DISABLE;

	data.cmd = DWC_ETH_QOS_MAC_LOOPBACK_MODE_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0) {
		printf("IOCTL Error\n");
		switch (data.command_error) {
		case DWC_ETH_QOS_CONFIG_FAIL:
			printf("mac loopback mode is already %s\n",
				(option ? "enabled" : "disable"));
			break;
		}
	}
	else
		printf("Successfully %s mac loopback mode\n\n",
			(option ? "enabled" : "disabled"));

	if (option)
		printf("WARNING: Please disable the mac loopback after the testing\n"
			   "         else it will interrupt the normal operation.\n");

	return ret;
}

/* return 0 on success and -ve on failure */
static int config_pfc(int sockfd, char *ifname, char *argv1)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int option = atoi(argv1);
	int ret = 0;

	if (option)
		data.flags = DWC_ETH_QOS_PFC_ENABLE;
	else
		data.flags = DWC_ETH_QOS_PFC_DISABLE;

	data.cmd = DWC_ETH_QOS_PFC_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret == DWC_ETH_QOS_NO_HW_SUPPORT)
		printf("No HW support present for PFC(Priority Based Flow Control)\n");
	else
		printf("Successfully %s PFC(Priority Based Flow Control)\n",
			(option ? "enabled" : "disabled"));

	return ret;
}


/* return 0 on success and -ve on failure */
static int config_dvlan_tx_processing(int sockfd, char *ifname,
		char *operation, char *mode, char *outer_vlt,
		char *inner_vlt, int is_via_reg)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	unsigned int opn = atoi(operation);
	unsigned int md = ~0;
	unsigned int ovlt = ~0;
	unsigned int ivlt = ~0;
	struct DWC_ETH_QOS_config_dvlan conf_dvlan;
	int ret = 0;

	if ((opn < 0) && (opn > 3)) {
		fprintf(stderr,"Invalide <operation> - %d\n",opn);
		return -1;
	}

	if ((opn != 0) && !mode) {
		fprintf(stderr,"Please specify the <mode>,"
				" as operation selected is %d\n",opn);
		return -1;
	}

	conf_dvlan.op_type = opn;
	if (mode) {
		md = atoi(mode);
		if (md == 2) {
			if (!inner_vlt) {
				fprintf(stderr, "Please specify the outer & inner "
						"vlan tags, as mode is `Inner and Outer VLAN tag`\n");
				return -1;
			}
		} else if (!outer_vlt) {
			fprintf(stderr, "Please specify the %s "
					"vlan tag, as mode is %s VLAN tag.\n",
					(md ? "Inner" : "Outer"),
					(md ? "Inner" : "Outer"));
			return -1;
		}

		if (inner_vlt) {
			ovlt = atoi(outer_vlt);
			ivlt = atoi(inner_vlt);
			fprintf(stderr, "outer_vlan_tag = %#x, inner_vlan_tag = %#x\n", ovlt, ivlt);
		} else if (outer_vlt) {
			ovlt = atoi(outer_vlt);
			fprintf(stderr, "outer_vlan_tag = %#x\n", ovlt);
		}

		if (md == 0) {
			conf_dvlan.in_out = DWC_ETH_QOS_DVLAN_OUTER;
		} else if (md == 1) {
			conf_dvlan.in_out = DWC_ETH_QOS_DVLAN_INNER;
		} else if (md == 2) {
			conf_dvlan.in_out = DWC_ETH_QOS_DVLAN_BOTH;
		} else {
			fprintf(stderr,"Invalide <mode> - %d\n",md);
			return -1;
		}
	}

	if ((opn == 0) && ((md == 2) || (!mode))) {
		data.flags = DWC_ETH_QOS_DVLAN_DISABLE;
		conf_dvlan.in_out = DWC_ETH_QOS_DVLAN_BOTH;
	} else {
		data.flags = DWC_ETH_QOS_DVLAN_ENABLE;
	}

	fprintf(stderr, "%s: operation = %d, mode = %d\n", __func__, conf_dvlan.op_type, conf_dvlan.in_out);
	conf_dvlan.outer_vlan_tag = ovlt;
	conf_dvlan.inner_vlan_tag = ivlt;
	conf_dvlan.via_reg_or_desc = is_via_reg;

	data.cmd = DWC_ETH_QOS_DVLAN_TX_PROCESSING_CMD;
	data.qInx = 0; /* Not used */
	data.ptr = &conf_dvlan;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret == DWC_ETH_QOS_CONFIG_FAIL) {
		fprintf(stderr, "IOCTL Error\n");
		fprintf(stderr, "Failed to configure double VLAN processing on Tx path\n");
	} else if (ret == DWC_ETH_QOS_NO_HW_SUPPORT) {
		fprintf(stderr, "No HW support for Single/Double VLAN\n");
	} else {
		fprintf(stderr, "Successfully configured double VLAN processing on Tx path\n");
	}

	return ret;
}

/* return 0 on success and -ve on failure */
int config_dvlan(int sockfd, int argc, char *argv[], int is_via_reg)
{
	int ret = 0;

	if (argc == 4) {
		ret = config_dvlan_tx_processing(sockfd,
				argv[1], argv[3], NULL,
				NULL, NULL, is_via_reg);
	} else if (argc == 5) {
		ret = config_dvlan_tx_processing(sockfd,
				argv[1], argv[3], argv[4],
				argv[5], NULL, is_via_reg);
	} else if (argc > 5) {
		ret = config_dvlan_tx_processing(sockfd,
				argv[1], argv[3], argv[4],
				argv[5], argv[6], is_via_reg);
	} else {
		ret = -1;
	}

	return ret;
}

/* return 0 on success and -ve on failure */
static int config_dvlan_rx_processing(int sockfd,
		char *ifname, char *operation)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	unsigned int opn = atoi(operation);
	int ret = 0;

	if (opn == 0) {
		data.flags = DWC_ETH_QOS_DVLAN_NONE;
	} else if (opn == 1) {
		data.flags = DWC_ETH_QOS_DVLAN_INNER;
	} else if (opn == 2) {
		data.flags = DWC_ETH_QOS_DVLAN_OUTER;
	} else if (opn == 3) {
		data.flags = DWC_ETH_QOS_DVLAN_BOTH;
	} else {
		fprintf(stderr,"Invalide <operation> - %d\n",opn);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_DVLAN_RX_PROCESSING_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret == DWC_ETH_QOS_CONFIG_FAIL) {
		fprintf(stderr, "IOCTL Error\n");
		fprintf(stderr, "Failed to configure double VLAN processing on Rx path\n");
	} else if (ret == DWC_ETH_QOS_NO_HW_SUPPORT) {
		fprintf(stderr, "No HW support for Single/Double VLAN\n");
	} else {
		fprintf(stderr, "Successfully configured double VLAN processing on Rx path\n");
	}

	return ret;
}

/* return 0 on success and -ve on failure */
static int config_svlan(int sockfd,
		char *ifname, char *operation)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	unsigned int opn = atoi(operation);
	int ret = 0;

	if (opn == 0) {
		data.flags = DWC_ETH_QOS_DVLAN_NONE;
	} else if (opn == 1) {
		data.flags = DWC_ETH_QOS_DVLAN_INNER;
	} else if (opn == 2) {
		data.flags = DWC_ETH_QOS_DVLAN_OUTER;
	} else if (opn == 3) {
		data.flags = DWC_ETH_QOS_DVLAN_BOTH;
	} else {
		fprintf(stderr,"Invalide <operation> - %d\n",opn);
		return -1;
	}

	data.cmd = DWC_ETH_QOS_SVLAN_CMD;
	data.qInx = 0; /* Not used */
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret == DWC_ETH_QOS_CONFIG_FAIL) {
		fprintf(stderr, "IOCTL Error\n");
		fprintf(stderr, "Error in configuring S-VLAN\n");
	} else if (ret == DWC_ETH_QOS_NO_HW_SUPPORT) {
		fprintf(stderr, "No HW support for Single/Double VLAN\n");
	} else {
		fprintf(stderr, "Successfully configured S-VLAN\n");
	}

	return ret;
}

/* return 0 on success and -ve on failure */
static int config_ptpoffloading(int sockfd,
		char *ifname, char *en_dis,
		char *mode, char *domain_num, char *multicast_or_unicast)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	unsigned int en_dis_flag = atoi(en_dis);
	unsigned int md;
	unsigned int mc_uc;
	int ret = 0;
	struct DWC_ETH_QOS_config_ptpoffloading conf_ptp;

	if (en_dis_flag == 0) {
		conf_ptp.en_dis = DWC_ETH_QOS_PTP_OFFLOADING_DISABLE;
	} else if (en_dis_flag == 1) {
		conf_ptp.en_dis = DWC_ETH_QOS_PTP_OFFLOADING_ENABLE;
		if ((!mode) || (!domain_num)) {
				fprintf(stderr,"Missing an argument <mode> and/or <domain_num>\n");
				return -1;
		}
	} else {
		fprintf(stderr,"Invalide option - %d\n",en_dis_flag);
		return -1;
	}

	if (en_dis_flag) {
		md = atoi(mode);
		if ((md < 0) && (md > 6)) {
				fprintf(stderr,"Invalide mode - %d\n",md);
				return -1;
		}
		conf_ptp.mode = md;
		conf_ptp.domain_num = atoi(domain_num);
        mc_uc = atoi(multicast_or_unicast);
        if((mc_uc < 0) || (mc_uc > 1)) {
				fprintf(stderr,"Invalide multicast_or_unicast parameter - %d\n", mc_uc);
				return -1;
        }
        conf_ptp.mc_uc = mc_uc;
	}

	data.cmd = DWC_ETH_QOS_PTPOFFLOADING_CMD;
	data.qInx = 0; /* Not used */
	data.ptr = &conf_ptp;
	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret == DWC_ETH_QOS_CONFIG_FAIL) {
		fprintf(stderr, "IOCTL Error\n");
		fprintf(stderr, "Error in configuring PTP Offloading.\n");
	} else if (ret == DWC_ETH_QOS_NO_HW_SUPPORT) {
		fprintf(stderr, "No PTP support in Hardware.\n");
	} else {
		fprintf(stderr, "Successfully configured PTP Offloading.\n");
	}

	return ret;
}










#ifdef DWC_ETH_QOS_CONFIG_PGTEST

/* Initialize default parameters for Channel 0, 1 and 2.
 */
static void DWC_ETH_QOS_init_default_pg(struct DWC_ETH_QOS_pg_user_input *pg_params)
{
	struct DWC_ETH_QOS_pg_user_ch_input *pg_user_ch_input = pg_params->ch_input;
	unsigned int qInx;
	unsigned int bandwidth = 100/tx_queue_count;

	/* duration of experiment is 5 Sec by default */
	pg_params->duration_of_exp = 5;
	pg_params->dma_ch_en = 1;

	pg_params->ch_tx_rx_arb_scheme = 0;
	pg_params->ch_use_tx_high_prio = 0;
	pg_params->ch_tx_rx_prio_ratio = 0;
	pg_params->dma_tx_arb_algo = eDWC_ETH_QOS_DMA_TX_WRR;
	pg_params->queue_dcb_algorithm = eDWC_ETH_QOS_DCB_WRR;

	pg_params->mac_lb_mode = 0; /* 0 => No MAC Loopback; 1 => MAC Loopback On */
	pg_params->speed_100M_1G = 0;     /* This is valid only after receiving after the test  */

	/* configure Channel qInx to DWC_ETH_QOS_QUEUE_CNT */
	for (qInx = 0; qInx < tx_queue_count; qInx++) {
		memset(&pg_user_ch_input[qInx], 0, sizeof(struct DWC_ETH_QOS_pg_user_ch_input));

		pg_params->dma_ch_en |= (0x1 << qInx);
		if (qInx == 0) {
			pg_user_ch_input[qInx].ch_arb_weight = 0;
			pg_user_ch_input[qInx].ch_fr_size = 1500;
			pg_user_ch_input[qInx].ch_bw_alloc = bandwidth;

			/* AVB can't be used on queue-0 */
			pg_user_ch_input[qInx].ch_operating_mode = eDWC_ETH_QOS_QDCB;
			pg_user_ch_input[qInx].ch_CreditControl = 0;

			/* slot facility not available for 0
			 * do not change below values */
			pg_user_ch_input[qInx].ch_tx_desc_slot_no_start = 0;
			pg_user_ch_input[qInx].ch_tx_desc_slot_no_skip = 0;
			pg_user_ch_input[qInx].ch_AvgBits = 0;
			pg_user_ch_input[qInx].ch_AvgBits_interrupt_count = 0;
			pg_user_ch_input[qInx].ch_use_slot_no_check = 0;
			pg_user_ch_input[qInx].ch_use_adv_slot_no_check = 0;
			pg_user_ch_input[qInx].ch_slot_count_to_use = 0;

			/* debug parameters */
			pg_user_ch_input[qInx].ch_max_tx_frame_cnt = 255;
			pg_user_ch_input[qInx].ch_debug_mode = 0;

		} else {
			pg_user_ch_input[qInx].ch_arb_weight = 0;
			pg_user_ch_input[qInx].ch_fr_size = 1500;
			pg_user_ch_input[qInx].ch_bw_alloc = bandwidth;

			pg_user_ch_input[qInx].ch_operating_mode = eDWC_ETH_QOS_QDCB;
			pg_user_ch_input[qInx].ch_CreditControl = 0;
			pg_user_ch_input[qInx].ch_avb_algorithm = eDWC_ETH_QOS_AVB_CBS;

			pg_user_ch_input[qInx].ch_tx_desc_slot_no_start = 0;
			pg_user_ch_input[qInx].ch_tx_desc_slot_no_skip = 0;
			pg_user_ch_input[qInx].ch_AvgBits = 0;
			pg_user_ch_input[qInx].ch_AvgBits_interrupt_count = 0;
			pg_user_ch_input[qInx].ch_use_slot_no_check = 0;
			pg_user_ch_input[qInx].ch_use_adv_slot_no_check = 0;
			pg_user_ch_input[qInx].ch_slot_count_to_use = 0;

			/* debug parameters */
			pg_user_ch_input[qInx].ch_max_tx_frame_cnt = 255;
			pg_user_ch_input[qInx].ch_debug_mode = 0;
		}
	}
}

static void DWC_ETH_QOS_calculate_slope(struct DWC_ETH_QOS_PGStruct *pg_struct,
		struct DWC_ETH_QOS_pg_ch_input *pg_ch_input, unsigned int qInx)
{
	unsigned int multiplier = 1;

	if (connected_speed == SPEED_1000)
		multiplier = 2;

	pg_ch_input[qInx].ch_IdleSlope = get_idle_slope(pg_ch_input[qInx].ch_bw);
	pg_ch_input[qInx].ch_SendSlope = get_send_slope(pg_ch_input[qInx].ch_bw);

	pg_ch_input[qInx].ch_HiCredit = get_hi_credit(pg_ch_input[qInx].ch_bw);
	pg_ch_input[qInx].ch_LoCredit = get_low_credit(pg_ch_input[qInx].ch_bw);

	return;
}

static void DWC_ETH_QOS_calculate_queue_weight(struct DWC_ETH_QOS_PGStruct *pg_struct,
		struct DWC_ETH_QOS_pg_ch_input *pg_ch_input,
		unsigned int max_frame_size,
		unsigned int qInx)
{
	pg_ch_input[qInx].ch_queue_weight =
		DWC_ETH_QOS_get_dcb_queue_weights(pg_struct->queue_dcb_algorithm,
			pg_ch_input[qInx].ch_bw, max_frame_size);

	return;
}

/*
 * Copies Channel parameters from pg user struct to pg kernel struct.
 */
static void DWC_ETH_QOS_populate_pg_struct(struct DWC_ETH_QOS_pg_user_input *pg_params,
					struct DWC_ETH_QOS_PGStruct *pg_struct)
{
	struct DWC_ETH_QOS_pg_user_ch_input *pg_user_ch_input = pg_params->ch_input;
	struct DWC_ETH_QOS_pg_ch_input *pg_ch_input = pg_struct->pg_ch_input;
	unsigned int qInx, channel_enable = 0, max_frame_size = 0;

	pg_struct->ch_SelMask = pg_params->dma_ch_en;
	pg_struct->DurationOfExp = pg_params->duration_of_exp;

	pg_struct->PrioTagForAV = 5; /* Priority of Ch2 */

	pg_struct->ch_tx_rx_arb_scheme = pg_params->ch_tx_rx_arb_scheme;
	pg_struct->ch_use_tx_high_prio = pg_params->ch_use_tx_high_prio;
	pg_struct->ch_tx_rx_prio_ratio = pg_params->ch_tx_rx_prio_ratio;
	pg_struct->dma_tx_arb_algo = pg_params->dma_tx_arb_algo;
	pg_struct->queue_dcb_algorithm = pg_params->queue_dcb_algorithm;
	pg_struct->mac_lb_mode = pg_params->mac_lb_mode;

	channel_enable = pg_params->dma_ch_en;
	max_frame_size = pg_user_ch_input[0].ch_fr_size;
	for (qInx = 0; qInx < tx_queue_count; qInx++) {
		if (pg_user_ch_input[0].ch_fr_size > max_frame_size)
			max_frame_size = pg_user_ch_input[0].ch_fr_size;
	}

	for (qInx = 0; qInx < tx_queue_count; qInx++) {
		/* updating parameters only if channel is enabled */
		if (channel_enable & (1 << qInx)) {
			/* copy Channel qInx parameters */
			pg_ch_input[qInx].ch_arb_weight = pg_user_ch_input[qInx].ch_arb_weight;
			pg_ch_input[qInx].ch_bw = pg_user_ch_input[qInx].ch_bw_alloc;

			DWC_ETH_QOS_calculate_queue_weight(pg_struct, pg_ch_input, max_frame_size, qInx);

			pg_ch_input[qInx].ch_frame_size = pg_user_ch_input[qInx].ch_fr_size;
			pg_ch_input[qInx].ch_debug_mode = pg_user_ch_input[qInx].ch_debug_mode;
			pg_ch_input[qInx].ch_max_tx_frame_cnt = pg_user_ch_input[qInx].ch_max_tx_frame_cnt;
			pg_ch_input[qInx].ch_operating_mode = pg_user_ch_input[qInx].ch_operating_mode;
			pg_ch_input[qInx].ch_AvgBits = pg_user_ch_input[qInx].ch_AvgBits;
			pg_ch_input[qInx].ch_AvgBits_interrupt_count =
				pg_user_ch_input[qInx].ch_AvgBits_interrupt_count;

			pg_ch_input[qInx].ch_avb_algorithm = pg_user_ch_input[qInx].ch_avb_algorithm;
			pg_ch_input[qInx].ch_CreditControl = pg_user_ch_input[qInx].ch_CreditControl;

			DWC_ETH_QOS_calculate_slope(pg_struct, pg_ch_input, qInx);

			pg_ch_input[qInx].ch_EnableSlotCheck = pg_user_ch_input[qInx].ch_use_slot_no_check;
			pg_ch_input[qInx].ch_EnableAdvSlotCheck = pg_user_ch_input[qInx].ch_use_adv_slot_no_check;
			pg_ch_input[qInx].ch_SlotCount = pg_user_ch_input[qInx].ch_slot_count_to_use;
			pg_ch_input[qInx].ch_tx_desc_slot_no_start = pg_user_ch_input[qInx].ch_tx_desc_slot_no_start;
			pg_ch_input[qInx].ch_tx_desc_slot_no_skip = pg_user_ch_input[qInx].ch_tx_desc_slot_no_skip;

			pg_ch_input[qInx].ch_FramecountTx = 0;
			pg_ch_input[qInx].ch_FramecountRx = 0;
		}
		else {
			pg_ch_input[qInx].ch_bw = 0;
		}
	}
}

static int DWC_ETH_QOS_send_pg_param_to_driver(int sockfd, char *ifname,
						struct DWC_ETH_QOS_PGStruct *pg_struct)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int ret = 0;

	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	data.cmd = DWC_ETH_QOS_PG_TEST;
	data.flags = DWC_ETH_QOS_PG_SET_CONFIG;
	data.ptr = pg_struct;
	data.qInx = 0;

	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf
		    ("Successfully configured PG parameters\n");

	return ret;
}

static int DWC_ETH_QOS_config_hw_for_pg_test(int sockfd, char *ifname)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int ret = 0;

	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	data.cmd = DWC_ETH_QOS_PG_TEST;
	data.flags = DWC_ETH_QOS_PG_CONFIG_HW;
	data.ptr = NULL;
	data.qInx = 0;

	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf
		    ("Successfully configured the HW for Test\n");

	return ret;
}

static int DWC_ETH_QOS_run_pg_test(int sockfd, char *ifname,
				struct DWC_ETH_QOS_PGStruct *pg_struct)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int ret = 0;

	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	data.cmd = DWC_ETH_QOS_PG_TEST;
	data.flags = DWC_ETH_QOS_PG_RUN_TEST;
	data.ptr = pg_struct;
	data.qInx = 0;

	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf
		    ("PG Test Started\n");

	return ret;
}

static int DWC_ETH_QOS_check_test_done(int sockfd, char *ifname)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int ret = 0;

	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	data.cmd = DWC_ETH_QOS_PG_TEST;
	data.flags = DWC_ETH_QOS_PG_TEST_DONE;
	data.qInx = 0;

	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		ret = data.test_done;

	return ret;
}

static int DWC_ETH_QOS_get_pg_result_from_hw(int sockfd, char *ifname,
				struct DWC_ETH_QOS_PGStruct *pg_struct)
{
	struct ifreq ifr;
	struct ifr_data_struct data;
	int ret = 0;

	strcpy(ifr.ifr_ifrn.ifrn_name, ifname);
	data.cmd = DWC_ETH_QOS_PG_TEST;
	data.flags = DWC_ETH_QOS_PG_GET_RESULT;
	data.ptr = pg_struct;
	data.qInx = 0;

	ifr.ifr_ifru.ifru_data = &data;

	ret = ioctl(sockfd, DWC_ETH_QOS_PRV_IOCTL, &ifr);
	if (ret < 0)
		printf("IOCTL Error\n");
	else
		printf
		    ("Successfully Retrieved PG Results\n");

	return ret;
}


static void DWC_ETH_QOS_process_ch_params(struct DWC_ETH_QOS_pg_user_ch_input *pg_user_ch_input,
					int user_input,
					int ch_no)
{
	char user_string[10];

	switch(user_input) {
	case 00:
		printf("(%d00) Channel %d Frame size in bytes (only payload)          : %04d\n",
			ch_no, ch_no, pg_user_ch_input[ch_no].ch_fr_size);
		printf("Your Option here::");
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);
		if ((user_input > 48) && (user_input <= 1500))
			pg_user_ch_input[ch_no].ch_fr_size = user_input;
		else
			printf("Invalid input\n");
		break;
	case 01:
		printf("(%d01) Channel %d Bandwidth Allocation                        : %02d%%\n",
			ch_no, ch_no, pg_user_ch_input[ch_no].ch_bw_alloc);
		printf("Your Option here::");
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);
		if ((user_input > 0) && (user_input <= 75))
			pg_user_ch_input[ch_no].ch_bw_alloc = user_input;
		else
			printf("Invalid input\n");
		break;
	case 02:
		printf("(%d02) Channel %d Enable Slot Number Check  <1/0>             : %s\n",
			ch_no, ch_no,
			(pg_user_ch_input[ch_no].ch_use_slot_no_check ? "YES" : "NO"));
		printf("     [0 - Disable slot number check]\n");
		printf("     [1 - Enable slot number check]\n");
		printf("Your Option here::");
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);
		if ((user_input == 0) || (user_input == 1))
			pg_user_ch_input[ch_no].ch_use_slot_no_check = user_input;
		else
			printf("Invalid input\n");
		break;
	case 03:
		printf("(%d03) Channel %d Enabled advance Slot check <1/0>            : %s\n",
			ch_no, ch_no,
			(pg_user_ch_input[ch_no].ch_use_adv_slot_no_check ? "YES" : "NO"));
		printf("     [0 - Disable advance slot number check]\n");
		printf("     [1 - Enable advance slot number check]\n");
		printf("Your Option here::");
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);
		if ((user_input == 0) || (user_input == 1))
			pg_user_ch_input[ch_no].ch_use_adv_slot_no_check = user_input;
		else
			printf("Invalid input\n");
		break;
	case 04:
		printf("(%d04) Channel %d Slot Counter for AVB Reporting <1/2/4/8/16> : %01d\n",
			ch_no, ch_no, pg_user_ch_input[ch_no].ch_slot_count_to_use);
		printf("Your Option here::");
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);
		if ((user_input == 1) || (user_input == 2) || (user_input == 4) ||
		    (user_input == 8) || (user_input == 16))
			pg_user_ch_input[ch_no].ch_slot_count_to_use = user_input;
		else
			printf("Invalid input\n");
		break;
	case 05:
		printf("(%d05a) Channel %d AVB algorithm <0/1>         : %s\n",
			ch_no, ch_no,
			(pg_user_ch_input[ch_no].ch_avb_algorithm == eDWC_ETH_QOS_AVB_SP ?
			"Strict Priority" : "Credit Based Shaper"));
		printf("     [0 - Strict Priority Algorithm]\n");
		printf("     [1 - Credit Shaper Algorithm]\n");
		printf("Your Option here::");
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);
		if ((user_input == 0) || (user_input == 1)) {
			pg_user_ch_input[ch_no].ch_avb_algorithm = user_input;
		}
		else
			printf("Invalid input\n");

		printf("(%d05b) Channel %d Uses Credit Control <0/1>        : %s\n",
			ch_no, ch_no,
			(pg_user_ch_input[ch_no].ch_CreditControl ? "YES" : "NO"));
		printf("     [0 - No Credit Control]\n");
		printf("     [1 - Enforce Credit Control]\n");
		printf("Your Option here::");
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);
		if ((user_input == 0) || (user_input == 1)) {
			pg_user_ch_input[ch_no].ch_CreditControl = user_input;
		}
		else
			printf("Invalid input\n");
		break;
	case 06:
		printf("(%d06) Channel %d Tx Desc Starting Slot No <0 .. 15>          : %01x\n",
			ch_no, ch_no, pg_user_ch_input[ch_no].ch_tx_desc_slot_no_start);
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);
		if ((user_input >= 0) && (user_input <= 15))
			pg_user_ch_input[ch_no].ch_tx_desc_slot_no_start = user_input;
		else
			printf("Invalid input\n");
		break;
	case 07:
		printf("(%d07) Channel %d Tx Desc Slot No Skip count <0 .. 15>        : %01x\n",
			ch_no, ch_no, pg_user_ch_input[ch_no].ch_tx_desc_slot_no_skip);
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);
		if ((user_input >= 0) && (user_input <= 15))
			pg_user_ch_input[ch_no].ch_tx_desc_slot_no_skip = user_input;
		else
			printf("Invalid input\n");
		break;
	case  8:
		printf("(%d08) Channel %d Arbitration Weight        <0,1,2,3,4,5,6,7> : %01x\n",
			ch_no, ch_no, pg_user_ch_input[ch_no].ch_arb_weight);
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);
		if ((user_input >= 0) && (user_input <= 7))
			pg_user_ch_input[ch_no].ch_arb_weight = user_input;
		else
			printf("Invalid input\n");
		break;
	case 9:
		printf("(%d09) Channel %d Operating mode                              : %01x\n",
			ch_no, ch_no, pg_user_ch_input[ch_no].ch_operating_mode);
		printf("        <0-Not enabled, 1-AVB, 2-DCB, 3-Generic>\n");
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);
		if ((user_input >= 0) && (user_input <= 3))
			pg_user_ch_input[ch_no].ch_operating_mode = user_input;
		else
			printf("Invalid input\n");
		break;
	case 10:
		printf("(%d10) Channel %d enable debug mode                          : %d\n",
			ch_no, ch_no, pg_user_ch_input[ch_no].ch_debug_mode);
		printf("        <0 - disable debug mode, 1 - enable debug mode>\n");
		printf("Your Option here::");
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);
		if ((user_input == 0) || (user_input == 1))
			pg_user_ch_input[ch_no].ch_debug_mode = user_input;
		else
			printf("Invalid input\n");
		break;
	case 11:
		printf("(%d11) Channel %d maximum Tx packet count                   : %d\n",
			ch_no, ch_no, pg_user_ch_input[ch_no].ch_max_tx_frame_cnt);
		printf("Your Option here::");
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);
		pg_user_ch_input[ch_no].ch_max_tx_frame_cnt = user_input;
		break;
	default:
		printf("Invalid Option\n");
	}
}

static void DWC_ETH_QOS_get_pg_params(struct DWC_ETH_QOS_pg_user_input *pg_params)
{
	struct DWC_ETH_QOS_pg_user_ch_input *pg_user_ch_input = pg_params->ch_input;
	char user_string[10];
	int user_input = 0;
	int i;
	int input_limit;

	do {
		system("clear");

		printf("--------------------------------------------------\n");
		printf("Select the Options Below:\n");
		printf("--------------------------------------------------\n");

		printf("(1000) Duration to run the Experiment(seconds)               : %02d\n\n",
			pg_params->duration_of_exp);
		printf("(1001) DMA Channel Enable                                    : %#02x\n\n",
			pg_params->dma_ch_en);
		printf("(1002) DMA Tx and Rx Priority Scheme(0/1)                    : %01d\n",
			pg_params->ch_tx_rx_arb_scheme);
		printf("     [0 - Weighted Round Robin, 1 - Fixed Priority]\n");
		printf("(1003) DMA Tx has High Priority over Rx(0/1)                 : %01d\n",
			pg_params->ch_use_tx_high_prio);
		printf("     [0 - Rx has High Priority, 1 - Tx has High Priority]\n");
		printf("(1004) DMA Tx and Rx Priority Ratio                          : %01d\n",
			pg_params->ch_tx_rx_prio_ratio);
		printf("     [Tx:Rx or Rx:Tx, 0=> 1:1, 1=>2:1, 2=>3:1, 3=>4:1\n"
		       "                      4=> 5:1, 5=>6:1, 6=>7:1, 7=>8:1]\n");
		printf("(1005) DMA Transmit Arbitration algorithm                    : %01d\n",
				pg_params->dma_tx_arb_algo);
		printf("     [0 - Fixed priority\n");
		printf("      1 - WSP (Weighted strict priority)\n");
		printf("      2 - WRR (Weighted Round Robin]\n");
		printf("(1006) MTL DCB algorithm                                     : %01d\n",
			pg_params->queue_dcb_algorithm);
		printf("     [0 - WRR  (Weighted Round Robin)\n"
		       "      1 - WFQ  (Weighted Fair Queuing)\n"
		       "      2 - DWRR (Deficit Weighted Round Robin)\n"
		       "      3 - SP   (Strict Priority)]\n");
		printf("(1007) MAC Loop-Back Mode                                    : %01d\n",
			pg_params->mac_lb_mode);
		printf("     [0 - Disable MAC Loop-Back Mode \n"
		       "      1 - Enable  MAC Loop-Back Mode ]\n");

		for (i = 0; i < tx_queue_count; i++) {
			printf("\n");
			printf("(%d00) Channel %d Frame size in bytes (only payload)          : %04d\n",
				i, i, pg_user_ch_input[i].ch_fr_size);
			printf("(%d01) Channel %d Bandwidth Allocation                        : %02d%%\n",
				i, i, pg_user_ch_input[i].ch_bw_alloc);
			printf("(%d02) Channel %d Enable Slot Number Check                    : %s\n",
				i, i, pg_user_ch_input[i].ch_use_slot_no_check ? "YES" : "NO");
			printf("(%d03) Channel %d Enable advance Slot slot check              : %s\n",
				i, i, (pg_user_ch_input[i].ch_use_adv_slot_no_check ? "YES" : "NO"));
			printf("(%d04) Channel %d Slot Counter for AVB Reporting              : %01d\n",
				i, i, pg_user_ch_input[i].ch_slot_count_to_use);
			printf("(%d05) Channel %d AVB Algorithm                               : %s\n",
				i, i, (pg_user_ch_input[i].ch_avb_algorithm == eDWC_ETH_QOS_AVB_SP ?
				"Strict Priority" : "Credit Based Shaper"));
			printf("       Channel %d Uses Credit Control(0/1)                    : %s\n",
				i, (pg_user_ch_input[i].ch_CreditControl ? "YES" : "NO"));
			printf("(%d06) Channel %d Tx Desc Starting Slot No <0 .. 15>          : %01x\n",
				i, i, pg_user_ch_input[i].ch_tx_desc_slot_no_start);
			printf("(%d07) Channel %d Tx Desc Slot No Skip count <0 .. 15>        : %01x\n",
				i, i, pg_user_ch_input[i].ch_tx_desc_slot_no_skip);
			printf("(%d08) Channel %d Arbitration Weight   <1,2,3,4,5,6,7,8>      : %01x\n",
				i, i, pg_user_ch_input[i].ch_arb_weight);
			printf("(%d09) Channel %d Operating mode                              : %01x\n",
				i, i, pg_user_ch_input[i].ch_operating_mode);
			printf("        <0-Not enabled, 1-AVB, 2-DCB, 3-Generic>\n");
			printf("(%d10) Channel %d enable debug mode                           : %s\n",
				i, i, (pg_user_ch_input[i].ch_debug_mode ? "YES" : "NO"));
			printf("(%d11) Channel %d maximum Tx packet limit                     : %d\n",
				i, i, pg_user_ch_input[i].ch_max_tx_frame_cnt);
			printf("\n");
		}
		printf("\n");
		printf("(9999) To quit this menu\n\n");
		printf("Your Option here::");
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);
		/*0 - 99 for Q0, 100 to 199 for Q1,, etc, hence input limit is number of queues * 100. */
		input_limit = tx_queue_count * 100;

		if (user_input == 1000) {
			printf("(1000) Duration to run the Experiment (Seconds) <1 ... 200>   :%02d\n\n",
				pg_params->duration_of_exp);
			printf("Your Option here::");
			fscanf(stdin, "%s", user_string);
			sscanf(user_string, "%d", &user_input);
			if ((user_input > 0) && (user_input <= 200))
				pg_params->duration_of_exp = user_input;
		}
		else if (user_input == 1001) {
			printf("(1001) DMA Channel Enabled                                    :%#02x\n\n",
				pg_params->dma_ch_en);
			printf("Your Option here::");
			fscanf(stdin, "%s", user_string);
			sscanf(user_string, "%x", &user_input);
			if (user_input < (1 << tx_queue_count))
				pg_params->dma_ch_en = user_input;
			else
				printf("Device supports only %d channels/queues\n",
						tx_queue_count);
		}
		else if (user_input == 1002) {
			printf("(1002) DMA Tx and Rx Priority Scheme <0/1>                    : %01d\n",
				pg_params->ch_tx_rx_arb_scheme);
			printf("     [0 - Weighted Round Robin, 1 - Strict Priority]\n");
			printf("Your Option here::");
			fscanf(stdin, "%s", user_string);
			sscanf(user_string, "%d", &user_input);
			if ((user_input == 0) || (user_input == 1))
				pg_params->ch_tx_rx_arb_scheme = user_input;
			else
				printf("Invalid input\n");
		}
		else if (user_input == 1003) {
			printf("(1003) DMA Tx has High Priority over Rx <0/1>                 : %01d\n",
				pg_params->ch_use_tx_high_prio);
			printf("     [0 - Rx has High Priority, 1 - Tx has High Priority]\n");
			printf("Your Option here::");
			fscanf(stdin, "%s", user_string);
			sscanf(user_string, "%d", &user_input);
			if ((user_input == 0) || (user_input == 1))
				pg_params->ch_use_tx_high_prio = user_input;
			else
				printf("Invalid input\n");
		}
		else if (user_input == 1004) {
			printf("(1004) DMA Tx and Rx Priority Ratio  <0/1/2/3>                : %01d\n",
				pg_params->ch_tx_rx_prio_ratio);
			printf("     [Tx:Rx or Rx:Tx, 0=> 1:1, 1=>2:1, 2=>3:1, 3=>4:1\n"
			     "                      4=> 5:1, 5=>6:1, 6=>7:1, 7=>8:1]\n");
			printf("Your Option here::");
			fscanf(stdin, "%s", user_string);
			sscanf(user_string, "%d", &user_input);
			if ((user_input >= 0) && (user_input <= 3))
				pg_params->ch_tx_rx_prio_ratio = user_input;
			else
				printf("Invalid input\n");
		}
		else if (user_input == 1005) {
			printf("(1005) DMA Transmit Arbitration algorithm                    : %01d\n",
					pg_params->dma_tx_arb_algo);
			printf("     [0 - Fixed priority\n");
			printf("      1 - WSP (Weighted strict priority)\n");
			printf("      2 - WRR (Weighted Round Robin]\n");
			printf("Your Option here::");
			fscanf(stdin, "%s", user_string);
			sscanf(user_string, "%d", &user_input);
			if ((user_input >= 0) && (user_input <= 2))
				pg_params->dma_tx_arb_algo = user_input;
			else
				printf("Invalid input\n");
		}
		else if (user_input == 1006) {
			printf("(1006) MTL DCB algorithm                                      : %01d\n",
				pg_params->queue_dcb_algorithm);
			printf("     [0 - WRR  (Weighted Round Robin)\n"
						 "      1 - WFQ  (Weighted Fair Queuing)\n"
						 "      2 - DWRR (Deficit Weighted Round Robin)\n"
						 "      3 - SP   (Strict Priority)]\n");
			printf("Your Option here::");
			fscanf(stdin, "%s", user_string);
			sscanf(user_string, "%d", &user_input);
			if ((user_input >= 0) && (user_input <= 3))
				pg_params->queue_dcb_algorithm = user_input;
			else
				printf("Invalid input\n");
		}
        else if (user_input == 1007) {
		    printf("(1007) mac_lb_mode                                            : %01d\n",
			    pg_params->mac_lb_mode);
    		printf("     [0 - Disable MAC Loop-Back Mode \n"
	    	       "      1 - Enable  MAC Loop-Back Mode ]\n");
			printf("Your Option here::");
			fscanf(stdin, "%s", user_string);
			sscanf(user_string, "%d", &user_input);
			if ((user_input >= 0) && (user_input <= 1))
				pg_params->mac_lb_mode = user_input;
			else
				printf("Invalid input\n");
        }

		else if ((user_input >= 0) && (user_input <= 99)
				&& (user_input < input_limit)) { /* ch 0 parameter */
			DWC_ETH_QOS_process_ch_params(pg_user_ch_input,
					user_input, 0);
		}
		else if ((user_input >= 100) && (user_input <= 199)
				&& (user_input < input_limit)) { /* ch 1 parameter */
			DWC_ETH_QOS_process_ch_params(pg_user_ch_input,
					(user_input - 100), 1);
		}
		else if ((user_input >= 200) && (user_input <= 299)
				&& (user_input < input_limit)) { /* ch 2 parameter */
			DWC_ETH_QOS_process_ch_params(pg_user_ch_input,
				(user_input - 200), 2);
		}
		else if ((user_input >= 300) && (user_input <= 399)
				&& (user_input < input_limit)) { /* ch 3 parameter */
			DWC_ETH_QOS_process_ch_params(pg_user_ch_input,
				(user_input - 300), 3);
		}
		else if ((user_input >= 400) && (user_input <= 499)
				&& (user_input < input_limit)) { /* ch 4 parameter */
			DWC_ETH_QOS_process_ch_params(pg_user_ch_input,
				(user_input - 400), 4);
		}
		else if ((user_input >= 500) && (user_input <= 599)
				&& (user_input < input_limit)) { /* ch 5 parameter */
			DWC_ETH_QOS_process_ch_params(pg_user_ch_input,
				(user_input - 500), 5);
		}
		else if ((user_input >= 600) && (user_input <= 699)
				&& (user_input < input_limit)) { /* ch 6 parameter */
			DWC_ETH_QOS_process_ch_params(pg_user_ch_input,
				(user_input - 600), 6);
		}
		else if ((user_input >= 700) && (user_input <= 799)
				&& (user_input < input_limit)) { /* ch 7 parameter */
			DWC_ETH_QOS_process_ch_params(pg_user_ch_input,
				(user_input - 700), 7);
		}
	} while (user_input != 9999);
}

static void DWC_ETH_QOS_gen_pg_report_in_file(struct DWC_ETH_QOS_pg_user_input *pg_params,
					struct DWC_ETH_QOS_PGStruct *pg_struct, FILE *fp)
{
	struct DWC_ETH_QOS_pg_ch_input *pg_ch_input = pg_struct->pg_ch_input;
	char user_string[10], *string;
	int user_input = 0;
	int i, display_avb_algo = 0, display_dcb_algo;
	unsigned long totalbytes = 0, totalbits = 0, time = pg_params->duration_of_exp;
    unsigned long long totalbytes_tx = 0;

	float ch_bw = 0, total_ch_bw = 0, total_nonav_bw = 0;
	char *space = "                                                  ";

	if (fp == stdout)
		system("clear");

	fprintf(fp, "===================================================================\n");

	display_avb_algo = 0;
	for (i = 0; i < tx_queue_count; i++) {
		if (pg_ch_input[i].ch_operating_mode == eDWC_ETH_QOS_QAVB) {
			display_avb_algo = 1;
			break;
		}
	}
	display_dcb_algo = 0;
	for (i = 0; i < tx_queue_count; i++) {
		if (pg_ch_input[i].ch_operating_mode == eDWC_ETH_QOS_QDCB) {
			display_dcb_algo = 1;
			break;
		}
	}

	fprintf(fp, "Duration Of The Experiment (Seconds)       : %02d\n",
		pg_params->duration_of_exp);
	fprintf(fp, "DMA Channel Enabled                        : %#02x\n",
		pg_params->dma_ch_en);
	fprintf(fp, "Tx:Rx Prio Scheme                          : %s\n",
		pg_params->ch_tx_rx_arb_scheme ? "SP" : "RR");
	fprintf(fp, "Tx has High Prio over Rx                   : %s\n",
		pg_params->ch_use_tx_high_prio ? "YES" : "NO");
	fprintf(fp, "Tx:Rx Prio Ratio (for RR)                  : %01d\n",
		pg_params->ch_tx_rx_prio_ratio);
	fprintf(fp, "DMA Transmit Arbitration algorithm         : %01d\n",
			pg_params->dma_tx_arb_algo);
	if (display_dcb_algo) {
		fprintf(fp, "MTL DCB algorithm                          : ");
		switch (pg_params->queue_dcb_algorithm) {
			case eDWC_ETH_QOS_DCB_WRR:
				fprintf(fp, "WRR (Weighted Round Robin)\n");
				break;
			case eDWC_ETH_QOS_DCB_WFQ:
				fprintf(fp, "WFQ (Weighted Fair Queuing)\n");
				break;
			case eDWC_ETH_QOS_DCB_DWRR:
				fprintf(fp, "DWRR (Deficit Weighted Round Robin)\n");
				break;
			case eDWC_ETH_QOS_DCB_SP:
				fprintf(fp, "SP (Strict Priority)\n");
				break;
		}
	}
    fprintf(fp, "mac_lb_mode                                : %01d\n",
			    pg_params->mac_lb_mode);
	fprintf(fp, "speed_100M_1G mode                         : %d\n",
		        pg_struct->speed_100M_1G);

	fprintf(fp, "\n");
	fprintf(fp, "-------------------------------------------------------------------\n");
	fprintf(fp, "Parameters\n");
	fprintf(fp, "-------------------------------------------------------------------\n");

	fprintf(fp, "Ch Operating mode\n");
	for (i = 0; i < tx_queue_count; i++) {
		switch (pg_ch_input[i].ch_operating_mode) {
		case eDWC_ETH_QOS_QAVB:
			string = "Audio Video Bridging";
			break;
		case eDWC_ETH_QOS_QDCB:
			string = "Data Centric Bridging";
			break;
		case eDWC_ETH_QOS_QGENERIC:
			string = "Non DCB (Generic)";
			break;
		case eDWC_ETH_QOS_QDISABLED:
			string = "Queue operating disabled";
			break;
		}
		fprintf(fp, "%s[CH%d]  %s\n",
			space, i, string);
	}

	fprintf(fp, "Ch Prio Weights [DMA]\n");
	for (i = 0; i < tx_queue_count; i++) {
		fprintf(fp, "%s[CH%d]  %01d\n",
			space, i, pg_ch_input[i].ch_arb_weight);
	}

	fprintf(fp, "Queue Weights [MTL]\n");
	for (i = 0; i < tx_queue_count; i++) {
		fprintf(fp, "%s[CH%d]  %03d\n",
			space, i, pg_ch_input[i].ch_queue_weight);
	}

	fprintf(fp, "Slot Number Check Enabled ?\n");
	for (i = 0; i < tx_queue_count; i++) {
		fprintf(fp, "%s[CH%d]  %s\n",
			space, i, pg_ch_input[i].ch_EnableSlotCheck ? "YES" : "NO");
	}

	fprintf(fp, "Adv Slot Number Check Enabled ?\n");
	for (i = 0; i < tx_queue_count; i++) {
		fprintf(fp, "%s[CH%d]  %s\n",
			space, i, pg_ch_input[i].ch_EnableAdvSlotCheck ? "YES" : "NO");
	}

	fprintf(fp, "Slot Counter for Avg bit Report\n");
	for (i = 0; i < tx_queue_count; i++) {
		fprintf(fp, "%s[CH%d]  %01d\n",
			space, i, pg_ch_input[i].ch_SlotCount);
	}

	fprintf(fp, "Ch Avb Bits\n");
	for (i = 0; i < tx_queue_count; i++) {
		fprintf(fp, "%s[CH%d]  %lu\n",
			space, i, pg_ch_input[i].ch_AvgBits);
	}

	fprintf(fp, "Ch Avb Bits Interrupt count\n");
	for (i = 0; i < tx_queue_count; i++) {
		fprintf(fp, "%s[CH%d]  %lu\n",
			space, i, pg_ch_input[i].ch_AvgBits_interrupt_count);
	}

	if (display_avb_algo) {
		fprintf(fp, "Ch AVB Algorithm\n");
		for (i = 0; i < tx_queue_count; i++) {
			if (pg_ch_input[i].ch_operating_mode == eDWC_ETH_QOS_QAVB)
				fprintf(fp, "%s[CH%d]  %s\n", space, i,
						(pg_ch_input[i].ch_avb_algorithm == eDWC_ETH_QOS_AVB_SP) ?
						"Strict Priority" : "Credit Based Shaper");
		}

		fprintf(fp, "Ch uses Credit Control ?\n");
		for (i = 0; i < tx_queue_count; i++) {
			if (pg_ch_input[i].ch_operating_mode == eDWC_ETH_QOS_QAVB)
				fprintf(fp, "%s[CH%d]  %s\n",
						space, i, pg_ch_input[i].ch_CreditControl ? "YES" : "NO");
		}

		fprintf(fp, "Ch Idle Slope\n");
		for (i = 0; i < tx_queue_count; i++) {
			if (pg_ch_input[i].ch_operating_mode == eDWC_ETH_QOS_QAVB)
				fprintf(fp, "%s[CH%d]  %#08x\n",
						space, i, pg_ch_input[i].ch_IdleSlope);
		}

		fprintf(fp, "Ch Send Slope\n");
		for (i = 0; i < tx_queue_count; i++) {
			if (pg_ch_input[i].ch_operating_mode == eDWC_ETH_QOS_QAVB)
				fprintf(fp, "%s[CH%d]  %#08x\n",
						space, i, pg_ch_input[i].ch_SendSlope);
		}

		fprintf(fp, "Ch High Credit\n");
		for (i = 0; i < tx_queue_count; i++) {
			if (pg_ch_input[i].ch_operating_mode == eDWC_ETH_QOS_QAVB)
				fprintf(fp, "%s[CH%d]  %#08x\n",
						space, i, pg_ch_input[i].ch_HiCredit);
		}

		fprintf(fp, "Ch Low Credit\n");
		for (i = 0; i < tx_queue_count; i++) {
			if (pg_ch_input[i].ch_operating_mode == eDWC_ETH_QOS_QAVB)
				fprintf(fp, "%s[CH%d]  %#08x\n",
						space, i, pg_ch_input[i].ch_LoCredit);
		}
	}

	/* required only for debugging */
	fprintf(fp, "Debug mode\n");
	for (i = 0; i < tx_queue_count; i++) {
		fprintf(fp, "%s[CH%d]  %s\n",
			space, i, (pg_ch_input[i].ch_debug_mode ? "YES" : "NO"));
	}

	/* required only for debugging */
	fprintf(fp, "Maximum Tx packet limit for debug mode\n");
	for (i = 0; i < tx_queue_count; i++) {
		fprintf(fp, "%s[CH%d]  %d\n",
			space, i, pg_ch_input[i].ch_max_tx_frame_cnt);
	}

	fprintf(fp, "Frame size\n");
	for (i = 0; i < tx_queue_count; i++) {
		fprintf(fp, "%s[CH%d]  %03d\n",
			space, i, pg_ch_input[i].ch_frame_size);
	}

	fprintf(fp, "Ch Tx Frame Count\n");
	for (i = 0; i < tx_queue_count; i++) {
		fprintf(fp, "%s[CH%d]  %ld\n",
			space, i, pg_ch_input[i].ch_FramecountTx);
	}

	fprintf(fp, "Ch Rx Frame Count\n");
	for (i = 0; i < tx_queue_count; i++) {
		fprintf(fp, "%s[CH%d]  %ld\n",
			space, i, pg_ch_input[i].ch_FramecountRx);
	}

	fprintf(fp, "Band Width Allocation\n");
	for (i = 0; i < tx_queue_count; i++) {
		fprintf(fp, "%s[CH%d]  %03d%%\n",
			space, i, pg_ch_input[i].ch_bw);
	}
	total_nonav_bw = 0;
	for (i = 0; i < tx_queue_count; i++) {
		if (pg_ch_input[i].ch_operating_mode != eDWC_ETH_QOS_QAVB)
			total_nonav_bw += pg_ch_input[i].ch_bw;
	}
	if (total_nonav_bw < 25)
		printf("WARNING: Total bandwidth for Non-AVB queues has gone below 25%%\n\n");

	fprintf(fp, "Ch BW utilization Achieved (based on Tx)\n");
	/* total = add_up(queue-rx-frames-count x queue-frame-size)
	 * individual-queue = ((queue-rx-frame-count x queue-frame-size) x 100) / total */
	totalbytes = 0;
	for (i = 0; i < tx_queue_count; i++) {
		totalbytes += (pg_ch_input[i].ch_FramecountTx * pg_ch_input[i].ch_frame_size);
	}
	for (i = 0; i < tx_queue_count; i++) {
		ch_bw = (pg_ch_input[i].ch_FramecountTx * pg_ch_input[i].ch_frame_size);
		if (ch_bw)
			ch_bw = (float)((ch_bw * 100)/totalbytes);
		fprintf(fp, "%s[CH%d]  %3.2F%%\n",
			space, i, ch_bw);
	}

	fprintf(fp, "Ch BW utilization Achieved (based on Rx)\n");
	/* total = add_up(queue-rx-frames-count x queue-frame-size)
	 * individual-queue = ((queue-rx-frame-count x queue-frame-size) x 100) / total */
	totalbytes = 0;
   	totalbytes_tx = 0;
	for (i = 0; i < rx_queue_count; i++) {
		totalbytes += (pg_ch_input[i].ch_FramecountRx * pg_ch_input[i].ch_frame_size);
       	totalbytes_tx += (pg_ch_input[i].ch_FramecountTx * pg_ch_input[i].ch_frame_size);

	}
	for (i = 0; i < rx_queue_count; i++) {
		ch_bw = (pg_ch_input[i].ch_FramecountRx * pg_ch_input[i].ch_frame_size);
		if (ch_bw)
			ch_bw = (float)((ch_bw * 100)/totalbytes_tx);
		fprintf(fp, "%s[CH%d]  %3.2F%%\n",
			space, i, ch_bw);
	}

	fprintf(fp, "Ch BW utilization Achieved on Tx in Mbps\n");
	for (i = 0; i < tx_queue_count; i++) {
		ch_bw = (pg_ch_input[i].ch_FramecountTx * (pg_ch_input[i].ch_frame_size + 8 + 4) * 8);
		if (ch_bw)
			ch_bw = (float)((ch_bw/time)/(1024*1024));
		fprintf(fp, "%s[CH%d]  %3.2F Mbps\n",
			space, i, ch_bw);
		total_ch_bw += ch_bw;
	}
	fprintf(fp, "%s----------------\n", space);
	fprintf(fp, "%sTotal %3.2F Mbps\n", space, total_ch_bw);

	fprintf(fp, "===================================================================\n");
	fprintf(fp, "\n");

	if (fp == stdout) {
		printf("Enter 99 to return to main menu::");
		do {
			fscanf(stdin, "%s", user_string);
			sscanf(user_string, "%d", &user_input);
			if (user_input != 99)
				printf("Invalid option %d\n", user_input);
		} while(user_input != 99);
	}
}


static void DWC_ETH_QOS_gen_pg_report(struct DWC_ETH_QOS_pg_user_input *pg_params,
					struct DWC_ETH_QOS_PGStruct *pg_struct)
{
#ifdef PGTEST_LOGFILE
	FILE *fp = NULL;
	time_t mytime;
	struct tm *mytm;
	char file_name[64], time_str[3][8], date_str[2][8];
#endif

	DWC_ETH_QOS_gen_pg_report_in_file(pg_params, pg_struct, stdout);

#ifdef PGTEST_LOGFILE

	/* log in file */
	mytime = time(NULL);
	mytm = localtime(&mytime);
	if (!mytm) {
		printf("Error in fetching system time\n");
		printf("No log file will be created\n");
		return;
	}

	sprintf(time_str[0], "%d", (mytm->tm_hour % 12));
	sprintf(time_str[1], "%d", mytm->tm_min);
	sprintf(time_str[2], "%d", mytm->tm_sec);
	switch (mytm->tm_mon) {
		case 0: sprintf(date_str[0], "Jan"); break;
		case 1: sprintf(date_str[0], "Feb"); break;
		case 2: sprintf(date_str[0], "Mar"); break;
		case 3: sprintf(date_str[0], "Apr"); break;
		case 4: sprintf(date_str[0], "May"); break;
		case 5: sprintf(date_str[0], "June"); break;
		case 6: sprintf(date_str[0], "Jul"); break;
		case 7: sprintf(date_str[0], "Aug"); break;
		case 8: sprintf(date_str[0], "Sep"); break;
		case 9: sprintf(date_str[0], "Oct"); break;
		case 10: sprintf(date_str[0], "Nov"); break;
		case 11: sprintf(date_str[0], "Dec"); break;
	}
	sprintf(date_str[1], "%d", mytm->tm_mday);
	sprintf(file_name, "pgtest_%s_%s_%s_%s_%s.log",
			date_str[0], date_str[1], time_str[0], time_str[1], time_str[2]);

	fp = fopen(file_name, "w+");
	if (fp == NULL) {
		printf("Error in file open\n");
		printf("No log file will be created\n");
		return;
	}
	fprintf(fp, "Report generated on %s %s, %s:%s:%s\n\n",
			date_str[0], date_str[1], time_str[0], time_str[1], time_str[2]);
	DWC_ETH_QOS_gen_pg_report_in_file(pg_params, pg_struct, fp);
	fclose(fp);
#endif

	return;
}

static void DWC_ETH_QOS_print_pg_struct(struct DWC_ETH_QOS_PGStruct *pg_struct)
{
	struct DWC_ETH_QOS_pg_ch_input *pg_ch_input = pg_struct->pg_ch_input;
	char user_string[10];
	int user_input = 0;
	int i;

	system("clear");
	printf("ch_SelMask                          = %#02x\n",
		pg_struct->ch_SelMask);
	printf("DurationOfExp                       = %02d\n",
		pg_struct->DurationOfExp);
	printf("PrioTagForAV                        = %#02x\n",
		pg_struct->PrioTagForAV);
	printf("ch_tx_rx_arb_scheme                 = %s\n",
		pg_struct->ch_tx_rx_arb_scheme ? "SP" : "RR");
	printf("ch_use_tx_high_prio                 = %01d\n",
		pg_struct->ch_use_tx_high_prio);
	printf("ch_tx_rx_prio_ratio                 = %01d\n",
		pg_struct->ch_tx_rx_prio_ratio);

	for (i = 0; i < tx_queue_count; i++) {
		printf("\n");
		printf("Ch%d ch_arb_weight                  = %01d\n",
			i, pg_ch_input[i].ch_arb_weight);
		printf("Ch%d ch_bw                          = %03d%%\n",
			i, pg_ch_input[i].ch_bw);
		printf("Ch%d ch_queue_weight                = %03d\n",
			i, pg_ch_input[i].ch_queue_weight);
		printf("Ch%d ch_frame_size                  = %03d\n",
			i, pg_ch_input[i].ch_frame_size);
		printf("Ch%d ch_EnableSlotCheck             = %s\n",
			i, pg_ch_input[i].ch_EnableSlotCheck ? "YES" : "NO");
		printf("Ch%d ch_EnableAdvSlotCheck          = %s\n",
			i, pg_ch_input[i].ch_EnableAdvSlotCheck ? "YES" : "NO");
		printf("Ch%d ch_avb_algorithm               = %s\n",
			i, ((pg_ch_input[i].ch_avb_algorithm == eDWC_ETH_QOS_AVB_SP) ?
				"Strict Priority" : "Credit Based Shaper"));
		printf("Ch%d ch_SlotCount                   = %01d\n",
			i, pg_ch_input[i].ch_SlotCount);
		printf("Ch%d ch_CreditControl               = %s\n",
			i, pg_ch_input[i].ch_CreditControl ? "YES" : "NO");
		printf("Ch%d ch_SendSlope                   = %#08x\n",
			i, pg_ch_input[i].ch_SendSlope);
		printf("Ch%d ch_IdleSlope                   = %#08x\n",
			i, pg_ch_input[i].ch_IdleSlope);
		printf("Ch%d ch_HiCredit                    = %#08x\n",
			i, pg_ch_input[i].ch_HiCredit);
		printf("Ch%d ch_LoCredit                    = %#08x\n",
			i, pg_ch_input[i].ch_LoCredit);
		printf("Ch%d ch_FramecountTx                = %ld\n",
			i, pg_ch_input[i].ch_FramecountTx);
		printf("Ch%d ch_FramecountRx                = %ld\n",
			i, pg_ch_input[i].ch_FramecountRx);
		printf("Ch%d ch_tx_desc_slot_no_start       = %#01x\n",
			i, pg_ch_input[i].ch_tx_desc_slot_no_start);
		printf("Ch%d ch_tx_desc_slot_no_skip        = %#01x\n",
			i, pg_ch_input[i].ch_tx_desc_slot_no_skip);
	}

	printf("\n");
	printf("Enter 99 to return to main menu::");
	do {
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);
		if (user_input != 99)
			printf("Invalid option %d\n", user_input);
	} while(user_input != 99);
}

static int DWC_ETH_QOS_reset_run_parameters(struct DWC_ETH_QOS_PGStruct *pg_struct)
{
	struct DWC_ETH_QOS_pg_ch_input *pg_ch_input = pg_struct->pg_ch_input;
	unsigned int qInx = 0;

	sleep(1);
	for (qInx = 0; qInx < tx_queue_count; qInx++) {
		pg_ch_input[qInx].ch_FramecountTx = 0;
		pg_ch_input[qInx].ch_AvgBits = 0;
		pg_ch_input[qInx].ch_AvgBits_interrupt_count = 0;
		pg_ch_input[qInx].tx_interrupts = 0;
		pg_ch_input[qInx].interrupt_prints = 0;
	}

	for (qInx = 0; qInx < rx_queue_count; qInx++) {
		pg_ch_input[qInx].ch_FramecountRx = 0;
	}

	return 0;
}

static int DWC_ETH_QOS_display_progress(int sockfd, char *ifname,
		struct DWC_ETH_QOS_PGStruct *pg_struct,
		struct DWC_ETH_QOS_pg_user_input *pg_params)
{
	int test_run = 1, total_seconds = 0;
	int dots = 3, user_input, i, blinker = 0;
	char user_string[10];

	printf("\n");
	do {
		/* print progress */
		printf("[%3dsecs] ", total_seconds);
		blinker = (total_seconds % dots);
		for (i = 0; i < dots; i++) {
			if (i <= blinker)
				printf(".");
			else
				printf(" ");
		}
		fflush(stdout);

		/* wait!! */
		sleep(1);

		/* clear display for next progress update */
		printf("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
		for (i = 0; i < dots; i++) {
			printf("\b");
		}
		fflush(stdout);
		total_seconds++;

		/* check for test complete */
		test_run = DWC_ETH_QOS_check_test_done(sockfd, ifname);
	} while (test_run);

	printf("\n");
	printf("RUN HAS COMPLETED\n");
	printf("\n");
	printf("Enter any number to view the run results::");
	fscanf(stdin, "%s", user_string);
	sscanf(user_string, "%d", &user_input);

	DWC_ETH_QOS_get_pg_result_from_hw(sockfd, ifname, pg_struct);
	DWC_ETH_QOS_gen_pg_report(pg_params, pg_struct);

	return 0;
}

/* returun 0 success and -ve number on failure */
static int DWC_ETH_QOS_pg_test(int sockfd, char *ifname)
{
	struct DWC_ETH_QOS_pg_user_input pg_params;
	struct DWC_ETH_QOS_PGStruct pg_struct;
	char user_string[10];
	int user_input = 0;

	if (tx_queue_count <= 0 || rx_queue_count <= 0) {
		printf("ERROR: Tx/Rx Queue count is zero\n");
		return -1;
	}

	DWC_ETH_QOS_init_default_pg(&pg_params);
	DWC_ETH_QOS_populate_pg_struct(&pg_params, &pg_struct);
	DWC_ETH_QOS_send_pg_param_to_driver(sockfd, ifname, &pg_struct);
	DWC_ETH_QOS_config_hw_for_pg_test(sockfd, ifname);

	printf("You are about to start PG Testing....\n");
	do {
		system("clear");
		printf("--------------------------------------------\n");
		printf("Select the Option Below:\n");
		printf("--------------------------------------------\n");

		printf("(01) Show/Get the PG Parameters\n");
		printf("(02) Send Parameters to Driver\n");
		printf("(03) Configure HW for PG Test\n");
		printf("(04) Run PG test\n");
		printf("(05) Show PG Test Reports\n");
		printf("(06) Print PG Structure\n");
		printf("(99) To quit this menu\n\n");

		printf("Your Option here::");
		fscanf(stdin, "%s", user_string);
		sscanf(user_string, "%d", &user_input);

		switch (user_input) {
		case 01:
			DWC_ETH_QOS_get_pg_params(&pg_params);
			DWC_ETH_QOS_populate_pg_struct(&pg_params, &pg_struct);
			break;
		case 02:
			DWC_ETH_QOS_send_pg_param_to_driver(sockfd, ifname, &pg_struct);
			break;
		case 03:
			DWC_ETH_QOS_config_hw_for_pg_test(sockfd, ifname);
			break;
		case 04:
			/* reset selected PG parameters in app and kernel before run */
			DWC_ETH_QOS_reset_run_parameters(&pg_struct);
			DWC_ETH_QOS_send_pg_param_to_driver(sockfd, ifname, &pg_struct);
			DWC_ETH_QOS_run_pg_test(sockfd, ifname, &pg_struct);
			DWC_ETH_QOS_display_progress(sockfd, ifname, &pg_struct, &pg_params);
			break;
		case 05:
			DWC_ETH_QOS_get_pg_result_from_hw(sockfd, ifname, &pg_struct);
			DWC_ETH_QOS_gen_pg_report(&pg_params, &pg_struct);
			break;
		case 06:
			DWC_ETH_QOS_print_pg_struct(&pg_struct);
			break;
		case 99:
			printf("Exiting....\n");
			break;
		default:
			printf("Sorry, wrong input....\n");
		}
	} while(user_input != 99);

	return 0;
}

#endif /* end of DWC_ETH_QOS_CONFIG_PGTEST */

void usage_common(char *cmd)
{
	if (print_all) {
		fprintf(stderr, "  %s",cmd);
	} else {
		fprintf(stderr, "\n  %s %s %s <parameters>\n\n", exe_name,
				(if_name ? if_name : "<interface_name>"),
				(cmd ? cmd : "<command>"));
		fprintf(stderr, "  supported parameters ::\n");
	}
}

void usage_qInx()
{
	if (!print_all)
		fprintf(stderr, "<qInx> is DMA channel number, it can be 0, "
				"1, 2, 3, 4, 5, 6, and 7\n\n");
}


void usage_dvlan_tx_via_reg()
{
	usage_common("dvlan_tx_via_reg");
	fprintf(stderr,
			"  <operation> <mode> <outer_vlan_tag> <inner_vlan_tag>\n"
			"           [operation\n"
			"                  0 - None, 1 - Deletion, 2 - Insertion, 3 - Replacement\n"
			"            mode\n"
			"                  0 - Outer VLAN, 1 - Inner VLAN, 2 - both Inner and Outer VLAN\n"
			"            outer_vlan_tag\n"
			"                  Outer VLAN tag value, including VLAN priority\n"
			"            inner_vlan_tag\n"
			"                  Inner VLAN tag value, including VLAN priority]\n"
			"\n");
}

void usage_dvlan_tx_via_desc()
{
	usage_common("dvlan_tx_via_desc");
	fprintf(stderr,
			"  <operation> <mode> <outer_vlan_tag> <inner_vlan_tag>\n"
			"           [operation\n"
			"                  0 - None, 1 - Deletion, 2 - Insertion, 3 - Replacement\n"
			"            mode\n"
			"                  0 - Outer VLAN, 1 - Inner VLAN, 2 - both Inner and Outer VLAN\n"
			"            outer_vlan_tag\n"
			"                  Outer VLAN tag value, including VLAN priority\n"
			"            inner_vlan_tag\n"
			"                  Inner VLAN tag value, including VLAN priority]\n"
			"\n");
}

void usage_dvlan_rx()
{
	usage_common("dvlan_rx");
	fprintf(stderr,
			"  <operation>\n"
			"           [operation\n"
			"                  0 - no vlan stripping.\n"
			"                  1 - Inner VLAN stripping only\n"
			"                  2 - Outer VLAN stripping only\n"
			"                  3 - Both Inner and Outer VLAN stripping]\n"
			"\n");
}

void usage_svlan_en()
{
	usage_common("svlan_en");
	fprintf(stderr,
			"  <operation>\n"
			"           [operation\n"
			"                  0 - disable S-VLAN\n"
			"                  1 - enable S-VLAN for Inner VLAN\n"
			"                  2 - enable S-VLAN for Outer VLAN\n"
			"                  3 - enable S-VLAN for both Inner and Outer VLAN]\n"
			"\n");
}
void usage_ptp_offload()
{
	usage_common("ptp_offload");
	fprintf(stderr,
			"  <disable/enable> <mode> <ptp_domain_num> <multicast_or_unicast>\n"
			"           [disable/enable\n"
			"                  0 - to disable ptp offloading\n"
			"                  1 - to enable ptp offloading\n"
			"            mode\n"
			"                  1 - Ordinary or Boundary Slave\n"
			"                  2 - Ordinary or Boundary Master\n"
			"                  3 - Transparent Slave\n"
			"                  4 - Transparent Master\n"
			"                  5 - Peer-to-Peer Transparent\n"
			"            ptp_domain_num\n"
			"                  PTP domain number]\n"
			"            multicast_or_unicast\n"
			"                  0 - multicast address]\n"
			"                  1 - unicast address]\n"
			"\n");
}


void usage_rxoutervlanstrip()
{
	usage_common("rxoutervlanstrip");
	fprintf(stderr,
			"  <0|1|2|3>\n"
			"     [0 - don't strip, 1 - strip if VLAN filter pass,\n"
			"      2 - strip if VLAN filter fail, 3 - strip always]\n"
			"\n");
}

void usage_rxinnervlanstrip()
{
	usage_common("rxinnervlanstrip");
	fprintf(stderr,
			"  <0|1|2|3>\n"
			"     [0 - don't strip, 1 - strip if VLAN filter pass,\n"
			"      2 - strip if VLAN filter fail, 3 - strip always]\n"
			"\n");
}

void usage_txvlan_desc()
{
	usage_common("txvlan_desc");
	fprintf(stderr,
			"  <qInx> <0|1|2|3>\n"
			"     [0 - No deletion/insertion/replacement, 1 - deletion,\n"
			"      2 - insertion 3 - replacement]\n"
			"\n");
}

void usage_txvlan_reg()
{
	usage_common("txvlan_reg");
	fprintf(stderr,
			"  <qInx> <0|1|2|3>\n"
			"     [0 - No deletion/insertion/replacement, 1 - deletion,\n"
			"      2 - insertion 3 - replacement]\n"
			"\n");
}

void usage_sa0_desc()
{
	usage_common("sa0_desc");
	fprintf(stderr,
			"  <0|1|2>\n"
			"     [0 - Do not include source address,\n"
			"      1 - include/insert source address,\n"
			"      2 - replace the source address]\n"
			"\n");
}

void usage_sa1_desc()
{
	usage_common("sa1_desc");
	fprintf(stderr,
			"  <0|1|2>\n"
			"     [0 - Do not include source address,\n"
			"      1 - include/insert source address,\n"
			"      2 - replace the source address]\n"
			"\n");
}

void usage_sa0_reg()
{
	usage_common("sa0_reg");
	fprintf(stderr,
			"  <0|1|2>\n"
			"     [0 - Do not include source address,\n"
			"      1 - include/insert source address,\n"
			"      2 - replace the source address]\n"
			"\n");
}

void usage_sa1_reg()
{
	usage_common("sa1_reg");
	fprintf(stderr,
			"  <0|1|2>\n"
			"      [0 - Do not include source address,\n"
			"       1 - include/insert source address,\n"
			"       2 - replace the source address]\n"
			"\n");
}

void usage_powerup()
{
	usage_common("powerup");
	fprintf(stderr,
			"  <magic|remote_wakeup>\n"
			"      [Gets the device out of powerdown mode (Allowed only if h/w supports\n"
			"       wake up on magic packet or remote wakeup)]\n"
			"\n");
}

void usage_powerdown()
{
	usage_common("powerdown");
	fprintf(stderr,
			"   <magic|remote_wakeup>\n"
			"     [Puts the device in powerdown mode. Device will powerup on manual 'powerup'\n"
			"      or on receiving magic packet/remote wakeup packet (Allowed only if\n"
			"      h/w supports wake up on magic packet or remote wakeup)]\n"
			"\n");
}

void usage_rx_threshold()
{
	usage_common("rx_threshold");
	fprintf(stderr,
			"  <qInx> <32|64|96|128> Bytes [To configure RX FIFO Threshold]\n"
			"\n");
}

void usage_tx_threshold()
{
	usage_common("tx_threshold");
	fprintf(stderr,
			"   <qInx> <32|64|96|128|192|256|384|512> Bytes [To configure TX FIFO Threshold]\n"
			"\n");
}

void usage_rsf()
{
	usage_common("rsf");
	fprintf(stderr,
			"  <qInx> <0|1> [0 - disable Receive Store and Forward Mode,\n"
			"                1 - enable Receive Store and Forward Mode]\n"
			"\n");
}

void usage_tsf()
{
	usage_common("tsf");
	fprintf(stderr,
			"  <qInx> <0|1> [0 - disable Transmit Store and Forward Mode,\n"
			"                1 - enable Transmit Store and Forward Mode]\n"
			"\n");
}

void usage_osf()
{
	usage_common("osf");
	fprintf(stderr,
			"  <qInx> <0|1> [0 - disable Transmit DMA to Operate on Second Frame,\n"
			"                1 - enable Transmit DMA to Operate on Second Frame]\n"
			"\n");
}

void usage_incrx_incr()
{
	usage_common("incrx_incr");
	fprintf(stderr,
			"  <0|1> [0 - select INCRX mode, 1 - select INCR mode]\n"
			"\n");
}

void usage_axi_pbl()
{
	usage_common("axi_pbl");
	fprintf(stderr,
			"  <4|8|16|32|64|128|256> [To configure AXI PBL]\n"
			"\n");
}

void usage_axi_worl()
{
	usage_common("axi_worl");
	fprintf(stderr,
			"  <1 to 31> [To configure AXI Maximum Write Outstanding Request Limit]\n"
			"\n");
}

void usage_axi_rorl()
{
	usage_common("axi_rorl");
	fprintf(stderr,
			"  <1 to 31> [To configure AXI Maximum Read Outstanding Request Limit]\n"
			"\n");
}

void usage_rx_pbl()
{
	usage_common("rx_pbl");
	fprintf(stderr,
			"  <qInx> <1|2|4|8|16|32|128|256> [To configre RX DMA PBL]\n"
			"\n");
}

void usage_tx_pbl()
{
	usage_common("tx_pbl");
	fprintf(stderr,
			"  <qInx> <1|2|4|8|16|32|128|256> [To configre TX DMA PBL]\n"
			"\n");
}

void usage_context()
{
	usage_common("context");
	fprintf(stderr,
			"  <qInx> <0|1>\n"
			"          [1 to setup a context descriptor with every normal descriptor\n"
			"           0 to set context descriptor only when VLAN ID Changes]\n"
			"\n");
}

void usage_queue_count()
{
	usage_common("queue_count");
	fprintf(stderr,
			"  [Displays number of Tx and Rx Queues/Channels supported by hardware]\n"
			"\n");
}

void usage_dcb()
{
	usage_common("dcb");
	fprintf(stderr,
			"  <qInx> <algorithm_value> <weights> <max-frame-size>\n"
			"            [algorithm_value\n"
			"      		  0 - WRR (Weighted Round Robin)\n"
			"      		  1 - WFQ (Weighted Fair Queuing)\n"
			"      		  2 - DWRR (Deficit Weighted Round Robin)\n"
			"      		  3 - SP (Strict Priority)\n"
			"      	      weights - value in terms of percentage of BW to be allocated\n"
			"             max-frame-size - maximum frame size]\n"
			"             NOTE:\n"
			"		\"max-frame-size\" is applicable for DWRR only\n"
			"               \"weigths\" are ignored for SP\n"
			"\n");
}

void usage_avb()
{
	usage_common("avb");
	fprintf(stderr,
			"  <qInx> <algorithm_value> <BW> <credit_control>\n"
			"             [algorithm value\n"
			"      		  0 - SP (Strict Priority)\n"
			"      		  1 - CBS (Credit Based Shaper)\n"
			"      	      BW - value in terms of percentage of BW to be allocated\n"
			"      	      credit_control\n"
			"                 0 - disabled credit_control\n"
			"                 1 - enabled credit_control]\n"
			"             NOTE:\n"
			"		\"credit_control\" is applicable for CBS only\n"
			"\n");
}

void usage_pgtest()
{
	usage_common("pgtest");
	fprintf(stderr, "\n\n");
}

void usage_split_hdr()
{
	usage_common("split_hdr");
	fprintf(stderr,
			"  <0|1>\n"
			"            [0 - disable split header, 1 - enable split header]\n"
			"\n");
}

void usage_l3_l4_filter()
{
	usage_common("l3_l4_filter");
	fprintf(stderr,
			"  <0/1>\n"
			"           [0 - disable l3/l4 filtering, 1 - enable l3/l4 filtering]\n"
			"\n");
}

void usage_ip4_filter()
{
	usage_common("ip4_filter");
	fprintf(stderr,
			"  <filter_no> <src/dst addr> <enable/disable> <perfect/inverse> <ip>\n"
			"             [filter_no - 0, 1, 2, 3, 4, 5, 6, or 7\n"
			"              src/dst addr\n"
			"                0 - src addr match, 1 - dst addr match\n"
			"              enable/disable\n"
			"                0 - to disable filter, 1 - to enable filter\n"
			"              perfect/inverse\n"
			"                0 - perfect match, 1 - inverse match\n"
			"              ip - IP address(eq-192.168.1.10)]\n"
			"\n");
}

void usage_ip6_filter()
{
	usage_common("ip6_filter");
	fprintf(stderr,
			"  <filter_no> <src/dst addr> <enable/disable> <perfect/inverse> <ip>\n"
			"             [filter_no - 0, 1, 2, 3, 4, 5, 6, or 7\n"
			"              src/dst addr\n"
			"                0 - src addr match, 1 - dst addr match\n"
			"              enable/disable\n"
			"                0 - to disable filter, 1 - to enable filter\n"
			"              perfect/inverse\n"
			"                0 - perfect match, 1 - inverse match\n"
			"              ip - IP address(eq - fe80:0:0:0:f2de:f1ff:fe58:de16)]\n"
			"\n");
}

void usage_udp_filter()
{
	usage_common("udp_filter");
	fprintf(stderr,
			"  <filter_no> <src/dst port> <enable/disable> <perfect/inverse> <port_no>\n"
			"             [filter_no - 0, 1, 2, 3, 4, 5, 6, or 7\n"
			"              src/dst port\n"
			"                0 - src port match, 1 - dst port match\n"
			"              enable/disable\n"
			"                0 - to disable filter, 1 - to enable filter\n"
			"              perfect/inverse\n"
			"                0 - perfect match, 1 - inverse match\n"
			"              port_no - port number to be matched]\n"
			"\n");
}

void usage_tcp_filter()
{
	usage_common("tcp_filter");
	fprintf(stderr,
			"  <filter_no> <src/dst port> <enable/disable> <perfect/inverse> <port_no>\n"
			"             [filter_no - 0, 1, 2, 3, 4, 5, 6, or 7\n"
			"              src/dst port\n"
			"                0 - src port match, 1 - dst port match\n"
			"              enable/disable\n"
			"                0 - to disable filter, 1 - to enable filter\n"
			"              perfect/inverse\n"
			"                0 - perfect match, 1 - inverse match\n"
			"              port_no - port number to be matched]\n"
			"\n");
}

void usage_vlan_filter()
{
	usage_common("vlan_filter");
	fprintf(stderr,
			"  <enable/disable> <perfect/hash filtering> <perfect/inverse matching>\n"
			"              [enable/disable\n"
			"                 0 - to disable, 1 - to enable VLAN filtering\n"
			"               perfect/hash filtering\n"
			"                 0 - perfect filtering, 1 - hash filtering\n"
			"               perfect/inverse matching\n"
			"                 0 - perfect matching, 1 - inverse matching]\n"
			"\n");
}

void usage_l2_da_filter()
{
	usage_common("l2_da_filter");
	fprintf(stderr,
			"  <perfect/hash filtering> <perfect/inverse match>\n"
			"            [perfect/hash filtering\n"
			"               0 - perfect filtering, 1 - hash filtering]\n"
			"             perfect/inverse matching\n"
			"               0 - perfect matching, 1 - inverse matching]\n"
			"\n");
}

void usage_arp()
{
	usage_common("arp");
	fprintf(stderr,
			"  <enable/disable>\n"
			"              [enable/disable\n"
			"                0 - to disable, 1 - to enable]\n"
			"\n");
}

void usage_mac_lb()
{
	usage_common("mac_lb");
	fprintf(stderr,
			"  <0|1>\n"
			"            [0 - to disable mac loopback, 1 - to enable mac loopback]\n\n"
			"            WARNING: Please disable the mac loopback after the testing\n"
			"                     else it will interrupt the normal operation.\n"
			"\n");
}

void usage_pfc()
{
	usage_common("pfc");
	fprintf(stderr,
			"  <enable|disable>\n"
			"     [enable/disable\n"
			"         0 - to disable, 1 - to enable]\n"
			"\n");
}

void print_all_usage()
{
	fprintf(stderr,
			"\nUsage:\n"
			"  %s <interface_name> <command> <parameters>\n\n",exe_name);
	print_all = 1;
	usage_qInx();
	usage_rxoutervlanstrip();
	usage_rxinnervlanstrip();
	usage_txvlan_desc();
	usage_txvlan_reg();
	usage_sa0_desc();
	usage_sa1_desc();
	usage_sa0_reg();
	usage_sa1_reg();
	usage_powerup();
	usage_powerdown();
	usage_rx_threshold();
	usage_tx_threshold();
	usage_rsf();
	usage_tsf();
	usage_osf();
	usage_incrx_incr();
	usage_axi_pbl();
	usage_axi_worl();
	usage_axi_rorl();
	usage_rx_pbl();
	usage_tx_pbl();
	usage_context();
	usage_queue_count();
	usage_dcb();
	usage_avb();
	usage_pgtest();
	usage_split_hdr();
	usage_l3_l4_filter();
	usage_ip4_filter();
	usage_ip6_filter();
	usage_udp_filter();
	usage_tcp_filter();
	usage_vlan_filter();
	usage_l2_da_filter();
	usage_arp();
	usage_mac_lb();
	usage_pfc();
	usage_dvlan_tx_via_reg();
	usage_dvlan_tx_via_desc();
	usage_dvlan_rx();
	usage_svlan_en();
   	usage_ptp_offload();
	print_all = 0;
}

main(int argc, char *argv[])
{
	int ret = 0;
	int sockfd;

	exe_name = strdup(argv[0]);

	if (argc >= 3) {
		sockfd = open_socket(argv[1]);
		if_name = strdup(argv[1]);
		if (sockfd < 0) {
			printf("unable to open %s socket\n", argv[1]);
			return sockfd;
		}

		tx_queue_count = DWC_ETH_QOS_get_tx_qcnt(sockfd, argv[1]);
		rx_queue_count = DWC_ETH_QOS_get_rx_qcnt(sockfd, argv[1]);
		connected_speed = DWC_ETH_QOS_get_connected_speed(sockfd, argv[1]);

		if (0 == strcmp(argv[2], "powerdown")) {
			if (argc < 4) {
				usage_powerdown();
				goto argc_failed;
			}
			ret = powerdown_device(sockfd, argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "powerup")) {
			if (argc < 4) {
				usage_powerup();
				goto argc_failed;
			}
			ret = powerup_device(sockfd, argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "rx_threshold")) {
			if (argc < 5) {
				usage_rx_threshold();
				goto argc_failed;
			}
			ret = config_rx_threshold(sockfd, argv[1], argv[3],
					argv[4]);
		} else if (0 == strcmp(argv[2], "tx_threshold")) {
			if (argc < 5) {
				usage_tx_threshold();
				goto argc_failed;
			}
			ret = config_tx_threshold(sockfd, argv[1], argv[3],
					argv[4]);
		} else if (0 == strcmp(argv[2], "rsf")) {
			if (argc < 5) {
				usage_rsf();
				goto argc_failed;
			}
			ret = config_rsf(sockfd, argv[1], argv[3], argv[4]);
		} else if (0 == strcmp(argv[2], "tsf")) {
			if (argc < 5) {
				usage_tsf();
				goto argc_failed;
			}
			ret = config_tsf(sockfd, argv[1], argv[3], argv[4]);
		} else if (0 == strcmp(argv[2], "osf")) {
			if (argc < 5) {
				usage_osf();
				goto argc_failed;
			}
			ret = config_osf(sockfd, argv[1], argv[3], argv[4]);
		} else if (0 == strcmp(argv[2], "incrx_incr")) {
			if (argc < 4) {
				usage_incrx_incr();
				goto argc_failed;
			}
			ret = config_incrx_incr(sockfd, argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "rx_pbl")) {
			if (argc < 5) {
				usage_rx_pbl();
				goto argc_failed;
			}
			ret = config_rx_pbl(sockfd, argv[1], argv[3], argv[4]);
		} else if (0 == strcmp(argv[2], "tx_pbl")) {
			if (argc < 5) {
				usage_tx_pbl();
				goto argc_failed;
			}
			ret = config_tx_pbl(sockfd, argv[1], argv[3], argv[4]);
		} 
#if 0        
        else if (0 == strcmp(argv[2], "rxoutervlanstrip")) {
			if (argc < 4) {
				usage_rxoutervlanstrip();
				goto argc_failed;
			}
			ret = rx_outer_vlan_strip(sockfd, argv[1], argv[3]);
		}
#endif        
           
                     
        else if (0 == strcmp(argv[2], "dvlan_tx_via_reg")) {
			if (argc < 4) {
				usage_dvlan_tx_via_reg();
				goto argc_failed;
			}

			ret = config_dvlan(sockfd, argc, argv,
					DWC_ETH_QOS_VIA_REG);
		} else if (0 == strcmp(argv[2], "dvlan_tx_via_desc")) {
			if (argc < 4) {
				usage_dvlan_tx_via_desc();
				goto argc_failed;
			}

			ret = config_dvlan(sockfd, argc, argv,
					DWC_ETH_QOS_VIA_DESC);
		} 
        else if (0 == strcmp(argv[2], "dvlan_rx")) {
			if (argc != 4) {
				usage_dvlan_rx();
				goto argc_failed;
			}

			ret = config_dvlan_rx_processing(sockfd,
					argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "svlan_en")) {
			if (argc != 4) {
				usage_svlan_en();
				goto argc_failed;
			}

			ret = config_svlan(sockfd, argv[1], argv[3]);
		}  else if (0 == strcmp(argv[2], "ptp_offload")) {
			if (argc < 5) {
				usage_ptp_offload();
				goto argc_failed;
			}

			if (argc == 4)
				ret = config_ptpoffloading(sockfd, argv[1],
								argv[3], NULL, NULL, NULL);
			else if (argc > 4)
				ret = config_ptpoffloading(sockfd, argv[1],
								argv[3], argv[4], argv[5], argv[6]);

		}       
        
        
        
        
        
        
        
        
        
#if 0        
        else if (0 == strcmp(argv[2], "rxinnervlanstrip")) {
			if (argc < 4) {
				usage_rxinnervlanstrip();
				goto argc_failed;
			}
			ret = rx_inner_vlan_strip(sockfd, argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "txvlan_desc")) {
			if (argc < 5) {
				usage_txvlan_desc();
				goto argc_failed;
			}
			ret = tx_vlan_ctrl_via_desc(sockfd, argv[1], argv[3], argv[4]);
		} else if (0 == strcmp(argv[2], "txvlan_reg")) {
			if (argc < 5) {
				usage_txvlan_reg();
				goto argc_failed;
			}
			ret = tx_vlan_ctrl_via_reg(sockfd, argv[1], argv[3], argv[4]);
		} 
#endif        
        else if (0 == strcmp(argv[2], "sa0_desc")) {
			if (argc < 4) {
				usage_sa0_desc();
				goto argc_failed;
			}
			ret = sa0_ctrl_via_desc(sockfd, argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "sa1_desc")) {
			if (argc < 4) {
				usage_sa1_desc();
				goto argc_failed;
			}
			ret = sa1_ctrl_via_desc(sockfd, argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "sa0_reg")) {
			if (argc < 4) {
				usage_sa0_reg();
				goto argc_failed;
			}
			ret = sa0_ctrl_via_reg(sockfd, argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "sa1_reg")) {
			if (argc < 4) {
				usage_sa1_reg();
				goto argc_failed;
			}
			ret = sa1_ctrl_via_reg(sockfd, argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "queue_count")) {
			DWC_ETH_QOS_get_rx_qcnt(sockfd, argv[1]);
			DWC_ETH_QOS_get_tx_qcnt(sockfd, argv[1]);
		} else if (0 == strcmp(argv[2], "context")) {
			if (argc < 5) {
				usage_context();
				goto argc_failed;
			}
			ret = setup_context_descriptor(sockfd, argv[1], argv[3], argv[4]);
		} else if (0 == strcmp(argv[2], "dcb")) {
			if (argc < 7) {
				usage_dcb();
				goto argc_failed;
			}
			ret = program_dcb_algorithm(sockfd, argv[1], argv[3], argv[4],
					argv[5], argv[6]);
		} else if (0 == strcmp(argv[2], "avb")) {
			if (argc < 7) {
				usage_avb();
				goto argc_failed;
			}
			ret = program_avb_algorithm(sockfd, argv[1], argv[3],
					argv[4], argv[5], argv[6]);
		}
#ifdef DWC_ETH_QOS_CONFIG_PGTEST
		else if (0 == strcmp(argv[2], "pgtest")) {
			ret = DWC_ETH_QOS_pg_test(sockfd, argv[1]);
		}
#endif /* DWC_ETH_QOS_CONFIG_PGTEST */
		else if (0 == strcmp(argv[2], "split_hdr")) {
			if (argc < 4) {
				usage_split_hdr();
				goto argc_failed;
			}
			ret = config_rx_split_hdr(sockfd, argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "l3_l4_filter")) {
			if (argc < 4) {
				usage_l3_l4_filter();
				goto argc_failed;
			}
			ret = config_l3_l4_filter(sockfd, argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "ip4_filter")) {
			if (argc < 5) {
				usage_ip4_filter();
				goto argc_failed;
			}
			if (!strcmp(argv[5], "1")) {
				if (argc != 8) {
					usage_ip4_filter();
					goto argc_failed;
				}

				ret = config_ipv4_filters(sockfd, argv[1],
					argv[3], argv[4], argv[5], argv[6],
					argv[7]);
			} else {
				ret = config_ipv4_filters(sockfd, argv[1],
					argv[3], argv[4], argv[5], NULL, NULL);
			}
		} else if (0 == strcmp(argv[2], "ip6_filter")) {
			if (argc < 5) {
				usage_ip6_filter();
				goto argc_failed;
			}
			if (!strcmp(argv[5], "1")) {
				if (argc != 8) {
					usage_ip6_filter();
					goto argc_failed;
				}

				ret = config_ipv6_filters(sockfd, argv[1],
					argv[3], argv[4], argv[5],
					argv[6], argv[7]);
			} else {
				ret = config_ipv6_filters(sockfd, argv[1],
					argv[3], argv[4], argv[5], NULL, NULL);
			}
		} else if ((0 == strcmp(argv[2], "udp_filter") ||
			 (0 == strcmp(argv[2], "tcp_filter")))) {
			if (argc < 5) {
				if (0 == strcmp(argv[2], "tcp_filter"))
					usage_tcp_filter();
				else
					usage_udp_filter();
				goto argc_failed;
			}
			if (!strcmp(argv[5], "1")) {
				if (argc != 8) {
					if (0 == strcmp(argv[2], "tcp_filter"))
						usage_tcp_filter();
					else
						usage_udp_filter();
					goto argc_failed;
				}

				ret = config_l4_filters(sockfd, argv[1],
					argv[2], argv[3], argv[4], argv[5],
					argv[6], argv[7]);
			} else {
				ret = config_l4_filters(sockfd, argv[1],
					argv[2], argv[3], argv[4], argv[5],
					NULL, NULL);
			}
		} else if (0 == strcmp(argv[2], "vlan_filter")) {
			if (argc < 4) {
				usage_vlan_filter();
				goto argc_failed;
			}
			if (!strcmp(argv[3], "1")) {
				if (argc != 6) {
					usage_vlan_filter();
					goto argc_failed;
				}

				ret = config_vlan_filters(sockfd, argv[1],
					argv[3], argv[4], argv[5]);
			} else {
				ret = config_vlan_filters(sockfd, argv[1],
					argv[3], NULL, NULL);
			}
		} else if (0 == strcmp(argv[2], "l2_da_filter")) {
			if (argc < 5) {
				usage_l2_da_filter();
				goto argc_failed;
			}

			ret = config_l2_da_filtering(sockfd, argv[1],
					 argv[3], argv[4]);
		} else if (0 == strcmp(argv[2], "arp")) {
			if (argc < 4) {
				usage_arp();
				goto argc_failed;
			}

			ret = config_arp_offload(sockfd, argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "axi_pbl")) {
			if (argc < 4) {
				usage_axi_pbl();
				goto argc_failed;
			}

			ret = config_axi_pbl(sockfd, argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "axi_worl")) {
			if (argc < 4) {
				usage_axi_worl();
				goto argc_failed;
			}

			ret = config_axi_worl(sockfd, argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "axi_rorl")) {
			if (argc < 4) {
				usage_axi_rorl();
				goto argc_failed;
			}

			ret = config_axi_rorl(sockfd, argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "mac_lb")) {
			if (argc < 4) {
				usage_mac_lb();
				goto argc_failed;
			}

			ret = config_loopback_mode(sockfd, argv[1], argv[3]);
		} else if (0 == strcmp(argv[2], "pfc")) {
			if (argc < 4) {
				usage_pfc();
				goto argc_failed;
			}

			ret = config_pfc(sockfd, argv[1], argv[3]);
		} else {
			print_all_usage();
		}

		close_socket(sockfd);

	} else {
		print_all_usage();
	}

	return ret;

argc_failed:
	printf("PLEASE SPECIFY CORRECT NUMBER OF ARGUMENTS\n");
	close_socket(sockfd);
}
