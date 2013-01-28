/*
 * Raydium RM31080 touchscreen driver
 *
 * Copyright (C) 2012 - 2013 Raydium Semiconductor Corporation
 * Copyright (C) 2012 NVIDIA Corporation, All Rights Reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
/*=============================================================================
 *INCLUDED FILES
 *=============================================================================
 */
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <linux/spi/rm31080a_ts.h>
#include <linux/spi/rm31080a_ctrl.h>
/*=============================================================================
 *DEFINITIONS
 *=============================================================================
 */
#define RM31080_RAW_DATA_LENGTH 2048

/*=============================================================================
 *STRUCTURE DECLARATION
 *=============================================================================
 */

/*=============================================================================
 *GLOBAL VARIABLES DECLARATION
 *=============================================================================
 */
struct rm31080a_ctrl_para g_stCtrl;

/*=============================================================================
 *FUNCTION DECLARATION
 *=============================================================================
 */
/*=============================================================================
 * Description:
 *      Control functions for Touch IC
 * Input:
 *
 * Output:
 *=============================================================================
 */
int rm31080_ctrl_clear_int(void)
{
	u8 flag;

	if (g_stCtrl.bICVersion != T007A6)
		return rm31080_spi_byte_read(RM31080B1_REG_BANK0_02H, &flag);
	else
		return rm31080_spi_byte_read(RM31080_REG_F2, &flag);
}

int rm31080_ctrl_scan_start(void)
{
	return rm31080_spi_byte_write(RM31080_REG_11, 0x17);
}

void rm31080_ctrl_wait_for_scan_finish(void)
{
	u8 u8reg11;
	int i;
	/* 50ms = 20Hz */
	for (i = 0; i < 50; i++) {
		rm31080_spi_byte_read(RM31080_REG_11, &u8reg11);
		if (u8reg11 & 0x01)
			usleep_range(1000, 2000);/*msleep(1); */
		else
			break;
	}
}


/*=============================================================================
 * Description:
 *
 * Input:
 *      N/A
 * Output:
 *      N/A
 *=============================================================================
 */
void rm_set_repeat_times(u8 u8Times)
{
	u8 bReg1_1Fh = 0x00;
	u8 u8Reg = 0x00;

	if (g_stCtrl.bICVersion != T007A6) {
		rm31080_spi_byte_read(0x0A, &u8Reg);
		u8Reg &= 0xF0;
		u8Reg |= (g_stCtrl.bSenseNumber - 1);
		rm31080_spi_byte_write(0x0A, u8Reg);

		rm31080_spi_byte_read(0x0E, &u8Reg);
		u8Reg &= ~0x1F;
		u8Reg |= u8Times;
		rm31080_spi_byte_write(0x0E, u8Reg);

		if ((g_stCtrl.bfADFC) &&
		(g_stCtrl.bICVersion != T007_VERSION_C))
			bReg1_1Fh |= ADFC;

		if (g_stCtrl.bfTHMode)
			bReg1_1Fh |= FILTER_THRESHOLD_MODE;
		else
			bReg1_1Fh &= ~FILTER_NONTHRESHOLD_MODE;
		/*bReg1_1Fh |= FILTER_NONTHRESHOLD_MODE;*/

		if (u8Times != REPEAT_1)
			bReg1_1Fh |= 0x44;	/*Marty 20120820*/

		rm31080_spi_byte_write(RM31080_REG_1F, bReg1_1Fh);
	} else {
		u8Reg = ((g_stCtrl.bSenseNumber - 1) << 3) | u8Times;

		rm31080_spi_byte_write(RM31080_REG_0E, u8Reg);
		if (g_stCtrl.bfADFC)
			bReg1_1Fh |= ADFC;

		if (g_stCtrl.bfTHMode)
			bReg1_1Fh |= FILTER_THRESHOLD_MODE;
		else
			bReg1_1Fh &= ~FILTER_NONTHRESHOLD_MODE;
			/*bReg1_1Fh |= FILTER_NONTHRESHOLD_MODE;*/

		if (u8Times != REPEAT_1)
			bReg1_1Fh |= (u8Times + 3);

		rm31080_spi_byte_write(RM31080_REG_1F, bReg1_1Fh);
	}
}

/*=============================================================================
 * Description:
 *
 * Input:
 *      N/A
 * Output:
 *      N/A
 *=============================================================================
 */
void rm31080_ctrl_init(void)
{
	u8 var;

	memset(&g_stCtrl, 0, sizeof(struct rm31080a_ctrl_para));

	g_stCtrl.bDummyRunCycle = 1;
	g_stCtrl.u16DataLength = RM31080_RAW_DATA_LENGTH;

	if (rm31080_spi_byte_read(RM31080_REG_7E, &var))
		g_stCtrl.bICVersion = var & 0xF0;
	else
		g_stCtrl.bICVersion = T007A6;
}

/*=============================================================================
 * Description:
 *
 * Input:
 *      N/A
 * Output:
 *      N/A
 *=============================================================================
 */
unsigned char rm31080_ctrl_get_noise_mode(u8 *p)
{
	u32 u32Ret;
	u32Ret = copy_to_user(p, &g_stCtrl.bfNoiseModeDetector, 1);
	if (u32Ret != 0)
		return 0;
	return 1;
}

/*=============================================================================
 * Description: To transfer the value to HAL layer
 *
 * Input:
 *      N/A
 * Output:
 *      N/A
 *===========================================================================*/
unsigned char rm31080_ctrl_get_idle_mode(u8 *p)
{
	u32 u32Ret;
	u32Ret = copy_to_user(p, &g_stCtrl.bfIdleModeCheck, 1);
	if (u32Ret != 0)
		return 0;
	return 1;
}

/*=============================================================================
 * Description:
 *
 * Input:
 *      N/A
 * Output:
 *      N/A
 *=============================================================================
 */
void rm31080_ctrl_get_parameter(void *arg)
{
#define PARA_BASIC_LEN 4
#define PARA_HARDWARE_LEN 28
#define PARA_NOISE_LEN 32
#define PARA_ALGORITHM_LEN 128

	/* Marty added*/
	u8 Temp;
	u8 *pPara;
	u8 idx;
	idx = PARA_BASIC_LEN;

	pPara = (u8 *) arg;
	Temp = pPara[idx + PARA_HARDWARE_LEN +
				PARA_NOISE_LEN + PARA_ALGORITHM_LEN + 3];
	rm31080_set_autoscan(Temp);

	g_stCtrl.bADCNumber = pPara[idx + 6];
	g_stCtrl.bChannelNumberX = pPara[idx];
	g_stCtrl.bChannelNumberY = pPara[idx + 1];
	g_stCtrl.bfNoiseDetector = pPara[idx + PARA_HARDWARE_LEN + 13];
	g_stCtrl.bChannelDetectorNum =	pPara[idx + PARA_HARDWARE_LEN + 19];
	g_stCtrl.bChannelDetectorDummy = pPara[idx + PARA_HARDWARE_LEN + 20];
	g_stCtrl.u16DataLength = (g_stCtrl.bChannelNumberX + 2 +
				g_stCtrl.bADCNumber) *
		(g_stCtrl.bChannelNumberY + g_stCtrl.bfNoiseDetector *
		(g_stCtrl.bChannelDetectorNum +
		g_stCtrl.bChannelDetectorDummy));
	g_stCtrl.bfNoiseMode = pPara[idx + PARA_HARDWARE_LEN + 14];
	g_stCtrl.bNoiseRepeatTimes = pPara[idx + PARA_HARDWARE_LEN + 15];
	/*Marty added*/
	g_stCtrl.bNoiseThresholdMax = pPara[idx + PARA_HARDWARE_LEN + 21];
	g_stCtrl.bNoiseThresholdMin = pPara[idx + PARA_HARDWARE_LEN + 22];
	g_stCtrl.bNoiseThresholdLowMax = pPara[idx + PARA_HARDWARE_LEN + 23];
	g_stCtrl.bNoiseThresholdLowMin = pPara[idx + PARA_HARDWARE_LEN + 24];
	g_stCtrl.bNoiseDetectThd = pPara[idx + PARA_HARDWARE_LEN + 16];
	g_stCtrl.bNoisePipelineBase = pPara[idx + PARA_HARDWARE_LEN + 17];
	g_stCtrl.bNewNoiseRepeatTimes = pPara[idx + PARA_HARDWARE_LEN + 18];
	g_stCtrl.bfMediumFilter = pPara[idx + PARA_HARDWARE_LEN + 25];
	g_stCtrl.bMFBlockNumber = pPara[idx + PARA_HARDWARE_LEN + 26];
	g_stCtrl.bRepeatTimes[0] = pPara[idx + PARA_HARDWARE_LEN + 4];
	g_stCtrl.bRepeatTimes[1] = pPara[idx + PARA_HARDWARE_LEN + 5];
	g_stCtrl.bIdleRepeatTimes[0] = pPara[idx + PARA_HARDWARE_LEN + 6];
	g_stCtrl.bSenseNumber = pPara[idx + 11];
	g_stCtrl.bfADFC = pPara[idx + 4];
	g_stCtrl.bfTHMode = pPara[idx + PARA_HARDWARE_LEN + 10];
	g_stCtrl.bfAnalogFilter = pPara[idx + PARA_HARDWARE_LEN + 8];
	g_stCtrl.bYChannel[0] = pPara[280];	/* Y chan start pin */
	g_stCtrl.bYChannel[1] = pPara[281];	/* Y chan end pin */
	g_stCtrl.bXChannel[0] = pPara[282];	/* X chan ADC1 start pin */
	g_stCtrl.bXChannel[1] = pPara[283];	/* X chan ADC1 end pin */
	g_stCtrl.bXChannel[2] = pPara[284];	/* X chan ADC2 start pin */
	g_stCtrl.bXChannel[3] = pPara[285];	/* X chan ADC2 end pin */
	g_stCtrl.bfSuspendReset = pPara[idx + PARA_HARDWARE_LEN + 27];
	g_stCtrl.bPressureResolution = pPara[105];
	g_stCtrl.bSTScan = pPara[191];
	g_stCtrl.bMTTouchThreshold = pPara[192];
	g_stCtrl.bTime2Idle = pPara[194];
	g_stCtrl.bfPowerMode = pPara[195];
	g_stCtrl.bfIdleMessage = pPara[207];
	g_stCtrl.bDummyRunCycle = pPara[31];

	/* Store dummy channel to skip it, data sequence:*/
	/* Dummy[0](single end) | raw_data |
		dummy[1](single end) dummy[2](single end) | raw_data |
		dummy[3](single end)*/

	g_stCtrl.u16ResolutionX = ((u16) pPara[PARA_BASIC_LEN + 13]) << 8 |
		((u16)pPara[PARA_BASIC_LEN + 12]);
	g_stCtrl.u16ResolutionY = ((u16) pPara[PARA_BASIC_LEN + 15]) << 8 |
		((u16)pPara[PARA_BASIC_LEN + 14]);

	if ((g_stCtrl.u16ResolutionX == 0) || (g_stCtrl.u16ResolutionY == 0)) {
		g_stCtrl.u16ResolutionX = RM_INPUT_RESOLUTION_X;
		g_stCtrl.u16ResolutionY = RM_INPUT_RESOLUTION_Y;
	}

}

/*===========================================================================*/
MODULE_AUTHOR("xxxxxxxxxx <xxxxxxxx@rad-ic.com>");
MODULE_DESCRIPTION("Raydium touchscreen control functions");
MODULE_LICENSE("GPL");
