/*
 * Raydium RM31080 touchscreen driver
 *
 * Copyright (C) 2012 Raydium Semiconductor Corporation
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
#define MAX_AVERAGE_TIMES		128
/*=============================================================================
 *STRUCTURE DECLARATION
 *=============================================================================
 */

/*=============================================================================
 *GLOBAL VARIABLES DECLARATION
 *=============================================================================
 */
struct rm31080a_ctrl_para g_stCtrl;

/* Marty added*/
static u8 g_pbBaseline[RM31080_RAW_DATA_LENGTH];	/* Noise_Detector */
#if ENABLE_NEW_NOISE_MODE
static u8 g_pbAnalogBaseline[RM31080_RAW_DATA_LENGTH];	/* Noise_Detector */
#endif
signed int g_noiseLevel[3];	/* Noise_Detector */
unsigned short g_noisechecktimes;	/* Noise_Detector */

/* Marty added*/
s8 g_bAverageBuf[MAX_AVERAGE_TIMES][RM31080_RAW_DATA_LENGTH];
s8 g_bMFBlockMax[MAX_AVERAGE_TIMES];
s8 g_bMFBlockMin[MAX_AVERAGE_TIMES];
u8 g_bMFCounter[MAX_AVERAGE_TIMES];
s8 g_bRawData[MAX_AVERAGE_TIMES][16];
u8 g_bfFirstAverage;
/* Marty added*/
signed char bDTImage[60][8];
unsigned char bfSign[60];

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
#if ENABLE_T007B1_SETTING
	if (g_stCtrl.bICVersion != T007A6)
		return rm31080_spi_byte_read(RM31080B1_REG_BANK0_02H, &flag);
	else
#endif
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
/*Marty added*/
int rm31080_soft_average(signed char *pSource)
{
	static u8 u8AverageIndex;/* = 0; */
	static u8 u8AverageIndexMinor;/* = 0; */
	static u8 u8StartAverage;/* = 0; */
	u16 i, j, k;
	s16 s16Sum;
	u8 bMaxBlockId;
	u8 TestA, TestB, TestC;
	s8 bMax, bMin;
	s8 bResult0, bResult1, bResult2;
	u8 bRes;
	u8 bAverageTimes;
	static u8 bAverageRatio;/* = 0; */

	bResult1 = 0;
	bResult2 = 0;
	bAverageTimes = g_stCtrl.bNoiseRepeatTimes;

	if (g_stCtrl.bfNoisePreHold) {
		u8AverageIndex = 0;
		u8AverageIndexMinor = 0;
		bAverageRatio = 0;
		g_bfFirstAverage = 0;

		return 0;
	}
#if ENABLE_NEW_NOISE_MODE
	if (!((g_stCtrl.bfNoiseMode & 0x01) || g_stCtrl.bfNoiseModeDetector)) {
#else
	if (!(g_stCtrl.bfNoiseMode || g_stCtrl.bfNoiseModeDetector)) {
#endif
		u8AverageIndex = 0;
		u8AverageIndexMinor = 0;
		bAverageRatio = 0;
		g_bfFirstAverage = 0;
		if (g_stCtrl.bfExitNoiseMode) {
			g_stCtrl.bfExitNoiseMode = 0;
			return 0;
		} else {
			return 1;
		}
	}

	for (i = 0; i < g_stCtrl.u16DataLength; i++) {
		g_bAverageBuf[u8AverageIndex][i] = g_pbBaseline[i]
							- pSource[i];
	}
	u8AverageIndex++;
	if (g_stCtrl.bNoiseRepeatTimes > g_stCtrl.bNoisePipelineBase)
		u8AverageIndexMinor++;

	if (g_stCtrl.bNoiseRepeatTimes <= g_stCtrl.bNoisePipelineBase) {
		bAverageTimes = g_stCtrl.bNoiseRepeatTimes;
		if (u8AverageIndex == g_stCtrl.bNoiseRepeatTimes) {
			u8StartAverage = 1;
			u8AverageIndex = 0;
		}
#if 1
		else
			u8StartAverage = 0;
#endif
	} else {
		if (!g_bfFirstAverage) {
			if (u8AverageIndex < g_stCtrl.bNoiseRepeatTimes) {
				if (u8AverageIndexMinor ==
				g_stCtrl.bNoisePipelineBase) {
					u8StartAverage = 1;
					u8AverageIndexMinor = 0;
					bAverageRatio++;
					bAverageTimes =
						g_stCtrl.bNoisePipelineBase *
								bAverageRatio;
				} else
					u8StartAverage = 0;
			} else {
				if (u8AverageIndex ==
				g_stCtrl.bNoiseRepeatTimes) {
					u8AverageIndex = 0;
					if (u8AverageIndexMinor ==
						g_stCtrl.bNoisePipelineBase) {
						u8StartAverage = 1;
						u8AverageIndexMinor = 0;
						bAverageRatio++;
						bAverageTimes =
						g_stCtrl.bNoisePipelineBase *
							bAverageRatio;
						g_bfFirstAverage = 1;
					} else {
						bAverageTimes =
						g_stCtrl.bNoiseRepeatTimes;
						g_bfFirstAverage = 1;
						u8StartAverage = 0;
					}
				}
			}
		} else {
			bAverageTimes = g_stCtrl.bNoiseRepeatTimes;
			if (u8AverageIndexMinor ==
			g_stCtrl.bNoisePipelineBase) {
				u8StartAverage = 1;
				u8AverageIndexMinor = 0;
			} else
				u8StartAverage = 0;

			if (u8AverageIndex == g_stCtrl.bNoiseRepeatTimes)
				u8AverageIndex = 0;
		}
	}

	if (u8StartAverage) {
		for (i = 0; i < g_stCtrl.u16DataLength; i++) {
			if (g_stCtrl.bfMediumFilter & 0x01) {
				for (j = 0; j < g_stCtrl.bMFBlockNumber; j++)
					g_bMFCounter[j] = 0;
				for (j = 0; j < bAverageTimes; j++) {
					for (k = 0;
					k < g_stCtrl.bMFBlockNumber;
					k++) {
						if (g_bAverageBuf[j][i] <=
						g_bMFBlockMax[k] &&
						g_bAverageBuf[j][i] >=
						g_bMFBlockMin[k]) {
							g_bRawData[
							g_bMFCounter[k]][k] =
							g_bAverageBuf[j][i];
							g_bMFCounter[k]++;
							break;
						}
					}
				}
				bMaxBlockId = 0;
				for (j = 0; j < g_stCtrl.bMFBlockNumber; j++) {
					if (g_bMFCounter[j] >
					g_bMFCounter[bMaxBlockId]) {
						bMaxBlockId = j;
					} else if (g_bMFCounter[j] ==
					g_bMFCounter[bMaxBlockId]) {
						if
						(j > g_stCtrl.bMFBlockNumber/2)
							TestA =
							j -
							g_stCtrl.bMFBlockNumber
							/ 2;
						else
							TestA =
							g_stCtrl.bMFBlockNumber
							/ 2 - j;
						if (bMaxBlockId >
						g_stCtrl.bMFBlockNumber / 2)
							TestB = bMaxBlockId -
							g_stCtrl.bMFBlockNumber
								/ 2;
						else
							TestB =
							g_stCtrl.bMFBlockNumber
								/ 2
								- bMaxBlockId;
						if (TestA < TestB)
							bMaxBlockId = j;
					}
				}

				s16Sum = 0;
				for (j = 0; j < g_bMFCounter[bMaxBlockId];
				j++) {
					s16Sum += g_bRawData[j][bMaxBlockId];
				}
				if (g_bMFCounter[bMaxBlockId] == 0) {
					bResult1 = 0;
				} else {
					bResult1 = (s16Sum /
						g_bMFCounter[bMaxBlockId]);
					/*if (g_stCtrl.bfNoiseDetector)
					{
						bResult1 = (s16)bResult1*4 / 5;
					}*/
				}
			}

			if (g_stCtrl.bfMediumFilter & 0x02) {
				bMax = -128;
				bMin = 127;
				for (j = 0; j < bAverageTimes; j++) {
					if (g_bAverageBuf[j][i] > bMax)
						bMax = g_bAverageBuf[j][i];
					if (g_bAverageBuf[j][i] < bMin)
						bMin = g_bAverageBuf[j][i];
				}
				bResult2 = (bMax + bMin) / 2;
				/*if (g_stCtrl.bfNoiseDetector)
				{
				bResult2 = (s16)bResult2 * 4 / 5;
				}*/

			}

			s16Sum = 0;
			for (j = 0; j < bAverageTimes; j++) {
				bRes = i %
					(g_stCtrl.bChannelNumberX + 2 +
						g_stCtrl.bADCNumber);
				if (bRes > 0 && bRes < 4) {
					if (g_bAverageBuf[j][i] <
					g_stCtrl.bNoiseThresholdLowMax &&
					g_bAverageBuf[j][i] >
					g_stCtrl.bNoiseThresholdLowMin)
						g_bAverageBuf[j][i] = 0;
					else if (g_bAverageBuf[j][i] > 40)
						g_bAverageBuf[j][i] = 40;
					else if (g_bAverageBuf[j][i] < -40)
						g_bAverageBuf[j][i] = -40;
				} else {
					if (g_bAverageBuf[j][i] <
					g_stCtrl.bNoiseThresholdLowMax &&
					g_bAverageBuf[j][i] >
					g_stCtrl.bNoiseThresholdLowMin)
						g_bAverageBuf[j][i] = 0;
					else if (g_bAverageBuf[j][i] >
					g_stCtrl.bNoiseThresholdMax)
						g_bAverageBuf[j][i] =
						g_stCtrl.bNoiseThresholdMax;
					else if (g_bAverageBuf[j][i] <
					g_stCtrl.bNoiseThresholdMin)
						g_bAverageBuf[j][i] =
						g_stCtrl.bNoiseThresholdMin;
				}

				s16Sum += g_bAverageBuf[j][i];
			}
			bResult0 = (s16Sum / g_stCtrl.bNoiseRepeatTimes);
			/*if (g_stCtrl.bfNoiseDetector)
			{
				bResult0 = (s16)bResult0 * 4 / 5;
			}*/

			if (g_stCtrl.bfMediumFilter & 0x01) {
				if (bResult0 > 0)
					TestA = bResult0;
				else
					TestA = -bResult0;
				if (bResult1 > 0)
					TestB = bResult1;
				else
					TestB = -bResult1;
				if (TestA < TestB)
					pSource[i] = g_pbBaseline[i] - bResult0;
				else
					pSource[i] = g_pbBaseline[i] - bResult1;
			} else if (g_stCtrl.bfMediumFilter & 0x02) {
				if (bResult0 > 0)
					TestA = bResult0;
				else
					TestA = -bResult0;
				if (bResult2 > 0)
					TestC = bResult2;
				else
					TestC = -bResult2;
				if (TestA < TestC)
					pSource[i] = g_pbBaseline[i] - bResult0;
				else
					pSource[i] = g_pbBaseline[i] - bResult2;
			} else if (g_stCtrl.bfMediumFilter & 0x03) {
				if (bResult0 > 0)
					TestA = bResult0;
				else
					TestA = -bResult0;
				if (bResult1 > 0)
					TestB = bResult1;
				else
					TestB = -bResult1;
				if (bResult2 > 0)
					TestC = bResult2;
				else
					TestC = -bResult2;
				if ((TestA < TestB) && (TestA < TestC))
					pSource[i] = g_pbBaseline[i]
						- bResult0;
				else if ((TestB < TestA) && (TestB < TestC))
					pSource[i] = g_pbBaseline[i]
						- bResult1;
				else if ((TestC < TestA) && (TestC < TestB))
					pSource[i] = g_pbBaseline[i]
						- bResult2;

			} else {
				pSource[i] = g_pbBaseline[i] - bResult0;
			}
		}

		return 1;
	}
	return 0;
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
#if ENABLE_T007B1_SETTING
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
	} else
#endif
	{
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
int rm_noise_detect(signed char *pSource)
{
	int tRet = ND_NORMAL;

	unsigned short Offset, XNum, i, j;
	signed int i32NoiseLevel = 0;
	signed int noise_value = 0;
	signed char bdata, bdata0;
	unsigned char bfTouched = 0;
	signed char bTestData = 0;
	unsigned char *pbBaseline;
	unsigned short wSENum = 2;

#if ENABLE_T007B1_SETTING
	if (g_stCtrl.bICVersion != T007A6) {
		wSENum = 0;
		return ND_DETECTOR_OFF;
	}
#endif

	if (!g_stCtrl.bfNoiseDetector)
		return ND_DETECTOR_OFF;

#if ENABLE_NEW_NOISE_MODE
	if (g_stCtrl.bfNoiseModeDetector)
		pSource[0] = 0x10;
	else
		pSource[0] = 0x00;

	if (g_stCtrl.bfNoiseModeDetector && (g_stCtrl.bfNoiseMode & 0x02))
		pbBaseline = g_pbAnalogBaseline;
	else
#endif
		pbBaseline = g_pbBaseline;

	if (!g_stCtrl.bBaselineReady)
		return tRet;

	if (g_stCtrl.bChannelDetectorNum <= 0)
		return ND_BASELINE_NOT_READY;
	bfTouched = 0;
	XNum = g_stCtrl.bChannelNumberX +
		wSENum + g_stCtrl.bADCNumber;
	Offset = XNum * (g_stCtrl.bChannelNumberY +
			g_stCtrl.bChannelDetectorDummy);
	for (i = 0; i < XNum; i++) {
		if (g_stCtrl.bADCNumber) {
			if ((i == g_stCtrl.bDummyChannel[0])
			|| (i == g_stCtrl.bDummyChannel[1])
			|| (i == g_stCtrl.bDummyChannel[2])
			|| (i == g_stCtrl.bDummyChannel[3])) {
				continue;
			}
		} else {
			if ((i == g_stCtrl.bDummyChannel[0]) ||
			(i == g_stCtrl.bDummyChannel[1])) {
				continue;
			}
		}

		bTestData = 0;
		for (j = 0; j < g_stCtrl.bChannelNumberY; j++) {
			bTestData = pbBaseline[j * XNum + i] -
					pSource[j * XNum + i];
			if (bTestData >
			g_stCtrl.bMTTouchThreshold ||
			bTestData <
			-g_stCtrl.bMTTouchThreshold) {
				bfTouched = 1;
				break;
			}
		}
		if (bfTouched == 1)
			break;
	}
	for (i = 0; i < XNum; i++) {
		bfSign[i] = 0;
		for (j = 0; j < g_stCtrl.bChannelDetectorNum;
		j++) {
			bDTImage[i][j] = pbBaseline[Offset
				+ j * XNum + i] -
				pSource[Offset + j * XNum + i];
		}
	}
	for (i = 0; i < XNum; i++) {
		bdata = 0;
		bdata0 = 0;
		for (j = 0; j < g_stCtrl.bChannelDetectorNum;
		j++) {
			if (bDTImage[i][j] > 0)
				bdata++;
			else if (bDTImage[i][j] < 0)
				bdata--;
			else
				bdata0++;
		}
		if (((bdata + bdata0) ==
		g_stCtrl.bChannelDetectorNum) ||
		((bdata0 - bdata) ==
		g_stCtrl.bChannelDetectorNum)) {
			bfSign[i] = 1;
		}
	}

	i32NoiseLevel = 0;
	for (i = 0; i < XNum; i++) {
		if (g_stCtrl.bADCNumber) {
			if ((i == g_stCtrl.bDummyChannel[0])
			|| (i == g_stCtrl.bDummyChannel[1])
			|| (i == g_stCtrl.bDummyChannel[2])
			|| (i == g_stCtrl.bDummyChannel[3])) {
				continue;
			}
		} else {
			if ((i == g_stCtrl.bDummyChannel[0]) ||
			(i == g_stCtrl.bDummyChannel[1])) {
				continue;
			}
		}

		for (j = 0; j < g_stCtrl.bChannelDetectorNum;
		j++) {
			if (bDTImage[i][j] < 0)
				noise_value = -bDTImage[i][j];
			else
				noise_value = bDTImage[i][j];

			if (noise_value > 2 &&
			((bfSign[i] == 0 &&
			g_stCtrl.bfNoiseModeDetector == 0) ||
			g_stCtrl.bfNoiseModeDetector == 1)) {
				i32NoiseLevel += noise_value;
			}
		}
	}

	if (g_noisechecktimes == 0) {
		g_noiseLevel[0] = i32NoiseLevel;
	} else if (g_noisechecktimes == 1) {
		g_noiseLevel[1] = i32NoiseLevel;
	} else if (g_noisechecktimes == 2) {
		g_noiseLevel[2] = i32NoiseLevel;
	} else {
		g_noiseLevel[0] = g_noiseLevel[1];
		g_noiseLevel[1] = g_noiseLevel[2];
		g_noiseLevel[2] = i32NoiseLevel;
	}
	i32NoiseLevel = g_noiseLevel[0] +
			g_noiseLevel[1] +
			g_noiseLevel[2];
	g_noisechecktimes++;
	if (g_noisechecktimes > 50)
		g_noisechecktimes = 50;

	if (g_stCtrl.bfNoiseModeDetector == 0) {
		if (i32NoiseLevel >
			g_stCtrl.bNoiseDetectThd ||
			g_stCtrl.bfNoisePreHold > 5) {
			if ((g_noiseLevel[0] != 0) &&
				(g_noiseLevel[1] != 0) &&
				(g_noiseLevel[2] != 0)) {
				rm31080_disable_touch();
				usleep_range(10000, 20000);/*msleep(10); */
				rm31080_ctrl_clear_int();
#if ENABLE_NEW_NOISE_MODE
				if (g_stCtrl.bfNoiseMode & 0x02) {
					rm31080_analog_filter_config(
					g_stCtrl.bNewNoiseRepeatTimes);
				} else
#endif
				{
					rm_set_repeat_times(REPEAT_1);
				}
				g_stCtrl.bfNoiseModeDetector = 1;
				g_stCtrl.bfNoisePreHold = 0;

				tRet = ND_NOISE_DETECTED;

				rm31080_enable_touch();
				rm31080_ctrl_scan_start();
			}
		} else {
			if (!g_stCtrl.bfTouched &&
			bfTouched && i32NoiseLevel > 0)
				g_stCtrl.bfNoisePreHold++;
			else if (g_stCtrl.bfNoisePreHold &&
				bfTouched &&
				i32NoiseLevel > 0
				&& (g_noiseLevel[0] != 0)
				&& (g_noiseLevel[1] != 0)
				&& (g_noiseLevel[2] != 0))
				g_stCtrl.bfNoisePreHold++;
			else
				g_stCtrl.bfNoisePreHold = 0;
		}
	} else {
		if (g_stCtrl.bfNoiseModeDetector == 1 &&
			!g_stCtrl.bfSuspendReset){
			if (bfTouched == 0) {
				rm31080_disable_touch();
				msleep(50);
				rm31080_ctrl_clear_int();
#if ENABLE_NEW_NOISE_MODE
				if (g_stCtrl.bfNoiseMode & 0x02) {
					if (g_stCtrl.bfAnalogFilter)
						rm_set_repeat_times(
						g_stCtrl.bRepeatTimes[1]);
					else
						rm31080_digital_filter_config();
				} else
#endif
				{
					rm_set_repeat_times(
					g_stCtrl.bRepeatTimes[
					g_stCtrl.bfAnalogFilter]);
				}
				g_stCtrl.bfNoiseModeDetector = 0;
				g_stCtrl.bfNoisePreHold = 0;
				g_stCtrl.bfExitNoiseMode = 1;

				tRet = ND_LEAVE_NOISE_MODE;

				rm31080_enable_touch();
				rm31080_ctrl_scan_start();
			}

		}
	}

	g_stCtrl.bfTouched = bfTouched;


	return tRet;
}

/*===========================================================================*/
int rm_noise_main(signed char *pSource)
{
	int iRet = 1;

	if (ND_NOISE_DETECTED == rm_noise_detect(pSource))
		iRet = 0;

#if ENABLE_T007B1_SETTING
	if (g_stCtrl.bICVersion != T007A6)
		return iRet;
#endif

#if ENABLE_NEW_NOISE_MODE
	if (!(g_stCtrl.bfNoiseMode & 0x02))
#endif
		iRet = rm31080_soft_average(pSource);

	return iRet;
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
void rm31080_ctrl_set_baseline(void *arg)
{
	u8 *pRawPtr;
	u16 ii;
	pRawPtr = (u8 *) arg;
	for (ii = 0; ii < g_stCtrl.u16DataLength; ii++)
		g_pbBaseline[ii] = pRawPtr[ii + 1];
	g_stCtrl.bBaselineReady = 1;
}

#if ENABLE_NEW_NOISE_MODE
/*=============================================================================
 * Description:
 *
 * Input:
 *      N/A
 * Output:
 *      N/A
 *=============================================================================
 */
void rm31080_ctrl_set_analog_baseline(void *arg)
{
	u8 *pRawPtr;
	u16 ii;
	pRawPtr = (u8 *) arg;
	for (ii = 0; ii < g_stCtrl.u16DataLength; ii++)
		g_pbAnalogBaseline[ii] = pRawPtr[ii];
	g_stCtrl.bBaselineReady = 1;
}
#endif
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
#if ENABLE_T007B1_SETTING
	u8 var;
#endif
	memset(&g_stCtrl, 0, sizeof(struct rm31080a_ctrl_para));
	/*Marty added */
	g_stCtrl.bBaselineReady = 0;	/* Noise_Detector*/
	g_noiseLevel[0] = 0;	/* Noise_Detector */
	g_noiseLevel[1] = 0;	/* Noise_Detector */
	g_noiseLevel[2] = 0;	/* Noise_Detector */
	g_noisechecktimes = 0;	/* Noise_Detector */
	g_stCtrl.bfNoiseModeDetector = 0;
	g_stCtrl.bfNoisePreHold = 0;
	g_stCtrl.bfTouched = 0;
	g_stCtrl.bfExitNoiseMode = 0;
	g_stCtrl.bDummyRunCycle = 1;

	g_stCtrl.u16DataLength = RM31080_RAW_DATA_LENGTH;
	g_bfFirstAverage = 0;

#if ENABLE_T007B1_SETTING
	if (rm31080_spi_byte_read(RM31080_REG_7E, &var))
		g_stCtrl.bICVersion = var & 0xF0;
	else
		g_stCtrl.bICVersion = T007A6;
#endif
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


#if ENABLE_T007B1_SETTING

void rm31080b_sw_reset(void)
{
	unsigned char u8Value;

	/* sw reset */
	rm31080_spi_byte_read(RM31080B1_REG_BANK0_11H, &u8Value);
	u8Value &= ~0x04;
	/*Send software reset.*/
	rm31080_spi_byte_write(RM31080B1_REG_BANK0_11H, u8Value);
}

/*=============================================================================
 * Description: Set T007B analog filter repeat
 *
 * Input:
 *      Analog average tap number
 * Output:
 *      none
 *=============================================================================
 */
void rm31080b_analog_filter_config(unsigned char u8Amount)
{
	unsigned char u8Value;

	rm31080_spi_byte_read(RM31080B1_REG_BANK0_0BH, &u8Value);
	u8Value &= ~0x0F;
	u8Value |= u8Amount;
	rm31080_spi_byte_write(RM31080B1_REG_BANK0_0BH, u8Amount);

	rm31080b_sw_reset();
}

/*=============================================================================
 * Description: Set T007B digital  filter repeat
 *
 * Input:
 *      Digital average tap number
 * Output:
 *      none
 *=============================================================================
 */
void rm31080b_digital_filter_config(unsigned char u8Amount)
{
	unsigned char u8Value;

	rm31080_spi_byte_read(RM31080B1_REG_BANK0_0EH, &u8Value);
	u8Value &= ~0x1F;
	u8Value |= u8Amount;
	rm31080_spi_byte_write(RM31080B1_REG_BANK0_0EH, u8Value);

	rm31080b_sw_reset();
}
#endif	/* #if ENABLE_T007B1_SETTING */

/*=============================================================================
 * Description:
 *
 * Input:
 *      N/A
 * Output:
 *      N/A
 *=============================================================================
 */

#if ENABLE_FILTER_SWITCH
void rm31080_analog_filter_config(u8 bRepeatTimes)
{
#if ENABLE_T007B1_SETTING
	if (g_stCtrl.bICVersion != T007A6)
		return;
#endif

	/*u8 bReg0_1Fh = 0x00;*/

	/*InitChargePump();*/
	rm31080_spi_byte_write(0x7F, 0x01);

	rm31080_spi_byte_write(0x09, g_stCtrl.bReg1_09h[1]);
	rm31080_spi_byte_write(0x43, g_stCtrl.bReg1_43h[1]);
	rm31080_spi_byte_write(0x48, g_stCtrl.bReg1_48h[1]);
	rm31080_spi_byte_write(0x49, g_stCtrl.bReg1_49h[1]);
	rm31080_spi_byte_write(0x4A, g_stCtrl.bReg1_4Ah[1]);
	rm31080_spi_byte_write(0x4B, g_stCtrl.bReg1_4Bh[1]);

	rm31080_spi_byte_write(0x7F, 0x00);

#if 0
	/*bSenseNumber = 8, REPEAT_1*/
	rm31080_spi_byte_write(0x0E, (0x38 | 0x00));

	rm31080_spi_byte_read(0x1F, &bReg0_1Fh);
	bReg0_1Fh = bReg0_1Fh & 0xF8;
	bReg0_1Fh = bReg0_1Fh | 0x00;	/*REPEAT_1*/
	rm31080_spi_byte_write(0x1F, bReg0_1Fh);
#endif
	rm_set_repeat_times(bRepeatTimes);

	/* -------- Set Driving / Sensing Control Mode --------*/
	rm31080_spi_byte_write(RM31080_REG_10, 0x80);	/*SSC_ENB*/

	/* -------- Set PGA/DAC -------- */
	rm31080_spi_byte_write(0x6B, 0xF1);	/*CSSEL*/

	/* -------- Scan Time Setting -------- */
	rm31080_spi_byte_write(RM31080_REG_42, g_stCtrl.bReg0_42h[1]);
	rm31080_spi_byte_write(RM31080_REG_43, g_stCtrl.bReg0_43h[1]);

	rm31080_spi_byte_write(0x20, g_stCtrl.bReg0_20h[1]);
	rm31080_spi_byte_write(0x21, g_stCtrl.bReg0_21h[1]);
	rm31080_spi_byte_write(0x22, g_stCtrl.bReg0_22h[1]);
	rm31080_spi_byte_write(0x23, g_stCtrl.bReg0_23h[1]);
	rm31080_spi_byte_write(0x24, g_stCtrl.bReg0_24h[1]);
	rm31080_spi_byte_write(0x25, g_stCtrl.bReg0_25h[1]);
	rm31080_spi_byte_write(0x26, g_stCtrl.bReg0_26h[1]);
	rm31080_spi_byte_write(0x27, g_stCtrl.bReg0_27h[1]);
	rm31080_spi_byte_write(0x28, g_stCtrl.bReg0_28h[1]);
	rm31080_spi_byte_write(0x29, g_stCtrl.bReg0_29h[1]);
	rm31080_spi_byte_write(0x2A, g_stCtrl.bReg0_2Ah[1]);
	rm31080_spi_byte_write(0x2B, g_stCtrl.bReg0_2Bh[1]);
	rm31080_spi_byte_write(0x2C, g_stCtrl.bReg0_2Ch[1]);
	rm31080_spi_byte_write(0x2D, g_stCtrl.bReg0_2Dh[1]);
	rm31080_spi_byte_write(0x2E, g_stCtrl.bReg0_2Eh[1]);
	rm31080_spi_byte_write(0x2F, g_stCtrl.bReg0_2Fh[1]);
	rm31080_spi_byte_write(0x30, g_stCtrl.bReg0_30h[1]);
	rm31080_spi_byte_write(0x31, g_stCtrl.bReg0_31h[1]);
	rm31080_spi_byte_write(0x32, g_stCtrl.bReg0_32h[1]);
	rm31080_spi_byte_write(0x33, g_stCtrl.bReg0_33h[1]);

}

void rm31080_digital_filter_config(void)
{
	int idx;
#if ENABLE_T007B1_SETTING
	if (g_stCtrl.bICVersion != T007A6)
		return;
#endif

	/*u8 bReg0_1Fh = 0x00;*/

	/*InitChargePump();*/
	rm31080_spi_byte_write(0x7F, 0x01);	/* Switch to BANK1*/

	rm31080_spi_byte_write(0x09, g_stCtrl.bReg1_09h[0]);
	rm31080_spi_byte_write(0x43, g_stCtrl.bReg1_43h[0]);
	rm31080_spi_byte_write(0x48, g_stCtrl.bReg1_48h[0]);
	rm31080_spi_byte_write(0x49, g_stCtrl.bReg1_49h[0]);
	rm31080_spi_byte_write(0x4A, g_stCtrl.bReg1_4Ah[0]);
	rm31080_spi_byte_write(0x4B, g_stCtrl.bReg1_4Bh[0]);

	rm31080_spi_byte_write(0x7F, 0x00);	/* Switch to 0x00*/

#if 0
	/*bSenseNumber = 8, REPEAT_5*/
	rm31080_spi_byte_write(0x0E, (0x38 | 0x04));

	rm31080_spi_byte_read(0x1F, &bReg0_1Fh);
	bReg0_1Fh = bReg0_1Fh & 0xF8;
	bReg0_1Fh = bReg0_1Fh | 0x07;	/*REPEAT_5*/

	rm31080_spi_byte_write(0x1F, bReg0_1Fh);
#endif
	rm_set_repeat_times(g_stCtrl.bRepeatTimes[0]);

/* -------- Set Driving / Sensing Control Mode --------*/
	if (!g_stCtrl.bfAnalogFilter) {
		/*ACC | DDSC*/
		rm31080_spi_byte_write(RM31080_REG_10, 0x10 | 0x40);
	}

/* -------- Set PGA/DAC -------- */
	rm31080_spi_byte_write(0x6B, 0x04); /*EN_C0 */

/* -------- Scan Time Setting -------- */
	idx = g_stCtrl.bfAnalogFilter;
	rm31080_spi_byte_write(
		RM31080_REG_42, g_stCtrl.bReg0_42h[idx]);
	rm31080_spi_byte_write(
		RM31080_REG_43, g_stCtrl.bReg0_43h[idx]);

	rm31080_spi_byte_write(0x20, g_stCtrl.bReg0_20h[idx]);
	rm31080_spi_byte_write(0x21, g_stCtrl.bReg0_21h[idx]);
	rm31080_spi_byte_write(0x22, g_stCtrl.bReg0_22h[idx]);
	rm31080_spi_byte_write(0x23, g_stCtrl.bReg0_23h[idx]);
	rm31080_spi_byte_write(0x24, g_stCtrl.bReg0_24h[idx]);
	rm31080_spi_byte_write(0x25, g_stCtrl.bReg0_25h[idx]);
	rm31080_spi_byte_write(0x26, g_stCtrl.bReg0_26h[idx]);
	rm31080_spi_byte_write(0x27, g_stCtrl.bReg0_27h[idx]);
	rm31080_spi_byte_write(0x28, g_stCtrl.bReg0_28h[idx]);
	rm31080_spi_byte_write(0x29, g_stCtrl.bReg0_29h[idx]);
	rm31080_spi_byte_write(0x2A, g_stCtrl.bReg0_2Ah[idx]);
	rm31080_spi_byte_write(0x2B, g_stCtrl.bReg0_2Bh[idx]);
	rm31080_spi_byte_write(0x2C, g_stCtrl.bReg0_2Ch[idx]);
	rm31080_spi_byte_write(0x2D, g_stCtrl.bReg0_2Dh[idx]);
	rm31080_spi_byte_write(0x2E, g_stCtrl.bReg0_2Eh[idx]);
	rm31080_spi_byte_write(0x2F, g_stCtrl.bReg0_2Fh[idx]);
	rm31080_spi_byte_write(0x30, g_stCtrl.bReg0_30h[idx]);
	rm31080_spi_byte_write(0x31, g_stCtrl.bReg0_31h[idx]);
	rm31080_spi_byte_write(0x32, g_stCtrl.bReg0_32h[idx]);
	rm31080_spi_byte_write(0x33, g_stCtrl.bReg0_33h[idx]);

}

void rm31080_filter_config(void)
{
#if ENABLE_FILTER_SWITCH
#if ENABLE_T007B1_SETTING
	if (g_stCtrl.bICVersion != T007A6)
		return;
#endif

	if (g_stCtrl.bfAnalogFilter) {
#if ENABLE_NEW_NOISE_MODE
		if ((g_stCtrl.bfNoiseMode & 0x02) &&
			g_stCtrl.bfNoiseDetector) {
			if (g_stCtrl.bfNoiseModeDetector) {
				rm_set_repeat_times(
					g_stCtrl.bNewNoiseRepeatTimes);
			} else {
				rm_set_repeat_times(
					g_stCtrl.bRepeatTimes[1]);
			}
		} else
			rm_set_repeat_times(g_stCtrl.bRepeatTimes[1]);
#else
		rm31080_analog_filter_config(g_stCtrl.bRepeatTimes[1]);
#endif
	} else {
#if ENABLE_NEW_NOISE_MODE
		if ((g_stCtrl.bfNoiseMode & 0x02)
		&& g_stCtrl.bfNoiseDetector) {
			if (g_stCtrl.bfNoiseModeDetector) {
				rm_set_repeat_times(
				g_stCtrl.bNewNoiseRepeatTimes);
			} else {
				rm31080_digital_filter_config();
			}
		} else
#endif
		rm31080_digital_filter_config();

	}
#endif
}

#endif
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
	s8 bBlockInterval;
	u16 ii;
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
	g_stCtrl.bMTTouchThreshold = pPara[192];
	g_stCtrl.bTime2Idle = pPara[194];
	g_stCtrl.bfPowerMode = pPara[195];
	g_stCtrl.bfIdleMessage = pPara[207];
	g_stCtrl.bDummyRunCycle = pPara[31];

	/* Store dummy channel to skip it, data sequence:*/
	/* Dummy[0](single end) | raw_data |
		dummy[1](single end) dummy[2](single end) | raw_data |
		dummy[3](single end)*/
	g_stCtrl.bDummyChannel[0] = 0;
	if (g_stCtrl.bfMediumFilter) {
		bBlockInterval = (g_stCtrl.bNoiseThresholdMax -
		g_stCtrl.bNoiseThresholdMin) / g_stCtrl.bMFBlockNumber;
		/*printk("Block Interval = %d\n", bBlockInterval); */
		for (ii = 0; ii < g_stCtrl.bMFBlockNumber; ii++) {
			g_bMFBlockMin[ii] = g_stCtrl.bNoiseThresholdMin +
							bBlockInterval * ii;
			g_bMFBlockMax[ii] = g_bMFBlockMin[ii] + bBlockInterval;
		}
	}
	if (g_stCtrl.bADCNumber) {
		if (g_stCtrl.bYChannel[0] > g_stCtrl.bXChannel[0]) {
			if (g_stCtrl.bXChannel[0] < g_stCtrl.bXChannel[2]) {
				if (g_stCtrl.bXChannel[0] <
				g_stCtrl.bXChannel[1])
					Temp = g_stCtrl.bXChannel[1] -
						g_stCtrl.bXChannel[0];
				else
					Temp = g_stCtrl.bXChannel[0] -
						g_stCtrl.bXChannel[1];
				g_stCtrl.bDummyChannel[1] = Temp + 1;
				g_stCtrl.bDummyChannel[2] = Temp + 2;
			} else {
				if (g_stCtrl.bXChannel[2] <
				g_stCtrl.bXChannel[3])
					Temp = g_stCtrl.bXChannel[3] -
						g_stCtrl.bXChannel[2];
				else
					Temp = g_stCtrl.bXChannel[2] -
						g_stCtrl.bXChannel[3];
				g_stCtrl.bDummyChannel[1] = Temp + 1;
				g_stCtrl.bDummyChannel[2] = Temp + 2;
			}

		} else {
			if (g_stCtrl.bXChannel[0] > g_stCtrl.bXChannel[2]) {
				if (g_stCtrl.bXChannel[0] <
				g_stCtrl.bXChannel[1])
					Temp = g_stCtrl.bXChannel[1] -
						g_stCtrl.bXChannel[0];
				else
					Temp = g_stCtrl.bXChannel[0] -
						g_stCtrl.bXChannel[1];
				g_stCtrl.bDummyChannel[1] = Temp + 1;
				g_stCtrl.bDummyChannel[2] = Temp + 2;
			} else {
				if (g_stCtrl.bXChannel[2] <
				g_stCtrl.bXChannel[3])
					Temp = g_stCtrl.bXChannel[3]
						- g_stCtrl.bXChannel[2];
				else
					Temp = g_stCtrl.bXChannel[2]
						- g_stCtrl.bXChannel[3];
				g_stCtrl.bDummyChannel[1] = Temp + 1;
				g_stCtrl.bDummyChannel[2] = Temp + 2;
			}

		}
		g_stCtrl.bDummyChannel[3] = g_stCtrl.bChannelNumberX + 2;
	} else {
		g_stCtrl.bDummyChannel[1] = g_stCtrl.bChannelNumberX + 1;
	}
#if ENABLE_RESOLUTION_SWITCH
	g_stCtrl.u16ResolutionX = ((u16) pPara[PARA_BASIC_LEN + 13]) << 8 |
		((u16)pPara[PARA_BASIC_LEN + 12]);
	g_stCtrl.u16ResolutionY = ((u16) pPara[PARA_BASIC_LEN + 15]) << 8 |
		((u16)pPara[PARA_BASIC_LEN + 14]);

	if ((g_stCtrl.u16ResolutionX == 0) || (g_stCtrl.u16ResolutionY == 0)) {
		g_stCtrl.u16ResolutionX = RM_INPUT_RESOLUTION_X;
		g_stCtrl.u16ResolutionY = RM_INPUT_RESOLUTION_Y;
	}
#endif

#if ENABLE_FILTER_SWITCH
#if ENABLE_T007B1_SETTING
	if (g_stCtrl.bICVersion == T007A6) {
#endif
		g_stCtrl.bReg1_09h[0] = pPara[327];	/* Addr. 0327 */
		g_stCtrl.bReg1_09h[1] = pPara[328];	/* Addr. 0328 */
		g_stCtrl.bReg1_43h[0] = pPara[337];	/* Addr. 0337 */
		g_stCtrl.bReg1_43h[1] = pPara[338];	/* Addr. 0338 */
		g_stCtrl.bReg1_48h[0] = pPara[339];	/* Addr. 0339 */
		g_stCtrl.bReg1_48h[1] = pPara[340];	/* Addr. 0340 */
		g_stCtrl.bReg1_49h[0] = pPara[341];	/* Addr. 0341 */
		g_stCtrl.bReg1_49h[1] = pPara[342];	/* Addr. 0342 */
		g_stCtrl.bReg1_4Ah[0] = pPara[343];	/* Addr. 0343 */
		g_stCtrl.bReg1_4Ah[1] = pPara[344];	/* Addr. 0344 */
		g_stCtrl.bReg1_4Bh[0] = pPara[345];	/* Addr. 0345 */
		g_stCtrl.bReg1_4Bh[1] = pPara[346];	/* Addr. 0346 */

		g_stCtrl.bReg0_40h[0] = pPara[258];
		g_stCtrl.bReg0_40h[1] = pPara[259];
		g_stCtrl.bReg0_41h[0] = pPara[260];
		g_stCtrl.bReg0_41h[1] = pPara[261];
		g_stCtrl.bReg0_42h[0] = pPara[262];	/* Addr. 0262 */
		g_stCtrl.bReg0_42h[1] = pPara[263];	/* Addr. 0263 */
		g_stCtrl.bReg0_43h[0] = pPara[264];	/* Addr. 0264 */
		g_stCtrl.bReg0_43h[1] = pPara[265];	/* Addr. 0265 */

/* time chart*/
		g_stCtrl.bReg0_20h[0] = pPara[213];	/* Addr. 0213 */
		g_stCtrl.bReg0_20h[1] = pPara[214];	/* Addr. 0214 */
		g_stCtrl.bReg0_21h[0] = pPara[215];	/* Addr. 0215 */
		g_stCtrl.bReg0_21h[1] = pPara[216];	/* Addr. 0216 */
		g_stCtrl.bReg0_22h[0] = pPara[217];	/* Addr. 0217 */
		g_stCtrl.bReg0_22h[1] = pPara[218];	/* Addr. 0218 */
		g_stCtrl.bReg0_23h[0] = pPara[219];	/* Addr. 0219 */
		g_stCtrl.bReg0_23h[1] = pPara[220];	/* Addr. 0220 */
		g_stCtrl.bReg0_24h[0] = pPara[221];	/* Addr. 0221 */
		g_stCtrl.bReg0_24h[1] = pPara[222];	/* Addr. 0222 */
		g_stCtrl.bReg0_25h[0] = pPara[223];	/* Addr. 0223 */
		g_stCtrl.bReg0_25h[1] = pPara[224];	/* Addr. 0224 */
		g_stCtrl.bReg0_26h[0] = pPara[225];	/* Addr. 0225 */
		g_stCtrl.bReg0_26h[1] = pPara[226];	/* Addr. 0226 */
		g_stCtrl.bReg0_27h[0] = pPara[227];	/* Addr. 0227 */
		g_stCtrl.bReg0_27h[1] = pPara[228];	/* Addr. 0228 */
		g_stCtrl.bReg0_28h[0] = pPara[229];	/* Addr. 0229 */
		g_stCtrl.bReg0_28h[1] = pPara[230];	/* Addr. 0230 */
		g_stCtrl.bReg0_29h[0] = pPara[231];	/* Addr. 0231 */
		g_stCtrl.bReg0_29h[1] = pPara[232];	/* Addr. 0232 */
		g_stCtrl.bReg0_2Ah[0] = pPara[233];	/* Addr. 0233 */
		g_stCtrl.bReg0_2Ah[1] = pPara[234];	/* Addr. 0234 */
		g_stCtrl.bReg0_2Bh[0] = pPara[235];	/* Addr. 0235 */
		g_stCtrl.bReg0_2Bh[1] = pPara[236];	/* Addr. 0236 */
		g_stCtrl.bReg0_2Ch[0] = pPara[237];	/* Addr. 0237 */
		g_stCtrl.bReg0_2Ch[1] = pPara[238];	/* Addr. 0238 */
		g_stCtrl.bReg0_2Dh[0] = pPara[239];	/* Addr. 0239 */
		g_stCtrl.bReg0_2Dh[1] = pPara[240];	/* Addr. 0240 */
		g_stCtrl.bReg0_2Eh[0] = pPara[241];	/* Addr. 0241 */
		g_stCtrl.bReg0_2Eh[1] = pPara[242];	/* Addr. 0242 */
		g_stCtrl.bReg0_2Fh[0] = pPara[243];	/* Addr. 0243 */
		g_stCtrl.bReg0_2Fh[1] = pPara[244];	/* Addr. 0244 */
		g_stCtrl.bReg0_30h[0] = pPara[245];	/* Addr. 0245 */
		g_stCtrl.bReg0_30h[1] = pPara[246];	/* Addr. 0246 */
		g_stCtrl.bReg0_31h[0] = pPara[247];	/* Addr. 0247 */
		g_stCtrl.bReg0_31h[1] = pPara[248];	/* Addr. 0248 */
		g_stCtrl.bReg0_32h[0] = pPara[249];	/* Addr. 0249 */
		g_stCtrl.bReg0_32h[1] = pPara[250];	/* Addr. 0250 */
		g_stCtrl.bReg0_33h[0] = pPara[251];	/* Addr. 0251 */
		g_stCtrl.bReg0_33h[1] = pPara[252];	/* Addr. 0252 */
#if ENABLE_T007B1_SETTING
	}
#endif
#endif
}

/*===========================================================================*/
MODULE_AUTHOR("xxxxxxxxxx <xxxxxxxx@rad-ic.com>");
MODULE_DESCRIPTION("Raydium touchscreen control functions");
MODULE_LICENSE("GPL");
