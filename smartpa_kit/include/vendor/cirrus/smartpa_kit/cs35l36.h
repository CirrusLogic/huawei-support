/*
 * Copyright (C) 2020 Cirrus Logic, Inc. and
 *                    Cirrus Logic International Semiconductor Ltd.
 *                    All rights reserved.
 *
 * This software as well as any related documentation is furnished under
 * license and may only be used or copied in accordance with the terms of the
 * license.  The information in this file is furnished for informational use
 * only, is subject to change without notice, and should not be construed as
 * a commitment by Cirrus Logic.  Cirrus Logic assumes no responsibility or
 * liability for any errors or inaccuracies that may appear in this file
 * or any software that may be provided in association with this file.
 *
 * Except as permitted by such license, no part of this file may be
 * reproduced, stored in a retrieval system, or transmitted in any form or by
 * any means without the express written consent of Cirrus Logic.
 *
 * Warning
 *    This software is specifically written for Cirrus Logic devices.
 *    It may not be used with other devices.
 */

#ifndef __CS35L36_H__
#define __CS35L36_H__

// IOCTL commands
#define CS35L36_MAGIC_NUMBER	0x323536

#define CS35L36_SPK_DAC_VOLUME			_IOWR(CS35L36_MAGIC_NUMBER, 1, void *)
#define CS35L36_SPK_POWER_ON			_IOWR(CS35L36_MAGIC_NUMBER, 2, void *)
#define CS35L36_SPK_POWER_OFF			_IOWR(CS35L36_MAGIC_NUMBER, 3, void *)
#define CS35L36_SPK_DSP_BYPASS			_IOWR(CS35L36_MAGIC_NUMBER, 4, void *)
#define CS35L36_SPK_SWITCH_CONFIGURATION	_IOWR(CS35L36_MAGIC_NUMBER, 5, void *)
#define CS35L36_SPK_SWITCH_CALIBRATION	_IOWR(CS35L36_MAGIC_NUMBER, 6, void *)
#define CS35L36_SPK_GET_R0				_IOWR(CS35L36_MAGIC_NUMBER, 7, void *)
#define CS35L36_SPK_GET_F0				_IOWR(CS35L36_MAGIC_NUMBER, 8, void *)
#define CS35L36_SPK_GET_CAL_STRUCT		_IOWR(CS35L36_MAGIC_NUMBER, 9, void *)
#define CS35L36_SPK_SET_CAL_STRUCT		_IOWR(CS35L36_MAGIC_NUMBER, 10, void *)
#define CS35L36_SPK_SET_AMBIENT			_IOWR(CS35L36_MAGIC_NUMBER, 11, void *)
#define CS35L36_SPK_SET_R0				_IOWR(CS35L36_MAGIC_NUMBER, 12, void *)
#define CS35L36_SPK_SWITCH_FIRMWARE		_IOWR(CS35L36_MAGIC_NUMBER, 13, void *)
#define CS35L36_SPK_GET_R0_REALTIME		_IOWR(CS35L36_MAGIC_NUMBER, 14, void *)
#define CS35L36_SPK_SET_DEFAULT_CALIB	_IOWR(CS35L36_MAGIC_NUMBER, 15, void *)
#define CS35L36_SPK_GET_CALIB_STATE		_IOWR(CS35L36_MAGIC_NUMBER, 16, void *)

#ifdef CONFIG_COMPAT
#define CS35L36_SPK_DAC_VOLUME_COMPAT	\
								_IOWR(CS35L36_MAGIC_NUMBER, 1, compat_uptr_t)
#define CS35L36_SPK_POWER_ON_COMPAT		\
								_IOWR(CS35L36_MAGIC_NUMBER, 2, compat_uptr_t)
#define CS35L36_SPK_POWER_OFF_COMPAT	\
								_IOWR(CS35L36_MAGIC_NUMBER, 3, compat_uptr_t)
#define CS35L36_SPK_DSP_BYPASS_COMPAT	\
								_IOWR(CS35L36_MAGIC_NUMBER, 4, compat_uptr_t)
#define CS35L36_SPK_SWITCH_CONFIGURATION_COMPAT	\
								_IOWR(CS35L36_MAGIC_NUMBER, 5, compat_uptr_t)
#define CS35L36_SPK_SWITCH_CALIBRATION_COMPAT	\
								_IOWR(CS35L36_MAGIC_NUMBER, 6, compat_uptr_t)
#define CS35L36_SPK_GET_R0_COMPAT	\
								_IOWR(CS35L36_MAGIC_NUMBER, 7, compat_uptr_t)
#define CS35L36_SPK_GET_F0_COMPAT	\
								_IOWR(CS35L36_MAGIC_NUMBER, 8, compat_uptr_t)
#define CS35L36_SPK_GET_CAL_STRUCT_COMPAT	\
								_IOWR(CS35L36_MAGIC_NUMBER, 9, compat_uptr_t)
#define CS35L36_SPK_SET_CAL_STRUCT_COMPAT	\
								_IOWR(CS35L36_MAGIC_NUMBER, 10, compat_uptr_t)
#define CS35L36_SPK_SET_AMBIENT_COMPAT	\
								_IOWR(CS35L36_MAGIC_NUMBER, 11, compat_uptr_t)
#define CS35L36_SPK_SET_R0_COMPAT	\
								_IOWR(CS35L36_MAGIC_NUMBER, 12, compat_uptr_t)
#define CS35L36_SPK_SWITCH_FIRMWARE_COMPAT	\
								_IOWR(CS35L36_MAGIC_NUMBER, 13, compat_uptr_t)
#define CS35L36_SPK_GET_R0_REALTIME_COMPAT	\
								_IOWR(CS35L36_MAGIC_NUMBER, 14, compat_uptr_t)
#define CS35L36_SPK_SET_DEFAULT_CALIB_COMPAT	\
								_IOWR(CS35L36_MAGIC_NUMBER, 15, compat_uptr_t)
#define CS35L36_SPK_GET_CALIB_STATE_COMPAT	\
								_IOWR(CS35L36_MAGIC_NUMBER, 16, compat_uptr_t)
#endif

#define CS_DEVICE "/dev/cs35l36"
#define MAX_BOX_NUM  6

// API Commands
enum algo_scene {
	MUSIC = 0,
	RINGTONE,
	RINGTONE_HS_SPK,
	VOICE,
	VOIP,
	LOW_POWER,
	MMI_PRI_L,
	ALGO_BYPSS,
	LOW_TEMP,
	CALIB,
	FM,
	ALGO_SCENE_MAX,
};

enum smartpa_scene {
	POWER_ON_L = 0,
	POWER_ON_R,
	POWER_OFF_L,
	POWER_OFF_R,
	SMARTPA_SCENE_MAX,
};

enum smartpa_cmd {
	POWER_ON = 0,
	POWER_OFF,
	CALIBRATE,
	GET_R0,
	GET_RE,
	GET_F0,
	DUMP_REG,
	PARSER_COEF_RANGE,
	SWITCH_USECASE,
	GET_TEMPRATURE,
	CALIB_START,
	CALIB_STOP,
	SET_CALIB_VALUE,
	DSP_BYPASS,
	SMARTPA_CMD_MAX,
};

#endif /* __CS35L36_H */
