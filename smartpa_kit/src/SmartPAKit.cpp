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

#include <cutils/log.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>

#include "SmartPAKit.h"

#define DEVICE_NAME "/dev/cs35l36"
#define DEVICE_ADDRESS "1-0040"

namespace cirrus {

CirrusSmartPAKit::CirrusSmartPAKit():
    mDevFd(-1),
    mIsDSPBypass(false)
{
    ALOGD("%s", __func__);
}

int CirrusSmartPAKit::init(void)
{
    ALOGD("%s", __func__);

    mDevFd = open(DEVICE_NAME, O_RDWR| O_NONBLOCK);
    if (mDevFd == -1) {
        ALOGE("%s: Failed to open device %s", __func__, DEVICE_NAME);
        return -1;
    }

    return 0;
}

void CirrusSmartPAKit::deinit(void)
{
    ALOGD("%s", __func__);

    if (mDevFd >= 0)
        close (mDevFd);

    mDevFd = -1;
    mIsDSPBypass = false;
}

int CirrusSmartPAKit::calibrate(void)
{
    ALOGD("%s", __func__);

    return 0;
}

int CirrusSmartPAKit::getR0(unsigned int *r0_array)
{
    ALOGD("%s", __func__);

    return 0;
}

int CirrusSmartPAKit::getRe(unsigned int *re_array)
{
    ALOGD("%s", __func__);

    return 0;
}

int CirrusSmartPAKit::getF0(unsigned int *f0_array)
{
    ALOGD("%s", __func__);

    return 0;
}

void CirrusSmartPAKit::dumpReg(void)
{
    ALOGD("%s", __func__);

    FILE *fp;
    char *filename = (char *) malloc(50);
    char *line = NULL;
    size_t len = 0;
    ssize_t read;

    sprintf(filename, "/sys/kernel/debug/regmap/%s/registers", DEVICE_ADDRESS);
    fp = fopen(filename, "r");
    if (fp == NULL) {
        ALOGE("Failed to open registers");
        delete(filename);
        return;
    }

    ALOGD("Start of registers dump:");
    while ((read = getline(&line, &len, fp)) != -1) {
        ALOGD("%s", line);
    }
    ALOGD("End of registers dump.");

    fclose(fp);
    if (line)
        delete(line);
    delete(filename);
}

int CirrusSmartPAKit::speakerOn(unsigned int scene)
{
    ALOGD("%s: %x", __func__, scene);

    int ret = 0;

    ret = sendIoctlCmd(CS35L36_SPK_POWER_ON, &scene);
    if (ret < 0) {
        ALOGE("%s: CS35L36_SPK_POWER_ON failed (%d)", __func__, ret);
        return ret;
    }

    return ret;
}

int CirrusSmartPAKit::speakerOff(unsigned int scene)
{
    ALOGD("%s: %x", __func__, scene);

    int ret = 0;

    ret = sendIoctlCmd(CS35L36_SPK_POWER_OFF, &scene);
    if (ret < 0) {
        ALOGE("%s: CS35L36_SPK_POWER_OFF failed (%d)", __func__, ret);
        return ret;
    }

    return ret;
}

void CirrusSmartPAKit::setCalibValue(unsigned int re)
{
    ALOGD("%s: %u", __func__, re);

    mReValue = re;

    ALOGD("%s: Set calibration value to %d", __func__, mReValue);
}

int CirrusSmartPAKit::getDefaultCalibState(void)
{
    ALOGD("%s", __func__);

    int state = 0;
    int ret;

    ret = sendIoctlCmd(CS35L36_SPK_GET_CALIB_STATE, &state);
    if (ret < 0) {
        ALOGE("%s: CS35L36_SPK_GET_CALIB_STATE failed (%d)", __func__, ret);
        return ret;
    }

    return state;
}

void CirrusSmartPAKit::setDefaultCalibValue(int value)
{
    ALOGD("%s", __func__);

    int ret;

    ret = sendIoctlCmd(CS35L36_SPK_SET_DEFAULT_CALIB, &value);
    if (ret < 0) {
        ALOGE("%s: CS35L36_SPK_GET_CALIB_STATE failed (%d)", __func__, ret);
        return;
    }

    ALOGD("%s: Set default calibration value to %d success", __func__, value);
}

// optional
int CirrusSmartPAKit::getTemprature(int *temprature_array)
{
    ALOGD("%s", __func__);

    return 0;
}

void CirrusSmartPAKit::startCalib(void)
{
    ALOGD("%s", __func__);
}

void CirrusSmartPAKit::stopCalib(void)
{
    ALOGD("%s", __func__);
}

void CirrusSmartPAKit::bypassDSP(bool bypass)
{
    ALOGD("%s: %d", __func__, (bypass ? 1 : 0));

    int ret;
    int val = bypass ? 1 : 0;

    if (mIsDSPBypass != bypass) {
        ret =  ioctl(mDevFd, CS35L36_SPK_DSP_BYPASS, &val);
        if (ret < 0) {
            ALOGE("%s: CS35L36_SPK_DSP_BYPASS failed", __func__);
            return;
        }

        mIsDSPBypass = bypass;
    } else {
        if (mIsDSPBypass)
            ALOGW("%s: Already enabled bypass", __func__);
        else
            ALOGW("%s: Already disabled bypass", __func__);
    }
}

// Private functions
CirrusSmartPAKit::~CirrusSmartPAKit()
{
    ALOGD("%s", __func__);
}

int CirrusSmartPAKit::sendIoctlCmd(unsigned int cmd, void *arg)
{
    ALOGD("%s: command: %x", __func__, cmd);

    int ret = 0;

    if (mDevFd == -1) {
        ALOGE("%s: Device fd not valid", __func__);
        return -1;
    }

    ret = ioctl(mDevFd, cmd, arg);
    if(ret < 0) {
        ALOGE("%s: ioctl command %x failed: %d", __func__, cmd, ret);
        return -1;
    }

    return 0;
}

} // namespace cirrus
