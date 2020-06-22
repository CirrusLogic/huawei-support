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

#ifndef CIRRUS_SMARTPA_H
#define CIRRUS_SMARTPA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <vendor/cirrus/smartpa_kit/cs35l36.h>

namespace cirrus {

class CirrusSmartPAKit {
public:
    CirrusSmartPAKit();

    int init(void);
    void deinit(void);

    // Mandatory
    int calibrate(void);
    int getR0(unsigned int *r0_array);
    int getRe(unsigned int *re_array);
    int getF0(unsigned int *f0_array);

    void dumpReg(void);

    int speakerOn(unsigned int scene);
    int speakerOff(unsigned int scene);

    void setCalibValue(unsigned int re);
    int getDefaultCalibState(void);
    void setDefaultCalibValue(int value);

    // Optional
    int getTemprature(int *temprature_array);
    void startCalib(void);
    void stopCalib(void);
    void bypassDSP(bool bypass);

private:
    ~CirrusSmartPAKit();

    int mDevFd;
    bool mIsDSPBypass;
    unsigned int mReValue;

    int sendIoctlCmd(unsigned int cmd, void *arg);
};

} //namespace cirrus

#ifdef __cplusplus
}
#endif

#endif //CIRRUS_SMARTPA_H
