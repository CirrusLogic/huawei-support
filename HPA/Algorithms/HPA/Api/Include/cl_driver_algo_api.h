/*******************************************************************************
*
* Copyright (c) 2019 Cirrus Logic, Inc and
* Cirrus Logic International Semiconductor Ltd.  All rights reserved.
*
* This software as well as any related documentation is furnished under
* license and may only be used or copied in accordance with the terms of the
* license.  The information in this file is furnished for informational use
* only, is subject to change without notice, and should not be construed as
* a commitment by Cirrus Logic.  Cirrus Logic assumes no responsibility or
* liability for any errors or inaccuracies that may appear in this document
* or any software that may be provided in association with this document.
*
* Except as permitted by such license, no part of this document may be
* reproduced, stored in a retrieval system, or transmitted in any form or by
* any means without the express written consent of Cirrus Logic.
*                                                                         *//**
* @file    cl_driver_algo_api.h
* @brief   Defines enums and macros used for both driver and algo playback
*
* @cond EXCLUDE_DOXYGEN
* @version $Rev:  $
*
* @warning
*     This software is specifically written for Cirrus Logic devices.
*     It may not be used with other devices.
*
* @endcond
******************************************************************************/
#ifndef CL_DRIVE_ALGO_API_H
#define CL_DRIVE_ALGO_API_H

//include header used by both driver and algo
#include <string.h> //for memset
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>


/*Error code used by the API implementation*/
typedef enum cl_driver_algo_status_t
{
    /* Driver algo communication error codes*/
    /// <summary>
    /// No error
    /// </summary>
    DRIVER_ALGO_NO_ERROR = 0,
    
    /// <summary>
    /// Error on sample rate
    /// </summary>
    DRIVER_ALGO_SAMPLE_RATE_ERROR,
    
    /// <summary>
    /// Error on bit depth
    /// </summary>
    DRIVER_ALGO_BIT_DEPTH_ERROR,    

}cl_driver_algo_status;


/*************************************************************************************************************************
* @fn set_attribute                                                                                     */ /**
*
* @brief    propagate the init parameter from driver to algo
* @param    int sample rate
* @param    int sample bit depth
*
* @return   cl_playback_status returned error code
*
**************************************************************************************************************************/
cl_driver_algo_status set_attribute(uint32_t sample_rate, uint32_t sample_bits);

#endif //#ifndef CL_DRIVE_ALGO_API_H
