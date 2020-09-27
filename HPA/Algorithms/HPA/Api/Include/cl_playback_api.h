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
* @file    cl_playback_api.h
* @brief   Defines enums and macros used for playback feature in firmware
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
#ifndef CL_PLAYBACK_API_H
#define CL_PLAYBACK_API_H

#include "cl_types.h"
#include <stdint.h>
#include <limits.h>

// Feature defines
#define ENABLE_REDC_ESTIMATION
#define EXPOSE_DF0_TUNING
//#define HARD_CODED_CH


typedef int16_t sampSize_t;
typedef uint32_t sampSizeVI_t;

/*Error code used by the API implementation*/
// User API states error codes shall be negative, success is 0
typedef enum cl_playback_status_t
{
    /* Playback Error codes*/
    /// <summary>
    /// No error
    /// </summary>
    PLAYBACK_NO_ERROR = 0,

    /// <summary>
    /// init failure
    /// </summary>        
    PLAYBACK_DRIVER_INIT_FAILURE = INT_MIN,

    /// <summary>
    /// obj not initialized
    /// </summary>
    PLAYBACK_NOT_INIT_ERROR,

    /// <summary>
    /// Invalid input sample rate
    /// </summary>
    PLAYBACK_INVALID_INPUT,

    /// <summary>
    /// Invalid input sample rate
    /// </summary>
    PLAYBACK_INVALID_SAMPLE_RATE,

    /// <summary>
    /// Invalid input sample bits
    /// </summary>
    PLAYBACK_INVALID_SAMPLE_BITS,

    /* Diagnostic Error codes*/
    /// <summary>
    /// No error
    /// </summary>
    DIAGNOSTIC_NO_ERROR,

    /// <summary>
    /// calibration has been completed
    /// </summary>
    DIAGNOSTIC_COMPLETED,

    /// <summary>
    /// Q out of Range
    /// </summary>
    DIAGNOSTIC_Q_OUT_OF_RANGE,

    /// <summary>
    /// ReDC out of Range
    /// </summary>
    DIAGNOSTIC_RE_DC_OUT_OF_RANGE,

    /// <summary>
    /// F0 out of Range
    /// </summary>
    DIAGNOSTIC_F0_OUT_OF_RANGE,

} cl_playback_status;

/*
* Structure to contains the calibration output parameters
*/
typedef struct cl_calibration_param_t
{
    int q_out;       //!< Estimated Q factor, q_out will be in Q8.24 format
    int f0_out;      //!< Calculated resonant frequency, f0_out will be in Q10.22 format
    int redc_out;    //!< Estimated DC Resistance, will be in Q7.25 format
}cl_calibration_param;

/*
* Structure to contains the Dynamic F0 tuning parameters
*/
typedef struct {
    uint32_t wave_out_threshold;             // wave_out threshold used to determine beginning of playback in case of initial silence
    uint32_t vmon_threshold;                 // vmon samples threshold used to determine end of playback by looking at V feedback data
    uint32_t minimum_viable_imon_p2p;       // minimum viable IMON peak-to-peak value for dynamic f0 detection
} dynamic_f0_tuning_params_t;

/*
* Structure to contains the Dynamic F0 tuning parameters
*/
typedef struct {
    uint32_t status;
    uint32_t max_imon_ringing_amp;          // maximum amplitude of the first IMON ringing
} dynamic_f0_debug_info_t;


/*
* Structure to contains the LRA parameters to inti the playback
*/
typedef struct cl_playback_lra_param_t
{
    int f0;              //!< Centre Frequency, f0 is expected to be in Q10.22 format
    int f_span;          //!< Frequency Span, f_span is expected to be in Q10.22 format
    int redc;            //!< DC Resistance, redc is expected to be in Q7.25 format
    int tone_duration;   //!< Duration of the chirp(tone), integer
    int amplitude;       //!< Amplitude of the chirp, is expected to be in Q1.31 format
    int q;               //!< Q factor, is expected to be in Q8.24 format
    int re_sample_fact;  //!< resampling factor, required for f0 compensation
    int gain_fact;       //!< gain factor, required for redc compensation
}cl_playback_lra_param;


/*
*  Structure containing parameters required for playback and calibration
*/
typedef struct cl_playback_obj_t
{
    cl_calibration_param cal_param_obj;
    cl_playback_lra_param lra_param_obj; //!< structure of LRA parameters
    clFract cc_resample_fact;
    clFract cc_gain_fact;
    bool cc_flag_compensation_enabled;
}cl_playback_obj;



/*************************************************************************************************************************
* @fn driver_init                                                                                     */ /**
*
* @brief    Initialize driver
* @param    int value of the sample rate
* @param    int value of the sample bit depth
*
* @return   cl_playback_status returned error code
*
**************************************************************************************************************************/
cl_playback_status driver_init(int sample_rate, int sample_bits);


/*************************************************************************************************************************
* @fn driver_open                                                                                     */ /**
*
* @brief    open the driver
* @param    int power to set
*
* @return   cl_playback_status returned error code
*
**************************************************************************************************************************/
cl_playback_status driver_open(int power_set);


/*************************************************************************************************************************
* @fn driver_close                                                                                     */ /**
*
* @brief    close the driver
* @param    None
*
* @return   cl_playback_status returned error code
*
**************************************************************************************************************************/
cl_playback_status driver_close(void);


/*************************************************************************************************************************
* @fn algo_init                                                                                     */ /**
*
* @brief    Initialize the algo with the input parameters
* @param    F0, resonant frequency
* @param    Q, Q factor
* @param    ReDC, DC resistance of the LRA
*
* @return   cl_playback_status returned error code
*
**************************************************************************************************************************/
cl_playback_status algo_init(uint32_t F0, uint32_t Q, uint32_t ReDC);


/*************************************************************************************************************************
* @fn algo_deinit                                                                                     */ /**
*
* @brief    de-init the algo and reset the internal object
* @param    None
*
* @return   cl_playback_status returned error code
*
**************************************************************************************************************************/
cl_playback_status algo_deinit(void);

/*************************************************************************************************************************
* @fn cl_get_playback_len                                                                                     */ /**
*
* @brief    calculates the new length of the playback based on the resample factor for click compensation
NOTE: This function is to be called only if compensation is enabled
* @param    int buffer_size, size of the input data
* @param    int estimated_F0, estimated value of resonant frequency of the lra of the same Q factor
* @param    int estimated_ReDC, estimted value of the dc resistance of the lra of the same Q factor
*
* @return   int , new size of the compensating data
*
**************************************************************************************************************************/
cl_playback_status cl_get_playback_len(uint32_t * in_buffer_size, uint32_t out_buffer_size, uint32_t estimated_F0, uint32_t estimated_ReDC);

/*************************************************************************************************************************
* @fn cl_play_effect                                                                                     */ /**
*
* @brief    run the playback. Supported sample rate: 8Khz, 16Khz, 48Khz (waveform stored at Fs and played back at the same Fs).
*           The bit width will be 16 bits and the data is one sample per word and 16 bits data is stored at LSB
* @param    int* pointer to the fist element of the wave to play
* @param    int volume to apply to the output (i.e.: 0xFFFFFFFF means 100% volume, 0x0 means 0% volume)
* @param    int length of the wave to play(size is not changing between different
*               calls. This is to ensure the there is no mismatch of the V/I data size between different playbacks)
* @param    int* pointer to the VI input
* @param    int* pointer to teh output buffer
*
* @return   playback status
*
**************************************************************************************************************************/
cl_playback_status cl_play_effect(sampSize_t *in_wave, uint32_t volume, uint32_t size, sampSizeVI_t *vi_in, sampSize_t *wave_out);

/*************************************************************************************************************************
* @fn cl_get_dynamic_f0                                                                                     */ /**
*
* @brief    get the current dynamic f0 value
*
* @param    int *dynamic_f0, pointer to memory where the dynamic f0 value will be copied
*
* @return   playback status
**************************************************************************************************************************/
cl_playback_status cl_get_dynamic_f0(uint32_t *dynamic_f0);

#ifdef EXPOSE_DF0_TUNING
cl_playback_status cl_set_dynamic_f0_params(dynamic_f0_tuning_params_t *dynamic_f0_tuning_params);
cl_playback_status cl_get_dynamic_f0_params(dynamic_f0_tuning_params_t *dynamic_f0_tuning_params);
cl_playback_status cl_get_dynamic_f0_debug_info(dynamic_f0_debug_info_t *dynamic_f0_debug_info);
#endif

/*************************************************************************************************************************
* @fn cl_get_calibrate_result                                                                                     */ /**
*
* @brief    Populate the calibration parameters
* @param    cl_calibration_param * (pointer to the calibration parameter to populate)
*
* @return   cl_playback_status returned error code
*
**************************************************************************************************************************/
cl_playback_status cl_get_calibrate_result(cl_calibration_param *calibration_param);


/*************************************************************************************************************************
* @fn cl_play_calibrate                                                                                     */ /**
*
* @brief    run the calibration
* @param    int* Pointer to the VI input buffer used to compute the calibration, expected in Q1.15 format
* @param    int size of the wave_out buffer i.e. number of samples processed in each frame (size is not changing between different
*               calls. This is to ensure the there is no mismatch of the V/I data size between different playbacks)
* @param    int* pointer to the output buffer, wave_out is generated in Q1.15 format
*
* @return   cl_playback_status returned error code
*
**************************************************************************************************************************/
cl_playback_status cl_play_calibrate(sampSizeVI_t *vi_in, uint32_t size, sampSize_t *wave_out);


/*************************************************************************************************************************
* @fn cl_get_version                                                                                     */ /**
*
* @brief    returns algo version
* @param    None
*
* @return   int algo version
*
**************************************************************************************************************************/
int cl_get_version(void);


#endif //#ifndef CL_PLAYBACK_API_H
