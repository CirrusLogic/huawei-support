/**
 * @file hpa.h
 *
 * @brief Functions and prototypes exported by the Cirrus Logic Haptics Processing Algorithm (HPA) module
 *
 * @copyright
 * Copyright (c) Cirrus Logic 2020 All Rights Reserved, http://www.cirrus.com/
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef HPA_H
#define HPA_H

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************************************************************
 * INCLUDES
 **********************************************************************************************************************/
#include <stdint.h>

/***********************************************************************************************************************
 * LITERALS & CONSTANTS
 **********************************************************************************************************************/

/**
 * @defgroup HPA_RET_
 * @brief Return values for all public API calls
 *
 * @{
 */
#define HPA_RET_OK                                      (0)
#define HPA_RET_PLAYBACK_DONE                           (1)
#define HPA_RET_FAIL                                    (-1)
#define HPA_RET_F0_OUT_OF_RANGE                         (-2)
#define HPA_RET_RE_DC_OUT_OF_RANGE                      (-3)
#define HPA_RET_NOT_INIT_ERROR                          (-4)
/** @} */

/**
 * @defgroup HPA_MODE_
 * @brief Possible modes for the HPA library
 *
 * @see cs_set_mode
 *
 * @{
 */
#define HPA_MODE_0                                      (0)
#define HPA_MODE_1                                      (1)
#define HPA_MODE_2                                      (2)
#define HPA_MODE_3                                      (3)
/** @} */

/**
 * @defgroup HPA_VOLUME_
 * @brief Min/max for the HPA volume gain
 *
 * @see cs_set_volume
 *
 * @{
 */
#define HPA_VOLUME_MIN                                  (0)
#define HPA_VOLUME_MAX                                  (1000)
/** @} */

/**
 * HPA Sample Rate - currently only 48kHz supported
 *
 */
#define HPA_SAMPLE_RATE_HZ                              (48000)

/**
 * HPA Block Size - currently only block size of 192 samples is supported
 *
 * This block size, at 48kHz, results in a 4ms block size.  This block size is the same for both IV data input blocks
 * as well as PCM output blocks.
 *
 */
#define HPA_BLOCK_SIZE_SAMPLES                          (192)

/**
 * @defgroup HPA_MODE3_ID_
 * @brief Min, max, and max length for each ID group
 *
 * @see cs_set_effect_wave
 *
 * @{
 */
#define HPA_MODE3_ID_MIN                                (0)
#define HPA_MODE3_ID_MAX                                (27)

#define HPA_MODE3_GROUP1_ID_MIN                         (0)
#define HPA_MODE3_GROUP1_ID_MAX                         (9)
#define HPA_MODE3_GROUP1_SAMPLE_RATE_HZ                 (4000)
#define HPA_MODE3_GROUP1_LENGTH_MAX_BYTES               (240)

#define HPA_MODE3_GROUP2_ID_MIN                         (10)
#define HPA_MODE3_GROUP2_ID_MAX                         (19)
#define HPA_MODE3_GROUP2_SAMPLE_RATE_HZ                 (4000)
#define HPA_MODE3_GROUP2_LENGTH_MAX_BYTES               (360)

#define HPA_MODE3_GROUP3_ID_MIN                         (20)
#define HPA_MODE3_GROUP3_ID_MAX                         (24)
#define HPA_MODE3_GROUP3_SAMPLE_RATE_HZ                 (4000)
#define HPA_MODE3_GROUP3_LENGTH_MAX_BYTES               (600)

#define HPA_MODE3_GROUP4_ID_MIN                         (25)
#define HPA_MODE3_GROUP4_ID_MAX                         (26)
#define HPA_MODE3_GROUP4_SAMPLE_RATE_HZ                 (4000)
#define HPA_MODE3_GROUP4_LENGTH_MAX_BYTES               (1440)

#define HPA_MODE3_GROUP5_ID                             (27)
#define HPA_MODE3_GROUP5_SAMPLE_RATE_HZ                 (2000)
#define HPA_MODE3_GROUP5_LENGTH_MAX_BYTES               (9000)
/** @} */

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * ENUMS, STRUCTS, UNIONS, TYPEDEFS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * GLOBAL VARIABLES
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * API FUNCTIONS
 **********************************************************************************************************************/

/**
 * Get the heap size required for the HPA library
 *
 * @return                      Size of heap library requires, in bytes
 *
 */
int32_t cs_get_cfg_size(void);

/*
 * Initialize the HPA library
 *
 * Initializes all library data structures and buffers used for processing.  When cs_algo_init is called for
 * calibration routine (Mode 0), f0/re/q are datasheet nominal values.  When cs_algo_init is called for playback
 * (Mode 1, 2, 3), f0/re/q are calibrated values.  Please note the fixed point formats for each of the LRA parameters.
 *
 * @param [in] cfg              Pointer to the heap
 * @param [in] f0               Nominal or Calibrated LRA F0 in Q13.18 format.  Must be in range [50 Hz, 511 Hz]
 * @param [in] re               Nominal or Calibrated LRA ReDC in Q15.16 format
 * @param [in] q                Nominal or Calibrated LRA Q in Q15.16 format
 *
 * @return
 * - HPA_RET_FAIL               if pointer to heap is NULL
 * - HPA_RET_F0_OUT_OF_RANGE    if f0 is out of range
 * - HPA_RET_RE_DC_OUT_OF_RANGE if re is out of range
 * - HPA_RET_OK                 otherwise
 *
 */
int32_t cs_algo_init(void *cfg, int32_t f0, int32_t re, int32_t q);

/*
 * Set the HPA library mode
 *
 * @param [in] mode             New operating mode
 *
 * @return
 * - HPA_RET_FAIL               if mode is invalid
 * - HPA_RET_OK                 otherwise
 *
 * @see HPA_MODE_
 *
 */
int32_t cs_set_mode(int32_t mode);

/*
 * Set the HPA library volume
 *
 * The library volume is an integer in the range of 0 (mute) and 1000 (full scale).
 *
 * @attention Volume control is only applied in Mode 1, 2, and 3.
 *
 * @param [in] volume           New operating volume
 *
 * @return
 * - HPA_RET_FAIL               if volume is invalid
 * - HPA_RET_OK                 otherwise
 *
 * @see HPA_VOLUME_
 *
 */
int32_t cs_set_volume(int32_t volume);

/*
 * Set the Mode 2 tone duration
 *
 * Set the duration of the Mode 2 tone in milliseconds.  After 'time' milliseconds of length, any subsequent calls
 * to cs_play_effect_i() will result in output of 0's.
 *
 * @param [in] time             New Mode 2 playback duration (ms)
 *
 * @return                      HPA_RET_OK
 *
 */
int32_t cs_set_tone_time(int32_t time);

/*
 * Stop Mode 1 tone playback
 *
 * If Mode 1 playback is currently selected, stop tone playback.  Any subsequent calls to cs_play_effect_i() will
 * result in output of 0's.
 *
 * @return                      HPA_RET_OK
 *
 */
int32_t cs_set_silence(void);

/*
 * HPA library block processing call
 *
 * Mode 0 - This call will start or continue the Calibration routine.  Calibration complete is indicated by
 * HPA_RET_PLAYBACK_DONE.
 *
 * Mode 1 - This call will start playback of the tone.  After cs_set_silence() is called, subsequent calls will output
 * 0s.
 *
 * Mode 2 - This call will start playback of the tone.  Once the tone is complete, it will return HPA_RET_PLAYBACK_DONE.
 *
 * Mode 3 - This call will start playback of the special effect.  Once the effect is complete, it will return
 * HPA_RET_PLAYBACK_DONE.
 *
 * The 'size' is expected to always be 192 samples.
 *
 * The expected format of the data give by 'vi_in' is:
 * - word0:MS 16-bits - Imon sample 0
 * - word0:LS 16-bits - Vmon sample 0
 * - word1:MS 16-bits - Imon sample 1
 * - word1:LS 16-bits - Vmon sample 1
 * ...
 *
 * The format of the 16-bit PCM 'wave_out' data is:
 * - word0 - sample 0
 * - word1 - sample 1
 * ...
 *
 * @param [in] size             size of 'vi_in' and 'wave_out' in samples
 * @param [in] vi_in            buffer with latest IV data read
 * @param [out] wave_out        buffer for processed haptic waveform
 *
 * @return
 * - HPA_RET_FAIL               if:
 *                                  - pointer to any buffer is NULL
 *                                  - if size is 0
 *                                  - if current mode is invalid
 *                                  - Mode 0:  Calibration initialization error
 *                                  - Mode 0:  Calibration processing failure
 * - HPA_RET_OK                 Processing is okay, continue sending blocks
 * - HPA_RET_PLAYBACK_DONE      Processing complete
 *
 */
int32_t cs_play_effect_i(int32_t size, int32_t *vi_in, int16_t *wave_out);

/*
 * Get the Mode 2 Dynamic F0 measurement
 *
 * @param [out] f0              Mode 2 Dynamic F0 in Q13.18 format
 *
 * @return
 * - HPA_RET_FAIL               if pointer is NULL
 * - HPA_RET_OK                 otherwise
 *
 */
int32_t cs_get_f0(int32_t *f0);

/*
 * Get the Mode 0 Calibration parameters
 *
 * @param [out] f0              Mode 0 Calibration F0 in Q13.18 format
 * @param [out] re              Mode 0 Calibration ReDC in Q15.16 format
 * @param [out] q               Mode 0 Calibration Q in Q15.16 format
 *
 * @return
 * - HPA_RET_FAIL               if any pointer is NULL
 * - HPA_RET_OK                 otherwise
 *
 */
int32_t cs_get_calibration(int32_t *f0, int32_t *re, int32_t *q);

/*
 * Get the HPA library version
 *
 * Return is in the form:
 * MSByte - Major Version
 * Byte 2 - Minor Version
 * Byte 1 - Patch Version
 * Byte 0 - 0
 *
 * @return                      version
 *
 */
int32_t cs_get_version(void);

/*
 * Fill entry in Mode 3 Special Effect Wavetable
 *
 * @param [in] data             Waveform data
 * @param [in] data_len         Length of waveform data in bytes
 * @param [in] id               Index in wavetable for assignment
 *
 * @return
 * - HPA_RET_FAIL               if 'data' is NULL, if 'id' is invalid, or if 'data_len' is not multiple of 3 bytes
 * - HPA_RET_OK                 otherwise
 *
 */
int32_t cs_set_effect_wave(const uint8_t *data, int32_t data_len, int32_t id);

/*
 * Set the current Mode 3 Special Effect
 *
 * @param [in] id               Index in wavetable for current playback
 *
 * @return
 * - HPA_RET_FAIL               if 'id' is invalid, or if 'id' entry in wavetable is invalid
 * - HPA_RET_OK                 otherwise
 *
 */
int32_t cs_convert_effect_wave(int32_t id);

/**********************************************************************************************************************/
#ifdef __cplusplus
}
#endif

#endif // HPA_H
