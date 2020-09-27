/**
 * @file system_test_bsp.h
 *
 * @brief Functions and prototypes exported by the BSP module for the system_test_hw_0 platform.
 *
 * @copyright
 * Copyright (c) Cirrus Logic 2019 All Rights Reserved, http://www.cirrus.com/
 *
 * This code and information are provided 'as-is' without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a particular
 * purpose.
 *
 */

#ifndef SYSTEM_TEST_BSP_H
#define SYSTEM_TEST_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************************************************************************
 * INCLUDES
 **********************************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "stm32f7xx_hal.h"
#include "eeprom.h"
#include "haptic_tables.h"
#include "cl_playback_api.h"

/***********************************************************************************************************************
 * LITERALS & CONSTANTS
 **********************************************************************************************************************/

#define BSP_STATUS_OK                   (0)
#define BSP_STATUS_FAIL                 (1)

#define BSP_GPIO_LOW                    (0)
#define BSP_GPIO_HIGH                   (1)

#define BSP_GPIO_ID_LD1                 (0)
#define BSP_GPIO_ID_LD2                 (1)
#define BSP_GPIO_ID_LD3                 (2)
#define BSP_GPIO_ID_HAPTIC_RESET        (3)
#define BSP_GPIO_ID_HAPTIC_INT          (4)
#define BSP_GPIO_ID_USER_PB             (5)
#define BSP_GPIO_ID_DEBUG_PIN           (6)
#define BSP_GPIO_ID_HAPTIC_TRIGGER      (7)

#define BSP_HAPTIC_BUZZ_CURRENT         (0)
#define BSP_HAPTIC_BUZZ_100HZ           (1)
#define BSP_HAPTIC_BUZZ_PATTERN         (2)

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/**
 * Macro to extract byte from multi-byte word
 *
 * @param [in] A                multi-byte word
 * @param [in] B                zero-indexed byte position
 *
 * @return                      byte at position B in word A
 */
#define GET_BYTE_FROM_WORD(A, B)   ((A >> (B * 8)) & 0xFF)

/***********************************************************************************************************************
 * ENUMS, STRUCTS, UNIONS, TYPEDEFS
 **********************************************************************************************************************/

/**
 * Callback type for BSP-to-Driver callbacks
 *
 * @param [in] status           Result of BSP call
 * @param [in] arg              Argument registered when BSP call was issued
 *
 * @return none
 *
 * @see BSP_STATUS_
 *
 */
typedef void (*bsp_callback_t)(uint32_t status, void* arg);

typedef void (*bsp_app_callback_t)(uint32_t status, void *arg);

typedef struct
{
    uint16_t default_block_size;
    uint16_t output_block_size;
    int16_t *current_block;
    uint16_t current_block_size;
    int16_t *first_block;
    int16_t *last_block;
    uint16_t last_block_word_count;
    uint8_t repetition_total;
    uint8_t current_repetition;
    bool is_last_block;
} waveform_blockizer_t;

typedef struct
{
    const haptic_tables_t *table;
    bool has_been_played;
    uint32_t f0_measured;
    uint32_t f0_used;
} haptic_waveform_log_t;

typedef union
{
    uint32_t words[6];
    struct
    {
        uint32_t algo_init;
        uint32_t cl_get_playback_len;
        uint32_t cl_play_effect;
        uint32_t cl_get_dynamic_f0;
        uint32_t cl_get_calibrate_result;
        uint32_t cl_play_calibrate;
    };
} algo_api_cycle_count_t;

typedef union
{
    uint32_t words[6];
    struct
    {
        cl_playback_status algo_init;
        cl_playback_status cl_get_playback_len;
        cl_playback_status cl_play_effect;
        cl_playback_status cl_get_dynamic_f0;
        cl_playback_status cl_get_calibrate_result;
        cl_playback_status cl_play_calibrate;
    };
} algo_api_ret_t;

typedef struct
{
    uint16_t gpio_pin;
    uint8_t bsp_gpio_id;
    uint32_t debounce_counter;
    bool triggered;
    uint8_t active_level;
} bsp_pb_debounce_data_t;

/***********************************************************************************************************************
 * GLOBAL VARIABLES
 **********************************************************************************************************************/
extern TIM_HandleTypeDef tim_drv_handle;
extern I2C_HandleTypeDef i2c_drv_handle;
extern SAI_HandleTypeDef sai_tx_drv_handle;
extern SAI_HandleTypeDef sai_rx_drv_handle;
extern bool bsp_pb_pressed_flag;
extern bool bsp_pb_por_is_pressed;
extern volatile bool bsp_i2c_triggered_buzz;
extern volatile bool bsp_i2c_triggered_calibrate;

/***********************************************************************************************************************
 * API FUNCTIONS
 **********************************************************************************************************************/
uint32_t bsp_initialize(bsp_app_callback_t cb, void *cb_arg);
uint32_t bsp_haptic_initialize(void);
uint32_t bsp_haptic_calibrate(void);
uint32_t bsp_haptic_power_up(void);
uint32_t bsp_haptic_power_down(void);
uint32_t bsp_haptic_mute(bool is_mute);
uint32_t bsp_haptic_process(void);
uint32_t bsp_haptic_buzz_record(uint8_t content);
uint32_t bsp_haptic_buzz_stop(void);
bool bsp_was_pb_pressed(void);
void bsp_sleep(void);
uint32_t bsp_pb_debouncer(void);
uint32_t bsp_cal_param_save(void);
uint32_t bsp_haptic_udpate_f0(void);
uint32_t bsp_set_gpio(uint32_t gpio_id, uint8_t gpio_state);
uint32_t bsp_enable_i2c_listener(void);
/**********************************************************************************************************************/
#ifdef __cplusplus
}
#endif

#endif // SYSTEM_TEST_BSP_H
