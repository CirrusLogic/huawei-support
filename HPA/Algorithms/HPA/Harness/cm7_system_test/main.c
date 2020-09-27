/**
 * @file main.c
 *
 * @brief The main function for CS35L41 System Test Harness
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
/***********************************************************************************************************************
 * INCLUDES
 **********************************************************************************************************************/
#include "system_test_bsp.h"
#include <stddef.h>
#include <stdlib.h>

/***********************************************************************************************************************
 * LOCAL LITERAL SUBSTITUTIONS
 **********************************************************************************************************************/
#define INCLUDE_CALIBRATION

#define APP_HAPTIC_STATE_CALIBRATING     (0)
#define APP_HAPTIC_STATE_PDN             (1)
#define APP_HAPTIC_STATE_PUP             (2)
/***********************************************************************************************************************
 * LOCAL VARIABLES
 **********************************************************************************************************************/
static uint8_t app_haptic_state = APP_HAPTIC_STATE_PDN;
static bool app_bsp_cb_called = false;

/***********************************************************************************************************************
 * GLOBAL VARIABLES
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * LOCAL FUNCTIONS
 **********************************************************************************************************************/
void app_bsp_callback(uint32_t status, void *arg)
{
    app_bsp_cb_called = true;

    if (status != BSP_STATUS_OK)
    {
        exit(1);
    }

    bsp_haptic_buzz_stop();

    return;
}

/***********************************************************************************************************************
 * API FUNCTIONS
 **********************************************************************************************************************/

/**
 * @brief The Main Entry Point from __main
 *  By this time, the RAM RW-Data section has been initialized by the ARM-provided __main function.
 *
 * @return N/A (does not return)
 */
int main(void)
{
    int ret_val = 0;

    bsp_initialize((bsp_app_callback_t) app_bsp_callback, NULL);
    bsp_haptic_power_up();
    bsp_haptic_mute(false);

#ifdef INCLUDE_CALIBRATION
    if (bsp_pb_por_is_pressed)
    {
        // Start calibration process
        bsp_haptic_calibrate();
        app_haptic_state = APP_HAPTIC_STATE_CALIBRATING;
    }
    else
    {
        bsp_set_gpio(BSP_GPIO_ID_LD3, BSP_GPIO_LOW);
        app_haptic_state = APP_HAPTIC_STATE_PDN;
    }
#else
    bsp_pb_por_is_pressed = false;
    bsp_cal_param_save();
    app_haptic_state = APP_HAPTIC_STATE_PDN;
#endif

    bsp_enable_i2c_listener();

    while (1)
    {
        bsp_haptic_process();
        bsp_pb_debouncer();

        switch (app_haptic_state)
        {
            case APP_HAPTIC_STATE_CALIBRATING:
                if (app_bsp_cb_called)
                {
                    // Save calibration parameters to virtual EEPROM
                    bsp_cal_param_save();
                    bsp_set_gpio(BSP_GPIO_ID_LD3, BSP_GPIO_LOW);
                    app_haptic_state = APP_HAPTIC_STATE_PDN;
                }
                break;

            case APP_HAPTIC_STATE_PDN:
                if ((bsp_pb_pressed_flag) || (bsp_i2c_triggered_buzz))
                {
                    bsp_haptic_buzz_record(BSP_HAPTIC_BUZZ_CURRENT);
                    app_haptic_state = APP_HAPTIC_STATE_PUP;
                }
                else if (bsp_i2c_triggered_calibrate)
                {
                    // Start calibration process
                    bsp_haptic_calibrate();
                    app_haptic_state = APP_HAPTIC_STATE_CALIBRATING;
                }
                break;

            case APP_HAPTIC_STATE_PUP:
                if (app_bsp_cb_called)
                {
                    bsp_haptic_udpate_f0();
                    bsp_set_gpio(BSP_GPIO_ID_LD3, BSP_GPIO_LOW);
                    app_haptic_state = APP_HAPTIC_STATE_PDN;
                }
                break;

            default:
                break;
        }

        app_bsp_cb_called = false;
        bsp_pb_pressed_flag = false;
        bsp_i2c_triggered_buzz = false;
        bsp_i2c_triggered_calibrate = false;

        bsp_sleep();
    }

    exit(1);

    return ret_val;
}
