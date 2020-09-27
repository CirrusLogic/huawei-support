/**
 * @file system_test_bsp.c
 *
 * @brief Implementation of the BSP for the system_test_hw_0 platform.
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
#include <stddef.h>
#include "system_test_bsp.h"
#include "cl_math.h"

/***********************************************************************************************************************
 * LOCAL LITERAL SUBSTITUTIONS
 **********************************************************************************************************************/
#define USE_VALIDATION_SETUP

#define BSP_DEV_ID_NULL                 (0)
#define BSP_HAPTIC_DEV_ID               (1)

#define BSP_CS35L36_I2C_ADDRESS         (0xA0)
#define BSP_TCA9539_I2C_ADDRESS         (0xE8)
#define BSP_LN2_FPGA_I2C_ADDRESS        (0x44)

/*
 * Setting for I2C_TIMINGR
 *
 * I2C_TIMINGR bitfields are:
 * - b31:28 - PRESC[3:0]
 * - b27:24 - reserved
 * - b23:20 - SCLDEL[3:0]
 * - b19:16 - SDALDEL[3:0]
 * - b15:8 - SCLH[7:0]
 * - b7:0 - SCLL[7:0]
 *
 * RCC_DCKCFGR2:I2C1SEL = 0b00, so I2CCLK = PCLK1
 * T_I2CCLK = 1/I2CCLK = 18.518 ns
 * PCLK1 (APB1 source clock) = HSI / 4 = 216 MHz / 4 = 54 MHz
 *
 * T_PRESC = (PRESC[3:0] + 1) x T_I2CCLK = 5 * 18.518 ns = 92.592 ns
 *
 */
#define BSP_I2C_TIMING                  0x40912732

#define BSP_MCU_CLOCK_CFG_HSI           (0)
#define BSP_MCU_CLOCK_CFG_HSE           (1)
#define BSP_MCU_CLOCK_CFG               (BSP_MCU_CLOCK_CFG_HSI)

#define BSP_I2C_TRANSACTION_TYPE_WRITE                  (0)
#define BSP_I2C_TRANSACTION_TYPE_READ_REPEATED_START    (1)
#define BSP_I2C_TRANSACTION_TYPE_DB_WRITE               (2)
#define BSP_I2C_TRANSACTION_TYPE_INVALID                (3)

#define SAI_TX_HW                       SAI1_Block_A
#define SAI_RX_HW                       SAI1_Block_B
#define SAI_CLK_ENABLE()                __HAL_RCC_SAI1_CLK_ENABLE()
#define SAI_CLK_DISABLE()               __HAL_RCC_SAI1_CLK_DISABLE()
#define SAI_GPIO_PORT_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE()
#define SAI_GPIO_PORT                   GPIOE
#define SAI_GPIO_AF                     GPIO_AF6_SAI1
#define SAI_LRCLK_PIN                   GPIO_PIN_4
#define SAI_SCLK_PIN                    GPIO_PIN_5
#define SAI_SDOUT_PIN                   GPIO_PIN_6
#define SAI_SDIN_PIN                    GPIO_PIN_3

/* SAI DMA Stream definitions */
#define SAI_TX_DMAx_CLK_ENABLE()        __HAL_RCC_DMA2_CLK_ENABLE()
#define SAI_TX_DMAx_CLK_DISABLE()       __HAL_RCC_DMA2_CLK_DISABLE()
#define SAI_TX_DMAx_STREAM              DMA2_Stream1
#define SAI_TX_DMAx_CHANNEL             DMA_CHANNEL_0
#define SAI_TX_DMAx_IRQ                 DMA2_Stream1_IRQn
#define SAI_TX_IRQHandler               DMA2_Stream1_IRQHandler

#define SAI_RX_DMAx_CLK_ENABLE()        __HAL_RCC_DMA2_CLK_ENABLE()
#define SAI_RX_DMAx_CLK_DISABLE()       __HAL_RCC_DMA2_CLK_DISABLE()
#define SAI_RX_DMAx_STREAM              DMA2_Stream4
#define SAI_RX_DMAx_CHANNEL             DMA_CHANNEL_1
#define SAI_RX_DMAx_IRQ                 DMA2_Stream4_IRQn
#define SAI_RX_IRQHandler               DMA2_Stream4_IRQHandler

/* Select the interrupt preemption priority and subpriority for the DMA interrupt */
#define SAI_TX_IRQ_PREPRIO                      0x0E   /* Select the preemption priority level(0 is the highest) */
#define SAI_RX_IRQ_PREPRIO                      0x0F   /* Select the preemption priority level(0 is the highest) */

/* BSP Audio Format definitions */
#define BSP_I2S_FS_HZ                           (SAI_AUDIO_FREQUENCY_48K)
#define BSP_TX_SAI_DATASIZE                     (SAI_DATASIZE_16)
#define BSP_RX_SAI_DATASIZE                     (SAI_DATASIZE_16)
#ifndef USE_VALIDATION_SETUP
#define BSP_I2S_SUBFRAME_SIZE_BITS              (16)
#define BSP_I2S_CHANNEL_NBR                     (2)
#else
#define BSP_I2S_SUBFRAME_SIZE_BITS              (16)
#define BSP_I2S_CHANNEL_NBR                     (16)
#endif
#define BUFFER_SIZE_MONO_4MS_SAMPLES            ((BSP_I2S_FS_HZ / 1000) * 4)
#define PLAYBACK_PATTERN_BUFFER_SIZE_HALFWORDS  (BUFFER_SIZE_MONO_4MS_SAMPLES * 4)
#define BUFFER_SIZE_PADDING_SAMPLES             (32)
#define SAI_TX_BUFFER_SIZE_HALFWORDS            BUFFER_SIZE_MONO_4MS_SAMPLES
#define SAI_RX_BUFFER_SIZE_WORDS                BUFFER_SIZE_MONO_4MS_SAMPLES
#define TX_PLAYBACK_CLICK_COMP_BUFF_SIZE        SAI_TX_BUFFER_SIZE_HALFWORDS + BUFFER_SIZE_PADDING_SAMPLES
#define PLAYBACK_PATTERN_BUFFER_DEFAULT_VALUE   (0xABCD)
#define SAI_TX_BUFFER_0_DEFAULT_VALUE           (0x1234)
#define SAI_TX_BUFFER_1_DEFAULT_VALUE           (0xABCD)

#define BSP_LED_CLK_ENABLE                      __HAL_RCC_GPIOB_CLK_ENABLE
#define BSP_LED_CLK_DISABLE                     __HAL_RCC_GPIOB_CLK_DISABLE
#define BSP_LED_LD1_PIN                         GPIO_PIN_0
#define BSP_LED_LD2_PIN                         GPIO_PIN_7
#define BSP_LED_LD3_PIN                         GPIO_PIN_14
#define BSP_LED_GPIO_PORT                       GPIOB

#define BSP_PB_CLK_ENABLE                       __HAL_RCC_GPIOC_CLK_ENABLE
#define BSP_PB_CLK_DISABLE                      __HAL_RCC_GPIOC_CLK_DISABLE
#define BSP_PB_PIN                              GPIO_PIN_13
#define BSP_PB_GPIO_PORT                        GPIOC

#define BSP_HAPTIC_RESET_CLK_ENABLE             __HAL_RCC_GPIOF_CLK_ENABLE
#define BSP_HAPTIC_RESET_CLK_DISABLE            __HAL_RCC_GPIOF_CLK_DISABLE
#define BSP_HAPTIC_RESET_PIN                    GPIO_PIN_2
#define BSP_HAPTIC_RESET_GPIO_PORT              GPIOF

#define BSP_HAPTIC_INT_CLK_ENABLE               __HAL_RCC_GPIOF_CLK_ENABLE
#define BSP_HAPTIC_INT_CLK_DISABLE              __HAL_RCC_GPIOF_CLK_DISABLE
#define BSP_HAPTIC_INT_PIN                      GPIO_PIN_8
#define BSP_HAPTIC_INT_GPIO_PORT                GPIOF

#define BSP_HAPTIC_TRIGGER_CLK_ENABLE             __HAL_RCC_GPIOF_CLK_ENABLE
#define BSP_HAPTIC_TRIGGER_CLK_DISABLE            __HAL_RCC_GPIOF_CLK_DISABLE
#define BSP_HAPTIC_TRIGGER_PIN                    GPIO_PIN_4
#define BSP_HAPTIC_TRIGGER_GPIO_PORT              GPIOF

#define BSP_DEBUG_PIN_CLK_ENABLE                __HAL_RCC_GPIOA_CLK_ENABLE
#define BSP_DEBUG_PIN_CLK_DISABLE               __HAL_RCC_GPIOA_CLK_DISABLE
#define BSP_DEBUG_PIN_PIN                       GPIO_PIN_7
#define BSP_DEBUG_PIN_GPIO_PORT                 GPIOA

#define EXTI15_10_IRQ_PREPRIO                   0x07

#define CS35L36_AMP_CTRL_RESET                  (0x8000)
#define CS35L36_DEFAULT_VOLUME                  (0x760) // -20dB
#define CS35L36_MUTE_VOLUME                     (0x400)
#define CS35L36_AMP_CTRL_DEFAULT                (CS35L36_AMP_CTRL_RESET | (CS35L36_DEFAULT_VOLUME << 3))
#define CS35L36_AMP_CTRL_MUTE                   (CS35L36_AMP_CTRL_RESET | (CS35L36_MUTE_VOLUME << 3))

#define BSP_PB_DEBOUNCE_TOTAL_TICKS             (50)

#define BSP_HOST_I2C_SLAVE_ADDRESS_8BIT         (0xDC)

#define BSP_DEBUG_PIN_TOGGLE                    GPIOA->BSRR = (GPIOA->ODR & GPIO_PIN_7) ? (uint32_t) (GPIO_PIN_7 << 16) : GPIO_PIN_7;
//#define BSP_DEBUG_PIN_TOGGLE

#define bsp_init_cycle_counter() \
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; \
    ITM->LAR = 0xC5ACCE55; \
    DWT->LAR = 0xC5ACCE55

#define bsp_reset_cycle_counter() \
    DWT->CYCCNT = 0

#define bsp_enable_cycle_counter() \
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk

#define BSP_DisableCycleCounter() \
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk

#define bsp_get_cycle_counter() \
    DWT->CYCCNT

#define LRA_NIDEC_SPRINTER_R                    (0)
#define LRA_AAC_ELA0815D                        (1)
#define LRA_PYLE_HOME_SPEAKER                   (2)
#define LRA_USED                                (LRA_AAC_ELA0815D)

#if (LRA_USED == LRA_NIDEC_SPRINTER_R)
#define LRA_SPEC_F0                             QFORMAT(150, 22)
#define LRA_SPEC_Q                              (0)
#define LRA_SPEC_REDC                           QFORMAT(14, 22)
#elif (LRA_USED == LRA_AAC_ELA0815D)
#define LRA_SPEC_F0                             QFORMAT(170, 22)
#define LRA_SPEC_Q                              (0)
#define LRA_SPEC_REDC                           QFORMAT(9, 22)
#elif (LRA_USED == LRA_PYLE_HOME_SPEAKER)
#define LRA_SPEC_F0                             QFORMAT(150, 22)
#define LRA_SPEC_Q                              (0)
#define LRA_SPEC_REDC                           QFORMAT(8, 22)
#else
#error "Unknown LRA_USED value!"
#endif

#define DEFAULT_PLAYBACK_VOLUME                 (0xFFFFFFFF)

#define MCPS_WRAP(A, ...) if (1) { \
        uint32_t temp_cycle_count_b; \
        uint32_t temp_cycle_count_a; \
        uint32_t diff_cycle_count; \
        bsp_reset_cycle_counter(); \
        bsp_enable_cycle_counter(); \
        temp_cycle_count_a = bsp_get_cycle_counter(); \
        algo_api_ret.A = A(__VA_ARGS__); \
        temp_cycle_count_b = bsp_get_cycle_counter(); \
        BSP_DisableCycleCounter(); \
        if (temp_cycle_count_a > temp_cycle_count_b) \
        { \
            diff_cycle_count = temp_cycle_count_a - temp_cycle_count_b; \
        } \
        else \
        { \
            diff_cycle_count = temp_cycle_count_b - temp_cycle_count_a; \
        } \
        if (diff_cycle_count > algo_api_max_cycle_count.cl_get_playback_len) \
        { \
            algo_api_max_cycle_count.A = diff_cycle_count; \
        } \
}

#define BSP_I2C_TRIGGER_REGISTER_BUZZ_MASK      (0x1)
#define BSP_I2C_TRIGGER_REGISTER_CALIBRATE_MASK (0x2)

/***********************************************************************************************************************
 * LOCAL VARIABLES
 **********************************************************************************************************************/

static bsp_callback_t bsp_timer_cb;
static void *bsp_timer_cb_arg;
static bool bsp_timer_has_started;

static uint8_t i2c_read_buffer[32];
static uint8_t i2c_read_buffer_index = 0;
static uint8_t i2c_write_buffer[32];
static uint8_t i2c_write_buffer_index = 0;
static bsp_callback_t bsp_i2c_done_cb;
static void *bsp_i2c_done_cb_arg;
static uint8_t bsp_i2c_current_transaction_type;
static uint8_t *bsp_i2c_read_buffer_ptr;
static uint32_t bsp_i2c_read_length;
static uint8_t bsp_i2c_read_address;
static uint32_t bsp_i2c_write_length;
static uint8_t *bsp_i2c_write_buffer_ptr;

static int16_t playback_pattern_buffer[PLAYBACK_PATTERN_BUFFER_SIZE_HALFWORDS];

static int16_t tx_buffer_last[TX_PLAYBACK_CLICK_COMP_BUFF_SIZE];
static int16_t sai_tx_buffer_0[SAI_TX_BUFFER_SIZE_HALFWORDS];
static int16_t sai_tx_buffer_1[SAI_TX_BUFFER_SIZE_HALFWORDS];
static uint32_t sai_rx_buffer_0[SAI_RX_BUFFER_SIZE_WORDS];
static uint32_t sai_rx_buffer_1[SAI_RX_BUFFER_SIZE_WORDS];
bool is_buffer_0 = true;
volatile bool vi_block_received = false;
static uint32_t playback_volume = DEFAULT_PLAYBACK_VOLUME;
static dynamic_f0_tuning_params_t dynamic_f0_tuning_params;
static bool f0_tuning_params_dirty = false;
static dynamic_f0_debug_info_t dynamic_f0_debug_info;

static bsp_callback_t bsp_haptic_int_cb;
static void *bsp_haptic_int_cb_arg;

static bsp_pb_debounce_data_t bsp_pb_debounce_data[] =
{
    {
        .gpio_pin = GPIO_PIN_13,
        .bsp_gpio_id = BSP_GPIO_ID_USER_PB,
        .debounce_counter = 0,
        .triggered = false,
        .active_level = BSP_GPIO_HIGH,
    },
    {
        .gpio_pin = BSP_HAPTIC_TRIGGER_PIN,
        .bsp_gpio_id = BSP_GPIO_ID_HAPTIC_TRIGGER,
        .debounce_counter = 0,
        .triggered = false,
        .active_level = BSP_GPIO_HIGH,
    }
};


static bsp_app_callback_t app_cb = NULL;
static void *app_cb_arg = NULL;

static volatile int32_t bsp_irq_count = 0;

static waveform_blockizer_t blockizer;

static algo_api_cycle_count_t algo_api_max_cycle_count = {0};

static cl_calibration_param calibration_parameters;

static bool bsp_haptic_is_calibrating = false;

static algo_api_ret_t algo_api_ret = {0};

static haptic_waveform_log_t haptic_waveform_logs[HAPTIC_TABLE_TOTAL];
static haptic_waveform_log_t *current_haptic_waveform_log = NULL;

static uint32_t bsp_i2c_current_register_index = 0;
static uint32_t bsp_haptic_trigger_index = 0;
static bool bsp_i2c_is_writing_register_index = true;

/***********************************************************************************************************************
 * GLOBAL VARIABLES
 **********************************************************************************************************************/
TIM_HandleTypeDef tim_drv_handle;
I2C_HandleTypeDef i2c_drv_handle;
SAI_HandleTypeDef sai_tx_drv_handle;
DMA_HandleTypeDef hdma_sai_tx;
SAI_HandleTypeDef sai_rx_drv_handle;
DMA_HandleTypeDef hdma_sai_rx;
FLASH_OBProgramInitTypeDef OBInit;
bool bsp_pb_pressed_flag = false;
bool bsp_pb_por_is_pressed = false;

/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0, 1, 2, 3, 4, 5};

volatile bool bsp_i2c_triggered_buzz = false;
volatile bool bsp_i2c_triggered_calibrate = false;

uint32_t bsp_toggle_gpio(uint32_t gpio_id);
uint32_t bsp_i2c_slave_register_access(uint32_t address, uint32_t *value, bool is_write);
static void CPU_CACHE_Enable(void);

/***********************************************************************************************************************
 * LOCAL FUNCTIONS
 **********************************************************************************************************************/
static void Error_Handler(void)
{
    bsp_set_gpio(BSP_GPIO_ID_LD3, BSP_GPIO_HIGH);
    while(1);

    return;
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

#if (BSP_MCU_CLOCK_CFG == BSP_MCU_CLOCK_CFG_HSE)
    /**
      * @brief  System Clock Configuration
      *         The system Clock is configured as follow :
      *            System Clock source            = PLL (HSE)
      *            SYSCLK(Hz)                     = 84000000
      *            HCLK(Hz)                       = 84000000
      *            AHB Prescaler                  = 1
      *            APB1 Prescaler                 = 2
      *            APB2 Prescaler                 = 1
      *            HSE Frequency(Hz)              = 8000000
      *            PLL_M                          = 8
      *            PLL_N                          = 336
      *            PLL_P                          = 4
      *            PLL_Q                          = 7
      *            VDD(V)                         = 3.3
      *            Main regulator output voltage  = Scale2 mode
      *            Flash Latency(WS)              = 2
      * @param  None
      * @retval None
      */

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
      Error_Handler();
    }
#elif (BSP_MCU_CLOCK_CFG == BSP_MCU_CLOCK_CFG_HSI)
    /**
      * @brief  Switch the PLL source from HSE  to HSI, and select the PLL as SYSCLK
      *         source.
      *         The system Clock is configured as follows :
      *            System Clock source            = PLL (HSI)
      *            SYSCLK(Hz)                     = 216000000
      *            HCLK(Hz)                       = 216000000
      *            AHB Prescaler                  = 1
      *            APB1 Prescaler                 = 4
      *            APB2 Prescaler                 = 2
      *            HSI Frequency(Hz)              = 16000000
      *            PLL_M                          = 16
      *            PLL_N                          = 432
      *            PLL_P                          = 2
      *            PLL_Q                          = 9
      *            PLL_R                          = 7
      *            VDD(V)                         = 3.3
      *            Flash Latency(WS)              = 7
      * @param  None
      * @retval None
      */

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* Enable HSI Oscillator and activate PLL with HSI as source */
    RCC_OscInitStruct.OscillatorType       = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState             = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue  = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState         = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource        = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM             = 16;
    RCC_OscInitStruct.PLL.PLLN             = 432;
    RCC_OscInitStruct.PLL.PLLP             = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ             = 9;
    RCC_OscInitStruct.PLL.PLLR             = 7;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    RCC_ClkInitStruct.ClockType       = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource    = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider   = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider  = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider  = RCC_HCLK_DIV2;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
    {
      Error_Handler();
    }
#endif

    RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;

    /* Configure PLLSAI prescalers */
    /* PLLSAI_VCO: VCO_429M
       SAI_CLK(first level) = PLLSAI_VCO/PLLSAIQ = 429/2 = 214.5 Mhz
       SAI_CLK_x = SAI_CLK(first level)/PLLSAIDIVQ = 214.5/19 = 11.289 Mhz
    */
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
    RCC_PeriphCLKInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI;
    RCC_PeriphCLKInitStruct.PLLSAI.PLLSAIN = 86;
    RCC_PeriphCLKInitStruct.PLLSAI.PLLSAIQ = 7;
    RCC_PeriphCLKInitStruct.PLLSAIDivQ = 1;

    if(HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    return;
}

static void I2C_Init(void)
{
    i2c_drv_handle.Instance = I2C2;

    i2c_drv_handle.Init.Timing = BSP_I2C_TIMING;
    i2c_drv_handle.Init.OwnAddress1 = BSP_HOST_I2C_SLAVE_ADDRESS_8BIT;
    i2c_drv_handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c_drv_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2c_drv_handle.Init.OwnAddress2 = 0xFF;
    i2c_drv_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2c_drv_handle.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
    if (HAL_I2C_Init(&i2c_drv_handle) != HAL_OK)
    {
      Error_Handler();
    }

    return;
}

static void SAI_Init(void)
{
    /* Initialize SAI */
    __HAL_SAI_RESET_HANDLE_STATE(&sai_tx_drv_handle);
    __HAL_SAI_RESET_HANDLE_STATE(&sai_rx_drv_handle);

    sai_tx_drv_handle.Instance = SAI_TX_HW;
    sai_rx_drv_handle.Instance = SAI_RX_HW;

    __HAL_SAI_DISABLE(&sai_tx_drv_handle);
    __HAL_SAI_DISABLE(&sai_rx_drv_handle);

#ifndef USE_VALIDATION_SETUP
    sai_tx_drv_handle.Init.MonoStereoMode = SAI_MONOMODE;
    sai_tx_drv_handle.Init.AudioFrequency = BSP_I2S_FS_HZ;
    sai_tx_drv_handle.Init.AudioMode      = SAI_MODEMASTER_TX;
    sai_tx_drv_handle.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
    sai_tx_drv_handle.Init.Protocol       = SAI_FREE_PROTOCOL;
    sai_tx_drv_handle.Init.DataSize       = BSP_TX_SAI_DATASIZE;
    sai_tx_drv_handle.Init.FirstBit       = SAI_FIRSTBIT_MSB;
    sai_tx_drv_handle.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;
    sai_tx_drv_handle.Init.Synchro        = SAI_ASYNCHRONOUS;
    sai_tx_drv_handle.Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
    //sai_tx_drv_handle.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
    sai_tx_drv_handle.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_EMPTY;
    sai_tx_drv_handle.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
    sai_tx_drv_handle.Init.CompandingMode = SAI_NOCOMPANDING;
    sai_tx_drv_handle.Init.TriState       = SAI_OUTPUT_NOTRELEASED;
    sai_tx_drv_handle.Init.Mckdiv         = 0;

    sai_tx_drv_handle.FrameInit.FrameLength       = (BSP_I2S_SUBFRAME_SIZE_BITS * BSP_I2S_CHANNEL_NBR);
    sai_tx_drv_handle.FrameInit.ActiveFrameLength = BSP_I2S_SUBFRAME_SIZE_BITS;
    sai_tx_drv_handle.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
    sai_tx_drv_handle.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
    sai_tx_drv_handle.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

    sai_tx_drv_handle.SlotInit.FirstBitOffset = 0;
    sai_tx_drv_handle.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
    sai_tx_drv_handle.SlotInit.SlotNumber     = 2;
    sai_tx_drv_handle.SlotInit.SlotActive     = (SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1);

    sai_rx_drv_handle.Init.MonoStereoMode = SAI_STEREOMODE;
    sai_rx_drv_handle.Init.AudioFrequency = BSP_I2S_FS_HZ;
    sai_rx_drv_handle.Init.AudioMode      = SAI_MODESLAVE_RX;
    sai_rx_drv_handle.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
    sai_rx_drv_handle.Init.Protocol       = SAI_FREE_PROTOCOL;
    sai_rx_drv_handle.Init.DataSize       = BSP_RX_SAI_DATASIZE;
    sai_rx_drv_handle.Init.FirstBit       = SAI_FIRSTBIT_MSB;
    sai_rx_drv_handle.Init.ClockStrobing  = SAI_CLOCKSTROBING_RISINGEDGE;
    sai_rx_drv_handle.Init.Synchro        = SAI_SYNCHRONOUS;
    sai_rx_drv_handle.Init.OutputDrive    = SAI_OUTPUTDRIVE_DISABLE;
    //sai_rx_drv_handle.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_1QF;
    sai_rx_drv_handle.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_EMPTY;
    sai_rx_drv_handle.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
    sai_rx_drv_handle.Init.CompandingMode = SAI_NOCOMPANDING;
    sai_rx_drv_handle.Init.TriState       = SAI_OUTPUT_RELEASED;
    sai_rx_drv_handle.Init.Mckdiv         = 0;

    sai_rx_drv_handle.FrameInit.FrameLength       = (BSP_I2S_SUBFRAME_SIZE_BITS * BSP_I2S_CHANNEL_NBR);
    sai_rx_drv_handle.FrameInit.ActiveFrameLength = BSP_I2S_SUBFRAME_SIZE_BITS;
    sai_rx_drv_handle.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
    sai_rx_drv_handle.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
    sai_rx_drv_handle.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

    sai_rx_drv_handle.SlotInit.FirstBitOffset = 0;
    sai_rx_drv_handle.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
    sai_rx_drv_handle.SlotInit.SlotNumber     = 2;
    sai_rx_drv_handle.SlotInit.SlotActive     = (SAI_SLOTACTIVE_0 | SAI_SLOTACTIVE_1);
#else
    sai_tx_drv_handle.Init.AudioFrequency = BSP_I2S_FS_HZ;
    sai_tx_drv_handle.Init.AudioMode      = SAI_MODESLAVE_TX;
    sai_tx_drv_handle.Init.NoDivider      = SAI_MASTERDIVIDER_DISABLE;
    sai_tx_drv_handle.Init.Protocol       = SAI_FREE_PROTOCOL;
    sai_tx_drv_handle.Init.DataSize       = BSP_TX_SAI_DATASIZE;
    sai_tx_drv_handle.Init.FirstBit       = SAI_FIRSTBIT_MSB;
    sai_tx_drv_handle.Init.ClockStrobing  = SAI_CLOCKSTROBING_FALLINGEDGE;
    sai_tx_drv_handle.Init.Synchro        = SAI_ASYNCHRONOUS;
    sai_tx_drv_handle.Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
    //sai_tx_drv_handle.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_EMPTY;
    sai_tx_drv_handle.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_HF;
    sai_tx_drv_handle.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
    sai_tx_drv_handle.Init.CompandingMode = SAI_NOCOMPANDING;
    sai_tx_drv_handle.Init.TriState       = SAI_OUTPUT_NOTRELEASED;
    sai_tx_drv_handle.Init.Mckdiv         = 0;

    sai_tx_drv_handle.FrameInit.FrameLength       = (BSP_I2S_SUBFRAME_SIZE_BITS * BSP_I2S_CHANNEL_NBR);
    sai_tx_drv_handle.FrameInit.ActiveFrameLength = 1;
    sai_tx_drv_handle.FrameInit.FSDefinition      = SAI_FS_STARTFRAME;
    sai_tx_drv_handle.FrameInit.FSPolarity        = SAI_FS_ACTIVE_HIGH;
    sai_tx_drv_handle.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

    sai_tx_drv_handle.SlotInit.FirstBitOffset = 0;
    sai_tx_drv_handle.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
    sai_tx_drv_handle.SlotInit.SlotNumber     = BSP_I2S_CHANNEL_NBR;
    sai_tx_drv_handle.SlotInit.SlotActive     = (SAI_SLOTACTIVE_8);

    sai_rx_drv_handle.Init.AudioFrequency = BSP_I2S_FS_HZ;
    sai_rx_drv_handle.Init.AudioMode      = SAI_MODESLAVE_RX;
    sai_rx_drv_handle.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
    sai_rx_drv_handle.Init.Protocol       = SAI_FREE_PROTOCOL;
    sai_rx_drv_handle.Init.DataSize       = BSP_RX_SAI_DATASIZE;
    sai_rx_drv_handle.Init.FirstBit       = SAI_FIRSTBIT_MSB;
    sai_rx_drv_handle.Init.ClockStrobing  = SAI_CLOCKSTROBING_RISINGEDGE;
    sai_rx_drv_handle.Init.Synchro        = SAI_SYNCHRONOUS;
    sai_rx_drv_handle.Init.OutputDrive    = SAI_OUTPUTDRIVE_DISABLE;
    sai_rx_drv_handle.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_EMPTY;
    sai_rx_drv_handle.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
    sai_rx_drv_handle.Init.CompandingMode = SAI_NOCOMPANDING;
    sai_rx_drv_handle.Init.TriState       = SAI_OUTPUT_RELEASED;
    sai_rx_drv_handle.Init.Mckdiv         = 0;

    sai_rx_drv_handle.FrameInit.FrameLength       = (BSP_I2S_SUBFRAME_SIZE_BITS * BSP_I2S_CHANNEL_NBR);
    sai_rx_drv_handle.FrameInit.ActiveFrameLength = 1;
    sai_rx_drv_handle.FrameInit.FSDefinition      = SAI_FS_STARTFRAME;
    sai_rx_drv_handle.FrameInit.FSPolarity        = SAI_FS_ACTIVE_HIGH;
    sai_rx_drv_handle.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

    sai_rx_drv_handle.SlotInit.FirstBitOffset = 0;
    sai_rx_drv_handle.SlotInit.SlotSize       = SAI_SLOTSIZE_DATASIZE;
    sai_rx_drv_handle.SlotInit.SlotNumber     = BSP_I2S_CHANNEL_NBR;
    sai_rx_drv_handle.SlotInit.SlotActive     = (SAI_SLOTACTIVE_8 | SAI_SLOTACTIVE_10);
#endif

    if(HAL_OK != HAL_SAI_Init(&sai_tx_drv_handle))
    {
      Error_Handler();
    }

    if(HAL_OK != HAL_SAI_Init(&sai_rx_drv_handle))
    {
      Error_Handler();
    }

    __HAL_SAI_ENABLE(&sai_rx_drv_handle);
    __HAL_SAI_ENABLE(&sai_tx_drv_handle);

    return;
}

static void Timer_Init(void)
{
    uint32_t uwPrescalerValue;
    /*##-1- Configure the TIM peripheral #######################################*/
     /* -----------------------------------------------------------------------
       In this example TIM2 input clock (TIM2CLK) is set to 2 * APB1 clock (PCLK1),
       since APB1 prescaler is different from 1.
         TIM2CLK = 2 * PCLK1
         PCLK1 = HCLK / 2
         => TIM2CLK = HCLK = SystemCoreClock
       To get TIM2 counter clock at 10 KHz, the Prescaler is computed as following:
       Prescaler = (TIM2CLK / TIM2 counter clock) - 1
       Prescaler = (SystemCoreClock /10 KHz) - 1

       Note:
        SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
        Each time the core clock (HCLK) changes, user had to update SystemCoreClock
        variable value. Otherwise, any configuration based on this variable will be incorrect.
        This variable is updated in three ways:
         1) by calling CMSIS function SystemCoreClockUpdate()
         2) by calling HAL API function HAL_RCC_GetSysClockFreq()
         3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
     ----------------------------------------------------------------------- */

     /* Compute the prescaler value to have TIM2 counter clock equal to 10 KHz */
     uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);

     /* Set TIMx instance */
     tim_drv_handle.Instance = TIM2;

     /* Initialize TIM3 peripheral as follow:
          + Period = 10000 - 1
          + Prescaler = ((SystemCoreClock/2)/10000) - 1
          + ClockDivision = 0
          + Counter direction = Up
     */
     tim_drv_handle.Init.Period = 10000 - 1;
     tim_drv_handle.Init.Prescaler = uwPrescalerValue;
     tim_drv_handle.Init.ClockDivision = 0;
     tim_drv_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
     tim_drv_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

     return;
}

static void Timer_Start(uint32_t delay_100us)
{
    if(HAL_TIM_Base_Stop_IT(&tim_drv_handle) != HAL_OK)
    {
      Error_Handler();
    }

    tim_drv_handle.Init.Period = delay_100us;
    if(HAL_TIM_Base_Init(&tim_drv_handle) != HAL_OK)
    {
      Error_Handler();
    }

    if(HAL_TIM_Base_Start_IT(&tim_drv_handle) != HAL_OK)
    {
      Error_Handler();
    }

    return;
}

/***********************************************************************************************************************
 * MCU HAL FUNCTIONS
 **********************************************************************************************************************/
void HAL_MspInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable clocks to ports used
    BSP_LED_CLK_ENABLE();
    BSP_PB_CLK_ENABLE();
    BSP_HAPTIC_RESET_CLK_ENABLE();
    BSP_HAPTIC_INT_CLK_ENABLE();
    BSP_DEBUG_PIN_CLK_ENABLE();

    // Configure the LED GPOs
    GPIO_InitStruct.Pin = (BSP_LED_LD1_PIN | BSP_LED_LD2_PIN | BSP_LED_LD3_PIN);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Alternate = 0;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BSP_LED_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BSP_LED_GPIO_PORT, BSP_LED_LD1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BSP_LED_GPIO_PORT, BSP_LED_LD2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BSP_LED_GPIO_PORT, BSP_LED_LD3_PIN, GPIO_PIN_SET);

    // Configure the Amp Reset GPO
/*
    HAL_GPIO_WritePin(BSP_HAPTIC_RESET_GPIO_PORT, BSP_HAPTIC_RESET_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = BSP_HAPTIC_RESET_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Alternate = 0;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BSP_HAPTIC_RESET_GPIO_PORT, &GPIO_InitStruct);
*/

    // Configure Amp Interrupt GPI
    GPIO_InitStruct.Pin = BSP_HAPTIC_INT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Alternate = 0;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BSP_HAPTIC_INT_GPIO_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority((IRQn_Type)EXTI2_IRQn, 0x0F, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)EXTI2_IRQn);

    // Configure the Push Button GPI
    GPIO_InitStruct.Pin = BSP_PB_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Alternate = 0;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BSP_PB_GPIO_PORT, &GPIO_InitStruct);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)EXTI15_10_IRQn, EXTI15_10_IRQ_PREPRIO, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)EXTI15_10_IRQn);

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
    {
        bsp_pb_por_is_pressed = true;
    }

    // Configure the Debug GPO
    HAL_GPIO_WritePin(BSP_DEBUG_PIN_GPIO_PORT, BSP_DEBUG_PIN_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = BSP_DEBUG_PIN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Alternate = 0;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BSP_DEBUG_PIN_GPIO_PORT, &GPIO_InitStruct);

#ifdef USE_VALIDATION_SETUP
    BSP_HAPTIC_TRIGGER_CLK_ENABLE();

    GPIO_InitStruct.Pin = BSP_HAPTIC_TRIGGER_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Alternate = 0;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(BSP_HAPTIC_TRIGGER_GPIO_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority((IRQn_Type)EXTI4_IRQn, 0x0F, 0x00);
    HAL_NVIC_EnableIRQ((IRQn_Type)EXTI4_IRQn);
#endif

    return;
}

void HAL_MspDeInit(void)
{
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_13);

    HAL_GPIO_DeInit(BSP_HAPTIC_RESET_GPIO_PORT, BSP_HAPTIC_RESET_PIN);
    HAL_GPIO_DeInit(BSP_HAPTIC_INT_GPIO_PORT, BSP_HAPTIC_INT_PIN);
    HAL_GPIO_DeInit(BSP_DEBUG_PIN_GPIO_PORT, BSP_DEBUG_PIN_PIN);

    BSP_LED_CLK_DISABLE();
    BSP_PB_CLK_DISABLE();
    BSP_HAPTIC_RESET_CLK_DISABLE();
    BSP_HAPTIC_INT_CLK_DISABLE();
    BSP_DEBUG_PIN_CLK_DISABLE();

#ifdef USE_VALIDATION_SETUP
    HAL_GPIO_DeInit(BSP_HAPTIC_TRIGGER_GPIO_PORT, BSP_HAPTIC_TRIGGER_PIN);
    BSP_HAPTIC_TRIGGER_CLK_DISABLE();
#endif

    return;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BSP_HAPTIC_INT_PIN)
    {
        if (bsp_haptic_int_cb != NULL)
        {
            bsp_haptic_int_cb(BSP_STATUS_OK, bsp_haptic_int_cb_arg);
        }
    }

    if (GPIO_Pin == GPIO_PIN_13)
    {
        bsp_pb_debounce_data[0].debounce_counter = HAL_GetTick();
        bsp_pb_debounce_data[0].triggered = true;
    }

    if (GPIO_Pin == BSP_HAPTIC_TRIGGER_PIN)
    {
        bsp_pb_debounce_data[1].debounce_counter = HAL_GetTick();
        bsp_pb_debounce_data[1].triggered = true;
    }

    bsp_irq_count++;

    return;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM2_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }

    return;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        if ((bsp_timer_has_started) && (bsp_timer_cb != NULL))
        {
            if(HAL_TIM_Base_Stop_IT(&tim_drv_handle) != HAL_OK)
            {
              Error_Handler();
            }

            bsp_timer_cb(BSP_STATUS_OK, bsp_timer_cb_arg);
            bsp_timer_cb = NULL;
            bsp_timer_cb_arg = NULL;
        }

        bsp_timer_has_started = !bsp_timer_has_started;
    }

    bsp_irq_count++;

    return;
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(hi2c->Instance==I2C2)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        __HAL_RCC_I2C2_CLK_ENABLE();

        //HAL_NVIC_SetPriority(I2C2_ER_IRQn, 1, 0);
        HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
        //HAL_NVIC_SetPriority(I2C2_EV_IRQn, 2, 0);
        HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 2);
        HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
    }

    return;
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
    if(hi2c->Instance==I2C2)
    {
        __HAL_RCC_I2C2_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOF, GPIO_PIN_0|GPIO_PIN_1);

        HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
        HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
    }

    return;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_READY)
    {
        if (bsp_i2c_current_transaction_type == BSP_I2C_TRANSACTION_TYPE_READ_REPEATED_START)
        {
            HAL_I2C_Master_Seq_Receive_IT(hi2c,
                                          bsp_i2c_read_address,
                                          bsp_i2c_read_buffer_ptr,
                                          bsp_i2c_read_length,
                                          I2C_LAST_FRAME);
        }
        else if (bsp_i2c_current_transaction_type == BSP_I2C_TRANSACTION_TYPE_WRITE)
        {
            if (bsp_i2c_done_cb != NULL)
            {
                bsp_i2c_done_cb(BSP_STATUS_OK, bsp_i2c_done_cb_arg);
            }
        }
        else if (bsp_i2c_current_transaction_type == BSP_I2C_TRANSACTION_TYPE_DB_WRITE)
        {
            if (bsp_i2c_write_length == 0)
            {
                if (bsp_i2c_done_cb != NULL)
                {
                    bsp_i2c_done_cb(BSP_STATUS_OK, bsp_i2c_done_cb_arg);
                }
            }
            else
            {
                HAL_I2C_Master_Seq_Transmit_IT(hi2c,
                                               bsp_i2c_read_address,
                                               bsp_i2c_write_buffer_ptr,
                                               bsp_i2c_write_length,
                                               I2C_LAST_FRAME);
                bsp_i2c_write_length = 0;
            }
        }
    }

    bsp_irq_count++;

    return;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (HAL_I2C_GetState(hi2c) == HAL_I2C_STATE_READY)
    {
        if (bsp_i2c_current_transaction_type != BSP_I2C_TRANSACTION_TYPE_INVALID)
        {
            if (bsp_i2c_done_cb != NULL)
            {
                bsp_i2c_done_cb(BSP_STATUS_OK, bsp_i2c_done_cb_arg);
            }
        }
    }

    bsp_irq_count++;

    return;
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    BSP_DEBUG_PIN_TOGGLE
    BSP_DEBUG_PIN_TOGGLE
    BSP_DEBUG_PIN_TOGGLE
    BSP_DEBUG_PIN_TOGGLE
    BSP_DEBUG_PIN_TOGGLE
    BSP_DEBUG_PIN_TOGGLE

    if (i2c_write_buffer_index >= 4)
    {
        uint32_t temp_data;

        bsp_i2c_slave_register_access(bsp_i2c_current_register_index++, &temp_data, false);

        i2c_write_buffer[0] = (temp_data >> 24) & 0x000000FF;
        i2c_write_buffer[1] = (temp_data >> 16) & 0x000000FF;
        i2c_write_buffer[2] = (temp_data >> 8) & 0x000000FF;
        i2c_write_buffer[3] = temp_data & 0x000000FF;

        i2c_write_buffer_index = 0;
        if (HAL_I2C_Slave_Seq_Transmit_IT(&i2c_drv_handle, (uint8_t*) &(i2c_write_buffer[i2c_write_buffer_index++]), 1, I2C_NEXT_FRAME)!= HAL_OK)
        {
          Error_Handler();
        }
    }

    if (HAL_I2C_Slave_Seq_Transmit_IT(&i2c_drv_handle, (uint8_t*) &(i2c_write_buffer[i2c_write_buffer_index++]), 1, I2C_NEXT_FRAME)!= HAL_OK)
    {
      Error_Handler();
    }

    return;
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    // If in the middle of a transaction, abort
    BSP_DEBUG_PIN_TOGGLE
    BSP_DEBUG_PIN_TOGGLE
    BSP_DEBUG_PIN_TOGGLE
    BSP_DEBUG_PIN_TOGGLE

    if (i2c_read_buffer_index >= 4)
    {
        uint32_t temp_data;

        temp_data = i2c_read_buffer[0] << 24;
        temp_data |= i2c_read_buffer[1] << 16;
        temp_data |= i2c_read_buffer[2] << 8;
        temp_data |= i2c_read_buffer[3];

        if (bsp_i2c_is_writing_register_index)
        {
            bsp_i2c_current_register_index = temp_data;
            bsp_i2c_is_writing_register_index = false;
        }
        else
        {
            bsp_i2c_slave_register_access(bsp_i2c_current_register_index++, &temp_data, true);
        }

        i2c_read_buffer_index = 0;
    }

    if(HAL_I2C_Slave_Seq_Receive_IT(&i2c_drv_handle, (uint8_t *) &(i2c_read_buffer[i2c_read_buffer_index++]), 1, I2C_FIRST_FRAME) != HAL_OK)
    {
      Error_Handler();
    }

    return;
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    if (AddrMatchCode == BSP_HOST_I2C_SLAVE_ADDRESS_8BIT)
    {
        /* A new communication with a Master is initiated */
        if (TransferDirection == I2C_DIRECTION_TRANSMIT)
        {
            i2c_read_buffer_index = 0;
            bsp_i2c_is_writing_register_index = true;
            if(HAL_I2C_Slave_Seq_Receive_IT(&i2c_drv_handle, (uint8_t *) &(i2c_read_buffer[i2c_read_buffer_index++]), 1, I2C_FIRST_FRAME) != HAL_OK)
            {
              Error_Handler();
            }
        }
        else
        {
            uint32_t temp_data;
            bsp_i2c_slave_register_access(bsp_i2c_current_register_index++, &temp_data, false);

            i2c_write_buffer[0] = (temp_data >> 24) & 0x000000FF;
            i2c_write_buffer[1] = (temp_data >> 16) & 0x000000FF;
            i2c_write_buffer[2] = (temp_data >> 8) & 0x000000FF;
            i2c_write_buffer[3] = temp_data & 0x000000FF;

            i2c_drv_handle.Instance->TXDR = i2c_write_buffer[0];
            i2c_write_buffer_index = 1;
            if (HAL_I2C_Slave_Seq_Transmit_IT(&i2c_drv_handle, (uint8_t*) &(i2c_write_buffer[i2c_write_buffer_index++]), 1, I2C_NEXT_FRAME) != HAL_OK)
            {
              Error_Handler();
            }
        }
    }
    else
    {
        Error_Handler();
    }

    return;
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (HAL_I2C_EnableListen_IT(&i2c_drv_handle) != HAL_OK)
    {
        Error_Handler();
    }
    return;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    uint32_t temp_data;

    temp_data = i2c_drv_handle.Instance->ISR;
    i2c_write_buffer_index = temp_data;

    if (hi2c->ErrorCode != HAL_I2C_ERROR_AF)
    {
        BSP_DEBUG_PIN_TOGGLE
        Error_Handler();
    }

    return;
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    Error_Handler();

    return;
}

void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai)
{
    GPIO_InitTypeDef  GPIO_Init;

    if (hsai->Instance == SAI_TX_HW)
    {
        SAI_CLK_ENABLE();
        SAI_GPIO_PORT_CLK_ENABLE();

        GPIO_Init.Pin       = (SAI_LRCLK_PIN | SAI_SCLK_PIN | SAI_SDOUT_PIN | SAI_SDIN_PIN);
        GPIO_Init.Mode      = GPIO_MODE_AF_PP;
        GPIO_Init.Pull      = GPIO_NOPULL;
        GPIO_Init.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_Init.Alternate = SAI_GPIO_AF;
        HAL_GPIO_Init(SAI_GPIO_PORT, &GPIO_Init);

        SAI_TX_DMAx_CLK_ENABLE();

        hdma_sai_tx.Init.Channel             = SAI_TX_DMAx_CHANNEL;
        hdma_sai_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_sai_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_sai_tx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_sai_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_sai_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_sai_tx.Init.Mode                = DMA_NORMAL;
        hdma_sai_tx.Init.Priority            = DMA_PRIORITY_HIGH;
        //hdma_sai_tx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
        hdma_sai_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        hdma_sai_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
        hdma_sai_tx.Init.MemBurst            = DMA_MBURST_SINGLE;
        hdma_sai_tx.Init.PeriphBurst         = DMA_PBURST_SINGLE;
        hdma_sai_tx.Instance                 = SAI_TX_DMAx_STREAM;

        __HAL_LINKDMA(hsai, hdmatx, hdma_sai_tx);
        HAL_DMA_DeInit(&hdma_sai_tx);
        HAL_DMA_Init(&hdma_sai_tx);

        HAL_NVIC_SetPriority(SAI_TX_DMAx_IRQ, SAI_TX_IRQ_PREPRIO, 0);
        HAL_NVIC_EnableIRQ(SAI_TX_DMAx_IRQ);
    }

    if (hsai->Instance == SAI_RX_HW)
    {
        SAI_RX_DMAx_CLK_ENABLE();

        hdma_sai_rx.Init.Channel             = SAI_RX_DMAx_CHANNEL;
        hdma_sai_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_sai_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_sai_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_sai_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_sai_rx.Init.MemDataAlignment    = DMA_PDATAALIGN_HALFWORD;
        hdma_sai_rx.Init.Mode                = DMA_NORMAL;
        hdma_sai_rx.Init.Priority            = DMA_PRIORITY_HIGH;
        //hdma_sai_tx.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;
        hdma_sai_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        hdma_sai_rx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
        hdma_sai_rx.Init.MemBurst            = DMA_MBURST_SINGLE;
        hdma_sai_rx.Init.PeriphBurst         = DMA_PBURST_SINGLE;
        hdma_sai_rx.Instance                 = SAI_RX_DMAx_STREAM;

        __HAL_LINKDMA(hsai, hdmarx, hdma_sai_rx);
        HAL_DMA_DeInit(&hdma_sai_rx);
        HAL_DMA_Init(&hdma_sai_rx);

        HAL_NVIC_SetPriority(SAI_RX_DMAx_IRQ, SAI_RX_IRQ_PREPRIO, 0);
        HAL_NVIC_EnableIRQ(SAI_RX_DMAx_IRQ);
    }

    return;
}

void HAL_SAI_MspDeInit(SAI_HandleTypeDef *hsai)
{
    GPIO_InitTypeDef  GPIO_Init;

    if ((hsai->Instance == SAI_TX_HW) || (hsai->Instance == SAI_RX_HW))
    {
        HAL_NVIC_DisableIRQ(SAI_TX_DMAx_IRQ);
        HAL_NVIC_DisableIRQ(SAI_RX_DMAx_IRQ);

        HAL_DMA_DeInit(hsai->hdmatx);
        HAL_DMA_DeInit(hsai->hdmarx);

        __HAL_SAI_DISABLE(hsai);

        GPIO_Init.Pin       = (SAI_LRCLK_PIN | SAI_SCLK_PIN | SAI_SDOUT_PIN | SAI_SDIN_PIN);
        HAL_GPIO_DeInit(SAI_GPIO_PORT, GPIO_Init.Pin);

        SAI_CLK_DISABLE();
    }

    return;
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
    if(hsai->Instance == SAI_TX_HW)
    {
        BSP_DEBUG_PIN_TOGGLE;
        BSP_DEBUG_PIN_TOGGLE;
        if (blockizer.current_block_size != 0)
        {
            if (is_buffer_0)
            {
                HAL_SAI_Transmit_DMA(&sai_tx_drv_handle, (uint8_t*) sai_tx_buffer_0, blockizer.output_block_size);
            }
            else
            {
                HAL_SAI_Transmit_DMA(&sai_tx_drv_handle, (uint8_t*) sai_tx_buffer_1, blockizer.output_block_size);
            }
        }
    }

    bsp_irq_count++;

    return;
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
    if(hsai->Instance == SAI_RX_HW)
    {
        BSP_DEBUG_PIN_TOGGLE;
        BSP_DEBUG_PIN_TOGGLE;
        BSP_DEBUG_PIN_TOGGLE;
        BSP_DEBUG_PIN_TOGGLE;
        vi_block_received = true;

        if (blockizer.current_block_size != 0)
        {
            if (is_buffer_0)
            {
                HAL_SAI_Receive_DMA(&sai_rx_drv_handle, (uint8_t*) sai_rx_buffer_0, (blockizer.output_block_size * 2));
            }
            else
            {
                HAL_SAI_Receive_DMA(&sai_rx_drv_handle, (uint8_t*) sai_rx_buffer_1, (blockizer.output_block_size * 2));
            }

        }
        else
        {
            app_cb(BSP_STATUS_OK, app_cb_arg);
        }
    }

    bsp_irq_count++;

    return;
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
    if ((hsai->Instance == SAI_TX_HW) || (hsai->Instance == SAI_RX_HW))
    {
        Error_Handler();
    }

    return;
}

/***********************************************************************************************************************
 * API FUNCTIONS
 **********************************************************************************************************************/
uint32_t bsp_set_gpio(uint32_t gpio_id, uint8_t gpio_state)
{
    switch (gpio_id)
    {
        case BSP_GPIO_ID_LD1:
            HAL_GPIO_WritePin(BSP_LED_GPIO_PORT, BSP_LED_LD1_PIN, (GPIO_PinState) gpio_state);
            break;


        case BSP_GPIO_ID_LD2:
            HAL_GPIO_WritePin(BSP_LED_GPIO_PORT, BSP_LED_LD2_PIN, (GPIO_PinState) gpio_state);
            break;


        case BSP_GPIO_ID_LD3:
            HAL_GPIO_WritePin(BSP_LED_GPIO_PORT, BSP_LED_LD3_PIN, (GPIO_PinState) gpio_state);
            break;

        case BSP_GPIO_ID_DEBUG_PIN:
            HAL_GPIO_WritePin(BSP_DEBUG_PIN_GPIO_PORT, BSP_DEBUG_PIN_PIN, (GPIO_PinState) gpio_state);
            break;

        case BSP_GPIO_ID_HAPTIC_RESET:
        {
            //HAL_GPIO_WritePin(BSP_HAPTIC_RESET_GPIO_PORT, BSP_HAPTIC_RESET_PIN, (GPIO_PinState) gpio_state);

            // Enable AIF1 and AIF2 path from Lochnagar to Amps, clear L Amp RESET (active high)
#ifndef USE_VALIDATION_SETUP
            uint8_t temp_buffer[3] = {0x02, 0x93, 0x00};
#else
            uint8_t temp_buffer[3] = {0x02, 0xD6, 0x9C};
#endif
            if (gpio_state == BSP_GPIO_HIGH)
            {
                temp_buffer[1] |= 0x08;
            }
            HAL_I2C_Master_Seq_Transmit_IT(&i2c_drv_handle,
                                           BSP_TCA9539_I2C_ADDRESS,
                                           temp_buffer,
                                           3,
                                           I2C_FIRST_AND_LAST_FRAME);
            while (HAL_I2C_GetState(&i2c_drv_handle) != HAL_I2C_STATE_READY)
            {
            }

            break;
        }

        default:
            break;
    }

    return BSP_STATUS_OK;
}

uint32_t bsp_get_gpio(uint32_t gpio_id, uint8_t *gpio_state)
{
    switch (gpio_id)
    {
        case BSP_GPIO_ID_USER_PB:
            *gpio_state = (uint8_t) HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
            break;

        case BSP_GPIO_ID_HAPTIC_TRIGGER:
            *gpio_state = (uint8_t) HAL_GPIO_ReadPin(BSP_HAPTIC_TRIGGER_GPIO_PORT, BSP_HAPTIC_TRIGGER_PIN);
            break;

        default:
            break;
    }

    return BSP_STATUS_OK;
}

uint32_t bsp_toggle_gpio(uint32_t gpio_id)
{
    switch (gpio_id)
    {
        case BSP_GPIO_ID_LD1:
            HAL_GPIO_TogglePin(BSP_LED_GPIO_PORT, BSP_LED_LD1_PIN);
            break;


        case BSP_GPIO_ID_LD2:
            HAL_GPIO_TogglePin(BSP_LED_GPIO_PORT, BSP_LED_LD2_PIN);
            break;


        case BSP_GPIO_ID_LD3:
            HAL_GPIO_TogglePin(BSP_LED_GPIO_PORT, BSP_LED_LD3_PIN);
            break;

        case BSP_GPIO_ID_DEBUG_PIN:
            HAL_GPIO_TogglePin(BSP_DEBUG_PIN_GPIO_PORT, BSP_DEBUG_PIN_PIN);
            break;

        default:
            break;
    }

    return BSP_STATUS_OK;
}

uint32_t bsp_set_timer(uint32_t duration_ms, bsp_callback_t cb, void *cb_arg)
{
    bsp_timer_cb = cb;
    bsp_timer_cb_arg = cb_arg;
    bsp_timer_has_started = false;

    Timer_Start(duration_ms * 10);

    return BSP_STATUS_OK;
}

uint32_t bsp_i2c_read_repeated_start(uint32_t bsp_dev_id,
                                     uint8_t *write_buffer,
                                     uint32_t write_length,
                                     uint8_t *read_buffer,
                                     uint32_t read_length,
                                     bsp_callback_t cb,
                                     void *cb_arg)
{
    switch (bsp_dev_id)
    {
        case BSP_HAPTIC_DEV_ID:
            if (cb == NULL)
            {
                HAL_I2C_Master_Seq_Transmit_IT(&i2c_drv_handle, BSP_CS35L36_I2C_ADDRESS, write_buffer, write_length, I2C_FIRST_FRAME);
                while (HAL_I2C_GetState(&i2c_drv_handle) != HAL_I2C_STATE_READY)
                {
                }
                HAL_I2C_Master_Seq_Receive_IT(&i2c_drv_handle, BSP_CS35L36_I2C_ADDRESS, read_buffer, read_length, I2C_LAST_FRAME);
                while (HAL_I2C_GetState(&i2c_drv_handle) != HAL_I2C_STATE_READY)
                {
                }
            }
            else
            {
                bsp_i2c_done_cb = cb;
                bsp_i2c_done_cb_arg = cb_arg;
                bsp_i2c_current_transaction_type = BSP_I2C_TRANSACTION_TYPE_READ_REPEATED_START;
                bsp_i2c_read_buffer_ptr = read_buffer;
                bsp_i2c_read_length = read_length;
                bsp_i2c_read_address = 0x80;
                HAL_I2C_Master_Seq_Transmit_IT(&i2c_drv_handle,
                                               bsp_i2c_read_address,
                                               write_buffer,
                                               write_length,
                                               I2C_FIRST_FRAME);
            }

            break;

        default:
            break;
    }

    return BSP_STATUS_OK;
}

uint32_t bsp_i2c_write(uint32_t bsp_dev_id,
                       uint8_t *write_buffer,
                       uint32_t write_length,
                       bsp_callback_t cb,
                       void *cb_arg)
{
    uint16_t i2c_address;

    switch (bsp_dev_id)
    {
        case BSP_HAPTIC_DEV_ID:
            i2c_address = BSP_CS35L36_I2C_ADDRESS;
            break;

        case BSP_DEV_ID_NULL:
        default:
            return BSP_STATUS_FAIL;
            break;
    }

    HAL_I2C_Master_Seq_Transmit_IT(&i2c_drv_handle, i2c_address, write_buffer, write_length, I2C_FIRST_AND_LAST_FRAME);
    while (HAL_I2C_GetState(&i2c_drv_handle) != HAL_I2C_STATE_READY)
    {
    }

    return BSP_STATUS_OK;
}

uint32_t bsp_i2c_db_write(uint32_t bsp_dev_id,
                          uint8_t *write_buffer_0,
                          uint32_t write_length_0,
                          uint8_t *write_buffer_1,
                          uint32_t write_length_1,
                          bsp_callback_t cb,
                          void *cb_arg)
{
    switch (bsp_dev_id)
    {
        case BSP_HAPTIC_DEV_ID:
            if (cb == NULL)
            {
                HAL_I2C_Master_Seq_Transmit_IT(&i2c_drv_handle, BSP_CS35L36_I2C_ADDRESS, write_buffer_0, write_length_0, I2C_FIRST_FRAME);
                while (HAL_I2C_GetState(&i2c_drv_handle) != HAL_I2C_STATE_READY)
                {
                }
                HAL_I2C_Master_Seq_Transmit_IT(&i2c_drv_handle, BSP_CS35L36_I2C_ADDRESS, write_buffer_0, write_length_0, I2C_LAST_FRAME);
                while (HAL_I2C_GetState(&i2c_drv_handle) != HAL_I2C_STATE_READY)
                {
                }
            }
            else
            {
                bsp_i2c_done_cb = cb;
                bsp_i2c_done_cb_arg = cb_arg;
                bsp_i2c_read_address = 0x80;
                bsp_i2c_write_length = write_length_1;
                bsp_i2c_write_buffer_ptr = write_buffer_1;

                bsp_i2c_current_transaction_type = BSP_I2C_TRANSACTION_TYPE_DB_WRITE;

                HAL_I2C_Master_Seq_Transmit_IT(&i2c_drv_handle, BSP_CS35L36_I2C_ADDRESS, write_buffer_0, write_length_0, I2C_FIRST_FRAME);
            }

            break;

        default:
            break;
    }

    return BSP_STATUS_OK;
}

uint32_t bsp_register_gpio_cb(uint32_t gpio_id, bsp_callback_t cb, void *cb_arg)
{
    bsp_haptic_int_cb = cb;
    bsp_haptic_int_cb_arg = cb_arg;

    return BSP_STATUS_OK;
}

uint32_t bsp_i2c_reset(uint32_t bsp_dev_id)
{
    switch (bsp_dev_id)
    {
        case BSP_HAPTIC_DEV_ID:
            HAL_I2C_Master_Abort_IT(&i2c_drv_handle, 0x80);
            break;

        default:
            break;
    }

    return BSP_STATUS_OK;
}

uint32_t bsp_enable_i2c_listener(void)
{
    // Re-initialize I2C buffer
    for (int i = 0; i < (sizeof(i2c_write_buffer)/sizeof(uint8_t)); i++)
    {
        i2c_read_buffer[i] = i + 1;
        i2c_write_buffer[i] = i + 1;
    }

    if (HAL_I2C_EnableListen_IT(&i2c_drv_handle) != HAL_OK)
    {
        Error_Handler();
    }

    return BSP_STATUS_OK;
}

void bsp_sleep(void)
{
    __disable_irq();
    bsp_irq_count--;

    if (bsp_irq_count <= 0)
    {
        bsp_irq_count = 0;
        __enable_irq();
        __WFI();
    }
    else
    {
        __enable_irq();
    }

    return;
}

uint32_t bsp_blockizer_initialize(uint16_t default_block_size)
{
    blockizer.default_block_size = default_block_size;
    blockizer.output_block_size = default_block_size;
    blockizer.current_block = NULL;
    blockizer.first_block = NULL;
    blockizer.last_block = NULL;
    blockizer.last_block_word_count = 0;
    blockizer.current_block_size = 0;
    blockizer.current_repetition = 0;
    blockizer.repetition_total = 0;
    blockizer.is_last_block = false;

    return BSP_STATUS_OK;
}

uint32_t bsp_blockizer_blockize(int16_t *waveform_buffer, uint32_t waveform_word_count, uint16_t block_size, uint8_t repetition_count)
{

    blockizer.default_block_size = block_size;
    blockizer.current_block = blockizer.first_block = waveform_buffer;
    blockizer.last_block_word_count = waveform_word_count % block_size;

    blockizer.last_block = blockizer.first_block;
    blockizer.last_block += (waveform_word_count / block_size) * block_size;
    blockizer.is_last_block = false;

    if (blockizer.last_block_word_count == 0)
    {
        blockizer.last_block_word_count = block_size;
        blockizer.last_block -= block_size;
    }

    if (blockizer.current_block != blockizer.last_block)
    {
        blockizer.current_block_size = block_size;
    }
    else
    {
        blockizer.current_block_size = waveform_word_count;
        blockizer.is_last_block = true;
    }

    blockizer.current_repetition = 1;
    blockizer.repetition_total = repetition_count;

    return BSP_STATUS_OK;
}

bool bsp_blockizer_advance(void)
{
    bool ret = true;

    if ((blockizer.is_last_block == false) || \
        ((blockizer.is_last_block == true) && \
         (blockizer.current_repetition < blockizer.repetition_total)))
    {
        if (blockizer.is_last_block == true)
        {
            // We were at last block before calling, time to wrap
            blockizer.current_repetition++;
            blockizer.current_block = blockizer.first_block;
            blockizer.current_block_size = blockizer.default_block_size;
            blockizer.is_last_block = false;
        }
        else
        {
            blockizer.current_block += blockizer.current_block_size;
        }

        if (blockizer.current_block == blockizer.last_block)
        {
            // TX block is a fixed size, so at the last block create a buffer with pads to feed to play_effect
            // this buffer, tx_buffer_last, needs to be large enough to handle sped up waveforms, where the input size is larger than 
            // the output size
            for (int i=0; i<blockizer.current_block_size; i++)
            {
                // reuse the initial sai buffer
                if (i<blockizer.last_block_word_count)
                {
                    tx_buffer_last[i] = blockizer.current_block[i];
                }
                else
                {
//                     tx_buffer_last[i] = (0x7fff) * (-1*(i&0x1));
                    tx_buffer_last[i] = 0x0000;
                }
                
            }
            blockizer.current_block_size = blockizer.last_block_word_count;
            blockizer.current_block = tx_buffer_last;
            blockizer.is_last_block = true;
        }
    }
    else
    {
        blockizer.current_block_size = 0;
        ret = false;
    }

    return ret;
}

uint32_t bsp_cs35l36_write_reg(uint32_t addr, uint32_t val)
{
    uint32_t ret = BSP_STATUS_OK;

    i2c_write_buffer[0] = GET_BYTE_FROM_WORD(addr, 3);
    i2c_write_buffer[1] = GET_BYTE_FROM_WORD(addr, 2);
    i2c_write_buffer[2] = GET_BYTE_FROM_WORD(addr, 1);
    i2c_write_buffer[3] = GET_BYTE_FROM_WORD(addr, 0);
    i2c_write_buffer[4] = GET_BYTE_FROM_WORD(val, 3);
    i2c_write_buffer[5] = GET_BYTE_FROM_WORD(val, 2);
    i2c_write_buffer[6] = GET_BYTE_FROM_WORD(val, 1);
    i2c_write_buffer[7] = GET_BYTE_FROM_WORD(val, 0);

    // Currently only I2C transactions are supported
    ret = bsp_i2c_write(BSP_HAPTIC_DEV_ID, i2c_write_buffer, 8, NULL, NULL);

    return ret;
}

uint32_t bsp_haptic_initialize(void)
{
    uint32_t ret = BSP_STATUS_OK;

    bsp_haptic_int_cb = NULL;
    bsp_haptic_int_cb_arg = NULL;

#ifndef USE_VALIDATION_SETUP
    /*
     * Configure TCA9539 to set Reset signals for AMPL and AMPR
     */
    uint8_t temp_buffer[4];

    // Configure I/O ports as inputs/outputs
    temp_buffer[0] = 0x06;
    temp_buffer[1] = 0x04;
    temp_buffer[2] = 0x83;
    HAL_I2C_Master_Seq_Transmit_IT(&i2c_drv_handle,
                                   BSP_TCA9539_I2C_ADDRESS,
                                   temp_buffer,
                                   3,
                                   I2C_FIRST_AND_LAST_FRAME);
    while (HAL_I2C_GetState(&i2c_drv_handle) != HAL_I2C_STATE_READY)
    {
    }

    // Enable AIF1 and AIF2 path from Lochnagar to Amps, keep R amp in reset, L amp ON
    temp_buffer[0] = 0x02;
    temp_buffer[1] = 0x9B;
    temp_buffer[2] = 0x00;
    HAL_I2C_Master_Seq_Transmit_IT(&i2c_drv_handle,
                                   BSP_TCA9539_I2C_ADDRESS,
                                   temp_buffer,
                                   3,
                                   I2C_FIRST_AND_LAST_FRAME);
    while (HAL_I2C_GetState(&i2c_drv_handle) != HAL_I2C_STATE_READY)
    {
    }

    /*
     * CS35L36 Device initialization
     */

    // Set CS35L36 Reset - active high
    bsp_set_gpio(BSP_GPIO_ID_HAPTIC_RESET, BSP_GPIO_HIGH);

    /* Delay 1ms - CS35L36 T_RLPW */
    HAL_Delay(2);

    // Release CS35L36 Reset - active high
    bsp_set_gpio(BSP_GPIO_ID_HAPTIC_RESET, BSP_GPIO_LOW);

    /* Delay 1ms - CS35L36 T_RLPW */
    HAL_Delay(2);

    // Step 1.5 Unlock test page
    bsp_cs35l36_write_reg(0x00000020, 0x00005555);
    bsp_cs35l36_write_reg(0x00000020, 0x0000AAAA);

    // Delay 200 ms
    HAL_Delay(100);
    HAL_Delay(100);

    // Step 5: Download JUMP Instruction into RAM.The driver writes a LJMP command at address 0x800
    bsp_cs35l36_write_reg(0x00000C00, 0x00000000); // Release PAC_RESET: Write MCU_CTRL_0 2’b0
    bsp_cs35l36_write_reg(0x00000C08, 0x00000001); // Set PAC_MEM_ACCESS: Write MCU_CTRL_2 0’b1
    bsp_cs35l36_write_reg(0x00E02800, 0x00DD0102); // ROM 0.9 Ver: Jump Instruction to PMEM
    bsp_cs35l36_write_reg(0x00000C08, 0x00000000); // Clear PAC_MEM_ACCESS: Write MCU_CTRL_2 0’b0

    // Step 6: Enable PAC to execute code
    bsp_cs35l36_write_reg(0x00000C00, 0x00000001); // Set PAC_EN: Write MCU_CTRL_0 0’b1

    // Step 7 requires polling status bits. Wait is used in this script instead
    HAL_Delay(100);
    HAL_Delay(100);

    // Step 10: Disable PAC - Set PAC_EN :  Write MCU_CTRL_0[0] = 1’b0
    bsp_cs35l36_write_reg(0x00000C00, 0x00000000);

    // Lock test page
    bsp_cs35l36_write_reg(0x00000020, 0x0000CCCC);
    bsp_cs35l36_write_reg(0x00000020, 0x00003333);

    /*
     * CS35L36A setup
     */
    /*
     * REFCLK_INPUT - 0x2C04 = 0x00000370
     *
     * b31:17 - Reserved - 0x0
     * b16 - PLL_FORCE_EN = 0b0
     * b15:12 - Reserved = 0b0000
     * b11 - PLL_OPEN_LOOP = 0b0
     * b10:5 - PLL_REFLCLK_FREQ = 0b011011 (1536000 Hz)
     * b4 - PLL_REFCLK_EN = 0b1
     * b3 - Reserved = 0b0
     * b2:0 - PLL_REFCLK_SEL = 0b000 (SCLK_INPUT)
     */
    bsp_cs35l36_write_reg(0x00002C04, 0x00000370);

    // Step 6 Configure startup calibration registers
    bsp_cs35l36_write_reg(0x00000020, 0x00005555);
    bsp_cs35l36_write_reg(0x00000020, 0x0000AAAA);
    bsp_cs35l36_write_reg(0x00007064, 0x0929A800);
    bsp_cs35l36_write_reg(0x00007850, 0x00002FA9);
    bsp_cs35l36_write_reg(0x00007854, 0x0003F1D5);
    bsp_cs35l36_write_reg(0x00007858, 0x0003F5E3);
    bsp_cs35l36_write_reg(0x0000785C, 0x00001137);
    bsp_cs35l36_write_reg(0x00007860, 0x0001A7A5);
    bsp_cs35l36_write_reg(0x00007864, 0x0002F16A);
    bsp_cs35l36_write_reg(0x00007868, 0x00003E21);
    bsp_cs35l36_write_reg(0x00007848, 0x00000001);

    /*
     * Errata writes
     */

    /*
     * WOM-1645 Group/path delay variation from power-up to power-up - also DS section 4.13.9
     * For F REFCLK = 3.072MHz
     */
    bsp_cs35l36_write_reg(0x00002D10, 0x002C01C);

    /*
     * WOM-2393 Wrong default setting for int1 internal supply generator causes AMP SPK DC offset at high Vboost.
     * Also decreases THD @ 1W by >3dB
     */
    bsp_cs35l36_write_reg(0x00007418, 0x909001C8);

    /*
     * WOM-2980 L36 need to limit the VBST to 10V
     */
    bsp_cs35l36_write_reg(0x00003800, 0x00000097); // boost_voltage_1(3800H):  0097  BST_CTL=VBST = 10 V (non-extended), VBST = 10.05 V (extended range)
    bsp_cs35l36_write_reg(0x00003804, 0x00000005); // boost_voltage_2(3804H):  0004  BST_CTL_LIM_EN=Maximum Class H BST_CTL generation is limited by BST_CTL configuration, BST_CTL_SEL=Control port BST_CTL register value
    bsp_cs35l36_write_reg(0x00003830, 0x00000110); // boost_vbst_ovp(3830H):   0110  BST_OVP_EN=(Default) Overvoltage protection enabled, BST_OVP_THLD=11V
    bsp_cs35l36_write_reg(0x0000394C, 0x028664B7); // boost_test_ana2(394CH):  28664B7

    /*
     * WOM-2951 Update The default K1,K2 Value for Boost
     */
    //bsp_cs35l36_write_reg(0x00003810, 0x00003C3C); // boost_coeffs(3810H):     3C3C  BST_K2=3C, BST_K1=3C

    bsp_cs35l36_write_reg(0x00000020, 0x0000CCCC);
    bsp_cs35l36_write_reg(0x00000020, 0x00003333);


    bsp_cs35l36_write_reg(0x00002C0C, 0x00000003); // GLOBAL_FS: 48kHz
    bsp_cs35l36_write_reg(0x00004808, 0x00000002); // ASP_FMT: I2S
    /*
     * Can add 0.375dB gain in PCM volume control to get closer gain to required
     * INTP_AMP_CTRL(6000H):    8030  AMP_HPF_PCM_EN=(Default) Enabled,
     * AMP_INV_PCM=(Default) PCM audio not inverted, AMP_VOL_PCM=0.75dB, AMP_RAMP_PCM=No Ramp
     */
    bsp_cs35l36_write_reg(0x00006000, CS35L36_AMP_CTRL_DEFAULT);
    bsp_cs35l36_write_reg(0x0000483C, 0x00010003); // TX1 (VMON) and TX2 (IMON) enabled, RX1 Enabled
    /*
     * Setting Rx, Tx packet width
     * Rx Slot width: 16, data width: 16; Tx Slot width: 16, data width: 16
     */
    bsp_cs35l36_write_reg(0x00004818, 0x00100010);

    // Set ASP_TX1_SEL to IMON and ASP_TX2_SEL to VMON
    bsp_cs35l36_write_reg(0x00004C20, 0x00000019);
    bsp_cs35l36_write_reg(0x00004C24, 0x00000018);
#endif

    // Initialize Haptic Waveform Log
    for (int i = 0; i < HAPTIC_TABLE_TOTAL; i++)
    {
        haptic_waveform_logs[i].table = &(haptic_tables[i]);
        haptic_waveform_logs[i].f0_measured = haptic_tables[i].f0_design;
        haptic_waveform_logs[i].f0_used = haptic_tables[i].f0_design;
        haptic_waveform_logs[i].has_been_played = false;
    }

    current_haptic_waveform_log = &(haptic_waveform_logs[0]);

    // Now initialize algorithm
    //driver_init(48000, 16);
    //driver_open(0);

    return ret;
}

uint32_t bsp_haptic_power_up(void)
{
#ifndef USE_VALIDATION_SETUP
    bsp_cs35l36_write_reg(0x2014, 0x0001); // Power Control 1(2014H):  0001  GLOBAL_EN=1
#endif

    return BSP_STATUS_OK;
}

uint32_t bsp_haptic_power_down(void)
{
#ifndef USE_VALIDATION_SETUP
    bsp_cs35l36_write_reg(0x2014, 0x0000); // Power Control 1(2014H):  0001  GLOBAL_EN=0
#endif

    return BSP_STATUS_FAIL;
}

uint32_t bsp_haptic_process(void)
{
    uint32_t ret = BSP_STATUS_OK;

    __disable_irq();
    if (vi_block_received)
    {
        vi_block_received = false;

        int16_t *tx_buffer = sai_tx_buffer_1;
        uint32_t *rx_buffer = sai_rx_buffer_1;

        is_buffer_0 = !is_buffer_0;

        if (is_buffer_0)
        {
            tx_buffer = sai_tx_buffer_0;
            rx_buffer = sai_rx_buffer_0;
        }

        if (bsp_haptic_is_calibrating)
        {
            MCPS_WRAP(cl_play_calibrate, rx_buffer, blockizer.current_block_size, tx_buffer)

            // If finished calibrating, set block_size to 0 so SAI RX ISR will terminate calibration process
            if (algo_api_ret.cl_play_calibrate == DIAGNOSTIC_COMPLETED)
            {
                blockizer.current_block_size = 0;
                bsp_haptic_is_calibrating = false;
            }
        }
        else
        {
            if (bsp_blockizer_advance())
            {
                BSP_DEBUG_PIN_TOGGLE;
                MCPS_WRAP(cl_play_effect,
                        blockizer.current_block,
                        playback_volume,
                        blockizer.output_block_size,
                        rx_buffer,
                        tx_buffer)
                BSP_DEBUG_PIN_TOGGLE;
            }
        }
    }

    __enable_irq();

    return ret;
}

uint32_t bsp_haptic_mute(bool is_mute)
{
#ifndef USE_VALIDATION_SETUP
    if (is_mute)
    {
        bsp_cs35l36_write_reg(0x6000, CS35L36_AMP_CTRL_MUTE);
    }
    else
    {
        bsp_cs35l36_write_reg(0x6000, CS35L36_AMP_CTRL_DEFAULT);
    }
#endif

    return BSP_STATUS_FAIL;
}

uint32_t bsp_haptic_calibrate(void)
{
    MCPS_WRAP(algo_init, LRA_SPEC_F0, LRA_SPEC_Q, LRA_SPEC_REDC);

    // Initialize SAI RX buffers
    for (int i = 0; i < SAI_RX_BUFFER_SIZE_WORDS; i++)
    {
        sai_rx_buffer_0[i] = 0;
        sai_rx_buffer_1[i] = 0;
    }

    bsp_haptic_is_calibrating = true;
    blockizer.current_block_size = SAI_TX_BUFFER_SIZE_HALFWORDS;

    MCPS_WRAP(cl_play_calibrate, sai_rx_buffer_0, blockizer.current_block_size, sai_tx_buffer_0)

    // If finished calibrating, set block_size to 0 so SAI RX ISR will terminate calibration process
    if (algo_api_ret.cl_play_calibrate == DIAGNOSTIC_COMPLETED)
    {
        blockizer.current_block_size = 0;
        bsp_haptic_is_calibrating = false;
    }

    vi_block_received = false;

    if (HAL_OK == HAL_SAI_Receive_DMA(&sai_rx_drv_handle, (uint8_t*) sai_rx_buffer_0, blockizer.current_block_size))
    {
        if (HAL_OK == HAL_SAI_Transmit_DMA(&sai_tx_drv_handle, (uint8_t*) sai_tx_buffer_0, blockizer.current_block_size))
        {
            is_buffer_0 = false;

            MCPS_WRAP(cl_play_calibrate, sai_rx_buffer_1, blockizer.current_block_size, sai_tx_buffer_1)
        }
    }

    bsp_set_gpio(BSP_GPIO_ID_LD3, BSP_GPIO_HIGH);

    return BSP_STATUS_OK;
}

uint32_t bsp_haptic_buzz_record(uint8_t content)
{
    // Initialize SAI RX buffers
    for (int i = 0; i < SAI_RX_BUFFER_SIZE_WORDS; i++)
    {
        sai_rx_buffer_0[i] = 0;
        sai_rx_buffer_1[i] = 0;
    }

    if (content == BSP_HAPTIC_BUZZ_PATTERN)
    {
        current_haptic_waveform_log = NULL;
        MCPS_WRAP(algo_init, LRA_SPEC_F0, LRA_SPEC_Q, LRA_SPEC_REDC);
        bsp_blockizer_blockize(playback_pattern_buffer,
                               PLAYBACK_PATTERN_BUFFER_SIZE_HALFWORDS,
                               SAI_TX_BUFFER_SIZE_HALFWORDS,
                               1);
    }
    else
    {
        switch (content)
        {
            case BSP_HAPTIC_BUZZ_100HZ:
                current_haptic_waveform_log = &(haptic_waveform_logs[HAPTIC_TABLE_100HZ]);
                break;

            default:
            case BSP_HAPTIC_BUZZ_CURRENT:
                break;
        }

        MCPS_WRAP(algo_init, current_haptic_waveform_log->table->f0_design, LRA_SPEC_Q, LRA_SPEC_REDC);

        if (f0_tuning_params_dirty == false)
        {
            cl_get_dynamic_f0_params(&dynamic_f0_tuning_params);
        }
        else 
        {
            cl_set_dynamic_f0_params(&dynamic_f0_tuning_params);
        }

        uint32_t playback_buffer_size_samples = 0;
        uint32_t f0_to_use;

        if (!(current_haptic_waveform_log->has_been_played))
        {
            f0_to_use = current_haptic_waveform_log->table->f0_design;
        }
        else
        {
            f0_to_use = current_haptic_waveform_log->f0_used;
        }

        MCPS_WRAP(cl_get_playback_len, &playback_buffer_size_samples,
                                      blockizer.output_block_size,
                                      f0_to_use,
                                      LRA_SPEC_REDC)

        bsp_blockizer_blockize((int16_t*) current_haptic_waveform_log->table->samples,
                               current_haptic_waveform_log->table->sample_total,
                               playback_buffer_size_samples,
                               current_haptic_waveform_log->table->repetitions);

    }

    BSP_DEBUG_PIN_TOGGLE;
    MCPS_WRAP(cl_play_effect,
            blockizer.current_block,
            playback_volume,
            blockizer.output_block_size,
            sai_rx_buffer_0,
            sai_tx_buffer_0)
    BSP_DEBUG_PIN_TOGGLE;

    vi_block_received = false;

    if (HAL_OK == HAL_SAI_Receive_DMA(&sai_rx_drv_handle, (uint8_t*) sai_rx_buffer_0, (blockizer.output_block_size * 2)))
    {
        if (HAL_OK == HAL_SAI_Transmit_DMA(&sai_tx_drv_handle, (uint8_t*) sai_tx_buffer_0, blockizer.output_block_size))
        {
            is_buffer_0 = false;

            if (bsp_blockizer_advance())
            {
                BSP_DEBUG_PIN_TOGGLE;
                MCPS_WRAP(cl_play_effect,
                        blockizer.current_block,
                        playback_volume,
                        blockizer.output_block_size,
                        sai_rx_buffer_1,
                        sai_tx_buffer_1)
                BSP_DEBUG_PIN_TOGGLE;
            }
        }
    }

    bsp_set_gpio(BSP_GPIO_ID_LD3, BSP_GPIO_HIGH);

    return BSP_STATUS_OK;
}

uint32_t bsp_haptic_buzz_stop(void)
{
    if (HAL_OK == HAL_SAI_DMAStop(&sai_tx_drv_handle))
    {
        if (bsp_haptic_is_calibrating)
        {
            bsp_haptic_is_calibrating = false;
        }
        else if (current_haptic_waveform_log != NULL)
        {
            current_haptic_waveform_log->has_been_played = true;
        }

        return BSP_STATUS_OK;
    }
    else
    {
        return BSP_STATUS_FAIL;
    }
}

uint32_t bsp_cal_param_read(void)
{
    uint16_t *calibration_parameters_ptr = (uint16_t *) &calibration_parameters;

    for (uint8_t i = 0; i < NB_OF_VAR; i++)
    {
        if (EE_ReadVariable(VirtAddVarTab[i], calibration_parameters_ptr) != HAL_OK)
        {
            Error_Handler();
        }

        calibration_parameters_ptr++;
    }

    return BSP_STATUS_OK;
}

uint32_t bsp_cal_param_save(void)
{
    MCPS_WRAP(cl_get_calibrate_result, &calibration_parameters)

    uint16_t *calibration_parameters_ptr = (uint16_t *) &calibration_parameters;
    for (uint8_t i = 0; i < NB_OF_VAR; i++)
    {
        if(EE_WriteVariable(VirtAddVarTab[i], *calibration_parameters_ptr) != HAL_OK)
        {
            Error_Handler();
        }

        calibration_parameters_ptr++;
    }

    algo_deinit();

    bsp_toggle_gpio(BSP_GPIO_ID_LD1);

    return BSP_STATUS_OK;
}

uint32_t bsp_haptic_udpate_f0(void)
{
    if (current_haptic_waveform_log != NULL)
    {
        MCPS_WRAP(cl_get_dynamic_f0, &(current_haptic_waveform_log->f0_measured))
        if (current_haptic_waveform_log->f0_measured)
        {
            // Only update f0_used if we actually detected a new/different f0_measured
            current_haptic_waveform_log->f0_used = current_haptic_waveform_log->f0_measured;
        }
    }

    algo_deinit();

    return BSP_STATUS_OK;
}

uint32_t bsp_initialize(bsp_app_callback_t cb, void *cb_arg)
{
    app_cb = cb;
    app_cb_arg = cb_arg;

    /* Enable the CPU Cache */
    CPU_CACHE_Enable();

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Unlock the Flash Program Erase controller */
    HAL_FLASH_Unlock();

    /* Get FLASH bank confiuration (single or dual) */
    HAL_FLASHEx_OBGetConfig(&OBInit);
    if ((OBInit.USERConfig & OB_NDBANK_SINGLE_BANK) != OB_NDBANK_SINGLE_BANK)
    {
        Error_Handler();
    }

    /* EEPROM Init */
    if (EE_Init() != EE_OK)
    {
        Error_Handler();
    }

    // If EEPROM Read Fails (Virtual Address Table is empty), initialize Virtual EEPROM
    uint16_t temp;
    if (EE_ReadVariable(VirtAddVarTab[0], &temp) != HAL_OK)
    {
        calibration_parameters.f0_out = 0xDEADBEEF;
        calibration_parameters.q_out = 0xDEADBEEF;
        calibration_parameters.redc_out = 0xDEADBEEF;

        uint16_t *calibration_parameters_ptr = (uint16_t *) &calibration_parameters;
        for (uint8_t i = 0; i < NB_OF_VAR; i++)
        {
            if (EE_WriteVariable(VirtAddVarTab[i], *calibration_parameters_ptr) != HAL_OK)
            {
                Error_Handler();
            }

            calibration_parameters_ptr++;
        }
    }
    else
    {
        // Read calibration parameters from eeprom
        bsp_cal_param_read();
    }

    // Since EEPROM is just initialized, fake out main to force calibration
    // If calibration parameters don't make sense, fake out main to force calibration
    if ((calibration_parameters.f0_out == 0xDEADBEEF) || \
        (calibration_parameters.q_out == 0xDEADBEEF) || \
        (calibration_parameters.redc_out == 0xDEADBEEF))
    {
        bsp_pb_por_is_pressed = true;
    }

    bsp_init_cycle_counter();

    /* Delay 3 seconds to allow LN2 to finish all I2C traffic */
    for (int i = 0; i < 30; i++)
    {
        HAL_Delay(100);
    }

    /* Initialize all peripheral drivers */
    Timer_Init();
    I2C_Init();
    SAI_Init();

    // Initialize playback buffer
    for (int i = 0; i < PLAYBACK_PATTERN_BUFFER_SIZE_HALFWORDS; i++)
    {
        //playback_pattern_buffer[i] = PLAYBACK_PATTERN_BUFFER_DEFAULT_VALUE;
        playback_pattern_buffer[i] = i;
    }

    // Initialize double buffer state
    for (int i = 0; i < SAI_TX_BUFFER_SIZE_HALFWORDS; i++)
    {
        sai_tx_buffer_0[i] = SAI_TX_BUFFER_0_DEFAULT_VALUE;
        sai_tx_buffer_1[i] = SAI_TX_BUFFER_1_DEFAULT_VALUE;
    }
    for (int i = 0; i < SAI_RX_BUFFER_SIZE_WORDS; i++)
    {
        sai_rx_buffer_0[i] = 0;
        sai_rx_buffer_1[i] = 0;
    }
    is_buffer_0 = true;

    bsp_blockizer_initialize(SAI_TX_BUFFER_SIZE_HALFWORDS);

    bsp_timer_cb = NULL;
    bsp_timer_cb_arg = NULL;
    bsp_timer_has_started = false;
    bsp_i2c_done_cb = NULL;
    bsp_i2c_done_cb_arg = NULL;
    bsp_i2c_current_transaction_type = BSP_I2C_TRANSACTION_TYPE_INVALID;

    bsp_pb_pressed_flag = false;

    algo_api_max_cycle_count.algo_init = 0;

    bsp_haptic_initialize();

    return BSP_STATUS_OK;
}

uint32_t bsp_pb_debouncer(void)
{
    for (uint8_t i = 0; i < (sizeof(bsp_pb_debounce_data)/sizeof(bsp_pb_debounce_data_t)); i++)
    {
        if (bsp_pb_debounce_data[i].triggered)
        {
            uint32_t current_tick = HAL_GetTick();
            uint32_t last_tick = bsp_pb_debounce_data[i].debounce_counter;
            uint32_t diff;

            // Get absolute value difference
            if (current_tick < last_tick)
            {
                diff = (current_tick) + (0xFFFFFFFF - last_tick);
            }
            else
            {
                diff = current_tick - last_tick;
            }

            if (diff > BSP_PB_DEBOUNCE_TOTAL_TICKS)
            {
                bsp_pb_debounce_data[i].triggered = false;

                uint8_t gpio_state;
                bsp_get_gpio(bsp_pb_debounce_data[i].bsp_gpio_id, &gpio_state);
                if (gpio_state == bsp_pb_debounce_data[i].active_level)
                {
                    // Currently for every push button, we just want to set this flag
                    bsp_pb_pressed_flag = true;
                }
            }
        }
    }

    return BSP_STATUS_OK;
}

uint32_t bsp_i2c_slave_register_access(uint32_t address, uint32_t *value, bool is_write)
{
    uint32_t temp_data = 0;
    uint32_t *temp_data_ptr = &temp_data;
    bool is_writeable = false;
    bool is_f0_debug = false;

    switch (address)
    {
        case 0x0:
            if (is_write)
            {
                if (*value & BSP_I2C_TRIGGER_REGISTER_BUZZ_MASK)
                {
                    bsp_i2c_triggered_buzz = true;
                }
                else if (*value & BSP_I2C_TRIGGER_REGISTER_CALIBRATE_MASK)
                {
                    bsp_i2c_triggered_calibrate = true;
                }
            }
            break;

        case 0x1:
            temp_data = HAPTIC_TABLE_TOTAL;
            break;

        case 0x2:
            temp_data_ptr = &bsp_haptic_trigger_index;

            if ((is_write) && (*value < HAPTIC_TABLE_TOTAL))
            {
                current_haptic_waveform_log = &(haptic_waveform_logs[*value]);
                is_writeable = true;
            }

            break;

        case 0x3:
            if (current_haptic_waveform_log != NULL)
            {
                temp_data_ptr = (uint32_t *) &(current_haptic_waveform_log->table->f0_design);
            }
            break;

        case 0x4:
            if (current_haptic_waveform_log != NULL)
            {
                temp_data_ptr = &(current_haptic_waveform_log->f0_measured);
            }
            break;

        case 0x5:
            if (current_haptic_waveform_log != NULL)
            {
                temp_data_ptr = &(current_haptic_waveform_log->f0_used);
            }
            is_writeable = true;
            break;

        case 0x6:
            temp_data_ptr = (uint32_t *) &(calibration_parameters.f0_out);
            break;

        case 0x7:
            temp_data_ptr = (uint32_t *) &(calibration_parameters.redc_out);
            break;

        case 0x8:
            temp_data_ptr = (uint32_t *) &(calibration_parameters.q_out);
            break;

        case 0x9 ... 0xE:
            temp_data_ptr = &(algo_api_max_cycle_count.words[(address - 0x9)]);
            break;

        case 0xF ... 0x14:
            temp_data_ptr = &(algo_api_ret.words[(address - 0xF)]);
            break;
        case 0x15:
            temp_data_ptr = &(playback_volume);
            is_writeable = true;
            break;
        case 0x16:
            temp_data_ptr = &(dynamic_f0_tuning_params.wave_out_threshold);
            is_writeable = true;
            is_f0_debug = true;
            break;
        case 0x17:
            temp_data_ptr = &(dynamic_f0_tuning_params.vmon_threshold);
            is_writeable = true;
            is_f0_debug = true;
            break;
        case 0x18:
            temp_data_ptr = &(dynamic_f0_tuning_params.minimum_viable_imon_p2p);
            is_writeable = true;
            is_f0_debug = true;
            break;
        case 0x19:
            temp_data_ptr = &(dynamic_f0_debug_info.max_imon_ringing_amp);
            is_f0_debug = true;
            break;
        case 0x1a:
            temp_data_ptr = &(dynamic_f0_debug_info.status);
            is_f0_debug = true;
            break;
        default:
            break;
    }

    if (!is_write)
    {
        if (is_f0_debug)
        {
            cl_get_dynamic_f0_params(&dynamic_f0_tuning_params);
            cl_get_dynamic_f0_debug_info(&dynamic_f0_debug_info);
        }
        *value = *temp_data_ptr;
    }
    else if (is_writeable)
    {
        *temp_data_ptr = *value;
        if (is_f0_debug)
        {
            cl_set_dynamic_f0_params(&dynamic_f0_tuning_params);
            f0_tuning_params_dirty = true;
        }
    }


    return BSP_STATUS_OK;
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}
