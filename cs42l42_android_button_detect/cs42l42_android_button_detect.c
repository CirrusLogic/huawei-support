////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2016, 2020 Cirrus Logic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
////////////////////////////////////////////////////////////////////////////////
//
// Revision 1.0.0 - 5/11/2016
//      -Initial Release
// Revision 1.1.0 - 10/12/2020
//      -Clean up to provide as generic reference code
//
////////////////////////////////////////////////////////////////////////////////
#include <CS42L42_Android_Button_Detect.h>
#include <CS42L42_registers.h> // CS42L42 register defintions

//
// CS42L42 EVENTs
//
typedef enum
{
    NO_EVENT = 0,
    RESET_EVENT,
    TS_PLUG_EVENT,
    TS_UNPLUG_EVENT,
    RS_PLUG_EVENT,
    RS_UNPLUG_EVENT,
    HEADSET_EVENT,
    HEADPHONE_EVENT,
    INVALID_EVENT,
    BUTTON_RELEASE_EVENT,
    BUTTON_A_PRESS_EVENT,
    BUTTON_B_PRESS_EVENT,
    BUTTON_C_PRESS_EVENT,
    BUTTON_D_PRESS_EVENT
} EVENT;

//
// Global variables
//

// This flag indicates the status of the CS4242L42_INTB and must be updated only
// by CS42L42_INTB_ISR()
volatile unsigned char cs42l42_intb_flag = DEASSERTED;

//
// Function Prototypes
//
static void ResetCodec(void);
static void InitializePlugDetect(void);
static void InitiateHSTypeDetection(void);
static void ClockConfig(void);
static void PowerUpCodec(STATE L42_state, unsigned char full_scale_vol);
static void PowerDownCodec(unsigned char full_scale_vol);
static unsigned char RunLoadDetect(void);
static void InitializeAndroidButtonDetect(void);
static EVENT CS42L42_INTB_handler (STATE L42_state);
static EVENT CheckForPlugEvent(void);
static EVENT CheckForHSdetectEvent(STATE L42_state);
static EVENT CheckForAndroidButtonEvent(void);

//
// ISR called on rising/falling edge of CS42L42_INTB and used to wake up MCU
// from IDLE
//
void CS42L42_INTB_ISR(void)
{
    if (CS42L42_INTB == 0)
        cs42l42_intb_flag = ASSERTED;
    else
        cs42l42_intb_flag = DEASSERTED;
}

//
// CS42L42 state machine
//
STATE CS42L42_state_machine(void)
{
    EVENT CS42L42_event;
    static unsigned char full_scale_vol = FULL_SCALE_VOL_0dB;
    static STATE CS42L42_state = UNPLUGGED;

    CS42L42_event = CS42L42_INTB_handler(CS42L42_state); // Handle CS42L42_INTB

    switch (CS42L42_event)
    {
        case RESET_EVENT:
            CS42L42_state = UNPLUGGED;
            ResetCodec();
            InitializePlugDetect();
            break;
        case TS_UNPLUG_EVENT:
        case RS_UNPLUG_EVENT:
            if (CS42L42_state != UNPLUGGED)
            {
                CS42L42_state = UNPLUGGED;
                PowerDownCodec(full_scale_vol);
                ResetCodec();
                InitializePlugDetect();
            }
            break;
        case TS_PLUG_EVENT:
            CS42L42_state = HS_PLUGGED;
            InitiateHSTypeDetection();
            break;
        case RS_PLUG_EVENT:
            CS42L42_state = OPTICAL; // CDB42L42 RING_SENSE is connected to S/PDIF TX connector
            ClockConfig();
            PowerUpCodec(CS42L42_state, full_scale_vol);
            break;
        case HEADSET_EVENT:
            CS42L42_state = HEADSET;
            ClockConfig();
            full_scale_vol = RunLoadDetect();
            PowerUpCodec(CS42L42_state, full_scale_vol);
            InitializeAndroidButtonDetect();
            break;
        case HEADPHONE_EVENT:
            CS42L42_state = HEADPHONE;
            ClockConfig();
            full_scale_vol = RunLoadDetect();
            PowerUpCodec(CS42L42_state, full_scale_vol);
            break;
        case INVALID_EVENT:
            CS42L42_state = INVALID;
            break;
        case BUTTON_RELEASE_EVENT:
            CS42L42_state = BUTTON_RELEASED;
            break;
        case BUTTON_A_PRESS_EVENT:
            CS42L42_state = BUTTON_A;
            break;
        case BUTTON_B_PRESS_EVENT:
            CS42L42_state = BUTTON_B;
            break;
        case BUTTON_C_PRESS_EVENT:
            CS42L42_state = BUTTON_C;
            break;
        case BUTTON_D_PRESS_EVENT:
            CS42L42_state = BUTTON_D;
            break;
        case NO_EVENT:
        default:
            break;
    }
    return CS42L42_state;
}

//
// Handler for CS42L42_INTB
//
static EVENT CS42L42_INTB_handler (STATE L42_state)
{
    // Initial EVENT is RESET, remember the EVENTS between calls
    static EVENT L42_event = RESET_EVENT;

    // Continue only if CS42L42_INTB is asserted (low)
    if (cs42l42_intb_flag == DEASSERTED)
        return L42_event;
    else
        L42_event = CheckForPlugEvent(); // Plug/Unplug

    // Continue only if there is NO_EVENT to service
    if (L42_event != NO_EVENT)
        return L42_event;
    else
        L42_event = CheckForHSdetectEvent(L42_state); // Headset detection done

    // Continue only if there is NO_EVENT to service
    if (L42_event != NO_EVENT)
        return L42_event;
    else
        L42_event = CheckForAndroidButtonEvent(); // Button press

    return L42_event;
}


static void ResetCodec(void)
{
    CS42L42_RSTB    = 0; // Put CODEC into reset

    Delay_ms(1); // wait for 1 ms

    CS42L42_RSTB    = 1; // Release CODEC from reset

    Delay_ms(3); // Wait for CODEC to boot up (2.5ms required)
}

//
// Minimum configuration to get interrupt upon plug/unplug
//
static void InitializePlugDetect(void)
{
    unsigned char reg_val;

    // MIC_DETECT_CTRL_1, 0x1B75
    reg_val = LATCH_TO_VP | HS_DETECT_LEVEL_DEFAULT;
    RegWrite (MIC_DETECT_CTRL_1, reg_val, CS42L42_ADDR);

    // TIP_SENSE_CTRL_1, 0x1113
    reg_val = TS_INV | TS_FALL_DBNCE_TIME_0MS | TS_RISE_DBNCE_TIME_250MS;
    RegWrite (TIP_SENSE_CTRL_1, reg_val, CS42L42_ADDR);

    // TIP_SENSE_CTRL_2, 0x1B73
    reg_val = TIP_SENSE_CTRL_SHORT_DET | TIP_SENSE_INV | TIP_SENSE_DEBOUNCE_500MS;
    RegWrite (TIP_SENSE_CTRL_2, reg_val, CS42L42_ADDR);

    // RNG_SENSE_CTRL_3, 0x1112
    reg_val = RS_INV | RS_PU_EN | RS_FALL_DBNCE_TIME_0MS | RS_RISE_DBNCE_TIME_500MS;
    RegWrite (RNG_SENSE_CTRL_3, reg_val, CS42L42_ADDR);

    // POWER_DOWN_CTRL_3, 0x1103
    reg_val = RING_SENSE_PUP;
    RegWrite (POWER_DOWN_CTRL_3, reg_val, CS42L42_ADDR);

    // TIP_RNG_SENS_PLUG_INT_MASK, 0x1320
    reg_val = M_TS_UNPLUG | M_RS_UNPLUG & ~M_TS_PLUG & ~M_RS_PLUG;
    RegWrite (TIP_RNG_SENS_INT_MASK, reg_val, CS42L42_ADDR);

    // CS42L42 will now generate an interrupt upon Plug/Unplug of TIP_SENSE
    // and RING_SENSE. The interrupt will be serviced by CS42L42_INTB_handler
    //function.
}

static void InitiateHSTypeDetection(void)
{
    unsigned char reg_value;

    // HEADSET_CLAMP_DISABLE, 0x1129
    reg_value = HS_CLAMP_DISABLE;
    RegWrite (HEADSET_CLAMP_DISABLE, reg_value, CS42L42_ADDR);

    // POWER_DOWN_CTRL_1, 0x1101
    reg_value = ASP_DAO_PDN | ASP_DAI_PDN | MIXER_PDN | EQ_PDN | HP_PDN | ADC_PDN | PDN_CTRL_1_RESERVED & ~PDN_ALL;
    RegWrite (POWER_DOWN_CTRL_1, reg_value, CS42L42_ADDR);

    // DAC_CTRL_2, 0x1F06
    reg_value = HPOUT_PULLDOWN_NONE | HPOUT_CLAMP_DISABLED | DAC_HPF_EN;
    RegWrite (DAC_CTRL_2>>8, DAC_CTRL_2, reg_value, CS42L42_ADDR);

    // HEADSET_DETECT_CTRL_2, 0x1120
    reg_value = HSDET_CTRL_AUTO_DISABLED | HSDET_SET_AHJ | HSBIAS_REF_HSX | HSDET_AUTO_TIME_20US;
    RegWrite (HEADSET_DETECT_CTRL_2, reg_value, CS42L42_ADDR);

    // HEADSET_BIAS_CTRL, 0x1C03
    reg_value = HSBIAS_CAPLESS_EN | HSBIAS_CTRL_RESERVED | HSBIAS_RAMP_FAST;
    RegWrite (HEADSET_BIAS_CTRL, reg_value, CS42L42_ADDR);

    // MISC_DETECT_CTRL, 0x1B74
    reg_value = DETECT_MODE_NORMAL | HSBIAS_CTRL_2p7V | PDN_MIC_LVL_DETECT;
    RegWrite (MISC_DETECT_CTRL, reg_value, CS42L42_ADDR);

    Delay_ms(24); // Wait for HSBIAS to ramp up

    // HEADSET_DETECT_CTRL_2, 0x1120
    reg_value = HSDET_CTRL_AUTO_DISABLED | HSDET_SET_AHJ | HSBIAS_REF_HSX | HSDET_AUTO_TIME_20US;
    RegWrite (HEADSET_DETECT_CTRL_2, reg_value, CS42L42_ADDR);

    // CODEC_INT_MASK, 0x131B
    reg_value = M_PDN_DONE & ~M_HSDET_AUTO_DONE;
    RegWrite (CODEC_INT_MASK, reg_value, CS42L42_ADDR);

    Delay_us(100); // Wait 100 us (datasheet)

    // HEADSET_DETECT_CTRL_1, 0x111F
    reg_value = HSDET_COMP2_LVL_2V | HSDET_COMP1_LVL_1V;
    RegWrite (HEADSET_DETECT_CTRL_1, reg_value, CS42L42_ADDR);

    // HEADSET_DETECT_CTRL_2, 0x1120
    reg_value = HSDET_CTRL_AUTO_ACTIVE | HSDET_SET_AHJ | HSBIAS_REF_HSX_REF | HSDET_AUTO_TIME_20US;
    RegWrite (HEADSET_DETECT_CTRL_2, reg_value, CS42L42_ADDR);

    // CS42L42 will now generate an interrupt when Headset Type Detect is
    // completed. The interrupt will be serviced by CS42L42_INTB_handler
    // function.
}

static void ClockConfig(void)
{
    unsigned char reg_value;

    // PLL_LOCK_INT_STAT, 0x130E
    RegRead (PLL_LOCK_INT_STAT, &reg_value, CS42L42_ADDR);

    if (reg_value & PLL_LOCK)
        return; // clocking already configured

    // MCLK_SOURCE_SELECT, 0x1201
    reg_value = MCLKDIV_1 | MCLK_SRC_SEL_PLL;
    RegWrite (MCLK_SOURCE_SELECT, reg_value, CS42L42_ADDR);

    // PLL_DIVIDE_CONFIG_1, 0x120C
    reg_value = SCLK_PREDIV_1;
    RegWrite (PLL_DIVIDE_CONFIG_1, reg_value, CS42L42_ADDR);

    // PLL_DIVISION_INT, 0x1505
    reg_value = PLL_DIV_INT_64;
    RegWrite (PLL_DIVISION_INT, reg_value, CS42L42_ADDR);

    // PLL_DIV_FRAC_B0, 0x1502
    reg_value = PLL_DIV_FRAC_B0_0;
    RegWrite (PLL_DIV_FRAC_B0, reg_value, CS42L42_ADDR);

    // PLL_DIV_FRAC_B1, 0x1503
    reg_value = PLL_DIV_FRAC_B1_0;
    RegWrite (PLL_DIV_FRAC_B1, reg_value, CS42L42_ADDR);

    // PLL_DIV_FRAC_B2, 0x1504
    reg_value = PLL_DIV_FRAC_B2_0;
    RegWrite (PLL_DIV_FRAC_B2, reg_value, CS42L42_ADDR);

    // PLL_CTRL_4, 0x151B
    reg_value = PLL_MODE_3;
    RegWrite (PLL_CTRL_4, reg_value, CS42L42_ADDR);

    // PLL_CTRL_3, 0x1508
    reg_value = PLL_DIVOUT_16;
    RegWrite (PLL_CTRL_3, reg_value, CS42L42_ADDR);

    // PLL_CAL_RATIO, 0x150A
    reg_value = PLL_CAL_RATIO_128;
    RegWrite (PLL_CAL_RATIO, reg_value, CS42L42_ADDR);

    // PLL_CTRL_1, 0x1501
    reg_value = PLL_START;
    RegWrite (PLL_CTRL_1, reg_value, CS42L42_ADDR);

    // CS42L42 will now lock its PLL to SCLK

    do {
        // PLL_LOCK_INT_STAT, 0x130E
        RegRead (PLL_LOCK_INT_STAT, &reg_value, CS42L42_ADDR);
    } while (!(reg_value & PLL_LOCK));

    // OSC_SW_CTRL, 0x1107
    reg_value = SCLK_PRESENT;
    RegWrite (OSC_SW_CTRL, reg_value, CS42L42_ADDR);

    Delay_us(150);  // Wait 150 us for RCO to power down
}

static void PowerUpCodec(STATE L42_state, unsigned char full_scale_vol)
{
    unsigned char reg_value;

    // MCLK_CTRL, 0x1009
    reg_value = INTERNAL_FS_MCLK_BY_256;
    RegWrite (MCLK_CTRL, reg_value, CS42L42_ADDR);

    if (L42_state == HEADSET)   // Config ASP TX if Headset is plugged in
    {
        // ASP_TX_CH_PH_RES, 0x2903
        reg_value = ~ASP_TX_CH1_AP & ASP_TX_CH2_AP | ASP_TX_CH2_RES_32BITS | ASP_TX_CH1_RES_32BITS;
        RegWrite (ASP_TX_CH_PH_RES, reg_value, CS42L42_ADDR);

        // ASP_TX_CH_EN, 0x2902
        reg_value = ASP_TX_CH2_EN | ASP_TX_CH1_EN;
        RegWrite (ASP_TX_CH_EN, reg_value, CS42L42_ADDR);

        // ASP_TX_SIZE_EN, 0x2901
        reg_value = ASP_TX_EN;
        RegWrite (ASP_TX_SIZE_EN, reg_value, CS42L42_ADDR);

        // ADC_CTRL_1, 0x1D01
        reg_value = ADC_DIG_BOOST;
        RegWrite (ADC_CTRL_1, reg_value, CS42L42_ADDR);

        // ADC_VOLUME, 0x1D03
        reg_value = ADC_VOL_12dB;
        RegWrite (ADC_VOLUME, reg_value, CS42L42_ADDR);

        // HSBIAS_SENS_HIZ_AUTOCTRL, 0x1B70
        reg_value = HSBIAS_SENSE_EN | AUTO_BIAS_HIZ | HSBIAS_SENS_TRIP_52uA;
        RegWrite (HSBIAS_SENS_HIZ_AUTOCTRL, reg_value, CS42L42_ADDR);
    }

    // ASP_RX_EN, 0x2A01
    reg_value = ASP_RX0_CH2_EN | ASP_RX0_CH1_EN;
    RegWrite (ASP_RX_EN, reg_value, CS42L42_ADDR);

    // ASP_RX_DAI0_CH1_PH_RES, 0x2A02
    reg_value = ~ASP_RX0_CH1_AP & ASP_RX0_CH1_RES_32BIT;
    RegWrite (ASP_RX_DAI0_CH1_PH_RES, reg_value, CS42L42_ADDR);

    // ASP_RX_DAI0_CH1_BS_MSB, 0x2A03
    reg_value = ASP_RX0_CH1_BIT_ST_MSB_0;
    RegWrite (ASP_RX_DAI0_CH1_BS_MSB, reg_value, CS42L42_ADDR);

    // ASP_RX_DAI0_CH1_BS_LSB, 0x2A04
    reg_value = ASP_RX0_CH1_BIT_ST_LSB_0;
    RegWrite (ASP_RX_DAI0_CH1_BS_LSB, reg_value, CS42L42_ADDR);

    // ASP_RX_DAI0_CH2_PH_RES, 0x2A05
    reg_value = ASP_RX0_CH2_AP | ASP_RX0_CH2_RES_32BIT;
    RegWrite (ASP_RX_DAI0_CH2_PH_RES, reg_value, CS42L42_ADDR);

    // ASP_RX_DAI0_CH2_BS_MSB, 0x2A06
    reg_value = ASP_RX0_CH2_BIT_ST_MSB_0;
    RegWrite (ASP_RX_DAI0_CH2_BS_MSB, reg_value, CS42L42_ADDR);

    // ASP_RX_DAI0_CH2_BS_LSB, 0x2A07
    reg_value = ASP_RX0_CH2_BIT_ST_LSB_0;
    RegWrite (ASP_RX_DAI0_CH2_BS_LSB, reg_value, CS42L42_ADDR);

    // HP_CONTROL, 0x2001
    reg_value = ~ANA_MUTE_B & ~ANA_MUTE_A & full_scale_vol | HP_CTRL_RESERVED;
    RegWrite (HP_CONTROL, reg_value, CS42L42_ADDR);

    // MIXER_CHA_VOL, 0x2301
    reg_value = MIXER_CHA_VOL_0DB;
    RegWrite (MIXER_CHA_VOL, reg_value, CS42L42_ADDR);

    // MIXER_ADC_VOL, 0x2302
    reg_value = MIXER_ADC_VOL_MUTE;
    RegWrite (MIXER_ADC_VOL, reg_value, CS42L42_ADDR);

    // MIXER_CHB_VOL, 0x2303
    reg_value = MIXER_CHB_VOL_0DB;
    RegWrite (MIXER_CHB_VOL, reg_value, CS42L42_ADDR);

    // FSYNC_PULSE_LOW_BYTE, 0x1203
    reg_value = FSYNC_PULSE_WIDTH_LB_32;    // default
    RegWrite (FSYNC_PULSE_LOW_BYTE, reg_value, CS42L42_ADDR);

    // FSYNC_PULSE_UPPER_BYTE, 0x1204
    reg_value = FSYNC_PULSE_WIDTH_UB_32;
    RegWrite (FSYNC_PULSE_UPPER_BYTE, reg_value, CS42L42_ADDR);

    // FSYNC_PERIOD_LOW_BYTE, 0x1205
    reg_value = FSYNC_PERIOD_LOW_BYTE_64;
    RegWrite (FSYNC_PERIOD_LOW_BYTE, reg_value, CS42L42_ADDR);

    // FSYNC_PERIOD_UPPER_BYTE, 0x1206
    reg_value = FSYNC_PERIOD_UPPER_BYTE_64;
    RegWrite (FSYNC_PERIOD_UPPER_BYTE, reg_value, CS42L42_ADDR);

    // ASP_FRAME_CONFIG, 0x1208
    reg_value = ASP_STP_0 | ASP_5050 | ASP_FSD_1;
    RegWrite (ASP_FRAME_CONFIG, reg_value, CS42L42_ADDR);

    // ASP_CLOCK_CONFIG_1, 0x1207
    reg_value = ASP_SCLK_EN | ASP_SCPOL_IN_ADC | ASP_SCPOL_IN_DAC;
    RegWrite (ASP_CLOCK_CONFIG_1, reg_value, CS42L42_ADDR);

    switch (L42_state)
    {
        default:
        case HEADSET:
            // POWER_DOWN_CTRL_1, 0x1101
            reg_value = EQ_PDN | PDN_CTRL_1_RESERVED & ~ASP_DAO_PDN & ~ASP_DAI_PDN & ~MIXER_PDN & ~HP_PDN & ~ADC_PDN & ~PDN_ALL;
            RegWrite (POWER_DOWN_CTRL_1, reg_value, CS42L42_ADDR);
            break;
        case HEADPHONE:
            // POWER_DOWN_CTRL_1, 0x1101
            reg_value = EQ_PDN | PDN_CTRL_1_RESERVED | ASP_DAO_PDN | ADC_PDN & ~ASP_DAI_PDN & ~MIXER_PDN & ~HP_PDN & ~PDN_ALL;
            RegWrite (POWER_DOWN_CTRL_1, reg_value, CS42L42_ADDR);
            break;
        case OPTICAL:
            // SPDIF_CLOCK_CONFIG, 0x1202
            reg_value = SPDIF_CLK_DIV_2 | SPDIF_LRCK_SRC_SEL_EXT;
            RegWrite (SPDIF_CLOCK_CONFIG, reg_value, CS42L42_ADDR);

            // SPDIF_CH_SEL, 0x2504
            reg_value = SPDIF_CHB_SEL_CH1 | SPDIF_CHA_SEL_CH0;
            RegWrite (SPDIF_CH_SEL, reg_value, CS42L42_ADDR);

            // POWER_DOWN_CTRL_1, 0x1101
            reg_value = EQ_PDN | PDN_CTRL_1_RESERVED | HP_PDN | ADC_PDN & ~ASP_DAO_PDN & ~ASP_DAI_PDN & ~MIXER_PDN & ~PDN_ALL;
            RegWrite (POWER_DOWN_CTRL_1, reg_value, CS42L42_ADDR);

            // SPDIF_CTRL_1, 0x2801
            reg_value = 0x00 & ~SPDIF_TX_PDN;
            RegWrite (SPDIF_CTRL_1, reg_value, CS42L42_ADDR);

            // SPDIF_CTRL_2, 0x2802
            reg_value = SPDIF_TX_DIGEN;
            RegWrite (SPDIF_CTRL_2, reg_value, CS42L42_ADDR);
            break;
    }
}

static void PowerDownCodec(unsigned char full_scale_vol)
{
    unsigned char reg_value;

    // OSC_SW_CTRL, 0x1107
    reg_value = 0x00 & ~SCLK_PRESENT;
    RegWrite (OSC_SW_CTRL, reg_value, CS42L42_ADDR);

    Delay_us(150); // Wait 150 us for RCO to power up

    // PLL_CTRL_1, 0x1501
    reg_value = 0x00 & ~PLL_START;
    RegWrite (PLL_CTRL_1>>8, PLL_CTRL_1, reg_value, CS42L42_ADDR);

    // MIXER_CHA_VOL, 0x2301
    reg_value = MIXER_CHA_VOL_MUTE;
    RegWrite (MIXER_CHA_VOL>>8, MIXER_CHA_VOL, reg_value, CS42L42_ADDR);

    // MIXER_ADC_VOL, 0x2302
    reg_value = MIXER_ADC_VOL_MUTE;
    RegWrite (MIXER_ADC_VOL, reg_value, CS42L42_ADDR);

    // MIXER_CHB_VOL, 0x2303
    reg_value = MIXER_CHB_VOL_MUTE;
    RegWrite (MIXER_CHB_VOL, reg_value, CS42L42_ADDR);

    // HP_CONTROL, 0x2001
    reg_value = ANA_MUTE_B | ANA_MUTE_A | full_scale_vol | HP_CTRL_RESERVED;
    RegWrite (HP_CONTROL, reg_value, CS42L42_ADDR);

    // ASP_RX_EN, 0x2A01
    reg_value = ASP_RX_DISABLE;
    RegWrite (ASP_RX_EN, reg_value, CS42L42_ADDR);

    // ASP_CLOCK_CONFIG_1, 0x1207
    reg_value = ASP_SCPOL_IN_ADC | ASP_SCPOL_IN_DAC;
    RegWrite (ASP_CLOCK_CONFIG_1, reg_value, CS42L42_ADDR);

    // POWER_DOWN_CTRL_1, 0x1101
    reg_value = ASP_DAO_PDN | ASP_DAI_PDN | MIXER_PDN | EQ_PDN | HP_PDN | ADC_PDN | PDN_CTRL_1_RESERVED | PDN_ALL;
    RegWrite (POWER_DOWN_CTRL_1, reg_value, CS42L42_ADDR);

    do {
        Delay_ms(10);

        // Read CODEC_INT_STAT, 0x1308
        RegRead (CODEC_INT_STAT, &reg_value, CS42L42_ADDR);
    } while (!(reg_value & PDN_DONE));
}


static unsigned char RunLoadDetect(void)
{
    unsigned char reg_value, hpout_load, full_scale_vol = FULL_SCALE_VOL_0dB;

    // POWER_DOWN_CTRL_1, 0x1101
    reg_value = ASP_DAO_PDN | ASP_DAI_PDN | MIXER_PDN | EQ_PDN | HP_PDN | ADC_PDN | PDN_CTRL_1_RESERVED & ~PDN_ALL;
    RegWrite (POWER_DOWN_CTRL_1, reg_value, CS42L42_ADDR);

    // HP_CONTROL, 0x2001
    reg_value = ANA_MUTE_B | ANA_MUTE_A | full_scale_vol | HP_CTRL_RESERVED;
    RegWrite (HP_CONTROL, reg_value, CS42L42_ADDR);

    // DAC_CTRL_2, 0x1F06
    reg_value = HPOUT_PULLDOWN_1K | HPOUT_CLAMP_DISABLED & ~DAC_HPF_EN; // disable HPF
    RegWrite (DAC_CTRL_2, reg_value, CS42L42_ADDR);

    // HEADSET_DETECT_CTRL_2, 0x1120
    reg_value = HSDET_CTRL_AUTO_DISABLED | HSDET_SET_AHJ | HSBIAS_REF_HSX | HSDET_AUTO_TIME_20US;
    RegWrite (HEADSET_DETECT_CTRL_2, reg_value, CS42L42_ADDR);

    // MISC_DETECT_CTRL, 0x1B74
    reg_value = DETECT_MODE_INACTIVE | HSBIAS_CTRL_HIZ | PDN_MIC_LVL_DETECT;
    RegWrite (MISC_DETECT_CTRL, reg_value, CS42L42_ADDR);

    Delay_ms(15); // Wait for HSBIAS to ramp down

    // HEADSET_DETECT_CTRL_2, 0x1120
    reg_value = HSDET_CTRL_AUTO_DISABLED | HSDET_SET_AHJ | HSBIAS_REF_HSX_REF | HSDET_AUTO_TIME_20US;
    RegWrite (HEADSET_DETECT_CTRL_2, reg_value, CS42L42_ADDR);

    // CLASS_H_CTRL, 0x2101
    reg_value = ADPTPWR_M3_VCP_DIV3;
    RegWrite (CLASS_H_CTRL, reg_value, CS42L42_ADDR);

    // SOFT_RAMP_RATE, 0x100A
    reg_value = ASR_RATE_16FS | DSR_RATE_2FS;
    RegWrite (SOFT_RAMP_RATE, reg_value, CS42L42_ADDR);

    Delay_ms(255);

    // HP_LOAD_DETECT, 0x1927
    reg_value = HP_LD_EN;
    RegWrite (HP_LOAD_DETECT, reg_value, CS42L42_ADDR);

    do {
        Delay_ms(10);

        // Read HP_LOAD_DETECT_DONE, 0x1926
        RegRead (HP_LOAD_DETECT_DONE, &reg_value, CS42L42_ADDR);
    } while (!(reg_value & HPLOAD_DET_DONE));

    // Read LOAD_DETECT_R_C_STAT, 0x1925
    RegRead (LOAD_DETECT_R_C_STAT, &reg_value, CS42L42_ADDR);

    if (reg_value & CLA_STAT)
        hpout_load = HPOUT_LOAD_1NF;
    else
        hpout_load = HPOUT_LOAD_10NF;

    switch (reg_value & RLA_STAT)
    {
        default:
        case RLA_STAT_15:
            full_scale_vol = FULL_SCALE_VOL_MINUS_6dB;
            break;
        case RLA_STAT_30:
        case RLA_STAT_3K:
            full_scale_vol = FULL_SCALE_VOL_0dB;
            break;
    }

    // DAC_CTRL_2, 0x1F06
    reg_value = HPOUT_PULLDOWN_1K | HPOUT_CLAMP_ENABLED | DAC_HPF_EN; // set load capacitance, restore HPF
    RegWrite (DAC_CTRL_2, reg_value, CS42L42_ADDR);

    // CLASS_H_CTRL, 0x2101
    reg_value = ADPTPWR_ADAPT; // restore adapt to signal
    RegWrite (CLASS_H_CTRL, reg_value, CS42L42_ADDR);

    // SOFT_RAMP_RATE, 0x100A
    reg_value = ASR_RATE_33FS | DSR_RATE_8FS; // restore ramp rates
    RegWrite (SOFT_RAMP_RATE, reg_value, CS42L42_ADDR);

    // HP_LOAD_DETECT, 0x1927
    reg_value = 0x00 & ~HP_LD_EN; // disable
    RegWrite (HP_LOAD_DETECT, reg_value, CS42L42_ADDR);

    // DAC_CTRL_2, 0x1F06
    reg_value = HPOUT_PULLDOWN_1K | hpout_load | HPOUT_CLAMP_ENABLED | DAC_HPF_EN; // set load capacitance, restore HPF
    RegWrite (DAC_CTRL_2, reg_value, CS42L42_ADDR);

    return full_scale_vol;
}

static void InitializeAndroidButtonDetect(void)
{
    unsigned char reg_value;

    // MIC_DETECT_CTRL_1, 0x1B75
    reg_value = LATCH_TO_VP | HS_DETECT_LEVEL_C;
    RegWrite (MIC_DETECT_CTRL_1, reg_value, CS42L42_ADDR);

    // MISC_DETECT_CTRL, 0x1B74
    reg_value = DETECT_MODE_NORMAL | HSBIAS_CTRL_2p7V & ~PDN_MIC_LVL_DETECT;
    RegWrite (MISC_DETECT_CTRL, reg_value, CS42L42_ADDR); // Page already set to 0x1B

    Delay_ms (50); // timeout required for HSBIAS and DC level detect power up

    // Read DETECT_INT_STAT_2, 0x130A to clear any pending interrupt
    RegRead (DETECT_INT_STAT_2, &reg_value, CS42L42_ADDR);

    // DETECT_INT_MASK_2, 0x1B7A
    reg_value = ~M_DETECT_TRUE_FALSE & ~M_DETECT_FALSE_TRUE & ~M_HSBIAS_HIZ | DETECT_INT_MASK_2_RESERVED    | M_SHORT_RELEASE | M_SHORT_DETECT;
    RegWrite (DETECT_INT_MASK_2, reg_value, CS42L42_ADDR);

    // CS42L42 will now generate an interrupt when a button is pressed or HSBIAS is set to Hi-Z
    // The interrupt will be serviced by CS42L42_INTB_handler function.
}



static EVENT CheckForPlugEvent(void)
{
    EVENT L42_event = NO_EVENT;

    // Maintain a copy of the last plug detection
    static unsigned char Last_TS_PLUG = 0, Last_RS_PLUG = 0;
    unsigned char reg_value;

    // Read TIP_RNG_SENS_INT_STAT, 0x130F to check for PLUG/UNPLUG event
    RegRead (TIP_RNG_SENS_INT_STAT, &reg_value, CS42L42_ADDR);

    // If the TS_PLUG status changed, then send an EVENT
    if ((reg_value & TS_PLUG) != Last_TS_PLUG)
    {
        if (reg_value & TS_PLUG)
            L42_event = TS_PLUG_EVENT;
        else
            L42_event = TS_UNPLUG_EVENT;
        Last_TS_PLUG = (reg_value & TS_PLUG);
    }

    // If the RS_PLUG status changed, then send an EVENT
    else if ((reg_value & RS_PLUG) != Last_RS_PLUG)
    {
        if (reg_value & RS_PLUG)
            L42_event = RS_PLUG_EVENT;
        else
            L42_event = RS_UNPLUG_EVENT;
        Last_RS_PLUG = (reg_value & RS_PLUG);
    }
    return L42_event;
}


static EVENT CheckForHSdetectEvent(STATE L42_state)
{
    EVENT L42_event = NO_EVENT;
    unsigned char reg_value;

    // Read CODEC_INT_STAT, 0x1308 to check for Headset Detect Done event
    RegRead (CODEC_INT_STAT, &reg_value, CS42L42_ADDR);

    if (L42_state == UNPLUGGED)
        L42_event = NO_EVENT;
    else if (reg_value & HSDET_AUTO_DONE)
    {
        // HSDET logic has completed its detection cycle

        // Read HEADSET_DETECT_STAT, 0x1124 to check Headset detect type
        RegRead (HEADSET_DETECT_STAT, &reg_value, CS42L42_ADDR);

        switch (reg_value & HSDET_TYPE)
        {
            case HSDET_TYPE_CTIA:
            case HSDET_TYPE_OMTP:
                L42_event = HEADSET_EVENT;
                break;
            case HSDET_TYPE_HEADPHONE:
                L42_event = HEADPHONE_EVENT;
                break;
            case HSDET_TYPE_INVALID:
                L42_event = INVALID_EVENT;
                break;
            default:
                break;
        }
        // CODEC_INT_MASK, 0x131B
        reg_value = M_HSDET_AUTO_DONE | M_PDN_DONE;
        RegWrite (CODEC_INT_MASK, reg_value, CS42L42_ADDR);

        // HEADSET_DETECT_CTRL_2, 0x1120
        reg_value = HSDET_CTRL_AUTO_DISABLED | HSDET_SET_AHJ | HSBIAS_REF_HSX_REF | HSDET_AUTO_TIME_100US;
        RegWrite (HEADSET_DETECT_CTRL_2, reg_value, CS42L42_ADDR);

        // DAC_CTRL_2, 0x1F06
        reg_value = HPOUT_PULLDOWN_1K | HPOUT_CLAMP_ENABLED | DAC_HPF_EN;
        RegWrite (DAC_CTRL_2, reg_value, CS42L42_ADDR);
    }

    return L42_event;
}

static EVENT CheckForAndroidButtonEvent(void)
{
    EVENT L42_event = NO_EVENT;
    unsigned char reg_value;

    // Read DETECT_INT_STAT_2, 0x130A
    RegRead (DETECT_INT_STAT_2, &reg_value, CS42L42_ADDR);

    if (reg_value & HSBIAS_HIZ)
    {
        //
        // HSBIAS_HIZ should only be set on an unplug, but it is possible it gets set by boucning button presses
        // When HSBIAS_HIZ is set the HSBIAS voltage is HiZ and we lose microphone and button functionality
        // We need to determine if HSBIAS_HIZ was set from a button press
        //

        // Check if this the jack is being unplugged, need check for ~500 ms
        for (reg_value=0; reg_value<50 && L42_event != TS_UNPLUG_EVENT; reg_value++)
        {
            Delay_ms(10);
            L42_event = CheckForPlugEvent();
        }

        if (L42_event == TS_UNPLUG_EVENT)
            return L42_event;
        else
        {
            // HSBIAS_SENS_HIZ_AUTOCTRL, 0x1B70
            reg_value = HSBIAS_SENSE_EN & ~AUTO_BIAS_HIZ | HSBIAS_SENS_TRIP_52uA;
            RegWrite (HSBIAS_SENS_HIZ_AUTOCTRL, reg_value, CS42L42_ADDR);

            // HSBIAS_SENS_HIZ_AUTOCTRL, 0x1B70
            reg_value = HSBIAS_SENSE_EN | AUTO_BIAS_HIZ | HSBIAS_SENS_TRIP_52uA;
            RegWrite (HSBIAS_SENS_HIZ_AUTOCTRL, reg_value, CS42L42_ADDR);

            goto exit;
        }
    }

    if (reg_value & DETECT_TRUE_FALSE) // Button release
        // Bias voltage risen above the threshold set by HS_DETECT_LEVEL (button release event)
        L42_event = BUTTON_RELEASE_EVENT;
    else if (reg_value & DETECT_FALSE_TRUE) // Button press
    {
        // Bias voltage dropped below the threshold set by HS_DETECT_LEVEL (button press event)

        // DETECT_INT_MASK_2, 0x1B7A
        reg_value = M_DETECT_TRUE_FALSE | M_DETECT_FALSE_TRUE   | DETECT_INT_MASK_2_RESERVED | M_HSBIAS_HIZ | M_SHORT_RELEASE | M_SHORT_DETECT;
        RegWrite (DETECT_INT_MASK_2, reg_value, CS42L42_ADDR);

        Delay_ms(10); // Wait for button status to debounce and settle

        // MIC_DETECT_CTRL_1, 0x1B75
        reg_value = LATCH_TO_VP | HS_DETECT_LEVEL_B;
        RegWrite MIC_DETECT_CTRL_1, reg_value, CS42L42_ADDR); // Page already set to 0x1B

        // DETECT_STATUS_2, 0x1B78
        RegRead (DETECT_STATUS_2, &reg_value, CS42L42_ADDR); // Page already set to 0x1B

        if (!(reg_value & HS_TRUE))
        {
            // level is above the specified threshold (B) --> button C was pressed
            L42_event = BUTTON_C_PRESS_EVENT;
            goto exit;
        }

        // MIC_DETECT_CTRL_1, 0x1B75
        reg_value = LATCH_TO_VP | HS_DETECT_LEVEL_D;
        RegWrite (MIC_DETECT_CTRL_1, reg_value, CS42L42_ADDR); // Page already set to 0x1B

        // DETECT_STATUS_2, 0x1B78
        RegRead (DETECT_STATUS_2, &reg_value, CS42L42_ADDR); // Page already set to 0x1B

        if (!(reg_value & HS_TRUE))
        {
            // level is above the specified threshold (D) --> button B was pressed
            L42_event = BUTTON_B_PRESS_EVENT;
            goto exit;
        }

        // MIC_DETECT_CTRL_1, 0x1B75
        reg_value = LATCH_TO_VP | HS_DETECT_LEVEL_A;
        RegWrite (MIC_DETECT_CTRL_1, reg_value, CS42L42_ADDR); // Page already set to 0x1B

        // DETECT_STATUS_2, 0x1B78
        RegRead (DETECT_STATUS_2, &reg_value, CS42L42_ADDR); // Page already set to 0x1B

        if (!(reg_value & HS_TRUE))
            // level is above the specified threshold (A) --> button D was pressed
            L42_event = BUTTON_D_PRESS_EVENT;
        else
            // level is below the specified threshold (A) --> button A was pressed
            L42_event = BUTTON_A_PRESS_EVENT;
    }
    else
        return L42_event;
exit:
    //MIC_DETECT_CTRL_1, 0x1B75
    reg_value = LATCH_TO_VP | HS_DETECT_LEVEL_C;
    RegWrite (MIC_DETECT_CTRL_1, reg_value, CS42L42_ADDR);

    // DETECT_INT_STAT_2, 0x130A read to clear any pending intterupts
    RegRead (DETECT_INT_STAT_2, &reg_value, CS42L42_ADDR);

    // DETECT_INT_MASK_2, 0x1B7A
    reg_value = ~M_DETECT_TRUE_FALSE & ~M_DETECT_FALSE_TRUE & ~M_HSBIAS_HIZ | DETECT_INT_MASK_2_RESERVED    | M_SHORT_RELEASE | M_SHORT_DETECT;
    RegWrite (DETECT_INT_MASK_2, reg_value, CS42L42_ADDR);

    // CS42L42 will now generate an interrupt when a button is pressed or released
    // The interrupt will be serviced by CS42L42_INTB_handler function.

    return L42_event;
}
