///////////////////////////////////////////////////////////////////////////////
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
///

///////////////////////////////////////////////////////////////////////////////
// Header File Preprocessor Directive
///////////////////////////////////////////////////////////////////////////////
#ifndef CS42L42_H
#define CS42L42_H


#define CS42L42_ADDR                    0x90    // CS42L42 I2C address


//
// define CS42L42 registers
//
#define MCLK_CTRL                       0x1009
#define SOFT_RAMP_RATE                  0x100A

#define POWER_DOWN_CTRL_1               0x1101
#define POWER_DOWN_CTRL_3               0x1103
#define RNG_SENSE_CTRL_1                0x1104
#define RNG_SENSE_CTRL_2                0x1105
#define OSC_SW_CTRL                     0x1107
#define RNG_SENSE_CTRL_3                0x1112
#define TIP_SENSE_CTRL_1                0x1113
#define TIP_RNG_SENS_STAT               0x1115
#define HEADSET_DETECT_CTRL_1           0x111F
#define HEADSET_DETECT_CTRL_2           0x1120
#define HEADSET_SWITCH_CTRL             0x1121
#define HEADSET_DETECT_STAT             0x1124
#define HEADSET_CLAMP_DISABLE           0x1129

#define MCLK_SOURCE_SELECT              0x1201
#define SPDIF_CLOCK_CONFIG              0x1202
#define FSYNC_PULSE_LOW_BYTE            0x1203
#define FSYNC_PULSE_UPPER_BYTE          0x1204
#define FSYNC_PERIOD_LOW_BYTE           0x1205
#define FSYNC_PERIOD_UPPER_BYTE         0x1206
#define ASP_CLOCK_CONFIG_1              0x1207
#define ASP_FRAME_CONFIG                0x1208
#define FS_RATE_ENABLE                  0x1209
#define INPUT_ASRC_CLOCK_CONFIG         0x120A
#define OUTPUT_ASRC_CLOCK_CONFIG        0x120B
#define PLL_DIVIDE_CONFIG_1             0x120C

#define CODEC_INT_STAT                  0x1308
#define DETECT_INT_STAT_1               0x1309
#define DETECT_INT_STAT_2               0x130A
#define PLL_LOCK_INT_STAT               0x130E
#define TIP_RNG_SENS_INT_STAT           0x130F
#define CODEC_INT_MASK                  0x131B
#define TIP_RNG_SENS_INT_MASK           0x1320

#define PLL_CTRL_1                      0x1501
#define PLL_DIV_FRAC_B0                 0x1502
#define PLL_DIV_FRAC_B1                 0x1503
#define PLL_DIV_FRAC_B2                 0x1504
#define PLL_DIVISION_INT                0x1505
#define PLL_CTRL_3                      0x1508
#define PLL_CAL_RATIO                   0x150A
#define PLL_CTRL_4                      0x151B

#define LOAD_DETECT_R_C_STAT            0x1925
#define HP_LOAD_DETECT_DONE             0x1926
#define HP_LOAD_DETECT                  0x1927

#define HSBIAS_SENS_HIZ_AUTOCTRL        0x1B70
#define TIP_SENSE_CTRL_2                0x1B73
#define MISC_DETECT_CTRL                0x1B74
#define MIC_DETECT_CTRL_1               0x1B75
#define DETECT_STATUS_1                 0x1B77
#define DETECT_STATUS_2                 0x1B78
#define DETECT_INT_MASK_1               0x1B79
#define DETECT_INT_MASK_2               0x1B7A

#define HEADSET_BIAS_CTRL               0x1C03

#define ADC_CTRL_1                      0x1D01
#define ADC_SOFT_RAMP                   0x1D02
#define ADC_VOLUME                      0x1D03
#define ADC_WNF_HPF_CTRL                0x1D04

#define DAC_CTRL_2                      0x1F06

#define HP_CONTROL                      0x2001

#define CLASS_H_CTRL                    0x2101

#define MIXER_CHA_VOL                   0x2301
#define MIXER_ADC_VOL                   0x2302
#define MIXER_CHB_VOL                   0x2303

#define SPDIF_CH_SEL                    0x2504

#define SPDIF_CTRL_1                    0x2801
#define SPDIF_CTRL_2                    0x2802

#define ASP_TX_SIZE_EN                  0x2901
#define ASP_TX_CH_EN                    0x2902
#define ASP_TX_CH_PH_RES                0x2903

#define ASP_RX_EN                       0x2A01
#define ASP_RX_DAI0_CH1_PH_RES          0x2A02
#define ASP_RX_DAI0_CH1_BS_MSB          0x2A03
#define ASP_RX_DAI0_CH1_BS_LSB          0x2A04
#define ASP_RX_DAI0_CH2_PH_RES          0x2A05
#define ASP_RX_DAI0_CH2_BS_MSB          0x2A06
#define ASP_RX_DAI0_CH2_BS_LSB          0x2A07

//
// Define CS42L42 Individual register bit-fields
//

// MCLK_CTRL, 0x1009
#define INTERNAL_FS_MCLK_BY_250         0x00    // 0b0 - FS internal = MCLK internal / 250
#define INTERNAL_FS_MCLK_BY_256         0x02    // 0b1 - FS internal = MCLK internal / 256

// SOFT_RAMP_RATE, 0x100A
#define ASR_RATE_33FS                   0xA0    // 0b1010 - 33FS Analog soft-ramp rate (# Fs periods between steps).  Step Size = 1 or 2 dB.
#define ASR_RATE_16FS                   0x70    // 0b0111 - 16FS Analog soft-ramp rate (# Fs periods between steps).  Step Size = 1 or 2 dB.
#define DSR_RATE_8FS                    0x04    // 0b0100 - 8FS Digital soft-ramp rate (# Fs periods between steps).  Step Size = 0.125 dB.
#define DSR_RATE_2FS                    0x01    // 0b0001 - 2FS Digital soft-ramp rate (# Fs periods between steps).  Step Size = 0.125 dB.

// POWER_DOWN_CTRL_1, 0x1101
#define ASP_DAO_PDN                     0x80
#define ASP_DAI_PDN                     0x40
#define MIXER_PDN                       0x20
#define EQ_PDN                          0x10
#define HP_PDN                          0x08
#define ADC_PDN                         0x04
#define PDN_CTRL_1_RESERVED             0x02    // 0b1 - Reserved field, must be set
#define PDN_ALL                         0x01

// POWER_DOWN_CTRL_3, 0x1103
#define RING_SENSE_PUP                  0x02    // 0b1 - Power up ring sense
#define RING_SENSE_PDN                  0x00    // 0b0 - Power down ring sense

// OSC_SW_CTRL, 0x1107
#define SCLK_PRESENT                    0x01    // 0b1 - SCLK present, power down RCO and switch to SCLK/PLL

// TIP_RNG_SENS_STAT, 0x1115
#define TS_PLUG_DBNC                    0x04

// RNG_SENSE_CTRL_3, 0x1112
#define RS_INV                          0x80    // 0b1 - Inverted
#define RS_PU_EN                        0x40    // 0b1 - Pull-up connected
#define RS_FALL_DBNCE_TIME_0MS          0x00    // 0b000 - 0 ms
#define RS_RISE_DBNCE_TIME_500MS        0x03    // 0b011 - 500 ms
#define RS_RISE_DBNCE_TIME_1S           0x05    // 0b101 - 1.0 second

// TIP_SENSE_CTRL_1, 0x1113
#define TS_INV                          0x80    // 0b1 - Invert TIP_SENSE
#define TS_FALL_DBNCE_TIME_0MS          0x00    // 0b000 - 0 ms debounce time on unplug
#define TS_RISE_DBNCE_TIME_250MS        0x02    // 0b010 - 250 ms debounce time on plug
#define TS_RISE_DBNCE_TIME_500MS        0x03    // 0b011 - 500 ms debounce time on plug
#define TS_RISE_DBNCE_TIME_1S           0x05    // 0b101 - 1.0 s debounce time on plug

// HEADSET_DETECT_CTRL_1, 0x111F
#define HSDET_COMP2_LVL_2V              0x70
#define HSDET_COMP1_LVL_1V              0x07

// HEADSET_DETECT_CTRL_2, 0x1120
#define HSDET_CTRL_MANUAL_DISABLED      0x00
#define HSDET_CTRL_MANUAL_ACTIVE        0x40
#define HSDET_CTRL_AUTO_DISABLED        0x80
#define HSDET_CTRL_AUTO_ACTIVE          0xC0
#define HSDET_SET_AHJ                   0x00
#define HSBIAS_REF_HSX_REF              0x00
#define HSBIAS_REF_HSX                  0x08
#define HSDET_AUTO_TIME_10US            0x00
#define HSDET_AUTO_TIME_20US            0x01
#define HSDET_AUTO_TIME_50US            0x02
#define HSDET_AUTO_TIME_100US           0x03

// HEADSET_DETECT_STAT, 0x1124
#define HSDET_TYPE                      0x03
#define HSDET_TYPE_CTIA                 0x00    // 0b00 - CTIA headset detected
#define HSDET_TYPE_OMTP                 0x01    // 0b01 - OMTP (AHJ) headset detect
#define HSDET_TYPE_HEADPHONE            0x02    // 0b10 - headphones detected
#define HSDET_TYPE_INVALID              0x03    // 0b11 - Invalid (on CDB42L42) type detected, could be Optical if headset jack supported Optical (combination jack Optical + Headset)

// HEADSET_CLAMP_DISABLE, 0x1129
#define HS_CLAMP_DISABLE                0x01    // 0b1 - HS clamps are disconnected
#define HS_CLAMP_ENABLE                 0x00    // 0b0 - HS clamps are connected

// MCLK_SOURCE_SELECT, 0x1201
#define MCLKDIV_1                       0x00    // 0b0 - Divide by 1
#define MCLKDIV_2                       0x02    // 0b1 - Divide by 2
#define MCLK_SRC_SEL_SCLK               0x00    // 0b0 - MCLK internal = SCLK pin
#define MCLK_SRC_SEL_PLL                0x01    // 0b1 - MCLK internal = PLL clock

// SPDIF_CLOCK_CONFIG, 0x1202
#define SPDIF_CLK_DIV_1                 0x00    // 0b000 - S/PDIF clock = MCLK internal / 1
#define SPDIF_CLK_DIV_2                 0x08    // 0b001 - S/PDIF clock = MCLK internal / 2
#define SPDIF_CLK_DIV_3                 0x10    // 0b010 - S/PDIF clock = MCLK internal / 3
#define SPDIF_CLK_DIV_4                 0x18    // 0b011 - S/PDIF clock = MCLK internal / 4
#define SPDIF_CLK_DIV_8                 0x20    // 0b100 - S/PDIF clock = MCLK internal / 8
#define SPDIF_LRCK_SRC_SEL_INT          0x00    // 0b0 - S/PDIF LRCK from internally generated LRCLK
#define SPDIF_LRCK_SRC_SEL_EXT          0x04    // 0b1 - S/PDIF LRCK from ASP_LRCK pin.
#define SPDIF_LRCK_CPOL                 0x02    // 0b1 - Invert LRCK

// FSYNC_PULSE_LOW_BYTE, 0x1203
#define FSYNC_PULSE_WIDTH_LB_32         0x1F    // 0b00000011111 - 31, LRCLK pulse is 32 SCLK's

// FSYNC_PULSE_UPPER_BYTE, 0x1204
#define FSYNC_PULSE_WIDTH_UB_32         0x00

// FSYNC_PERIOD_LOW_BYTE, 0x1205
#define FSYNC_PERIOD_LOW_BYTE_64        0x3F    // 0b00000111111 - 63, 64 SCLK's per LRCLK

// FSYNC_PERIOD_UPPER_BYTE, 0x1206
#define FSYNC_PERIOD_UPPER_BYTE_64      0x00

// ASP_CLOCK_CONFIG_1, 0x1207
#define ASP_SCLK_EN                     0x20    // 0b1 - ASP SCLK Enabled
#define ASP_HYBRID_MODE                 0x10    // 0b1 - LRCLK is an output generated from SCLK (Hybrid Master Mode).
#define ASP_SCPOL_IN_ADC                0x08    // 0b1 - ASP SCLK input polarity to ADC = Inverted
#define ASP_SCPOL_IN_DAC                0x04    // 0b1 - ASP SCLK input polarity to DAC = Inverted
#define ASP_LCPOL_OUT                   0x02    // 0b1 - ASP LRCK output drive polarity = Inverted
#define ASP_LCPOL_IN                    0x01    // 0b1 - ASP LRCK input polarity = Inverted

// ASP_FRAME_CONFIG, 0x1208
#define ASP_STP_0                       0x00    // 0b0 - Frame begins when LRCLK transitions from high to low
#define ASP_STP_1                       0x10    // 0b1 - Frame begins when LRCLK transitions from low to high
#define ASP_5050                        0x08    // 0b1 - 50/50 Mode (LRCK has fixed 50% dugy cycle)
#define ASP_FSD_0                       0x00    // 0b000 - 0 SCLK's frame-start delay
#define ASP_FSD_0p5                     0x01    // 0b001 - 0.5 SCLK's frame-start delay
#define ASP_FSD_1                       0x02    // 0b010 - 1 SCLK frame-start delay

// PLL_DIVIDE_CONFIG_1, 0x120C
#define PLL_REF_INV                     0x02    // 0b1 - Invert PLL reference clock
#define SCLK_PREDIV_1                   0x00    // 0b00 - PLL reference  = SCLK/1
#define SCLK_PREDIV_2                   0x01    // 0b01 - PLL reference  = SCLK/2
#define SCLK_PREDIV_4                   0x02    // 0b10 - PLL reference  = SCLK/4
#define SCLK_PREDIV_8                   0x03    // 0b11 - PLL reference  = SCLK/8

// CODEC_INT_STAT, 0x1308
#define HSDET_AUTO_DONE                 0x02    // 0b1 - Automotic Headset detect is done
#define PDN_DONE                        0x01    // 0b1 - Powered down as a result of PDN_ALL having been set

// DETECT_INT_STAT_1, 0x1309
#define HSBIAS_SENSE                    0x80    // 0b1 - Output current has gone below the specified threshold.
#define TIP_SENSE_PLUG                  0x40    // 0b1 - HP plug event
#define TIP_SENSE_UNPLUG                0x20    // 0b1 - HP unplug event

// DETECT_INT_STAT_2, 0x130A
#define DETECT_TRUE_FALSE               0x80
#define DETECT_FALSE_TRUE               0x40
#define HSBIAS_HIZ                      0x04

// PLL_LOCK_INT_STAT, 0x130E
#define PLL_LOCK                        0x01    // 0b1 - PLL is locked

// TIP_RNG_SENS_INT_STAT, 0x130F
#define TS_PLUG                         0x04    // 0b1 - Tip Sense Plug event
#define RS_PLUG                         0x01    // 0b1 - Ring Sense Plug event

// CODEC_INT_MASK, 0x131B
#define M_HSDET_AUTO_DONE               0x02    // 0b1 - Mask HSDET_AUTO_DONE interrupt
#define M_PDN_DONE                      0x01    // 0b1 - Mask PDN_DONE interrupt

// TIP_RNG_SENS_PLUG_INT_MASK, 0x1320
#define M_TS_UNPLUG                     0x08    // 0b1 - Mask Tip Sense Unplug interrupt
#define M_TS_PLUG                       0x04    // 0b1 - Mask Tip Sense Plug interrupt
#define M_RS_UNPLUG                     0x02    // 0b1 - Mask Ring Sense Unplug interrupt
#define M_RS_PLUG                       0x01    // 0b1 - Mask Ring Sense Plug interrupt

// PLL_CTRL_1, 0x1501
#define PLL_START                       0x01    // 0b1 - Powered On

// PLL_DIV_FRAC_B0, 0x1502
#define PLL_DIV_FRAC_B0_0               0x00

// PLL_DIV_FRAC_B1, 0x1503
#define PLL_DIV_FRAC_B1_0               0x00

// PLL_DIV_FRAC_B2, 0x1504
#define PLL_DIV_FRAC_B2_0               0x00

// PLL_DIVISION_INT, 0x1505
#define PLL_DIV_INT_64                  0x40    // 64 - PLL integer portion of divide ratio. Integer portion of PLL feedback divider.

// PLL_CTRL_3, 0x1508
#define PLL_DIVOUT_16                   0x10    // 16 - Final PLL clock output divide value.

// PLL_CAL_RATIO, 0x150A
#define PLL_CAL_RATIO_128               0x80    // 128 - PLL calibration ratio. Target value for PLL VCO calibration.

// PLL_CTRL_4, 0x151B
#define PLL_MODE_1                      0x01    // 0b01 - 500/512
#define PLL_MODE_2                      0x02    // 0b10 - 1029/1024
#define PLL_MODE_3                      0x03    // 0b11 - Bypass 500/512 and 1029/1024

// LOAD_DETECT_R_C_STAT, 0x1925
#define CLA_STAT                        0x10    // 0b1 - bit-field for CLA_STAT
#define RLA_STAT                        0x03    // 0b11 - bit-field for RLA_STAT
#define RLA_STAT_15                     0x00    // 0b00 - 15 ohms detected
#define RLA_STAT_30                     0x01    // 0b01 - 30 ohms detected
#define RLA_STAT_3K                     0x02    // 0b10 - 3k ohms detected

// HP_LOAD_DETECT_DONE, 0x1926
#define HPLOAD_DET_DONE                 0x01    // 0b1 - HP load is finished.

// HP_LOAD_DETECT, 0x1927
#define HP_LD_EN                        0x01    // 0b1 - HP load detect enable. A 0-to-1 bit transition initiates load detection.

// HSBIAS_SENS_HIZ_AUTOCTRL, 0x1B70
#define HSBIAS_SENSE_EN                 0x80    // 0b1 - HSBIAS current sense enabled
#define AUTO_BIAS_HIZ                   0x40    // 0b1 - Sets HSBIAS to Hi-Z Mode when the current sense goes below its trip point or a HP unplug event occurs, depending on which detector is enabled. To disengage Hi-Z Mode, clear this bit before resetting it to 1.
#define TIP_SENSE_EN                    0x20    // 0b1 - TIP_SENSE unplug event affects the HSBIAS Hi-Z Mode if AUTO_HSBIAS_HIZ = 1.
#define HSBIAS_SENS_TRIP_12uA           0x00    // 0b000 - HSBIAS current sense trip point set to 12 uA
#define HSBIAS_SENS_TRIP_23uA           0x01    // 0b001 - HSBIAS current sense trip point set to 23 uA
#define HSBIAS_SENS_TRIP_41uA           0x02    // 0b010 - HSBIAS current sense trip point set to 41 uA
#define HSBIAS_SENS_TRIP_52uA           0x03    // 0b011 - HSBIAS current sense trip point set to 52 uA
#define HSBIAS_SENS_TRIP_64uA           0x04    // 0b100 - HSBIAS current sense trip point set to 64 uA
#define HSBIAS_SENS_TRIP_75uA           0x05    // 0b101 - HSBIAS current sense trip point set to 75 uA
#define HSBIAS_SENS_TRIP_93uA           0x06    // 0b110 - HSBIAS current sense trip point set to 93 uA
#define HSBIAS_SENS_TRIP_104uA          0x07    // 0b111 - HSBIAS current sense trip point set to 104 uA

// TIP_SENSE_CTRL_2, 0x1B73
#define TIP_SENSE_CTRL_SHORT_DET        0xC0    // 0b11 - Short detect, Internal weak current source pull-up is enabled
#define TIP_SENSE_INV                   0x20    // 0b1 - Invert TIP_SENSE
#define TIP_SENSE_DEBOUNCE_500MS        0x02    // 0b10 - 500 ms debounce time on tip sense

// MISC_DETECT_CTRL, 0x1B74
#define DETECT_MODE_INACTIVE            0x00    // 0b00 - Inactive (SHORT_DETECTED and SHORT_RELEASE in the VP domain are also cleared)
#define DETECT_MODE_SHORT               0x01    // 0b01 - Short Detect only. Normal interrupts do not function;
                                                //        the INT pin follows the S0 comparator directly while the SHORT_DETECTED
                                                //        mask is cleared and remains high while the SHORT_DETECTED mask is set.
#define DETECT_MODE_NORMAL              0x18    // 0b11 - Normal Mode. HSBIAS output uses a high-performance reference for 2.0- or 2.7-V Mode. See HSBIAS_CTRL.
#define HSBIAS_CTRL_HIZ                 0x00    // 0b00 - Output is Hi-Z. The HSBIAS output uses a low-performance, low-power reference.
#define HSBIAS_CTRL_0V                  0x02    // 0b10 - 0.0 V (weak ground)
#define HSBIAS_CTRL_2p0V                0x04    // 0b10 - 2.0V Wait for circuits to completely power up.
#define HSBIAS_CTRL_2p7V                0x06    // 0b11 - 2.7V Wait for circuits to completely power up.
#define PDN_MIC_LVL_DETECT              0x01    // 0b1 - Power Down mic DC level detect

// MIC_DETECT_CTRL_1, 0x1B75
#define LATCH_TO_VP                     0x80
#define EVENT_STATUS_SEL                0x40
#define HS_DETECT_LEVEL_A               0x01    // recommended threshold between Function D and Function A
#define HS_DETECT_LEVEL_D               0x03    // recommended threshold between Function B and Function D
#define HS_DETECT_LEVEL_B               0x06    // recommended threshold between Function C and Function B
#define HS_DETECT_LEVEL_C               0x0F    // recommended threshold between Mic only and Function C
#define HS_DETECT_LEVEL_DEFAULT         0x1F    // default

// DETECT_STATUS_2, 0x1B78
#define HS_TRUE                         0x02

// DETECT_INT_MASK_1, 0x1B79
#define M_HSBIAS_SENSE                  0x80    // 0b1 - Mask HSBIAS_SENSE interrupt
#define M_TIP_SENSE_PLUG                0x40    // 0b1 - Mask TIP_SENSE_PLUG interrupt
#define M_TIP_SENSE_UNPLUG              0x20    // 0b1 - Mask TIP_SENSE_UNPLUG interrupt

// DETECT_INT_MASK_2, 0x1B7A
#define M_DETECT_TRUE_FALSE             0x80    // 0b1 - Mask DETECT_TRUE_FALSE interrupt
#define M_DETECT_FALSE_TRUE             0x40    // 0b1 - Mask M_DETECT_FALSE_TRUE interrupt
#define DETECT_INT_MASK_2_RESERVED      0x38    // 0b111 - Reserved value, must write 0b111
#define M_HSBIAS_HIZ                    0x04    // 0b1 - Mask HSBIAS Hi-Z interrupt
#define M_SHORT_RELEASE                 0x02    // 0b1 - Mask Short Release interrupt
#define M_SHORT_DETECT                  0x01    // 0b1 - Mask Short Detect interrupt

// HEADSET_BIAS_CTRL, 0x1C03
#define HSBIAS_CAPLESS_EN               0x80    // 0b1 - No external capacitor
#define HSBIAS_CTRL_RESERVED            0x40
#define HSBIAS_PD                       0x10    // 0b1 - Pulldown resistor on
#define HSBIAS_RAMP_FAST_SLOW           0x00    // 0b00 - Fast rise time, slow, load-dependent fall time
#define HSBIAS_RAMP_FAST                0x01    // 0b01 - Fast
#define HSBIAS_RAMP_SLOW                0x02    // 0b10 - Slow
#define HSBIAS_RAMP_SLOWEST             0x03    // 0b10 - Slowest

// ADC_CTRL_1, 0x1D01
#define ADC_NOTCH_DIS                   0x20    // 0b1 - Disable ADC notch filter
#define ADC_FORCE_WEAK_VCM              0x10    // 0b1 - Force ADC input weak VCM
#define ADC_INV                         0x04    // 0b1 - Invert ADC signal polarity
#define ADC_DIG_BOOST                   0x01    // 0b1 - +20 dB digitial boost applied

// ADC_SOFT_RAMP, 0x1D02
#define ADC_SOFTRAMP_EN                 0x04    // 0b1 - Enable digital volume soft-ramp, soft-ramp rate is set by DSR_RATE
#define ADC_RESERVED                    0x02    // 0b1 - Must set this bit

// ADC_VOLUME, 0x1D03
#define ADC_VOL_12dB                    0x0C    // 0b00001100 - +12 dB ADC digital volume

// ADC_WNF_HPF_CTRL, 0x1D04
#define ADC_WNF_CF                      0x70    // 0b111 - Default
#define ADC_WNF_EN                      0x08    // 0b1 - Enable ADC Wind Noise Filter
#define ADC_HPF_CF_1p86Hz               0x00    // 0b00 - 1.86 Hz ADC HPF corner frequency
#define ADC_HPF_EN                      0x01    // 0b1 - Enable ADC HPF

// DAC_CTRL_2, 0x1F06
#define HPOUT_PULLDOWN_NONE             0x80
#define HPOUT_PULLDOWN_1K               0x00
#define HPOUT_LOAD_1NF                  0x00
#define HPOUT_LOAD_10NF                 0x08
#define HPOUT_CLAMP_ENABLED             0x00    // HPOUT clamps are enabled when channels are powered down
#define HPOUT_CLAMP_DISABLED            0x04    // HPOUT clamps are disabled when channels are powered down
#define DAC_HPF_EN                      0x02    // HPF enabled
#define DAC_HPF_DISABLED                0x00    // HPF disabled

// HP_CONTROL, 0x2001
#define ANA_MUTE_B                      0x08    // 0b1 - Mute analog channel B
#define ANA_MUTE_A                      0x04    // 0b1 - Mute analog channel A
#define FULL_SCALE_VOL_0dB              0x00    // 0b0 - Full-scale volume is 0 dB
#define FULL_SCALE_VOL_MINUS_6dB        0x02    // 0b1 - Full-scale volume is -6 dB
#define HP_CTRL_RESERVED                0x01    // 0b1 - Resered field, must be set

// CLASS_H_CTRL, 0x2101
#define ADPTPWR_M0_VP_CP                0x01    // 0b001 - Fixed, Mode 0�VP_CP Mode (�2.5V)
#define ADPTPWR_M1_VCP                  0x02    // 0b010 - Fixed, Mode 1�VCP Mode (�VCP)
#define ADPTPWR_M2_VCP_DIV2             0x03    // 0b011 - Fixed, Mode 2 �VCP/2 Mode (�VCP/2)
#define ADPTPWR_M3_VCP_DIV3             0x04    // 0b100 - Fixed, Mode 3 �VCP/3 Mode (�VCP/3)
#define ADPTPWR_ADAPT                   0x07    // 0b111 - Adapt to signal

// MIXER_CHA_VOL, 0x2301
#define MIXER_CHA_VOL_0DB               0x00    // 0b000000 - CHA attenuated 0 dB
#define MIXER_CHA_VOL_MUTE              0x3F    // 0b000000 - CHA Muted

// MIXER_ADC_VOL, 0x2302
#define MIXER_ADC_VOL_MUTE              0x3F    // 0b000000 - ADC Muted

// MIXER_CHB_VOL, 0x2303
#define MIXER_CHB_VOL_0DB               0x00    // 0b000000 - CHB attenuated 0 dB
#define MIXER_CHB_VOL_MUTE              0x3F    // 0b000000 - CHB Muted

// SPDIF_CH_SEL, 0x2504
#define SPDIF_CHB_SEL_CH1               0x04    // 0b01 - S/PDIF CHB = CH1
#define SPDIF_CHA_SEL_CH0               0x00    // 0b00 - S/PDIF CHA = CH0

// SPDIF_CTRL_1, 0x2801
#define SPDIF_TX_PDN                    0x01    // 0b1 - Power Down S/PDIF

// SPDIF_CTRL_2, 0x2802
#define SPDIF_TX_DIGEN                  0x01    // 0b1 - S/PDIF transmit enbabled

// ASP_TX_SIZE_EN, 0x2901
#define ASP_TX_2FS                      0x02    // 0b1 - 2FS Mode (doubles incoming LRCLK rate)
#define ASP_TX_EN                       0x01    // 0b1 - Enabled (driven)

// ASP_TX_CH_EN, 0x2902
#define ASP_TX_CH2_EN                   0x02    // 0b1 - Enabled, ASP Transmit Channel 2 enable.  Data replicated from CH1
#define ASP_TX_CH1_EN                   0x01    // 0b1 - Enabled, ASP Transmit Channel 1 enable.

// ASP_TX_CH_PH_RES, 0x2903
#define ASP_TX_CH1_AP                   0x80    // 0b1 - High, CH1 transmit active phase (data active when LRCLK is high)
#define ASP_TX_CH2_AP                   0x40    // 0b1 - High, CH2 transmit active phase (data active when LRCLK is high)
#define ASP_TX_CH2_RES_32BITS           0x0C    // 0b11 - 32 bits per sample
#define ASP_TX_CH1_RES_32BITS           0x03    // 0b11 - 32 bits per sample

// ASP_RX_EN, 0x2A01
#define ASP_RX_DISABLE                  0x00    // 0b000000 - The corresponding channel buffers are disabled
#define ASP_RX1_CH2_EN                  0x80    // 0b1 - The corresponding channel buffer receives data (only when using S/PDIF in 2Fs Mode and playback in Fs Mode)
#define ASP_RX1_CH1_EN                  0x40    // 0b1 - The corresponding channel buffer receives data (only when using S/PDIF in 2Fs Mode and playback in Fs Mode)
#define ASP_RX0_CH4_EN                  0x20    // 0b1 - The corresponding channel buffer is populated
#define ASP_RX0_CH3_EN                  0x10    // 0b1 - The corresponding channel buffer is populated
#define ASP_RX0_CH2_EN                  0x08    // 0b1 - The corresponding channel buffer is populated
#define ASP_RX0_CH1_EN                  0x04    // 0b1 - The corresponding channel buffer is populated
#define ASP_RX1_2FS                     0x02    // 0b1 - Sample rate is doubled, 2 Fs
#define ASP_RX0_2FS                     0x01    // 0b1 - Sample rate is doubled, 2 Fs

// ASP_RX_DAI0_CH1_PH_RES, 0x2A02
#define ASP_RX0_CH1_AP                  0x40    // 0b1 - High, ASP receive DAI0 active phase, Valid only in 50/50 Mode, channel data is valid when LRCK/FSYNC is high.
#define ASP_RX0_CH1_RES_32BIT           0x03    // 0b11 - 32 bits per sample

// ASP_RX_DAI0_CH1_BS_MSB, 0x2A03
#define ASP_RX0_CH1_BIT_ST_MSB_0        0x00    // ASP receive DAI0 Channel 1 bit start MSB. Configures the MSB location of the channel with respect to SOF (LRCK edge + phase lag)

// ASP_RX_DAI0_CH1_BS_LSB, 0x2A04
#define ASP_RX0_CH1_BIT_ST_LSB_0        0x00    // ASP receive DAI0 Channel 1 bit start LSB. Configures the LSB location of the channel with respect to SOF (LRCK edge + phase lag)

// ASP_RX_DAI0_CH2_PH_RES, 0x2A05
#define ASP_RX0_CH2_AP                  0x40    // 0b1 - High, ASP receive DAI0 active phase, Valid only in 50/50 Mode, channel data is valid when LRCK/FSYNC is high.
#define ASP_RX0_CH2_RES_32BIT           0x03    // 0b11 - 32 bits per sample

// ASP_RX_DAI0_CH2_BS_MSB, 0x2A06
#define ASP_RX0_CH2_BIT_ST_MSB_0        0x00    // ASP receive DAI0 Channel 2 bit start MSB. Configures the MSB location of the channel with respect to SOF (LRCK edge + phase lag)

// ASP_RX_DAI0_CH2_BS_LSB, 0x2A07
#define ASP_RX0_CH2_BIT_ST_LSB_0        0x00    // ASP receive DAI0 Channel 2 bit start LSB. Configures the LSB location of the channel with respect to SOF (LRCK edge + phase lag)

#endif
