/*
 * Copyright (c) 2026 Sacra Systems Private Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ADC_ADS126X_H_
#define ZEPHYR_INCLUDE_DRIVERS_ADC_ADS126X_H_

/**
 * @file
 * @brief Texas Instruments ADS126X ADC driver API
 *
 * This file contains the API for the ADS126X 32-bit, 10-channel, low-power,
 * Delta-Sigma ADC with integrated PGA, VREF, SPI interface, and two IDACs.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ADS126X ADC driver API
 * @defgroup ads126x_interface ADS126X ADC driver API
 * @ingroup adc_interface
 * @{
 */

enum ads126x_chip_id {
    ADS126X_CHIP_ADS1262 = 0,
    ADS126X_CHIP_ADS1263 = 1,
};

#define ADS126X_ADC1_CHANNELS       15U
#define ADS126X_ADC2_CHANNELS       8U
#define ADS126X_ADC2_CHANNEL_OFFSET 16U
#define ADS126X_MAX_CHANNELS        (ADS126X_ADC2_CHANNEL_OFFSET + \
                                     ADS126X_ADC2_CHANNELS)

/* ID register */
#define ADS126X_ID_DEV_MASK     0xE0
#define ADS126X_ID_ADS1262      0x00
#define ADS126X_ID_ADS1263      0x20

/** ADC resolution in bits */
#define ADS126X_RESOLUTION 32u
/** Number of input channels */
#define ADS126X_CHANNELS   10u
/** Device ID register value */
#define ADS126X_CHANNEL_ID 0x00u

/* INPMUX Register Masks */
#define ADS126X_INPMUX_MUXP_MSK    0xF0u
#define ADS126X_INPMUX_MUXN_MSK    0x0Fu

/* Positive Input Multiplexer (Bits 7:4) */

#define ADS126X_MUXP_AIN0          (0x0u << 4)
#define ADS126X_MUXP_AIN1          (0x1u << 4)
#define ADS126X_MUXP_AIN2          (0x2u << 4)
#define ADS126X_MUXP_AIN3          (0x3u << 4)
#define ADS126X_MUXP_AIN4          (0x4u << 4)
#define ADS126X_MUXP_AIN5          (0x5u << 4)
#define ADS126X_MUXP_AIN6          (0x6u << 4)
#define ADS126X_MUXP_AIN7          (0x7u << 4)
#define ADS126X_MUXP_AIN8          (0x8u << 4)
#define ADS126X_MUXP_AIN9          (0x9u << 4)

#define ADS126X_MUXP_AINCOM        (0xAu << 4)
#define ADS126X_MUXP_TEMP          (0xBu << 4)
#define ADS126X_MUXP_AVDD          (0xCu << 4)
#define ADS126X_MUXP_DVDD          (0xDu << 4)
#define ADS126X_MUXP_TDAC          (0xEu << 4)
#define ADS126X_MUXP_FLOAT         (0xFu << 4)

/* Negative Input Multiplexer (Bits 3:0) */

#define ADS126X_MUXN_AIN0          (0x0u << 0)
#define ADS126X_MUXN_AIN1          (0x1u << 0)
#define ADS126X_MUXN_AIN2          (0x2u << 0)
#define ADS126X_MUXN_AIN3          (0x3u << 0)
#define ADS126X_MUXN_AIN4          (0x4u << 0)
#define ADS126X_MUXN_AIN5          (0x5u << 0)
#define ADS126X_MUXN_AIN6          (0x6u << 0)
#define ADS126X_MUXN_AIN7          (0x7u << 0)
#define ADS126X_MUXN_AIN8          (0x8u << 0)
#define ADS126X_MUXN_AIN9          (0x9u << 0)

#define ADS126X_MUXN_AINCOM        (0xAu << 0)
#define ADS126X_MUXN_TEMP          (0xBu << 0)
#define ADS126X_MUXN_AVSS          (0xCu << 0)
#define ADS126X_MUXN_DVSS          (0xDu << 0)
#define ADS126X_MUXN_TDAC          (0xEu << 0)
#define ADS126X_MUXN_FLOAT         (0xFu << 0)

/* MODE2 Register */

/* PGA Gain */
// #define ADS126X_MODE2_GAIN_1          (0x0u << 4)
// #define ADS126X_MODE2_GAIN_2          (0x1u << 4)
// #define ADS126X_MODE2_GAIN_4          (0x2u << 4)
// #define ADS126X_MODE2_GAIN_8          (0x3u << 4)
// #define ADS126X_MODE2_GAIN_16         (0x4u << 4)
// #define ADS126X_MODE2_GAIN_32         (0x5u << 4)

/* PGA Bypass Bit */
#define ADS126X_MODE2_PGA_BYPASS      BIT(7)

// MODE 0 Configuration Register (Bit7 REFREV, Bit6 RUNMODE, Bit5:3 - DELAY, Bit2:1 - CHOP, Bit0 - RESERVED)

/* Continuous Conversion */
#define ADS126X_MODE0_RUNMODE_CONTINUOUS    (0U << 6)

/* Pulse Conversion */
#define ADS126X_MODE0_RUNMODE_PULSE         (1U << 6)

#define ADS126X_MODE0_DELAY_0US             (0U << 3)
#define ADS126X_MODE0_DELAY_8US7            (1U << 3)
#define ADS126X_MODE0_DELAY_17US            (2U << 3)
#define ADS126X_MODE0_DELAY_35US            (3U << 3)
#define ADS126X_MODE0_DELAY_69US            (4U << 3)
#define ADS126X_MODE0_DELAY_139US           (5U << 3)
#define ADS126X_MODE0_DELAY_278US           (6U << 3)
#define ADS126X_MODE0_DELAY_555US           (7U << 3)

#define ADS126X_MODE0_CHOP_DISABLE          (0U << 1)
#define ADS126X_MODE0_CHOP_ENABLE           (1U << 1)

// MODE 1 Configuration Register ( Bit7:5 Filter, Bit4:2 SBMAG, Bit1 - SBPOL, Bit0 - RESERVED)

#define ADS126X_MODE1_FILTER_SINC1          (0U << 5)
#define ADS126X_MODE1_FILTER_SINC2          (1U << 5)
#define ADS126X_MODE1_FILTER_SINC3          (2U << 5)
#define ADS126X_MODE1_FILTER_SINC4          (3U << 5)
#define ADS126X_MODE1_FILTER_FIR            (4U << 5)

#define ADS126X_MODE1_BCS_OFF               (0U << 2)
#define ADS126X_MODE1_BCS_50NA              (1U << 2)
#define ADS126X_MODE1_BCS_200NA             (2U << 2)
#define ADS126X_MODE1_BCS_1UA               (3U << 2)
#define ADS126X_MODE1_BCS_10UA              (4U << 2)

#define ADS126X_MODE1_POL_NEGATIVE          (0U << 1)
#define ADS126X_MODE1_POL_POSITIVE          (1U << 1)

// MODE 2 Configuration Register  

#define ADS126X_MODE2_GAIN_1                (0U << 4)
#define ADS126X_MODE2_GAIN_2                (1U << 4)
#define ADS126X_MODE2_GAIN_4                (2U << 4)
#define ADS126X_MODE2_GAIN_8                (3U << 4)
#define ADS126X_MODE2_GAIN_16               (4U << 4)
#define ADS126X_MODE2_GAIN_32               (5U << 4)

#define ADS126X_MODE2_DR_2P5                (0U  << 0)
#define ADS126X_MODE2_DR_5                  (1U  << 0)
#define ADS126X_MODE2_DR_10                 (2U  << 0)
#define ADS126X_MODE2_DR_16P6               (3U  << 0)
#define ADS126X_MODE2_DR_20                 (4U  << 0)
#define ADS126X_MODE2_DR_50                 (5U  << 0)
#define ADS126X_MODE2_DR_60                 (6U  << 0)
#define ADS126X_MODE2_DR_100                (7U  << 0)
#define ADS126X_MODE2_DR_400                (8U  << 0)
#define ADS126X_MODE2_DR_1200               (9U  << 0)
#define ADS126X_MODE2_DR_2400               (10U << 0)
#define ADS126X_MODE2_DR_4800               (11U << 0)
#define ADS126X_MODE2_DR_7200               (12U << 0)
#define ADS126X_MODE2_DR_14400              (13U << 0)
#define ADS126X_MODE2_DR_19200              (14U << 0)
#define ADS126X_MODE2_DR_38400              (15U << 0)

// REFMUX Register (Bits7:4  RMUXP, Bits3:0  RMUXN)
#define ADS126X_REFMUX_P_INT_2_5V              (0U << 3)
#define ADS126X_REFMUX_P_EXT_AIN0              (1U << 3)
#define ADS126X_REFMUX_P_EXT_AIN2              (2U << 3)
#define ADS126X_REFMUX_P_EXT_AIN4              (3U << 3)
#define ADS126X_REFMUX_P_INT_AVDD              (4U << 3)

#define ADS126X_REFMUX_N_INT_2_5V              (0U << 0)
#define ADS126X_REFMUX_N_EXT_AIN0              (1U << 0)
#define ADS126X_REFMUX_N_EXT_AIN2              (2U << 0)
#define ADS126X_REFMUX_N_EXT_AIN4              (3U << 0)
#define ADS126X_REFMUX_N_INT_AVSS              (4U << 0)

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_ESF_DRIVERS_ADC_ADS126X_H */
