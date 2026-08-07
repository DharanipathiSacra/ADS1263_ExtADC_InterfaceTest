/*
 * Copyright (c) 2026 Sacra Systems Private Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ADC_ADS1263_H_
#define ZEPHYR_INCLUDE_DRIVERS_ADC_ADS1263_H_

/**
 * @file
 * @brief Texas Instruments ADS1263 ADC driver API
 *
 * This file contains the API for the ADS1263 32-bit, 10-channel, low-power,
 * Delta-Sigma ADC with integrated PGA, VREF, SPI interface, and two IDACs.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ADS1263 ADC driver API
 * @defgroup ads1220_interface ADS1263 ADC driver API
 * @ingroup adc_interface
 * @{
 */

/** ADC resolution in bits */
#define ADS1263_RESOLUTION 32u
/** Number of input channels */
#define ADS1263_CHANNELS   10u
/** Device ID register value */
#define ADS1263_CHANNEL_ID 0x00u

/* INPMUX Register Masks */
#define ADS1263_INPMUX_MUXP_MSK    0xF0u
#define ADS1263_INPMUX_MUXN_MSK    0x0Fu

/* Positive Input Multiplexer (Bits 7:4) */

#define ADS1263_MUXP_AIN0          (0x0u << 4)
#define ADS1263_MUXP_AIN1          (0x1u << 4)
#define ADS1263_MUXP_AIN2          (0x2u << 4)
#define ADS1263_MUXP_AIN3          (0x3u << 4)
#define ADS1263_MUXP_AIN4          (0x4u << 4)
#define ADS1263_MUXP_AIN5          (0x5u << 4)
#define ADS1263_MUXP_AIN6          (0x6u << 4)
#define ADS1263_MUXP_AIN7          (0x7u << 4)
#define ADS1263_MUXP_AIN8          (0x8u << 4)
#define ADS1263_MUXP_AIN9          (0x9u << 4)

#define ADS1263_MUXP_AINCOM        (0xAu << 4)
#define ADS1263_MUXP_TEMP          (0xBu << 4)
#define ADS1263_MUXP_AVDD          (0xCu << 4)
#define ADS1263_MUXP_DVDD          (0xDu << 4)
#define ADS1263_MUXP_TDAC          (0xEu << 4)
#define ADS1263_MUXP_FLOAT         (0xFu << 4)

/* Negative Input Multiplexer (Bits 3:0) */

#define ADS1263_MUXN_AIN0          (0x0u)
#define ADS1263_MUXN_AIN1          (0x1u)
#define ADS1263_MUXN_AIN2          (0x2u)
#define ADS1263_MUXN_AIN3          (0x3u)
#define ADS1263_MUXN_AIN4          (0x4u)
#define ADS1263_MUXN_AIN5          (0x5u)
#define ADS1263_MUXN_AIN6          (0x6u)
#define ADS1263_MUXN_AIN7          (0x7u)
#define ADS1263_MUXN_AIN8          (0x8u)
#define ADS1263_MUXN_AIN9          (0x9u)

#define ADS1263_MUXN_AINCOM        (0xAu)
#define ADS1263_MUXN_TEMP          (0xBu)
#define ADS1263_MUXN_AVSS          (0xCu)
#define ADS1263_MUXN_DVSS          (0xDu)
#define ADS1263_MUXN_TDAC          (0xEu)
#define ADS1263_MUXN_FLOAT         (0xFu)

/* MODE2 Register */

/* Gain Field Mask (Bits 6:4) */
#define ADS1263_MODE2_GAIN_MASK        0x70u


/* PGA Bypass Bit */
#define ADS1263_MODE2_PGA_BYPASS      BIT(7)

// MODE 0 Configuration Register (Bit7 REFREV, Bit6 RUNMODE, Bit5:3 - DELAY, Bit2:1 - CHOP, Bit0 - RESERVED)

/* Continuous Conversion */
#define ADS1263_MODE0_RUNMODE_CONTINUOUS    (0U << 6)

/* Pulse Conversion */
#define ADS1263_MODE0_RUNMODE_PULSE         (1U << 6)

#define ADS1263_MODE0_DELAY_0US             (0U << 3)
#define ADS1263_MODE0_DELAY_8US7            (1U << 3)
#define ADS1263_MODE0_DELAY_17US            (2U << 3)
#define ADS1263_MODE0_DELAY_35US            (3U << 3)
#define ADS1263_MODE0_DELAY_69US            (4U << 3)
#define ADS1263_MODE0_DELAY_139US           (5U << 3)
#define ADS1263_MODE0_DELAY_278US           (6U << 3)
#define ADS1263_MODE0_DELAY_555US           (7U << 3)

#define ADS1263_MODE0_CHOP_DISABLE          (0U << 1)
#define ADS1263_MODE0_CHOP_ENABLE           (1U << 1)

// MODE 1 Configuration Register ( Bit7:5 Filter, Bit4:2 SBMAG, Bit1 - SBPOL, Bit0 - RESERVED)

#define ADS1263_MODE1_FILTER_SINC1          (0U << 5)
#define ADS1263_MODE1_FILTER_SINC2          (1U << 5)
#define ADS1263_MODE1_FILTER_SINC3          (2U << 5)
#define ADS1263_MODE1_FILTER_SINC4          (3U << 5)
#define ADS1263_MODE1_FILTER_FIR            (4U << 5)

#define ADS1263_MODE1_BCS_OFF               (0U << 2)
#define ADS1263_MODE1_BCS_50NA              (1U << 2)
#define ADS1263_MODE1_BCS_200NA             (2U << 2)
#define ADS1263_MODE1_BCS_1UA               (3U << 2)
#define ADS1263_MODE1_BCS_10UA              (4U << 2)

#define ADS1263_MODE1_POL_NEGATIVE          (0U << 1)
#define ADS1263_MODE1_POL_POSITIVE          (1U << 1)

// MODE 2 Configuration Register  

#define ADS1263_MODE2_GAIN_1                (0U << 4)
#define ADS1263_MODE2_GAIN_2                (1U << 4)
#define ADS1263_MODE2_GAIN_4                (2U << 4)
#define ADS1263_MODE2_GAIN_8                (3U << 4)
#define ADS1263_MODE2_GAIN_16               (4U << 4)
#define ADS1263_MODE2_GAIN_32               (5U << 4)

#define ADS1263_MODE2_DR_2P5                (0U  << 0)
#define ADS1263_MODE2_DR_5                  (1U  << 0)
#define ADS1263_MODE2_DR_10                 (2U  << 0)
#define ADS1263_MODE2_DR_16P6               (3U  << 0)
#define ADS1263_MODE2_DR_20                 (4U  << 0)
#define ADS1263_MODE2_DR_50                 (5U  << 0)
#define ADS1263_MODE2_DR_60                 (6U  << 0)
#define ADS1263_MODE2_DR_100                (7U  << 0)
#define ADS1263_MODE2_DR_400                (8U  << 0)
#define ADS1263_MODE2_DR_1200               (9U  << 0)
#define ADS1263_MODE2_DR_2400               (10U << 0)
#define ADS1263_MODE2_DR_4800               (11U << 0)
#define ADS1263_MODE2_DR_7200               (12U << 0)
#define ADS1263_MODE2_DR_14400              (13U << 0)
#define ADS1263_MODE2_DR_19200              (14U << 0)
#define ADS1263_MODE2_DR_38400              (15U << 0)

// REFMUX Register (Bits7:4  RMUXP, Bits3:0  RMUXN)
#define ADS1263_REFMUX_P_INT_2_5V              (0U << 3)
#define ADS1263_REFMUX_P_EXT_AIN0              (1U << 3)
#define ADS1263_REFMUX_P_EXT_AIN2              (2U << 3)
#define ADS1263_REFMUX_P_EXT_AIN4              (3U << 3)
#define ADS1263_REFMUX_P_INT_AVDD              (4U << 3)

#define ADS1263_REFMUX_N_INT_2_5V              (0U << 0)
#define ADS1263_REFMUX_N_EXT_AIN0              (1U << 0)
#define ADS1263_REFMUX_N_EXT_AIN2              (2U << 0)
#define ADS1263_REFMUX_N_EXT_AIN4              (3U << 0)
#define ADS1263_REFMUX_N_INT_AVSS              (4U << 0)

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_ESF_DRIVERS_ADC_ADS1263_H */
