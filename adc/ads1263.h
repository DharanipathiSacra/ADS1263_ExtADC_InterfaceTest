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

/* PGA Gain */
#define ADS1263_MODE2_GAIN_1          (0x0u << 4)
#define ADS1263_MODE2_GAIN_2          (0x1u << 4)
#define ADS1263_MODE2_GAIN_4          (0x2u << 4)
#define ADS1263_MODE2_GAIN_8          (0x3u << 4)
#define ADS1263_MODE2_GAIN_16         (0x4u << 4)
#define ADS1263_MODE2_GAIN_32         (0x5u << 4)

/* PGA Bypass Bit */
#define ADS1263_MODE2_PGA_BYPASS      BIT(7)

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_ESF_DRIVERS_ADC_ADS1263_H */
