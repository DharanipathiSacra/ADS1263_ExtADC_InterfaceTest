/*
 * Copyright 2026 Sacra Systems Private Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc/ads126x.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define ADS126X_ADC1_RESOLUTION 32U
#define ADS126X_REF_INTERNAL    2500 /*< Internal reference voltage in mV */

#define ADS126X_DRDY_TIMEOUT_US 10000U
#define ADS126X_DRDY_POLL_US    10U

/* System Commands */
#define ADS126X_CMD_NOP   0x00
#define ADS126X_CMD_RESET 0x06

/* ADC1 Commands */
#define ADS126X_CMD_START1  0x08
#define ADS126X_CMD_STOP1   0x0A
#define ADS126X_CMD_RDATA1  0x12
#define ADS126X_CMD_SFOCAL1 0x19
#define ADS126X_CMD_SFOCAL2 0x1E /* ADS1263 only */

/* ADC2 Commands */
#define ADS126X_CMD_START2 0x0C
#define ADS126X_CMD_STOP2  0x0E
#define ADS126X_CMD_RDATA2 0x14

/* Register Commands */
#define ADS126X_CMD_RREG 0x20
#define ADS126X_CMD_WREG 0x40

/* Power Register */
#define ADS126X_POWER_INTREF BIT(0)

/* INTERFACE register */
#define ADS126X_INTF_STATUS   BIT(2)
#define ADS126X_INTF_CRC_MASK 0x03
#define ADS126X_INTF_CRC_NONE 0x00
#define ADS126X_INTF_CRC_CHK  0x01
#define ADS126X_INTF_CRC_CRC  0x02

/* RDATA1: cmd(1) + status(1) + data(4) + crc(1) = 7 bytes max */
#define ADS126X_RDATA1_NO_STATUS_NO_CRC 5 /* cmd + 4 bytes data */
#define ADS126X_RDATA1_STATUS_NO_CRC    6
#define ADS126X_RDATA1_STATUS_CRC       7

/* RDATA2: cmd(1) + status(1) + data(3) + crc(1) = 6 bytes max */
#define ADS1263_RDATA2_NO_STATUS_NO_CRC 4
#define ADS1263_RDATA2_STATUS_NO_CRC    5
#define ADS1263_RDATA2_STATUS_CRC       6

/* MODE1 - Filter */
#define ADS126X_MODE1_FILTER_MASK 0xE0

/* MODE2 - PGA / Data Rate */
#define ADS126X_MODE2_BYPASS     BIT(7)
#define ADS126X_MODE2_GAIN_MASK  0x70u
#define ADS126X_MODE2_GAIN_SHIFT 4
#define ADS126X_MODE2_DR_MASK    0x0F

/* ADC2CFG (ADS1263 only) */
#define ADS1263_ADC2CFG_REF_MASK  0xE0
#define ADS1263_ADC2CFG_GAIN_MASK 0x1C
#define ADS1263_ADC2CFG_DR_MASK   0x03
#define ADS1263_REG_ADC2CFG       0x15 /* ADS1263 only */
#define ADS1263_REG_ADC2MUX       0x16 /* ADS1263 only */

#define ADS126X_START_DELAY_US 1000U

/* Register Addresses */
#define ADS126X_REG_ID        0x00
#define ADS126X_REG_POWER     0x01
#define ADS126X_REG_INTERFACE 0x02
#define ADS126X_REG_MODE0     0x03
#define ADS126X_REG_MODE1     0x04
#define ADS126X_REG_MODE2     0x05
#define ADS126X_REG_INPMUX    0x06
#define ADS126X_REG_OFCAL0    0x07
#define ADS126X_REG_OFCAL1    0x08
#define ADS126X_REG_OFCAL2    0x09
#define ADS126X_REG_FSCAL0    0x0A
#define ADS126X_REG_FSCAL1    0x0B
#define ADS126X_REG_FSCAL2    0x0C
#define ADS126X_REG_IDACMUX   0x0D
#define ADS126X_REG_IDACMAG   0x0E
#define ADS126X_REG_REFMUX    0x0F
#define ADS126X_REG_ADC2CFG   0x15 /* ADS1263 only */
#define ADS126X_REG_ADC2MUX   0x16 /* ADS1263 only */

#define ADS126X_INTERFACE_STATUS_ENABLE BIT(2)

#define ADS126X_RDATA_TIMEOUT_MS 500U

/*TDAC*/
#define ADS126X_TDAC_CHANNEL_MIN 0U
#define ADS126X_TDAC_CHANNEL_MAX 7U

#define ADS126X_TDAC_LEVEL_MIN 0x00U
#define ADS126X_TDAC_LEVEL_MAX 0x19U

#define ADS126X_REG_TDACP 0x10
#define ADS126X_REG_TDACN 0x11

/* TDAC Output Channel (Bits 7:5) */

#define ADS126X_TDAC_OUT_AIN0 (0x0U << 5)
#define ADS126X_TDAC_OUT_AIN1 (0x1U << 5)
#define ADS126X_TDAC_OUT_AIN2 (0x2U << 5)
#define ADS126X_TDAC_OUT_AIN3 (0x3U << 5)
#define ADS126X_TDAC_OUT_AIN4 (0x4U << 5)
#define ADS126X_TDAC_OUT_AIN5 (0x5U << 5)
#define ADS126X_TDAC_OUT_AIN6 (0x6U << 5)
#define ADS126X_TDAC_OUT_AIN7 (0x7U << 5)

/* TDAC Output Magnitude (MAGP / MAGN), Bits [4:0], Output voltage with respect to VAVSS */

#define ADS126X_TDAC_2V500000 0x00U /* 2.500000 V */
#define ADS126X_TDAC_2V507812 0x01U /* 2.5078125 V */
#define ADS126X_TDAC_2V515625 0x02U /* 2.515625 V */
#define ADS126X_TDAC_2V531250 0x03U /* 2.531250 V */
#define ADS126X_TDAC_2V562500 0x04U /* 2.562500 V */
#define ADS126X_TDAC_2V625000 0x05U /* 2.625000 V */
#define ADS126X_TDAC_2V750000 0x06U /* 2.750000 V */
#define ADS126X_TDAC_3V000000 0x07U /* 3.000000 V */
#define ADS126X_TDAC_3V500000 0x08U /* 3.500000 V */
#define ADS126X_TDAC_4V500000 0x09U /* 4.500000 V */

#define ADS126X_TDAC_2V492187 0x11U /* 2.4921875 V */
#define ADS126X_TDAC_2V484375 0x12U /* 2.484375 V */
#define ADS126X_TDAC_2V468750 0x13U /* 2.468750 V */
#define ADS126X_TDAC_2V437500 0x14U /* 2.437500 V */
#define ADS126X_TDAC_2V375000 0x15U /* 2.375000 V */
#define ADS126X_TDAC_2V250000 0x16U /* 2.250000 V */
#define ADS126X_TDAC_2V000000 0x17U /* 2.000000 V */
#define ADS126X_TDAC_1V500000 0x18U /* 1.500000 V */
#define ADS126X_TDAC_0V500000 0x19U /* 0.500000 V */

#define ADS126X_TDAC_LEVEL(x) ((x) & 0x1F)

#define ADS126X_ADC2_OFFSET 16
#define ADS126X_IS_ADC2(ch) ((ch) >= ADS126X_ADC2_OFFSET)
#define ADS126X_HW_CH(ch)   ((ch) & 0x0F)

LOG_MODULE_REGISTER(adc_ads1263, CONFIG_ADC_LOG_LEVEL);

struct ads126x_config {
	struct spi_dt_spec bus;

	struct gpio_dt_spec drdy_gpio;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec start_gpio;

	enum ads126x_chip_id chip_id;

	uint8_t adc1_data_rate; /* MODE2 DR field value */
	uint8_t adc1_gain;      /* MODE2 GAIN field value */
	uint8_t adc1_filter;    /* MODE1 FILTER field value */
	uint8_t adc1_ref_mux;   /* REFMUX encoded value */
	bool adc1_pga_bypass;   /* PGA bypass for low noise */
	bool internal_vref;     /* Use internal 2.5V reference */

	/* ADC2 defaults (ADS1263 only) */
	uint8_t adc2_data_rate; /* ADC2CFG DR field value */
	uint8_t adc2_gain;      /* ADC2CFG GAIN field value */
	uint8_t adc2_ref_mux;   /* ADC2CFG REF field value */

	/* Interface options */
	uint8_t crc_mode; /* INTERFACE.CRC_EN field */
	bool status_byte; /* INTERFACE.STATUS enable */

	/* Reference voltage in mV (for Vref = AVDD-AVSS if external) */
	uint32_t vref_mv;
};

struct ads126x_data {

	struct adc_context ctx;

	const struct device *dev;

	int32_t *buffer;
	int32_t *repeat_buffer;

	struct k_mutex lock;

	struct k_sem drdy_sem;

	int32_t sample;

	uint8_t interface_reg;

	/* Cached ADC configuration */
	uint8_t positive_input;

	uint8_t negative_input;

	enum adc_gain gain;

	enum adc_reference reference;

	uint8_t channel_id;

	bool status_enabled;

	bool crc_enabled;

	bool checksum_enabled;

	bool config_dirty;

	uint8_t cached_mode2;

	uint8_t cached_inpmux;
};

static int ads126x_spi_write(const struct device *dev, const uint8_t *tx_buf, size_t len)
{
	const struct ads126x_config *config = dev->config;

	struct spi_buf tx_bufs = {
		.buf = (void *)tx_buf,
		.len = len,
	};

	struct spi_buf_set tx = {
		.buffers = &tx_bufs,
		.count = 1,
	};

	int ret = spi_write_dt(&config->bus, &tx);

	return ret;
}

static int ads126x_spi_transceive(const struct device *dev, const uint8_t *tx_buf, uint8_t *rx_buf,
				  size_t len)
{
	const struct ads126x_config *config = dev->config;

	struct spi_buf tx = {
		.buf = (void *)tx_buf,
		.len = len,
	};

	struct spi_buf rx = {
		.buf = rx_buf,
		.len = len,
	};

	struct spi_buf_set tx_set = {
		.buffers = &tx,
		.count = 1,
	};

	struct spi_buf_set rx_set = {
		.buffers = &rx,
		.count = 1,
	};

	int ret = spi_transceive_dt(&config->bus, &tx_set, &rx_set);

	return ret;
}

static int ads126x_send_command(const struct device *dev, uint8_t command)
{
	return ads126x_spi_write(dev, &command, 1);
}

static int ads126x_read_reg(const struct device *dev, uint8_t reg, uint8_t *value)
{
	uint8_t tx[3] = {ADS126X_CMD_RREG | reg, 0x00, 0x00};

	uint8_t rx[3];

	int ret = ads126x_spi_transceive(dev, tx, rx, sizeof(tx));

	if (ret) {
		return ret;
	}

	*value = rx[2];

	return 0;
}

static int ads126x_write_reg(const struct device *dev, uint8_t reg, uint8_t value)
{
	uint8_t tx[3] = {ADS126X_CMD_WREG | reg, 0x00, value};

	return ads126x_spi_write(dev, tx, sizeof(tx));
}

static int ads126x_wait_drdy(const struct device *dev)
{
	/* 	int ret; */

	struct ads126x_data *data = dev->data;
	const struct ads126x_config *config = dev->config;

	// LOG_INF("R9a");

	/* 	k_sem_reset(&data->drdy_sem);
		ret = k_sem_take(&data->drdy_sem, K_MSEC(ADS126X_RDATA_TIMEOUT_MS)); */

	// LOG_INF("R9b");

	// Need to add interupt based drdy wait, for now polling is used

	uint32_t elapsed = 0U;

	while (1) {

		int ret = gpio_pin_get_dt(&config->drdy_gpio);

		if (ret < 0) {
			return ret;
		}

		if (ret == 0) {
			return 0;
		}

		if (elapsed >= ADS126X_DRDY_TIMEOUT_US) {
			return -ETIMEDOUT;
		}

		k_busy_wait(ADS126X_DRDY_POLL_US);
		elapsed += ADS126X_DRDY_POLL_US;
	}

	return 0;
}

static int ads126x_reset(const struct device *dev)
{
	const struct ads126x_config *config = dev->config;

	if (config->reset_gpio.port == NULL) {
		return 0;
	}

	gpio_pin_set_dt(&config->reset_gpio, 0);

	k_msleep(5);

	gpio_pin_set_dt(&config->reset_gpio, 1);

	k_msleep(10);

	return 0;
}

static int ads126x_verify_id(const struct device *dev)
{
	const struct ads126x_config *config = dev->config;

	uint8_t id;
	int ret;

	ret = ads126x_read_reg(dev, ADS126X_REG_ID, &id);
	if (ret) {
		LOG_ERR("Failed to read device ID: %d", ret);
		return ret;
	}

	uint8_t dev_id = id & ADS126X_ID_DEV_MASK;

	LOG_DBG("ADS126x Device ID register: 0x%02X", id);

	if (config->chip_id == ADS126X_CHIP_ADS1262 && dev_id != ADS126X_ID_ADS1262) {
		LOG_ERR("Expected ADS1262 (0x00) but got 0x%02X", dev_id);
		return -EINVAL;
	}
	if (config->chip_id == ADS126X_CHIP_ADS1263 && dev_id != ADS126X_ID_ADS1263) {
		LOG_ERR("Expected ADS1263 (0x20) but got 0x%02X", dev_id);
		return -EINVAL;
	}

	LOG_INF("ADS126x ID verified: 0x%02X", id);
	return 0;
}
static int ads126x_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{

	LOG_INF("ads126x_channel_setup Start");
	const struct ads126x_config *config = dev->config;

	if (channel_cfg->channel_id >= ADS126X_MAX_CHANNELS) {
		LOG_ERR("Channel ID %d exceeds maximum %d", channel_cfg->channel_id,
			ADS126X_MAX_CHANNELS - 1);
		return -EINVAL;
	}

	/* ADC2 channels only on ADS1263 */
	if (channel_cfg->channel_id >= ADS126X_ADC2_CHANNEL_OFFSET &&
	    config->chip_id != ADS126X_CHIP_ADS1263) {
		LOG_ERR("ADC2 channels require ADS1263");
		return -ENOTSUP;
	}

	/* Gain validation: ADC1 supports 1, 2, 4, 8, 16, 32 */
	switch (channel_cfg->gain) {
	case ADC_GAIN_1:
	case ADC_GAIN_2:
	case ADC_GAIN_4:
	case ADC_GAIN_8:
	case ADC_GAIN_16:
	case ADC_GAIN_32:
		break;
	default:
		LOG_ERR("Unsupported gain");
		return -EINVAL;
	}

	/* Reference validation */
	if (channel_cfg->reference == ADC_REF_INTERNAL) {
		if (!config->internal_vref) {
			LOG_ERR("Internal reference not enabled in DTS");
			return -EINVAL;
		}
	}

	LOG_DBG("Channel %d configured", channel_cfg->channel_id);
	LOG_INF("ads126x_channel_setup End");

	return 0;
}

static int ads126x_set_mux1(const struct device *dev, uint8_t muxp, uint8_t muxn)
{
	uint8_t val = ((muxp & 0x0F) << 4) | (muxn & 0x0F);
	LOG_INF("R7");

	return ads126x_write_reg(dev, ADS126X_REG_INPMUX, val);
}

static int ads126x_set_mux2(const struct device *dev, uint8_t muxp, uint8_t muxn)
{
	const struct ads126x_config *config = dev->config;

	if (config->chip_id != ADS126X_CHIP_ADS1263) {
		return -ENOTSUP;
	}

	uint8_t val = ((muxp & 0x0F) << 4) | (muxn & 0x0F);

	return ads126x_write_reg(dev, ADS126X_REG_ADC2MUX, val);
}

static uint8_t ads126x_crc8(const uint8_t *data, size_t len)
{
	uint8_t crc = 0xFF;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (int b = 0; b < 8; b++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

static int ads126x_rdata1(const struct device *dev, int32_t *result, uint8_t *status_out)
{
	const struct ads126x_config *config = dev->config;
	uint8_t rx[7] = {0};
	uint8_t tx[7] = {ADS126X_CMD_RDATA1, 0, 0, 0, 0, 0, 0};
	size_t frame_len;
	uint8_t data_offset;

	LOG_INF("R11");

	/* Determine frame length */
	if (config->status_byte && config->crc_mode != ADS126X_INTF_CRC_NONE) {
		frame_len = ADS126X_RDATA1_STATUS_CRC;
		data_offset = 2; /* after cmd byte + status */
	} else if (config->status_byte) {
		frame_len = ADS126X_RDATA1_STATUS_NO_CRC;
		data_offset = 2;
	} else {
		frame_len = ADS126X_RDATA1_NO_STATUS_NO_CRC;
		data_offset = 1;
	}

	int ret = ads126x_spi_transceive(dev, tx, rx, frame_len);

	if (ret) {
		return ret;
	}

	if (status_out) {
		*status_out = config->status_byte ? rx[1] : 0x00;
	}

	LOG_INF("R12");
	/* Verify CRC if enabled */
	if (config->crc_mode == ADS126X_INTF_CRC_CRC) {
		uint8_t calc = ads126x_crc8(&rx[1], frame_len - 2);

		if (calc != rx[frame_len - 1]) {
			LOG_ERR("RDATA1 CRC mismatch: expected 0x%02X got 0x%02X", calc,
				rx[frame_len - 1]);
			return -EIO;
		}
	} else if (config->crc_mode == ADS126X_INTF_CRC_CHK) {
		/* Checksum: simple XOR byte */
		uint8_t chk = 0x9B;

		for (int i = data_offset; i < (int)(frame_len - 1); i++) {
			chk += rx[i];
		}

		if (chk != rx[frame_len - 1]) {
			LOG_ERR("RDATA1 checksum mismatch");
			return -EIO;
		}
	}

	/* Reconstruct 32-bit signed integer (big-endian on wire) */
	*result = (int32_t)((uint32_t)rx[data_offset] << 24 | (uint32_t)rx[data_offset + 1] << 16 |
			    (uint32_t)rx[data_offset + 2] << 8 | (uint32_t)rx[data_offset + 3]);

	double voltage = ((double)((*result) * 2.5f)) / ((double)(INT32_MAX * 1.0f));

	LOG_INF("R13 result: %d, voltage: %.2f mV", *result, voltage);

	return 0;
}

static int ads126x_rdata2(const struct device *dev, int32_t *result, uint8_t *status_out)
{
	const struct ads126x_config *config = dev->config;

	if (config->chip_id != ADS126X_CHIP_ADS1263) {
		return -ENOTSUP;
	}

	uint8_t rx[6] = {0};
	uint8_t tx[6] = {ADS126X_CMD_RDATA2, 0, 0, 0, 0, 0};
	size_t frame_len;
	uint8_t data_offset;

	if (config->status_byte && config->crc_mode != ADS126X_INTF_CRC_NONE) {
		frame_len = ADS1263_RDATA2_STATUS_CRC;
		data_offset = 2;
	} else if (config->status_byte) {
		frame_len = ADS1263_RDATA2_STATUS_NO_CRC;
		data_offset = 2;
	} else {
		frame_len = ADS1263_RDATA2_NO_STATUS_NO_CRC;
		data_offset = 1;
	}

	int ret = ads126x_spi_transceive(dev, tx, rx, frame_len);

	if (ret) {
		return ret;
	}

	if (status_out) {
		*status_out = config->status_byte ? rx[1] : 0x00;
	}

	if (config->crc_mode == ADS126X_INTF_CRC_CRC) {
		uint8_t calc = ads126x_crc8(&rx[1], frame_len - 2);

		if (calc != rx[frame_len - 1]) {
			LOG_ERR("RDATA2 CRC mismatch");
			return -EIO;
		}
	}

	/* 24-bit two's complement → sign-extend to 32-bit */
	uint32_t raw = ((uint32_t)rx[data_offset] << 16 | (uint32_t)rx[data_offset + 1] << 8 |
			(uint32_t)rx[data_offset + 2]);

	/* Sign extension from bit 23 */
	if (raw & BIT(23)) {
		raw |= 0xFF000000U;
	}
	*result = (int32_t)raw;

	return 0;
}

static int ads126x_read_channel_adc1(const struct device *dev, uint8_t channel, int32_t *result)
{
	const struct ads126x_config *config = dev->config;

	int ret;

	/* Mux mapping: single-ended channels 0..9 */
	uint8_t muxp, muxn;

	// LOG_INF("R6");

	if (channel < 10) {
		muxp = channel;
		muxn = ADS126X_MUXN_AINCOM;
	} else if (channel < 15) {
		/* Differential pairs */
		muxp = (channel - 10) * 2;
		muxn = muxp + 1;
	} else {
		LOG_ERR("ADC1 channel %d out of range", channel);
		return -EINVAL;
	}

	ret = ads126x_set_mux1(dev, muxp, muxn);
	if (ret) {
		return ret;
	}

	/* Start single conversion */
	ret = ads126x_send_command(dev, ADS126X_CMD_START1);
	if (ret) {
		return ret;
	}

	/* Wait for DRDY */
	LOG_INF("R8");
	if (config->drdy_gpio.port != NULL) {
		ret = ads126x_wait_drdy(dev);
		LOG_INF("R9");
		if (ret) {
			LOG_ERR("ADC1 DRDY timeout on channel %d", channel);
			ads126x_send_command(dev, ADS126X_CMD_STOP1);
			return -ETIMEDOUT;
		}
	} else {
		/* Poll DRDY: in continuous mode DRDY goes low when data ready.*/
		/* In pulse mode read after a fixed delay based on ODR. */
		k_sleep(K_USEC(1000));
	}

	LOG_INF("R10");

	ret = ads126x_rdata1(dev, result, NULL);
	if (ret) {
		return ret;
	}

	/* Stop ADC1 (pulse mode: auto-stops, but stop for safety) */
	ads126x_send_command(dev, ADS126X_CMD_STOP1);

	return 0;
}

static int ads126x_read_channel_adc2(const struct device *dev, uint8_t channel, int32_t *result)
{
	const struct ads126x_config *config = dev->config;

	int ret;

	if (config->chip_id != ADS126X_CHIP_ADS1263) {
		return -ENOTSUP;
	}

	if (channel > 7) {
		LOG_ERR("ADC2 channel %d out of range", channel);
		return -EINVAL;
	}

	ret = ads126x_set_mux2(dev, channel, ADS126X_MUXN_AINCOM);
	if (ret) {
		return ret;
	}

	ret = ads126x_send_command(dev, ADS126X_CMD_START2);
	if (ret) {
		return ret;
	}

	if (config->drdy_gpio.port != NULL) {

		ret = ads126x_wait_drdy(dev);

		if (ret) {
			LOG_ERR("ADC2 DRDY timeout on channel %d", channel);
			ads126x_send_command(dev, ADS126X_CMD_STOP2);
			return -ETIMEDOUT;
		}
	} else {
		k_sleep(K_USEC(2000));
	}

	ret = ads126x_rdata2(dev, result, NULL);
	if (ret) {
		return ret;
	}

	ads126x_send_command(dev, ADS126X_CMD_STOP2);

	return 0;
}

static int ads126x_perform_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct ads126x_data *data = dev->data;
	uint32_t channels = sequence->channels;
	int32_t *buf = data->buffer;
	int buf_idx = 0;
	int ret = 0;

	// LOG_INF("R4");
	while (channels) {
		uint8_t ch = find_lsb_set(channels) - 1;

		channels &= ~BIT(ch);

		LOG_INF("Channels 0x%X, reading channel %d", channels, ch);

		int32_t result = 0;

		if (ch < ADS126X_ADC2_CHANNEL_OFFSET) {
			/* ADC1 channel */
			// LOG_INF("R5");
			ret = ads126x_read_channel_adc1(dev, ch, &result);
		} else {
			/* ADC2 channel (ADS1263 only) */
			uint8_t adc2_ch = ch - ADS126X_ADC2_CHANNEL_OFFSET;

			ret = ads126x_read_channel_adc2(dev, adc2_ch, &result);
		}

		if (ret) {
			LOG_ERR("Channel %d read failed: %d", ch, ret);
			return ret;
		}

		buf[buf_idx++] = result;
		LOG_DBG("ch=%d raw=0x%08X (%d)", ch, (uint32_t)result, result);
	}

	return 0;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ads126x_data *data = CONTAINER_OF(ctx, struct ads126x_data, ctx);

	int ret = ads126x_perform_read(data->dev, &ctx->sequence);

	if (ret) {
		LOG_ERR("ads126x_perform_read failed: %d", ret);
	}

	adc_context_on_sampling_done(ctx, data->dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct ads126x_data *data = CONTAINER_OF(ctx, struct ads126x_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static int ads126x_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct ads126x_data *data = dev->data;

	/* Set buffer pointer */
	data->buffer = sequence->buffer;
	// LOG_INF("R3");

	/* Start ADC context */
	adc_context_start_read(&data->ctx, sequence);
	return 0;
}

static int ads126x_read(const struct device *dev, const struct adc_sequence *sequence)
{
	LOG_INF("ads126x_read called");
	struct ads126x_data *data = dev->data;
	int ret;

	ret = k_mutex_lock(&data->lock, K_FOREVER);
	if (ret) {
		return ret;
	}

	adc_context_lock(&data->ctx, false, NULL);
	// LOG_INF("R2");

	ret = ads126x_start_read(dev, sequence);

	while ((ret == 0) && k_sem_take(&data->ctx.sync, K_NO_WAIT) != 0) {
		ret = ads126x_perform_read(dev, sequence);
	}

	// LOG_INF("R14");

	ret = adc_context_wait_for_completion(&data->ctx);
	adc_context_release(&data->ctx, ret);

	k_mutex_unlock(&data->lock);
	return ret;
}

static int ads126x_config_power(const struct device *dev)
{
	const struct ads126x_config *config = dev->config;
	uint8_t val = 0;

	if (config->internal_vref) {
		val |= ADS126X_POWER_INTREF;
	}

	return ads126x_write_reg(dev, ADS126X_REG_POWER, val);
}

static int ads126x_config_interface(const struct device *dev)
{
	const struct ads126x_config *config = dev->config;
	uint8_t val = 0;

	if (config->status_byte) {
		val |= ADS126X_INTF_STATUS;
	}
	val |= (config->crc_mode & ADS126X_INTF_CRC_MASK);

	return ads126x_write_reg(dev, ADS126X_REG_INTERFACE, val);
}

static int ads126x_config_adc1(const struct device *dev)
{
	const struct ads126x_config *config = dev->config;
	int ret;

	/* MODE0: pulse conversion, default delay */
	ret = ads126x_write_reg(dev, ADS126X_REG_MODE0, 0x00);
	if (ret) {
		return ret;
	}

	uint8_t mode1 = (config->adc1_filter << 5) & ADS126X_MODE1_FILTER_MASK;

	// LOG_INF("config->adc1_filter = 0x%02X, mode1 = 0x%02X", config->adc1_filter, mode1);

	/* MODE1: filter selection */
	ret = ads126x_write_reg(dev, ADS126X_REG_MODE1, mode1);
	if (ret) {
		return ret;
	}

	/* MODE2: PGA gain + data rate */
	uint8_t mode2 = 0;

	if (config->adc1_pga_bypass) {
		mode2 |= ADS126X_MODE2_BYPASS;
	}

	mode2 |= ((config->adc1_gain << ADS126X_MODE2_GAIN_SHIFT) & ADS126X_MODE2_GAIN_MASK);
	mode2 |= (config->adc1_data_rate & ADS126X_MODE2_DR_MASK);

	ret = ads126x_write_reg(dev, ADS126X_REG_MODE2, mode2);
	if (ret) {
		return ret;
	}

	/* REFMUX */
	ret = ads126x_write_reg(dev, ADS126X_REG_REFMUX, config->adc1_ref_mux);

	return ret;
}

static int ads126x_config_adc2(const struct device *dev)
{
	const struct ads126x_config *config = dev->config;

	if (config->chip_id != ADS126X_CHIP_ADS1263) {
		return 0;
	}

	/* ADC2CFG: reference + gain + data rate */
	uint8_t adc2cfg = 0;

	adc2cfg |= (config->adc2_ref_mux << 5) & ADS1263_ADC2CFG_REF_MASK;
	adc2cfg |= (config->adc2_gain << 2) & ADS1263_ADC2CFG_GAIN_MASK;
	adc2cfg |= (config->adc2_data_rate) & ADS1263_ADC2CFG_DR_MASK;

	int ret = ads126x_write_reg(dev, ADS1263_REG_ADC2CFG, adc2cfg);

	if (ret) {
		return ret;
	}

	/* ADC2MUX: default AIN6 vs AIN7 (differential pair) */
	return ret;
}

int ads126x_selfoffset_cal_adc1(const struct device *dev)
{
	int ret = ads126x_send_command(dev, ADS126X_CMD_SFOCAL1);

	if (ret) {
		return ret;
	}

	k_sleep(K_MSEC(10)); /* Wait calibration to complete */

	LOG_INF("ADC1 self-offset calibration done");

	return 0;
}

int ads126x_selfoffset_cal_adc2(const struct device *dev)
{
	const struct ads126x_config *config = dev->config;

	if (config->chip_id != ADS126X_CHIP_ADS1263) {
		return -ENOTSUP;
	}

	int ret = ads126x_send_command(dev, ADS126X_CMD_SFOCAL2);

	if (ret) {
		return ret;
	}

	k_sleep(K_MSEC(10));

	LOG_INF("ADC2 self-offset calibration done");

	return 0;
}

static int ads126x_init(const struct device *dev)
{
	struct ads126x_data *data = dev->data;
	const struct ads126x_config *config = dev->config;

	data->dev = dev;

	int ret;

	k_sem_init(&data->drdy_sem, 0, 1);
	k_mutex_init(&data->lock);

	adc_context_init(&data->ctx);

	/* Verify peripherals */

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI not ready");
		return -ENODEV;
	}

	if (config->drdy_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->drdy_gpio)) {
			LOG_ERR("DRDY GPIO not ready");
			return -ENODEV;

			ret = gpio_pin_configure_dt(&config->drdy_gpio, GPIO_INPUT);

			if (ret) {
				return ret;
			}
		}
	}

	if (config->reset_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->reset_gpio)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->reset_gpio, (GPIO_OUTPUT_HIGH));

		if (ret) {
			return ret;
		}

		gpio_pin_set_dt(&config->reset_gpio, 1);
	}

	/* Reset device */

	ret = ads126x_reset(dev);

	if (ret) {
		LOG_ERR("Failed to reset ADS1263");
		return ret;
	}

	/* Read and validate device ID */

	ret = ads126x_verify_id(dev);
	if (ret) {
		return ret;
	}

	/* Stop ADC1 before configuration */

	ret = ads126x_send_command(dev, ADS126X_CMD_STOP1);

	if (ret) {
		return ret;
	}

	/* --- Configure registers --- */
	ret = ads126x_config_power(dev);
	if (ret) {
		return ret;
	}

	ret = ads126x_config_interface(dev);
	if (ret) {
		return ret;
	}

	ret = ads126x_config_adc1(dev);
	if (ret) {
		return ret;
	}

	/* ret = ads126x_selfoffset_cal_adc1(dev); */
	if (ret) {
		LOG_WRN("ADC1 self-offset cal failed: %d (non-fatal)", ret);
	}

	if ((config->chip_id == ADS126X_CHIP_ADS1263) && 0) {
		ret = ads126x_config_adc2(dev);
		if (ret) {
			return ret;
		}

		ret = ads126x_selfoffset_cal_adc2(dev);
		if (ret) {
			LOG_WRN("ADC2 self-offset cal failed: %d (non-fatal)", ret);
		}
	}

	/* adc context unlock */
	adc_context_unlock_unconditionally(&data->ctx);

	LOG_INF("ADS126x (%s) initialised",
		config->chip_id == ADS126X_CHIP_ADS1263 ? "ADS1263" : "ADS1262");

	return 0;
}

static DEVICE_API(adc, ads126x_driver_api) = {
	.channel_setup = ads126x_channel_setup,
	.read = ads126x_read,
	.ref_internal = ADS126X_REF_INTERNAL,
};

#define ADS126X_INIT(inst)                                                                         \
	static const struct ads126x_config config_##inst = {                                       \
		.bus = SPI_DT_SPEC_INST_GET(inst,                                                  \
					    SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA),   \
		.drdy_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drdy_gpios, {0}),                      \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                    \
		.start_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, start_gpios, {0}),                    \
		.chip_id = DT_INST_PROP_OR(inst, chip_id, 0),                                      \
		.adc1_data_rate = DT_INST_PROP(inst, adc1_data_rate),                              \
		.adc1_gain = DT_INST_PROP(inst, adc1_gain),                                        \
		.adc1_filter = DT_INST_PROP(inst, adc1_filter),                                    \
		.adc1_ref_mux = DT_INST_PROP(inst, adc1_ref_mux),                                  \
		.adc1_pga_bypass = DT_INST_PROP(inst, adc1_pga_bypass),                            \
		.internal_vref = DT_INST_PROP(inst, internal_vref),                                \
		.adc2_data_rate = DT_INST_PROP_OR(inst, adc2_data_rate, 0),                        \
		.adc2_gain = DT_INST_PROP_OR(inst, adc2_gain, 0),                                  \
		.adc2_ref_mux = DT_INST_PROP_OR(inst, adc2_ref_mux, 0),                            \
		.crc_mode = DT_INST_PROP(inst, crc_mode),                                          \
		.status_byte = DT_INST_PROP(inst, status_byte),                                    \
		.vref_mv = DT_INST_PROP(inst, vref_mv),                                            \
	};                                                                                         \
	static struct ads126x_data data_##inst = {                                                 \
		ADC_CONTEXT_INIT_LOCK(data_##inst, ctx),                                           \
		ADC_CONTEXT_INIT_TIMER(data_##inst, ctx),                                          \
		ADC_CONTEXT_INIT_SYNC(data_##inst, ctx),                                           \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, ads126x_init, NULL, &data_##inst, &config_##inst, POST_KERNEL, \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ads126x_driver_api);

#define DT_DRV_COMPAT ti_ads1262

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY(ADS126X_INIT)
#endif

#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT ti_ads1263

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY(ADS126X_INIT)
#endif

#undef DT_DRV_COMPAT
