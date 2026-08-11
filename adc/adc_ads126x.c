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

#define ADS126X_INTERFACE_STATUS_ENABLE BIT(2)
#define ADS126X_INTERFACE_CRC_MASK      0x03
#define ADS126X_INTERFACE_CRC_CRC       0x01
#define ADS126X_INTERFACE_CRC_CHECKSUM  0x02

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

	struct k_mutex lock;

	int32_t sample;

	uint8_t device_id;

	uint8_t interface_reg;

	bool initialized;

	/* Cached ADC configuration */
	uint8_t positive_input;

	uint8_t negative_input;

	bool differential;

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

	struct ads126x_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

	struct spi_buf tx_bufs = {
		.buf = (void *)tx_buf,
		.len = len,
	};

	struct spi_buf_set tx = {
		.buffers = &tx_bufs,
		.count = 1,
	};

	int ret = spi_write_dt(&config->bus, &tx);

	k_mutex_unlock(&data->lock);

	return ret;
}

static int ads126x_spi_transceive(const struct device *dev, const uint8_t *tx_buf, uint8_t *rx_buf,
				  size_t len)
{
	const struct ads126x_config *config = dev->config;

	struct ads126x_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);

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

	k_mutex_unlock(&data->lock);

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

static int ads126x_update_reg(const struct device *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t reg_val;

	int ret;

	ret = ads126x_read_reg(dev, reg, &reg_val);

	if (ret) {
		return ret;
	}

	reg_val &= ~mask;

	reg_val |= (value & mask);

	return ads126x_write_reg(dev, reg, reg_val);
}

static int ads126x_select_channel(const struct device *dev, uint8_t positive, uint8_t negative)
{
	uint8_t mux = 0;

	switch (positive) {
	case 0:
		mux = ADS126X_MUXP_AIN0;
		break;
	case 1:
		mux = ADS126X_MUXP_AIN1;
		break;
	case 2:
		mux = ADS126X_MUXP_AIN2;
		break;
	case 3:
		mux = ADS126X_MUXP_AIN3;
		break;
	case 4:
		mux = ADS126X_MUXP_AIN4;
		break;
	case 5:
		mux = ADS126X_MUXP_AIN5;
		break;
	case 6:
		mux = ADS126X_MUXP_AIN6;
		break;
	case 7:
		mux = ADS126X_MUXP_AIN7;
		break;
	case 8:
		mux = ADS126X_MUXP_AIN8;
		break;
	case 9:
		mux = ADS126X_MUXP_AIN9;
		break;
	case 10:
		mux = ADS126X_MUXP_AINCOM;
		break;
	case 11:
		mux = ADS126X_MUXP_TEMP;
		break;
	case 12:
		mux = ADS126X_MUXP_AVDD;
		break;
	case 13:
		mux = ADS126X_MUXP_DVDD;
		break;
	case 14:
		mux = ADS126X_MUXP_TDAC;
		break;
	case 15:
		mux = ADS126X_MUXP_FLOAT;
		break;
	default:
		return -EINVAL;
	}

	switch (negative) {
	case 0:
		mux |= ADS126X_MUXN_AIN0;
		break;
	case 1:
		mux |= ADS126X_MUXN_AIN1;
		break;
	case 2:
		mux |= ADS126X_MUXN_AIN2;
		break;
	case 3:
		mux |= ADS126X_MUXN_AIN3;
		break;
	case 4:
		mux |= ADS126X_MUXN_AIN4;
		break;
	case 5:
		mux |= ADS126X_MUXN_AIN5;
		break;
	case 6:
		mux |= ADS126X_MUXN_AIN6;
		break;
	case 7:
		mux |= ADS126X_MUXN_AIN7;
		break;
	case 8:
		mux |= ADS126X_MUXN_AIN8;
		break;
	case 9:
		mux |= ADS126X_MUXN_AIN9;
		break;
	case 10:
		mux |= ADS126X_MUXN_AINCOM;
		break;
	case 11:
		mux |= ADS126X_MUXN_TEMP;
		break;
	case 12:
		mux |= ADS126X_MUXN_AVSS;
		break;
	case 13:
		mux |= ADS126X_MUXN_DVSS;
		break;
	case 14:
		mux |= ADS126X_MUXN_TDAC;
		break;
	case 15:
		mux |= ADS126X_MUXN_FLOAT;
		break;
	default:
		return -EINVAL;
	}

	int ret = ads126x_write_reg(dev, ADS126X_REG_INPMUX, mux);

	if (ret) {
		return ret;
	}

	ret = ads126x_send_command(dev, ADS126X_CMD_START1);

	if (ret) {
		return ret;
	}

	return ret;
}

static int ads126x_wait_drdy(const struct device *dev)
{
	const struct ads126x_config *config = dev->config;

	uint32_t elapsed = 0U;

	while (gpio_pin_get_dt(&config->drdy_gpio) != 0) {

		if (elapsed >= ADS126X_DRDY_TIMEOUT_US) {
			LOG_ERR("DRDY timeout");
			return -ETIMEDOUT;
		}

		k_busy_wait(ADS126X_DRDY_POLL_US);

		elapsed += ADS126X_DRDY_POLL_US;
	}

	return 0;
}

static int ads126x_read_data(const struct device *dev, int32_t *sample)
{
	const struct ads126x_config *config = dev->config;
	const struct ads126x_data *data = dev->data;

	uint8_t tx_buf[8] = {0};
	uint8_t rx_buf[8] = {0};
	uint8_t interface_reg = data->interface_reg;
	uint8_t frame_len;
	uint8_t data_index;
	int ret;

	if (sample == NULL) {
		return -EINVAL;
	}

	bool status_enable = (interface_reg & ADS126X_INTERFACE_STATUS_ENABLE) != 0U;

	uint8_t crc_mode = (interface_reg & ADS126X_INTERFACE_CRC_MASK);

	bool crc_enable = (crc_mode == ADS126X_INTERFACE_CRC_CRC);

	bool checksum_enable = (crc_mode == ADS126X_INTERFACE_CRC_CHECKSUM);

	/* Determine response length*/

	frame_len = 4; /* ADC result */

	if (status_enable) {
		frame_len++;
	}

	if (crc_enable || checksum_enable) {
		frame_len++;
	}

	tx_buf[0] = ADS126X_CMD_RDATA1;

	struct spi_buf tx_spi = {
		.buf = tx_buf,
		.len = frame_len + 1,
	};

	struct spi_buf rx_spi = {
		.buf = rx_buf,
		.len = frame_len + 1,
	};

	struct spi_buf_set tx = {
		.buffers = &tx_spi,
		.count = 1,
	};

	struct spi_buf_set rx = {
		.buffers = &rx_spi,
		.count = 1,
	};

	ret = spi_transceive_dt(&config->bus, &tx, &rx);

	if (ret < 0) {
		return ret;
	}

	/* Decode frame */

	data_index = 1;

	if (status_enable) {
		uint8_t command_status = rx_buf[0];

		LOG_DBG("command_status = 0x%02X", command_status);

		data_index++;
	}

	*sample = (int32_t)(((uint32_t)rx_buf[data_index] << 24) |
			    ((uint32_t)rx_buf[data_index + 1] << 16) |
			    ((uint32_t)rx_buf[data_index + 2] << 8) | rx_buf[data_index + 3]);

	LOG_HEXDUMP_INF(rx_buf, frame_len + 1, "ADC Frame");

	LOG_INF("Sample= %d", *sample);

	if (crc_enable) {

		/* TODO: */
		/* Verify CRC here. */
	}

	if (checksum_enable) {

		/* TODO: */
		/* Verify checksum here. */
	}

	return 0;
}

static int ads126x_adc_config_gain(enum adc_gain gain, uint8_t *gain_config, uint32_t *gain_value)
{
	if (gain_config == NULL) {
		return -EINVAL;
	}

	*gain_value = 0;

	switch (gain) {
	case ADC_GAIN_1:
		*gain_config = ADS126X_MODE2_GAIN_1;
		*gain_value = 1;
		break;

	case ADC_GAIN_2:
		*gain_config = ADS126X_MODE2_GAIN_2;
		*gain_value = 2;
		break;

	case ADC_GAIN_4:
		*gain_config = ADS126X_MODE2_GAIN_4;
		*gain_value = 4;
		break;

	case ADC_GAIN_8:
		*gain_config = ADS126X_MODE2_GAIN_8;
		*gain_value = 8;
		break;

	case ADC_GAIN_16:
		*gain_config = ADS126X_MODE2_GAIN_16;
		*gain_value = 16;
		break;

	case ADC_GAIN_32:
		*gain_config = ADS126X_MODE2_GAIN_32;
		*gain_value = 32;
		break;

	case ADC_GAIN_64:
		return -ENOTSUP;

	case ADC_GAIN_128:
		return -ENOTSUP;

	default:
		LOG_ERR("Unsupported gain: %d", gain);
		return -EINVAL;
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
	return 0;
}

static int ads126x_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct ads126x_data *data = dev->data;

	int ret = ads126x_send_command(dev, ADS126X_CMD_STOP1);

	if (ret) {
		return ret;
	}

	ret = ads126x_select_channel(dev, data->positive_input, data->negative_input);

	if (ret) {
		return ret;
	}

	ret = ads126x_wait_drdy(dev);

	if (ret) {
		return ret;
	}

	if (data->config_dirty == true) {

		uint8_t mux;

		mux = data->positive_input << 4 | data->negative_input;
		/* ads126x_mux_value(data->positive_input, data->negative_input); */

		if (mux != data->cached_inpmux) {

			ret = ads126x_write_reg(dev, ADS126X_REG_INPMUX, mux);

			if (ret) {
				return ret;
			}

			data->cached_inpmux = mux;
		}

		uint8_t gain_cfg;
		uint32_t gain_value;

		ret = ads126x_adc_config_gain(data->gain, &gain_cfg, &gain_value);

		if (ret) {
			return ret;
		}

		if (gain_cfg != data->cached_mode2) {

			ret = ads126x_update_reg(dev, ADS126X_REG_MODE2, ADS126X_MODE2_GAIN_MASK,
						 gain_cfg);

			if (ret) {
				return ret;
			}

			data->cached_mode2 = gain_cfg;
		}

		data->config_dirty = false;
	}

	uint8_t gain_cfg;
	uint32_t gain_value;

	ret = ads126x_adc_config_gain(data->gain, &gain_cfg, &gain_value);

	if (ret) {
		return ret;
	}

	ret = ads126x_update_reg(dev, ADS126X_REG_MODE2, ADS126X_MODE2_GAIN_MASK, gain_cfg);

	if (ret) {
		return ret;
	}

	ret = ads126x_send_command(dev, ADS126X_CMD_START1);

	if (ret) {
		return ret;
	}

	ret = ads126x_wait_drdy(dev);

	if (ret) {
		return ret;
	}

	int32_t dummy = 0;

	ret = ads126x_read_data(dev, &dummy);

	if (ret) {
		return ret;
	}

	ret = ads126x_send_command(dev, ADS126X_CMD_START1);

	ret = ads126x_wait_drdy(dev);

	if (ret) {
		return ret;
	}

	int32_t sample = 0;

	ret = ads126x_read_data(dev, &sample);

	if (ret) {
		return ret;
	}

	if (sequence->buffer == NULL) {
		return -EINVAL;
	}

	if (sequence->buffer_size < sizeof(int32_t)) {
		return -ENOMEM;
	}

	*(int32_t *)sequence->buffer = sample;

	/* float voltage = ((float)sample * 2.5f / (float)(1 << 31)); */
	for (uint8_t reg = 1; reg <= 0x1B; ++reg) {
		uint8_t value;

		ads126x_read_reg(dev, reg, &value);
		LOG_INF("REG %02X = %02X", reg, value);
	}

	double voltage = ((double)(sample * 2.5f)) / ((double)(INT32_MAX * gain_value));

	LOG_INF("ADS1263 read adc %d, Voltage: %f, gain_value: %d", sample, (double)voltage,
		gain_value);

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

	LOG_INF("config->adc1_filter = 0x%02X, mode1 = 0x%02X", config->adc1_filter, mode1);

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

	int ret;

	k_mutex_init(&data->lock);

	/* Verify peripherals */

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI not ready");
		return -ENODEV;
	}

	if (config->drdy_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->drdy_gpio)) {
			LOG_ERR("DRDY GPIO not ready");
			return -ENODEV;
		}
	}

	ret = gpio_pin_configure_dt(&config->drdy_gpio, GPIO_INPUT);

	if (ret) {
		return ret;
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

	LOG_INF("ADS126x (%s) initialised",
		config->chip_id == ADS126X_CHIP_ADS1263 ? "ADS1263" : "ADS1262");

	LOG_INF("ADS1263 initialization complete Interface Reg=0x%02X", data->interface_reg);

	return 0;
}

static DEVICE_API(adc, ads126x_driver_api) = {
	.channel_setup = ads126x_channel_setup,
	.read = ads126x_read,
	.ref_internal = ADS126X_REF_INTERNAL,
};

#define ADS126X_INIT(inst)                                                                         \
	static struct ads126x_data data_##inst;                                                    \
	static const struct ads126x_config config_##inst = {                                       \
		.bus = SPI_DT_SPEC_INST_GET(inst,                                                  \
					    SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA),   \
		.drdy_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drdy_gpios, {0}),                      \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                    \
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
                                                                                                   \
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
