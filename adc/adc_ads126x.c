/*
 * Copyright 2026 Sacra Systems Private Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_ads1263

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc/ads126x.h>

#define ADS1263_ADC1_RESOLUTION 32U
#define ADS1263_REF_INTERNAL    2500 /*< Internal reference voltage in mV */

#define ADS1263_DRDY_TIMEOUT_US 10000U
#define ADS1263_DRDY_POLL_US    10U

/* System Commands */
#define ADS1263_CMD_NOP   0x00
#define ADS1263_CMD_RESET 0x06

/* ADC1 Commands */
#define ADS1263_CMD_START1 0x08
#define ADS1263_CMD_STOP1  0x0A
#define ADS1263_CMD_RDATA1 0x12

/* ADC2 Commands */
#define ADS1263_CMD_START2 0x0C
#define ADS1263_CMD_STOP2  0x0E
#define ADS1263_CMD_RDATA2 0x14

/* Register Commands */
#define ADS1263_CMD_RREG 0x20
#define ADS1263_CMD_WREG 0x40

/* Register Addresses */
#define ADS1263_REG_ID        0x00
#define ADS1263_REG_POWER     0x01
#define ADS1263_REG_INTERFACE 0x02
#define ADS1263_REG_MODE0     0x03
#define ADS1263_REG_MODE1     0x04
#define ADS1263_REG_MODE2     0x05
#define ADS1263_REG_INPMUX    0x06
#define ADS1263_REG_OFCAL0    0x07
#define ADS1263_REG_OFCAL1    0x08
#define ADS1263_REG_OFCAL2    0x09
#define ADS1263_REG_FSCAL0    0x0A
#define ADS1263_REG_FSCAL1    0x0B
#define ADS1263_REG_FSCAL2    0x0C
#define ADS1263_REG_IDACMUX   0x0D
#define ADS1263_REG_IDACMAG   0x0E
#define ADS1263_REG_REFMUX    0x0F

#define ADS1263_INTERFACE_STATUS_ENABLE BIT(2)
#define ADS1263_INTERFACE_CRC_MASK      0x03
#define ADS1263_INTERFACE_CRC_CRC       0x01
#define ADS1263_INTERFACE_CRC_CHECKSUM  0x02

/*TDAC*/
#define ADS1263_TDAC_CHANNEL_MIN 0U
#define ADS1263_TDAC_CHANNEL_MAX 7U

#define ADS1263_TDAC_LEVEL_MIN 0x00U
#define ADS1263_TDAC_LEVEL_MAX 0x19U

#define ADS1263_REG_TDACP 0x10
#define ADS1263_REG_TDACN 0x11

/* TDAC Output Channel (Bits 7:5) */

#define ADS1263_TDAC_OUT_AIN0 (0x0U << 5)
#define ADS1263_TDAC_OUT_AIN1 (0x1U << 5)
#define ADS1263_TDAC_OUT_AIN2 (0x2U << 5)
#define ADS1263_TDAC_OUT_AIN3 (0x3U << 5)
#define ADS1263_TDAC_OUT_AIN4 (0x4U << 5)
#define ADS1263_TDAC_OUT_AIN5 (0x5U << 5)
#define ADS1263_TDAC_OUT_AIN6 (0x6U << 5)
#define ADS1263_TDAC_OUT_AIN7 (0x7U << 5)

/* TDAC Output Magnitude (MAGP / MAGN), Bits [4:0], Output voltage with respect to VAVSS */

#define ADS1263_TDAC_2V500000 0x00U /* 2.500000 V */
#define ADS1263_TDAC_2V507812 0x01U /* 2.5078125 V */
#define ADS1263_TDAC_2V515625 0x02U /* 2.515625 V */
#define ADS1263_TDAC_2V531250 0x03U /* 2.531250 V */
#define ADS1263_TDAC_2V562500 0x04U /* 2.562500 V */
#define ADS1263_TDAC_2V625000 0x05U /* 2.625000 V */
#define ADS1263_TDAC_2V750000 0x06U /* 2.750000 V */
#define ADS1263_TDAC_3V000000 0x07U /* 3.000000 V */
#define ADS1263_TDAC_3V500000 0x08U /* 3.500000 V */
#define ADS1263_TDAC_4V500000 0x09U /* 4.500000 V */

#define ADS1263_TDAC_2V492187 0x11U /* 2.4921875 V */
#define ADS1263_TDAC_2V484375 0x12U /* 2.484375 V */
#define ADS1263_TDAC_2V468750 0x13U /* 2.468750 V */
#define ADS1263_TDAC_2V437500 0x14U /* 2.437500 V */
#define ADS1263_TDAC_2V375000 0x15U /* 2.375000 V */
#define ADS1263_TDAC_2V250000 0x16U /* 2.250000 V */
#define ADS1263_TDAC_2V000000 0x17U /* 2.000000 V */
#define ADS1263_TDAC_1V500000 0x18U /* 1.500000 V */
#define ADS1263_TDAC_0V500000 0x19U /* 0.500000 V */

#define ADS1263_TDAC_LEVEL(x) ((x) & 0x1F)

#define ADS126X_ADC2_OFFSET 16
#define ADS126X_IS_ADC2(ch) ((ch) >= ADS126X_ADC2_OFFSET)
#define ADS126X_HW_CH(ch)   ((ch) & 0x0F)

LOG_MODULE_REGISTER(adc_ads1263, CONFIG_ADC_LOG_LEVEL);

enum ads126x_variant {
	ADS126X_VARIANT_1262,
	ADS126X_VARIANT_1263
};

struct ads126x_config {

	struct spi_dt_spec bus;
	struct gpio_dt_spec drdy;
	struct gpio_dt_spec reset;
	enum ads126x_variant variant;
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

static int ads1263_spi_write(const struct device *dev, const uint8_t *tx_buf, size_t len)
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

static int ads1263_spi_transceive(const struct device *dev, const uint8_t *tx_buf, uint8_t *rx_buf,
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

static int ads1263_send_command(const struct device *dev, uint8_t command)
{
	return ads1263_spi_write(dev, &command, 1);
}

static int ads1263_read_reg(const struct device *dev, uint8_t reg, uint8_t *value)
{
	uint8_t tx[3] = {ADS1263_CMD_RREG | reg, 0x00, 0x00};

	uint8_t rx[3];

	int ret = ads1263_spi_transceive(dev, tx, rx, sizeof(tx));

	if (ret) {
		return ret;
	}

	*value = rx[2];

	return 0;
}

static int ads1263_write_reg(const struct device *dev, uint8_t reg, uint8_t value)
{
	uint8_t tx[3] = {ADS1263_CMD_WREG | reg, 0x00, value};

	return ads1263_spi_write(dev, tx, sizeof(tx));
}

static int ads1263_update_reg(const struct device *dev, uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t reg_val;

	int ret;

	ret = ads1263_read_reg(dev, reg, &reg_val);

	if (ret) {
		return ret;
	}

	reg_val &= ~mask;

	reg_val |= (value & mask);

	return ads1263_write_reg(dev, reg, reg_val);
}

static int ads1263_select_channel(const struct device *dev, uint8_t positive, uint8_t negative)
{
	uint8_t mux = 0;

	switch (positive) {
	case 0:
		mux = ADS1263_MUXP_AIN0;
		break;
	case 1:
		mux = ADS1263_MUXP_AIN1;
		break;
	case 2:
		mux = ADS1263_MUXP_AIN2;
		break;
	case 3:
		mux = ADS1263_MUXP_AIN3;
		break;
	case 4:
		mux = ADS1263_MUXP_AIN4;
		break;
	case 5:
		mux = ADS1263_MUXP_AIN5;
		break;
	case 6:
		mux = ADS1263_MUXP_AIN6;
		break;
	case 7:
		mux = ADS1263_MUXP_AIN7;
		break;
	case 8:
		mux = ADS1263_MUXP_AIN8;
		break;
	case 9:
		mux = ADS1263_MUXP_AIN9;
		break;
	case 10:
		mux = ADS1263_MUXP_AINCOM;
		break;
	case 11:
		mux = ADS1263_MUXP_TEMP;
		break;
	case 12:
		mux = ADS1263_MUXP_AVDD;
		break;
	case 13:
		mux = ADS1263_MUXP_DVDD;
		break;
	case 14:
		mux = ADS1263_MUXP_TDAC;
		break;
	case 15:
		mux = ADS1263_MUXP_FLOAT;
		break;
	default:
		return -EINVAL;
	}

	switch (negative) {
	case 0:
		mux |= ADS1263_MUXN_AIN0;
		break;
	case 1:
		mux |= ADS1263_MUXN_AIN1;
		break;
	case 2:
		mux |= ADS1263_MUXN_AIN2;
		break;
	case 3:
		mux |= ADS1263_MUXN_AIN3;
		break;
	case 4:
		mux |= ADS1263_MUXN_AIN4;
		break;
	case 5:
		mux |= ADS1263_MUXN_AIN5;
		break;
	case 6:
		mux |= ADS1263_MUXN_AIN6;
		break;
	case 7:
		mux |= ADS1263_MUXN_AIN7;
		break;
	case 8:
		mux |= ADS1263_MUXN_AIN8;
		break;
	case 9:
		mux |= ADS1263_MUXN_AIN9;
		break;
	case 10:
		mux |= ADS1263_MUXN_AINCOM;
		break;
	case 11:
		mux |= ADS1263_MUXN_TEMP;
		break;
	case 12:
		mux |= ADS1263_MUXN_AVSS;
		break;
	case 13:
		mux |= ADS1263_MUXN_DVSS;
		break;
	case 14:
		mux |= ADS1263_MUXN_TDAC;
		break;
	case 15:
		mux |= ADS1263_MUXN_FLOAT;
		break;
	default:
		return -EINVAL;
	}

	int ret = ads1263_write_reg(dev, ADS1263_REG_INPMUX, mux);

	if (ret) {
		return ret;
	}

	ret = ads1263_send_command(dev, ADS1263_CMD_START1);

	if (ret) {
		return ret;
	}

	return ret;
}

static int ads1263_wait_drdy(const struct device *dev)
{
	const struct ads126x_config *config = dev->config;

	uint32_t elapsed = 0U;

	while (gpio_pin_get_dt(&config->drdy) != 0) {

		if (elapsed >= ADS1263_DRDY_TIMEOUT_US) {
			LOG_ERR("DRDY timeout");
			return -ETIMEDOUT;
		}

		k_busy_wait(ADS1263_DRDY_POLL_US);

		elapsed += ADS1263_DRDY_POLL_US;
	}

	return 0;
}

static int ads1263_read_data(const struct device *dev, int32_t *sample)
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

	bool status_enable = (interface_reg & ADS1263_INTERFACE_STATUS_ENABLE) != 0U;

	uint8_t crc_mode = (interface_reg & ADS1263_INTERFACE_CRC_MASK);

	bool crc_enable = (crc_mode == ADS1263_INTERFACE_CRC_CRC);

	bool checksum_enable = (crc_mode == ADS1263_INTERFACE_CRC_CHECKSUM);

	/* Determine response length*/

	frame_len = 4; /* ADC result */

	if (status_enable) {
		frame_len++;
	}

	if (crc_enable || checksum_enable) {
		frame_len++;
	}

	tx_buf[0] = ADS1263_CMD_RDATA1;

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

static int ads1263_adc_config_gain(enum adc_gain gain, uint8_t *gain_config, uint32_t *gain_value)
{
	if (gain_config == NULL) {
		return -EINVAL;
	}

	*gain_value = 0;

	switch (gain) {
	case ADC_GAIN_1:
		*gain_config = ADS1263_MODE2_GAIN_1;
		*gain_value = 1;
		break;

	case ADC_GAIN_2:
		*gain_config = ADS1263_MODE2_GAIN_2;
		*gain_value = 2;
		break;

	case ADC_GAIN_4:
		*gain_config = ADS1263_MODE2_GAIN_4;
		*gain_value = 4;
		break;

	case ADC_GAIN_8:
		*gain_config = ADS1263_MODE2_GAIN_8;
		*gain_value = 8;
		break;

	case ADC_GAIN_16:
		*gain_config = ADS1263_MODE2_GAIN_16;
		*gain_value = 16;
		break;

	case ADC_GAIN_32:
		*gain_config = ADS1263_MODE2_GAIN_32;
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

static int ads1263_reset(const struct device *dev)
{
	const struct ads126x_config *config = dev->config;

	if (config->reset.port == NULL) {
		return 0;
	}

	gpio_pin_set_dt(&config->reset, 0);

	k_msleep(5);

	gpio_pin_set_dt(&config->reset, 1);

	k_msleep(10);

	return 0;
}

static int ads126x_init(const struct device *dev)
{
	struct ads126x_data *data = dev->data;
	const struct ads126x_config *config = dev->config;

	uint8_t id;
	uint8_t device;
	int ret;

	k_mutex_init(&data->lock);

	/* Verify peripherals */

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->drdy)) {
		LOG_ERR("DRDY GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->drdy, GPIO_INPUT);

	if (ret) {
		return ret;
	}

	if (config->reset.port != NULL) {

		if (!gpio_is_ready_dt(&config->reset)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->reset, (GPIO_OUTPUT_HIGH));

		if (ret) {
			return ret;
		}
	}

	gpio_pin_set_dt(&config->reset, 1);

	/* Reset device */

	ret = ads1263_reset(dev);

	if (ret) {
		LOG_ERR("Failed to reset ADS1263");
		return ret;
	}

	/* Read and validate device ID */

	ret = ads1263_read_reg(dev, ADS1263_REG_ID, &id);

	if (ret) {
		LOG_ERR("Unable to read ID register");
		return ret;
	}

	device = (id >> 5) & 0x07;

	if (device != 1U) {
		LOG_ERR("ADS1263 not detected. ID=0x%02X", id);
		return -ENODEV;
	}

	LOG_INF("ADS1263 detected");

	data->device_id = device;

	/* Stop ADC1 before configuration */

	ret = ads1263_send_command(dev, ADS1263_CMD_STOP1);

	if (ret) {
		return ret;
	}

	/* MODE2 Gain = 1, Data Rate = 400 SPS */

	ret = ads1263_write_reg(dev, ADS1263_REG_MODE2,
				(ADS1263_MODE2_GAIN_1 | ADS1263_MODE2_DR_400));

	if (ret) {
		return ret;
	}

	/* REFMUX, Internal reference, REFP0 / REFN0 */

	ret = ads1263_write_reg(dev, ADS1263_REG_REFMUX,
				ADS1263_REFMUX_P_INT_2_5V | ADS1263_REFMUX_N_INT_AVSS);

	if (ret) {
		return ret;
	}

	/* MODE0 Continuous Conversion, Delay = 35us, Chop Disabled */

	ret = ads1263_write_reg(dev, ADS1263_REG_MODE0,
				(ADS1263_MODE0_RUNMODE_CONTINUOUS | ADS1263_MODE0_DELAY_35US));

	if (ret) {
		return ret;
	}

	/* MODE1 FIR Filter */

	ret = ads1263_write_reg(dev, ADS1263_REG_MODE1, ADS1263_MODE1_FILTER_FIR);

	if (ret) {
		return ret;
	}

	/* Enable Status byte */

	ret = ads1263_write_reg(dev, ADS1263_REG_INTERFACE, ADS1263_INTERFACE_STATUS_ENABLE);

	if (ret) {
		return ret;
	}

	ret = ads1263_read_reg(dev, ADS1263_REG_INTERFACE, &data->interface_reg);

	if (ret) {
		return ret;
	}

	/* Start ADC1 */

	ret = ads1263_send_command(dev, ADS1263_CMD_START1);

	if (ret) {
		return ret;
	}

	data->initialized = true;
	data->config_dirty = true;
	data->cached_mode2 = 0xFF;
	data->cached_inpmux = 0xFF;

	for (uint8_t reg = 1; reg <= 0x1B; ++reg) {
		uint8_t value;

		ads1263_read_reg(dev, reg, &value);
		LOG_INF("REG %02X = %02X", reg, value);
	}

	LOG_INF("ADS1263 initialization complete Interface Reg=0x%02X", data->interface_reg);

	return 0;
}

static int ads1263_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	/* Validate channel ID */

	struct ads126x_data *data = dev->data;

	if (!data->initialized) {
		return -EIO;
	}

	if (channel_cfg->gain > ADC_GAIN_32) {
		return -EINVAL;
	}

	if (channel_cfg->channel_id != 0) {
		LOG_ERR("Only channel 0 supported");
		return -EINVAL;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL &&
	    channel_cfg->reference != ADC_REF_EXTERNAL0) {
		return -EINVAL;
	}

	data->channel_id = channel_cfg->channel_id;

	data->gain = channel_cfg->gain;

	data->reference = channel_cfg->reference;

	data->differential = channel_cfg->differential;

	data->positive_input = channel_cfg->input_positive;

	data->negative_input = channel_cfg->input_negative;

	data->config_dirty = true;

	LOG_DBG("Channel configuration stored");

	uint8_t inpmux_config = 0;

	LOG_DBG("Configured MUX: %u (differential=%s, pos=%d, neg=%d)", inpmux_config,
		channel_cfg->differential ? "true" : "false", channel_cfg->input_positive,
		channel_cfg->input_negative);

	LOG_INF("ADS1263 channel setup");

	return 0;
}

static int ads1263_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct ads126x_data *data = dev->data;

	int ret = ads1263_send_command(dev, ADS1263_CMD_STOP1);

	if (ret) {
		return ret;
	}

	ret = ads1263_select_channel(dev, data->positive_input, data->negative_input);

	if (ret) {
		return ret;
	}

	ret = ads1263_wait_drdy(dev);

	if (ret) {
		return ret;
	}

	if (data->config_dirty == true) {

		uint8_t mux;

		mux = data->positive_input << 4 | data->negative_input;
		/* ads1263_mux_value(data->positive_input, data->negative_input); */

		if (mux != data->cached_inpmux) {

			ret = ads1263_write_reg(dev, ADS1263_REG_INPMUX, mux);

			if (ret) {
				return ret;
			}

			data->cached_inpmux = mux;
		}

		uint8_t gain_cfg;
		uint32_t gain_value;

		ret = ads1263_adc_config_gain(data->gain, &gain_cfg, &gain_value);

		if (ret) {
			return ret;
		}

		if (gain_cfg != data->cached_mode2) {

			ret = ads1263_update_reg(dev, ADS1263_REG_MODE2, ADS1263_MODE2_GAIN_MASK,
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

	ret = ads1263_adc_config_gain(data->gain, &gain_cfg, &gain_value);

	if (ret) {
		return ret;
	}

	ret = ads1263_update_reg(dev, ADS1263_REG_MODE2, ADS1263_MODE2_GAIN_MASK, gain_cfg);

	if (ret) {
		return ret;
	}

	ret = ads1263_send_command(dev, ADS1263_CMD_START1);

	if (ret) {
		return ret;
	}

	ret = ads1263_wait_drdy(dev);

	if (ret) {
		return ret;
	}

	int32_t dummy = 0;

	ret = ads1263_read_data(dev, &dummy);

	if (ret) {
		return ret;
	}

	ret = ads1263_send_command(dev, ADS1263_CMD_START1);

	ret = ads1263_wait_drdy(dev);

	if (ret) {
		return ret;
	}

	int32_t sample = 0;

	ret = ads1263_read_data(dev, &sample);

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

	double voltage = ((double)(sample * 2.5f)) / ((double)(INT32_MAX * gain_value));

	LOG_INF("ADS1263 read adc %d, Voltage: %f", sample, (double)voltage);

	return ret;
}

static DEVICE_API(adc, ads126x_driver_api) = {
	.channel_setup = ads1263_channel_setup,
	.read = ads1263_read,
	.ref_internal = ADS1263_REF_INTERNAL,
};

#define ADS126X_INIT(inst, var)                                                                    \
	static struct ads126x_data data_##inst;                                                    \
	static const struct ads126x_config config_##inst = {                                       \
		.bus = SPI_DT_SPEC_INST_GET(inst,                                                  \
					    SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA),   \
		.drdy = GPIO_DT_SPEC_INST_GET(inst, drdy_gpios),                                   \
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                         \
		.variant = var,                                                                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, ads126x_init, NULL, &data_##inst, &config_##inst, POST_KERNEL, \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ads126x_driver_api);

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT      ti_ads1262
#define INST_ADS1262(inst) ADS126X_INIT(inst, ADS126X_VARIANT_1262)
DT_INST_FOREACH_STATUS_OKAY(INST_ADS1262)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT      ti_ads1263
#define INST_ADS1263(inst) ADS126X_INIT(inst, ADS126X_VARIANT_1263)
DT_INST_FOREACH_STATUS_OKAY(INST_ADS1263)
