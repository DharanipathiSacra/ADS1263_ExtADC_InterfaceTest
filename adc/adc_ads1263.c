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
#include <zephyr/drivers/adc/ads1263.h>

#define ADS1263_ADC1_RESOLUTION 32U
#define ADS1263_REF_INTERNAL    2048 /**< Internal reference voltage in mV */

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

#define ADS1263_INTERFACE_STATUS_ENABLE BIT(2)
#define ADS1263_INTERFACE_CRC_MASK      0x03
#define ADS1263_INTERFACE_CRC_CRC       0x01
#define ADS1263_INTERFACE_CRC_CHECKSUM  0x02

LOG_MODULE_REGISTER(adc_ads1263, CONFIG_ADC_LOG_LEVEL);

struct ads1263_config {

	struct spi_dt_spec bus;
	struct gpio_dt_spec drdy;
	struct gpio_dt_spec reset;
	struct gpio_dt_spec start;
};

struct ads1263_data {

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
};

static int ads1263_spi_write(const struct device *dev, const uint8_t *tx_buf, size_t len)
{
	const struct ads1263_config *config = dev->config;

	struct ads1263_data *data = dev->data;

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
	const struct ads1263_config *config = dev->config;

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

	return spi_transceive_dt(&config->bus, &tx_set, &rx_set);
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

	return ads1263_write_reg(dev, ADS1263_REG_INPMUX, mux);
}

static int ads1263_wait_drdy(const struct device *dev)
{
	const struct ads1263_config *config = dev->config;

	int timeout = 1000;

	while (gpio_pin_get_dt(&config->drdy) != 0) {

		k_busy_wait(10);

		if (--timeout == 0) {
			LOG_ERR("DRDY timeout");
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int ads1263_read_data(const struct device *dev, int32_t *sample)
{
	const struct ads1263_config *config = dev->config;
	uint8_t tx_buf[8] = {0};
	uint8_t rx_buf[8] = {0};
	uint8_t interface_reg;
	uint8_t frame_len;
	uint8_t data_index;
	int ret;

	if (sample == NULL) {
		return -EINVAL;
	}

	/*----------------------------------------------------------
	 * Read INTERFACE register to determine frame format
	 *---------------------------------------------------------*/

	ret = ads1263_read_reg(dev, ADS1263_REG_INTERFACE, &interface_reg);

	if (ret < 0) {
		return ret;
	}

	bool status_enable = (interface_reg & ADS1263_INTERFACE_STATUS_ENABLE) != 0U;

	uint8_t crc_mode = (interface_reg & ADS1263_INTERFACE_CRC_MASK);

	bool crc_enable = (crc_mode == ADS1263_INTERFACE_CRC_CRC);

	bool checksum_enable = (crc_mode == ADS1263_INTERFACE_CRC_CHECKSUM);

	/*----------------------------------------------------------
	 * Determine response length
	 *---------------------------------------------------------*/

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

	/*----------------------------------------------------------
	 * Decode frame
	 *---------------------------------------------------------*/

	data_index = 1;

	if (status_enable) {

		// uint8_t status = rx_buf[data_index];

		// LOG_DBG("Status = 0x%02X", status);

		data_index++;
	}

	*sample = ((int32_t)rx_buf[data_index] << 24) | ((int32_t)rx_buf[data_index + 1] << 16) |
		  ((int32_t)rx_buf[data_index + 2] << 8) | ((int32_t)rx_buf[data_index + 3]);

	if (crc_enable) {

		// uint8_t received_crc = rx_buf[data_index + 4];

		// LOG_DBG("CRC = 0x%02X", received_crc);

		/* TODO:
		 * Verify CRC here.
		 */
	}

	if (checksum_enable) {

		// uint8_t received_checksum = rx_buf[data_index + 4];

		// LOG_DBG("Checksum = 0x%02X", received_checksum);

		/* TODO:
		 * Verify checksum here.
		 */
	}

	return 0;
}

static int ads1263_adc_config_gain(enum adc_gain gain, uint8_t *gain_config)
{
	if (gain_config == NULL) {
		return -EINVAL;
	}

	switch (gain) {
	case ADC_GAIN_1:
		*gain_config = ADS1263_MODE2_GAIN_1;
		break;

	case ADC_GAIN_2:
		*gain_config = ADS1263_MODE2_GAIN_2;
		break;

	case ADC_GAIN_4:
		*gain_config = ADS1263_MODE2_GAIN_4;
		break;

	case ADC_GAIN_8:
		*gain_config = ADS1263_MODE2_GAIN_8;
		break;

	case ADC_GAIN_16:
		*gain_config = ADS1263_MODE2_GAIN_16;
		break;

	case ADC_GAIN_32:
		*gain_config = ADS1263_MODE2_GAIN_32;
		break;

	default:
		LOG_ERR("Unsupported gain: %d", gain);
		return -EINVAL;
	}

	return 0;
}

static int ads1263_reset(const struct device *dev)
{
	const struct ads1263_config *config = dev->config;

	if (config->reset.port == NULL) {
		return 0;
	}

	gpio_pin_set_dt(&config->reset, 0);

	k_msleep(5);

	gpio_pin_set_dt(&config->reset, 1);

	k_msleep(10);

	return 0;
}

static int ads1263_init(const struct device *dev)
{
	struct ads1263_data *data = dev->data;

	k_mutex_init(&data->lock);

	int ret;

	const struct ads1263_config *config = dev->config;

	/* Check SPI bus readiness */

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("%s: SPI device is not ready", dev->name);
		return -ENODEV;
	}

	/* Configure DRDY GPIO */
	if (!device_is_ready(config->drdy.port)) {
		LOG_ERR("DRDY GPIO port not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->drdy, GPIO_INPUT);

	if (ret < 0) {
		LOG_ERR("Failed to configure DRDY GPIO: %d", ret);
		return ret;
	}

	if (config->reset.port != NULL) {

		if (!gpio_is_ready_dt(&config->reset)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_ACTIVE);

		if (ret) {
			return ret;
		}
	}

	if (config->start.port != NULL) {

		if (!gpio_is_ready_dt(&config->start)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->start, GPIO_OUTPUT_ACTIVE);

		if (ret) {
			return ret;
		}
	}

	ret = ads1263_reset(dev);

	if (ret) {
		LOG_ERR("Failed to reset ADS1263");
		return ret;
	}

	uint8_t id;

	ret = ads1263_read_reg(dev, ADS1263_REG_ID, &id);

	if (ret) {
		LOG_ERR("Cannot read ID register");
		return ret;
	}

	uint8_t device = (id >> 5) & 0x07;

	switch (device) {

	case 0:
		LOG_INF("ADS1262 detected");
		break;

	case 1:
		LOG_INF("ADS1263 detected");
		break;

	default:
		LOG_ERR("Unknown device ID %u", device);
		return -ENODEV;
	}

	data->device_id = device;

	data->initialized = true;

	data->status_enabled = true;

	data->crc_enabled = false;

	data->checksum_enabled = false;

	ads1263_read_reg(dev, ADS1263_REG_INTERFACE, &data->interface_reg);

	data->status_enabled = (data->interface_reg & ADS1263_INTERFACE_STATUS_ENABLE) != 0U;

	uint8_t crc_mode = data->interface_reg & ADS1263_INTERFACE_CRC_MASK;

	data->crc_enabled = (crc_mode == ADS1263_INTERFACE_CRC_CRC);

	data->checksum_enabled = (crc_mode == ADS1263_INTERFACE_CRC_CHECKSUM);

	LOG_INF("ADS1263 initialization complete");

	return 0;
}

static int ads1263_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{
	/* Validate channel ID */

	struct ads1263_data *data = dev->data;

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
	struct ads1263_data *data = dev->data;

	int ret = ads1263_select_channel(dev, data->positive_input, data->negative_input);

	if (ret) {
		return ret;
	}

	uint8_t gain_cfg;

	ret = ads1263_adc_config_gain(data->gain, &gain_cfg);

	if (ret) {
		return ret;
	}

	ret = ads1263_update_reg(dev, ADS1263_REG_MODE2, ADS1263_MODE2_GAIN_MASK, gain_cfg);

	if (ret) {
		return ret;
	}

	ads1263_send_command(dev, ADS1263_CMD_START1);

	ads1263_wait_drdy(dev);

	int32_t sample = 0;

	ads1263_read_data(dev, &sample);

	*(int32_t *)sequence->buffer = sample;

	float voltage = ((float)sample * 2.5f / (float)(1 << 31));

	LOG_INF("ADS1263 read adc %d, Voltage: %f", sample, (double)voltage);

	return ret;
}

static DEVICE_API(adc, ads1263_driver_api) = {
	.channel_setup = ads1263_channel_setup,
	.read = ads1263_read,
	.ref_internal = ADS1263_REF_INTERNAL,
};

#define ADS1263_INIT(inst)                                                                         \
	static struct ads1263_data data_##inst;                                                    \
                                                                                                   \
	static const struct ads1263_config config_##inst = {                                       \
		.bus = SPI_DT_SPEC_INST_GET(inst,                                                  \
					    SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA),   \
                                                                                                   \
		.drdy = GPIO_DT_SPEC_INST_GET(inst, drdy_gpios),                                   \
                                                                                                   \
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                         \
                                                                                                   \
		.start = GPIO_DT_SPEC_INST_GET_OR(inst, start_gpios, {0}),                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, ads1263_init, NULL, &data_##inst, &config_##inst, POST_KERNEL, \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ads1263_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADS1263_INIT)
