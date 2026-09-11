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
#include <zephyr/sys/util.h>
#include <errno.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

enum ads126x_chip_id {
	ADS126X_CHIP_ADS1262 = 0,
	ADS126X_CHIP_ADS1263 = 1,
};

/* ID register */
#define ADS126X_ID_DEV_MASK 0xE0
#define ADS126X_ID_ADS1262  0x00
#define ADS126X_ID_ADS1263  0x20

/* INPMUX Register Masks */
#define ADS126X_INPMUX_MUXP_SHIFT 4U
#define ADS126X_INPMUX_MUXN_SHIFT 0U

#define ADS126X_INPMUX_MUXP(value) (((value) & 0x0FU) << ADS126X_INPMUX_MUXP_SHIFT)

#define ADS126X_INPMUX_MUXN(value) (((value) & 0x0FU) << ADS126X_INPMUX_MUXN_SHIFT)

/* MODE 2 Configuration Register */

#define ADS126X_MODE2_GAIN_1  0U
#define ADS126X_MODE2_GAIN_2  1U
#define ADS126X_MODE2_GAIN_4  2U
#define ADS126X_MODE2_GAIN_8  3U
#define ADS126X_MODE2_GAIN_16 4U
#define ADS126X_MODE2_GAIN_32 5U

/* ADC1 Channel Id: (0  - 15) */
#define ADS126X_ADC1_CHANNEL_MIN 0U
#define ADS126X_ADC1_CHANNEL_MAX 15U
#define ADS126X_ADC1_RESOLUTION 32U

#define ADS126X_MODE0_PULSE_CONVERSION BIT(6)
#define ADS126X_MODE0_CHOP_DISABLED    0U
#define ADS126X_MODE0_DELAY_DEFAULT    0U

#define ADS126X_REF_INTERNAL 2500 /*< Internal reference voltage in mV */

#define ADS126X_DRDY_WAIT_TIMEOUT_MS         K_MSEC(250U)
#define ADS126X_MAX_CAL_DRDY_WAIT_TIMEOUT_MS K_MSEC(10000U)
#define ADS126X_RESET_DELAY_MS               10U

/* System Commands */
#define ADS126X_CMD_RESET 0x06

/* ADC1 Commands */
#define ADS126X_CMD_START1  0x08
#define ADS126X_CMD_STOP1   0x0A
#define ADS126X_CMD_RDATA1  0x12
#define ADS126X_CMD_SFOCAL1 0x19

/* Register Commands */
#define ADS126X_CMD_RREG 0x20
#define ADS126X_CMD_WREG 0x40

/* Power Register */
#define ADS126X_POWER_INTREF BIT(0)

/* INTERFACE register */
#define ADS126X_INTF_STATUS        BIT(2)
#define ADS126X_INTF_CRC_MASK      0x03
#define ADS126X_INTF_CHECKSUM_NONE 0x00
#define ADS126X_INTF_CHECKSUM_XOR  0x01
#define ADS126X_INTF_CHECKSUM_CRC  0x02

/* RDATA1: cmd(1) + status(1) + data(4) + crc(1) = 7 bytes max */
#define ADS126X_RDATA1_NO_STATUS_NO_CRC 5 /* cmd + 4 bytes data */
#define ADS126X_RDATA1_STATUS_NO_CRC    6
#define ADS126X_RDATA1_STATUS_CRC       7

/* MODE1 - Filter */
#define ADS126X_MODE1_FILTER_MASK 0xE0

/* MODE2 - PGA / Data Rate */
#define ADS126X_MODE2_BYPASS     BIT(7)
#define ADS126X_MODE2_GAIN_MASK  0x70u
#define ADS126X_MODE2_GAIN_SHIFT 4
#define ADS126X_MODE2_DR_MASK    0x0F

/* Register Addresses */
#define ADS126X_REG_ID        0x00
#define ADS126X_REG_POWER     0x01
#define ADS126X_REG_INTERFACE 0x02
#define ADS126X_REG_MODE0     0x03
#define ADS126X_REG_MODE1     0x04
#define ADS126X_REG_MODE2     0x05
#define ADS126X_REG_INPMUX    0x06
#define ADS126X_REG_REFMUX    0x0F

#define ADS126X_MUXP_SHIFT 4U
#define ADS126X_MUX_MASK   0x0FU

#define ADS126X_BUILD_MUX(muxp, muxn)                                                              \
	(((((uint8_t)(muxp)) & ADS126X_MUX_MASK) << ADS126X_MUXP_SHIFT) |                          \
	 ((((uint8_t)(muxn)) & ADS126X_MUX_MASK)))

LOG_MODULE_REGISTER(adc_ads126x, CONFIG_ADC_LOG_LEVEL);

struct ads126x_config {
	struct spi_dt_spec bus;

	struct gpio_dt_spec drdy_gpio;
	struct gpio_dt_spec reset_gpio;

	enum ads126x_chip_id chip_id;

	uint8_t adc1_data_rate; /* MODE2 DR field value */
	uint8_t adc1_filter;    /* MODE1 FILTER field value */
	uint8_t adc1_ref_mux;   /* REFMUX encoded value */

	/* Interface options */
	uint8_t crc_mode; /* INTERFACE.CRC_EN field */
	bool status_byte; /* INTERFACE.STATUS enable */
};

struct ads126x_channel_config {
	uint8_t input_positive;
	uint8_t input_negative;
	uint8_t channel_id;
	bool configured;
	bool isDifferential;
	uint8_t gain;
	uint8_t reference;
};

struct ads126x_data {

	struct ads126x_channel_config channels[ADS126X_ADC1_CHANNEL_MAX + 1];
	struct gpio_callback drdy_callback;
	const struct device *dev;
	struct adc_context ctx;
	struct k_sem drdy_sem;
	int32_t *buffer;
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

static int ads126x_reset(const struct device *dev)
{
	const struct ads126x_config *config = dev->config;
	int ret = 0;

	if (config->reset_gpio.port != NULL) {
		/* Assert hard reset */
		ret = gpio_pin_set_dt(&config->reset_gpio, 0);

		if (ret) {
			return ret;
		}

		k_msleep(ADS126X_RESET_DELAY_MS);

		ret = gpio_pin_set_dt(&config->reset_gpio, 1);

		if (ret) {
			return ret;
		}
	} else {
		/* Software reset */
		ret = ads126x_send_command(dev, ADS126X_CMD_RESET);

		if (ret) {
			return ret;
		}

		k_msleep(ADS126X_RESET_DELAY_MS);
	}

	return ret;
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

static uint8_t ads126x_crc8(const uint8_t *data, size_t len)
{
	uint8_t crc = 0xFF;

	for (size_t i = 0; i < len; ++i) {
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

	/* Determine frame length */
	if (config->status_byte && config->crc_mode != ADS126X_INTF_CHECKSUM_NONE) {
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

	/* Verify CRC if enabled */
	if (config->crc_mode == ADS126X_INTF_CHECKSUM_CRC) {

		uint8_t *rx_buffer_start = &rx[0];

		if (config->status_byte) {

			rx_buffer_start = &rx[1];
		}

		uint8_t calc = ads126x_crc8(rx_buffer_start, frame_len - 2);

		if (calc != rx[frame_len - 1]) {
			LOG_ERR("RDATA1 CRC mismatch: expected 0x%02X got 0x%02X", calc,
				rx[frame_len - 1]);
			return -EIO;
		}
	} else if (config->crc_mode == ADS126X_INTF_CHECKSUM_XOR) {
		/* Checksum: simple XOR byte */
		uint8_t chk = 0x9B;

		for (int i = data_offset; i < (int)(frame_len - 1); ++i) {
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

	double voltage_v = ((double)*result * 2.5) / (double)INT32_MAX;

	LOG_INF("result=%d voltage=%.6f V\r\n", *result, voltage_v);

	return 0;
}

static void ads126x_clear_drdy(const struct device *dev)
{
	struct ads126x_data *data = dev->data;

	while (k_sem_take(&data->drdy_sem, K_NO_WAIT) == 0) {
	}
}

static int ads126x_wait_data_ready(const struct device *dev, k_timeout_t timeout)
{
	struct ads126x_data *data = dev->data;

	/* k_sem_reset(&data->drdy_sem); */

	return k_sem_take(&data->drdy_sem, timeout);
}

static int ads126x_config_voltage_reference(const struct device *dev)
{
	struct ads126x_data *data = dev->data;
	uint8_t val;

	int ret = ads126x_read_reg(dev, ADS126X_REG_POWER, &val);

	if (ret) {
		return ret;
	}

	val |= ADS126X_POWER_INTREF;

	return ads126x_write_reg(dev, ADS126X_REG_POWER, val);
}

static int ads126x_config_adc1_gain(const struct device *dev, uint8_t channel)
{
	struct ads126x_data *data = dev->data;
	uint8_t mode2;
	int ret;

	ret = ads126x_read_reg(dev, ADS126X_REG_MODE2, &mode2);
	if (ret) {
		return ret;
	}

	mode2 &= ~ADS126X_MODE2_GAIN_MASK;
	mode2 |= (data->channels[channel].gain << ADS126X_MODE2_GAIN_SHIFT) &
		 ADS126X_MODE2_GAIN_MASK;

	return ads126x_write_reg(dev, ADS126X_REG_MODE2, mode2);
}

static int ads126x_read_channel_adc1(const struct device *dev, uint8_t channel, int32_t *result)
{
	struct ads126x_data *data = dev->data;

	int ret;

	ret = ads126x_config_adc1_gain(dev, channel);

	if (ret) {
		return ret;
	}

	k_sem_reset(&data->drdy_sem);

	/* Start single conversion */
	ads126x_clear_drdy(dev);

	ret = ads126x_send_command(dev, ADS126X_CMD_START1);

	if (ret) {
		return ret;
	}	

	/* Wait for DRDY */
	ret = ads126x_wait_data_ready(dev, ADS126X_DRDY_WAIT_TIMEOUT_MS);
	if (ret) {
		LOG_ERR("ADC1 DRDY timeout on channel %d", channel);
		ads126x_send_command(dev, ADS126X_CMD_STOP1);
		return -ETIMEDOUT;
	}

	ret = ads126x_rdata1(dev, result, NULL);

	if (ret) {
		LOG_ERR("RDATA1 failed: %d", ret);

		ads126x_send_command(dev, ADS126X_CMD_STOP1);

		return ret;
	}

	ret = ads126x_send_command(dev, ADS126X_CMD_STOP1);

	return ret;
}

static int ads126x_input_to_mux(uint8_t input, uint8_t *mux)
{
	if (mux == NULL) {
		return -EINVAL;
	}

	switch (input) {

	case ADS126X_MUX_AIN0:
	case ADS126X_MUX_AIN1:
	case ADS126X_MUX_AIN2:
	case ADS126X_MUX_AIN3:
	case ADS126X_MUX_AIN4:
	case ADS126X_MUX_AIN5:
	case ADS126X_MUX_AIN6:
	case ADS126X_MUX_AIN7:
	case ADS126X_MUX_AIN8:
	case ADS126X_MUX_AIN9:
	case ADS126X_MUX_AINCOM:
		*mux = input;
		return 0;

	default:
		LOG_ERR("Invalid ADS126x input: %u", input);
		return -EINVAL;
	}
}

static int ads126x_configure_input_mux(const struct device *dev, uint8_t channel)
{
	struct ads126x_data *data = dev->data;

	uint8_t muxp;
	uint8_t muxn;
	uint8_t inpmux;
	int ret;

	if (data == NULL) {
		return -EINVAL;
	}

	ret = ads126x_input_to_mux(data->channels[channel].input_positive, &muxp);
	if (ret != 0) {
		return ret;
	}

	if (data->channels[channel].isDifferential) {

		ret = ads126x_input_to_mux(data->channels[channel].input_negative, &muxn);
		if (ret != 0) {
			return ret;
		}

		if (muxp == muxn) {
			LOG_ERR("Invalid differential input: "
				"AIN%u - AIN%u",
				muxp, muxn);

			return -EINVAL;
		}

	} else {

		/*
		 * Single-ended input always uses AINCOM
		 * as the negative input.
		 */
		muxn = ADS126X_MUX_AINCOM;
	}

	inpmux = ADS126X_INPMUX_MUXP(muxp) | ADS126X_INPMUX_MUXN(muxn);

	LOG_DBG("Configure INPMUX: "
		"P=%u N=%u REG=0x%02x",
		muxp, muxn, inpmux);

	return ads126x_write_reg(dev, ADS126X_REG_INPMUX, inpmux);
}

static int ads126x_perform_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct ads126x_data *data = dev->data;
	uint32_t channels = sequence->channels;
	int ret = 0;

	/*  Need to change this logic based on channel 0 to 15 not a bitwise channels*/
	/*  Buffer and buffer size validation return -ENOMEM; */

	if (sequence->buffer == NULL) {
		return -EINVAL;
	}

	if (sequence->buffer_size < sizeof(int32_t)) {
		return -ENOMEM;
	}

	if (sequence->resolution != ADS126X_ADC1_RESOLUTION) {
		LOG_ERR("Unsupported resolution: %u", sequence->resolution);
		return -EINVAL;
	}

	if (!data->channels[channels].configured) {
		LOG_ERR("ADC channel %u is not configured", channels);
		return -EINVAL;
	}

	ret = ads126x_configure_input_mux(dev, channels);

	if (ret) {
		LOG_ERR("Failed to configure mux");
		return ret;
	}

	int32_t result = 0;

	if (channels <= ADS126X_ADC1_CHANNEL_MAX) {
		/* ADC1 channel */
		ret = ads126x_read_channel_adc1(dev, channels, &result);
	}

	if (ret) {
		LOG_ERR("Channel %d read failed: %d", channels, ret);
		return ret;
	}

	*data->buffer = result;

	return 0;
}
static int ads126x_validate_mux_input(uint8_t input)
{
	if (input > ADS126X_MUX_AINCOM) {
		return -EINVAL;
	}

	return 0;
}

static int ads126x_get_gain_value(uint8_t gain)
{
	int ret;

	switch (gain) {
	case ADC_GAIN_1:
		ret = ADS126X_MODE2_GAIN_1;
		break;
	case ADC_GAIN_2:
		ret = ADS126X_MODE2_GAIN_2;
		break;
	case ADC_GAIN_4:
		ret = ADS126X_MODE2_GAIN_4;
		break;
	case ADC_GAIN_8:
	case ADC_GAIN_16:
		ret = ADS126X_MODE2_GAIN_16;
		break;
	case ADC_GAIN_32:
		ret = ADS126X_MODE2_GAIN_32;
		break;
	default:
		LOG_ERR("Unsupported gain");
		return -EINVAL;
	}

	return ret;
}

static int ads126x_validate_channel_inputs(const struct adc_channel_cfg *channel_cfg)
{
	int ret;

	ret = ads126x_validate_mux_input(channel_cfg->input_positive);

	if (ret != 0) {
		return ret;
	}

	ret = ads126x_validate_mux_input(channel_cfg->input_negative);

	if (ret != 0) {
		return ret;
	}

	if (channel_cfg->channel_id > ADS126X_ADC1_CHANNEL_MAX) {
		return -EINVAL;
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

	/* Need To check this*/
	switch (channel_cfg->reference) {
	case ADC_REF_INTERNAL:
		break;
	default:
		LOG_ERR("Unsupported reference");
		return -EINVAL;
	}

	return 0;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ads126x_data *data = CONTAINER_OF(ctx, struct ads126x_data, ctx);

	int ret = ads126x_perform_read(data->dev, &ctx->sequence);

	if (ret) {
		LOG_ERR("ads126x_perform_read failed: %d", ret);
		adc_context_complete(ctx, ret);
		return;

	}

	adc_context_on_sampling_done(ctx, data->dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	ARG_UNUSED(ctx);
	ARG_UNUSED(repeat_sampling);
}

static int ads126x_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct ads126x_data *data = dev->data;

	if (sequence->options != NULL) {
		LOG_ERR("ADC sequence options are not supported");
		return -ENOTSUP;
	}

	adc_context_lock(&data->ctx, false, NULL);

	data->buffer = sequence->buffer;

	adc_context_start_read(&data->ctx, sequence);

	int ret = adc_context_wait_for_completion(&data->ctx);

	adc_context_release(&data->ctx, ret);

	return ret;
}

static int ads126x_config_interface(const struct device *dev)
{
	const struct ads126x_config *config = dev->config;

	uint8_t val;

	int ret = ads126x_read_reg(dev, ADS126X_REG_INTERFACE, &val);

	if (ret) {
		return ret;
	}

	if (config->status_byte) {
		val |= ADS126X_INTF_STATUS;
	} else {
		val &= ~ADS126X_INTF_STATUS;
	}

	val &= ~ADS126X_INTF_CRC_MASK;

	val |= (config->crc_mode & ADS126X_INTF_CRC_MASK);

	ret = ads126x_write_reg(dev, ADS126X_REG_INTERFACE, val);

	return ret;
}

static int ads126x_config_adc1(const struct device *dev)
{
	const struct ads126x_config *config = dev->config;
	int ret;

	/* MODE0: pulse conversion, default delay */
	uint8_t mode0 = ADS126X_MODE0_PULSE_CONVERSION | ADS126X_MODE0_DELAY_DEFAULT;

	ret = ads126x_write_reg(dev, ADS126X_REG_MODE0, mode0);
	if (ret) {
		return ret;
	}

	uint8_t mode1 = (config->adc1_filter << 5) & ADS126X_MODE1_FILTER_MASK;

	/* MODE1: filter selection */
	ret = ads126x_write_reg(dev, ADS126X_REG_MODE1, mode1);
	if (ret) {
		return ret;
	}

	/* MODE2: PGA gain + data rate */
	uint8_t mode2 = 0;

	mode2 |= (config->adc1_data_rate & ADS126X_MODE2_DR_MASK);

	ret = ads126x_write_reg(dev, ADS126X_REG_MODE2, mode2);
	if (ret) {
		return ret;
	}

	ret = ads126x_config_voltage_reference(dev);

	/* Need to check the ret is validated as same in all places */
	if (ret) {
		return ret;
	}

	return ads126x_write_reg(dev, ADS126X_REG_REFMUX, config->adc1_ref_mux);
}

static void ads126x_data_ready_handler(const struct device *dev, struct gpio_callback *gpio_cb,
				       uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	struct ads126x_data *data =
		(struct ads126x_data *)((char *)gpio_cb -
					offsetof(struct ads126x_data, drdy_callback));

	k_sem_give(&data->drdy_sem);
}

static int ads126x_channel_setup(const struct device *dev,
				 const struct adc_channel_cfg *channel_cfg)
{

	LOG_INF("ads126x channel setup Start");
	/* const struct ads126x_config *config = dev->config; */

	struct ads126x_data *data = dev->data;
	int ret;

	if (channel_cfg == NULL) {
		return -EINVAL;
	}

	ret = ads126x_validate_channel_inputs(channel_cfg);

	if (ret != 0) {
		return ret;
	}

	/*
	 * Store configuration.
	 *
	 * This is important because adc_read()
	 * only gives us sequence.channels.
	 */

	data->channels[channel_cfg->channel_id].configured = true;
	data->channels[channel_cfg->channel_id].channel_id = channel_cfg->channel_id;
	data->channels[channel_cfg->channel_id].isDifferential = channel_cfg->differential;
	data->channels[channel_cfg->channel_id].input_positive = channel_cfg->input_positive;
	data->channels[channel_cfg->channel_id].input_negative = channel_cfg->input_negative;
	data->channels[channel_cfg->channel_id].reference = channel_cfg->reference;
	data->channels[channel_cfg->channel_id].gain = ads126x_get_gain_value(channel_cfg->gain);

	LOG_DBG("Channel %d configured", channel_cfg->channel_id);
	LOG_INF("ads126x channel setup End\r\n\r\n");

	return 0;
}

static int ads126x_init(const struct device *dev)
{
	struct ads126x_data *data = dev->data;
	const struct ads126x_config *config = dev->config;

	data->dev = dev;

	int ret;

	k_sem_init(&data->drdy_sem, 0, 1);

	adc_context_init(&data->ctx);

	/* Verify peripherals */
	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI not ready");
		return -ENODEV;
	}

	/* Verify DRDY GPIO */
	if (!gpio_is_ready_dt(&config->drdy_gpio)) {
		LOG_ERR("DRDY GPIO not ready");
		return -ENODEV;
	}

	/* Configure DRDY as input */
	ret = gpio_pin_configure_dt(&config->drdy_gpio, GPIO_INPUT);
	if (ret) {
		LOG_ERR("Failed to configure DRDY GPIO: %d", ret);
		return ret;
	}

	/*Initialise callback */
	gpio_init_callback(&data->drdy_callback, ads126x_data_ready_handler,
			   BIT(config->drdy_gpio.pin));

	/* Register callback */
	ret = gpio_add_callback(config->drdy_gpio.port, &data->drdy_callback);

	if (ret) {
		LOG_ERR("Failed to add DRDY callback: %d", ret);
		return ret;
	}

	/* Configure interrupt */
	ret = gpio_pin_interrupt_configure_dt(&config->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);

	if (ret) {
		LOG_ERR("Failed to configure DRDY interrupt: %d", ret);
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

	ret = ads126x_config_interface(dev);
	if (ret) {
		return ret;
	}

	ret = ads126x_config_adc1(dev);
	if (ret) {
		return ret;
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

#define ADS126X_INIT(inst, chip_type)                                                              \
	static const struct ads126x_config chip_type##_config_##inst = {                           \
		.bus = SPI_DT_SPEC_INST_GET(inst,                                                  \
					    SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA),   \
		.drdy_gpio = GPIO_DT_SPEC_INST_GET(inst, drdy_gpios),                              \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                    \
		.chip_id = chip_type,                                                              \
		.adc1_data_rate = DT_INST_PROP(inst, adc1_data_rate),                              \
		.adc1_filter = DT_INST_PROP(inst, adc1_filter),                                    \
		.adc1_ref_mux = DT_INST_PROP(inst, adc1_ref_mux),                                  \
		.crc_mode = DT_INST_PROP(inst, crc_mode),                                          \
		.status_byte = DT_INST_PROP(inst, status_byte),                                    \
	};                                                                                         \
	static struct ads126x_data chip_type##_data_##inst = {                                     \
		ADC_CONTEXT_INIT_LOCK(chip_type##_data_##inst, ctx),                               \
		ADC_CONTEXT_INIT_TIMER(chip_type##_data_##inst, ctx),                              \
		ADC_CONTEXT_INIT_SYNC(chip_type##_data_##inst, ctx),                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, ads126x_init, NULL, &chip_type##_data_##inst,                  \
			      &chip_type##_config_##inst, POST_KERNEL,                             \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &ads126x_driver_api);

#define DT_DRV_COMPAT ti_ads1262

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY_VARGS(ADS126X_INIT, ADS126X_CHIP_ADS1262)
#endif

#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT ti_ads1263

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
DT_INST_FOREACH_STATUS_OKAY_VARGS(ADS126X_INIT, ADS126X_CHIP_ADS1263)
#endif

#undef DT_DRV_COMPAT
