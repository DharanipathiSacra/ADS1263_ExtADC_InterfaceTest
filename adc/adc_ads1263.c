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

LOG_MODULE_REGISTER(adc_ads1263, CONFIG_ADC_LOG_LEVEL);

struct ads1263_config {

    struct spi_dt_spec spi;
    struct gpio_dt_spec drdy;
    struct gpio_dt_spec reset;
    struct gpio_dt_spec start;
};

struct ads1263_data {

    struct k_mutex lock;

    int32_t sample;
};


static int ads1263_init(const struct device *dev)
{
    ARG_UNUSED(dev);

    LOG_INF("ADS1263 initialized");

    return 0;
}

#define ADS1263_DEFINE(inst)                                      \
static struct ads1263_data data_##inst;                           \
                                                                  \
static const struct ads1263_config config_##inst = {              \
    .spi = SPI_DT_SPEC_INST_GET(inst,                             \
                SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA, 0),           \
                                                                  \
    .drdy = GPIO_DT_SPEC_INST_GET(inst, drdy_gpios),              \
                                                                  \
    .reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),    \
                                                                  \
    .start = GPIO_DT_SPEC_INST_GET_OR(inst, start_gpios, {0}),    \
};                                                                \
                                                                  \
DEVICE_DT_INST_DEFINE(inst,                                       \
        ads1263_init,                                             \
        NULL,                                                     \
        &data_##inst,                                             \
        &config_##inst,                                           \
        POST_KERNEL,                                              \
        CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                       \
        NULL);

DT_INST_FOREACH_STATUS_OKAY(ADS1263_DEFINE)