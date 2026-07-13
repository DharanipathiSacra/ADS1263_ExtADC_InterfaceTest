/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

/* SPI1 controller */
#define SPI_NODE DT_NODELABEL(spi1)

static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);

/* SPI configuration */
static struct spi_config spi_cfg = {
    .frequency = 1000000,                  /* 1 MHz */
    // .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA,
    .slave = 0,                            /* Chip Select 0 */
    .cs = NULL                            /* Hardware CS (or configure manually) */
};

int main(void)
{

	uint8_t tx_buf[4] = {
	0x20,      /* RREG command for register 0x00 */
	0x00,      /* Read 1 register */
	0x00,
	0x00       /* Dummy byte */
    };

    uint8_t rx_buf[4] = {0};

    struct spi_buf tx_spi_buf = {
        .buf = tx_buf,
        .len = sizeof(tx_buf),
    };

    struct spi_buf rx_spi_buf = {
        .buf = rx_buf,
        .len = sizeof(rx_buf),
    };

    struct spi_buf_set tx = {
        .buffers = &tx_spi_buf,
        .count = 1,
    };

    struct spi_buf_set rx = {
        .buffers = &rx_spi_buf,
        .count = 1,
    };

    if (!device_is_ready(spi_dev)) {
        printk("SPI device not ready\n");
        return 0;
    }

    int ret = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);

    if (ret) {
        printk("SPI read failed (%d)\n", ret);
        return 0;
    }

    uint8_t id = rx_buf[2];

    uint8_t rev_id = id & 0x1F;
    uint8_t dev_id = (id >> 5) & 0x07;

    printk("Raw ID Register : 0x%02X\n", id);
    printk("Revision ID     : %u\n", rev_id);

    switch (dev_id) {
    case 0:
        printk("Device : ADS1262\n");
        break;

    case 1:
        printk("Device : ADS1263\n");
        break;

    default:
        printk("Device : Unknown (%u)\n", dev_id);
        break;
    }	

	return 0;
}
