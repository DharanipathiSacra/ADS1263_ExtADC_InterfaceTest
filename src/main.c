/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#define ADS1263_NODE DT_NODELABEL(ads1263)

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(ADS1263_NODE);

	printk("\nADS1263 Driver Validation\n");

	if (!device_is_ready(dev)) {
		printk("ADS1263 device not ready\n");
		return 0;
	}

	printk("ADS1263 device ready\n");

	while (1) {
		k_sleep(K_SECONDS(5));
	}
}