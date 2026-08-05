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

    int ret;
    int32_t sample;

    struct adc_channel_cfg channel_cfg = {
        .channel_id = 0,
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.differential = false,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
        .input_positive = 3,
        .input_negative = 12, /* AVSS */
#endif
    };

    struct adc_sequence sequence = {
        .channels = BIT(0),
        .buffer = &sample,
        .buffer_size = sizeof(sample),
        .resolution = 32,
    };

    printf("ADS1263 Driver Validation\r\n");

    if (!device_is_ready(dev)) {
        printf("ADS1263 device not ready\r\n");
        return 0;
    }

    ret = adc_channel_setup(dev, &channel_cfg);
    if (ret) {
        printf("adc_channel_setup() failed (%d)\r\n", ret);
        return 0;
    }

    adc_read(dev, &sequence);

    while (1) {

        static bool once = true;

        if (once) {
            // once = false;
            adc_read(dev, &sequence);
        }
        k_sleep(K_SECONDS(5));
    }
}