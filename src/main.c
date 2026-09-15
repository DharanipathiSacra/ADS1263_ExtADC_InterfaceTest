/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/adc/ads126x.h>

#define ADS1263_NODE DT_NODELABEL(ads1263)

int main(void)
{
    const struct device *dev = DEVICE_DT_GET(ADS1263_NODE);

    int ret;
    int32_t sample;
    int32_t sample1;

    struct adc_channel_cfg channel_cfg = {
        .channel_id = 0,
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.differential = false,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
        .input_positive = ADS126X_MUX_AIN0, /* AIN0 */
#endif
    };

    struct adc_channel_cfg channel_2_cfg = {
        .channel_id = 1,
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.differential = true,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
        .input_positive = ADS126X_MUX_AIN3, /* AIN0 */
        .input_negative = ADS126X_MUX_AIN2, /* AIN0 */
#endif
    };

    struct adc_sequence sequence = {
        .channels = 1,
        .buffer = &sample,
        .buffer_size = sizeof(sample),
        .resolution = 32,
        .oversampling = 0,
    };

    struct adc_sequence sequence_1 = {
        .channels = 2,
        .buffer = &sample1,
        .buffer_size = sizeof(sample1),
        .resolution = 32,
        .oversampling = 0,
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

    ret = adc_channel_setup(dev, &channel_2_cfg);
    if (ret) {
        printf("adc channel_2_cfg setup() failed (%d)\r\n", ret);
        return 0;
    }

    k_sleep(K_SECONDS(1));

    // adc_read(dev, &sequence);

    while (1) {

        static bool once = true;

        if (once) {
            // once = false;
            adc_read(dev, &sequence);
            adc_read(dev, &sequence_1);
        }
        k_sleep(K_SECONDS(1));
    }
}