// I2S ADC 2Msps example
// You need to connect GPIO36(ADC1_CHANNEL_0) and GPIO25(DAC_1)
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"

#include "driver/i2s.h"
#include "driver/adc.h"
#include "driver/dac.h"
#include "soc/syscon_periph.h"
#include "soc/i2s_periph.h"
#include "soc/sens_periph.h"

#define SAMPLE_RATE (2000 * 1000)
#define BUF_LEN 1024
#define PRINT_SIZE 256

void i2s_init(void)
{
    i2s_config_t i2s_conf = {
    	.mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
	    .sample_rate = 1000000,
	    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
	    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
	    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
	    .intr_alloc_flags = 0,
	    .dma_buf_count = 2,
	    .dma_buf_len = BUF_LEN,
	    .use_apll = false
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_conf, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0));
    ESP_ERROR_CHECK(i2s_adc_enable(I2S_NUM_0));

    // delay for I2S bug workaround
    vTaskDelay(10 / portTICK_PERIOD_MS);

    // ***IMPORTANT*** enable continuous adc sampling
    SYSCON.saradc_ctrl2.meas_num_limit = 0;

    // ADC setting
    SYSCON.saradc_sar1_patt_tab[0] = ((ADC1_CHANNEL_0 << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_11) << 24;
    SYSCON.saradc_ctrl.sar1_patt_len = 0;

    // reduce sample time for 2Msps
    SYSCON.saradc_ctrl.sar_clk_div = 2;
    SYSCON.saradc_fsm.sample_cycle = 2;

    // sampling rate 2Msps setting
    I2S0.clkm_conf.clkm_div_num = 20;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_a = 1;
    I2S0.sample_rate_conf.rx_bck_div_num = 2;
}

void dac_init(void)
{
    dac_cw_config_t cw = {
        .en_ch = DAC_CHAN_0,
        .scale = DAC_CW_SCALE_2,
        .phase = DAC_CW_PHASE_0,
        .freq = 10500, // about 10Khz
        .offset = 1,
    };

    ESP_ERROR_CHECK(dac_output_enable(DAC_CHAN_0));
    ESP_ERROR_CHECK(dac_cw_generator_config(&cw));
    ESP_ERROR_CHECK(dac_cw_generator_enable());
}

#define TAG "I2S"

void i2s_read_samples(void)
{
    static uint16_t buf[BUF_LEN];
    int samples = 0;
    size_t bytes_read;
    int i;

    while (1) {
        if (i2s_read(I2S_NUM_0, buf, BUF_LEN*sizeof(uint16_t), &bytes_read, portMAX_DELAY) != ESP_OK) {
            ESP_LOGW(TAG, "i2s_read() fail");
            continue;
        }

        samples += BUF_LEN;

        // output samples every 5sec
        if (samples >= SAMPLE_RATE * 5) {
            samples -= SAMPLE_RATE * 5;
            
            // output only 256 samples
    	    for (i = 0; i < PRINT_SIZE; i++) {
                printf("%d\n", buf[i ^ 1] & 0x0fff);
	        }
            printf("----------------\n");
        }
    }
}

void app_main(void)
{
    dac_init();
    i2s_init();
    i2s_read_samples();
}