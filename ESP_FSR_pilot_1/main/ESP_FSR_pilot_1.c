// TEST

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_log.h"

// 电路参数
#define R_FIXED 1000  // 1kΩ
#define R_SERIES 220  // 串联电阻 220Ω
#define R_DVD 2000    // 2kΩ

#define ADC_WIDTH ADC_WIDTH_BIT_12  // 12 位 ADC
#define ADC_ATTEN ADC_ATTEN_DB_11   // 0V - 3.3V

#define NUM_ADC_CHANNELS 5
#define ADC_CHANNEL_2 ADC1_CHANNEL_6   // A2 34
#define ADC_CHANNEL_3 ADC1_CHANNEL_3   // A3 39
#define ADC_CHANNEL_4 ADC1_CHANNEL_0   // A4 36
#define ADC_CHANNEL_7 ADC1_CHANNEL_4   // A7 32
#define ADC_CHANNEL_9 ADC1_CHANNEL_5   // A9 33

#define SAMPLE_INTERVAL_MS 10  // 采样间隔
#define BUF_SIZE 64

static const char *TAG = "VARISTOR_TEST";

// 初始化 ADC
void init_adc() {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL_2, ADC_ATTEN);
    adc1_config_channel_atten(ADC_CHANNEL_3, ADC_ATTEN);
    adc1_config_channel_atten(ADC_CHANNEL_4, ADC_ATTEN);
    adc1_config_channel_atten(ADC_CHANNEL_7, ADC_ATTEN);
    adc1_config_channel_atten(ADC_CHANNEL_9, ADC_ATTEN);
}

// 读取 ADC 值
void read_adc(uint32_t adc_values[NUM_ADC_CHANNELS]) {
    adc_values[0] = adc1_get_raw(ADC_CHANNEL_2);  // 读取通道 2
    adc_values[1] = adc1_get_raw(ADC_CHANNEL_3);  // 读取通道 3
    adc_values[2] = adc1_get_raw(ADC_CHANNEL_4);  // 读取通道 4
    adc_values[3] = adc1_get_raw(ADC_CHANNEL_7);  // 读取通道 7
    adc_values[4] = adc1_get_raw(ADC_CHANNEL_9);  // 读取通道 9
}

// 计算压敏电阻的阻值
void calculate_varistor_resistance(float r_var[NUM_ADC_CHANNELS], float voltage[NUM_ADC_CHANNELS], uint32_t adc_value[NUM_ADC_CHANNELS]) {
    float current;
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        voltage[i] = (float)adc_value[i] / 4095 * 3.3;  // 将 ADC 值转换为电压
        if (adc_value[i] == 0) {
            r_var[i] = INFINITY;  // 如果 ADC 值达到最大值，阻值为无穷大
        } else {
            current = voltage[i] / R_DVD;  // 计算电流
            r_var[i] = (3.3 - voltage[i]) / current;  // 计算压敏电阻阻值
        }
    }
}

// 将阻值转换为力
void resistance_to_force(float resistance[NUM_ADC_CHANNELS], float force[NUM_ADC_CHANNELS]) {
    float fsrConductance;  // 电导（单位：微西门子）
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        if (resistance[i] == INFINITY) {
            force[i] = 0;  // 如果阻值为无穷大，力为 0
        } else {
            fsrConductance = 1000000 / resistance[i];  // 计算电导
            if (fsrConductance <= 1000) {
                force[i] = fsrConductance / 80;  // 小力范围
            } else {
                force[i] = (fsrConductance - 1000) / 30;  // 大力范围
            }
        }
    }
}

// 打印采集到的数据
void print_sensor_data(uint32_t adc_values[NUM_ADC_CHANNELS], float voltage[NUM_ADC_CHANNELS], float r_var[NUM_ADC_CHANNELS], float force[NUM_ADC_CHANNELS]) {
    ESP_LOGI(TAG, "Voltage: [%.3f, %.3f, %.3f, %.3f, %.3f] V",
             voltage[0], voltage[1], voltage[2], voltage[3], voltage[4]);
}

// 主任务
void app_main() {
    // 初始化 ADC
    init_adc();

    uint32_t adc_values[NUM_ADC_CHANNELS];
    float voltage[NUM_ADC_CHANNELS];
    float r_var[NUM_ADC_CHANNELS];
    float force[NUM_ADC_CHANNELS];

    while (1) {
        // 读取 ADC 值
        read_adc(adc_values);

        // 计算电压、阻值和力
        calculate_varistor_resistance(r_var, voltage, adc_values);
        resistance_to_force(r_var, force);

        // 打印采集到的数据
        print_sensor_data(adc_values, voltage, r_var, force);

        // 等待采样间隔
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}