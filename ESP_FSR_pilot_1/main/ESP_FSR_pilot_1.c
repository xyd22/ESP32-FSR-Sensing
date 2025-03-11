#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"


// Curcuit
#define R_FIXED 1000  // 1kΩ
#define R_SERIES 220 // seried resistor 220Ω
#define R_DVD 2000 // 2kΩ
// #define R_DVD 100000 // 100kΩ

#define ADC_WIDTH ADC_WIDTH_BIT_12  // 12 bits ADC
#define ADC_ATTEN ADC_ATTEN_DB_11   // 0V - 3.3V

#define NUM_ADC_CHANNELS 5
#define ADC_CHANNEL_2 ADC1_CHANNEL_6   // A2 34
#define ADC_CHANNEL_3 ADC1_CHANNEL_3   // A3 39
#define ADC_CHANNEL_4 ADC1_CHANNEL_0   // A4 36
#define ADC_CHANNEL_7 ADC1_CHANNEL_4   // A7 32
#define ADC_CHANNEL_9 ADC1_CHANNEL_5   // A9 33

// Server
#define WIFI_SSID "TP-Link_AA24"
#define WIFI_PASS "lab237237"
#define SERVER_IP "192.168.0.114"  // 上位机的 IP 地址
#define SERVER_PORT 1234

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define EXAMPLE_ESP_MAXIMUM_RETRY 5 
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

#define SAMPLE_INTERVAL_MS 10 // sampling interval

#define BUF_SIZE 64

static const char *TAG = "VARISTOR_TEST";
static const char *TAG_Wifi = "WIFI_TCP";


void init_adc() {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL_2, ADC_ATTEN);
    adc1_config_channel_atten(ADC_CHANNEL_3, ADC_ATTEN);
    adc1_config_channel_atten(ADC_CHANNEL_4, ADC_ATTEN);
    adc1_config_channel_atten(ADC_CHANNEL_7, ADC_ATTEN);
    adc1_config_channel_atten(ADC_CHANNEL_9, ADC_ATTEN);
}

void read_adc(uint32_t adc_values[NUM_ADC_CHANNELS]) {
    adc_values[0] = adc1_get_raw(ADC_CHANNEL_2);  // 读取通道 2
    adc_values[1] = adc1_get_raw(ADC_CHANNEL_3);  // 读取通道 3
    adc_values[2] = adc1_get_raw(ADC_CHANNEL_4);  // 读取通道 4
    adc_values[3] = adc1_get_raw(ADC_CHANNEL_7);  // 读取通道 7
    adc_values[4] = adc1_get_raw(ADC_CHANNEL_9);  // 读取通道 9
}

void calculate_varistor_resistance(float r_var[NUM_ADC_CHANNELS], float voltage[NUM_ADC_CHANNELS],uint32_t adc_value[NUM_ADC_CHANNELS]) {
    float current;
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        voltage[i] = (float)adc_value[i] / 4095 * 3.3;
        if (adc_value[i] == 4095) {
            r_var[i] = INFINITY;
        }
        current =  voltage[i] / R_DVD;
        r_var[i] = (3.3 - voltage[i]) / current;
    }
}

void resistance_to_force(float resistance[NUM_ADC_CHANNELS], float force[NUM_ADC_CHANNELS]) {
    // get the transition from website
    float fsrConductance;           // measure in micromhos
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        fsrConductance = 1000000; 
        fsrConductance /= resistance[i];
        if (fsrConductance <= 1000) {
            force[i] = fsrConductance / 80;   
        } else {
            force[i] = fsrConductance - 1000;
            force[i] /= 30;        
        }
    }
}

void float_array_to_bytes(float *float_array, uint8_t *byte_array, size_t float_count) {
    memcpy(byte_array, float_array, float_count * sizeof(float));
}

static void event_handler(void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
void wifi_init_sta() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            // .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            // .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG_Wifi, "Wi-Fi initialized");
}

void send_sensor_data() {
    // TCP socket
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG_Wifi, "Failed to create socket");
        return;
    }

    // Server Address
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    s_wifi_event_group = xEventGroupCreate();
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
    * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap");
    }

    // Connect to server
    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
        printf("Failed to connect to server: %s\n", strerror(errno));
        close(sock);
        return;
    }

    uint32_t adc_values[NUM_ADC_CHANNELS];
    float v_out[NUM_ADC_CHANNELS];
    float r_var[NUM_ADC_CHANNELS];
    float force[NUM_ADC_CHANNELS];
    uint8_t data_bytes[BUF_SIZE * NUM_ADC_CHANNELS * sizeof(float)];

    while(1){
        for(int i = 0; i < BUF_SIZE; i++) {
            // Read sensor data
            read_adc(adc_values);
            calculate_varistor_resistance(r_var, v_out, adc_values);
            resistance_to_force(r_var, force);

            // Convert
            float_array_to_bytes(&v_out, &data_bytes + i * NUM_ADC_CHANNELS * sizeof(float), NUM_ADC_CHANNELS);
        }

        // 发送数据
        int sent = send(sock, data_bytes, sizeof(float), 0);
        if (sent < 0) {
            ESP_LOGE(TAG, "Failed to send data");
        } else {
            ESP_LOGI(TAG, "Data sent");
        }
    }

    // 关闭套接字
    close(sock);
}

void app_main() {
    init_adc();

    // uint32_t adc_values[NUM_ADC_CHANNELS];
    // float buffer[BUF_SIZE * NUM_ADC_CHANNELS];
    
    // float v_out[NUM_ADC_CHANNELS];
    // float r_var[NUM_ADC_CHANNELS];
    // float force[NUM_ADC_CHANNELS];

    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Init Wi-Fi
    wifi_init_sta();

    // Waiting for Wifi
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    // Send sensor data
    send_sensor_data();
}