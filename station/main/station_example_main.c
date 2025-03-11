#include <string.h>
#include <stdio.h>
#include <math.h>
#include "driver/adc.h"
// #include "esp_adc/adc_oneshot.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

/* Circuit */
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

#define SAMPLE_INTERVAL_MS 10 // sampling interval
#define BUF_SIZE 64

/* WiFi configuration */
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

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

/* Server configuration */
#define SERVER_IP   "192.168.0.114"  // 替换为你的 PC 的 IP 地址
#define SERVER_PORT 1234             // 替换为你的服务器端口

/* Event group and retry counter */
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "wifi station";
static int s_retry_num = 0;

/* FreeRTOS event bits */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/*Hardware related functions*/
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

/* WiFi event handler */
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

/* Initialize WiFi station */
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Wait for WiFi connection */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

/* TCP client task */
void tcp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char tx_buffer[128];
    int sock;

    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);

    while(1){
        /* Create socket */
        sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", SERVER_IP, SERVER_PORT);

        /* Connect to server */
        if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr))) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "Successfully connected");

        uint32_t adc_values[NUM_ADC_CHANNELS];
        float v_out[NUM_ADC_CHANNELS];
        float r_var[NUM_ADC_CHANNELS];
        float force[NUM_ADC_CHANNELS];
        uint8_t data_bytes[BUF_SIZE * NUM_ADC_CHANNELS * sizeof(float)];

        while (1) {
            /* Send data to server */
            for(int i = 0; i < BUF_SIZE; i++) {
                // Read sensor data
                read_adc(adc_values);
                calculate_varistor_resistance(r_var, v_out, adc_values);
                resistance_to_force(r_var, force);

                // Convert
                float_array_to_bytes(v_out, data_bytes + i * NUM_ADC_CHANNELS * sizeof(float), NUM_ADC_CHANNELS);

                // Frequency control
                vTaskDelay(SAMPLE_INTERVAL_MS / portTICK_PERIOD_MS);
            }

            // 发送数据
            int sent = send(sock, data_bytes, BUF_SIZE * NUM_ADC_CHANNELS * sizeof(float), 0);
            if (sent < 0) {
                ESP_LOGE(TAG, "Failed to send data");
            } else {
                ESP_LOGI(TAG, "Data sent");
            }
        }

        /* Close socket and retry */
        close(sock);
        ESP_LOGI(TAG, "Shutting down socket and retrying...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/* Main application */
void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    // Start TCP client task
    xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
}