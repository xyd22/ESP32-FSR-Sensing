#include <string.h>
#include <stdio.h>
#include <math.h>
#include "driver/adc.h"
// #include "esp_adc/adc_oneshot.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sntp.h"
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
#define LEN_TIMESTAMP 2

/* WiFi configuration */
// Wi-Fi MAC Address: 3C:71:BF:14:E6:C4
// !!! Remember to change the wifi config from public to private
// #define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
// #define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_SSID      "RedRover"
#define EXAMPLE_ESP_WIFI_PASS      ""
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

/* Server configuration */
// #define SERVER_IP   "192.168.0.114" // TP-Link_AA24
#define SERVER_IP "10.49.28.95" // RedRover (changes every time)
#define SERVER_PORT 1234     

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

/* FreeRTOS event bits */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/* Event group and retry counter */
static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "ESP32";
static int s_retry_num = 0;

uint32_t Package_count = 0;

/* Data structure */
typedef struct {
    uint8_t package_count[sizeof(uint32_t)];
    uint8_t time_bytes[LEN_TIMESTAMP * sizeof(uint32_t)];
    uint8_t data_bytes[BUF_SIZE * NUM_ADC_CHANNELS * sizeof(float)];
} sensor_data_t;

/* Queue handle */
QueueHandle_t data_queue;


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

void calculate_varistor_resistance(float r_var[NUM_ADC_CHANNELS], float voltage[NUM_ADC_CHANNELS], uint32_t adc_value[NUM_ADC_CHANNELS]) {
    float current;
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
        voltage[i] = (float)adc_value[i] / 4095 * 3.3;
        if (adc_value[i] == 0) {
            r_var[i] = INFINITY;
        }
        else{
            current =  voltage[i] / R_DVD;
            r_var[i] = (3.3 - voltage[i]) / current;
        }
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

void uint32_array_to_bytes(uint32_t *data_uint32, uint8_t *byte_array, size_t uint32_count) {
    memcpy(byte_array, data_uint32, uint32_count * sizeof(uint32_t));
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
            // .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .threshold.authmode = WIFI_AUTH_OPEN,
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

void set_timezone_est(void)
{
    // Set the timezone to EST (UTC-5) and EDT (UTC-4)
    setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
    tzset();
}

void initialize_sntp(void)
{
    ESP_LOGI("SNTP", "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    // Wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI("SNTP", "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
}

void format_timestamp(char *buffer, size_t buffer_size, uint32_t *time_output)
{
    struct timeval tv;
    struct tm timeinfo;

    gettimeofday(&tv, NULL);

    localtime_r(&tv.tv_sec, &timeinfo);

    time_t unix_ts_s = tv.tv_sec;
    time_t unix_ts_us = tv.tv_usec;

    time_output[0] = (uint32_t)unix_ts_s;
    time_output[1] = (uint32_t)unix_ts_us;
}

void data_acquisition_task(void *pvParameters)
{
    uint32_t adc_values[NUM_ADC_CHANNELS];
    float v_out[NUM_ADC_CHANNELS];
    float r_var[NUM_ADC_CHANNELS];
    float force[NUM_ADC_CHANNELS];
    char timestamp[32];
    uint32_t time_output[LEN_TIMESTAMP];

    while (1) {
        // Malloc
        sensor_data_t *data_collect = malloc(sizeof(sensor_data_t));
        if (data_collect == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for sensor data");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        // Package Count
        Package_count++;
        if(Package_count % 100 == 0) Package_count = 0;
        uint32_array_to_bytes(&Package_count, data_collect->package_count, 1);
        // printf("%u", data_collect->package_count[0]);

        // Timestamp
        format_timestamp(timestamp, sizeof(timestamp), time_output);
        uint32_array_to_bytes(time_output, data_collect->time_bytes, LEN_TIMESTAMP);

        // Read
        for (int i = 0; i < BUF_SIZE; i++) {
            read_adc(adc_values);
            calculate_varistor_resistance(r_var, v_out, adc_values);
            resistance_to_force(r_var, force);

            float_array_to_bytes(v_out, data_collect->data_bytes + i * NUM_ADC_CHANNELS * sizeof(float), NUM_ADC_CHANNELS);
            // printf("Data: ");
            // for (int j = 0; j < NUM_ADC_CHANNELS; j++) {
            //     printf("%lu ", adc_values[j]);
            // }
            // printf("\n");

            vTaskDelay(SAMPLE_INTERVAL_MS / portTICK_PERIOD_MS);
        }

        // Send to queue
        if (xQueueSend(data_queue, &data_collect, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to send data to queue");
            free(data_collect);
        }
    }
}

void data_sending_task(void *pvParameters)
{
    int sock;
    struct sockaddr_in server_addr;
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    int send_buf_size = 2048;

    while (1) {
        // Create socket
        sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size));

        // Connect to server
        if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr))) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "Successfully connected");

        // Clear queue and free memory
        while (uxQueueMessagesWaiting(data_queue) > 0) {
            sensor_data_t *data;
            if (xQueueReceive(data_queue, &data, 0) == pdTRUE) {
                free(data);
                // ??
            }
        }

        ESP_LOGI(TAG, "Queue cleared");

        Package_count = 0;

        while (1) {
            // Receive from queue
            // sensor_data_t *data_send = malloc(sizeof(sensor_data_t));
            sensor_data_t *data_send;
            if (xQueueReceive(data_queue, &data_send, portMAX_DELAY) == pdTRUE) {

                // Merge
                uint8_t merged_array[sizeof(uint32_t) + LEN_TIMESTAMP * sizeof(uint32_t) + BUF_SIZE * NUM_ADC_CHANNELS * sizeof(float)];
                memcpy(merged_array, data_send->package_count, sizeof(uint32_t));
                memcpy(merged_array + sizeof(uint32_t), data_send->time_bytes, LEN_TIMESTAMP * sizeof(uint32_t));
                memcpy(merged_array + sizeof(uint32_t) + LEN_TIMESTAMP * sizeof(uint32_t), data_send->data_bytes, BUF_SIZE * NUM_ADC_CHANNELS * sizeof(float));
                ESP_LOGI(TAG, "Data Length: %u", sizeof(merged_array));

                // Send
                int sent = send(sock, merged_array, sizeof(merged_array), 0);
                if (sent < 0) {
                    ESP_LOGE(TAG, "Failed to send data, errno: %d", errno);
                    printf("Data: ");
                    for (int i = 0; i < sizeof(merged_array); i++) {
                        printf("%02x ", merged_array[i]);
                    }                    
                    break;
                } else {
                    ESP_LOGI(TAG, "# %lu, Data sent %u", Package_count, sizeof(merged_array));
                    // printf("Data: ");
                    // for (int i = 0; i < sizeof(merged_array); i++) {
                    //     printf("%02x ", merged_array[i]);
                    // }
                }
                ESP_LOGI(TAG, "Queue messages: %u", uxQueueMessagesWaiting(data_queue));
                if (uxQueueSpacesAvailable(data_queue) == 0) {
                    ESP_LOGE(TAG, "Queue is full, dropping data");
                }

                free(data_send);
                ESP_LOGI(TAG, "Free heap: %lu", esp_get_free_heap_size());
            }
        }

        close(sock);
        ESP_LOGI(TAG, "Shutting down socket and retrying...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/* Main application */
void app_main(void)
{
    // Initialize ADC
    init_adc();

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    set_timezone_est();
    initialize_sntp();

    // Start TCP client task
    // xTaskCreate(tcp_client_task, "tcp_client", 8192, NULL, 5, NULL);

    // Multithreads
    data_queue = xQueueCreate(10, sizeof(sensor_data_t *)); 
    if (data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }
    // xTaskCreate(data_acquisition_task, "data_acquisition", 8192, NULL, 5, NULL);
    // xTaskCreate(data_sending_task, "data_sending", 8192, NULL, 5, NULL);

    xTaskCreatePinnedToCore(data_acquisition_task, "data_acquisition", 8192, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(data_sending_task, "data_sending", 8192, NULL, 6, NULL, 1);
}