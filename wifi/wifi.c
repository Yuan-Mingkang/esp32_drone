#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/udp.h"
#include "lwip/sockets.h"
#include "wifi.h"
#include "PWM.h"


/* The examples use WiFi configuration that you can set via project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d, reason=%d",
                 MAC2STR(event->mac), event->aid, event->reason);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = true,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
    
}

void udp_server_task(void *pvParameters)
{

    struct sockaddr_in server_addr, client_addr;
    char buf[UDP_BUFFER_SIZE + 1];
    int sock, len;
    socklen_t client_addr_len = sizeof(client_addr);

    // Create UDP socket
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        vTaskDelete(NULL);
    }

    // Initialize server address
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    // Bind the socket to the address
    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind socket");
        close(sock);
        vTaskDelete(NULL);
    }
    char roll_channel_data[5];
    char pitch_channel_data[5];
    char yaw_channel_data[5];
    char throttle_channel_data[5];
    // Receive data from the client
    udpTx.yaw_channel = 0;
    udpTx.throttle_channel = 500;
    udpTx.roll_channel = 500;
    udpTx.pitch_channel = 500;

    udpDataTx = xQueueCreate(2, ITEM_SIZE_UDP);
    
    while (1) {
        // Receive data from the client
        len = recvfrom(sock, buf, UDP_BUFFER_SIZE, 0, (struct sockaddr *)&client_addr, &client_addr_len);
        if (len > 0) {
            // Ensure we received the expected length of data
            if (len == UDP_BUFFER_SIZE) {
                
                // ESP_LOGI(TAG, "Received %d bytes from %s:%d", len, inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
                
                // Process the 16-byte data
                // For demonstration, we simply print the data in hexadecimal format
                // printf("Received data: ");
                for (int i = 0; i < 4; i++) {
                    // printf("%d ", (unsigned char)buf[i]);
                    yaw_channel_data[i] = buf[i];
                    throttle_channel_data[i] = buf[i+4];
                    roll_channel_data[i] = buf[i+8];
                    pitch_channel_data[i] = buf[i+12];
                }
                udpTx.yaw_channel = (int)(atoi(yaw_channel_data) - 1000);
                udpTx.throttle_channel = (int)(atoi(throttle_channel_data)- 1000);
                udpTx.roll_channel = (int)(atoi(roll_channel_data)- 1000);
                udpTx.pitch_channel = (int)(atoi(pitch_channel_data)- 1000);
                        
                // printf("throttle:%d   yaw:%d   roll:%d   pitch:%d\n", 
                // udpTx.throttle_channel, udpTx.yaw_channel, udpTx.roll_channel, udpTx.pitch_channel);
            } else {
                ESP_LOGW(TAG, "Received data length %d is not as expected %d", len, UDP_BUFFER_SIZE);
            }
        }
        xQueueSend(udpDataTx, &udpTx, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(8));
    }
    // close(sock);
    // vTaskDelete(NULL);
}


void wifi_task(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();

    xTaskCreatePinnedToCore(udp_server_task, "WIFI", 2048, NULL, 5, NULL, 0);
}