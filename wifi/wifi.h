#ifndef WIFI_H_
#define WIFI_H_

#define EXAMPLE_ESP_WIFI_SSID      "PicoW"
#define EXAMPLE_ESP_WIFI_PASS      "password"
#define EXAMPLE_ESP_WIFI_CHANNEL   1
#define EXAMPLE_MAX_STA_CONN       5
#define PORT 8888
#define UDP_BUFFER_SIZE 16


static const char *TAG = "wifi softAP";
static const char *TAG_re = "udp_server";

typedef struct {
    int yaw_channel;
    int throttle_channel;
    int roll_channel;
    int pitch_channel;
} UDPPacket;

static QueueHandle_t udpDataTx;
static UDPPacket udpTx;
#define ITEM_SIZE_UDP sizeof(UDPPacket)

// extern struct Remote inPacket;

void wifi_init_softap(void);
// void udp_server_task(void);
void wifi_task(void);
#endif