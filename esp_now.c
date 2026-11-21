#include <stdio.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_now.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "ESP_NOW_NODE";

// Must match the channel the kinematics board is on (both boards unconnected
// to an AP -> channel 1 unless you set otherwise).
#define ESPNOW_CHANNEL 1

// Kinematics Board MAC Address
static uint8_t kinematics_mac[6] = {0xB8, 0x1F, 0x3F, 0xDA, 0x80, 0x28};

typedef enum {
    STATE_IDLE = 0,
    STATE_TRACKING,
    STATE_GRASP_READY
} HarvestState_t;

typedef struct {
    uint8_t target_id;
    HarvestState_t state;
    float x_mm;
    float y_mm;
    float z_mm;
    float yaw_deg;
    uint8_t ripeness_class;
} __attribute__((packed)) RobotCommandPacket_t;

// What actually goes over the air. MUST match the kinematics board's
// vision_message struct exactly: 4 floats, 16 bytes, positions in METRES.
typedef struct {
    float x_m;
    float y_m;
    float z_m;
    float yaw_deg;
} VisionWire_t;
_Static_assert(sizeof(VisionWire_t) == 16, "VisionWire_t must be 16 bytes");

// 1 = after startup, pretend a target was seen at a fixed point so the other
// board can be tested without the tracker. Set to 0 once real data flows.
#define ESPNOW_SELFTEST 1

static RobotCommandPacket_t current_packet = {
    .target_id = 0,
    .state = STATE_IDLE,
    .x_mm = 0.0f,
    .y_mm = 0.0f,
    .z_mm = 0.0f,
    .yaw_deg = 0.0f,
    .ripeness_class = 0
};

static SemaphoreHandle_t packet_mutex = NULL;

// IDF 5.5+ signature. On older IDF use: (const uint8_t *mac_addr, esp_now_send_status_t status)
static void esp_now_send_cb(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    (void)tx_info;
    (void)status;
}

// Log-and-return instead of ESP_ERROR_CHECK, so a Wi-Fi/ESP-NOW failure
// can never abort() the chip before BLE has started.
#define TRY(x) do {                                                     \
        esp_err_t _e = (x);                                             \
        if (_e != ESP_OK) {                                             \
            ESP_LOGE(TAG, "%s failed: %s", #x, esp_err_to_name(_e));    \
            return _e;                                                  \
        }                                                               \
    } while (0)

// Thread-safe update with a 2 mm deadband
void update_strawberry_target(uint8_t id, HarvestState_t st, float x, float y, float z, float yaw, uint8_t ripeness) {
    if (packet_mutex == NULL) return;

    // Ignore empty frames where nothing was detected
    if (x == 0.0f && y == 0.0f && z == 0.0f) {
        return;
    }

    if (xSemaphoreTake(packet_mutex, portMAX_DELAY) == pdTRUE) {
        float x_diff = fabsf(current_packet.x_mm - x);
        float y_diff = fabsf(current_packet.y_mm - y);
        float z_diff = fabsf(current_packet.z_mm - z);

        if (x_diff > 2.0f || y_diff > 2.0f || z_diff > 2.0f || current_packet.state != st) {
            current_packet.target_id = id;
            current_packet.state = st;
            current_packet.x_mm = x;
            current_packet.y_mm = y;
            current_packet.z_mm = z;
            current_packet.yaw_deg = yaw;
            current_packet.ripeness_class = ripeness;
        }
        xSemaphoreGive(packet_mutex);
    }
}

// Background task, 10 Hz
static void esp_now_task(void *pvParameters) {
    RobotCommandPacket_t packet_to_send;

    while (1) {
        if (packet_mutex != NULL && xSemaphoreTake(packet_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            memcpy(&packet_to_send, &current_packet, sizeof(RobotCommandPacket_t));
            xSemaphoreGive(packet_mutex);

            // Only transmit while there is a real target: the other board treats
            // every packet as a new target, so idle zeros would make it solve IK for (0,0,0).
            if (packet_to_send.state != STATE_IDLE) {
                VisionWire_t wire = {
                    .x_m     = packet_to_send.x_mm / 1000.0f,   // mm -> m
                    .y_m     = packet_to_send.y_mm / 1000.0f,
                    .z_m     = packet_to_send.z_mm / 1000.0f,
                    .yaw_deg = packet_to_send.yaw_deg,
                };
                esp_err_t result = esp_now_send(kinematics_mac, (uint8_t *)&wire, sizeof(wire));
                if (result != ESP_OK) {
                    ESP_LOGE(TAG, "esp_now_send failed: %s", esp_err_to_name(result));
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Call this AFTER the BLE stack is up. Returns an error instead of aborting.
esp_err_t init_esp_now_node(void) {
    ESP_LOGI(TAG, "Internal heap before Wi-Fi: free=%u, largest block=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    packet_mutex = xSemaphoreCreateMutex();
    if (packet_mutex == NULL) return ESP_ERR_NO_MEM;

    TRY(esp_netif_init());

    // Default event loop is required by the Wi-Fi driver. It may already exist.
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        return err;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    TRY(esp_wifi_init(&cfg));
    TRY(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    TRY(esp_wifi_set_mode(WIFI_MODE_STA));
    TRY(esp_wifi_start());
    TRY(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    TRY(esp_now_init());
    TRY(esp_now_register_send_cb(esp_now_send_cb));

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, kinematics_mac, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;
    TRY(esp_now_add_peer(&peerInfo));

    if (xTaskCreate(esp_now_task, "esp_now_task", 3072, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create esp_now_task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "ESP-NOW ready. Internal heap now: free=%u, largest block=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
#if ESPNOW_SELFTEST
    update_strawberry_target(1, STATE_TRACKING, 123.0f, 45.0f, 200.0f, 10.0f, 2);   // mm, mm, mm, deg
#endif
    return ESP_OK;
}