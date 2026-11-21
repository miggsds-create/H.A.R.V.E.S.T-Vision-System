#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_camera.h"
#include "nvs_flash.h"
// NimBLE Includes
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
// Forward declaration of ESP-NOW initializer from esp_now.c
esp_err_t init_esp_now_node(void);
static const char *TAG = "CAMERA_BLE";

#define CHUNK_SIZE 490   // 1 flag byte + 489 payload bytes per notification
static uint8_t own_addr_type;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t gatt_chr_val_handle;
// True only while the browser has switched notifications on for the camera characteristic
static volatile bool notify_enabled = false;
static void ble_app_advertise(void);
// -----------------------------------------------------------------------------
// BLE GATT & GAP Configuration
// -----------------------------------------------------------------------------
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x2d, 0x71, 0xa2, 0x59, 0xb4, 0x58, 0x47, 0xef,
                     0xbf, 0x2b, 0x82, 0x12, 0x01, 0x3d, 0xfa, 0x12);
static const ble_uuid128_t gatt_svr_chr_uuid =
    BLE_UUID128_INIT(0x2d, 0x71, 0xa2, 0x59, 0xb4, 0x58, 0x47, 0xef,
                     0xbf, 0x2b, 0x82, 0x12, 0x02, 0x3d, 0xfa, 0x12);
static int gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
}
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) { {
            .uuid = &gatt_svr_chr_uuid.u,
            .access_cb = gatt_svr_chr_access,
            .val_handle = &gatt_chr_val_handle,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        }, { 0 } }
    },
    { 0 }
};
static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "Connect event, status=%d", event->connect.status);
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "BLE Client Connected!");
            conn_handle = event->connect.conn_handle;

            struct ble_gap_upd_params params = {
                .itvl_min = 6,  // 7.5 ms
                .itvl_max = 12, // 15 ms
                .latency = 0,
                .supervision_timeout = 400,
                .min_ce_len = 0,
                .max_ce_len = 0
            };
            ble_gap_update_params(conn_handle, &params);
            ble_gattc_exchange_mtu(conn_handle, NULL, NULL);
        } else {
            // The connection attempt failed: advertising has stopped, so start it again
            ble_app_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        // reason = 0x200 + HCI code, e.g. 0x208 supervision timeout, 0x213 remote hung up, 0x23e failed to establish
        ESP_LOGI(TAG, "BLE Client Disconnected! reason=0x%x", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        notify_enabled = false;
        ble_app_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        // Only start streaming once the browser has finished discovering services and turned notifications on
        if (event->subscribe.attr_handle == gatt_chr_val_handle) {
            notify_enabled = event->subscribe.cur_notify;
            ESP_LOGI(TAG, "Notifications %s", notify_enabled ? "enabled" : "disabled");
        }
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU = %d (each notification carries MTU-3 bytes; needs >= %d)",
                 event->mtu.value, CHUNK_SIZE + 3);
        break;
    }
    return 0;
}

static void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)"ESP32S3_CAM";
    fields.name_len = strlen("ESP32S3_CAM");
    fields.name_is_complete = 1;
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) return;

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(20);
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(40);
    ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_cb, NULL);
}
static void ble_on_sync(void) {
    int rc;
    ble_addr_t rnd_addr;
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        rc = ble_hs_id_gen_rnd(0, &rnd_addr);
        if (rc == 0) {
            ble_hs_id_set_rnd(rnd_addr.val);
            own_addr_type = BLE_OWN_ADDR_RANDOM;
        }
    }
    ble_app_advertise();
}
static void host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}
// -----------------------------------------------------------------------------
// Camera Setup (AI-Thinker / ESP32-S3 Sense Pinout)
// -----------------------------------------------------------------------------
#define XCLK_GPIO_NUM  45
#define SIOD_GPIO_NUM  1
#define SIOC_GPIO_NUM  2
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    46
#define Y7_GPIO_NUM    8
#define Y6_GPIO_NUM    7
#define Y5_GPIO_NUM    4
#define Y4_GPIO_NUM    41
#define Y3_GPIO_NUM    40
#define Y2_GPIO_NUM    39
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  42
#define PCLK_GPIO_NUM  5
static camera_config_t camera_config = {
    .pin_pwdn = -1,
    .pin_reset = -1,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sccb_sda = SIOD_GPIO_NUM,
    .pin_sccb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_HVGA,   // 320x240
    .jpeg_quality = 10,
    .fb_count = 2,
    .grab_mode = CAMERA_GRAB_LATEST,
    .fb_location = CAMERA_FB_IN_PSRAM,
};
// -----------------------------------------------------------------------------
// Stream Task
// -----------------------------------------------------------------------------
static void stream_task(void *pvParameters) {
    uint8_t packet[CHUNK_SIZE];
    while (1) {
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE && notify_enabled) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                size_t offset = 0;
                while (offset < fb->len && conn_handle != BLE_HS_CONN_HANDLE_NONE && notify_enabled) {
                    size_t remaining = fb->len - offset;
                    size_t payload_len = (remaining > (CHUNK_SIZE - 1)) ? (CHUNK_SIZE - 1) : remaining;
                    if (offset == 0) {
                        packet[0] = 0x01; // START flag
                    } else if (offset + payload_len >= fb->len) {
                        packet[0] = 0x02; // END flag
                    } else {
                        packet[0] = 0x00; // MID flag
                    }
                    memcpy(&packet[1], fb->buf + offset, payload_len);
                    struct os_mbuf *om = ble_hs_mbuf_from_flat(packet, payload_len + 1);
                    if (om != NULL) {
                        int rc = ble_gatts_notify_custom(conn_handle, gatt_chr_val_handle, om);
                        if (rc == 0) {
                            offset += payload_len;
                        } else {
                            os_mbuf_free_chain(om);
                            if (rc == BLE_HS_ENOMEM || rc == BLE_HS_EBUSY || rc == BLE_HS_EAGAIN) {
                                vTaskDelay(pdMS_TO_TICKS(5)); // Back off slightly to clear stack
                            } else {
                                ESP_LOGW(TAG, "GATT notification error: %d, breaking frame stream", rc);
                                break;
                            }
                        }
                    } else {
                        vTaskDelay(pdMS_TO_TICKS(2));
                    }
                }
                esp_camera_fb_return(fb);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
// -----------------------------------------------------------------------------
// Application Main
// -----------------------------------------------------------------------------
void app_main(void) {
    // NimBLE prints 2-3 lines per notification; at ~90 notifications/s that floods the 115200 baud console
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    esp_err_t err = esp_camera_init(&camera_config);
    if (err == ESP_OK) {
        sensor_t *s = esp_camera_sensor_get();
        if (s != NULL) {
            s->set_vflip(s, 1);
            s->set_sharpness(s, 2);
            s->set_contrast(s, 1);
            s->set_brightness(s, 0);
        }
        ESP_LOGI(TAG, "Camera Initialized Successfully!");
    } else {
        ESP_LOGE(TAG, "Camera Init Failed: 0x%x", err);
    }
    // Initialize NimBLE Stack & Streaming Task
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
        return;
    }

    int rc = ble_svc_gap_device_name_set("ESP32S3_CAM");
    assert(rc == 0);

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(host_task);

    xTaskCreate(stream_task, "ble_cam_stream", 4096, NULL, 5, NULL);

    // Start Wi-Fi / ESP-NOW LAST so BLE gets its memory first and a Wi-Fi
    // failure can never stop the camera stream.
    if (init_esp_now_node() != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW failed to start; BLE camera keeps running without it");
    }
}
