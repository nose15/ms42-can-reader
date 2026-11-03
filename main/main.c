#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_twai_types.h"

static const char *logging_tag = "CAN";

static portMUX_TYPE rx_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool frame_pending = false;
static twai_frame_header_t latest_header;
static uint8_t latest_data[8];

static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };

    if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK) {
        portENTER_CRITICAL_ISR(&rx_mux);
        latest_header = rx_frame.header;
        memcpy(latest_data, recv_buff, sizeof(recv_buff));
        frame_pending = true;
        portEXIT_CRITICAL_ISR(&rx_mux);
    }

    return false;
}

static void log_latest_frame(void)
{
    twai_frame_header_t header;
    uint8_t data[8];

    portENTER_CRITICAL(&rx_mux);
    if (!frame_pending) {
        portEXIT_CRITICAL(&rx_mux);
        return;
    }
    header = latest_header;
    memcpy(data, latest_data, sizeof(data));
    frame_pending = false;
    portEXIT_CRITICAL(&rx_mux);

    uint16_t len = twaifd_dlc2len(header.dlc);
    if (len > sizeof(data)) {
        len = sizeof(data);
    }

    char data_str[8 * 3 + 1] = "";
    for (uint16_t i = 0; i < len; i++) {
        snprintf(data_str + i * 3, 4, "%02X ", data[i]);
    }

    ESP_LOGI(logging_tag, "ID: 0x%lX  DLC: %u  %s",
             (unsigned long)header.id, header.dlc, data_str);
}

void app_main(void)
{
    twai_node_handle_t node_handle = NULL;
    twai_onchip_node_config_t node_cfg = {
        .io_cfg.rx = 4,
        .io_cfg.tx = 5,
        .bit_timing.bitrate = 500000,
        .flags.enable_listen_only = true
    };

    twai_event_callbacks_t user_cbs = {
        .on_rx_done = twai_rx_cb,
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_cfg, &node_handle));
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_handle, &user_cbs, NULL));
    ESP_ERROR_CHECK(twai_node_enable(node_handle));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        log_latest_frame();
    }
}