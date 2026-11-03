#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_twai_types.h"

#include "hal/twai_types.h"
#include "soc/gpio_num.h"
#include <stdatomic.h>
#include <stdint.h>

const uint32_t DME2_MESSAGE_ID = 0x329;
const uint32_t MASK_FIRST_11_BITS = 0x7FF;
const uint32_t CAN_BUS_BITRATE = 500000;
const gpio_num_t CAN_BUS_RX_PIN = 4;
const gpio_num_t CAN_BUS_TX_PIN = 5;

static const char *LOGGING_TAG = "CAN";

static const twai_onchip_node_config_t node_cfg = {
    .io_cfg.rx = CAN_BUS_RX_PIN,
    .io_cfg.tx = CAN_BUS_TX_PIN,
    .bit_timing.bitrate = CAN_BUS_BITRATE,
    .flags.enable_listen_only = true
};

static const twai_mask_filter_config_t twai_filter_config = {
    .id = DME2_MESSAGE_ID,
    .mask = MASK_FIRST_11_BITS,
    .is_ext = false
};

atomic_uint_fast8_t engine_temp_reading = 0;

static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx) {
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };

    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
        engine_temp_reading = recv_buff[1];
    }

    return false;
}

inline uint8_t calculate_engine_temp(uint8_t reading) {
    /*
        It is calculated using the method provided by MS4X Wiki.
        https://www.ms4x.net/index.php?title=Siemens_MS42_CAN_Bus
    */

    return reading * 0.75 - 48;
}

void configure_twai(void) {
    twai_node_handle_t node_handle = NULL;

    twai_event_callbacks_t user_cbs = {
        .on_rx_done = twai_rx_cb,
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_cfg, &node_handle));
    ESP_ERROR_CHECK(twai_node_config_mask_filter(node_handle, 0, &twai_filter_config));
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_handle, &user_cbs, NULL));
    ESP_ERROR_CHECK(twai_node_enable(node_handle));

    while (1) {
        uint8_t engine_temp = calculate_engine_temp(engine_temp_reading);
        ESP_LOGI(LOGGING_TAG, "Tmp = %d°", engine_temp);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void) {
    configure_twai();
}
