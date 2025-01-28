#include "driver/i2s_std.h"
#include "esp_log.h"
#include <string.h>
#include "i2s_debug.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "I2S_FLOW_DEBUG"

void debug_i2s_data_flow() {
    ESP_LOGI(TAG, "Debugging I2S data flow");

    // Declare and initialize the I2S channel
    i2s_chan_handle_t tx_handle = NULL;
    i2s_chan_config_t chan_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_config, &tx_handle, NULL));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    // Prepare test data
    uint8_t test_data[] = {0x55, 0xAA, 0x55, 0xAA}; // Sample test pattern
    size_t bytes_written;

    // Write data to the I2S channel
    esp_err_t ret = i2s_channel_write(tx_handle, test_data, sizeof(test_data), &bytes_written, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S write failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "I2S write successful, bytes written: %zu", bytes_written);
    }

    // Disable and delete the I2S channel
    ESP_ERROR_CHECK(i2s_channel_disable(tx_handle));
    ESP_ERROR_CHECK(i2s_del_channel(tx_handle));
}