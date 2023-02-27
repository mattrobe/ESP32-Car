#include "ble.hpp"

#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define LOG_TAG "Main"

extern "C" void
app_main(void)
{
    nvs_flash_init();
    auto *bt = Ble::getInstance();

    for (auto device : bt->scan(10))
    {
        ESP_LOGI(LOG_TAG, "Device Name: %s", device.getName().c_str());
        // esp_log_buffer_hex(LOG_TAG, device.bda, 6);
    }
}
