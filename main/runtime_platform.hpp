#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

namespace triwhirl::runtime {

// Create an ESP-IDF I2C master bus from the requested CPU core. ESP32 external
// peripheral interrupts are allocated on the core that performs allocation, so
// this API makes sensor interrupt affinity an explicit runtime decision.
esp_err_t createI2cMasterBusOnCore(
    const i2c_master_bus_config_t* config,
    i2c_master_bus_handle_t* output,
    BaseType_t target_core);

}  // namespace triwhirl::runtime
