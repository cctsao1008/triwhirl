#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

namespace triwhirl::runtime {

// ESP-IDF allocates a peripheral interrupt on the core that creates the I2C
// master bus. Create the bus from the core that owns the corresponding sensor
// service so independent controllers do not share one ISR service core by
// accident.
esp_err_t newI2cMasterBusOnCore(const i2c_master_bus_config_t* config,
                                i2c_master_bus_handle_t* output,
                                BaseType_t core_id);

}  // namespace triwhirl::runtime
