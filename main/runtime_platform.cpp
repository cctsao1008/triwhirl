#include "runtime_platform.hpp"

#include "freertos/semphr.h"
#include "freertos/task.h"

namespace triwhirl::runtime {
namespace {

struct I2cBusCreateContext {
  const i2c_master_bus_config_t* config = nullptr;
  i2c_master_bus_handle_t* output = nullptr;
  SemaphoreHandle_t done = nullptr;
  esp_err_t result = ESP_FAIL;
};

void createI2cBusPinnedTask(void* opaque) {
  auto* context = static_cast<I2cBusCreateContext*>(opaque);
  if (context != nullptr && context->config != nullptr &&
      context->output != nullptr) {
    context->result = i2c_new_master_bus(context->config, context->output);
  }
  if (context != nullptr && context->done != nullptr) {
    xSemaphoreGive(context->done);
  }
  vTaskDelete(nullptr);
}

}  // namespace

esp_err_t createI2cMasterBusOnCore(
    const i2c_master_bus_config_t* config,
    i2c_master_bus_handle_t* output,
    const BaseType_t target_core) {
  if (config == nullptr || output == nullptr ||
      (target_core != 0 && target_core != 1)) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xPortGetCoreID() == target_core) {
    return i2c_new_master_bus(config, output);
  }

  I2cBusCreateContext context{};
  context.config = config;
  context.output = output;
  context.done = xSemaphoreCreateBinary();
  if (context.done == nullptr) {
    return ESP_ERR_NO_MEM;
  }

  const BaseType_t created = xTaskCreatePinnedToCore(
      createI2cBusPinnedTask, "triwhirl_i2c_init", 4096, &context,
      configMAX_PRIORITIES - 1, nullptr, target_core);
  if (created != pdPASS) {
    vSemaphoreDelete(context.done);
    return ESP_FAIL;
  }

  xSemaphoreTake(context.done, portMAX_DELAY);
  vSemaphoreDelete(context.done);
  return context.result;
}

}  // namespace triwhirl::runtime
