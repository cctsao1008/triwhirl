#include "triwhirl/runtime_settings.hpp"

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace triwhirl {
namespace settings {
namespace {

constexpr char kNamespace[] = "triwhirl";
constexpr char kMotorKey[] = "motor_v1";
constexpr char kImuMapKey[] = "imumap_v1";
constexpr std::uint32_t kSchemaVersion = 1U;

struct MotorBlob {
  std::uint32_t version = kSchemaVersion;
  std::int32_t pole_pairs = 0;
  std::int32_t sensor_direction = 1;
  float electrical_offset_rad = 0.0F;
};

struct ImuMapBlob {
  std::uint32_t version = kSchemaVersion;
  std::int32_t accel_sin_axis = 0;
  std::int32_t accel_cos_axis = 1;
  std::int32_t gyro_axis = 2;
  std::int32_t accel_sin_sign = 1;
  std::int32_t accel_cos_sign = 1;
  std::int32_t gyro_sign = 1;
};

bool openReadOnly(nvs_handle_t* handle) {
  return handle != nullptr &&
         nvs_open(kNamespace, NVS_READONLY, handle) == ESP_OK;
}

bool openReadWrite(nvs_handle_t* handle) {
  return handle != nullptr &&
         nvs_open(kNamespace, NVS_READWRITE, handle) == ESP_OK;
}

template <typename T>
bool loadBlob(const char* key, T* value) {
  if (key == nullptr || value == nullptr) {
    return false;
  }
  nvs_handle_t handle = 0;
  if (!openReadOnly(&handle)) {
    return false;
  }
  std::size_t length = sizeof(T);
  const esp_err_t result = nvs_get_blob(handle, key, value, &length);
  nvs_close(handle);
  return result == ESP_OK && length == sizeof(T);
}

template <typename T>
bool saveBlob(const char* key, const T& value) {
  if (key == nullptr) {
    return false;
  }
  nvs_handle_t handle = 0;
  if (!openReadWrite(&handle)) {
    return false;
  }
  const esp_err_t set_result = nvs_set_blob(handle, key, &value, sizeof(T));
  const esp_err_t commit_result =
      set_result == ESP_OK ? nvs_commit(handle) : set_result;
  nvs_close(handle);
  return set_result == ESP_OK && commit_result == ESP_OK;
}

bool eraseKey(const char* key) {
  nvs_handle_t handle = 0;
  if (key == nullptr || !openReadWrite(&handle)) {
    return false;
  }
  const esp_err_t erase_result = nvs_erase_key(handle, key);
  const bool absent = erase_result == ESP_ERR_NVS_NOT_FOUND;
  const esp_err_t commit_result =
      (erase_result == ESP_OK) ? nvs_commit(handle) : erase_result;
  nvs_close(handle);
  return absent || (erase_result == ESP_OK && commit_result == ESP_OK);
}

}  // namespace

bool init() {
  esp_err_t result = nvs_flash_init();
  if (result == ESP_ERR_NVS_NO_FREE_PAGES ||
      result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    if (nvs_flash_erase() != ESP_OK) {
      return false;
    }
    result = nvs_flash_init();
  }
  return result == ESP_OK;
}

bool loadMotor(MotorSettings* const settings) {
  if (settings == nullptr) {
    return false;
  }
  MotorBlob blob{};
  if (!loadBlob(kMotorKey, &blob) || blob.version != kSchemaVersion) {
    return false;
  }
  settings->pole_pairs = static_cast<int>(blob.pole_pairs);
  settings->sensor_direction = static_cast<int>(blob.sensor_direction);
  settings->electrical_offset_rad = blob.electrical_offset_rad;
  return true;
}

bool saveMotor(const MotorSettings& settings) {
  MotorBlob blob{};
  blob.pole_pairs = static_cast<std::int32_t>(settings.pole_pairs);
  blob.sensor_direction = static_cast<std::int32_t>(settings.sensor_direction);
  blob.electrical_offset_rad = settings.electrical_offset_rad;
  return saveBlob(kMotorKey, blob);
}

bool clearMotor() {
  return eraseKey(kMotorKey);
}

bool loadImuMap(ImuMapSettings* const settings) {
  if (settings == nullptr) {
    return false;
  }
  ImuMapBlob blob{};
  if (!loadBlob(kImuMapKey, &blob) || blob.version != kSchemaVersion) {
    return false;
  }
  settings->accel_sin_axis = static_cast<int>(blob.accel_sin_axis);
  settings->accel_cos_axis = static_cast<int>(blob.accel_cos_axis);
  settings->gyro_axis = static_cast<int>(blob.gyro_axis);
  settings->accel_sin_sign = static_cast<int>(blob.accel_sin_sign);
  settings->accel_cos_sign = static_cast<int>(blob.accel_cos_sign);
  settings->gyro_sign = static_cast<int>(blob.gyro_sign);
  return true;
}

bool saveImuMap(const ImuMapSettings& settings) {
  ImuMapBlob blob{};
  blob.accel_sin_axis = static_cast<std::int32_t>(settings.accel_sin_axis);
  blob.accel_cos_axis = static_cast<std::int32_t>(settings.accel_cos_axis);
  blob.gyro_axis = static_cast<std::int32_t>(settings.gyro_axis);
  blob.accel_sin_sign = static_cast<std::int32_t>(settings.accel_sin_sign);
  blob.accel_cos_sign = static_cast<std::int32_t>(settings.accel_cos_sign);
  blob.gyro_sign = static_cast<std::int32_t>(settings.gyro_sign);
  return saveBlob(kImuMapKey, blob);
}

bool clearImuMap() {
  return eraseKey(kImuMapKey);
}

}  // namespace settings
}  // namespace triwhirl
