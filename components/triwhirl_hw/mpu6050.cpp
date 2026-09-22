#include "triwhirl/drivers/mpu6050.hpp"

#include <algorithm>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace triwhirl {
namespace drivers {
namespace {

constexpr char kTag[] = "triwhirl_mpu";

constexpr std::uint8_t kRegSampleRateDivider = 0x19U;
constexpr std::uint8_t kRegConfig = 0x1AU;
constexpr std::uint8_t kRegGyroConfig = 0x1BU;
constexpr std::uint8_t kRegAccelConfig = 0x1CU;
constexpr std::uint8_t kRegFifoEnable = 0x23U;
constexpr std::uint8_t kRegIntPinConfig = 0x37U;
constexpr std::uint8_t kRegIntEnable = 0x38U;
constexpr std::uint8_t kRegUserControl = 0x6AU;
constexpr std::uint8_t kRegPowerManagement1 = 0x6BU;
constexpr std::uint8_t kRegFifoCountHigh = 0x72U;
constexpr std::uint8_t kRegFifoReadWrite = 0x74U;
constexpr std::uint8_t kRegWhoAmI = 0x75U;

constexpr std::uint8_t kExpectedWhoAmI = 0x68U;
constexpr int kI2cTimeoutMs = 20;
constexpr std::uint32_t kPowerOnSettleMs = 100U;
constexpr std::uint32_t kWakeSettleMs = 30U;
constexpr std::uint32_t kRetrySettleMs = 30U;
constexpr std::uint32_t kFifoPrimeMs = 3U;
constexpr unsigned kInitAttempts = 3U;
constexpr float kGravityMps2 = 9.80665F;
constexpr float kAccelLsbPerG = 8192.0F;  // +/-4 g
constexpr float kGyroLsbPerDps = 32.8F;  // +/-1000 deg/s
constexpr float kDegToRad = 0.01745329251994329577F;

// FIFO_EN register bits: X/Y/Z gyro + accel, temperature deliberately omitted.
constexpr std::uint8_t kRuntimeFifoSources = 0x78U;
constexpr std::uint8_t kUserControlFifoEnable = 1U << 6;
constexpr std::uint8_t kUserControlFifoReset = 1U << 2;
constexpr std::uint8_t kIntEnableFifoOverflow = 1U << 4;
constexpr std::uint8_t kIntEnableDataReady = 1U << 0;
constexpr TickType_t kAsyncTransferWaitTicks = pdMS_TO_TICKS(2);

std::int16_t readBigEndianI16(const std::uint8_t high,
                              const std::uint8_t low) {
  const std::uint16_t raw =
      (static_cast<std::uint16_t>(high) << 8U) |
      static_cast<std::uint16_t>(low);
  return static_cast<std::int16_t>(raw);
}

void decodePhysicalSample(Mpu6050Sample* const sample) {
  const float accel_scale = kGravityMps2 / kAccelLsbPerG;
  const float gyro_scale = kDegToRad / kGyroLsbPerDps;
  for (int axis = 0; axis < 3; ++axis) {
    sample->accel_mps2[axis] =
        static_cast<float>(sample->accel_raw[axis]) * accel_scale;
    sample->gyro_rad_s[axis] =
        static_cast<float>(sample->gyro_raw[axis]) * gyro_scale;
  }
}

}  // namespace

bool Mpu6050::init(const i2c_master_bus_handle_t bus,
                   const std::uint8_t address) {
  who_am_i_ = 0U;
  who_am_i_valid_ = false;
  last_init_who_am_i_ = 0U;
  last_init_who_am_i_valid_ = false;
  fifo_enabled_ = false;
  async_i2c_enabled_ = false;
  async_in_flight_.store(false, std::memory_order_relaxed);
  async_event_.store(0U, std::memory_order_relaxed);

  if (bus == nullptr) {
    ESP_LOGW(kTag, "init stage=bus result=null");
    return false;
  }

  if (async_done_ == nullptr) {
    async_done_ = xSemaphoreCreateBinary();
    if (async_done_ == nullptr) {
      ESP_LOGW(kTag, "init stage=semaphore result=no_mem");
      return false;
    }
  }

  ESP_LOGI(kTag, "init begin preferred_addr=0x%02x", static_cast<unsigned>(address));

  // The MPU-60X0 may not accept register traffic immediately after power-on.
  vTaskDelay(pdMS_TO_TICKS(kPowerOnSettleMs));

  const std::uint8_t alternate = address == 0x68U ? 0x69U : 0x68U;
  const std::uint8_t candidates[2] = {address, alternate};

  for (unsigned attempt = 0U; attempt < kInitAttempts; ++attempt) {
    if (attempt > 0U) {
      if (device_ != nullptr) {
        i2c_master_bus_rm_device(device_);
        device_ = nullptr;
      }
      who_am_i_ = 0U;
      who_am_i_valid_ = false;
      fifo_enabled_ = false;
      async_i2c_enabled_ = false;
      const esp_err_t reset_result = i2c_master_bus_reset(bus);
      ESP_LOGI(kTag, "init attempt=%u stage=bus_reset result=%s",
               attempt + 1U, esp_err_to_name(reset_result));
      vTaskDelay(pdMS_TO_TICKS(kRetrySettleMs));
    }

    for (const std::uint8_t candidate : candidates) {
      who_am_i_ = 0U;
      who_am_i_valid_ = false;

      const esp_err_t probe_result =
          i2c_master_probe(bus, candidate, kI2cTimeoutMs);
      ESP_LOGI(kTag, "init attempt=%u stage=probe addr=0x%02x result=%s",
               attempt + 1U, static_cast<unsigned>(candidate),
               esp_err_to_name(probe_result));
      if (probe_result != ESP_OK) {
        continue;
      }

      i2c_device_config_t config{};
      config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
      config.device_address = candidate;
      config.scl_speed_hz = 1000000U;
      const esp_err_t add_result =
          i2c_master_bus_add_device(bus, &config, &device_);
      ESP_LOGI(kTag, "init attempt=%u stage=add_device addr=0x%02x result=%s",
               attempt + 1U, static_cast<unsigned>(candidate),
               esp_err_to_name(add_result));
      if (add_result != ESP_OK) {
        device_ = nullptr;
        continue;
      }

      auto discard_device = [this]() {
        if (device_ != nullptr) {
          i2c_master_bus_rm_device(device_);
          device_ = nullptr;
        }
        who_am_i_ = 0U;
        who_am_i_valid_ = false;
        fifo_enabled_ = false;
        async_i2c_enabled_ = false;
      };

      std::uint8_t who_am_i = 0U;
      const bool who_ok = readWhoAmI(&who_am_i);
      if (who_ok) {
        last_init_who_am_i_ = who_am_i;
        last_init_who_am_i_valid_ = true;
      }
      ESP_LOGI(kTag,
               "init attempt=%u stage=who addr=0x%02x read_ok=%d value=0x%02x match=%d",
               attempt + 1U, static_cast<unsigned>(candidate), who_ok ? 1 : 0,
               static_cast<unsigned>(who_am_i),
               who_ok && who_am_i == kExpectedWhoAmI ? 1 : 0);
      if (!who_ok || who_am_i != kExpectedWhoAmI) {
        discard_device();
        continue;
      }

      // Wake the device and use the X-axis gyro PLL as the clock source.
      if (!writeRegister(kRegPowerManagement1, 0x01U)) {
        ESP_LOGW(kTag, "init attempt=%u stage=wake failed", attempt + 1U);
        discard_device();
        continue;
      }
      ESP_LOGI(kTag, "init attempt=%u stage=wake ok", attempt + 1U);
      vTaskDelay(pdMS_TO_TICKS(kWakeSettleMs));

      // Low-latency control profile: with DLPF_CFG=0 the gyro output rate is
      // 8 kHz, so SMPLRT_DIV=7 preserves a 1-kHz FIFO/control sample rate while
      // using the MPU6050's lowest-delay 256-Hz gyro / 260-Hz accel bandwidth.
      if (!writeRegister(kRegSampleRateDivider, 0x07U)) {
        ESP_LOGW(kTag, "init attempt=%u stage=sample_rate failed", attempt + 1U);
        discard_device();
        continue;
      }
      if (!writeRegister(kRegConfig, 0x00U)) {
        ESP_LOGW(kTag, "init attempt=%u stage=dlpf failed", attempt + 1U);
        discard_device();
        continue;
      }
      if (!writeRegister(kRegGyroConfig, 0x10U)) {
        ESP_LOGW(kTag, "init attempt=%u stage=gyro_config failed", attempt + 1U);
        discard_device();
        continue;
      }
      if (!writeRegister(kRegAccelConfig, 0x08U)) {
        ESP_LOGW(kTag, "init attempt=%u stage=accel_config failed", attempt + 1U);
        discard_device();
        continue;
      }
      ESP_LOGI(kTag, "init attempt=%u stage=base_config ok", attempt + 1U);

      if (!configureRuntimeFifo()) {
        ESP_LOGW(kTag, "init attempt=%u stage=fifo_config failed", attempt + 1U);
        discard_device();
        continue;
      }
      ESP_LOGI(kTag, "init attempt=%u stage=fifo_config ok", attempt + 1U);

      // Keep the validated physical-board baseline synchronous. DATA_RDY IRQ,
      // not the ESP-IDF I2C completion callback, is the hardware acquisition
      // trigger. Core-0 owns the blocking FIFO transfers, so Core 1 remains free
      // of MPU I2C while we avoid re-entering the async-bus probe regression.
      async_i2c_enabled_ = false;
      vTaskDelay(pdMS_TO_TICKS(kFifoPrimeMs));
      ESP_LOGI(kTag, "init ready addr=0x%02x fifo=1 async=0 mode=sync_fifo",
               static_cast<unsigned>(candidate));
      return true;
    }
  }

  ESP_LOGW(kTag, "init failed after %u attempts last_who_valid=%d last_who=0x%02x",
           kInitAttempts, last_init_who_am_i_valid_ ? 1 : 0,
           static_cast<unsigned>(last_init_who_am_i_));
  return false;
}

bool Mpu6050::writeRegister(const std::uint8_t reg,
                            const std::uint8_t value) {
  if (device_ == nullptr || async_i2c_enabled_) {
    return false;
  }
  const std::uint8_t data[2] = {reg, value};
  const esp_err_t result =
      i2c_master_transmit(device_, data, sizeof(data), kI2cTimeoutMs);
  if (result != ESP_OK) {
    ESP_LOGW(kTag, "write reg=0x%02x value=0x%02x result=%s",
             static_cast<unsigned>(reg), static_cast<unsigned>(value),
             esp_err_to_name(result));
  }
  return result == ESP_OK;
}

bool Mpu6050::readRegisters(const std::uint8_t first_register,
                            std::uint8_t* const data,
                            const std::size_t length) {
  if (device_ == nullptr || data == nullptr || length == 0U ||
      async_i2c_enabled_) {
    return false;
  }
  return i2c_master_transmit_receive(device_, &first_register, 1U, data, length,
                                     kI2cTimeoutMs) == ESP_OK;
}

bool Mpu6050::readWhoAmI(std::uint8_t* const who_am_i) {
  if (who_am_i == nullptr) {
    return false;
  }
  if (who_am_i_valid_) {
    *who_am_i = who_am_i_;
    return true;
  }

  std::uint8_t value = 0U;
  if (!readRegisters(kRegWhoAmI, &value, 1U)) {
    return false;
  }
  who_am_i_ = value;
  who_am_i_valid_ = true;
  *who_am_i = value;
  return true;
}

bool Mpu6050::lastInitWhoAmI(std::uint8_t* const who_am_i) const {
  if (who_am_i == nullptr || !last_init_who_am_i_valid_) {
    return false;
  }
  *who_am_i = last_init_who_am_i_;
  return true;
}

bool Mpu6050::configureRuntimeFifo() {
  // Stop FIFO writes, reset the FIFO, then enable accel + all gyro axes. The
  // temperature channel is intentionally excluded: Balance does not consume it,
  // reducing each 1 kHz packet from 14 bytes to 12 bytes without coupling FIFO
  // layout to the runtime-selectable planar gyro axis.
  if (!writeRegister(kRegFifoEnable, 0x00U)) {
    ESP_LOGW(kTag, "fifo stage=disable_sources failed");
    return false;
  }
  if (!writeRegister(kRegUserControl, kUserControlFifoReset)) {
    ESP_LOGW(kTag, "fifo stage=reset failed");
    return false;
  }
  vTaskDelay(pdMS_TO_TICKS(1));
  if (!writeRegister(kRegFifoEnable, kRuntimeFifoSources)) {
    ESP_LOGW(kTag, "fifo stage=enable_sources failed");
    return false;
  }
  if (!writeRegister(kRegUserControl, kUserControlFifoEnable)) {
    ESP_LOGW(kTag, "fifo stage=enable_user_control failed");
    return false;
  }
  // Active-high, push-pull, 50 us pulse (register reset/default behavior).
  if (!writeRegister(kRegIntPinConfig, 0x00U)) {
    ESP_LOGW(kTag, "fifo stage=int_pin_config failed");
    return false;
  }
  if (!writeRegister(kRegIntEnable,
                     kIntEnableFifoOverflow | kIntEnableDataReady)) {
    ESP_LOGW(kTag, "fifo stage=int_enable failed");
    return false;
  }
  fifo_enabled_ = true;
  return true;
}

bool Mpu6050::asyncTransactionDone(
    i2c_master_dev_handle_t,
    const i2c_master_event_data_t* const event,
    void* const context) {
  auto* self = static_cast<Mpu6050*>(context);
  if (self == nullptr) {
    return false;
  }
  self->async_event_.store(
      static_cast<std::uint8_t>(event != nullptr ? event->event : I2C_EVENT_TIMEOUT),
      std::memory_order_release);
  self->async_in_flight_.store(false, std::memory_order_release);

  BaseType_t task_woken = pdFALSE;
  if (self->async_done_ != nullptr) {
    xSemaphoreGiveFromISR(self->async_done_, &task_woken);
  }
  return task_woken == pdTRUE;
}

bool Mpu6050::asyncRead(const std::uint8_t first_register,
                        std::uint8_t* const data,
                        const std::size_t length,
                        const TickType_t timeout_ticks) {
  if (!async_i2c_enabled_ || device_ == nullptr || async_done_ == nullptr ||
      data == nullptr || length == 0U) {
    return false;
  }

  // A timed-out transfer may still be completing in hardware. Never reuse the
  // persistent transaction buffers until its callback has cleared in-flight.
  if (async_in_flight_.load(std::memory_order_acquire)) {
    return false;
  }
  while (xSemaphoreTake(async_done_, 0) == pdTRUE) {
  }

  async_register_ = first_register;
  async_event_.store(static_cast<std::uint8_t>(I2C_EVENT_ALIVE),
                     std::memory_order_relaxed);
  async_in_flight_.store(true, std::memory_order_release);

  const esp_err_t result = i2c_master_transmit_receive(
      device_, &async_register_, 1U, data, length, 0);
  if (result != ESP_OK) {
    async_in_flight_.store(false, std::memory_order_release);
    return false;
  }

  if (xSemaphoreTake(async_done_, timeout_ticks) != pdTRUE) {
    return false;
  }
  return async_event_.load(std::memory_order_acquire) ==
         static_cast<std::uint8_t>(I2C_EVENT_DONE);
}

bool Mpu6050::decodeFifoSample(const std::uint8_t* const data,
                               const std::size_t length,
                               Mpu6050Sample* const sample) const {
  if (data == nullptr || sample == nullptr || length < kFifoSampleBytes) {
    return false;
  }
  const std::size_t offset = length - kFifoSampleBytes;
  const std::uint8_t* const frame = data + offset;

  sample->accel_raw[0] = readBigEndianI16(frame[0], frame[1]);
  sample->accel_raw[1] = readBigEndianI16(frame[2], frame[3]);
  sample->accel_raw[2] = readBigEndianI16(frame[4], frame[5]);
  sample->gyro_raw[0] = readBigEndianI16(frame[6], frame[7]);
  sample->gyro_raw[1] = readBigEndianI16(frame[8], frame[9]);
  sample->gyro_raw[2] = readBigEndianI16(frame[10], frame[11]);
  sample->temperature_raw = 0;
  sample->temperature_c = 0.0F;
  decodePhysicalSample(sample);
  return true;
}

bool Mpu6050::readSample(Mpu6050Sample* const sample) {
  if (sample == nullptr || device_ == nullptr || !fifo_enabled_) {
    return false;
  }

  const bool profile = timing_profile_enabled_.load(std::memory_order_relaxed);
  const std::int64_t transfer_begin_us = profile ? esp_timer_get_time() : 0;

  const bool count_ok = async_i2c_enabled_
                            ? asyncRead(kRegFifoCountHigh, fifo_count_data_,
                                        sizeof(fifo_count_data_),
                                        kAsyncTransferWaitTicks)
                            : readRegisters(kRegFifoCountHigh, fifo_count_data_,
                                            sizeof(fifo_count_data_));
  if (!count_ok) {
    return false;
  }

  const std::uint16_t fifo_count = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(fifo_count_data_[0]) << 8U) |
      fifo_count_data_[1]);
  const std::size_t aligned_bytes =
      static_cast<std::size_t>(fifo_count) -
      (static_cast<std::size_t>(fifo_count) % kFifoSampleBytes);
  if (aligned_bytes < kFifoSampleBytes) {
    return false;
  }

  // Read multiple queued packets when necessary to catch up, then decode the
  // newest packet in this bounded chunk. At steady state this is one 12-byte
  // packet. A short scheduler hiccup therefore self-recovers instead of locking
  // control onto an increasingly old FIFO sample.
  const std::size_t read_bytes =
      std::min(aligned_bytes, kFifoReadBufferBytes);
  const bool fifo_ok = async_i2c_enabled_
                           ? asyncRead(kRegFifoReadWrite, fifo_data_, read_bytes,
                                       kAsyncTransferWaitTicks)
                           : readRegisters(kRegFifoReadWrite, fifo_data_,
                                           read_bytes);
  if (!fifo_ok) {
    return false;
  }

  const std::int64_t transfer_end_us = profile ? esp_timer_get_time() : 0;
  const bool decoded = decodeFifoSample(fifo_data_, read_bytes, sample);
  if (profile && decoded) {
    const std::int64_t decode_end_us = esp_timer_get_time();
    recordSampleTiming(
        static_cast<std::uint32_t>(transfer_end_us - transfer_begin_us),
        static_cast<std::uint32_t>(decode_end_us - transfer_end_us));
  }
  return decoded;
}

void Mpu6050::setTimingProfileEnabled(const bool enabled) {
  timing_profile_enabled_.store(enabled, std::memory_order_relaxed);
}

void Mpu6050::resetTimingProfile() {
  portENTER_CRITICAL(&timing_mux_);
  timing_stats_ = {};
  portEXIT_CRITICAL(&timing_mux_);
}

Mpu6050TimingStats Mpu6050::timingProfile() const {
  portENTER_CRITICAL(&timing_mux_);
  const Mpu6050TimingStats snapshot = timing_stats_;
  portEXIT_CRITICAL(&timing_mux_);
  return snapshot;
}

void Mpu6050::recordSampleTiming(const std::uint32_t transfer_us,
                                 const std::uint32_t decode_us) {
  portENTER_CRITICAL(&timing_mux_);
  ++timing_stats_.sample_reads;
  timing_stats_.transfer_total_us += transfer_us;
  timing_stats_.decode_total_us += decode_us;
  if (timing_stats_.sample_reads == 1U) {
    timing_stats_.transfer_min_us = transfer_us;
    timing_stats_.transfer_max_us = transfer_us;
    timing_stats_.decode_min_us = decode_us;
    timing_stats_.decode_max_us = decode_us;
    portEXIT_CRITICAL(&timing_mux_);
    return;
  }
  timing_stats_.transfer_min_us =
      std::min(timing_stats_.transfer_min_us, transfer_us);
  timing_stats_.transfer_max_us =
      std::max(timing_stats_.transfer_max_us, transfer_us);
  timing_stats_.decode_min_us =
      std::min(timing_stats_.decode_min_us, decode_us);
  timing_stats_.decode_max_us =
      std::max(timing_stats_.decode_max_us, decode_us);
  portEXIT_CRITICAL(&timing_mux_);
}

}  // namespace drivers
}  // namespace triwhirl