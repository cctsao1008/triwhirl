#pragma once

#include <cstddef>
#include <cstdint>

namespace triwhirl {
namespace ble {

// Initializes the native ESP-IDF NimBLE peripheral transport.
// The transport is intentionally byte-stream based so the same newline-delimited
// command/telemetry protocol can be shared with UART.
bool init();

// Non-blocking byte-stream access. read() is polled by the control task; write()
// only queues data and never waits for BLE transmission.
std::size_t read(std::uint8_t* data, std::size_t capacity);
std::size_t write(const std::uint8_t* data, std::size_t length);

bool connected();
bool subscribed();
std::uint32_t rxDroppedBytes();
std::uint32_t txDroppedBytes();

}  // namespace ble
}  // namespace triwhirl
