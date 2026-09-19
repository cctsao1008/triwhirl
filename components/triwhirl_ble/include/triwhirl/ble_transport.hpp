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

// Bulk-transfer helper for non-realtime tasks such as post-run binary log dump.
// It applies backpressure to the caller until the TX stream has room or the
// timeout expires. Never call this from the 1 kHz control task.
std::size_t writeBlocking(const std::uint8_t* data,
                          std::size_t length,
                          std::uint32_t timeout_ms);

bool connected();
bool subscribed();
std::uint32_t rxDroppedBytes();
std::uint32_t txDroppedBytes();

}  // namespace ble
}  // namespace triwhirl
