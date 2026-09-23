#pragma once

#include <cstddef>
#include <cstdint>

namespace triwhirl {
namespace ble {

// Initializes the native ESP-IDF NimBLE peripheral transport.
// Console traffic and realtime trace traffic use separate notify
// characteristics so binary trace streaming can never corrupt line parsing.
bool init();

// Non-blocking byte-stream access for the console characteristic.
std::size_t read(std::uint8_t* data, std::size_t capacity);
std::size_t write(const std::uint8_t* data, std::size_t length);

// Bulk-transfer helper for non-realtime tasks such as post-run binary log dump.
// Never call this from the 1 kHz control task.
std::size_t writeBlocking(const std::uint8_t* data,
                          std::size_t length,
                          std::uint32_t timeout_ms);

// Dedicated binary trace stream. The producer must remain outside the realtime
// task; the trace transport owns its own Core-0 TX worker and queue.
std::size_t traceWriteBlocking(const std::uint8_t* data,
                               std::size_t length,
                               std::uint32_t timeout_ms);

bool connected();
bool subscribed();
bool traceSubscribed();
std::uint32_t rxDroppedBytes();
std::uint32_t txDroppedBytes();
std::uint32_t traceTxDroppedBytes();

}  // namespace ble
}  // namespace triwhirl
