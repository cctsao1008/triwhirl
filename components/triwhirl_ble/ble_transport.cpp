#include "triwhirl/ble_transport.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "sdkconfig.h"

#if CONFIG_BT_NIMBLE_ENABLED

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace triwhirl {
namespace ble {
namespace {

constexpr char kTag[] = "triwhirl_ble";
constexpr char kDeviceName[] = "TriWhirl";
constexpr std::size_t kRxBufferBytes = 1024U;
constexpr std::size_t kTxBufferBytes = 8192U;
constexpr std::size_t kTraceTxBufferBytes = 8192U;
constexpr std::size_t kTxScratchBytes = 512U;
constexpr std::size_t kTraceMaxNotifyBytes = 240U;
constexpr std::uint16_t kPreferredMtu = 256U;

// UUIDs are expressed in the little-endian byte order expected by
// BLE_UUID128_INIT.
static const ble_uuid128_t kServiceUuid = BLE_UUID128_INIT(
    0x4c, 0x48, 0x57, 0x49, 0x52, 0x54, 0x91, 0xb6,
    0x3a, 0x4f, 0x4d, 0x8f, 0x00, 0x00, 0xf1, 0x54);
static const ble_uuid128_t kRxUuid = BLE_UUID128_INIT(
    0x4c, 0x48, 0x57, 0x49, 0x52, 0x54, 0x91, 0xb6,
    0x3a, 0x4f, 0x4d, 0x8f, 0x01, 0x00, 0xf1, 0x54);
static const ble_uuid128_t kTxUuid = BLE_UUID128_INIT(
    0x4c, 0x48, 0x57, 0x49, 0x52, 0x54, 0x91, 0xb6,
    0x3a, 0x4f, 0x4d, 0x8f, 0x02, 0x00, 0xf1, 0x54);
static const ble_uuid128_t kTraceUuid = BLE_UUID128_INIT(
    0x4c, 0x48, 0x57, 0x49, 0x52, 0x54, 0x91, 0xb6,
    0x3a, 0x4f, 0x4d, 0x8f, 0x03, 0x00, 0xf1, 0x54);

StreamBufferHandle_t rx_stream = nullptr;
StreamBufferHandle_t tx_stream = nullptr;
StreamBufferHandle_t trace_tx_stream = nullptr;
volatile std::uint16_t connection_handle = BLE_HS_CONN_HANDLE_NONE;
volatile bool tx_subscribed = false;
volatile bool trace_tx_subscribed = false;
volatile bool initialized = false;
std::uint16_t tx_value_handle = 0U;
std::uint16_t trace_value_handle = 0U;
std::uint8_t own_addr_type = 0U;
std::uint32_t rx_dropped_bytes = 0U;
std::uint32_t tx_dropped_bytes = 0U;
std::uint32_t trace_tx_dropped_bytes = 0U;

ble_gatt_chr_def characteristics[4]{};
ble_gatt_svc_def services[2]{};

int startAdvertising();

int characteristicAccess(std::uint16_t,
                         std::uint16_t,
                         ble_gatt_access_ctxt* ctxt,
                         void*) {
  if (ctxt == nullptr || ctxt->chr == nullptr) {
    return BLE_ATT_ERR_UNLIKELY;
  }

  if (ble_uuid_cmp(ctxt->chr->uuid, &kRxUuid.u) != 0 ||
      ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
  }

  const std::uint16_t total = OS_MBUF_PKTLEN(ctxt->om);
  if (total == 0U) {
    return 0;
  }
  if (total > 512U || rx_stream == nullptr) {
    rx_dropped_bytes += total;
    return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  }
  if (xStreamBufferSpacesAvailable(rx_stream) < total) {
    rx_dropped_bytes += total;
    return BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  std::uint8_t buffer[512];
  std::uint16_t copied = 0U;
  if (ble_hs_mbuf_to_flat(ctxt->om, buffer, total, &copied) != 0) {
    rx_dropped_bytes += total;
    return BLE_ATT_ERR_UNLIKELY;
  }

  const std::size_t queued = xStreamBufferSend(rx_stream, buffer, copied, 0);
  if (queued != copied) {
    rx_dropped_bytes += static_cast<std::uint32_t>(copied - queued);
    return BLE_ATT_ERR_INSUFFICIENT_RES;
  }
  return 0;
}

void buildGattTable() {
  characteristics[0] = {};
  characteristics[0].uuid = &kRxUuid.u;
  characteristics[0].access_cb = characteristicAccess;
  characteristics[0].flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP;

  characteristics[1] = {};
  characteristics[1].uuid = &kTxUuid.u;
  characteristics[1].access_cb = characteristicAccess;
  characteristics[1].flags = BLE_GATT_CHR_F_NOTIFY;
  characteristics[1].val_handle = &tx_value_handle;

  characteristics[2] = {};
  characteristics[2].uuid = &kTraceUuid.u;
  characteristics[2].access_cb = characteristicAccess;
  characteristics[2].flags = BLE_GATT_CHR_F_NOTIFY;
  characteristics[2].val_handle = &trace_value_handle;

  characteristics[3] = {};

  services[0] = {};
  services[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
  services[0].uuid = &kServiceUuid.u;
  services[0].characteristics = characteristics;
  services[1] = {};
}

int gapEvent(ble_gap_event* event, void*) {
  if (event == nullptr) {
    return 0;
  }

  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0) {
        connection_handle = event->connect.conn_handle;
        tx_subscribed = false;
        trace_tx_subscribed = false;
      } else {
        startAdvertising();
      }
      return 0;

    case BLE_GAP_EVENT_DISCONNECT:
      connection_handle = BLE_HS_CONN_HANDLE_NONE;
      tx_subscribed = false;
      trace_tx_subscribed = false;
      startAdvertising();
      return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
      if (event->subscribe.attr_handle == tx_value_handle) {
        tx_subscribed = event->subscribe.cur_notify != 0;
      } else if (event->subscribe.attr_handle == trace_value_handle) {
        trace_tx_subscribed = event->subscribe.cur_notify != 0;
      }
      return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
      if (connection_handle == BLE_HS_CONN_HANDLE_NONE) {
        startAdvertising();
      }
      return 0;

    default:
      return 0;
  }
}

int startAdvertising() {
  ble_hs_adv_fields primary{};
  primary.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  primary.name = reinterpret_cast<std::uint8_t*>(const_cast<char*>(kDeviceName));
  primary.name_len = std::strlen(kDeviceName);
  primary.name_is_complete = 1;
  int rc = ble_gap_adv_set_fields(&primary);
  if (rc != 0) {
    return rc;
  }

  ble_hs_adv_fields response{};
  response.uuids128 = const_cast<ble_uuid128_t*>(&kServiceUuid);
  response.num_uuids128 = 1;
  response.uuids128_is_complete = 1;
  rc = ble_gap_adv_rsp_set_fields(&response);
  if (rc != 0) {
    return rc;
  }

  ble_gap_adv_params params{};
  params.conn_mode = BLE_GAP_CONN_MODE_UND;
  params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  return ble_gap_adv_start(own_addr_type, nullptr, BLE_HS_FOREVER,
                           &params, gapEvent, nullptr);
}

void onReset(int reason) {
  ESP_LOGE(kTag, "NimBLE reset reason=%d", reason);
  connection_handle = BLE_HS_CONN_HANDLE_NONE;
  tx_subscribed = false;
  trace_tx_subscribed = false;
}

void onSync() {
  if (ble_hs_util_ensure_addr(0) != 0 ||
      ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
    ESP_LOGE(kTag, "BLE identity address unavailable");
    return;
  }
  const int rc = startAdvertising();
  if (rc != 0) {
    ESP_LOGE(kTag, "BLE advertising failed rc=%d", rc);
  }
}

void hostTask(void*) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}

bool notifyChunk(const std::uint16_t value_handle,
                 const std::uint8_t* data,
                 const std::size_t length) {
  const std::uint16_t conn = connection_handle;
  if (conn == BLE_HS_CONN_HANDLE_NONE || data == nullptr || length == 0U) {
    return false;
  }
  os_mbuf* om = ble_hs_mbuf_from_flat(data, length);
  if (om == nullptr) {
    return false;
  }
  return ble_gatts_notify_custom(conn, value_handle, om) == 0;
}

void txTask(void*) {
  std::uint8_t buffer[kTxScratchBytes];
  while (true) {
    const std::size_t received = xStreamBufferReceive(
        tx_stream, buffer, sizeof(buffer), portMAX_DELAY);
    if (received == 0U) {
      continue;
    }

    const std::uint16_t conn = connection_handle;
    if (conn == BLE_HS_CONN_HANDLE_NONE || !tx_subscribed) {
      tx_dropped_bytes += static_cast<std::uint32_t>(received);
      continue;
    }

    const std::uint16_t mtu = ble_att_mtu(conn);
    const std::size_t chunk = mtu > 3U ? static_cast<std::size_t>(mtu - 3U) : 20U;
    std::size_t offset = 0U;
    while (offset < received) {
      const std::size_t count = std::min(chunk, received - offset);
      while (!notifyChunk(tx_value_handle, buffer + offset, count)) {
        if (connection_handle == BLE_HS_CONN_HANDLE_NONE || !tx_subscribed) {
          tx_dropped_bytes += static_cast<std::uint32_t>(received - offset);
          offset = received;
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
      }
      if (offset < received) {
        offset += count;
      }
    }
  }
}

void traceTxTask(void*) {
  std::uint8_t buffer[kTxScratchBytes];
  while (true) {
    const std::size_t received = xStreamBufferReceive(
        trace_tx_stream, buffer, sizeof(buffer), portMAX_DELAY);
    if (received == 0U) {
      continue;
    }

    const std::uint16_t conn = connection_handle;
    if (conn == BLE_HS_CONN_HANDLE_NONE || !trace_tx_subscribed) {
      trace_tx_dropped_bytes += static_cast<std::uint32_t>(received);
      continue;
    }

    const std::uint16_t mtu = ble_att_mtu(conn);
    const std::size_t att_payload =
        mtu > 3U ? static_cast<std::size_t>(mtu - 3U) : 20U;
    const std::size_t chunk = std::min(att_payload, kTraceMaxNotifyBytes);
    std::size_t offset = 0U;
    while (offset < received) {
      const std::size_t count = std::min(chunk, received - offset);
      while (!notifyChunk(trace_value_handle, buffer + offset, count)) {
        if (connection_handle == BLE_HS_CONN_HANDLE_NONE ||
            !trace_tx_subscribed) {
          trace_tx_dropped_bytes +=
              static_cast<std::uint32_t>(received - offset);
          offset = received;
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
      }
      if (offset < received) {
        offset += count;
      }
    }
  }
}

bool initNvs() {
  esp_err_t result = nvs_flash_init();
  if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    if (nvs_flash_erase() != ESP_OK) {
      return false;
    }
    result = nvs_flash_init();
  }
  return result == ESP_OK;
}

std::size_t writeBlockingTo(StreamBufferHandle_t stream,
                            volatile std::uint32_t* dropped,
                            const std::uint8_t* data,
                            const std::size_t length,
                            const std::uint32_t timeout_ms) {
  if (data == nullptr || length == 0U || stream == nullptr ||
      connection_handle == BLE_HS_CONN_HANDLE_NONE) {
    return 0U;
  }

  const TickType_t start = xTaskGetTickCount();
  const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
  std::size_t total = 0U;
  while (total < length) {
    if (connection_handle == BLE_HS_CONN_HANDLE_NONE) {
      break;
    }
    const TickType_t elapsed = xTaskGetTickCount() - start;
    if (elapsed >= timeout) {
      break;
    }
    const TickType_t remaining = timeout - elapsed;
    const std::size_t sent = xStreamBufferSend(
        stream, data + total, length - total, remaining);
    if (sent == 0U) {
      break;
    }
    total += sent;
  }
  if (total < length && dropped != nullptr) {
    *dropped += static_cast<std::uint32_t>(length - total);
  }
  return total;
}

}  // namespace

bool init() {
  if (initialized) {
    return true;
  }
  if (!initNvs()) {
    return false;
  }

  rx_stream = xStreamBufferCreate(kRxBufferBytes, 1U);
  tx_stream = xStreamBufferCreate(kTxBufferBytes, 1U);
  trace_tx_stream = xStreamBufferCreate(kTraceTxBufferBytes, 1U);
  if (rx_stream == nullptr || tx_stream == nullptr || trace_tx_stream == nullptr) {
    return false;
  }

  if (nimble_port_init() != ESP_OK) {
    return false;
  }

  ble_hs_cfg.reset_cb = onReset;
  ble_hs_cfg.sync_cb = onSync;

#if CONFIG_BT_NIMBLE_GAP_SERVICE
  ble_svc_gap_init();
  if (ble_svc_gap_device_name_set(kDeviceName) != 0) {
    return false;
  }
#endif
  ble_svc_gatt_init();

  buildGattTable();
  if (ble_gatts_count_cfg(services) != 0 || ble_gatts_add_svcs(services) != 0) {
    return false;
  }

  if (ble_att_set_preferred_mtu(kPreferredMtu) != 0) {
    return false;
  }

  if (xTaskCreatePinnedToCore(txTask, "triwhirl_ble_tx", 4096, nullptr, 2,
                              nullptr, 0) != pdPASS) {
    return false;
  }
  if (xTaskCreatePinnedToCore(traceTxTask, "triwhirl_trace_tx", 4096, nullptr,
                              2, nullptr, 0) != pdPASS) {
    return false;
  }

  initialized = true;
  nimble_port_freertos_init(hostTask);
  return true;
}

std::size_t read(std::uint8_t* data, const std::size_t capacity) {
  if (data == nullptr || capacity == 0U || rx_stream == nullptr) {
    return 0U;
  }
  return xStreamBufferReceive(rx_stream, data, capacity, 0);
}

std::size_t write(const std::uint8_t* data, const std::size_t length) {
  if (data == nullptr || length == 0U || tx_stream == nullptr ||
      connection_handle == BLE_HS_CONN_HANDLE_NONE || !tx_subscribed) {
    return 0U;
  }
  const std::size_t queued = xStreamBufferSend(tx_stream, data, length, 0);
  if (queued < length) {
    tx_dropped_bytes += static_cast<std::uint32_t>(length - queued);
  }
  return queued;
}

std::size_t writeBlocking(const std::uint8_t* data,
                          const std::size_t length,
                          const std::uint32_t timeout_ms) {
  if (!tx_subscribed) return 0U;
  return writeBlockingTo(tx_stream, &tx_dropped_bytes, data, length, timeout_ms);
}

std::size_t traceWriteBlocking(const std::uint8_t* data,
                               const std::size_t length,
                               const std::uint32_t timeout_ms) {
  if (!trace_tx_subscribed) return 0U;
  return writeBlockingTo(trace_tx_stream, &trace_tx_dropped_bytes, data, length,
                         timeout_ms);
}

bool connected() {
  return connection_handle != BLE_HS_CONN_HANDLE_NONE;
}

bool subscribed() {
  return tx_subscribed;
}

bool traceSubscribed() {
  return trace_tx_subscribed;
}

std::uint32_t rxDroppedBytes() {
  return rx_dropped_bytes;
}

std::uint32_t txDroppedBytes() {
  return tx_dropped_bytes;
}

std::uint32_t traceTxDroppedBytes() {
  return trace_tx_dropped_bytes;
}

}  // namespace ble
}  // namespace triwhirl

#else

namespace triwhirl {
namespace ble {

bool init() { return false; }
std::size_t read(std::uint8_t*, std::size_t) { return 0U; }
std::size_t write(const std::uint8_t*, std::size_t) { return 0U; }
std::size_t writeBlocking(const std::uint8_t*, std::size_t, std::uint32_t) {
  return 0U;
}
std::size_t traceWriteBlocking(const std::uint8_t*, std::size_t,
                               std::uint32_t) {
  return 0U;
}
bool connected() { return false; }
bool subscribed() { return false; }
bool traceSubscribed() { return false; }
std::uint32_t rxDroppedBytes() { return 0U; }
std::uint32_t txDroppedBytes() { return 0U; }
std::uint32_t traceTxDroppedBytes() { return 0U; }

}  // namespace ble
}  // namespace triwhirl

#endif
