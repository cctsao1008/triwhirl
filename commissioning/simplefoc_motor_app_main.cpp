#include "runtime_simplefoc_motor_commissioning.hpp"

#if !defined(TRIWHIRL_ROUTE_B_MOTOR_COMMISSIONING)
#error "simplefoc_motor_app_main.cpp is motor-commissioning-profile only"
#endif

#include "sdkconfig.h"

#if !defined(CONFIG_BT_NIMBLE_ENABLED) || !CONFIG_BT_NIMBLE_ENABLED
#error "SimpleFOC motor commissioning requires CONFIG_BT_NIMBLE_ENABLED"
#endif

#if !defined(CONFIG_BT_NIMBLE_ROLE_PERIPHERAL) || !CONFIG_BT_NIMBLE_ROLE_PERIPHERAL
#error "SimpleFOC motor commissioning requires NimBLE peripheral role"
#endif

// Arduino-ESP32 4.x releases Bluetooth controller memory from initArduino()
// unless the linked image marks BLE as in use before app_main(). The motor
// commissioning image initializes the repository's native NimBLE transport
// after initArduino(), so retain the BLE controller memory explicitly.
#include "esp32-hal-alloc-ble-mem.h"

extern "C" void app_main(void) {
  triwhirl::runtime::runSimpleFocMotorCommissioning();
}
