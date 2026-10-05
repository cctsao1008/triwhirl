#include "runtime_simplefoc_motor_commissioning.hpp"

#if !defined(TRIWHIRL_ROUTE_B_MOTOR_COMMISSIONING)
#error "simplefoc_motor_app_main.cpp is motor-commissioning-profile only"
#endif

// Arduino-ESP32 4.x releases unused Bluetooth memory during initArduino().
// The motor commissioning image uses Classic-BT SPP directly rather than the
// BluetoothSerial library, so explicitly mark Classic BT as linked/in-use
// before app_main() and therefore before initArduino() runs.
#include "esp32-hal-alloc-bt-classic-mem.h"

extern "C" void app_main(void) {
  triwhirl::runtime::runSimpleFocMotorCommissioning();
}
