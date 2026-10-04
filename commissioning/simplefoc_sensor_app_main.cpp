#include "runtime_simplefoc_sensor_commissioning.hpp"

#if !defined(TRIWHIRL_ROUTE_B_SENSOR_COMMISSIONING)
#error "simplefoc_sensor_app_main.cpp is commissioning-profile only"
#endif

extern "C" void app_main(void) {
  triwhirl::runtime::runSimpleFocSensorCommissioning();
}
