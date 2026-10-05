#include "runtime_simplefoc_motor_commissioning.hpp"

#if !defined(TRIWHIRL_ROUTE_B_MOTOR_COMMISSIONING)
#error "simplefoc_motor_app_main.cpp is motor-commissioning-profile only"
#endif

extern "C" void app_main(void) {
  triwhirl::runtime::runSimpleFocMotorCommissioning();
}
