#pragma once

namespace triwhirl::runtime {

// Dedicated Route-B passive AS5600 commissioning entry. This profile is
// selected only by TRIWHIRL_ROUTE_B_SENSOR_COMMISSIONING and never enters the
// native motor runtime or the SimpleFOC motor/driver/FOC path.
[[noreturn]] void runSimpleFocSensorCommissioning();

}  // namespace triwhirl::runtime
