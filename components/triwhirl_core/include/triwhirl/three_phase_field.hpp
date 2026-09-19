#pragma once

namespace triwhirl {

struct PhaseVoltages {
  float a;
  float b;
  float c;
};

PhaseVoltages makeRotatingField(float electrical_angle_rad,
                                float amplitude_v,
                                float voltage_limit_v);

}  // namespace triwhirl
