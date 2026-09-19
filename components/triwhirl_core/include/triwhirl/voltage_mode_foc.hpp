#pragma once

#include "triwhirl/three_phase_field.hpp"

namespace triwhirl {

struct MotorElectricalConfig {
  int pole_pairs = 0;
  int sensor_direction = 1;
  float electrical_offset_rad = 0.0F;
};

bool validMotorElectricalConfig(const MotorElectricalConfig& config);
float wrapElectricalAngle(float angle_rad);
float electricalAngleFromMechanical(float mechanical_angle_rad,
                                    const MotorElectricalConfig& config);

PhaseVoltages makeDqVoltage(float electrical_angle_rad,
                            float vd_v,
                            float vq_v,
                            float bus_voltage_v,
                            float vector_limit_v);

}  // namespace triwhirl
