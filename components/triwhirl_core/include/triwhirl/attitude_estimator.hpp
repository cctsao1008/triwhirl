#pragma once

namespace triwhirl {

struct AttitudeEstimatorConfig {
  float kp = 8.0F;
  float ki = 0.5F;
  float accel_norm_min_mps2 = 6.0F;
  float accel_norm_max_mps2 = 13.5F;
};

struct AttitudeEstimate {
  float angle_rad = 0.0F;
  float rate_rad_s = 0.0F;
  float gyro_bias_rad_s = 0.0F;
  float gravity_innovation = 0.0F;
  float accel_weight = 0.0F;
  bool valid = false;
};

class PlanarAttitudeEstimator {
 public:
  explicit PlanarAttitudeEstimator(
      const AttitudeEstimatorConfig& config = AttitudeEstimatorConfig{});

  void reset(float angle_rad = 0.0F, float gyro_bias_rad_s = 0.0F);

  AttitudeEstimate update(float body_accel_x_mps2,
                          float body_accel_z_mps2,
                          float body_gyro_rad_s,
                          float dt_s,
                          bool allow_bias_update = true);

  const AttitudeEstimate& state() const { return state_; }

 private:
  AttitudeEstimatorConfig config_{};
  AttitudeEstimate state_{};
};

}  // namespace triwhirl
