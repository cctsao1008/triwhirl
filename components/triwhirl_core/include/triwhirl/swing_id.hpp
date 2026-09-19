#pragma once

#include <cstdint>

namespace triwhirl {

enum class SwingIdState : std::uint8_t {
  kIdle,
  kPump,
  kProbe,
  kRearm,
  kComplete,
  kAborted,
};

enum class SwingIdVertex : std::uint8_t {
  kNone,
  kA,
  kB,
  kC,
};

enum class SwingIdStopReason : std::uint8_t {
  kNone,
  kTargetReached,
  kTimeout,
  kInvalidInput,
  kSafetyFault,
  kExternalAbort,
};

struct SwingIdConfig {
  std::uint32_t target_captures = 12U;
  float pump_v_low = 0.533333F;
  float pump_v_high = 0.733333F;
  float probe_v_negative = -0.25F;
  float probe_v_positive = 0.25F;
  float capture_deg = 8.0F;
  float probe_exit_deg = 12.0F;
  float rearm_deg = 18.0F;
  std::uint32_t probe_duration_us = 160000U;
  float rate_switch_rad_s = 0.03F;
  int pump_polarity = -1;
  float vertex_a_deg = 68.0F;
  std::uint32_t max_duration_us = 50000000U;
};

struct SwingIdInput {
  std::uint32_t now_us = 0U;
  float theta_rad = 0.0F;
  float theta_rate_rad_s = 0.0F;
  bool attitude_valid = false;
  bool safety_faulted = false;
};

struct SwingIdOutput {
  SwingIdState state = SwingIdState::kIdle;
  SwingIdVertex vertex = SwingIdVertex::kNone;
  SwingIdStopReason stop_reason = SwingIdStopReason::kNone;
  float desired_vq_v = 0.0F;
  float vertex_error_deg = 0.0F;
  std::uint32_t capture_count = 0U;
  std::uint32_t half_cycle_index = 0U;
  bool pump_active = false;
  bool probe_active = false;
  bool critical_window = false;
  bool transition = false;
};

class SwingIdRunner {
 public:
  explicit SwingIdRunner(const SwingIdConfig& config = SwingIdConfig{});

  bool configure(const SwingIdConfig& config);
  const SwingIdConfig& config() const { return config_; }

  bool start(const SwingIdInput& input);
  SwingIdOutput update(const SwingIdInput& input);
  SwingIdOutput abort(SwingIdStopReason reason = SwingIdStopReason::kExternalAbort);
  const SwingIdOutput& output() const { return output_; }
  bool active() const;

 private:
  static bool validConfig(const SwingIdConfig& config);
  static float wrapDeg(float angle_deg);
  static float angleDiffDeg(float angle_deg, float reference_deg);
  static float radiansToDegrees(float angle_rad);
  static int vertexIndex(SwingIdVertex vertex);

  void classifyVertex(float theta_rad, SwingIdVertex* vertex,
                      float* center_deg, float* error_deg) const;
  void updatePumpHalfCycle(float theta_rate_rad_s);
  float pumpCommand() const;
  float scheduledProbeCommand(SwingIdVertex vertex) const;
  void setState(SwingIdState state, bool transition = true);
  SwingIdOutput stop(SwingIdState state, SwingIdStopReason reason);

  SwingIdConfig config_{};
  SwingIdOutput output_{};
  std::uint32_t start_us_ = 0U;
  std::uint32_t probe_start_us_ = 0U;
  std::uint32_t rearm_start_half_cycle_ = 0U;
  int pump_rate_sign_ = 1;
  float current_pump_v_ = 0.733333F;
  float probe_vq_v_ = 0.0F;
  SwingIdVertex probe_vertex_ = SwingIdVertex::kNone;
  float probe_center_deg_ = 0.0F;
  std::uint32_t vertex_capture_counts_[3]{};
};

const char* swingIdStateName(SwingIdState state);
const char* swingIdVertexName(SwingIdVertex vertex);
const char* swingIdStopReasonName(SwingIdStopReason reason);

}  // namespace triwhirl
