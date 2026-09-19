#include "triwhirl/swing_id.hpp"

#include <cmath>

namespace triwhirl {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kRadToDeg = 180.0F / kPi;

}  // namespace

SwingIdRunner::SwingIdRunner(const SwingIdConfig& config) {
  if (!configure(config)) {
    config_ = SwingIdConfig{};
  }
}

bool SwingIdRunner::validConfig(const SwingIdConfig& config) {
  return config.target_captures > 0U &&
         std::isfinite(config.pump_v_low) &&
         std::isfinite(config.pump_v_high) &&
         config.pump_v_low > 0.0F &&
         config.pump_v_high >= config.pump_v_low &&
         std::isfinite(config.probe_v_negative) &&
         std::isfinite(config.probe_v_positive) &&
         config.probe_v_negative < 0.0F &&
         config.probe_v_positive > 0.0F &&
         std::isfinite(config.capture_deg) &&
         std::isfinite(config.probe_exit_deg) &&
         std::isfinite(config.rearm_deg) &&
         config.capture_deg > 0.0F &&
         config.capture_deg < config.probe_exit_deg &&
         config.probe_exit_deg < config.rearm_deg &&
         config.rearm_deg < 60.0F &&
         config.probe_duration_us > 0U &&
         std::isfinite(config.rate_switch_rad_s) &&
         config.rate_switch_rad_s >= 0.0F &&
         (config.pump_polarity == -1 || config.pump_polarity == 1) &&
         std::isfinite(config.vertex_a_deg) &&
         config.max_duration_us > 0U;
}

bool SwingIdRunner::configure(const SwingIdConfig& config) {
  if (active() || !validConfig(config)) {
    return false;
  }
  config_ = config;
  current_pump_v_ = config_.pump_v_high;
  return true;
}

float SwingIdRunner::wrapDeg(float angle_deg) {
  if (!std::isfinite(angle_deg)) {
    return 0.0F;
  }
  angle_deg = std::fmod(angle_deg + 180.0F, 360.0F);
  if (angle_deg < 0.0F) {
    angle_deg += 360.0F;
  }
  return angle_deg - 180.0F;
}

float SwingIdRunner::angleDiffDeg(const float angle_deg,
                                  const float reference_deg) {
  return wrapDeg(angle_deg - reference_deg);
}

float SwingIdRunner::radiansToDegrees(const float angle_rad) {
  return angle_rad * kRadToDeg;
}

int SwingIdRunner::vertexIndex(const SwingIdVertex vertex) {
  switch (vertex) {
    case SwingIdVertex::kA:
      return 0;
    case SwingIdVertex::kB:
      return 1;
    case SwingIdVertex::kC:
      return 2;
    case SwingIdVertex::kNone:
      return -1;
  }
  return -1;
}

void SwingIdRunner::classifyVertex(const float theta_rad,
                                   SwingIdVertex* const vertex,
                                   float* const center_deg,
                                   float* const error_deg) const {
  const float theta_deg = wrapDeg(radiansToDegrees(theta_rad));
  const float centers[3] = {
      wrapDeg(config_.vertex_a_deg),
      wrapDeg(config_.vertex_a_deg - 120.0F),
      wrapDeg(config_.vertex_a_deg + 120.0F),
  };
  const SwingIdVertex ids[3] = {
      SwingIdVertex::kA,
      SwingIdVertex::kB,
      SwingIdVertex::kC,
  };

  int best = 0;
  float best_error = angleDiffDeg(theta_deg, centers[0]);
  for (int index = 1; index < 3; ++index) {
    const float candidate = angleDiffDeg(theta_deg, centers[index]);
    if (std::fabs(candidate) < std::fabs(best_error)) {
      best = index;
      best_error = candidate;
    }
  }

  if (vertex != nullptr) {
    *vertex = ids[best];
  }
  if (center_deg != nullptr) {
    *center_deg = centers[best];
  }
  if (error_deg != nullptr) {
    *error_deg = best_error;
  }
}

void SwingIdRunner::updatePumpHalfCycle(const float theta_rate_rad_s) {
  if (!std::isfinite(theta_rate_rad_s) ||
      std::fabs(theta_rate_rad_s) < config_.rate_switch_rad_s) {
    return;
  }

  const int sign = theta_rate_rad_s > 0.0F ? 1 : -1;
  if (sign == pump_rate_sign_) {
    return;
  }

  pump_rate_sign_ = sign;
  ++output_.half_cycle_index;
  current_pump_v_ = (output_.half_cycle_index % 2U) == 0U
                        ? config_.pump_v_high
                        : config_.pump_v_low;
}

float SwingIdRunner::pumpCommand() const {
  return static_cast<float>(config_.pump_polarity * pump_rate_sign_) *
         current_pump_v_;
}

float SwingIdRunner::scheduledProbeCommand(const SwingIdVertex vertex) const {
  const int index = vertexIndex(vertex);
  if (index < 0) {
    return 0.0F;
  }

  switch (vertex_capture_counts_[index] % 3U) {
    case 0U:
      return config_.probe_v_negative;
    case 1U:
      return config_.probe_v_positive;
    default:
      return 0.0F;
  }
}

void SwingIdRunner::setState(const SwingIdState state, const bool transition) {
  output_.state = state;
  output_.transition = transition;
  // Probe excitation is deliberately independent of the coarse swing pump.
  // Keep PumpActive semantically strict so TWLG can distinguish pump, probe,
  // and zero-vector identification intervals without inference from Vq sign.
  output_.pump_active =
      state == SwingIdState::kPump || state == SwingIdState::kRearm;
  output_.probe_active = state == SwingIdState::kProbe;
  output_.critical_window = state == SwingIdState::kProbe;
}

bool SwingIdRunner::start(const SwingIdInput& input) {
  if (active() || !input.attitude_valid || input.safety_faulted ||
      !std::isfinite(input.theta_rad) || !std::isfinite(input.theta_rate_rad_s)) {
    return false;
  }

  output_ = {};
  start_us_ = input.now_us;
  probe_start_us_ = 0U;
  pump_rate_sign_ =
      std::fabs(input.theta_rate_rad_s) >= config_.rate_switch_rad_s
          ? (input.theta_rate_rad_s > 0.0F ? 1 : -1)
          : 1;
  current_pump_v_ = config_.pump_v_high;
  probe_vq_v_ = 0.0F;
  probe_vertex_ = SwingIdVertex::kNone;
  probe_center_deg_ = 0.0F;
  for (std::uint32_t& count : vertex_capture_counts_) {
    count = 0U;
  }
  setState(SwingIdState::kPump, true);
  output_.desired_vq_v = pumpCommand();
  return true;
}

SwingIdOutput SwingIdRunner::stop(const SwingIdState state,
                                  const SwingIdStopReason reason) {
  setState(state, true);
  output_.stop_reason = reason;
  output_.vertex = SwingIdVertex::kNone;
  output_.vertex_error_deg = 0.0F;
  output_.desired_vq_v = 0.0F;
  output_.pump_active = false;
  output_.probe_active = false;
  output_.critical_window = false;
  probe_vertex_ = SwingIdVertex::kNone;
  probe_vq_v_ = 0.0F;
  return output_;
}

SwingIdOutput SwingIdRunner::abort(const SwingIdStopReason reason) {
  if (!active()) {
    output_.transition = false;
    return output_;
  }
  const SwingIdStopReason resolved =
      reason == SwingIdStopReason::kNone ? SwingIdStopReason::kExternalAbort
                                         : reason;
  return stop(SwingIdState::kAborted, resolved);
}

bool SwingIdRunner::active() const {
  return output_.state == SwingIdState::kPump ||
         output_.state == SwingIdState::kProbe ||
         output_.state == SwingIdState::kRearm;
}

SwingIdOutput SwingIdRunner::update(const SwingIdInput& input) {
  output_.transition = false;

  if (!active()) {
    return output_;
  }

  if (input.safety_faulted) {
    return stop(SwingIdState::kAborted, SwingIdStopReason::kSafetyFault);
  }
  if (!input.attitude_valid || !std::isfinite(input.theta_rad) ||
      !std::isfinite(input.theta_rate_rad_s)) {
    return stop(SwingIdState::kAborted, SwingIdStopReason::kInvalidInput);
  }
  if ((input.now_us - start_us_) >= config_.max_duration_us) {
    return stop(SwingIdState::kAborted, SwingIdStopReason::kTimeout);
  }

  SwingIdVertex nearest = SwingIdVertex::kNone;
  float nearest_center_deg = 0.0F;
  float nearest_error_deg = 0.0F;
  classifyVertex(input.theta_rad, &nearest, &nearest_center_deg,
                 &nearest_error_deg);

  if (output_.state == SwingIdState::kProbe) {
    output_.vertex = probe_vertex_;
    output_.vertex_error_deg =
        angleDiffDeg(radiansToDegrees(input.theta_rad), probe_center_deg_);
    output_.desired_vq_v = probe_vq_v_;

    const bool timed_out =
        (input.now_us - probe_start_us_) >= config_.probe_duration_us;
    const bool left_window =
        std::fabs(output_.vertex_error_deg) >= config_.probe_exit_deg;
    if (timed_out || left_window) {
      const int index = vertexIndex(probe_vertex_);
      if (index >= 0) {
        ++vertex_capture_counts_[index];
      }
      ++output_.capture_count;
      if (output_.capture_count >= config_.target_captures) {
        return stop(SwingIdState::kComplete,
                    SwingIdStopReason::kTargetReached);
      }
      setState(SwingIdState::kRearm, true);
      output_.desired_vq_v = pumpCommand();
    }
    return output_;
  }

  updatePumpHalfCycle(input.theta_rate_rad_s);
  output_.desired_vq_v = pumpCommand();
  output_.vertex = nearest;
  output_.vertex_error_deg = nearest_error_deg;

  if (output_.state == SwingIdState::kPump) {
    if (std::fabs(nearest_error_deg) <= config_.capture_deg) {
      probe_vertex_ = nearest;
      probe_center_deg_ = nearest_center_deg;
      probe_start_us_ = input.now_us;
      probe_vq_v_ = scheduledProbeCommand(nearest);
      setState(SwingIdState::kProbe, true);
      output_.vertex = probe_vertex_;
      output_.vertex_error_deg = nearest_error_deg;
      output_.desired_vq_v = probe_vq_v_;
    }
    return output_;
  }

  if (output_.state == SwingIdState::kRearm) {
    const float probe_error = angleDiffDeg(
        radiansToDegrees(input.theta_rad), probe_center_deg_);
    output_.vertex = probe_vertex_;
    output_.vertex_error_deg = probe_error;
    if (std::fabs(probe_error) >= config_.rearm_deg) {
      probe_vertex_ = SwingIdVertex::kNone;
      probe_center_deg_ = 0.0F;
      setState(SwingIdState::kPump, true);
      output_.vertex = nearest;
      output_.vertex_error_deg = nearest_error_deg;
    }
  }

  return output_;
}

const char* swingIdStateName(const SwingIdState state) {
  switch (state) {
    case SwingIdState::kIdle:
      return "idle";
    case SwingIdState::kPump:
      return "pump";
    case SwingIdState::kProbe:
      return "probe";
    case SwingIdState::kRearm:
      return "rearm";
    case SwingIdState::kComplete:
      return "complete";
    case SwingIdState::kAborted:
      return "aborted";
  }
  return "unknown";
}

const char* swingIdVertexName(const SwingIdVertex vertex) {
  switch (vertex) {
    case SwingIdVertex::kNone:
      return "none";
    case SwingIdVertex::kA:
      return "A";
    case SwingIdVertex::kB:
      return "B";
    case SwingIdVertex::kC:
      return "C";
  }
  return "none";
}

const char* swingIdStopReasonName(const SwingIdStopReason reason) {
  switch (reason) {
    case SwingIdStopReason::kNone:
      return "none";
    case SwingIdStopReason::kTargetReached:
      return "target_reached";
    case SwingIdStopReason::kTimeout:
      return "timeout";
    case SwingIdStopReason::kInvalidInput:
      return "invalid_input";
    case SwingIdStopReason::kSafetyFault:
      return "safety";
    case SwingIdStopReason::kExternalAbort:
      return "external_abort";
  }
  return "unknown";
}

}  // namespace triwhirl
