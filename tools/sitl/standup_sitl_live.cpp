// Continuous interactive runner for the TriWhirl standup SITL.
//
// The browser/server never computes plant or controller dynamics. This process
// owns the fixed-step plant and the production StandupController for the entire
// live session. Commands arrive on stdin and JSON-lines evidence is emitted on
// stdout until an explicit `stop` command is received.
#define main triwhirl_standup_sitl_embedded_main
#include "standup_sitl.cpp"
#undef main

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

enum class LiveCommandKind {
  kBodyRate,
  kWheelRate,
  kStop,
};

struct LiveCommand {
  LiveCommandKind kind = LiveCommandKind::kStop;
  double delta_rad_s = 0.0;
};

std::mutex command_mutex;
std::deque<LiveCommand> command_queue;

void enqueueCommand(const LiveCommand command) {
  std::lock_guard<std::mutex> lock(command_mutex);
  command_queue.push_back(command);
}

std::vector<LiveCommand> takeCommands() {
  std::lock_guard<std::mutex> lock(command_mutex);
  std::vector<LiveCommand> commands;
  commands.reserve(command_queue.size());
  while (!command_queue.empty()) {
    commands.push_back(command_queue.front());
    command_queue.pop_front();
  }
  return commands;
}

void inputThread() {
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream stream(line);
    std::string command;
    stream >> command;
    if (command.empty()) continue;
    if (command == "stop") {
      enqueueCommand(LiveCommand{LiveCommandKind::kStop, 0.0});
      return;
    }

    double delta = 0.0;
    if (!(stream >> delta) || !std::isfinite(delta)) {
      std::cout << "{\"type\":\"error\",\"message\":\"invalid command value\"}"
                << std::endl;
      continue;
    }
    if (command == "body") {
      if (std::fabs(delta) > 5.0) {
        std::cout << "{\"type\":\"error\",\"message\":\"body kick exceeds 5 rad/s\"}"
                  << std::endl;
        continue;
      }
      enqueueCommand(LiveCommand{LiveCommandKind::kBodyRate, delta});
    } else if (command == "wheel") {
      if (std::fabs(delta) > 100.0) {
        std::cout << "{\"type\":\"error\",\"message\":\"wheel kick exceeds 100 rad/s\"}"
                  << std::endl;
        continue;
      }
      enqueueCommand(LiveCommand{LiveCommandKind::kWheelRate, delta});
    } else {
      std::cout << "{\"type\":\"error\",\"message\":\"unknown command\"}"
                << std::endl;
    }
  }
}

void emitSample(const double t_s, const State& state,
                const triwhirl::StandupControllerOutput& output) {
  std::cout << std::setprecision(10)
            << "{\"type\":\"sample\",\"t_s\":" << t_s
            << ",\"true_error_rad\":" << state.theta_error_rad
            << ",\"true_error_deg\":" << state.theta_error_rad * kRadToDeg
            << ",\"true_theta_rate_rad_s\":" << state.theta_rate_rad_s
            << ",\"true_wheel_rate_rad_s\":" << state.wheel_rate_rad_s
            << ",\"true_wheel_angle_rad\":" << state.wheel_angle_rad
            << ",\"phase\":\"" << triwhirl::standupPhaseName(output.phase)
            << "\",\"settling\":" << (output.settling ? "true" : "false")
            << ",\"stable\":" << (output.stable ? "true" : "false")
            << ",\"filtered_rate_rad_s\":" << output.filtered_rate_rad_s
            << ",\"target_velocity_rad_s\":" << output.target_velocity_rad_s
            << ",\"velocity_error_rad_s\":" << output.velocity_error_rad_s
            << ",\"velocity_integral_v\":" << output.velocity_integral_v
            << ",\"vq_unclamped_v\":" << output.vq_unclamped_v
            << ",\"vq_target_v\":" << output.vq_target_v
            << ",\"vq_applied_v\":" << output.vq_v
            << ",\"target_saturated\":"
            << (output.target_saturated ? "true" : "false")
            << ",\"vq_saturated\":"
            << (output.vq_saturated ? "true" : "false") << "}"
            << std::endl;
}

void emitDisturbance(const double t_s, const char* kind, const double delta) {
  std::cout << std::setprecision(10)
            << "{\"type\":\"disturbance\",\"t_s\":" << t_s
            << ",\"kind\":\"" << kind << "\",\"delta_rad_s\":"
            << delta << "}" << std::endl;
}

void usageLive(const char* argv0) {
  std::cout << "usage:\n"
            << "  " << argv0
            << " [--profile nominal|B|C] [--speed X] [--fps N]\n"
            << "stdin commands:\n"
            << "  body <delta_rad_s>\n"
            << "  wheel <delta_rad_s>\n"
            << "  stop\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::string profile = "nominal";
    double speed = 1.0;
    double fps = 60.0;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      auto requireValue = [&](const char* option) -> std::string {
        if (i + 1 >= argc) {
          throw std::runtime_error(std::string("missing value for ") + option);
        }
        return argv[++i];
      };
      if (arg == "--profile") {
        profile = requireValue("--profile");
      } else if (arg == "--speed") {
        speed = std::stod(requireValue("--speed"));
      } else if (arg == "--fps") {
        fps = std::stod(requireValue("--fps"));
      } else if (arg == "--help" || arg == "-h") {
        usageLive(argv[0]);
        return 0;
      } else {
        throw std::runtime_error("unknown argument: " + arg);
      }
    }
    if (!std::isfinite(speed) || speed < 0.1 || speed > 10.0) {
      throw std::runtime_error("speed must be in [0.1, 10]");
    }
    if (!std::isfinite(fps) || fps < 5.0 || fps > 120.0) {
      throw std::runtime_error("fps must be in [5, 120]");
    }

    const PlantModel plant = plantByName(profile);
    const auto config = triwhirl::makeStandupCommissioningConfig(
        static_cast<float>(kThetaReferenceRad), 4.0F);
    triwhirl::StandupController controller(config);
    State state{3.0 / kRadToDeg, -0.5, 0.0, 0.0};
    std::uint32_t now_us = kStartTimeUs;
    auto input = controllerInput(now_us, state);
    controller.reset(input);
    auto output = controller.update(input);

    std::thread reader(inputThread);
    reader.detach();

    std::uint64_t step = 0U;
    double next_emit_s = 0.0;
    const double emit_period_s = 1.0 / fps;
    const auto wall_start = std::chrono::steady_clock::now();

    emitSample(0.0, state, output);

    bool stop_requested = false;
    while (!stop_requested) {
      const double t_s = static_cast<double>(step) * kControlPeriodS;
      for (const LiveCommand& command : takeCommands()) {
        switch (command.kind) {
          case LiveCommandKind::kBodyRate:
            state.theta_rate_rad_s += command.delta_rad_s;
            emitDisturbance(t_s, "body", command.delta_rad_s);
            break;
          case LiveCommandKind::kWheelRate:
            state.wheel_rate_rad_s += command.delta_rad_s;
            emitDisturbance(t_s, "wheel", command.delta_rad_s);
            break;
          case LiveCommandKind::kStop:
            stop_requested = true;
            break;
        }
      }
      if (stop_requested) break;

      for (int substep = 0; substep < kPlantSubsteps; ++substep) {
        state = rk4Step(plant, state, static_cast<double>(output.vq_v),
                        kPlantStepS);
      }
      ++step;
      now_us += kControlPeriodUs;
      const double new_t_s = static_cast<double>(step) * kControlPeriodS;
      input = controllerInput(now_us, state);
      output = controller.update(input);
      if (!output.valid || !std::isfinite(output.vq_v) ||
          !std::isfinite(state.theta_error_rad) ||
          !std::isfinite(state.theta_rate_rad_s) ||
          !std::isfinite(state.wheel_rate_rad_s) ||
          !std::isfinite(state.wheel_angle_rad)) {
        std::cout << "{\"type\":\"error\",\"message\":\"non-finite live SITL state/output\"}"
                  << std::endl;
        return 2;
      }

      if (new_t_s + 1.0e-12 >= next_emit_s) {
        emitSample(new_t_s, state, output);
        do {
          next_emit_s += emit_period_s;
        } while (next_emit_s <= new_t_s + 1.0e-12);
      }

      const auto due = wall_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(new_t_s / speed));
      std::this_thread::sleep_until(due);
    }

    const double final_t_s = static_cast<double>(step) * kControlPeriodS;
    std::cout << std::setprecision(10)
              << "{\"type\":\"end\",\"reason\":\"user_stop\",\"t_s\":"
              << final_t_s << "}" << std::endl;
    return 0;
  } catch (const std::exception& error) {
    std::cout << "{\"type\":\"error\",\"message\":\"" << error.what()
              << "\"}" << std::endl;
    return 2;
  }
}
