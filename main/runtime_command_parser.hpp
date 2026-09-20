#pragma once

#include "runtime_command.hpp"

namespace triwhirl::runtime {

enum class RuntimeCommandParseStatus {
  kNotMatched,
  kCommand,
  kUsageError,
};

struct RuntimeCommandParseResult {
  RuntimeCommandParseStatus status = RuntimeCommandParseStatus::kNotMatched;
  RuntimeCommand command{};
  const char* error = nullptr;
};

RuntimeCommandParseResult parseRuntimeCommand(const char* line);

}  // namespace triwhirl::runtime
