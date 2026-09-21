#pragma once

#include <cstddef>

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

// Preserve the historical CLI's token semantics without changing echoed input:
// trim leading/trailing horizontal whitespace and collapse internal runs to one
// ASCII space. The returned count excludes the terminating NUL.
std::size_t normalizeRuntimeCommandLine(const char* input, char* output,
                                        std::size_t output_bytes);

RuntimeCommandParseResult parseRuntimeCommand(const char* line);

}  // namespace triwhirl::runtime
