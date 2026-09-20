#pragma once

#include <cstdint>

#include "runtime_profile_report.hpp"
#include "runtime_reply.hpp"
#include "runtime_state_event.hpp"
#include "runtime_telemetry.hpp"

namespace triwhirl::runtime {

// All Core-1 -> Core-0 protocol data crosses bounded, zero-timeout mailboxes.
// These APIs copy fixed-size records only; formatting and transport remain in
// the supervisor domain.
bool publishRuntimeReply(const RuntimeReply& reply);
bool publishRuntimeStateEvent(const RuntimeStateEvent& event);
bool publishRuntimeTelemetry(const RuntimeTelemetryFrame& frame);
bool publishRuntimeProfileReport(const RuntimeProfileReport& report);

std::uint32_t runtimeEgressDroppedCount();
// Compatibility name retained while callers/diagnostics transition to the
// aggregate egress-drop counter.
std::uint32_t runtimeReplyDroppedCount();

}  // namespace triwhirl::runtime
