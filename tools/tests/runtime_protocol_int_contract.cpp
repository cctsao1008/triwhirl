#include <cassert>
#include <cstdint>
#include <type_traits>

#include "runtime_reply.hpp"
#include "runtime_state_event.hpp"

int main() {
  using triwhirl::runtime::RuntimeReply;
  using triwhirl::runtime::RuntimeStateEvent;

  static_assert(sizeof(int) == sizeof(std::int32_t));
  static_assert(std::is_same_v<decltype(RuntimeReply{}.value0), int>);
  static_assert(std::is_same_v<decltype(RuntimeStateEvent{}.value0), int>);
  static_assert(std::is_trivially_copyable_v<RuntimeReply>);
  static_assert(std::is_trivially_copyable_v<RuntimeStateEvent>);

  assert(sizeof(RuntimeReply) <= 104U);
  assert(sizeof(RuntimeStateEvent) <= 32U);
  return 0;
}
