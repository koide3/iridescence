#include <guik/viewer/anonymous.hpp>

#include <atomic>

namespace guik {
  namespace {
  std::atomic_uint64_t anonymous_id(0);
  }

  std::string anon() {
    return std::string("anon_") + std::to_string(anonymous_id++);
  }
}