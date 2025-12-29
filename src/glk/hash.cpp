#include <glk/hash.hpp>

#include <rapidhash.h>

namespace glk {

std::uint64_t hash(const void* data, std::size_t size, std::uint64_t seed) {
  return rapidhash_withSeed(data, size, seed);
}

// https://gist.github.com/ruby0x1/81308642d0325fd386237cfa3b44785c
std::uint64_t fnv1hash(const void* data, std::size_t size, std::uint64_t seed) {
  constexpr uint64_t prime = 0x100000001b3;
  uint64_t hash = 0xcbf29ce484222325 ^ seed;

  for (int i = 0; i < size; ++i) {
    const uint8_t value = static_cast<const uint8_t*>(data)[i];
    hash = (hash ^ value) * prime;
  }

  return hash;
}

}  // namespace glk
