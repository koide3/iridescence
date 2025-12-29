#ifndef GLK_HASH_HPP
#define GLK_HASH_HPP

#include <cstdint>
#include <string>

namespace glk {

/// @brief RapidHash (https://github.com/Nicoshev/rapidhash)
/// @param data Pointer to the data to be hashed
/// @param size Size of the data in bytes
/// @param seed Seed value for the hash function
/// @return     64-bit hash value
std::uint64_t hash(const void* data, std::size_t size, std::uint64_t seed = 0);

/// @brief RapidHash for std::string
inline std::uint64_t hash(const std::string& str, std::uint64_t seed = 0) {
  return hash(str.data(), str.size(), seed);
}

/// @brief RapidHash (same as hash())
inline std::uint64_t rapidhash(const void* data, std::size_t size, std::uint64_t seed = 0) {
  return hash(data, size, seed);
}

/// @brief RapidHash for std::string
inline std::uint64_t rapidhash(const std::string& str, std::uint64_t seed = 0) {
  return hash(str.data(), str.size(), seed);
}

/// @brief FNV1hash (https://gist.github.com/ruby0x1/81308642d0325fd386237cfa3b44785c)
/// @note  This is slower and worse than RapidHash. Do not use this unless you have a specific reason.
std::uint64_t fnv1hash(const void* data, std::size_t size, std::uint64_t seed = 0);

/// @brief FNV1hash for std::string
inline std::uint64_t fnv1hash(const std::string& str, std::uint64_t seed = 0) {
  return fnv1hash(str.data(), str.size(), seed);
}

}  // namespace glk

#endif