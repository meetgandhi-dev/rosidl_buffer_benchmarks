// Copyright 2026 Meet Gandhi
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SHMEM_BB_BENCHMARK__TEST_PATTERN_UTILS_HPP_
#define SHMEM_BB_BENCHMARK__TEST_PATTERN_UTILS_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

namespace shmem_bb_benchmark
{

/// \brief Generate a deterministic byte for a given sequence and element offset.
inline std::uint8_t generate_test_byte(std::size_t sequence, std::size_t index)
{
  return static_cast<std::uint8_t>((sequence * 37u + index * 17u + 11u) & 0xFFu);
}

/// \brief Fill a buffer with deterministic test pattern.
inline void fill_test_pattern(std::uint8_t * data, std::size_t size, std::size_t sequence)
{
  if (data == nullptr) {
    return;
  }
  for (std::size_t i = 0; i < size; ++i) {
    data[i] = generate_test_byte(sequence, i);
  }
}

/// \brief Verify that a buffer matches the deterministic test pattern.
inline bool verify_test_pattern(
  const std::uint8_t * data, std::size_t size, std::size_t sequence)
{
  if (data == nullptr && size > 0) {
    return false;
  }
  for (std::size_t i = 0; i < size; ++i) {
    if (data[i] != generate_test_byte(sequence, i)) {
      return false;
    }
  }
  return true;
}

}  // namespace shmem_bb_benchmark

#endif  // SHMEM_BB_BENCHMARK__TEST_PATTERN_UTILS_HPP_
