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

#ifndef CUDA_BB_BENCHMARK__TEST_PATTERN_UTILS_HPP_
#define CUDA_BB_BENCHMARK__TEST_PATTERN_UTILS_HPP_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace cuda_bb_benchmark
{

/// \brief Generate deterministic test byte for a given sequence and offset.
inline std::uint8_t generate_test_byte(std::size_t sequence, std::size_t index)
{
  return static_cast<std::uint8_t>((sequence * 37u + index * 17u + 11u) & 0xFFu);
}

/// \brief Generate uniform byte value for a sequence number.
inline std::uint8_t generate_sequence_byte(std::size_t sequence)
{
  return static_cast<std::uint8_t>((sequence * 37u + 11u) & 0xFFu);
}

/// \brief Fill host buffer with uniform byte for sequence.
inline void fill_host_uniform(std::uint8_t * data, std::size_t size, std::size_t sequence)
{
  if (data == nullptr) {
    return;
  }
  const std::uint8_t val = generate_sequence_byte(sequence);
  std::memset(data, static_cast<int>(val), size);
}

/// \brief Fill host buffer with deterministic patterned bytes.
inline void fill_host_pattern(std::uint8_t * data, std::size_t size, std::size_t sequence)
{
  if (data == nullptr) {
    return;
  }
  for (std::size_t i = 0; i < size; ++i) {
    data[i] = generate_test_byte(sequence, i);
  }
}

/// \brief Verify host buffer against expected uniform byte.
inline bool verify_host_uniform(
  const std::uint8_t * data, std::size_t size, std::size_t sequence)
{
  if (data == nullptr && size > 0) {
    return false;
  }
  const std::uint8_t expected = generate_sequence_byte(sequence);
  for (std::size_t i = 0; i < size; ++i) {
    if (data[i] != expected) {
      return false;
    }
  }
  return true;
}

/// \brief Verify host buffer against expected deterministic pattern.
inline bool verify_host_pattern(
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

}  // namespace cuda_bb_benchmark

#endif  // CUDA_BB_BENCHMARK__TEST_PATTERN_UTILS_HPP_
