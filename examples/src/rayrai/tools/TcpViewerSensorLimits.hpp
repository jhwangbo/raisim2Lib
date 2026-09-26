// Copyright (c) 2026 Raion Robotics Inc.
// All rights reserved.
#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

namespace raisin::tcp_viewer {

inline bool validSensorDimensions(int width, int height) {
  constexpr int kMaxSide = 8192;
  constexpr int64_t kMaxPixels = 8 * 1024 * 1024;
  return width > 0 && height > 0 && width <= kMaxSide &&
         height <= kMaxSide &&
         static_cast<int64_t>(width) * height <= kMaxPixels;
}

// RaiSimServer's 32 MiB receive buffer includes a four-byte frame length.
constexpr size_t kMaxSensorUpdatePayloadBytes = 33554432u - 4u;

inline std::vector<size_t> selectSensorUpdateIndices(
    const std::vector<size_t>& entryBytes, size_t start,
    size_t headerBytes = 20u) {
  std::vector<size_t> selected;
  if (entryBytes.empty()) return selected;
  size_t used = headerBytes;
  for (size_t offset = 0; offset < entryBytes.size(); ++offset) {
    const size_t index = (start + offset) % entryBytes.size();
    const size_t bytes = entryBytes[index];
    if (bytes <= kMaxSensorUpdatePayloadBytes - used) {
      selected.push_back(index);
      used += bytes;
    }
  }
  return selected;
}

} // namespace raisin::tcp_viewer
