// Copyright (c) 2026 Raion Robotics Inc.
// All rights reserved.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace raisin::tcp_viewer
{

inline constexpr const char* kSessionMagic = "RAYRAI_TCP_VIEWER_SESSION_V1\n";

struct RecordedFrame {
  uint64_t timeMicros = 0;
  std::vector<char> payload;
};

class SessionRecorder {
 public:
  bool open(const std::filesystem::path& path, std::string& status);
  void close();

  bool active() const;
  size_t frameCount() const;
  size_t byteCount() const;
  std::string pathString() const;

  bool record(const std::vector<char>& payload, std::chrono::steady_clock::time_point now,
              std::string& status);

 private:
  std::ofstream output_;
  std::filesystem::path path_;
  std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();
  size_t frames_ = 0;
  size_t bytes_ = 0;
};

bool loadSessionFile(const std::filesystem::path& path, std::vector<RecordedFrame>& frames,
                     std::string& status);

} // namespace raisin::tcp_viewer
