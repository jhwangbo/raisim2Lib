// Copyright (c) 2026 Raion Robotics Inc.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rayrai/TcpRemoteSceneState.hpp"

namespace raisin
{
class Camera;
class RayraiWindow;
}

namespace raisin::tcp_viewer
{

namespace detail
{

/** Convert metric depth to RGBA using a log grayscale anchored to the camera clip range. */
std::vector<unsigned char> makeDepthPreviewRgba(const std::vector<float>& depth,
  float clipMinimum, float clipMaximum, float& minimumDepth, float& maximumDepth);

} // namespace detail

struct SensorPreviewInfo {
  uint32_t parentTag = 0;
  raisim::Sensor::Type type = raisim::Sensor::Type::UNKNOWN;
  std::string name;
  int width = 0;
  int height = 0;
  unsigned int texture = 0;
  double renderMilliseconds = 0.0;
  float minimumDepth = 0.0f;
  float maximumDepth = 0.0f;
};

/** Renders manual TCP camera requests and retains textures for the diagnostics UI. */
class SensorRenderer {
 public:
  SensorRenderer();
  ~SensorRenderer();
  SensorRenderer(const SensorRenderer&) = delete;
  SensorRenderer& operator=(const SensorRenderer&) = delete;

  bool render(RayraiWindow& viewer, std::vector<PendingSensorUpdate>& updates,
              std::string& status);
  std::vector<SensorPreviewInfo> previews() const;
  std::vector<SensorPreviewInfo> previewsForTag(uint32_t parentTag) const;
  void clear();

 private:
  struct CachedSensor;
  std::unordered_map<std::string, std::unique_ptr<CachedSensor>> sensors_;
};

} // namespace raisin::tcp_viewer
