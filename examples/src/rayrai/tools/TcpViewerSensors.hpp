// Copyright (c) 2026 Raion Robotics Inc.
// All rights reserved.

#pragma once

#include <array>
#include <cstdint>
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

/**
 * Log grayscale for depth previews, anchored to a camera clip range.
 *
 * A shade never decreases with depth, so the mapping is stored as the smallest depth that
 * reaches each shade, found with the per-pixel formula itself. A pixel then needs a bucket
 * lookup and a comparison or two instead of a logarithm, and gets the formula's shade.
 */
class DepthPreviewShading {
 public:
  DepthPreviewShading(float clipMinimum, float clipMaximum);

  /** @return True when this table was built for the given clip range. */
  bool matches(float clipMinimum, float clipMaximum) const {
    return clipMinimum == clipMinimum_ && clipMaximum == clipMaximum_;
  }

  /**
   * @param depth Finite, positive depth.
   * @return Shade of @p depth.
   */
  unsigned char shade(float depth) const;

 private:
  float clipMinimum_;
  float clipMaximum_;
  float scaleMinimum_;
  float scaleMaximum_;
  // A depth has shade k when bounds_[k] <= depth < bounds_[k + 1]. Shades no depth reaches
  // have empty intervals.
  std::array<float, 257> bounds_{};
  // Depths sharing their upper 16 bits (exponent and 7 mantissa bits) form a bucket. This is the
  // shade at the lower edge of each bucket from the one holding scaleMinimum_ on.
  uint32_t firstBucket_ = 0;
  std::vector<unsigned char> bucketShades_;
};

/**
 * Convert metric depth to RGBA using a log grayscale anchored to the camera clip range.
 * @param rgba Resized to four bytes per depth sample; reusing it across frames avoids an
 *        allocation per frame.
 */
void makeDepthPreviewRgba(const std::vector<float>& depth, const DepthPreviewShading& shading,
  std::vector<unsigned char>& rgba, float& minimumDepth, float& maximumDepth);

/** Same as above, building the shading table and the image for one call. */
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
