// Copyright (c) 2026 Raion Robotics Inc.
// All rights reserved.

#include "TcpViewerSensors.hpp"
#include "TcpViewerSensorLimits.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <utility>

#include <SDL.h>
#include <glbinding/gl/gl.h>
#include <glm/gtc/quaternion.hpp>

#include "rayrai/Camera.hpp"
#include "rayrai/RayraiWindow.hpp"
#include "raisim/sensors/DepthSensor.hpp"
#include "raisim/sensors/RGBSensor.hpp"

namespace raisin::tcp_viewer
{
namespace
{

std::string sensorKey(const SensorInfo& info) {
  return std::to_string(info.parentTag) + ":" + std::to_string(static_cast<int>(info.type)) +
         ":" + info.name;
}

void configureCamera(Camera& camera, const PendingSensorUpdate& update) {
  const auto& info = update.info;
  camera.ensureRenderTargets(info.width, info.height);
  camera.aspect = static_cast<float>(info.width) / static_cast<float>(info.height);
  camera.nearPlane = camera.zNear = static_cast<float>(std::max(1.0e-4, info.clipNear));
  camera.farPlane = camera.zFar = static_cast<float>(std::max(info.clipFar, info.clipNear + 1.0e-3));
  camera.setLensModel(info.lens, info.hFov);
  const double rasterHFov = info.lens.isFisheye() ? std::min(info.hFov, 3.124139361) : info.hFov;
  camera.zoom = glm::degrees(static_cast<float>(
    2.0 * std::atan(std::tan(0.5 * rasterHFov) / std::max(1.0e-6f, camera.aspect))));
  camera.setProjectionMode(Camera::ProjectionMode::PERSPECTIVE);

  glm::quat orientation(update.orientation.w, update.orientation.x,
                        update.orientation.y, update.orientation.z);
  const float norm = glm::length(orientation);
  orientation = std::isfinite(norm) && norm > 1.0e-6f
                  ? orientation / norm
                  : glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  const glm::mat3 rotation = glm::mat3_cast(orientation);
  camera.position = update.position;
  camera.front = glm::normalize(rotation[0]);
  camera.up = glm::normalize(rotation[2]);
  camera.right = glm::normalize(glm::cross(camera.front, camera.up));
  camera.worldUp = camera.up;
  camera.target = camera.position + camera.front;
}

/** RaiSim camera properties matching a remote sensor, for reading back its image. */
template <typename Properties>
Properties sensorCameraProperties(const SensorInfo& info) {
  Properties properties;
  properties.name = info.name;
  properties.full_name = info.name;
  properties.width = info.width;
  properties.height = info.height;
  properties.clipNear = info.clipNear;
  properties.clipFar = info.clipFar;
  properties.hFOV = info.hFov;
  properties.lens = info.lens;
  return properties;
}

/**
 * Returns a RaiSim camera with the request's resolution for Camera::getRawImage(). With a custom
 * buffer that call reads only the resolution, and constructing a DepthCamera computes a ray
 * direction per pixel, so one camera is kept per stream and rebuilt when the resolution changes.
 */
template <typename SensorType, typename Properties>
SensorType& readbackSensor(std::unique_ptr<SensorType>& sensor, const SensorInfo& info) {
  if (!sensor || sensor->getProperties().width != info.width ||
      sensor->getProperties().height != info.height) {
    raisim::Vec<3> position;
    position.setZero();
    raisim::Mat<3, 3> rotation;
    rotation.setIdentity();
    sensor = std::make_unique<SensorType>(
      sensorCameraProperties<Properties>(info), nullptr, position, rotation);
  }
  return *sensor;
}

void uploadDepthPreview(unsigned int& texture, int width, int height,
                        const std::vector<float>& depth,
                        const detail::DepthPreviewShading& shading,
                        std::vector<unsigned char>& rgba,
                        float& minimumDepth, float& maximumDepth) {
  detail::makeDepthPreviewRgba(depth, shading, rgba, minimumDepth, maximumDepth);

  if (texture == 0) {
    gl::glGenTextures(1, &texture);
  }
  gl::glBindTexture(gl::GL_TEXTURE_2D, texture);
  gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_MIN_FILTER, gl::GL_LINEAR);
  gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_MAG_FILTER, gl::GL_LINEAR);
  gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_WRAP_S, gl::GL_CLAMP_TO_EDGE);
  gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_WRAP_T, gl::GL_CLAMP_TO_EDGE);
  gl::glPixelStorei(gl::GL_UNPACK_ALIGNMENT, 1);
  gl::glTexImage2D(gl::GL_TEXTURE_2D, 0, gl::GL_RGBA8, width, height, 0,
                   gl::GL_RGBA, gl::GL_UNSIGNED_BYTE, rgba.data());
  gl::glBindTexture(gl::GL_TEXTURE_2D, 0);
}

} // namespace

namespace detail
{

DepthPreviewShading::DepthPreviewShading(float clipMinimum, float clipMaximum)
    : clipMinimum_(clipMinimum), clipMaximum_(clipMaximum) {
  scaleMinimum_ = std::isfinite(clipMinimum) ? std::max(clipMinimum, 1.0e-6f) : 1.0e-6f;
  scaleMaximum_ = std::isfinite(clipMaximum) && clipMaximum > scaleMinimum_
    ? clipMaximum
    : scaleMinimum_ + 1.0e-6f;
  const float scaleMinimum = scaleMinimum_;
  const float scaleMaximum = scaleMaximum_;
  const float inverseLogRange = 1.0f / std::log(scaleMaximum / scaleMinimum);
  const auto formulaShade = [&](float value) {
    const float normalized = std::log(value / scaleMinimum) * inverseLogRange;
    return static_cast<unsigned char>(std::lround(255.0f * normalized));
  };
  // Positive floats are ordered like their bit patterns, so bisect over those for the first
  // depth of each shade. Every shade starts at or after the previous one.
  const auto bits = [](float value) {
    uint32_t result = 0;
    std::memcpy(&result, &value, sizeof(result));
    return result;
  };
  const auto fromBits = [](uint32_t value) {
    float result = 0.0f;
    std::memcpy(&result, &value, sizeof(result));
    return result;
  };
  const unsigned char maximumShade = formulaShade(scaleMaximum);
  bounds_[0] = -std::numeric_limits<float>::infinity();
  uint32_t low = bits(scaleMinimum);
  for (int shade = 1; shade <= 255; ++shade) {
    if (maximumShade < shade) {
      bounds_[static_cast<size_t>(shade)] = std::numeric_limits<float>::infinity();
      continue;
    }
    uint32_t high = bits(scaleMaximum);
    while (low < high) {
      const uint32_t middle = low + (high - low) / 2u;
      if (formulaShade(fromBits(middle)) >= shade) {
        high = middle;
      } else {
        low = middle + 1u;
      }
    }
    bounds_[static_cast<size_t>(shade)] = fromBits(low);
  }
  bounds_[256] = std::numeric_limits<float>::infinity();

  firstBucket_ = bits(scaleMinimum) >> 16u;
  const uint32_t lastBucket = bits(scaleMaximum) >> 16u;
  bucketShades_.resize(lastBucket - firstBucket_ + 1u);
  const auto firstBound = bounds_.begin() + 1;
  for (uint32_t bucket = firstBucket_; bucket <= lastBucket; ++bucket) {
    const float edge = std::max(fromBits(bucket << 16u), scaleMinimum);
    bucketShades_[bucket - firstBucket_] = static_cast<unsigned char>(
      std::upper_bound(firstBound, bounds_.begin() + 256, edge) - firstBound);
  }
}

unsigned char DepthPreviewShading::shade(float depth) const {
  const float clamped = std::clamp(depth, scaleMinimum_, scaleMaximum_);
  uint32_t bits = 0;
  std::memcpy(&bits, &clamped, sizeof(bits));
  size_t shade = bucketShades_[(bits >> 16u) - firstBucket_];
  // A bucket spans less than one shade for typical clip ranges, so one step usually suffices.
  shade += clamped >= bounds_[shade + 1u] ? 1u : 0u;
  while (clamped >= bounds_[shade + 1u]) ++shade;
  return static_cast<unsigned char>(shade);
}

void makeDepthPreviewRgba(const std::vector<float>& depth, const DepthPreviewShading& shading,
  std::vector<unsigned char>& rgba, float& minimumDepth, float& maximumDepth) {
  rgba.resize(depth.size() * 4u);
  minimumDepth = std::numeric_limits<float>::infinity();
  maximumDepth = 0.0f;
  for (size_t i = 0; i < depth.size(); ++i) {
    const float value = depth[i];
    unsigned char shade = 0;
    if (std::isfinite(value) && value > 0.0f) {
      minimumDepth = std::min(minimumDepth, value);
      maximumDepth = std::max(maximumDepth, value);
      shade = shading.shade(value);
    }
    rgba[i * 4u + 0u] = shade;
    rgba[i * 4u + 1u] = shade;
    rgba[i * 4u + 2u] = shade;
    rgba[i * 4u + 3u] = 255;
  }
  if (!std::isfinite(minimumDepth)) {
    minimumDepth = 0.0f;
  }
}

std::vector<unsigned char> makeDepthPreviewRgba(const std::vector<float>& depth,
  float clipMinimum, float clipMaximum, float& minimumDepth, float& maximumDepth) {
  std::vector<unsigned char> rgba;
  makeDepthPreviewRgba(depth, DepthPreviewShading(clipMinimum, clipMaximum), rgba, minimumDepth,
    maximumDepth);
  return rgba;
}

} // namespace detail

struct SensorRenderer::CachedSensor {
  std::unique_ptr<Camera> camera;
  std::unique_ptr<raisim::RGBCamera> rgbSensor;
  std::unique_ptr<raisim::DepthCamera> depthSensor;
  std::optional<detail::DepthPreviewShading> depthShading;
  std::vector<unsigned char> depthPreviewRgba;
  SensorPreviewInfo preview;
  unsigned int depthPreviewTexture = 0;
};

SensorRenderer::SensorRenderer() = default;

SensorRenderer::~SensorRenderer() {
  clear();
}

bool SensorRenderer::render(RayraiWindow& viewer, std::vector<PendingSensorUpdate>& updates,
                            std::string& status) {
  // Keep the update handshake alive even when a remote sensor advertises an
  // unusable size. The caller sends an empty acknowledgment if all updates
  // are filtered, and the server can continue serving scene frames.
  updates.erase(std::remove_if(updates.begin(), updates.end(),
      [](const PendingSensorUpdate& update) {
        return !validSensorDimensions(update.info.width, update.info.height);
      }), updates.end());
  RayraiWindow::RenderOverrides overrides;
  overrides.doShadows = true;
  overrides.drawCoordinateFrames = false;
  overrides.drawPointClouds = false;
  overrides.drawVisualizationObjects = true;
  overrides.drawWeatherEffects = false;
  overrides.postProcess = false;
  overrides.allowViewerMsaa = false;
  overrides.allowTemporalAa = false;
  overrides.allowViewerUpscale = false;

  for (auto& update : updates) {
    const auto& info = update.info;
    if ((info.type != raisim::Sensor::Type::RGB &&
         info.type != raisim::Sensor::Type::DEPTH) ||
        !validSensorDimensions(info.width, info.height)) {
      status = "unsupported or invalid sensor request: " + info.name;
      return false;
    }

    const std::string key = sensorKey(info);
    auto& cached = sensors_[key];
    if (!cached) {
      cached = std::make_unique<CachedSensor>();
      cached->camera = std::make_unique<Camera>();
    }
    configureCamera(*cached->camera, update);

    const auto started = std::chrono::steady_clock::now();
    if (info.type == raisim::Sensor::Type::RGB) {
      viewer.renderWithExternalCamera(*cached->camera, overrides);
      auto& sensor = readbackSensor<raisim::RGBCamera, raisim::RGBCamera::RGBCameraProperties>(
        cached->rgbSensor, info);
      update.colorBgra.resize(static_cast<size_t>(info.width) *
                              static_cast<size_t>(info.height) * 4u);
      cached->camera->getRawImage(sensor, Camera::SensorStorageMode::CUSTOM_BUFFER,
        reinterpret_cast<char*>(update.colorBgra.data()), update.colorBgra.size(), true);
      cached->preview.texture = cached->camera->getFinalTexture();
    } else {
      cached->camera->ensureLinearDepthRenderTarget();
      // TCP physical geometry is marked detectable when RemoteScene reconstructs
      // it. Use the same filter as RGB so viewer-only helpers do not leak into
      // either sensor stream.
      viewer.renderDepthPlaneDistance(*cached->camera, nullptr, true, true);
      auto& sensor =
        readbackSensor<raisim::DepthCamera, raisim::DepthCamera::DepthCameraProperties>(
          cached->depthSensor, info);
      update.depth.resize(static_cast<size_t>(info.width) * static_cast<size_t>(info.height));
      cached->camera->getRawImage(sensor, Camera::SensorStorageMode::CUSTOM_BUFFER,
        update.depth.data(), update.depth.size(), true);
      const float clipNear = static_cast<float>(info.clipNear);
      const float clipFar = static_cast<float>(info.clipFar);
      if (!cached->depthShading || !cached->depthShading->matches(clipNear, clipFar)) {
        cached->depthShading.emplace(clipNear, clipFar);
      }
      uploadDepthPreview(cached->depthPreviewTexture, info.width, info.height, update.depth,
        *cached->depthShading, cached->depthPreviewRgba, cached->preview.minimumDepth,
        cached->preview.maximumDepth);
      cached->preview.texture = cached->depthPreviewTexture;
    }
    update.renderMilliseconds = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    cached->preview.parentTag = info.parentTag;
    cached->preview.type = info.type;
    cached->preview.name = info.name;
    cached->preview.width = info.width;
    cached->preview.height = info.height;
    cached->preview.renderMilliseconds = update.renderMilliseconds;
  }
  status = "rendered " + std::to_string(updates.size()) + " sensor request(s)";
  return true;
}

std::vector<SensorPreviewInfo> SensorRenderer::previews() const {
  std::vector<SensorPreviewInfo> result;
  result.reserve(sensors_.size());
  for (const auto& item : sensors_) {
    result.push_back(item.second->preview);
  }
  std::sort(result.begin(), result.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.name < rhs.name;
  });
  return result;
}

std::vector<SensorPreviewInfo> SensorRenderer::previewsForTag(uint32_t parentTag) const {
  auto result = previews();
  result.erase(std::remove_if(result.begin(), result.end(), [parentTag](const auto& preview) {
    return preview.parentTag != parentTag;
  }), result.end());
  return result;
}

void SensorRenderer::clear() {
  if (SDL_GL_GetCurrentContext() != nullptr) {
    for (const auto& item : sensors_) {
      if (item.second && item.second->depthPreviewTexture != 0) {
        gl::glDeleteTextures(1, &item.second->depthPreviewTexture);
      }
    }
  }
  sensors_.clear();
}

} // namespace raisin::tcp_viewer
