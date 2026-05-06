#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <glbinding/gl/gl.h>

#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#include "rayrai/example_common.hpp"
#include "rayrai_example_resources.hpp"
#include "rayrai_example_compat.hpp"
#include "rayrai/Visuals.hpp"
#include "raisim/math.hpp"
#include "raisim/World.hpp"

namespace {

void flipRows(std::vector<unsigned char>& rgba, int width, int height) {
  const size_t stride = static_cast<size_t>(width) * 4u;
  std::vector<unsigned char> row(stride);
  for (int y = 0; y < height / 2; ++y) {
    auto* top = rgba.data() + static_cast<size_t>(y) * stride;
    auto* bottom = rgba.data() + static_cast<size_t>(height - 1 - y) * stride;
    std::memcpy(row.data(), top, stride);
    std::memcpy(top, bottom, stride);
    std::memcpy(bottom, row.data(), stride);
  }
}

void saveFinalTexturePng(raisin::Camera& camera, const std::filesystem::path& path) {
  std::vector<unsigned char> rgba(
    static_cast<size_t>(camera.rtWidth()) * static_cast<size_t>(camera.rtHeight()) * 4u);
  gl::glBindTexture(gl::GL_TEXTURE_2D, camera.getFinalTexture());
  gl::glGetTexImage(gl::GL_TEXTURE_2D, 0, gl::GL_RGBA, gl::GL_UNSIGNED_BYTE, rgba.data());
  gl::glBindTexture(gl::GL_TEXTURE_2D, 0);
  flipRows(rgba, camera.rtWidth(), camera.rtHeight());
  stbi_write_png(path.string().c_str(), camera.rtWidth(), camera.rtHeight(), 4,
    rgba.data(), camera.rtWidth() * 4);
}

std::string optionalRayraiRscPath(char* argv0, const std::string& relative) {
  const auto binaryPath = std::filesystem::path(argv0).parent_path();
  const std::vector<std::filesystem::path> candidates = {
    binaryPath / "rsc" / relative,
    binaryPath / ".." / "rsc" / relative,
    binaryPath / ".." / ".." / "rsc" / relative,
    std::filesystem::path("rsc") / relative,
    std::filesystem::path("..") / "rsc" / relative,
    std::filesystem::path("..") / ".." / "rsc" / relative,
  };

  for (const auto& candidate : candidates) {
    if (std::filesystem::exists(candidate)) {
      return candidate.lexically_normal().string();
    }
  }
  return {};
}

int wrapPixelX(int x, int width) {
  x %= width;
  return x < 0 ? x + width : x;
}

float sampleHdrBilinear(const float* pixels, int width, int height, int channels, float u, float v, int channel) {
  u -= std::floor(u);
  v = std::clamp(v, 0.0f, 1.0f);

  const float x = u * static_cast<float>(width) - 0.5f;
  const float y = v * static_cast<float>(height) - 0.5f;
  const int x0 = static_cast<int>(std::floor(x));
  const int y0 = static_cast<int>(std::floor(y));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;
  const float tx = x - static_cast<float>(x0);
  const float ty = y - static_cast<float>(y0);

  const auto value = [&](int sx, int sy) {
    sx = wrapPixelX(sx, width);
    sy = std::clamp(sy, 0, height - 1);
    return pixels[(static_cast<size_t>(sy) * width + sx) * channels + channel];
  };

  const float a = value(x0, y0) * (1.0f - tx) + value(x1, y0) * tx;
  const float b = value(x0, y1) * (1.0f - tx) + value(x1, y1) * tx;
  return a * (1.0f - ty) + b * ty;
}

glm::vec3 equirectangularDirection(float u, float v) {
  constexpr float kPi = 3.14159265358979323846f;
  const float theta = u * 2.0f * kPi - kPi;
  const float phi = v * kPi;
  const float sinPhi = std::sin(phi);
  return {
    sinPhi * std::cos(theta),
    sinPhi * std::sin(theta),
    std::cos(phi),
  };
}

std::pair<float, float> directionToEquirectangularUv(const glm::vec3& direction) {
  constexpr float kPi = 3.14159265358979323846f;
  const glm::vec3 d = glm::normalize(direction);
  float u = (std::atan2(d.y, d.x) + kPi) / (2.0f * kPi);
  const float v = std::acos(std::clamp(d.z, -1.0f, 1.0f)) / kPi;
  u -= std::floor(u);
  return {u, v};
}

glm::vec3 inverseRotate(const raisim::Mat<3, 3>& rotation, const glm::vec3& direction) {
  return {
    static_cast<float>(rotation[0] * direction.x + rotation[3] * direction.y + rotation[6] * direction.z),
    static_cast<float>(rotation[1] * direction.x + rotation[4] * direction.y + rotation[7] * direction.z),
    static_cast<float>(rotation[2] * direction.x + rotation[5] * direction.y + rotation[8] * direction.z),
  };
}

std::string rotateHdrEnvironment(
    const std::string& hdrPath,
    const raisim::Mat<3, 3>& rotation,
    const std::string& cacheName = "rayrai_pbr_texture_maps_hdr_rotated.hdr") {
  int width = 0;
  int height = 0;
  int channels = 0;
  float* pixels = stbi_loadf(hdrPath.c_str(), &width, &height, &channels, 3);
  if (!pixels || width <= 0 || height <= 0) {
    if (pixels)
      stbi_image_free(pixels);
    return hdrPath;
  }

  constexpr int kChannels = 3;
  std::vector<float> rotated(static_cast<size_t>(width) * height * kChannels);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(width);
      const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(height);
      const glm::vec3 dstDirection = equirectangularDirection(u, v);
      const auto [srcU, srcV] = directionToEquirectangularUv(inverseRotate(rotation, dstDirection));
      const size_t dst = static_cast<size_t>(y * width + x) * kChannels;
      for (int channel = 0; channel < kChannels; ++channel) {
        rotated[dst + channel] = sampleHdrBilinear(pixels, width, height, kChannels, srcU, srcV, channel);
      }
    }
  }
  stbi_image_free(pixels);

  const auto rotatedPath = std::filesystem::temp_directory_path() / cacheName;
  if (!stbi_write_hdr(rotatedPath.string().c_str(), width, height, kChannels, rotated.data())) {
    std::cerr << "Failed to write rotated HDR environment: " << rotatedPath << std::endl;
    return hdrPath;
  }
  return rotatedPath.string();
}

std::string rotateHdrEnvironment(
    const std::string& hdrPath,
    const raisim::Vec<4>& quaternionWxyz,
    const std::string& cacheName = "rayrai_pbr_texture_maps_hdr_rotated.hdr") {
  raisim::Vec<4> unitQuaternion = quaternionWxyz;
  const double norm = std::sqrt(
    unitQuaternion[0] * unitQuaternion[0] +
    unitQuaternion[1] * unitQuaternion[1] +
    unitQuaternion[2] * unitQuaternion[2] +
    unitQuaternion[3] * unitQuaternion[3]);
  if (norm > 1.0e-12) {
    for (size_t i = 0; i < 4; ++i)
      unitQuaternion[i] /= norm;
  }

  raisim::Mat<3, 3> rotation;
  raisim::quatToRotMat(unitQuaternion, rotation);
  return rotateHdrEnvironment(hdrPath, rotation, cacheName);
}

void centerImportedAssetAt(const std::shared_ptr<raisin::Visuals>& visual, const glm::vec3& target) {
  if (!visual)
    return;

  glm::vec3 center(0.0f);
  float radius = 0.0f;
  if (!visual->approximateBounds(center, radius))
    return;

  const glm::vec3 delta = target - center;
  visual->setPosition(visual->getPosition() + delta);
}

} // namespace

int main(int argc, char* argv[]) {
  std::filesystem::path screenshotPath;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--screenshot" && i + 1 < argc) {
      screenshotPath = argv[++i];
    }
  }

  ExampleApp app;
  const bool showWindow = screenshotPath.empty();
  if (!app.init("rayrai_pbr_texture_maps", 1280, 720, showWindow))
    return -1;

  auto world = std::make_shared<raisim::World>();
  world->addGround();

  auto viewer = std::make_shared<raisin::RayraiWindow>(world, 1280, 720);
  auto quality = raisin::RayraiWindow::defaultRenderQualitySettings(
    raisin::RayraiWindow::RenderQualityPreset::Ultra);
  quality.pbrExposure = 1.26f;
  quality.pbrEnvironmentIntensity = 0.28f;
  quality.pbrEnvironmentMaxLod = 5.0f;
  quality.pbrKeyLightIntensity = 1.0f;
  quality.addViewerFillLights = false;
  quality.gamma = 2.2f;
  quality.pbrToneMapping = true;
  quality.colorMode = raisin::RayraiWindow::ViewerColorMode::UnrealPreviewApprox;
  quality.bloomEnabled = true;
  quality.bloomThreshold = 1.45f;
  quality.bloomStrength = 0.08f;
  quality.screenSpaceAoEnabled = true;
  quality.screenSpaceAoRadius = 4.0f;
  quality.screenSpaceAoStrength = 0.42f;
  quality.mainLightAmbient = glm::vec3(0.34f);
  quality.mainLightDiffuse = glm::vec3(1.55f);
  quality.mainLightSpecular = glm::vec3(0.02f);
  quality.mainLightDirection = glm::normalize(glm::vec3(-0.35f, -0.55f, -0.78f));
  quality.reflectiveGround = false;
  quality.shadowStrength = 0.22f;
  quality.depthOfFieldEnabled = false;
  quality.shadowOrthoHalfSize = 7.0f;
  quality.shadowCenterOffset = 3.0f;
  viewer->setRenderQualitySettings(quality);
  raisim_examples::setRayraiBackgroundColorRgb255(*viewer, {38, 42, 52, 255});
  viewer->setFogDensity(0.0f);
  viewer->setShadowOrtho(7.0f, 0.1f, 30.0f);
  viewer->setShadowCenterOffset(3.0f);

  raisin::RayraiWindow::AdditionalLight warmFill;
  warmFill.type = raisin::LightType::POINT;
  warmFill.position = glm::vec3(2.4f, -3.0f, 3.2f);
  warmFill.diffuse = glm::vec3(1.35f, 1.12f, 0.90f);
  warmFill.specular = glm::vec3(0.0f);
  warmFill.linear = 0.09f;
  warmFill.quadratic = 0.032f;
  viewer->addAdditionalLight(warmFill);

  unsigned int environment = 0;
  unsigned int irradiance = 0;
  unsigned int prefiltered = 0;
  unsigned int brdf = 0;
  const std::string hdrPath =
    optionalRayraiRscPath(argv[0], "rayrai/hdr/polyhaven/potsdamer_platz_1k.hdr");
  if (std::filesystem::exists(hdrPath)) {
    environment = raisin::RayraiWindow::loadHdrEquirectangularCubemap(hdrPath.c_str(), 128, true);
    irradiance = raisin::RayraiWindow::createHdrIrradianceCubemap(hdrPath.c_str(), 32, 64);
    prefiltered =
      raisin::RayraiWindow::createHdrPrefilteredEnvironmentCubemap(hdrPath.c_str(), 128, 6, 128);
    brdf = raisin::RayraiWindow::createSplitSumBrdfLut(128, 128);
  }

  struct PbrAsset {
    std::string name;
    std::string path;
    double scale;
    double x;
    double y;
    double z;
    double yaw;
  };
  const auto setUprightYaw = [](const std::shared_ptr<raisin::Visuals>& visual, double yaw) {
    constexpr double kSqrtHalf = 0.7071067811865476;
    const double halfYaw = 0.5 * yaw;
    const double cy = std::cos(halfYaw);
    const double sy = std::sin(halfYaw);
    visual->setOrientation(cy * kSqrtHalf, cy * kSqrtHalf, sy * kSqrtHalf, sy * kSqrtHalf);
  };

  // Khronos glTF Sample Assets. Together these exercise base color, normal,
  // emissive, metallic-roughness, roughness-only, and occlusion texture import.
  const std::vector<PbrAsset> assets = {
    {"flight_helmet", "rayrai/pbr/FlightHelmet/glTF/FlightHelmet.gltf",
      1.0, -2.10, 0.00, 1.65, 0.10},
    {"damaged_helmet", "rayrai/pbr/DamagedHelmet/glTF/DamagedHelmet.gltf",
      0.48, -0.70, 0.00, 1.62, -0.05},
    {"sci_fi_helmet", "rayrai/pbr/SciFiHelmet/glTF/SciFiHelmet.gltf",
      0.42, 0.70, 0.00, 1.62, 0.10},
    {"antique_camera", "rayrai/pbr/AntiqueCamera/glTF/AntiqueCamera.gltf",
      0.12, 2.10, 0.00, 1.54, -0.15},
    {"lantern", "rayrai/pbr/Lantern/glTF/Lantern.gltf",
      0.060, -2.15, 0.00, -0.05, 0.12},
    {"boombox", "rayrai/pbr/BoomBox/glTF/BoomBox.gltf",
      46.0, -0.70, 0.00, 0.68, -0.10},
    {"avocado", "rayrai/pbr/Avocado/glTF/Avocado.gltf",
      15.0, 0.72, 0.00, 0.07, 0.05},
    {"water_bottle", "rayrai/pbr/WaterBottle/glTF/WaterBottle.gltf",
      5.6, 2.10, 0.00, 0.78, -0.06},
  };

  std::vector<std::shared_ptr<raisin::Visuals>> visuals;
  visuals.reserve(assets.size());
  for (const auto& asset : assets) {
    const std::string meshPath = rayraiRscPath(argv[0], asset.path);
    auto visual = viewer->addVisualMesh(asset.name, meshPath,
      asset.scale, asset.scale, asset.scale, 1.0f, 1.0f, 1.0f, 1.0f);
    visual->setPosition(asset.x, asset.y, asset.z);
    setUprightYaw(visual, asset.yaw);
    visual->setTwoSided(true);
    visual->setDetectable(true);
    if (environment != 0) {
      visual->setPbrEnvironment(environment, irradiance, prefiltered, brdf, 1.0f);
    }
    visuals.push_back(visual);
  }

  bool layoutApplied = false;
  const std::array<glm::vec3, 8> previewCenters = {{
    {-2.85f, 0.02f, 2.10f}, {-0.95f, 0.02f, 2.10f},
    { 0.95f, 0.02f, 2.10f}, { 2.85f, 0.02f, 2.10f},
    {-2.85f, -0.02f, 0.70f}, {-0.95f, -0.02f, 0.70f},
    { 0.95f, -0.02f, 0.70f}, { 2.85f, -0.02f, 0.70f},
  }};
  const std::array<float, 8> targetRadii = {{
    0.86f, 0.86f, 0.86f, 0.84f,
    0.80f, 0.86f, 0.82f, 0.86f,
  }};
  const auto applyAssetLayout = [&]() {
    if (layoutApplied || viewer->pendingAsyncMeshLoadCount() != 0)
      return;

    for (size_t i = 0; i < visuals.size(); ++i) {
      auto& visual = visuals[i];
      glm::vec3 center(0.0f);
      float radius = 0.0f;
      if (visual && visual->approximateBounds(center, radius) && radius > 1.0e-5f) {
        const float normalizedScale =
          static_cast<float>(assets[i].scale) * targetRadii[i] / radius;
        visual->setMeshScale(normalizedScale, normalizedScale, normalizedScale);
      }
      centerImportedAssetAt(visual, previewCenters[i]);
    }
    layoutApplied = true;
  };

  auto& camera = viewer->getCamera();
  camera.target = {0.0f, 0.00f, 1.22f};
  camera.position = {0.05f, -5.55f, 1.58f};
  camera.yaw = 89.5f;
  camera.pitch = -3.7f;
  camera.farPlane = camera.zFar = 5000.0f;
  camera.setCameraFixedTarget(true);
  camera.setCameraFixedDistance(true);

  auto renderOneFrame = [&]() {
    world->integrate();
    viewer->pollAsyncMeshLoads(8);
    applyAssetLayout();

    const double t = world->getWorldTime();
    for (size_t i = 0; i < visuals.size(); ++i) {
      setUprightYaw(visuals[i], assets[i].yaw + 0.18 * t);
    }

    app.beginFrame();
    app.renderViewer(*viewer);
    if (environment != 0) {
      raisin::RayraiWindow::RenderOverrides overrides;
      overrides.environmentBackgroundMap = environment;
      overrides.environmentBackgroundExposure = 0.55f;
      viewer->renderWithExternalCamera(viewer->getCamera(), overrides);
    }
    app.endFrame();
  };

  if (!screenshotPath.empty()) {
    while (viewer->pendingAsyncMeshLoadCount() != 0) {
      viewer->pollAsyncMeshLoads(std::numeric_limits<size_t>::max());
    }
    applyAssetLayout();
    for (int i = 0; i < 3 && !app.quit; ++i) {
      renderOneFrame();
    }
    saveFinalTexturePng(camera, screenshotPath);
    std::cout << "Saved screenshot: " << screenshotPath.string() << "\n";
    viewer.reset();
    app.shutdown();
    return 0;
  }

  while (!app.quit) {
    app.processEvents();
    if (app.quit)
      break;

    renderOneFrame();
  }

  viewer.reset();
  app.shutdown();
  return 0;
}
