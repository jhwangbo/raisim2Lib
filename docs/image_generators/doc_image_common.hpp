// Shared scaffolding for the rayrai documentation image generators.
//
// Each generator runs headless, builds a small scene, and writes one or more
// PNGs into the configured output directory. The output path is taken from
// the DOC_IMAGE_OUTPUT_DIR compile definition or the first command-line
// argument.

#pragma once

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <SDL.h>

#include <raisim/World.hpp>
#include <rayrai/Camera.hpp>
#include <rayrai/RayraiWindow.hpp>

namespace doc_image {

// Exit the process without running global destructors. rayrai keeps a global
// asset cache that holds GL handles which can crash on teardown after the
// offscreen GL context is gone. Generators are short-lived; the OS reclaims
// resources cleanly.
[[noreturn]] inline void finishAndExit(int exitCode) {
  std::fflush(stdout);
  std::fflush(stderr);
  std::_Exit(exitCode);
}

inline std::filesystem::path resolveOutputDir(int argc, char** argv) {
  if (argc >= 2) {
    return std::filesystem::path(argv[1]);
  }
#ifdef DOC_IMAGE_OUTPUT_DIR
  return std::filesystem::path(DOC_IMAGE_OUTPUT_DIR);
#else
  return std::filesystem::current_path();
#endif
}

inline void flipRowsRgba(std::vector<unsigned char>& rgba, int width, int height) {
  const size_t stride = static_cast<size_t>(width) * 4u;
  std::vector<unsigned char> row(stride);
  for (int y = 0; y < height / 2; ++y) {
    auto* top = rgba.data() + static_cast<size_t>(y) * stride;
    auto* bottom = rgba.data() + static_cast<size_t>(height - 1 - y) * stride;
    std::copy(top, top + stride, row.data());
    std::copy(bottom, bottom + stride, top);
    std::copy(row.data(), row.data() + stride, bottom);
  }
}

inline bool writePng(const std::filesystem::path& path, int width, int height,
                     std::vector<unsigned char> rgba, bool flipVertical = true) {
  if (flipVertical) flipRowsRgba(rgba, width, height);
  std::filesystem::create_directories(path.parent_path());
  const int ok = stbi_write_png(path.string().c_str(), width, height, 4,
                                rgba.data(), width * 4);
  if (!ok) {
    std::fprintf(stderr, "doc_image: failed to write %s\n", path.string().c_str());
  }
  return ok != 0;
}

// Owns the offscreen SDL window + GL context.
//
// We intentionally do not clean up the SDL/GL handles on destruction. Calling
// SDL_GL_DeleteContext / SDL_DestroyWindow / SDL_Quit while rayrai globals
// (notably the shared global asset cache) still hold GL resources can crash
// during process teardown. The handles are short-lived process resources;
// the OS reclaims them on exit. Skipping this teardown keeps the generators
// crash-free even when CMake re-runs them in chained sequence.
class OffscreenContext {
 public:
  OffscreenContext() = default;
  ~OffscreenContext() = default;
  OffscreenContext(const OffscreenContext&) = delete;
  OffscreenContext& operator=(const OffscreenContext&) = delete;

  bool init(const char* tag) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
      std::fprintf(stderr, "doc_image: SDL_Init failed: %s\n", SDL_GetError());
      return false;
    }
    raisin::RayraiWindow::createOffscreenGlContext(window_, context_, tag);
    if (!window_ || !context_) {
      std::fprintf(stderr, "doc_image: createOffscreenGlContext failed\n");
      return false;
    }
    raisin::RayraiWindow::makeOffscreenContextCurrent(window_, context_);
    return true;
  }

 private:
  SDL_Window* window_ = nullptr;
  SDL_GLContext context_ = nullptr;
};

// Bake one frame into the renderer and write a supersampled PNG.
inline bool captureScene(raisin::RayraiWindow& renderer, int width, int height,
                         const std::filesystem::path& outputPath,
                         int supersampleScale = 2,
                         const raisin::RayraiWindow::RenderOverrides& overrides = {}) {
  // Match camera aspect to the requested output so non-16:9 images frame
  // their contents correctly.
  renderer.getCamera().aspect =
      static_cast<float>(width) / static_cast<float>(std::max(1, height));
  // Bake the procedural sky into an IBL environment cubemap + irradiance
  // map so PBR surfaces actually pick up the sky's ambient colour instead
  // of falling to mainLightAmbient only. The procedural sky is only a
  // background by default; this call materialises it as a real environment
  // sampleable by the PBR shader.
  renderer.generateWeatherSkyEnvironment(/*envFaceSize=*/128,
                                         /*irradianceFaceSize=*/32,
                                         /*setAsBackground=*/true);
  // Warm the frame loop so deferred resources (shadow maps, IBL probes,
  // particle systems) are allocated and stabilized before the capture.
  for (int i = 0; i < 3; ++i) {
    renderer.update(width, height, /*save=*/false, /*saveW=*/0, /*saveH=*/0,
                    /*headless=*/true);
  }
  auto capture = renderer.captureSupersampledRgba(
      renderer.getCamera(), supersampleScale, overrides);
  if (capture.rgba.empty()) {
    std::fprintf(stderr, "doc_image: captureSupersampledRgba returned empty buffer\n");
    return false;
  }
  // captureSupersampledRgba emits rows in top-left order already, so no flip.
  return writePng(outputPath, capture.width, capture.height, std::move(capture.rgba),
                  /*flipVertical=*/false);
}

// Common scene options applied to every doc image. Adds a procedural sky as
// the background and, for the High and Ultra presets, switches the ground to
// (Reflective ground is on for High and Ultra by default in the rayrai
// library itself, so the helper does not touch it.)
inline void applyCommonSceneOptions(
    raisin::RenderQualitySettings& quality,
    raisin::RayraiWindow::RenderQualityPreset preset) {
  // Always enable the analytic procedural sky so scenes have a real sky
  // instead of a flat colour.
  quality.proceduralSkyBackgroundEnabled = true;
  quality.proceduralSkySunStrength = 0.6f;
  quality.proceduralSkySunSize = 0.018f;
  // Subtle procedural cloud layer for visual interest in the upper hemisphere.
  quality.proceduralCloudLayerEnabled = true;
  quality.proceduralCloudCoverage = 0.35f;
  quality.proceduralCloudDensity = 0.55f;
  quality.proceduralCloudScale = 0.20f;

  const bool highOrUltra =
      preset == raisin::RayraiWindow::RenderQualityPreset::High ||
      preset == raisin::RayraiWindow::RenderQualityPreset::Ultra;
  if (highOrUltra) {
    // High/Ultra get crisper contact shadows on top of the main shadow map.
    quality.contactShadowsEnabled = true;
    quality.contactShadowsLength = 0.14f;
    quality.contactShadowsStrength = 0.6f;
  }
}

// Position the camera at `eye`, look at `target`, keep Z up, and refresh the
// yaw/pitch state so subsequent renders use the right basis (rayrai's camera
// computes front/right/up from yaw and pitch on `update()`).
inline void setCameraLookAt(raisin::Camera& cam,
                            const glm::vec3& eye,
                            const glm::vec3& target,
                            float horizontalFovDeg = 60.0f) {
  cam.position = eye;
  cam.target = target;
  cam.front = glm::normalize(target - eye);
  cam.worldUp = glm::vec3(0.0f, 0.0f, 1.0f);
  cam.up = cam.worldUp;
  cam.yaw = glm::degrees(std::atan2(cam.front.y, cam.front.x));
  cam.pitch = glm::degrees(std::asin(std::clamp(cam.front.z, -1.0f, 1.0f)));
  cam.zoom = horizontalFovDeg;
  cam.horizontalFovRad = glm::radians(horizontalFovDeg);
  cam.nearPlane = cam.zNear = 0.05f;
  cam.farPlane = cam.zFar = 80.0f;
  cam.update(/*processKeyboard=*/false);
}

// Standard composition: ground + a few primitives + one articulated robot if
// available. Adds modest visuals so light, shadow, and material changes are
// all visible in the same frame.
struct StarterSceneOptions {
  bool addRobot = true;
  bool addBoxes = true;
  bool addSpheres = true;
  bool addWalls = true;
  std::string urdfPath;  // Optional override; if empty, no robot is loaded.
};

inline void buildStarterScene(raisim::World& world,
                              raisin::RayraiWindow& renderer,
                              const StarterSceneOptions& options = {}) {
  world.addGround();

  if (options.addBoxes) {
    auto* boxA = world.addBox(0.4, 0.4, 0.4, 1.0);
    boxA->setPosition(-0.8, 0.0, 0.2);
    boxA->setBodyType(raisim::BodyType::STATIC);
    boxA->setAppearance("0.86,0.34,0.18,1");

    auto* boxB = world.addBox(0.3, 0.3, 0.6, 1.0);
    boxB->setPosition(0.6, -0.5, 0.3);
    boxB->setBodyType(raisim::BodyType::STATIC);
    boxB->setAppearance("0.18,0.46,0.74,1");
  }
  if (options.addSpheres) {
    auto* sphere = world.addSphere(0.25, 1.0);
    sphere->setPosition(0.0, 0.7, 0.25);
    sphere->setBodyType(raisim::BodyType::STATIC);
    sphere->setAppearance("0.92,0.86,0.34,1");
  }
  if (options.addWalls) {
    auto* wall = world.addBox(3.0, 0.05, 1.2, 1.0);
    wall->setPosition(0.0, -1.5, 0.6);
    wall->setBodyType(raisim::BodyType::STATIC);
    wall->setAppearance("0.66,0.66,0.68,1");
  }
  if (options.addRobot && !options.urdfPath.empty()) {
    try {
      world.addArticulatedSystem(options.urdfPath);
    } catch (...) {
      // Robot is optional; ignore failures so the generator still produces output.
    }
  }

  // Camera framing.
  setCameraLookAt(renderer.getCamera(),
                  glm::vec3(2.6f, 2.2f, 1.6f),
                  glm::vec3(0.0f, 0.0f, 0.35f));
}

}  // namespace doc_image
