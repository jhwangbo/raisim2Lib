// Copyright (c) 2025 Raion Robotics Inc.
// All rights reserved.

#include <SDL.h>

#include <glbinding/glbinding.h>
#include <glbinding/gl/gl.h>
#include <imgui/imgui.h>

#include "imgui/backend/imgui_impl_opengl3.h"
#include "imgui/backend/imgui_impl_sdl2.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <glm/glm.hpp>

#include "rayrai/RayraiWindow.hpp"
#include "rayrai_example_compat.hpp"
#include "rayrai/Visuals.hpp"
#include "rayrai/OpenGLMesh.hpp"
#include "rayrai/CoordinateFrame.hpp"
#include "rayrai/RaisimTcpCommon.hpp"
#include "rayrai/raisin_imgui_style.h"
#include "raisim/configure.hpp"
#include "raisim/sensors/Sensors.hpp"

#if defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace
{

// SDL installs signal handlers by default (SIGINT/SIGTERM) that may call SDL_Quit()
// from inside the handler. That can conflict with our normal shutdown sequence and
// lead to double-free/corruption. We disable SDL signal handlers and handle SIGINT
// ourselves by requesting a graceful shutdown.
std::atomic<bool> gSignalQuit{false};

void handleSignalQuit(int /*sig*/) {
  gSignalQuit.store(true, std::memory_order_relaxed);
}

constexpr int kDefaultPort = raisin::tcp_viewer::kDefaultPort;
constexpr int kConnectTimeoutMs = 2000;
constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
constexpr float kRadToDeg = 180.0f / 3.14159265358979323846f;
constexpr auto kAutoConnectInterval = std::chrono::seconds(3);
constexpr float kBaseFontSize = 24.0f;
constexpr float kFontScale = 0.75f;
constexpr float kUiScaleEpsilon = 0.01f;
constexpr const char* kRobotoFontRelativePath = "rsc/fonts/roboto/Roboto-Medium.ttf";

using raisin::tcp_viewer::BufferReader;
using raisin::tcp_viewer::ObjectListItem;
using raisin::tcp_viewer::PendingSensorUpdate;
using raisin::tcp_viewer::RemoteScene;
using raisin::tcp_viewer::SelectedObjectInfo;
using raisin::tcp_viewer::sendSensorUpdate;
using raisin::tcp_viewer::sendUpdateRequest;
using raisin::tcp_viewer::TcpClient;
using raisin::tcp_viewer::VisualEntry;

struct ConnectionEntry {
  std::string host;
  int port = kDefaultPort;
};

struct ViewerSettings {
  int renderQuality = 1;
  glm::vec4 backgroundColorRgb255{20.0f, 20.0f, 30.0f, 255.0f};
  glm::vec3 mainLightAmbient{0.42f, 0.42f, 0.42f};
  glm::vec3 mainLightDiffuse{1.0f, 1.0f, 1.0f};
  glm::vec3 mainLightSpecular{0.22f, 0.22f, 0.22f};
  float cameraSpeed = 5.0f;
  float cameraFovDeg = 45.0f;
  float cameraNear = 0.01f;
  float cameraFar = 1000.0f;
  float lightYawDeg = 0.0f;
  float lightPitchDeg = -30.0f;
  float lightStrength = 1.0f;
  float ambientStrength = 1.0f;
  bool shadowsEnabled = true;
  int shadowResolution = 2048;
  float shadowBias = 0.0008f;
  float shadowStrength = 0.6f;
  float shadowPcfRadius = 1.25f;
  float shadowOrthoHalfSize = 12.5f;
  float shadowNear = 0.1f;
  float shadowFar = 55.0f;
  float shadowCenterOffset = 10.0f;
  float fogDensity = 0.01f;
  float gamma = 1.0f;
  int colorMode = static_cast<int>(raisin::RayraiWindow::ViewerColorMode::FastLinear);
  bool fxaaEnabled = false;
  bool bloomEnabled = false;
  float bloomThreshold = 0.82f;
  float bloomStrength = 0.18f;
  float bloomRadius = 4.0f;
  float bloomKnee = 0.22f;
  int bloomQuality = 1;
  bool screenSpaceAoEnabled = false;
  float screenSpaceAoRadius = 2.0f;
  float screenSpaceAoStrength = 0.0f;
  float screenSpaceAoBias = 0.02f;
  bool opaqueDepthPrepass = false;
  bool depthOfFieldEnabled = false;
  float depthOfFieldFocusDistance = 1.0f;
  float depthOfFieldFocusRange = 10.0f;
  float depthOfFieldMaxRadius = 1.25f;
  bool highFidelityPbr = false;
  bool pbrToneMapping = false;
  float pbrExposure = 1.0f;
  float pbrEnvironmentMaxLod = 0.0f;
  float pbrEnvironmentIntensity = 1.0f;
  float pbrKeyLightIntensity = 1.0f;
  bool reflectiveGround = true;
  float reflectiveGroundRoughness = 0.24f;
  float reflectiveGroundMetallic = 0.0f;
  int shadowedLightBudget = 1;
  int maxPointShadowLights = 4;
  float additionalShadowResolutionScale = 0.5f;
  float pointShadowResolutionScale = 0.5f;
  int minAdditionalShadowResolution = 256;
  bool updateShadowsEveryFrame = true;
  int maxAdditionalLightsPerFrame = 8;
  float minAdditionalLightInfluence = 0.0f;
  bool autoSelectImportedShadowLight = false;
  bool sortTransparentInstances = false;
  bool addViewerFillLights = true;
  float uiScale = 1.0f;
  bool uiScaleUserSet = false;
};

std::string shortenPathLabel(const std::string& value, size_t maxLen) {
  if (value.size() <= maxLen) {
    return value;
  }
  if (maxLen <= 3) {
    return value.substr(0, maxLen);
  }
  return value.substr(0, maxLen - 3) + "...";
}

std::string formatConnectionLabel(const ConnectionEntry& entry) {
  if (entry.host.empty()) {
    return {};
  }
  return entry.host + ":" + std::to_string(entry.port);
}

std::string toLowerAscii(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

std::string trimAscii(const std::string& value) {
  const auto first = std::find_if_not(value.begin(), value.end(),
    [](unsigned char c) { return std::isspace(c); });
  const auto last = std::find_if_not(value.rbegin(), value.rend(),
    [](unsigned char c) { return std::isspace(c); }).base();
  if (first >= last) {
    return {};
  }
  return std::string(first, last);
}

bool readEnvBool(const char* name, bool defaultValue) {
  const char* rawValue = std::getenv(name);
  if (!rawValue || !*rawValue) {
    return defaultValue;
  }
  const std::string value = toLowerAscii(rawValue);
  if (value == "0" || value == "false" || value == "off" || value == "no") {
    return false;
  }
  if (value == "1" || value == "true" || value == "on" || value == "yes") {
    return true;
  }
  return true;
}

bool parseBoolValue(const std::string& rawValue, bool fallback) {
  const std::string value = toLowerAscii(trimAscii(rawValue));
  if (value == "true" || value == "yes" || value == "on" || value == "1") {
    return true;
  }
  if (value == "false" || value == "no" || value == "off" || value == "0") {
    return false;
  }
  return fallback;
}

float parseFloatValue(const std::string& rawValue, float fallback) {
  char* end = nullptr;
  const float parsed = std::strtof(rawValue.c_str(), &end);
  return end != rawValue.c_str() ? parsed : fallback;
}

int parseIntValue(const std::string& rawValue, int fallback) {
  char* end = nullptr;
  const long parsed = std::strtol(rawValue.c_str(), &end, 10);
  return end != rawValue.c_str() ? static_cast<int>(parsed) : fallback;
}

glm::vec3 parseVec3Value(const std::string& rawValue, const glm::vec3& fallback) {
  glm::vec3 parsed = fallback;
  char separator0 = '\0';
  char separator1 = '\0';
  if (std::sscanf(rawValue.c_str(), " %f %c %f %c %f ",
        &parsed.x, &separator0, &parsed.y, &separator1, &parsed.z) == 5) {
    return parsed;
  }
  return fallback;
}

glm::vec4 parseVec4Value(const std::string& rawValue, const glm::vec4& fallback) {
  glm::vec4 parsed = fallback;
  char separator0 = '\0';
  char separator1 = '\0';
  char separator2 = '\0';
  if (std::sscanf(rawValue.c_str(), " %f %c %f %c %f %c %f ",
        &parsed.x, &separator0, &parsed.y, &separator1, &parsed.z, &separator2, &parsed.w) == 7) {
    return parsed;
  }
  return fallback;
}

const char* qualityName(int quality) {
  static constexpr const char* kNames[] = {"Fast", "Balanced", "High", "Ultra", "Custom"};
  return kNames[std::clamp(quality, 0, 4)];
}

const char* colorModeName(int colorMode) {
  static constexpr const char* kNames[] = {"Fast Linear", "ACES Approx", "Unreal Preview"};
  return kNames[std::clamp(colorMode, 0, 2)];
}

int qualityIndexFromName(const std::string& rawValue, int fallback) {
  const std::string value = toLowerAscii(trimAscii(rawValue));
  if (value == "fast" || value == "0") return 0;
  if (value == "balanced" || value == "balance" || value == "1") return 1;
  if (value == "high" || value == "2") return 2;
  if (value == "ultra" || value == "3") return 3;
  if (value == "custom" || value == "4") return 4;
  return fallback;
}

int colorModeIndexFromName(const std::string& rawValue, int fallback) {
  const std::string value = toLowerAscii(trimAscii(rawValue));
  if (value == "fast_linear" || value == "fast linear" || value == "linear" || value == "0") return 0;
  if (value == "aces_approx" || value == "aces approx" || value == "aces" || value == "1") return 1;
  if (value == "unreal_preview" || value == "unreal preview" || value == "unreal" || value == "2") return 2;
  return fallback;
}

raisin::RayraiWindow::RenderQualityPreset qualityPresetFromIndex(int quality) {
  switch (std::clamp(quality, 0, 4)) {
    case 0: return raisin::RayraiWindow::RenderQualityPreset::Fast;
    case 2: return raisin::RayraiWindow::RenderQualityPreset::High;
    case 3: return raisin::RayraiWindow::RenderQualityPreset::Ultra;
    case 4: return raisin::RayraiWindow::RenderQualityPreset::Custom;
    case 1:
    default: return raisin::RayraiWindow::RenderQualityPreset::Balanced;
  }
}

std::filesystem::path settingsFilePath() {
  const char* home = std::getenv("HOME");
  if (home && *home) {
    return std::filesystem::path(home) / ".rayrai" / "settings.yaml";
  }
  return std::filesystem::current_path() / ".rayrai" / "settings.yaml";
}

void loadViewerSettings(ViewerSettings& settings) {
  std::ifstream input(settingsFilePath());
  if (!input) {
    return;
  }

  std::string line;
  while (std::getline(input, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line.resize(comment);
    }
    const auto sep = line.find(':');
    if (sep == std::string::npos) {
      continue;
    }
    const std::string key = trimAscii(line.substr(0, sep));
    const std::string value = trimAscii(line.substr(sep + 1));
    if (key == "render_quality") settings.renderQuality = qualityIndexFromName(value, settings.renderQuality);
    else if (key == "background_color_rgb255") settings.backgroundColorRgb255 = parseVec4Value(value, settings.backgroundColorRgb255);
    else if (key == "main_light_ambient") settings.mainLightAmbient = parseVec3Value(value, settings.mainLightAmbient);
    else if (key == "main_light_diffuse") settings.mainLightDiffuse = parseVec3Value(value, settings.mainLightDiffuse);
    else if (key == "main_light_specular") settings.mainLightSpecular = parseVec3Value(value, settings.mainLightSpecular);
    else if (key == "camera_speed") settings.cameraSpeed = parseFloatValue(value, settings.cameraSpeed);
    else if (key == "camera_fov_deg") settings.cameraFovDeg = parseFloatValue(value, settings.cameraFovDeg);
    else if (key == "camera_near") settings.cameraNear = parseFloatValue(value, settings.cameraNear);
    else if (key == "camera_far") settings.cameraFar = parseFloatValue(value, settings.cameraFar);
    else if (key == "light_yaw_deg") settings.lightYawDeg = parseFloatValue(value, settings.lightYawDeg);
    else if (key == "light_pitch_deg") settings.lightPitchDeg = parseFloatValue(value, settings.lightPitchDeg);
    else if (key == "light_strength") settings.lightStrength = parseFloatValue(value, settings.lightStrength);
    else if (key == "ambient_strength") settings.ambientStrength = parseFloatValue(value, settings.ambientStrength);
    else if (key == "shadows_enabled") settings.shadowsEnabled = parseBoolValue(value, settings.shadowsEnabled);
    else if (key == "shadow_resolution") settings.shadowResolution = parseIntValue(value, settings.shadowResolution);
    else if (key == "shadow_bias") settings.shadowBias = parseFloatValue(value, settings.shadowBias);
    else if (key == "shadow_strength") settings.shadowStrength = parseFloatValue(value, settings.shadowStrength);
    else if (key == "shadow_pcf_radius") settings.shadowPcfRadius = parseFloatValue(value, settings.shadowPcfRadius);
    else if (key == "shadow_ortho_half_size") settings.shadowOrthoHalfSize = parseFloatValue(value, settings.shadowOrthoHalfSize);
    else if (key == "shadow_near") settings.shadowNear = parseFloatValue(value, settings.shadowNear);
    else if (key == "shadow_far") settings.shadowFar = parseFloatValue(value, settings.shadowFar);
    else if (key == "shadow_center_offset") settings.shadowCenterOffset = parseFloatValue(value, settings.shadowCenterOffset);
    else if (key == "fog_density") settings.fogDensity = parseFloatValue(value, settings.fogDensity);
    else if (key == "gamma") settings.gamma = parseFloatValue(value, settings.gamma);
    else if (key == "color_mode") settings.colorMode = colorModeIndexFromName(value, settings.colorMode);
    else if (key == "fxaa_enabled") settings.fxaaEnabled = parseBoolValue(value, settings.fxaaEnabled);
    else if (key == "bloom_enabled") settings.bloomEnabled = parseBoolValue(value, settings.bloomEnabled);
    else if (key == "bloom_threshold") settings.bloomThreshold = parseFloatValue(value, settings.bloomThreshold);
    else if (key == "bloom_strength") settings.bloomStrength = parseFloatValue(value, settings.bloomStrength);
    else if (key == "bloom_radius") settings.bloomRadius = parseFloatValue(value, settings.bloomRadius);
    else if (key == "bloom_knee") settings.bloomKnee = parseFloatValue(value, settings.bloomKnee);
    else if (key == "bloom_quality") settings.bloomQuality = parseIntValue(value, settings.bloomQuality);
    else if (key == "screen_space_ao_enabled") settings.screenSpaceAoEnabled = parseBoolValue(value, settings.screenSpaceAoEnabled);
    else if (key == "screen_space_ao_radius") settings.screenSpaceAoRadius = parseFloatValue(value, settings.screenSpaceAoRadius);
    else if (key == "screen_space_ao_strength") settings.screenSpaceAoStrength = parseFloatValue(value, settings.screenSpaceAoStrength);
    else if (key == "screen_space_ao_bias") settings.screenSpaceAoBias = parseFloatValue(value, settings.screenSpaceAoBias);
    else if (key == "opaque_depth_prepass") settings.opaqueDepthPrepass = parseBoolValue(value, settings.opaqueDepthPrepass);
    else if (key == "depth_of_field_enabled") settings.depthOfFieldEnabled = parseBoolValue(value, settings.depthOfFieldEnabled);
    else if (key == "depth_of_field_focus_distance") settings.depthOfFieldFocusDistance = parseFloatValue(value, settings.depthOfFieldFocusDistance);
    else if (key == "depth_of_field_focus_range") settings.depthOfFieldFocusRange = parseFloatValue(value, settings.depthOfFieldFocusRange);
    else if (key == "depth_of_field_max_radius") settings.depthOfFieldMaxRadius = parseFloatValue(value, settings.depthOfFieldMaxRadius);
    else if (key == "high_fidelity_pbr") settings.highFidelityPbr = parseBoolValue(value, settings.highFidelityPbr);
    else if (key == "pbr_tone_mapping") settings.pbrToneMapping = parseBoolValue(value, settings.pbrToneMapping);
    else if (key == "pbr_exposure") settings.pbrExposure = parseFloatValue(value, settings.pbrExposure);
    else if (key == "pbr_environment_max_lod") settings.pbrEnvironmentMaxLod = parseFloatValue(value, settings.pbrEnvironmentMaxLod);
    else if (key == "pbr_environment_intensity") settings.pbrEnvironmentIntensity = parseFloatValue(value, settings.pbrEnvironmentIntensity);
    else if (key == "pbr_key_light_intensity") settings.pbrKeyLightIntensity = parseFloatValue(value, settings.pbrKeyLightIntensity);
    else if (key == "reflective_ground") settings.reflectiveGround = parseBoolValue(value, settings.reflectiveGround);
    else if (key == "reflective_ground_roughness") settings.reflectiveGroundRoughness = parseFloatValue(value, settings.reflectiveGroundRoughness);
    else if (key == "reflective_ground_metallic") settings.reflectiveGroundMetallic = parseFloatValue(value, settings.reflectiveGroundMetallic);
    else if (key == "shadowed_light_budget") settings.shadowedLightBudget = parseIntValue(value, settings.shadowedLightBudget);
    else if (key == "max_point_shadow_lights") settings.maxPointShadowLights = parseIntValue(value, settings.maxPointShadowLights);
    else if (key == "additional_shadow_resolution_scale") settings.additionalShadowResolutionScale = parseFloatValue(value, settings.additionalShadowResolutionScale);
    else if (key == "point_shadow_resolution_scale") settings.pointShadowResolutionScale = parseFloatValue(value, settings.pointShadowResolutionScale);
    else if (key == "min_additional_shadow_resolution") settings.minAdditionalShadowResolution = parseIntValue(value, settings.minAdditionalShadowResolution);
    else if (key == "update_shadows_every_frame") settings.updateShadowsEveryFrame = parseBoolValue(value, settings.updateShadowsEveryFrame);
    else if (key == "max_additional_lights_per_frame") settings.maxAdditionalLightsPerFrame = parseIntValue(value, settings.maxAdditionalLightsPerFrame);
    else if (key == "min_additional_light_influence") settings.minAdditionalLightInfluence = parseFloatValue(value, settings.minAdditionalLightInfluence);
    else if (key == "auto_select_imported_shadow_light") settings.autoSelectImportedShadowLight = parseBoolValue(value, settings.autoSelectImportedShadowLight);
    else if (key == "sort_transparent_instances") settings.sortTransparentInstances = parseBoolValue(value, settings.sortTransparentInstances);
    else if (key == "add_viewer_fill_lights") settings.addViewerFillLights = parseBoolValue(value, settings.addViewerFillLights);
    else if (key == "ui_scale") settings.uiScale = parseFloatValue(value, settings.uiScale);
    else if (key == "ui_scale_user_set") settings.uiScaleUserSet = parseBoolValue(value, settings.uiScaleUserSet);
  }
}

void saveViewerSettings(const ViewerSettings& settings) {
  const std::filesystem::path path = settingsFilePath();
  std::error_code ec;
  std::filesystem::create_directories(path.parent_path(), ec);
  std::ofstream output(path);
  if (!output) {
    std::cerr << "WARN: Failed to write " << path << "\n";
    return;
  }

  output << "# rayrai TCP viewer settings\n";
  output << std::boolalpha << std::setprecision(6);
  output << "render_quality: " << qualityName(settings.renderQuality) << "\n";
  output << "background_color_rgb255: " << settings.backgroundColorRgb255.r << ", "
         << settings.backgroundColorRgb255.g << ", " << settings.backgroundColorRgb255.b << ", "
         << settings.backgroundColorRgb255.a << "\n";
  output << "main_light_ambient: " << settings.mainLightAmbient.r << ", "
         << settings.mainLightAmbient.g << ", " << settings.mainLightAmbient.b << "\n";
  output << "main_light_diffuse: " << settings.mainLightDiffuse.r << ", "
         << settings.mainLightDiffuse.g << ", " << settings.mainLightDiffuse.b << "\n";
  output << "main_light_specular: " << settings.mainLightSpecular.r << ", "
         << settings.mainLightSpecular.g << ", " << settings.mainLightSpecular.b << "\n";
  output << "camera_speed: " << settings.cameraSpeed << "\n";
  output << "camera_fov_deg: " << settings.cameraFovDeg << "\n";
  output << "camera_near: " << settings.cameraNear << "\n";
  output << "camera_far: " << settings.cameraFar << "\n";
  output << "light_yaw_deg: " << settings.lightYawDeg << "\n";
  output << "light_pitch_deg: " << settings.lightPitchDeg << "\n";
  output << "light_strength: " << settings.lightStrength << "\n";
  output << "ambient_strength: " << settings.ambientStrength << "\n";
  output << "shadows_enabled: " << settings.shadowsEnabled << "\n";
  output << "shadow_resolution: " << settings.shadowResolution << "\n";
  output << "shadow_bias: " << settings.shadowBias << "\n";
  output << "shadow_strength: " << settings.shadowStrength << "\n";
  output << "shadow_pcf_radius: " << settings.shadowPcfRadius << "\n";
  output << "shadow_ortho_half_size: " << settings.shadowOrthoHalfSize << "\n";
  output << "shadow_near: " << settings.shadowNear << "\n";
  output << "shadow_far: " << settings.shadowFar << "\n";
  output << "shadow_center_offset: " << settings.shadowCenterOffset << "\n";
  output << "fog_density: " << settings.fogDensity << "\n";
  output << "gamma: " << settings.gamma << "\n";
  output << "color_mode: " << colorModeName(settings.colorMode) << "\n";
  output << "fxaa_enabled: " << settings.fxaaEnabled << "\n";
  output << "bloom_enabled: " << settings.bloomEnabled << "\n";
  output << "bloom_threshold: " << settings.bloomThreshold << "\n";
  output << "bloom_strength: " << settings.bloomStrength << "\n";
  output << "bloom_radius: " << settings.bloomRadius << "\n";
  output << "bloom_knee: " << settings.bloomKnee << "\n";
  output << "bloom_quality: " << settings.bloomQuality << "\n";
  output << "screen_space_ao_enabled: " << settings.screenSpaceAoEnabled << "\n";
  output << "screen_space_ao_radius: " << settings.screenSpaceAoRadius << "\n";
  output << "screen_space_ao_strength: " << settings.screenSpaceAoStrength << "\n";
  output << "screen_space_ao_bias: " << settings.screenSpaceAoBias << "\n";
  output << "opaque_depth_prepass: " << settings.opaqueDepthPrepass << "\n";
  output << "depth_of_field_enabled: " << settings.depthOfFieldEnabled << "\n";
  output << "depth_of_field_focus_distance: " << settings.depthOfFieldFocusDistance << "\n";
  output << "depth_of_field_focus_range: " << settings.depthOfFieldFocusRange << "\n";
  output << "depth_of_field_max_radius: " << settings.depthOfFieldMaxRadius << "\n";
  output << "high_fidelity_pbr: " << settings.highFidelityPbr << "\n";
  output << "pbr_tone_mapping: " << settings.pbrToneMapping << "\n";
  output << "pbr_exposure: " << settings.pbrExposure << "\n";
  output << "pbr_environment_max_lod: " << settings.pbrEnvironmentMaxLod << "\n";
  output << "pbr_environment_intensity: " << settings.pbrEnvironmentIntensity << "\n";
  output << "pbr_key_light_intensity: " << settings.pbrKeyLightIntensity << "\n";
  output << "reflective_ground: " << settings.reflectiveGround << "\n";
  output << "reflective_ground_roughness: " << settings.reflectiveGroundRoughness << "\n";
  output << "reflective_ground_metallic: " << settings.reflectiveGroundMetallic << "\n";
  output << "shadowed_light_budget: " << settings.shadowedLightBudget << "\n";
  output << "max_point_shadow_lights: " << settings.maxPointShadowLights << "\n";
  output << "additional_shadow_resolution_scale: " << settings.additionalShadowResolutionScale << "\n";
  output << "point_shadow_resolution_scale: " << settings.pointShadowResolutionScale << "\n";
  output << "min_additional_shadow_resolution: " << settings.minAdditionalShadowResolution << "\n";
  output << "update_shadows_every_frame: " << settings.updateShadowsEveryFrame << "\n";
  output << "max_additional_lights_per_frame: " << settings.maxAdditionalLightsPerFrame << "\n";
  output << "min_additional_light_influence: " << settings.minAdditionalLightInfluence << "\n";
  output << "auto_select_imported_shadow_light: " << settings.autoSelectImportedShadowLight << "\n";
  output << "sort_transparent_instances: " << settings.sortTransparentInstances << "\n";
  output << "add_viewer_fill_lights: " << settings.addViewerFillLights << "\n";
  output << "ui_scale: " << settings.uiScale << "\n";
  output << "ui_scale_user_set: " << settings.uiScaleUserSet << "\n";
}

std::string findRobotoFontPath(const std::filesystem::path& binaryDir) {
  if (const char* envPath = std::getenv("RAYRAI_TCP_VIEWER_FONT")) {
    if (*envPath && std::filesystem::exists(envPath)) {
      return envPath;
    }
  }

  const std::vector<std::filesystem::path> candidates = {
    binaryDir / kRobotoFontRelativePath,
    binaryDir / ".." / kRobotoFontRelativePath,
    binaryDir / ".." / ".." / kRobotoFontRelativePath,
    std::filesystem::current_path() / kRobotoFontRelativePath,
  };

  for (const auto& path : candidates) {
    if (std::filesystem::exists(path)) {
      return path.string();
    }
  }

  return {};
}

bool isContactLabel(const std::string& label) {
  if (label.empty()) {
    return false;
  }
  const std::string lower = toLowerAscii(label);
  if (lower.find("contact") == std::string::npos) {
    return false;
  }
  return lower.find("point") != std::string::npos || lower.find("force") != std::string::npos ||
         lower.find("contacts") != std::string::npos;
}

bool isContactEntry(const VisualEntry* entry) {
  if (!entry) {
    return false;
  }
  return isContactLabel(entry->name) || isContactLabel(entry->objectName) ||
         isContactLabel(entry->meshFile);
}

bool isContactItem(const ObjectListItem& item) {
  return isContactLabel(item.name);
}

void recordConnection(
  std::vector<ConnectionEntry>& connections, const std::string& host, int port) {
  if (host.empty()) {
    return;
  }
  connections.erase(
    std::remove_if(connections.begin(), connections.end(),
      [&](const ConnectionEntry& entry) { return entry.host == host && entry.port == port; }),
    connections.end());
  connections.insert(connections.begin(), ConnectionEntry{host, port});
  if (connections.size() > 8) {
    connections.resize(8);
  }
}

void recordResourceDir(std::vector<std::string>& dirs, const std::string& path) {
  if (path.empty()) {
    return;
  }
  dirs.erase(std::remove(dirs.begin(), dirs.end(), path), dirs.end());
  dirs.insert(dirs.begin(), path);
  if (dirs.size() > 24) {
    dirs.resize(24);
  }
}

glm::vec3 lightDirectionFromYawPitch(float yawDeg, float pitchDeg) {
  const float yawRad = yawDeg * kDegToRad;
  const float pitchRad = pitchDeg * kDegToRad;
  glm::vec3 dir(std::cos(pitchRad) * std::cos(yawRad), std::cos(pitchRad) * std::sin(yawRad),
    std::sin(pitchRad));
  return glm::normalize(dir);
}

void yawPitchFromDirection(const glm::vec3& dir, float& yawDeg, float& pitchDeg) {
  const glm::vec3 n = glm::normalize(dir);
  const float yaw = std::atan2(n.y, n.x);
  const float pitch = std::asin(std::clamp(n.z, -1.0f, 1.0f));
  yawDeg = yaw * kRadToDeg;
  pitchDeg = pitch * kRadToDeg;
}

bool parseVec3Env(const char* value, glm::vec3& vec) {
  if (!value || !*value) {
    return false;
  }
  float values[3]{};
  const char* cursor = value;
  for (int i = 0; i < 3; ++i) {
    char* end = nullptr;
    values[i] = std::strtof(cursor, &end);
    if (end == cursor) {
      return false;
    }
    cursor = end;
    while (*cursor == ',' || *cursor == ';' || std::isspace(static_cast<unsigned char>(*cursor))) {
      ++cursor;
    }
  }
  vec = glm::vec3(values[0], values[1], values[2]);
  return glm::length(vec) > 1e-4f;
}

bool parseCameraLookAtEnv(const char* value, glm::vec3& pos, glm::vec3& target) {
  if (!value || !*value) {
    return false;
  }
  float values[6]{};
  const char* cursor = value;
  for (int i = 0; i < 6; ++i) {
    char* end = nullptr;
    values[i] = std::strtof(cursor, &end);
    if (end == cursor) {
      return false;
    }
    cursor = end;
    while (*cursor == ',' || *cursor == ';' || std::isspace(static_cast<unsigned char>(*cursor))) {
      ++cursor;
    }
  }
  pos = glm::vec3(values[0], values[1], values[2]);
  target = glm::vec3(values[3], values[4], values[5]);
  return glm::length(target - pos) > 1e-4f;
}

void applyCameraLookAt(raisin::Camera& camera, const glm::vec3& pos, const glm::vec3& target) {
  camera.setCameraFixedTarget(false);
  camera.setCameraFixedDistance(false);
  camera.targetObject = nullptr;
  camera.position = pos;
  camera.target = target;
  camera.worldUp = glm::vec3(0.0f, 0.0f, 1.0f);
  const glm::vec3 dir = glm::normalize(target - pos);
  yawPitchFromDirection(dir, camera.yaw, camera.pitch);
  camera.front = dir;
  camera.update(false);
}

void configureViewerFillLights(raisin::RayraiWindow& viewer, bool enabled) {
  viewer.clearAdditionalLights();
  if (!enabled) {
    return;
  }

  raisin::RayraiWindow::AdditionalLight fillLight;
  fillLight.type = raisin::LightType::DIRECTIONAL;
  fillLight.direction = glm::normalize(glm::vec3(0.45f, -0.25f, -0.85f));
  fillLight.diffuse = glm::vec3(0.14f, 0.16f, 0.20f);
  fillLight.specular = glm::vec3(0.03f);
  viewer.addAdditionalLight(fillLight);

  raisin::RayraiWindow::AdditionalLight rimLight;
  rimLight.type = raisin::LightType::POINT;
  rimLight.position = glm::vec3(-2.0f, -3.0f, 3.0f);
  rimLight.diffuse = glm::vec3(0.16f, 0.10f, 0.05f);
  rimLight.specular = glm::vec3(0.04f);
  rimLight.linear = 0.08f;
  rimLight.quadratic = 0.025f;
  viewer.addAdditionalLight(rimLight);
}

void copyRenderDefaultsToSettings(ViewerSettings& settings, int quality) {
  const auto renderSettings =
    raisin::RayraiWindow::defaultRenderQualitySettings(qualityPresetFromIndex(quality));
  settings.renderQuality = std::clamp(quality, 0, 4);
  settings.backgroundColorRgb255 = renderSettings.backgroundColorRgb255;
  settings.mainLightAmbient = renderSettings.mainLightAmbient;
  settings.mainLightDiffuse = renderSettings.mainLightDiffuse;
  settings.mainLightSpecular = renderSettings.mainLightSpecular;
  settings.shadowsEnabled = renderSettings.shadowsEnabled;
  settings.shadowResolution = renderSettings.shadowResolution;
  settings.shadowBias = renderSettings.shadowBias;
  settings.shadowStrength = renderSettings.shadowStrength;
  settings.shadowPcfRadius = renderSettings.shadowPcfRadius;
  settings.shadowOrthoHalfSize = renderSettings.shadowOrthoHalfSize;
  settings.shadowNear = renderSettings.shadowNear;
  settings.shadowFar = renderSettings.shadowFar;
  settings.shadowCenterOffset = renderSettings.shadowCenterOffset;
  settings.fogDensity = renderSettings.fogDensity;
  settings.gamma = renderSettings.gamma;
  settings.colorMode = static_cast<int>(renderSettings.colorMode);
  settings.fxaaEnabled = renderSettings.fxaaEnabled;
  settings.bloomEnabled = renderSettings.bloomEnabled;
  settings.bloomThreshold = renderSettings.bloomThreshold;
  settings.bloomStrength = renderSettings.bloomStrength;
  settings.bloomRadius = renderSettings.bloomRadius;
  settings.bloomKnee = renderSettings.bloomKnee;
  settings.bloomQuality = renderSettings.bloomQuality;
  settings.screenSpaceAoEnabled = renderSettings.screenSpaceAoEnabled;
  settings.screenSpaceAoRadius = renderSettings.screenSpaceAoRadius;
  settings.screenSpaceAoStrength = renderSettings.screenSpaceAoStrength;
  settings.screenSpaceAoBias = renderSettings.screenSpaceAoBias;
  settings.opaqueDepthPrepass = renderSettings.opaqueDepthPrepass;
  settings.depthOfFieldEnabled = renderSettings.depthOfFieldEnabled;
  settings.depthOfFieldFocusDistance = renderSettings.depthOfFieldFocusDistance;
  settings.depthOfFieldFocusRange = renderSettings.depthOfFieldFocusRange;
  settings.depthOfFieldMaxRadius = renderSettings.depthOfFieldMaxRadius;
  settings.highFidelityPbr = renderSettings.highFidelityPbr;
  settings.pbrToneMapping = renderSettings.pbrToneMapping;
  settings.pbrExposure = renderSettings.pbrExposure;
  settings.pbrEnvironmentMaxLod = renderSettings.pbrEnvironmentMaxLod;
  settings.pbrEnvironmentIntensity = renderSettings.pbrEnvironmentIntensity;
  settings.pbrKeyLightIntensity = renderSettings.pbrKeyLightIntensity;
  settings.reflectiveGround = renderSettings.reflectiveGround;
  settings.reflectiveGroundRoughness = renderSettings.reflectiveGroundRoughness;
  settings.reflectiveGroundMetallic = renderSettings.reflectiveGroundMetallic;
  settings.shadowedLightBudget = renderSettings.shadowedLightBudget;
  settings.maxPointShadowLights = renderSettings.maxPointShadowLights;
  settings.additionalShadowResolutionScale = renderSettings.additionalShadowResolutionScale;
  settings.pointShadowResolutionScale = renderSettings.pointShadowResolutionScale;
  settings.minAdditionalShadowResolution = renderSettings.minAdditionalShadowResolution;
  settings.updateShadowsEveryFrame = renderSettings.updateShadowsEveryFrame;
  settings.maxAdditionalLightsPerFrame = renderSettings.maxAdditionalLightsPerFrame;
  settings.minAdditionalLightInfluence = renderSettings.minAdditionalLightInfluence;
  settings.autoSelectImportedShadowLight = renderSettings.autoSelectImportedShadowLight;
  settings.sortTransparentInstances = renderSettings.sortTransparentInstances;
  settings.addViewerFillLights = renderSettings.addViewerFillLights;
}

void applyViewerSettings(raisin::RayraiWindow& viewer, const ViewerSettings& settings) {
  auto quality = qualityPresetFromIndex(settings.renderQuality);
  auto renderSettings = raisin::RayraiWindow::defaultRenderQualitySettings(quality);
  renderSettings.backgroundColorRgb255 = settings.backgroundColorRgb255;
  renderSettings.mainLightAmbient = settings.mainLightAmbient;
  renderSettings.mainLightDiffuse = settings.mainLightDiffuse;
  renderSettings.mainLightSpecular = settings.mainLightSpecular;
  renderSettings.mainLightDirection =
    lightDirectionFromYawPitch(settings.lightYawDeg, settings.lightPitchDeg);
  renderSettings.shadowsEnabled = settings.shadowsEnabled;
  renderSettings.shadowResolution = settings.shadowResolution;
  renderSettings.shadowBias = settings.shadowBias;
  renderSettings.shadowStrength = settings.shadowStrength;
  renderSettings.shadowPcfRadius = settings.shadowPcfRadius;
  renderSettings.shadowOrthoHalfSize = settings.shadowOrthoHalfSize;
  renderSettings.shadowNear = settings.shadowNear;
  renderSettings.shadowFar = settings.shadowFar;
  renderSettings.shadowCenterOffset = settings.shadowCenterOffset;
  renderSettings.fogDensity = settings.fogDensity;
  renderSettings.gamma = settings.gamma;
  renderSettings.colorMode =
    static_cast<raisin::RayraiWindow::ViewerColorMode>(std::clamp(settings.colorMode, 0, 2));
  renderSettings.fxaaEnabled = settings.fxaaEnabled;
  renderSettings.bloomEnabled = settings.bloomEnabled;
  renderSettings.bloomThreshold = settings.bloomThreshold;
  renderSettings.bloomStrength = settings.bloomStrength;
  renderSettings.bloomRadius = settings.bloomRadius;
  renderSettings.bloomKnee = settings.bloomKnee;
  renderSettings.bloomQuality = settings.bloomQuality;
  renderSettings.screenSpaceAoEnabled = settings.screenSpaceAoEnabled;
  renderSettings.screenSpaceAoRadius = settings.screenSpaceAoRadius;
  renderSettings.screenSpaceAoStrength = settings.screenSpaceAoStrength;
  renderSettings.screenSpaceAoBias = settings.screenSpaceAoBias;
  renderSettings.opaqueDepthPrepass = settings.opaqueDepthPrepass;
  renderSettings.depthOfFieldEnabled = settings.depthOfFieldEnabled;
  renderSettings.depthOfFieldFocusDistance = settings.depthOfFieldFocusDistance;
  renderSettings.depthOfFieldFocusRange = settings.depthOfFieldFocusRange;
  renderSettings.depthOfFieldMaxRadius = settings.depthOfFieldMaxRadius;
  renderSettings.highFidelityPbr = settings.highFidelityPbr;
  renderSettings.pbrToneMapping = settings.pbrToneMapping;
  renderSettings.pbrExposure = settings.pbrExposure;
  renderSettings.pbrEnvironmentMaxLod = settings.pbrEnvironmentMaxLod;
  renderSettings.pbrEnvironmentIntensity = settings.pbrEnvironmentIntensity;
  renderSettings.pbrKeyLightIntensity = settings.pbrKeyLightIntensity;
  renderSettings.reflectiveGround = settings.reflectiveGround;
  renderSettings.reflectiveGroundRoughness = settings.reflectiveGroundRoughness;
  renderSettings.reflectiveGroundMetallic = settings.reflectiveGroundMetallic;
  renderSettings.shadowedLightBudget = settings.shadowedLightBudget;
  renderSettings.maxPointShadowLights = settings.maxPointShadowLights;
  renderSettings.additionalShadowResolutionScale = settings.additionalShadowResolutionScale;
  renderSettings.pointShadowResolutionScale = settings.pointShadowResolutionScale;
  renderSettings.minAdditionalShadowResolution = settings.minAdditionalShadowResolution;
  renderSettings.updateShadowsEveryFrame = settings.updateShadowsEveryFrame;
  renderSettings.maxAdditionalLightsPerFrame = settings.maxAdditionalLightsPerFrame;
  renderSettings.minAdditionalLightInfluence = settings.minAdditionalLightInfluence;
  renderSettings.autoSelectImportedShadowLight = settings.autoSelectImportedShadowLight;
  renderSettings.sortTransparentInstances = settings.sortTransparentInstances;
  renderSettings.addViewerFillLights = settings.addViewerFillLights;
  viewer.setRenderQualitySettings(renderSettings);
  viewer.setBackgroundColorRgb255(settings.backgroundColorRgb255);
  viewer.setFogDensity(settings.fogDensity);
  viewer.setShadowOrtho(settings.shadowOrthoHalfSize, settings.shadowNear, settings.shadowFar);
  viewer.setShadowCenterOffset(settings.shadowCenterOffset);
  viewer.setUpdateShadowsEveryFrame(settings.updateShadowsEveryFrame);
  configureViewerFillLights(viewer, settings.addViewerFillLights);

  auto& light = viewer.getLight();
  light.setShadowResolution(settings.shadowResolution);
  light.setShadowsEnabled(settings.shadowsEnabled);
  light.setShadowParams(settings.shadowBias, settings.shadowStrength, settings.shadowPcfRadius);
}

double computeWorldFrameSize(const raisin::Camera& cam) {
  constexpr double kMinDistance = 0.05;
  double visibleHeight = 0.0;
  if (cam.getProjectionMode() == raisin::Camera::ProjectionMode::ORTHOGRAPHIC) {
    visibleHeight = cam.orthoScale;
  } else {
    const glm::vec3 camPos = cam.getPosition();
    const double distance =
      std::max(kMinDistance, static_cast<double>(glm::distance(camPos, glm::vec3(0.0f))));
    const double fovy = glm::radians(static_cast<double>(cam.zoom));
    visibleHeight = 2.0 * distance * std::tan(0.5 * fovy);
  }
  if (visibleHeight <= 0.0) {
    return 0.1;
  }
  return visibleHeight * 0.1;
}

const char* objectTypeLabel(int objectTypeRaw) {
  if (objectTypeRaw == -1) {
    return "visual";
  }
  if (objectTypeRaw == 10) {
    return "deformable";
  }
  if (objectTypeRaw == 11) {
    return "granular";
  }
  if (objectTypeRaw < 0) {
    return "unknown";
  }
  switch (static_cast<raisim::ObjectType>(objectTypeRaw)) {
    case raisim::ObjectType::SPHERE:
      return "sphere";
    case raisim::ObjectType::BOX:
      return "box";
    case raisim::ObjectType::CYLINDER:
      return "cylinder";
    case raisim::ObjectType::CAPSULE:
      return "capsule";
    case raisim::ObjectType::MESH:
      return "mesh";
    case raisim::ObjectType::HALFSPACE:
      return "halfspace";
    case raisim::ObjectType::HEIGHTMAP:
      return "heightmap";
    case raisim::ObjectType::ARTICULATED_SYSTEM:
      return "articulated system";
    case raisim::ObjectType::COMPOUND:
      return "compound";
    default:
      return "unknown";
  }
}

raisin::Visuals* chooseDefaultFollowTarget(const RemoteScene& scene) {
  const auto items = scene.getSelectableObjects();
  raisin::Visuals* fallback = nullptr;
  for (const auto& item : items) {
    if (isContactItem(item) || !item.visual) {
      continue;
    }
    if (!fallback) {
      fallback = item.visual.get();
    }
    if (item.objectTypeRaw >= 0 &&
        static_cast<raisim::ObjectType>(item.objectTypeRaw) == raisim::ObjectType::ARTICULATED_SYSTEM) {
      return item.visual.get();
    }
  }
  return fallback;
}

bool drawOverlaySlider(const char* id, const char* label, float* value, float min, float max,
  const char* format, float valueWidth, float itemWidth, bool disabled) {
  ImGui::PushItemWidth(itemWidth);
  ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
  const bool changed = ImGui::SliderFloat(id, value, min, max, format);
  ImGui::PopStyleColor();
  ImGui::PopItemWidth();

  char valueBuf[32];
  std::snprintf(valueBuf, sizeof(valueBuf), format, *value);

  const ImVec2 itemMin = ImGui::GetItemRectMin();
  const ImVec2 itemMax = ImGui::GetItemRectMax();
  const float textHeight = ImGui::GetFontSize();
  const float textY = itemMin.y + (itemMax.y - itemMin.y - textHeight) * 0.5f;
  const float padding = ImGui::GetStyle().FramePadding.x;
  const float labelX = itemMin.x + padding;
  const float valueX = itemMax.x - padding - valueWidth;
  ImDrawList* drawList = ImGui::GetWindowDrawList();
  drawList->PushClipRect(itemMin, itemMax, true);
  const ImU32 textColor = ImGui::GetColorU32(disabled ? ImGuiCol_TextDisabled : ImGuiCol_Text);
  drawList->AddText(ImVec2(labelX, textY), textColor, label);
  drawList->AddText(ImVec2(valueX, textY), textColor, valueBuf);
  drawList->PopClipRect();
  return changed;
}

void renderViewer(raisin::RayraiWindow& viewer, SDL_Window* window) {
  int fbW = 0;
  int fbH = 0;
  SDL_GL_GetDrawableSize(window, &fbW, &fbH);

  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(ImVec2((float)fbW, (float)fbH));
  ImGui::Begin("Viewer", nullptr,
    ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings |
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse |
      ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
      ImGuiWindowFlags_NoFocusOnAppearing);

  ImTextureID tex = (ImTextureID)(intptr_t)viewer.getImageTexture();
  ImVec2 windowPos = ImGui::GetCursorScreenPos();
  ImGuiIO& io = ImGui::GetIO();
  ImGui::Image(tex, ImVec2((float)fbW, (float)fbH), ImVec2(0, 1), ImVec2(1, 0));

  const bool isHovered = ImGui::IsItemHovered();
  const int cursorX = static_cast<int>(io.MousePos.x - windowPos.x);
  const int cursorY = static_cast<int>(io.MousePos.y - windowPos.y);
  viewer.update(fbW, fbH, isHovered, cursorX, cursorY, true);

  ImGui::End();
  ImGui::PopStyleVar(2);
}

} // namespace

int main(int /*argc*/, char* argv[]) {
  const std::filesystem::path binaryDir = std::filesystem::absolute(argv[0]).parent_path();
  const std::string robotoFontPath = findRobotoFontPath(binaryDir);

  SDL_SetHint(SDL_HINT_VIDEO_HIGHDPI_DISABLED, "permonitorv2");
  SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1");
  std::signal(SIGINT, handleSignalQuit);
#if defined(SIGTERM)
  std::signal(SIGTERM, handleSignalQuit);
#endif
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
    std::cerr << "FATAL ERROR: Failed to initialize SDL: " << SDL_GetError() << "\n";
    return -1;
  }

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

  SDL_Window* window = SDL_CreateWindow("Rayrai Raisim TCP Viewer", SDL_WINDOWPOS_CENTERED,
    SDL_WINDOWPOS_CENTERED, 1280, 720,
    SDL_WindowFlags(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI));

  if (!window) {
    std::cerr << "FATAL ERROR: Failed to create SDL window: " << SDL_GetError() << "\n";
    SDL_Quit();
    return -1;
  }

  SDL_GLContext context = SDL_GL_CreateContext(window);
  if (!context) {
    std::cerr << "FATAL ERROR: Failed to create OpenGL context: " << SDL_GetError() << "\n";
    SDL_DestroyWindow(window);
    SDL_Quit();
    return -1;
  }

  SDL_GL_MakeCurrent(window, context);
  SDL_GL_SetSwapInterval(1);

  glbinding::initialize(
    [](const char* name) {
      return reinterpret_cast<glbinding::ProcAddress>(SDL_GL_GetProcAddress(name));
    },
    false);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui_ImplSDL2_InitForOpenGL(window, context);
  ImGui_ImplOpenGL3_Init("#version 330");
  raionrobotics_imgui_theme();

  auto world = std::make_shared<raisim::World>();
  auto viewer = std::make_shared<raisin::RayraiWindow>(world, 1280, 720);
  ViewerSettings settings;
  copyRenderDefaultsToSettings(settings, settings.renderQuality);
  settings.cameraSpeed = viewer->getCamera().movementSpeed;
  settings.cameraFovDeg = viewer->getCamera().zoom;
  loadViewerSettings(settings);

  raisim_examples::setRayraiBackgroundColorRgb255(*viewer, {20, 20, 30, 255});
  // viewer->setGroundPatternResourcePath(
  //   raisin::getResourceDirectory("raisin_gui") + "material/checkerboard/checker_gray-01.png");
  viewer->setShowCollisionBodies(false);
  auto& camera = viewer->getCamera();
  camera.nearPlane = 0.01f;
  camera.farPlane = 1000.0f;
  camera.zNear = 0.01f;
  camera.zFar = 1000.0f;
  const char* cameraEnv = std::getenv("RAYRAI_TCP_VIEWER_CAMERA_LOOKAT");
  const bool forceCameraEnv = std::getenv("RAYRAI_TCP_VIEWER_FORCE_CAMERA_LOOKAT") != nullptr;
  glm::vec3 forcedCameraPos{0.0f};
  glm::vec3 forcedCameraTarget{0.0f};
  const bool hasForcedCamera = parseCameraLookAtEnv(cameraEnv, forcedCameraPos, forcedCameraTarget);
  glm::vec3 forcedTargetOffset{0.0f};
  const bool hasForcedTargetOffset = parseVec3Env(
    std::getenv("RAYRAI_TCP_VIEWER_CAMERA_OFFSET_FROM_TARGET"), forcedTargetOffset);
  if (hasForcedCamera) {
    applyCameraLookAt(camera, forcedCameraPos, forcedCameraTarget);
  }

  auto& light = viewer->getLight();
  light.type = raisin::LightType::DIRECTIONAL;
  light.ambient = glm::vec3(0.42f, 0.42f, 0.42f);
  light.diffuse = glm::vec3(1.0f, 1.0f, 1.0f);
  light.specular = glm::vec3(0.22f, 0.22f, 0.22f);
  light.setShadowParams(0.0008f, 0.6f, 1.25f);
  light.setShadowsEnabled(true);
  applyViewerSettings(*viewer, settings);

  TcpClient client;
  RemoteScene scene(viewer);
  scene.setShowCollisionBodies(false);
  scene.setForceTransparent(false);

  char host[128] = "127.0.0.1";
  int port = kDefaultPort;
  char portBuf[16];
  std::snprintf(portBuf, sizeof(portBuf), "%d", port);
  std::vector<ConnectionEntry> recentConnections;
  std::vector<std::string> resourceDirs;
  char searchPathBuf[256] = "";
  bool quit = false;
  std::string lastStatus = "disconnected";
  bool verboseParsing = false;
  bool showCollisionBodies = false;
  bool showWorldFrame = false;
  bool showContactPoints = false;
  bool showContactForces = false;
  bool forceTransparent = false;
  float contactPointSize = 0.05f;
  float contactForceSize = 0.3f;
  float cameraSpeed = settings.cameraSpeed;
  float lightYawDeg = settings.lightYawDeg;
  float lightPitchDeg = settings.lightPitchDeg;
  float lightStrength = settings.lightStrength;
  float ambientStrength = settings.ambientStrength;
  ImVec2 overlayOffset(0.0f, 0.0f);
  ImVec2 detailOffset(0.0f, 0.0f);
  ImVec2 detailSize(260.0f, 200.0f);
  const bool defaultAutoConnect = readEnvBool("RAYRAI_TCP_VIEWER_AUTO_CONNECT", true);
  const bool envMinimizePanels = std::getenv("RAYRAI_TCP_VIEWER_MINIMIZE_PANELS") != nullptr;
  const bool envAutoFrame = std::getenv("RAYRAI_TCP_VIEWER_AUTO_FRAME") != nullptr;
  bool autoFrameApplied = false;
  bool overlayMinimized = envMinimizePanels;
  bool detailMinimized = envMinimizePanels;
  std::shared_ptr<raisin::CoordinateFrame> worldFrame;
  bool awaitingResponse = false;
  bool awaitingSensorAck = false;
  bool autoConnect = defaultAutoConnect;
  float uiScale = settings.uiScale;
  float defaultUiScale = 1.0f;
  bool uiScaleInitialized = false;
  bool uiScaleUserSet = settings.uiScaleUserSet;
  float appliedUiScale = 0.0f;
  bool baseStyleCaptured = false;
  ImGuiStyle baseStyle;
  ImVec2 lastDisplaySize(0.0f, 0.0f);
  bool settingsDirty = false;
  bool settingsApplied = false;
  auto nextAutoConnectAttempt = std::chrono::steady_clock::now();
  light.direction = lightDirectionFromYawPitch(lightYawDeg, lightPitchDeg);

  while (!quit && !gSignalQuit.load(std::memory_order_relaxed)) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT)
        quit = true;
      if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE &&
          event.window.windowID == SDL_GetWindowID(window))
        quit = true;
    }

    int fbW = 0;
    int fbH = 0;
    SDL_GL_GetDrawableSize(window, &fbW, &fbH);
    const ImVec2 displaySize(static_cast<float>(fbW), static_cast<float>(fbH));
    const float scaleX = displaySize.x / 1920.0f;
    const float scaleY = displaySize.y / 1080.0f;
    defaultUiScale = std::clamp(std::min(scaleX, scaleY) * 1.25f, 1.1f, 2.6f);
    if (!uiScaleInitialized || (!uiScaleUserSet && (displaySize.x != lastDisplaySize.x ||
                                                     displaySize.y != lastDisplaySize.y))) {
      uiScale = defaultUiScale;
      uiScaleInitialized = true;
    }
    lastDisplaySize = displaySize;
    ImGuiIO& io = ImGui::GetIO();
    io.FontGlobalScale = 1.0f;
    if (!baseStyleCaptured) {
      baseStyle = ImGui::GetStyle();
      baseStyleCaptured = true;
    }
    if (std::abs(appliedUiScale - uiScale) > kUiScaleEpsilon) {
      const float fontSize = std::max(1.0f, std::round(kBaseFontSize * uiScale * kFontScale));
      ImFontConfig fontConfig;
      fontConfig.SizePixels = fontSize;
      fontConfig.OversampleH = 2;
      fontConfig.OversampleV = 2;
      fontConfig.PixelSnapH = true;
      fontConfig.RasterizerMultiply = 1.0f;
      io.Fonts->Clear();
      ImFont* uiFont = nullptr;
      if (!robotoFontPath.empty()) {
        uiFont = io.Fonts->AddFontFromFileTTF(robotoFontPath.c_str(), fontSize, &fontConfig);
      }
      io.FontDefault = uiFont ? uiFont : io.Fonts->AddFontDefault(&fontConfig);
      ImGui_ImplOpenGL3_DestroyFontsTexture();
      ImGui_ImplOpenGL3_CreateFontsTexture();
      ImGuiStyle scaledStyle = baseStyle;
      scaledStyle.ScaleAllSizes(uiScale);
      ImGui::GetStyle() = scaledStyle;
      appliedUiScale = uiScale;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    float menuBarHeight = 0.0f;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    if (ImGui::BeginMainMenuBar()) {
      menuBarHeight = ImGui::GetWindowHeight();
      if (ImGui::BeginMenu("View")) {
        ImGui::SetNextItemWidth(160.0f);
        if (ImGui::SliderFloat("UI Scale", &uiScale, 0.8f, 2.6f, "%.2f")) {
          uiScaleUserSet = true;
          settings.uiScale = uiScale;
          settings.uiScaleUserSet = true;
          settingsDirty = true;
        }
        if (ImGui::MenuItem("Reset to Screen Scale")) {
          uiScale = defaultUiScale;
          uiScaleUserSet = false;
          settings.uiScale = uiScale;
          settings.uiScaleUserSet = false;
          settingsDirty = true;
        }
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Rendering")) {
        bool changed = false;
        bool detailChanged = false;
        ImGui::SeparatorText("Quality");
        int quality = std::clamp(settings.renderQuality, 0, 4);
        constexpr const char* qualityItems[] = {"Fast", "Balanced", "High", "Ultra", "Custom"};
        if (ImGui::Combo("Quality", &quality, qualityItems, IM_ARRAYSIZE(qualityItems))) {
          if (quality == 4) {
            settings.renderQuality = quality;
          } else {
            copyRenderDefaultsToSettings(settings, quality);
          }
          changed = true;
        }

        ImGui::SeparatorText("Background");
        detailChanged |= ImGui::DragFloat4(
          "Background RGBA", &settings.backgroundColorRgb255.x, 1.0f, 0.0f, 255.0f, "%.0f");

        ImGui::SeparatorText("Camera");
        detailChanged |= ImGui::SliderFloat("Move speed", &settings.cameraSpeed, 0.1f, 30.0f, "%.1f");
        detailChanged |= ImGui::SliderFloat("FOV (deg)", &settings.cameraFovDeg, 20.0f, 100.0f, "%.1f");
        detailChanged |= ImGui::SliderFloat("Near clip", &settings.cameraNear, 0.001f, 1.0f, "%.3f");
        detailChanged |= ImGui::SliderFloat("Far clip", &settings.cameraFar, 10.0f, 5000.0f, "%.0f");

        ImGui::SeparatorText("Light");
        detailChanged |= ImGui::SliderFloat("Yaw (deg)", &settings.lightYawDeg, -180.0f, 180.0f, "%.1f");
        detailChanged |= ImGui::SliderFloat("Pitch (deg)", &settings.lightPitchDeg, -89.0f, 89.0f, "%.1f");
        detailChanged |= ImGui::SliderFloat("Key strength", &settings.lightStrength, 0.0f, 2.0f, "%.2f");
        detailChanged |= ImGui::SliderFloat("Ambient", &settings.ambientStrength, 0.0f, 2.0f, "%.2f");
        detailChanged |= ImGui::ColorEdit3("Ambient color", &settings.mainLightAmbient.x);
        detailChanged |= ImGui::ColorEdit3("Diffuse color", &settings.mainLightDiffuse.x);
        detailChanged |= ImGui::ColorEdit3("Specular color", &settings.mainLightSpecular.x);
        detailChanged |= ImGui::Checkbox("Fill/rim lights", &settings.addViewerFillLights);

        ImGui::SeparatorText("Shadows");
        detailChanged |= ImGui::Checkbox("Enable shadows", &settings.shadowsEnabled);
        int shadowResolutionIndex =
          settings.shadowResolution <= 1024 ? 0 : settings.shadowResolution <= 2048 ? 1 :
          settings.shadowResolution <= 4096 ? 2 : 3;
        constexpr const char* shadowResolutionItems[] = {"1024", "2048", "4096", "8192"};
        if (ImGui::Combo("Resolution", &shadowResolutionIndex, shadowResolutionItems,
              IM_ARRAYSIZE(shadowResolutionItems))) {
          constexpr int values[] = {1024, 2048, 4096, 8192};
          settings.shadowResolution = values[shadowResolutionIndex];
          detailChanged = true;
        }
        detailChanged |= ImGui::SliderFloat("Bias", &settings.shadowBias, 0.0f, 0.01f, "%.5f");
        detailChanged |= ImGui::SliderFloat("Strength", &settings.shadowStrength, 0.0f, 1.0f, "%.2f");
        detailChanged |= ImGui::SliderFloat("PCF radius", &settings.shadowPcfRadius, 0.0f, 4.0f, "%.2f");
        detailChanged |= ImGui::SliderFloat("Ortho half-size", &settings.shadowOrthoHalfSize, 1.0f, 100.0f, "%.1f");
        detailChanged |= ImGui::SliderFloat("Near plane", &settings.shadowNear, 0.01f, 10.0f, "%.2f");
        detailChanged |= ImGui::SliderFloat("Far plane", &settings.shadowFar, 1.0f, 250.0f, "%.1f");
        detailChanged |= ImGui::SliderFloat("Center offset", &settings.shadowCenterOffset, 0.0f, 80.0f, "%.1f");
        detailChanged |= ImGui::Checkbox("Update every frame", &settings.updateShadowsEveryFrame);
        detailChanged |= ImGui::SliderInt("Shadowed light budget", &settings.shadowedLightBudget, 0, 8);
        detailChanged |= ImGui::SliderInt("Point shadow lights", &settings.maxPointShadowLights, 0, 8);
        detailChanged |= ImGui::SliderFloat(
          "Additional resolution scale", &settings.additionalShadowResolutionScale, 0.05f, 2.0f, "%.2f");
        detailChanged |= ImGui::SliderFloat(
          "Point resolution scale", &settings.pointShadowResolutionScale, 0.05f, 2.0f, "%.2f");
        detailChanged |= ImGui::SliderInt(
          "Min additional resolution", &settings.minAdditionalShadowResolution, 64, 2048);
        detailChanged |= ImGui::Checkbox(
          "Auto imported shadow light", &settings.autoSelectImportedShadowLight);

        ImGui::SeparatorText("Post");
        detailChanged |= ImGui::SliderFloat("Fog density", &settings.fogDensity, 0.0f, 0.08f, "%.4f");
        detailChanged |= ImGui::SliderFloat("Gamma", &settings.gamma, 0.5f, 2.5f, "%.2f");
        int colorMode = std::clamp(settings.colorMode, 0, 2);
        constexpr const char* colorModeItems[] = {"Fast Linear", "ACES Approx", "Unreal Preview"};
        if (ImGui::Combo("Color mode", &colorMode, colorModeItems, IM_ARRAYSIZE(colorModeItems))) {
          settings.colorMode = colorMode;
          detailChanged = true;
        }
        detailChanged |= ImGui::Checkbox("FXAA", &settings.fxaaEnabled);
        detailChanged |= ImGui::Checkbox("Bloom", &settings.bloomEnabled);
        ImGui::BeginDisabled(!settings.bloomEnabled);
        detailChanged |= ImGui::SliderFloat("Bloom threshold", &settings.bloomThreshold, 0.0f, 4.0f, "%.2f");
        detailChanged |= ImGui::SliderFloat("Bloom strength", &settings.bloomStrength, 0.0f, 2.0f, "%.2f");
        detailChanged |= ImGui::SliderFloat("Bloom radius", &settings.bloomRadius, 0.0f, 12.0f, "%.1f");
        detailChanged |= ImGui::SliderFloat("Bloom knee", &settings.bloomKnee, 0.0f, 1.0f, "%.2f");
        detailChanged |= ImGui::SliderInt("Bloom quality", &settings.bloomQuality, 0, 3);
        ImGui::EndDisabled();
        detailChanged |= ImGui::Checkbox("Screen-space AO", &settings.screenSpaceAoEnabled);
        ImGui::BeginDisabled(!settings.screenSpaceAoEnabled);
        detailChanged |= ImGui::SliderFloat("AO radius", &settings.screenSpaceAoRadius, 0.05f, 10.0f, "%.2f");
        detailChanged |= ImGui::SliderFloat("AO strength", &settings.screenSpaceAoStrength, 0.0f, 4.0f, "%.2f");
        detailChanged |= ImGui::SliderFloat("AO bias", &settings.screenSpaceAoBias, 0.0f, 0.25f, "%.3f");
        ImGui::EndDisabled();
        detailChanged |= ImGui::Checkbox("Opaque depth prepass", &settings.opaqueDepthPrepass);
        detailChanged |= ImGui::Checkbox("Depth of field", &settings.depthOfFieldEnabled);
        ImGui::BeginDisabled(!settings.depthOfFieldEnabled);
        detailChanged |= ImGui::SliderFloat(
          "Focus distance", &settings.depthOfFieldFocusDistance, 0.05f, 30.0f, "%.2f");
        detailChanged |= ImGui::SliderFloat(
          "Focus range", &settings.depthOfFieldFocusRange, 1.0f, 100.0f, "%.1f");
        detailChanged |= ImGui::SliderFloat(
          "Max blur radius", &settings.depthOfFieldMaxRadius, 0.0f, 8.0f, "%.2f");
        ImGui::EndDisabled();

        ImGui::SeparatorText("PBR");
        detailChanged |= ImGui::Checkbox("High fidelity PBR", &settings.highFidelityPbr);
        detailChanged |= ImGui::Checkbox("Tone mapping", &settings.pbrToneMapping);
        detailChanged |= ImGui::SliderFloat("Exposure", &settings.pbrExposure, 0.1f, 4.0f, "%.2f");
        detailChanged |= ImGui::SliderFloat("Environment max LOD", &settings.pbrEnvironmentMaxLod, 0.0f, 12.0f, "%.1f");
        detailChanged |= ImGui::SliderFloat("Environment intensity", &settings.pbrEnvironmentIntensity, 0.0f, 4.0f, "%.2f");
        detailChanged |= ImGui::SliderFloat("Key intensity", &settings.pbrKeyLightIntensity, 0.0f, 4.0f, "%.2f");

        ImGui::SeparatorText("Ground");
        detailChanged |= ImGui::Checkbox("Reflective checkerboard", &settings.reflectiveGround);
        ImGui::BeginDisabled(!settings.reflectiveGround);
        detailChanged |= ImGui::SliderFloat(
          "Ground roughness", &settings.reflectiveGroundRoughness, 0.02f, 1.0f, "%.2f");
        detailChanged |= ImGui::SliderFloat(
          "Ground metallic", &settings.reflectiveGroundMetallic, 0.0f, 1.0f, "%.2f");
        ImGui::EndDisabled();

        detailChanged |= ImGui::Checkbox("Sort transparent instances", &settings.sortTransparentInstances);
        detailChanged |= ImGui::SliderInt(
          "Additional lights per frame", &settings.maxAdditionalLightsPerFrame, 0, 16);
        detailChanged |= ImGui::SliderFloat(
          "Min light influence", &settings.minAdditionalLightInfluence, 0.0f, 1.0f, "%.3f");

        if (ImGui::Button("Reset Rendering Settings")) {
          ViewerSettings defaults;
          copyRenderDefaultsToSettings(defaults, defaults.renderQuality);
          defaults.uiScale = settings.uiScale;
          defaults.uiScaleUserSet = settings.uiScaleUserSet;
          settings = defaults;
          changed = true;
        }

        changed |= detailChanged;
        if (changed) {
          if (detailChanged) {
            settings.renderQuality = 4;
          }
          cameraSpeed = settings.cameraSpeed;
          lightYawDeg = settings.lightYawDeg;
          lightPitchDeg = settings.lightPitchDeg;
          lightStrength = settings.lightStrength;
          ambientStrength = settings.ambientStrength;
          settingsDirty = true;
        }
        ImGui::EndMenu();
      }
      ImGui::EndMainMenuBar();
    }
    ImGui::PopStyleVar(3);

    const auto now = std::chrono::steady_clock::now();
    if (autoConnect && !client.isConnected() && now >= nextAutoConnectAttempt) {
      lastStatus = "auto-connecting";
      if (client.connectTo(host, port, false, kConnectTimeoutMs)) {
        lastStatus = "connected";
        awaitingResponse = false;
        awaitingSensorAck = false;
        recordConnection(recentConnections, host, port);
      } else {
        lastStatus = "auto-connect failed";
      }
      nextAutoConnectAttempt = now + kAutoConnectInterval;
    }

    uint32_t requestedTag = 0;
    int requestedIndex = 0;
    const VisualEntry* requestedEntry = nullptr;
    scene.getVisualInfo(viewer->getTargetVisual(), requestedTag, requestedIndex, requestedEntry);
    static_cast<void>(requestedIndex);
    static_cast<void>(requestedEntry);
    if (requestedEntry && (requestedEntry->shape == raisim::Shape::Ground ||
                            requestedEntry->shape == raisim::Shape::HeightMap)) {
      viewer->setTargetVisual(nullptr);
      requestedTag = 0;
    }
    scene.setSelectionTag(hasForcedTargetOffset ? 0 : requestedTag);
    const uint32_t updateRequestTag = hasForcedTargetOffset ? 0 : requestedTag;

    if (client.isConnected()) {
      std::vector<char> payload;
      bool networkFailed = false;

      if (awaitingSensorAck) {
        if (!client.recvMessage(payload)) {
          if (!client.lastIoWouldBlock()) {
            lastStatus = "sensor ack failed";
            networkFailed = true;
          }
        } else {
          awaitingSensorAck = false;
        }
      }

      if (!networkFailed && !awaitingSensorAck) {
        if (!awaitingResponse) {
          if (!sendUpdateRequest(client, updateRequestTag)) {
            if (!client.lastIoWouldBlock()) {
              lastStatus = "connection lost";
              networkFailed = true;
            }
          } else {
            awaitingResponse = true;
          }
        }

        if (!networkFailed && awaitingResponse) {
          if (!client.recvMessage(payload)) {
            if (!client.lastIoWouldBlock()) {
              lastStatus = "connection lost";
              networkFailed = true;
            }
          } else {
            awaitingResponse = false;
            BufferReader reader(payload);
            std::vector<PendingSensorUpdate> pending;
            scene.setVerbose(verboseParsing);
            const bool parsedOk = scene.applyResponse(reader, pending);
            const bool disconnectRequested = scene.consumeDisconnectRequested();
            if (disconnectRequested) {
              lastStatus = "protocol error (disconnect)";
              networkFailed = true;
            } else if (!parsedOk) {
              lastStatus = "parse error (dropped update)";
            } else {
              glm::vec3 sceneMin, sceneMax;
              if (envAutoFrame && !autoFrameApplied && scene.computeSceneBounds(sceneMin, sceneMax)) {
                const glm::vec3 center = 0.5f * (sceneMin + sceneMax);
                const glm::vec3 extent = glm::max(sceneMax - sceneMin, glm::vec3(0.25f));
                const float radius = 0.5f * glm::length(extent);
                const float fovy = glm::radians(std::max(20.0f, viewer->getCamera().zoom));
                const float distance = std::max(1.0f, radius / std::tan(0.5f * fovy) * 1.55f);
                const glm::vec3 viewDir = glm::normalize(glm::vec3(1.55f, -1.85f, 1.05f));
                applyCameraLookAt(viewer->getCamera(), center + viewDir * distance, center);
                autoFrameApplied = true;
              }
            }
            if (hasForcedTargetOffset && viewer->getTargetVisual()) {
              const glm::vec3 target = viewer->getTargetVisual()->getPosition();
              applyCameraLookAt(viewer->getCamera(), target + forcedTargetOffset, target);
            } else if (hasForcedTargetOffset) {
              if (raisin::Visuals* followTarget = chooseDefaultFollowTarget(scene)) {
                viewer->setTargetVisual(followTarget);
                const glm::vec3 target = followTarget->getPosition();
                applyCameraLookAt(viewer->getCamera(), target + forcedTargetOffset, target);
              }
            } else if (forceCameraEnv && hasForcedCamera) {
              viewer->setTargetVisual(nullptr);
              applyCameraLookAt(viewer->getCamera(), forcedCameraPos, forcedCameraTarget);
            }
            if (!parsedOk) {
              // handled above
            } else if (!pending.empty()) {
              if (!sendSensorUpdate(client, pending)) {
                if (!client.lastIoWouldBlock()) {
                  lastStatus = "sensor update failed";
                  networkFailed = true;
                }
              } else {
                if (!client.recvMessage(payload)) {
                  if (client.lastIoWouldBlock()) {
                    awaitingSensorAck = true;
                  } else {
                    lastStatus = "sensor ack failed";
                    networkFailed = true;
                  }
                }
              }
            }
          }
        }
      }

      if (networkFailed) {
        awaitingResponse = false;
        awaitingSensorAck = false;
        client.disconnect();
        scene.clear();
      }
    }

    if (showWorldFrame) {
      if (!worldFrame) {
        worldFrame = viewer->addCoordinateFrame("world_frame");
      }
      if (worldFrame) {
        worldFrame->poses.resize(1);
        worldFrame->poses[0].position = Eigen::Vector3d(0.0, 0.0, 0.0);
        worldFrame->poses[0].quaternion = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
        worldFrame->frameSize = computeWorldFrameSize(viewer->getCamera());
      }
    } else if (worldFrame) {
      viewer->removeCoordinateFrame("world_frame");
      worldFrame.reset();
    }
    if (showWorldFrame && worldFrame) {
      worldFrame->frameSize = computeWorldFrameSize(viewer->getCamera());
    }

    auto& cam = viewer->getCamera();
    cam.nearPlane = settings.cameraNear;
    cam.farPlane = settings.cameraFar;
    cam.zNear = settings.cameraNear;
    cam.zFar = settings.cameraFar;
    cam.zoom = settings.cameraFovDeg;
    cam.movementSpeed = cameraSpeed;
    if (!settingsApplied || settingsDirty) {
      applyViewerSettings(*viewer, settings);
      saveViewerSettings(settings);
      settingsApplied = true;
      settingsDirty = false;
    }
    auto& lightRef = viewer->getLight();
    lightRef.type = raisin::LightType::DIRECTIONAL;
    lightRef.ambient = settings.mainLightAmbient * ambientStrength;
    lightRef.diffuse = settings.mainLightDiffuse * lightStrength;
    lightRef.specular = settings.mainLightSpecular * lightStrength;
    lightRef.direction = lightDirectionFromYawPitch(lightYawDeg, lightPitchDeg);
    lightRef.setShadowsEnabled(settings.shadowsEnabled);
    lightRef.setShadowResolution(settings.shadowResolution);
    lightRef.setShadowParams(settings.shadowBias, settings.shadowStrength, settings.shadowPcfRadius);

    scene.updateContactVisuals(
      showContactPoints, contactPointSize, showContactForces, contactForceSize);

    renderViewer(*viewer, window);

    const ImVec2 overlayBase(12.0f, 12.0f + menuBarHeight);
    const ImVec2 overlayPos(overlayBase.x + overlayOffset.x, overlayBase.y + overlayOffset.y);
    ImGui::SetNextWindowBgAlpha(0.5f);
    ImGui::SetNextWindowPos(overlayPos, ImGuiCond_Always);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));
    const ImGuiWindowFlags overlayFlags =
      ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
      ImGuiWindowFlags_NoNavFocus;
    bool overlayHovered = false;
    if (ImGui::Begin("Raisim TCP##Overlay", nullptr, overlayFlags)) {
      overlayHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
      const ImGuiStyle& style = ImGui::GetStyle();
      const float toggleWidth =
        ImGui::CalcTextSize("-").x + style.FramePadding.x * 2.0f;
      const float toggleButtonWidth = toggleWidth * 1.6f;
      const float headerWidth = ImGui::GetContentRegionAvail().x;
      const float dragWidth =
        std::max(0.0f, headerWidth - toggleButtonWidth - style.ItemSpacing.x);

      ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.35f, 0.8f, 1.0f, 1.0f));
      ImGui::Selectable("Raisim TCP", false, ImGuiSelectableFlags_None, ImVec2(dragWidth, 0.0f));
      ImGui::PopStyleColor();
      if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        ImVec2 delta = ImGui::GetIO().MouseDelta;
        overlayOffset.x = std::max(0.0f, overlayOffset.x + delta.x);
        overlayOffset.y = std::max(0.0f, overlayOffset.y + delta.y);
      }
      ImGui::SameLine();
      const float toggleHeight = ImGui::GetFrameHeight();
      if (ImGui::Button(overlayMinimized ? "+##overlay_toggle" : "-##overlay_toggle", ImVec2(toggleButtonWidth, toggleHeight))) {
        overlayMinimized = !overlayMinimized;
      }
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s", overlayMinimized ? "Expand" : "Minimize");
      }

      if (!overlayMinimized && ImGui::BeginTabBar("##LeftTabs")) {
        if (ImGui::BeginTabItem("Control")) {
          ConnectionEntry current;
          current.host = host;
          current.port = port;
          std::string preview = formatConnectionLabel(current);
          if (preview.empty()) {
            preview = "set host:port";
          }
          float comboLabelWidth = ImGui::CalcTextSize(preview.c_str()).x;
          for (const auto& entry : recentConnections) {
            const std::string label = formatConnectionLabel(entry);
            comboLabelWidth = std::max(comboLabelWidth, ImGui::CalcTextSize(label.c_str()).x);
          }
          const float minHostTextWidth = ImGui::CalcTextSize("255.255.255.255").x;
          const float minPortTextWidth = ImGui::CalcTextSize("65535").x;
          const float hostTextWidth =
            std::max(minHostTextWidth, ImGui::CalcTextSize(host).x);
          const float portTextWidth =
            std::max(minPortTextWidth, ImGui::CalcTextSize(portBuf).x);
          const float hostInputWidth = hostTextWidth + style.FramePadding.x * 2.0f;
          const float portInputWidth = portTextWidth + style.FramePadding.x * 2.0f;
          const float hostLabelWidth = ImGui::CalcTextSize("Host").x;
          const float portLabelWidth = ImGui::CalcTextSize("Port").x;
          const float labelSpacing = style.ItemInnerSpacing.x;
          const float segmentSpacing = style.ItemSpacing.x;
          const float hostSegmentWidth = hostLabelWidth + labelSpacing + hostInputWidth;
          const float portSegmentWidth = portLabelWidth + labelSpacing + portInputWidth;
          const float hostRowWidth = hostSegmentWidth + segmentSpacing + portSegmentWidth;
          const float comboWidth =
            comboLabelWidth + style.FramePadding.x * 2.0f + ImGui::GetFrameHeight();
          const float popupMinWidth = std::max(comboWidth, hostRowWidth) + style.WindowPadding.x * 2.0f;
          ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1.0f);
          ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.35f, 0.35f, 0.35f, 0.9f));
          ImGui::SetNextItemWidth(comboWidth);
          ImGui::SetNextWindowSizeConstraints(
            ImVec2(popupMinWidth, 0.0f), ImVec2(FLT_MAX, FLT_MAX));
          if (ImGui::BeginCombo("##Connection", preview.c_str(), ImGuiComboFlags_HeightSmall)) {
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted("Host");
            ImGui::SameLine(0.0f, labelSpacing);
            ImGui::SetNextItemWidth(hostInputWidth);
            ImGui::InputText("##Host", host, sizeof(host));
            ImGui::SameLine(0.0f, segmentSpacing);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted("Port");
            ImGui::SameLine(0.0f, labelSpacing);
            ImGui::SetNextItemWidth(portInputWidth);
            ImGui::InputText("##Port", portBuf, sizeof(portBuf),
              ImGuiInputTextFlags_CharsDecimal);
            if (portBuf[0] != '\0') {
              int parsed = std::atoi(portBuf);
              if (parsed < 0) {
                parsed = 0;
              } else if (parsed > 65535) {
                parsed = 65535;
              }
              port = parsed;
            }
            ImGui::SeparatorText("Recent");
            if (recentConnections.empty()) {
              ImGui::TextDisabled("No recent connections");
            } else {
              for (const auto& entry : recentConnections) {
                const std::string label = formatConnectionLabel(entry);
                if (ImGui::Selectable(label.c_str())) {
                  std::snprintf(host, sizeof(host), "%s", entry.host.c_str());
                  port = entry.port;
                  std::snprintf(portBuf, sizeof(portBuf), "%d", port);
                }
              }
            }
            ImGui::EndCombo();
          }
          ImGui::PopStyleColor();
          ImGui::PopStyleVar();

          ImGui::SameLine();
          if (!client.isConnected()) {
            if (ImGui::Button("Connect")) {
              lastStatus = "connecting";
              if (client.connectTo(host, port, true, kConnectTimeoutMs)) {
                lastStatus = "connected";
                awaitingResponse = false;
                awaitingSensorAck = false;
                recordConnection(recentConnections, host, port);
              } else {
                lastStatus = "connect failed";
              }
            }
          } else {
            if (ImGui::Button("Disconnect")) {
              client.disconnect();
              awaitingResponse = false;
              awaitingSensorAck = false;
              lastStatus = "disconnected";
              scene.clear();
            }
          }

          ImGui::SameLine();
          if (ImGui::Checkbox("Auto-connect", &autoConnect)) {
            if (autoConnect) {
              nextAutoConnectAttempt = now;
            }
          }

          char worldText[32];
          if (scene.hasServerWorldTime()) {
            std::snprintf(worldText, sizeof(worldText), "World %.3f s", scene.getServerWorldTime());
          } else {
            std::snprintf(worldText, sizeof(worldText), "World --");
          }
          const ImVec4 statusColor =
            client.isConnected() ? ImVec4(0.2f, 0.8f, 0.2f, 1.0f) : ImVec4(0.9f, 0.2f, 0.2f, 1.0f);
          const float statusWidth =
            ImGui::CalcTextSize("Status: parse error (dropped update)").x;
          const float worldTextWidth = ImGui::CalcTextSize("World 12345678.901 s").x;
          const float rowWidth = statusWidth + style.ItemSpacing.x + worldTextWidth;
          const float rowStartX = ImGui::GetCursorPosX();
          ImGui::TextColored(statusColor, "Status: %s", lastStatus.c_str());
          ImGui::SameLine();
          const float worldX = rowStartX + rowWidth - worldTextWidth;
          if (worldX > ImGui::GetCursorPosX()) {
            ImGui::SetCursorPosX(worldX);
          }
          ImGui::TextUnformatted(worldText);
          ImGui::TextDisabled("Heightmap colors: server color map");

          if (ImGui::BeginTable("##viewer_checkboxes", 2, ImGuiTableFlags_SizingFixedFit)) {
            ImGui::TableNextColumn();
            ImGui::Checkbox("Verbose parsing", &verboseParsing);

            ImGui::TableNextColumn();
            if (ImGui::Checkbox("Show Collision Bodies", &showCollisionBodies)) {
              viewer->setShowCollisionBodies(showCollisionBodies);
              scene.setShowCollisionBodies(showCollisionBodies);
            }

            ImGui::TableNextColumn();
            if (ImGui::Checkbox("All Transparent", &forceTransparent)) {
              scene.setForceTransparent(forceTransparent);
            }

            ImGui::TableNextColumn();
            if (ImGui::Checkbox("Show World Frame", &showWorldFrame)) {
              if (showWorldFrame) {
                if (!worldFrame) {
                  worldFrame = viewer->addCoordinateFrame("world_frame");
                }
                if (worldFrame) {
                  worldFrame->poses.resize(1);
                  worldFrame->poses[0].position = Eigen::Vector3d(0.0, 0.0, 0.0);
                  worldFrame->poses[0].quaternion = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
                  worldFrame->frameSize = computeWorldFrameSize(viewer->getCamera());
                }
              } else if (worldFrame) {
                viewer->removeCoordinateFrame("world_frame");
                worldFrame.reset();
              }
            }

            ImGui::TableNextColumn();
            ImGui::Checkbox("Show Contact Points", &showContactPoints);

            ImGui::TableNextColumn();
            ImGui::Checkbox("Show Contact Forces", &showContactForces);
            ImGui::EndTable();
          }

          const bool contactPointsEnabled = showContactPoints;
          const bool contactForcesEnabled = showContactForces;
          auto measureValueWidth = [](float value, const char* format) {
            char buffer[32];
            std::snprintf(buffer, sizeof(buffer), format, value);
            return ImGui::CalcTextSize(buffer).x;
          };
          const float leftValueWidth = std::max({measureValueWidth(contactPointSize, "%.3f"),
            measureValueWidth(cameraSpeed, "%.1f"), measureValueWidth(lightPitchDeg, "%.1f"),
            measureValueWidth(ambientStrength, "%.2f")});
          const float rightValueWidth = std::max({measureValueWidth(contactForceSize, "%.2f"),
            measureValueWidth(lightYawDeg, "%.1f"), measureValueWidth(lightStrength, "%.2f")});
          const float innerSpacing = ImGui::GetStyle().ItemInnerSpacing.x;
          const float padding = ImGui::GetStyle().FramePadding.x;
          const float leftLabelWidth = std::max({ImGui::CalcTextSize("Contact Pt (m)").x,
            ImGui::CalcTextSize("Camera Speed").x, ImGui::CalcTextSize("Light Pitch (deg)").x,
            ImGui::CalcTextSize("Ambient Strength").x});
          const float rightLabelWidth = std::max({ImGui::CalcTextSize("Contact Force (m)").x,
            ImGui::CalcTextSize("Light Yaw (deg)").x, ImGui::CalcTextSize("Light Strength").x});
          const float leftItemWidth = leftLabelWidth + leftValueWidth + innerSpacing + padding * 2;
          const float rightItemWidth =
            rightLabelWidth + rightValueWidth + innerSpacing + padding * 2;
          const float cellPadX = ImGui::GetStyle().CellPadding.x;
          const float sliderRowWidth =
            leftItemWidth + rightItemWidth + ImGui::GetStyle().ItemSpacing.x + cellPadX * 4.0f;
          ImGui::PushStyleVar(
            ImGuiStyleVar_CellPadding, ImVec2(8.0f, ImGui::GetStyle().CellPadding.y));
          if (ImGui::BeginTable("##viewer_sliders", 2, ImGuiTableFlags_SizingFixedFit)) {
            ImGui::TableSetupColumn("left", ImGuiTableColumnFlags_WidthFixed, leftItemWidth);
            ImGui::TableSetupColumn("right", ImGuiTableColumnFlags_WidthFixed, rightItemWidth);
            ImGui::TableNextColumn();
            ImGui::BeginDisabled(!contactPointsEnabled);
            drawOverlaySlider("##contact_point_size", "Contact Pt (m)", &contactPointSize, 0.001f,
              0.4f, "%.3f", leftValueWidth, leftItemWidth, !contactPointsEnabled);
            ImGui::EndDisabled();

            ImGui::TableNextColumn();
            ImGui::BeginDisabled(!contactForcesEnabled);
            drawOverlaySlider("##contact_force_size", "Contact Force (m)", &contactForceSize, 0.01f,
              2.0f, "%.2f", rightValueWidth, rightItemWidth, !contactForcesEnabled);
            ImGui::EndDisabled();

            ImGui::TableNextColumn();
            if (drawOverlaySlider("##camera_speed", "Camera Speed", &cameraSpeed, 0.1f, 30.0f, "%.1f",
                  leftValueWidth, leftItemWidth, false)) {
              settings.cameraSpeed = cameraSpeed;
              settingsDirty = true;
            }

            ImGui::TableNextColumn();
            if (drawOverlaySlider("##light_yaw", "Light Yaw (deg)", &lightYawDeg, -180.0f, 180.0f,
                  "%.1f", rightValueWidth, rightItemWidth, false)) {
              settings.lightYawDeg = lightYawDeg;
              settingsDirty = true;
            }

            ImGui::TableNextColumn();
            if (drawOverlaySlider("##light_pitch", "Light Pitch (deg)", &lightPitchDeg, -89.0f, 89.0f,
                  "%.1f", leftValueWidth, leftItemWidth, false)) {
              settings.lightPitchDeg = lightPitchDeg;
              settingsDirty = true;
            }

            ImGui::TableNextColumn();
            if (drawOverlaySlider("##light_strength", "Light Strength", &lightStrength, 0.1f, 1.2f,
                  "%.2f", rightValueWidth, rightItemWidth, false)) {
              settings.lightStrength = lightStrength;
              settingsDirty = true;
            }

            ImGui::TableNextColumn();
            if (drawOverlaySlider("##ambient_strength", "Ambient Strength", &ambientStrength, 0.0f,
                  2.0f, "%.2f", leftValueWidth, leftItemWidth, false)) {
              settings.ambientStrength = ambientStrength;
              settingsDirty = true;
            }

            ImGui::TableNextColumn();
            ImGui::Dummy(ImVec2(rightItemWidth, 0.0f));
            ImGui::EndTable();
          }
          ImGui::PopStyleVar();

          ImGui::SeparatorText("Resource dirs");
          ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.12f, 0.15f, 0.55f));
          ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
          ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
          const ImGuiChildFlags resourceFlags = ImGuiChildFlags_Border |
                                                ImGuiChildFlags_AutoResizeY |
                                                ImGuiChildFlags_AlwaysUseWindowPadding;
          if (ImGui::BeginChild(
                "##resource_dirs_box", ImVec2(sliderRowWidth, 0.0f), resourceFlags)) {
            const float rowWidth = ImGui::GetContentRegionAvail().x;
            const ImVec2 buttonTextSize = ImGui::CalcTextSize("Add");
            const float buttonWidth = buttonTextSize.x + ImGui::GetStyle().FramePadding.x * 2.0f;
            const float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
            const float inputWidth = std::max(0.0f, rowWidth - buttonWidth - spacing);
            ImGui::SetNextItemWidth(inputWidth);
            ImGui::InputTextWithHint(
              "##resource_dir_input", "path/to/resources", searchPathBuf, sizeof(searchPathBuf));
            ImGui::SameLine(0.0f, spacing);
            if (ImGui::Button("Add", ImVec2(buttonWidth, 0.0f))) {
              std::string path(searchPathBuf);
              if (!path.empty()) {
                scene.addSearchPath(path);
                recordResourceDir(resourceDirs, path);
                searchPathBuf[0] = '\0';
              }
            }

            if (!resourceDirs.empty()) {
              const float removeSize = ImGui::GetFontSize() * 1.1f;
              if (ImGui::BeginTable("##resource_dirs", 2, ImGuiTableFlags_SizingFixedFit)) {
                ImGui::TableSetupColumn("path", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("remove", ImGuiTableColumnFlags_WidthFixed, removeSize);
                for (size_t i = 0; i < resourceDirs.size(); ++i) {
                  const auto& entry = resourceDirs[i];
                  const std::string label = shortenPathLabel(entry, 50);
                  ImGui::TableNextRow();
                  ImGui::TableSetColumnIndex(0);
                  ImGui::TextUnformatted(label.c_str());
                  ImGui::TableSetColumnIndex(1);
                  ImGui::PushID(static_cast<int>(i));
                  if (ImGui::Button("x", ImVec2(removeSize, removeSize))) {
                    resourceDirs.erase(resourceDirs.begin() + static_cast<long>(i));
                    ImGui::PopID();
                    break;
                  }
                  ImGui::PopID();
                }
                ImGui::EndTable();
              }
            }
          }
          ImGui::EndChild();
          ImGui::PopStyleVar(2);
          ImGui::PopStyleColor();
          ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Objects")) {
          uint32_t selectedTag = 0;
          int selectedIndex = 0;
          const VisualEntry* selectedEntry = nullptr;
          const raisin::Visuals* selectedVisual = viewer->getTargetVisual();
          const bool hasSelectedRaw =
            scene.getVisualInfo(selectedVisual, selectedTag, selectedIndex, selectedEntry);
          const bool selectedIsContact = hasSelectedRaw && isContactEntry(selectedEntry);
          if (selectedIsContact && viewer) {
            viewer->setTargetVisual(nullptr);
          }
          const bool hasSelected = hasSelectedRaw && !selectedIsContact;
          static_cast<void>(selectedIndex);
          static_cast<void>(selectedEntry);
          const auto items = scene.getSelectableObjects();

          if (items.empty()) {
            ImGui::TextDisabled("No objects available");
          } else {
            const ImU32 typeColor = ImGui::GetColorU32(ImVec4(0.35f, 0.8f, 1.0f, 1.0f));
            const float listHeight = ImGui::GetFontSize() * 12.0f;
            if (ImGui::BeginChild("##ObjectList", ImVec2(0.0f, listHeight), true)) {
              for (const auto& item : items) {
                if (isContactItem(item)) {
                  continue;
                }
                const char* typeName = objectTypeLabel(item.objectTypeRaw);
                const bool selected = hasSelected && item.tag == selectedTag;
                const std::string id =
                  "##obj_" + std::to_string(item.tag) + "_" + std::to_string(item.index);
                if (ImGui::Selectable(id.c_str(), selected, ImGuiSelectableFlags_SpanAllColumns)) {
                  if (viewer) {
                    viewer->setTargetVisual(item.visual.get());
                  }
                }
                const ImVec2 itemMin = ImGui::GetItemRectMin();
                const ImVec2 textPos(itemMin.x + ImGui::GetStyle().FramePadding.x,
                  itemMin.y + ImGui::GetStyle().FramePadding.y);
                const std::string nameText =
                  item.name.empty() ? ("tag " + std::to_string(item.tag)) : item.name;
                const char* sep = ": ";
                const ImVec2 typeSize = ImGui::CalcTextSize(typeName);
                const ImVec2 sepSize = ImGui::CalcTextSize(sep);
                ImDrawList* drawList = ImGui::GetWindowDrawList();
                drawList->AddText(
                  ImGui::GetFont(), ImGui::GetFontSize(), textPos, typeColor, typeName);
                drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize(),
                  ImVec2(textPos.x + typeSize.x, textPos.y), ImGui::GetColorU32(ImGuiCol_Text),
                  sep);
                drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize(),
                  ImVec2(textPos.x + typeSize.x + sepSize.x, textPos.y),
                  ImGui::GetColorU32(ImGuiCol_Text), nameText.c_str());
              }
              ImGui::EndChild();
            }
          }
          ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
      }
    }
    ImGui::End();
    ImGui::PopStyleVar();
    if (!overlayMinimized && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !overlayHovered &&
        !ImGui::IsAnyItemActive()) {
      overlayMinimized = true;
    }

    const raisin::Visuals* selectedVisual = viewer->getTargetVisual();
    const VisualEntry* selectedEntry = nullptr;
    uint32_t selectedTag = 0;
    int selectedIndex = 0;
    scene.getVisualInfo(selectedVisual, selectedTag, selectedIndex, selectedEntry);

    if (selectedEntry && isContactEntry(selectedEntry)) {
      selectedEntry = nullptr;
    }

    if (selectedEntry) {
      const ImVec2 detailsBasePos(displaySize.x - detailSize.x - 12.0f, 12.0f + menuBarHeight);
      ImVec2 detailsPos = detailsBasePos;
      detailsPos.x += detailOffset.x;
      detailsPos.y += detailOffset.y;
      ImGui::SetNextWindowBgAlpha(0.5f);
      ImGui::SetNextWindowPos(detailsPos, ImGuiCond_Always);
      const ImVec2 detailMinSize = detailMinimized ? ImVec2(160.0f, 0.0f) : detailSize;
      ImGui::SetNextWindowSizeConstraints(detailMinSize, ImVec2(FLT_MAX, FLT_MAX));
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));
      const ImGuiWindowFlags detailFlags =
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNavFocus;
      ImVec2 detailWindowSize(0.0f, 0.0f);
      if (ImGui::Begin("Selected Object##Overlay", nullptr, detailFlags)) {
        const ImGuiStyle& detailStyle = ImGui::GetStyle();
        const float detailToggleWidth =
          (ImGui::CalcTextSize("-").x + detailStyle.FramePadding.x * 2.0f) * 1.6f;
        const float detailDragWidth = std::max(0.0f,
          ImGui::GetContentRegionAvail().x - detailToggleWidth - detailStyle.ItemSpacing.x);
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.35f, 0.8f, 1.0f, 1.0f));
        ImGui::Selectable("Selected Object", false, ImGuiSelectableFlags_None,
          ImVec2(detailDragWidth, 0.0f));
        ImGui::PopStyleColor();
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
          ImVec2 delta = ImGui::GetIO().MouseDelta;
          detailOffset.x = std::min(0.0f, detailOffset.x + delta.x);
          detailOffset.y = std::max(0.0f, detailOffset.y + delta.y);
        }
        ImGui::SameLine();
        if (ImGui::Button(detailMinimized ? "+##detail_toggle" : "-##detail_toggle",
              ImVec2(detailToggleWidth, ImGui::GetFrameHeight()))) {
          detailMinimized = !detailMinimized;
        }
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip("%s", detailMinimized ? "Expand" : "Minimize");
        }

        if (!detailMinimized) {
            ImGui::Separator();
          const ImVec4 tagColor(0.95f, 0.72f, 0.2f, 1.0f);
          const ImVec4 indexColor(0.25f, 0.85f, 0.7f, 1.0f);
          const ImVec4 shapeColor(0.35f, 0.6f, 1.0f, 1.0f);
          const ImVec4 nameColor(0.95f, 0.95f, 0.95f, 1.0f);
          const ImVec4 metaColor(0.85f, 0.85f, 0.85f, 1.0f);
          const SelectedObjectInfo& selectedInfo = scene.getSelectedInfo();

          std::string objectName = selectedEntry->objectName;
          if (objectName.empty()) {
            objectName = scene.getObjectName(selectedTag);
          }
          if (objectName.empty()) {
            objectName = "unnamed";
          }

          if (ImGui::BeginTable("##selected_props", 2, ImGuiTableFlags_SizingFixedFit)) {
            ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Name");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(nameColor, "%s", objectName.c_str());

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Tag");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(tagColor, "%u", selectedTag);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Index");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(indexColor, "%d", selectedIndex);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Type");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(shapeColor, "%s", objectTypeLabel(selectedEntry->objectTypeRaw));

            if (!selectedEntry->meshFile.empty()) {
              std::string meshLabel = selectedEntry->meshFile;
              const size_t slashPos = meshLabel.find_last_of("/\\");
              if (slashPos != std::string::npos && slashPos + 1 < meshLabel.size()) {
                meshLabel = meshLabel.substr(slashPos + 1);
              }
              ImGui::TableNextRow();
              ImGui::TableSetColumnIndex(0);
              ImGui::TextUnformatted("Mesh");
              ImGui::TableSetColumnIndex(1);
              ImGui::TextColored(metaColor, "%s", meshLabel.c_str());
            }

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Articulated");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(metaColor, "%s", selectedEntry->isArticulated ? "yes" : "no");

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Collision");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(metaColor, "%s", selectedEntry->isCollision ? "yes" : "no");

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Pos");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(metaColor, "%.3f %.3f %.3f", selectedEntry->lastPos.x,
              selectedEntry->lastPos.y, selectedEntry->lastPos.z);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Quat");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(metaColor, "%.3f %.3f %.3f %.3f", selectedEntry->lastQuat.w,
              selectedEntry->lastQuat.x, selectedEntry->lastQuat.y, selectedEntry->lastQuat.z);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Size");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(metaColor, "%.3f %.3f %.3f", selectedEntry->lastSize.x,
              selectedEntry->lastSize.y, selectedEntry->lastSize.z);

            ImGui::EndTable();
          }

          if (selectedEntry->isArticulated) {
            ImGui::SeparatorText("Joints");
            if (selectedInfo.valid && selectedInfo.isArticulated && selectedInfo.tag == selectedTag &&
                !selectedInfo.jointNames.empty()) {
              if (ImGui::BeginTable("##selected_joints", 2,
                    ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                      ImGuiTableFlags_SizingFixedFit)) {
                ImGui::TableSetupColumn("Joint", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Angle", ImGuiTableColumnFlags_WidthFixed);
                ImGui::TableHeadersRow();
                for (size_t i = 0; i < selectedInfo.jointNames.size(); ++i) {
                  ImGui::TableNextRow();
                  ImGui::TableSetColumnIndex(0);
                  ImGui::TextUnformatted(selectedInfo.jointNames[i].c_str());
                  ImGui::TableSetColumnIndex(1);
                  const float angle =
                    (i < selectedInfo.jointAngles.size()) ? selectedInfo.jointAngles[i] : 0.0f;
                  ImGui::TextColored(ImVec4(0.85f, 0.9f, 1.0f, 1.0f), "%.4f", angle);
                }
                ImGui::EndTable();
              }
            } else {
              ImGui::TextDisabled("Joint data not available");
            }
          }
        }
        detailWindowSize = ImGui::GetWindowSize();
      }
      ImGui::End();
      ImGui::PopStyleVar();
      detailSize.x = std::max(detailSize.x, detailWindowSize.x);
      detailSize.y = std::max(detailSize.y, detailWindowSize.y);
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
  }

  client.disconnect();
  scene.shutdown();
  viewer.reset();

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();

  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
