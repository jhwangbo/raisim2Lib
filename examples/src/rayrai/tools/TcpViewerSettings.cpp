// Copyright (c) 2026 Raion Robotics Inc.
// All rights reserved.

#include "TcpViewerSettings.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

namespace raisin::tcp_viewer
{
namespace
{

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

bool parseLongStrict(const char* value, int base, long& out) {
  if (!value) {
    return false;
  }
  while (std::isspace(static_cast<unsigned char>(*value))) {
    ++value;
  }
  if (*value == '\0') {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  out = std::strtol(value, &end, base);
  if (end == value || errno == ERANGE) {
    return false;
  }
  while (std::isspace(static_cast<unsigned char>(*end))) {
    ++end;
  }
  return *end == '\0';
}

bool parseFloatStrict(const char* value, float& out) {
  if (!value) {
    return false;
  }
  while (std::isspace(static_cast<unsigned char>(*value))) {
    ++value;
  }
  if (*value == '\0') {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  out = std::strtof(value, &end);
  if (end == value || errno == ERANGE || !std::isfinite(out)) {
    return false;
  }
  while (std::isspace(static_cast<unsigned char>(*end))) {
    ++end;
  }
  return *end == '\0';
}

bool parseFloatListStrict(const char* value, float* values, size_t count) {
  if (!value || !values || count == 0) {
    return false;
  }

  const char* cursor = value;
  for (size_t i = 0; i < count; ++i) {
    while (std::isspace(static_cast<unsigned char>(*cursor))) {
      ++cursor;
    }
    if (*cursor == '\0') {
      return false;
    }

    errno = 0;
    char* end = nullptr;
    values[i] = std::strtof(cursor, &end);
    if (end == cursor || errno == ERANGE || !std::isfinite(values[i])) {
      return false;
    }
    cursor = end;

    if (i + 1 < count) {
      bool sawSeparator = false;
      while (std::isspace(static_cast<unsigned char>(*cursor))) {
        sawSeparator = true;
        ++cursor;
      }
      if (*cursor == ',' || *cursor == ';') {
        sawSeparator = true;
        ++cursor;
        while (std::isspace(static_cast<unsigned char>(*cursor))) {
          ++cursor;
        }
      }
      if (!sawSeparator) {
        return false;
      }
    }
  }

  while (std::isspace(static_cast<unsigned char>(*cursor))) {
    ++cursor;
  }
  return *cursor == '\0';
}

std::string toLowerAscii(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
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
  float parsed = fallback;
  return parseFloatStrict(rawValue.c_str(), parsed) ? parsed : fallback;
}

int parseIntValue(const std::string& rawValue, int fallback) {
  long parsed = 0;
  if (!parseLongStrict(rawValue.c_str(), 10, parsed) ||
      parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max()) {
    return fallback;
  }
  return static_cast<int>(parsed);
}

bool parseVec3Text(const std::string& value, glm::vec3& out) {
  float values[3]{};
  if (!parseFloatListStrict(value.c_str(), values, 3)) {
    return false;
  }
  out = glm::vec3(values[0], values[1], values[2]);
  return true;
}

bool parseVec4Text(const std::string& value, glm::vec4& out) {
  float values[4]{};
  if (!parseFloatListStrict(value.c_str(), values, 4)) {
    return false;
  }
  out = glm::vec4(values[0], values[1], values[2], values[3]);
  return true;
}

glm::vec3 parseVec3Value(const std::string& rawValue, const glm::vec3& fallback) {
  glm::vec3 parsed = fallback;
  return parseVec3Text(rawValue, parsed) ? parsed : fallback;
}

glm::vec4 parseVec4Value(const std::string& rawValue, const glm::vec4& fallback) {
  glm::vec4 parsed = fallback;
  return parseVec4Text(rawValue, parsed) ? parsed : fallback;
}

float normalizedDegreesForSettings(float degrees) {
  if (!std::isfinite(degrees)) {
    return 0.0f;
  }
  float wrapped = std::fmod(degrees, 360.0f);
  if (wrapped < 0.0f) {
    wrapped += 360.0f;
  }
  return wrapped;
}

template <typename T>
T clampValue(T value, T minValue, T maxValue) {
  return std::clamp(value, minValue, maxValue);
}

glm::vec3 clampVec3(const glm::vec3& value, float minValue, float maxValue) {
  return glm::vec3(
    clampValue(value.x, minValue, maxValue),
    clampValue(value.y, minValue, maxValue),
    clampValue(value.z, minValue, maxValue));
}

glm::vec4 clampVec4(const glm::vec4& value, float minValue, float maxValue) {
  return glm::vec4(
    clampValue(value.x, minValue, maxValue),
    clampValue(value.y, minValue, maxValue),
    clampValue(value.z, minValue, maxValue),
    clampValue(value.w, minValue, maxValue));
}

} // namespace

bool parsePortStrict(const std::string& value, int& port) {
  long parsed = 0;
  if (!parseLongStrict(value.c_str(), 10, parsed) || parsed <= 0 || parsed > 65535) {
    return false;
  }
  port = static_cast<int>(parsed);
  return true;
}

std::string normalizeConnectionHost(const std::string& value) {
  std::string host = trimAscii(value);
  if (host.size() >= 2 && host.front() == '[' && host.back() == ']') {
    host = trimAscii(host.substr(1, host.size() - 2));
  }
  if (host.empty()) {
    return {};
  }
  const bool hasInvalidChar = std::any_of(host.begin(), host.end(), [](unsigned char c) {
    return std::iscntrl(c) || std::isspace(c);
  });
  return hasInvalidChar ? std::string{} : host;
}

bool normalizeConnectionEndpoint(const std::string& host, int port, ConnectionEntry& entry) {
  if (port <= 0 || port > 65535) {
    return false;
  }
  const std::string normalizedHost = normalizeConnectionHost(host);
  if (normalizedHost.empty()) {
    return false;
  }
  entry.host = normalizedHost;
  entry.port = port;
  return true;
}

std::string formatEndpointHost(const std::string& host) {
  if (host.find(':') != std::string::npos &&
      !(host.size() >= 2 && host.front() == '[' && host.back() == ']')) {
    return "[" + host + "]";
  }
  return host;
}

std::string formatConnectionLabel(const ConnectionEntry& entry) {
  if (entry.host.empty()) {
    return {};
  }
  return formatEndpointHost(entry.host) + ":" + std::to_string(entry.port);
}

bool parseConnectionLabel(const std::string& value, ConnectionEntry& entry) {
  const std::string trimmed = trimAscii(value);
  if (trimmed.empty()) {
    return false;
  }

  std::string hostPart;
  std::string portPart;
  if (trimmed.front() == '[') {
    const auto close = trimmed.find(']');
    if (close == std::string::npos || close + 2 > trimmed.size() || trimmed[close + 1] != ':') {
      return false;
    }
    hostPart = trimmed.substr(1, close - 1);
    portPart = trimmed.substr(close + 2);
  } else {
    const auto sep = trimmed.rfind(':');
    if (sep == std::string::npos || sep == 0 || sep + 1 >= trimmed.size()) {
      return false;
    }
    hostPart = trimmed.substr(0, sep);
    if (hostPart.find(':') != std::string::npos) {
      return false;
    }
    portPart = trimmed.substr(sep + 1);
  }

  int parsedPort = 0;
  if (!parsePortStrict(trimAscii(portPart), parsedPort)) {
    return false;
  }

  return normalizeConnectionEndpoint(hostPart, parsedPort, entry);
}

void recordConnection(std::vector<ConnectionEntry>& connections, const std::string& host, int port) {
  ConnectionEntry normalized;
  if (!normalizeConnectionEndpoint(host, port, normalized)) {
    return;
  }
  connections.erase(
    std::remove_if(connections.begin(), connections.end(),
      [&](const ConnectionEntry& entry) {
        return entry.host == normalized.host && entry.port == normalized.port;
      }),
    connections.end());
  connections.insert(connections.begin(), normalized);
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

float sanitizeTcpUpdateRateHz(float value) {
  if (!std::isfinite(value)) {
    return kTcpUpdateRateDefaultHz;
  }
  return std::clamp(value, kTcpUpdateRateMinHz, kTcpUpdateRateMaxHz);
}

std::chrono::steady_clock::duration tcpUpdatePeriodForHz(float rateHz) {
  const double seconds = 1.0 / static_cast<double>(sanitizeTcpUpdateRateHz(rateHz));
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(seconds));
}

bool consumeTcpUpdateSlot(std::chrono::steady_clock::time_point now,
                          std::chrono::steady_clock::time_point& nextRequestTime,
                          float rateHz) {
  if (now < nextRequestTime) {
    return false;
  }
  nextRequestTime = now + tcpUpdatePeriodForHz(rateHz);
  return true;
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

int cloudQualityIndexFromName(const std::string& rawValue, int fallback) {
  const std::string value = toLowerAscii(trimAscii(rawValue));
  if (value == "auto" || value == "0") return 0;
  if (value == "off" || value == "none" || value == "1") return 1;
  if (value == "texture" || value == "2d" || value == "fast" || value == "2") return 2;
  if (value == "volumetric" || value == "3d" || value == "ultra" || value == "3") return 3;
  return fallback;
}

const char* cloudQualityName(int index) {
  switch (std::clamp(index, 0, 3)) {
    case 0: return "auto";
    case 1: return "off";
    case 2: return "texture";
    case 3: return "volumetric";
  }
  return "auto";
}

const char* weatherPresetName(int preset) {
  static constexpr const char* kNames[] = {
    "Clear", "Hazy", "Overcast", "Fog", "Rain", "Heavy Rain",
    "Snow", "Storm", "Night Clear", "Night Rain", "Custom"};
  return kNames[std::clamp(preset, 0, 10)];
}

const char* weatherQualityName(int quality) {
  static constexpr const char* kNames[] = {"Low", "Medium", "High", "Ultra"};
  return kNames[std::clamp(quality, 0, 3)];
}

int weatherPresetIndexFromName(const std::string& rawValue, int fallback) {
  const std::string value = toLowerAscii(trimAscii(rawValue));
  if (value == "clear" || value == "0") return 0;
  if (value == "hazy" || value == "1") return 1;
  if (value == "overcast" || value == "2") return 2;
  if (value == "fog" || value == "3") return 3;
  if (value == "rain" || value == "4") return 4;
  if (value == "heavy_rain" || value == "heavy rain" || value == "5") return 5;
  if (value == "snow" || value == "6") return 6;
  if (value == "storm" || value == "7") return 7;
  if (value == "night_clear" || value == "night clear" || value == "8") return 8;
  if (value == "night_rain" || value == "night rain" || value == "9") return 9;
  if (value == "custom" || value == "10") return 10;
  return fallback;
}

int weatherQualityIndexFromName(const std::string& rawValue, int fallback) {
  const std::string value = toLowerAscii(trimAscii(rawValue));
  if (value == "low" || value == "0") return 0;
  if (value == "medium" || value == "1") return 1;
  if (value == "high" || value == "2") return 2;
  if (value == "ultra" || value == "3") return 3;
  return fallback;
}

bool weatherDefaultEnabledForQuality(int quality) {
  const int clampedQuality = std::clamp(quality, 0, 4);
  return clampedQuality >= 2;
}

bool highFidelityPbrAllowedForQuality(int quality) {
  const int clampedQuality = std::clamp(quality, 0, 4);
  return clampedQuality >= 2;
}

void sanitizeViewerSettings(ViewerSettings& settings) {
  settings.renderQuality = clampValue(settings.renderQuality, 0, 4);
  settings.backgroundColorRgb255 = clampVec4(settings.backgroundColorRgb255, 0.0f, 255.0f);
  settings.mainLightAmbient = clampVec3(settings.mainLightAmbient, 0.0f, 4.0f);
  settings.mainLightDiffuse = clampVec3(settings.mainLightDiffuse, 0.0f, 4.0f);
  settings.mainLightSpecular = clampVec3(settings.mainLightSpecular, 0.0f, 4.0f);
  settings.cameraSpeed = clampValue(settings.cameraSpeed, 0.1f, 30.0f);
  settings.cameraFovDeg = clampValue(settings.cameraFovDeg, 20.0f, 100.0f);
  settings.cameraNear = clampValue(settings.cameraNear, 0.001f, 1.0f);
  settings.cameraFar = clampValue(settings.cameraFar, 10.0f, 5000.0f);
  if (settings.cameraFar <= settings.cameraNear) {
    settings.cameraFar = std::min(5000.0f, std::max(10.0f, settings.cameraNear * 10.0f));
  }
  settings.lightYawDeg = clampValue(settings.lightYawDeg, -180.0f, 180.0f);
  settings.lightPitchDeg = clampValue(settings.lightPitchDeg, -89.0f, 89.0f);
  settings.lightStrength = clampValue(settings.lightStrength, 0.0f, 2.0f);
  settings.ambientStrength = clampValue(settings.ambientStrength, 0.0f, 2.0f);
  settings.shadowResolution = clampValue(settings.shadowResolution, 64, 8192);
  settings.shadowBias = clampValue(settings.shadowBias, 0.0f, 0.01f);
  settings.shadowStrength = clampValue(settings.shadowStrength, 0.0f, 1.0f);
  settings.shadowPcfRadius = clampValue(settings.shadowPcfRadius, 0.0f, 4.0f);
  settings.shadowOrthoHalfSize = clampValue(settings.shadowOrthoHalfSize, 1.0f, 100.0f);
  settings.shadowNear = clampValue(settings.shadowNear, 0.01f, 10.0f);
  settings.shadowFar = clampValue(settings.shadowFar, 1.0f, 250.0f);
  if (settings.shadowFar <= settings.shadowNear) {
    settings.shadowFar = std::min(250.0f, settings.shadowNear + 1.0f);
  }
  settings.shadowCenterOffset = clampValue(settings.shadowCenterOffset, 0.0f, 80.0f);
  settings.fogDensity = clampValue(settings.fogDensity, 0.0f, 0.08f);
  settings.gamma = clampValue(settings.gamma, 0.5f, 2.5f);
  settings.colorMode = clampValue(settings.colorMode, 0, 2);
  settings.bloomThreshold = clampValue(settings.bloomThreshold, 0.0f, 4.0f);
  settings.bloomStrength = clampValue(settings.bloomStrength, 0.0f, 2.0f);
  settings.bloomRadius = clampValue(settings.bloomRadius, 0.0f, 12.0f);
  settings.bloomKnee = clampValue(settings.bloomKnee, 0.0f, 1.0f);
  settings.bloomQuality = clampValue(settings.bloomQuality, 0, 3);
  settings.screenSpaceAoRadius = clampValue(settings.screenSpaceAoRadius, 0.05f, 10.0f);
  settings.screenSpaceAoStrength = clampValue(settings.screenSpaceAoStrength, 0.0f, 4.0f);
  settings.screenSpaceAoBias = clampValue(settings.screenSpaceAoBias, 0.0f, 0.25f);
  settings.depthOfFieldFocusDistance = clampValue(settings.depthOfFieldFocusDistance, 0.05f, 30.0f);
  settings.depthOfFieldFocusRange = clampValue(settings.depthOfFieldFocusRange, 1.0f, 100.0f);
  settings.depthOfFieldMaxRadius = clampValue(settings.depthOfFieldMaxRadius, 0.0f, 8.0f);
  settings.pbrExposure = clampValue(settings.pbrExposure, 0.1f, 4.0f);
  settings.pbrEnvironmentMaxLod = clampValue(settings.pbrEnvironmentMaxLod, 0.0f, 12.0f);
  settings.pbrEnvironmentIntensity = clampValue(settings.pbrEnvironmentIntensity, 0.0f, 4.0f);
  settings.pbrKeyLightIntensity = clampValue(settings.pbrKeyLightIntensity, 0.0f, 4.0f);
  if (!highFidelityPbrAllowedForQuality(settings.renderQuality)) {
    settings.highFidelityPbr = false;
  }
  settings.skySunStrength = clampValue(settings.skySunStrength, 0.0f, 8.0f);
  settings.skySunSize = clampValue(settings.skySunSize, 0.001f, 0.08f);
  if (!weatherDefaultEnabledForQuality(settings.renderQuality)) {
    settings.skyWeatherEnabled = false;
  }
  settings.skyWeatherPreset = clampValue(settings.skyWeatherPreset, 0, 10);
  settings.skyWeatherQuality = clampValue(settings.skyWeatherQuality, 0, 3);
  settings.skyWeatherSeed = clampValue(settings.skyWeatherSeed, 1, 1000000);
  settings.skyTimeOfDayHours = clampValue(settings.skyTimeOfDayHours, 0.0f, 24.0f);
  settings.skyLatitude = clampValue(settings.skyLatitude, -89.9f, 89.9f);
  settings.skyLongitude = clampValue(settings.skyLongitude, -180.0f, 180.0f);
  settings.skyYear = clampValue(settings.skyYear, 1900, 2500);
  settings.skyMonth = clampValue(settings.skyMonth, 1, 12);
  settings.skyDay = clampValue(settings.skyDay, 1, 31);
  settings.skyWindDirectionDeg = normalizedDegreesForSettings(settings.skyWindDirectionDeg);
  settings.skyWindSpeed = clampValue(settings.skyWindSpeed, 0.0f, 80.0f);
  settings.skyCloudCoverage = clampValue(settings.skyCloudCoverage, 0.0f, 1.0f);
  settings.skyCloudDensity = clampValue(settings.skyCloudDensity, 0.0f, 1.0f);
  settings.skyCloudAltitudeMeters = clampValue(settings.skyCloudAltitudeMeters, 20.0f, 12000.0f);
  settings.skyCloudThicknessMeters = clampValue(settings.skyCloudThicknessMeters, 1.0f, 4000.0f);
  settings.skyCloudShadowStrength = clampValue(settings.skyCloudShadowStrength, 0.0f, 1.0f);
  settings.skyCloudScale = clampValue(settings.skyCloudScale, 0.01f, 2.0f);
  settings.skyCloudAnimationSpeed = clampValue(settings.skyCloudAnimationSpeed, 0.0f, 200.0f);
  settings.skyCloudQuality = clampValue(settings.skyCloudQuality, 0, 3);
  settings.skyPrecipitationRate = clampValue(settings.skyPrecipitationRate, 0.0f, 1.0f);
  settings.skyRainOcclusionStrength = clampValue(settings.skyRainOcclusionStrength, 0.0f, 1.0f);
  settings.skySnowCoverage = clampValue(settings.skySnowCoverage, 0.0f, 1.0f);
  settings.skyHumidity = clampValue(settings.skyHumidity, 0.0f, 1.0f);
  settings.skyWetness = clampValue(settings.skyWetness, 0.0f, 1.0f);
  settings.skyWetnessAccumulationRate = clampValue(settings.skyWetnessAccumulationRate, 0.0f, 4.0f);
  settings.skyWetnessDryingRate = clampValue(settings.skyWetnessDryingRate, 0.0f, 4.0f);
  settings.skyLightningRate = clampValue(settings.skyLightningRate, 0.0f, 16.0f);
  settings.skyFogDensity = clampValue(settings.skyFogDensity, 0.0f, 1.0f);
  settings.skyVisibilityMeters = clampValue(settings.skyVisibilityMeters, 1.0f, 100000.0f);
  settings.skyFogColor = clampVec3(settings.skyFogColor, 0.0f, 4.0f);
  settings.skyFogAnisotropy = clampValue(settings.skyFogAnisotropy, -0.85f, 0.85f);
  settings.skyAirTurbidity = clampValue(settings.skyAirTurbidity, 1.0f, 12.0f);
  settings.skyGroundAlbedo = clampValue(settings.skyGroundAlbedo, 0.0f, 1.0f);
  settings.skySunAzimuthDeg = normalizedDegreesForSettings(settings.skySunAzimuthDeg);
  settings.skySunElevationDeg = clampValue(settings.skySunElevationDeg, -8.0f, 89.0f);
  settings.skyMoonSize = clampValue(settings.skyMoonSize, 0.001f, 0.08f);
  settings.skyLensDropletStrength = clampValue(settings.skyLensDropletStrength, 0.0f, 1.0f);
  settings.reflectiveGroundRoughness = clampValue(settings.reflectiveGroundRoughness, 0.02f, 1.0f);
  settings.reflectiveGroundMetallic = clampValue(settings.reflectiveGroundMetallic, 0.0f, 1.0f);
  settings.shadowedLightBudget = clampValue(settings.shadowedLightBudget, 0, 8);
  settings.maxPointShadowLights = clampValue(settings.maxPointShadowLights, 0, 8);
  settings.additionalShadowResolutionScale = clampValue(settings.additionalShadowResolutionScale, 0.05f, 2.0f);
  settings.pointShadowResolutionScale = clampValue(settings.pointShadowResolutionScale, 0.05f, 2.0f);
  settings.minAdditionalShadowResolution = clampValue(settings.minAdditionalShadowResolution, 64, 2048);
  settings.maxAdditionalLightsPerFrame = clampValue(settings.maxAdditionalLightsPerFrame, 0, 16);
  settings.minAdditionalLightInfluence = clampValue(settings.minAdditionalLightInfluence, 0.0f, 1.0f);
  settings.uiScale = clampValue(settings.uiScale, 0.8f, 2.6f);
  settings.tcpUpdateRateHz = sanitizeTcpUpdateRateHz(settings.tcpUpdateRateHz);
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
    else if (key == "render_quality_user_set") settings.renderQualityUserSet = parseBoolValue(value, settings.renderQualityUserSet);
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
    else if (key == "sky_enabled") settings.skyEnabled = parseBoolValue(value, settings.skyEnabled);
    else if (key == "sky_sun_strength") settings.skySunStrength = parseFloatValue(value, settings.skySunStrength);
    else if (key == "sky_sun_size") settings.skySunSize = parseFloatValue(value, settings.skySunSize);
    else if (key == "sky_weather_enabled") settings.skyWeatherEnabled = parseBoolValue(value, settings.skyWeatherEnabled);
    else if (key == "sky_weather_preset") settings.skyWeatherPreset = weatherPresetIndexFromName(value, settings.skyWeatherPreset);
    else if (key == "sky_weather_quality") settings.skyWeatherQuality = weatherQualityIndexFromName(value, settings.skyWeatherQuality);
    else if (key == "sky_weather_seed") settings.skyWeatherSeed = parseIntValue(value, settings.skyWeatherSeed);
    else if (key == "sky_time_of_day_hours") settings.skyTimeOfDayHours = parseFloatValue(value, settings.skyTimeOfDayHours);
    else if (key == "sky_latitude") settings.skyLatitude = parseFloatValue(value, settings.skyLatitude);
    else if (key == "sky_longitude") settings.skyLongitude = parseFloatValue(value, settings.skyLongitude);
    else if (key == "sky_year") settings.skyYear = parseIntValue(value, settings.skyYear);
    else if (key == "sky_month") settings.skyMonth = parseIntValue(value, settings.skyMonth);
    else if (key == "sky_day") settings.skyDay = parseIntValue(value, settings.skyDay);
    else if (key == "sky_wind_direction_deg") settings.skyWindDirectionDeg = parseFloatValue(value, settings.skyWindDirectionDeg);
    else if (key == "sky_wind_speed") settings.skyWindSpeed = parseFloatValue(value, settings.skyWindSpeed);
    else if (key == "sky_cloud_coverage") settings.skyCloudCoverage = parseFloatValue(value, settings.skyCloudCoverage);
    else if (key == "sky_cloud_density") settings.skyCloudDensity = parseFloatValue(value, settings.skyCloudDensity);
    else if (key == "sky_cloud_altitude_m") settings.skyCloudAltitudeMeters = parseFloatValue(value, settings.skyCloudAltitudeMeters);
    else if (key == "sky_cloud_thickness_m") settings.skyCloudThicknessMeters = parseFloatValue(value, settings.skyCloudThicknessMeters);
    else if (key == "sky_cloud_shadow_strength") settings.skyCloudShadowStrength = parseFloatValue(value, settings.skyCloudShadowStrength);
    else if (key == "sky_cloud_scale") settings.skyCloudScale = parseFloatValue(value, settings.skyCloudScale);
    else if (key == "sky_cloud_animation_speed") settings.skyCloudAnimationSpeed = parseFloatValue(value, settings.skyCloudAnimationSpeed);
    else if (key == "sky_cloud_quality") settings.skyCloudQuality = cloudQualityIndexFromName(value, settings.skyCloudQuality);
    else if (key == "sky_precipitation_rate") settings.skyPrecipitationRate = parseFloatValue(value, settings.skyPrecipitationRate);
    else if (key == "sky_rain_occlusion_strength") settings.skyRainOcclusionStrength = parseFloatValue(value, settings.skyRainOcclusionStrength);
    else if (key == "sky_snow_coverage") settings.skySnowCoverage = parseFloatValue(value, settings.skySnowCoverage);
    else if (key == "sky_humidity") settings.skyHumidity = parseFloatValue(value, settings.skyHumidity);
    else if (key == "sky_wetness") settings.skyWetness = parseFloatValue(value, settings.skyWetness);
    else if (key == "sky_wetness_accumulation") settings.skyWetnessAccumulationEnabled = parseBoolValue(value, settings.skyWetnessAccumulationEnabled);
    else if (key == "sky_wetness_accumulation_rate") settings.skyWetnessAccumulationRate = parseFloatValue(value, settings.skyWetnessAccumulationRate);
    else if (key == "sky_wetness_drying_rate") settings.skyWetnessDryingRate = parseFloatValue(value, settings.skyWetnessDryingRate);
    else if (key == "sky_lightning_rate") settings.skyLightningRate = parseFloatValue(value, settings.skyLightningRate);
    else if (key == "sky_fog_density") settings.skyFogDensity = parseFloatValue(value, settings.skyFogDensity);
    else if (key == "sky_visibility_m") settings.skyVisibilityMeters = parseFloatValue(value, settings.skyVisibilityMeters);
    else if (key == "sky_fog_color") settings.skyFogColor = parseVec3Value(value, settings.skyFogColor);
    else if (key == "sky_fog_anisotropy") settings.skyFogAnisotropy = parseFloatValue(value, settings.skyFogAnisotropy);
    else if (key == "sky_air_turbidity") settings.skyAirTurbidity = parseFloatValue(value, settings.skyAirTurbidity);
    else if (key == "sky_ground_albedo") settings.skyGroundAlbedo = parseFloatValue(value, settings.skyGroundAlbedo);
    else if (key == "sky_use_explicit_sun_angles") settings.skyUseExplicitSunAngles = parseBoolValue(value, settings.skyUseExplicitSunAngles);
    else if (key == "sky_sun_azimuth_deg") settings.skySunAzimuthDeg = parseFloatValue(value, settings.skySunAzimuthDeg);
    else if (key == "sky_sun_elevation_deg") settings.skySunElevationDeg = parseFloatValue(value, settings.skySunElevationDeg);
    else if (key == "sky_moon_size") settings.skyMoonSize = parseFloatValue(value, settings.skyMoonSize);
    else if (key == "sky_lens_droplets_enabled") settings.skyLensDropletsEnabled = parseBoolValue(value, settings.skyLensDropletsEnabled);
    else if (key == "sky_lens_droplet_strength") settings.skyLensDropletStrength = parseFloatValue(value, settings.skyLensDropletStrength);
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
    else if (key == "show_collapsed_logo") settings.showCollapsedLogo = parseBoolValue(value, settings.showCollapsedLogo);
    else if (key == "tcp_update_rate_hz") settings.tcpUpdateRateHz = parseFloatValue(value, settings.tcpUpdateRateHz);
    else if (key == "recent_connection") {
      ConnectionEntry entry;
      if (parseConnectionLabel(value, entry)) {
        recordConnection(settings.recentConnections, entry.host, entry.port);
      }
    }
    else if (key == "resource_dir") {
      if (!value.empty()) {
        recordResourceDir(settings.resourceDirs, value);
      }
    }
  }
  sanitizeViewerSettings(settings);
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
  output << "render_quality_user_set: " << settings.renderQualityUserSet << "\n";
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
  output << "sky_enabled: " << settings.skyEnabled << "\n";
  output << "sky_sun_strength: " << settings.skySunStrength << "\n";
  output << "sky_sun_size: " << settings.skySunSize << "\n";
  output << "sky_weather_enabled: " << settings.skyWeatherEnabled << "\n";
  output << "sky_weather_preset: " << weatherPresetName(settings.skyWeatherPreset) << "\n";
  output << "sky_weather_quality: " << weatherQualityName(settings.skyWeatherQuality) << "\n";
  output << "sky_weather_seed: " << settings.skyWeatherSeed << "\n";
  output << "sky_time_of_day_hours: " << settings.skyTimeOfDayHours << "\n";
  output << "sky_latitude: " << settings.skyLatitude << "\n";
  output << "sky_longitude: " << settings.skyLongitude << "\n";
  output << "sky_year: " << settings.skyYear << "\n";
  output << "sky_month: " << settings.skyMonth << "\n";
  output << "sky_day: " << settings.skyDay << "\n";
  output << "sky_wind_direction_deg: " << settings.skyWindDirectionDeg << "\n";
  output << "sky_wind_speed: " << settings.skyWindSpeed << "\n";
  output << "sky_cloud_coverage: " << settings.skyCloudCoverage << "\n";
  output << "sky_cloud_density: " << settings.skyCloudDensity << "\n";
  output << "sky_cloud_altitude_m: " << settings.skyCloudAltitudeMeters << "\n";
  output << "sky_cloud_thickness_m: " << settings.skyCloudThicknessMeters << "\n";
  output << "sky_cloud_shadow_strength: " << settings.skyCloudShadowStrength << "\n";
  output << "sky_cloud_scale: " << settings.skyCloudScale << "\n";
  output << "sky_cloud_animation_speed: " << settings.skyCloudAnimationSpeed << "\n";
  output << "sky_cloud_quality: " << cloudQualityName(settings.skyCloudQuality) << "\n";
  output << "sky_precipitation_rate: " << settings.skyPrecipitationRate << "\n";
  output << "sky_rain_occlusion_strength: " << settings.skyRainOcclusionStrength << "\n";
  output << "sky_snow_coverage: " << settings.skySnowCoverage << "\n";
  output << "sky_humidity: " << settings.skyHumidity << "\n";
  output << "sky_wetness: " << settings.skyWetness << "\n";
  output << "sky_wetness_accumulation: " << settings.skyWetnessAccumulationEnabled << "\n";
  output << "sky_wetness_accumulation_rate: " << settings.skyWetnessAccumulationRate << "\n";
  output << "sky_wetness_drying_rate: " << settings.skyWetnessDryingRate << "\n";
  output << "sky_lightning_rate: " << settings.skyLightningRate << "\n";
  output << "sky_fog_density: " << settings.skyFogDensity << "\n";
  output << "sky_visibility_m: " << settings.skyVisibilityMeters << "\n";
  output << "sky_fog_color: " << settings.skyFogColor.r << ", "
         << settings.skyFogColor.g << ", " << settings.skyFogColor.b << "\n";
  output << "sky_fog_anisotropy: " << settings.skyFogAnisotropy << "\n";
  output << "sky_air_turbidity: " << settings.skyAirTurbidity << "\n";
  output << "sky_ground_albedo: " << settings.skyGroundAlbedo << "\n";
  output << "sky_use_explicit_sun_angles: " << settings.skyUseExplicitSunAngles << "\n";
  output << "sky_sun_azimuth_deg: " << settings.skySunAzimuthDeg << "\n";
  output << "sky_sun_elevation_deg: " << settings.skySunElevationDeg << "\n";
  output << "sky_moon_size: " << settings.skyMoonSize << "\n";
  output << "sky_lens_droplets_enabled: " << settings.skyLensDropletsEnabled << "\n";
  output << "sky_lens_droplet_strength: " << settings.skyLensDropletStrength << "\n";
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
  output << "show_collapsed_logo: " << settings.showCollapsedLogo << "\n";
  output << "tcp_update_rate_hz: " << settings.tcpUpdateRateHz << "\n";
  for (const auto& entry : settings.recentConnections) {
    output << "recent_connection: " << formatConnectionLabel(entry) << "\n";
  }
  for (const auto& dir : settings.resourceDirs) {
    output << "resource_dir: " << dir << "\n";
  }
}

} // namespace raisin::tcp_viewer
