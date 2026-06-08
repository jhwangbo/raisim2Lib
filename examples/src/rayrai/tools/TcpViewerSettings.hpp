// Copyright (c) 2026 Raion Robotics Inc.
// All rights reserved.

#pragma once

#include <chrono>
#include <filesystem>
#include <string>
#include <vector>

#include "rayrai/RaisimTcpCommon.hpp"
#include "rayrai/RenderQuality.hpp"

#include <glm/glm.hpp>

namespace raisin::tcp_viewer
{

inline constexpr float kTcpUpdateRateDefaultHz = 60.0f;
inline constexpr float kTcpUpdateRateMinHz = 15.0f;
inline constexpr float kTcpUpdateRateMaxHz = 120.0f;

struct ConnectionEntry {
  std::string host;
  int port = kDefaultPort;
};

struct ViewerSettings {
  int renderQuality = 1;
  bool renderQualityUserSet = false;
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
  int colorMode = static_cast<int>(raisin::ViewerColorMode::FastLinear);
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
  bool skyEnabled = true;
  float skySunStrength = 1.8f;
  float skySunSize = 0.015f;
  bool skyWeatherEnabled = false;
  int skyWeatherPreset = 0;
  int skyWeatherQuality = 2;
  int skyWeatherSeed = 1;
  float skyTimeOfDayHours = 13.0f;
  float skyLatitude = 37.0f;
  float skyLongitude = 127.0f;
  int skyYear = 2026;
  int skyMonth = 5;
  int skyDay = 8;
  float skyWindDirectionDeg = 14.0f;
  float skyWindSpeed = 1.5f;
  float skyCloudCoverage = 0.05f;
  float skyCloudDensity = 0.05f;
  float skyCloudAltitudeMeters = 850.0f;
  float skyCloudThicknessMeters = 180.0f;
  float skyCloudShadowStrength = 0.04f;
  float skyCloudScale = 0.18f;
  float skyCloudAnimationSpeed = 0.0f;
  // skyCloudQuality: 0 = Auto (driven by render preset + weather),
  //                  1 = Off, 2 = Texture (cheap 2D), 3 = Volumetric (raymarched).
  int skyCloudQuality = 0;
  float skyPrecipitationRate = 0.0f;
  float skyRainOcclusionStrength = 0.0f;
  float skySnowCoverage = 0.0f;
  float skyHumidity = 0.35f;
  float skyWetness = 0.0f;
  bool skyWetnessAccumulationEnabled = true;
  float skyWetnessAccumulationRate = 0.35f;
  float skyWetnessDryingRate = 0.10f;
  float skyLightningRate = 0.0f;
  float skyFogDensity = 0.0f;
  float skyVisibilityMeters = 10000.0f;
  glm::vec3 skyFogColor{0.72f, 0.80f, 0.90f};
  float skyFogAnisotropy = 0.0f;
  float skyAirTurbidity = 2.0f;
  float skyGroundAlbedo = 0.35f;
  bool skyUseExplicitSunAngles = false;
  float skySunAzimuthDeg = 180.0f;
  float skySunElevationDeg = 42.0f;
  float skyMoonSize = 0.014f;
  bool skyLensDropletsEnabled = false;
  float skyLensDropletStrength = 1.0f;
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
  bool showCollapsedLogo = true;
  float tcpUpdateRateHz = kTcpUpdateRateDefaultHz;
  std::vector<ConnectionEntry> recentConnections;
  std::vector<std::string> resourceDirs;
};

bool parsePortStrict(const std::string& value, int& port);
std::string normalizeConnectionHost(const std::string& value);
bool normalizeConnectionEndpoint(const std::string& host, int port, ConnectionEntry& entry);
std::string formatEndpointHost(const std::string& host);
std::string formatConnectionLabel(const ConnectionEntry& entry);
bool parseConnectionLabel(const std::string& value, ConnectionEntry& entry);
void recordConnection(std::vector<ConnectionEntry>& connections, const std::string& host, int port);
void recordResourceDir(std::vector<std::string>& dirs, const std::string& path);

float sanitizeTcpUpdateRateHz(float value);
std::chrono::steady_clock::duration tcpUpdatePeriodForHz(float rateHz);
bool consumeTcpUpdateSlot(std::chrono::steady_clock::time_point now,
                          std::chrono::steady_clock::time_point& nextRequestTime,
                          float rateHz);

const char* qualityName(int quality);
const char* colorModeName(int colorMode);
int qualityIndexFromName(const std::string& rawValue, int fallback);
int colorModeIndexFromName(const std::string& rawValue, int fallback);
int cloudQualityIndexFromName(const std::string& rawValue, int fallback);
const char* cloudQualityName(int index);
const char* weatherPresetName(int preset);
const char* weatherQualityName(int quality);
int weatherPresetIndexFromName(const std::string& rawValue, int fallback);
int weatherQualityIndexFromName(const std::string& rawValue, int fallback);
bool weatherDefaultEnabledForQuality(int quality);
bool highFidelityPbrAllowedForQuality(int quality);
void sanitizeViewerSettings(ViewerSettings& settings);
std::filesystem::path settingsFilePath();
void loadViewerSettings(ViewerSettings& settings);
void saveViewerSettings(const ViewerSettings& settings);

} // namespace raisin::tcp_viewer
