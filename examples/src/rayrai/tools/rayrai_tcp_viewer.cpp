// Copyright (c) 2025 Raion Robotics Inc.
// All rights reserved.

#include <SDL.h>

#include <glbinding/glbinding.h>
#include <glbinding/gl/gl.h>
#include <imgui/imgui.h>

#include "imgui/backend/imgui_impl_opengl3.h"
#include "imgui/backend/imgui_impl_sdl2.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cfloat>
#include <cerrno>
#include <chrono>
#include <clocale>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <locale>
#include <sstream>
#include <string>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"
#include "stb/stb_image.h"

#include "rayrai/RayraiWindow.hpp"
#include "rayrai/Visuals.hpp"
#include "rayrai/OpenGLMesh.hpp"
#include "rayrai/CoordinateFrame.hpp"
#include "rayrai/RaisimTcpCommon.hpp"
#include "rayrai/raisin_imgui_style.h"
#include "raisim/configure.hpp"
#include "raisim/sensors/Sensors.hpp"
#include "raisim/World.hpp"
#include "raisim/object/ArticulatedSystem/ArticulatedSystem.hpp"
#include "raisim/object/ArticulatedSystem/JointAndBodies.hpp"

#if defined(__linux__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <fcntl.h>
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
constexpr int kDiscoveryPort = 59312;
constexpr const char* kDiscoveryMagic = "RAISIM_TCP_DISCOVERY_V1";
constexpr int kConnectTimeoutMs = 2000;
constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
constexpr float kRadToDeg = 180.0f / 3.14159265358979323846f;
constexpr auto kAutoConnectInterval = std::chrono::seconds(3);
constexpr auto kDiscoveryBeaconTimeout = std::chrono::seconds(8);
constexpr auto kOverlayAutoCollapseDelay = std::chrono::seconds(5);
constexpr auto kSettingsSaveDebounce = std::chrono::milliseconds(750);
constexpr int kTransferRateGraphBuckets = 60;
constexpr double kTransferRateGraphWindowSeconds = 30.0;
constexpr float kBaseFontSize = 24.0f;
constexpr float kFontScale = 0.75f;
constexpr float kDefaultFontRasterizerDensity = 1.75f;
constexpr float kUiScaleEpsilon = 0.01f;
constexpr const char* kRobotoFontRelativePath = "rsc/fonts/roboto/Roboto-Medium.ttf";
constexpr const char* kSessionMagic = "RAYRAI_TCP_VIEWER_SESSION_V1\n";

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

struct ServerEntry {
  ConnectionEntry endpoint;
  std::string bindHost;
  std::string process;
  std::string protocol;
  std::unordered_map<std::string, std::string> metadata;
  bool remoteBeacon = false;
  std::chrono::steady_clock::time_point lastSeen;
};

struct ViewerViewportState {
  ImVec2 origin{0.0f, 0.0f};
  ImVec2 size{0.0f, 0.0f};
  bool hovered = false;
  int cursorX = 0;
  int cursorY = 0;
};

struct MouseForceGesture {
  bool active = false;
  uint32_t tag = 0;
  int index = 0;
  int localBodyIdx = 0;
  glm::vec3 applicationPoint{0.0f};
  glm::vec3 force{0.0f};
  ImVec2 pressMouse{0.0f, 0.0f};
  ImVec2 currentMouse{0.0f, 0.0f};
  size_t pendingRequestIndex = std::numeric_limits<size_t>::max();
};

struct RulerToolState {
  bool enabled = false;
  bool hasA = false;
  bool hasB = false;
  int nextPoint = 0;
  glm::vec3 a{0.0f};
  glm::vec3 b{0.0f};
  std::string aLabel;
  std::string bLabel;
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
  std::vector<ConnectionEntry> recentConnections;
  std::vector<std::string> resourceDirs;
};

struct ProgramOptions {
  std::string host = "127.0.0.1";
  int port = kDefaultPort;
  bool autoConnect = true;
  bool autoConnectSet = false;
  bool minimizePanels = false;
  bool minimizePanelsSet = false;
  bool keepOverlayOpen = false;  // --keep-overlay-open: never auto-collapse the left panel
  bool autoFrame = false;
  bool autoFrameSet = false;
  bool fullscreen = false;
  int windowWidth = 1280;
  int windowHeight = 720;
  bool hasCameraLookAt = false;
  bool forceCameraLookAt = false;
  glm::vec3 cameraPos{0.0f};
  glm::vec3 cameraTarget{0.0f};
  bool hasTargetOffset = false;
  glm::vec3 targetOffset{0.0f};
  std::vector<std::string> resourceDirs;
  std::filesystem::path screenshotPath;
  std::filesystem::path screenshotDir = std::filesystem::current_path();
  std::filesystem::path recordSessionPath;
  std::filesystem::path replaySessionPath;
  std::filesystem::path exportScenePath;
  std::filesystem::path trajectoryCsvPath;
  std::filesystem::path endpointListPath;
  std::filesystem::path inspectorPath; // --inspect FILE: load URDF/MJCF as if drag-dropped
  // --inspect-after-frames N: defer the --inspect load until the main loop has
  // ticked N frames. Lets the headless harness measure drag-drop latency after
  // the background shader warmup has had time to run.
  int inspectAfterFrames = -1;
  bool preWarmShaders = true;          // --no-pre-warm to skip the targeted shader pre-compile
  // --warm-at-startup: drive one content render at startup with a transient AS to
  // warm the renderer's non-shader lazy init (IBL convolution / FBOs / texture pools).
  // Costs ~13 s at startup; makes every drag-drop after that ~30 ms. Default off so
  // empty-viewer launches stay fast.
  bool warmAtStartup = false;
  // --inspect-close-after-frames N: drive load → render N frames → close → render N → exit.
  // Headless reproducer for "close inspector segfaults" bug reports.
  int inspectCloseAfterFrames = -1;
  // --inspect-reload PATH: after the close phase, also reload this second path so we
  // can measure first-frame shader-compile cost across the two loads (proves that the
  // World/Renderer + compiled shaders are reused between drops).
  std::filesystem::path inspectReloadPath;
  bool exitAfterScreenshot = false;
  bool replayLoop = false;
  float replaySpeed = 1.0f;
  double waitForServerSeconds = 0.0;
  double exitAfterSeconds = 0.0;
  bool printHelp = false;
};

struct CameraBookmark {
  bool valid = false;
  glm::vec3 position{0.0f};
  glm::vec3 target{0.0f};
};

// Local "AS inspector" mode: activated when the user drags a URDF/MJCF onto the viewer
// while there is no TCP connection. The robot is loaded into the viewer's own
// raisim::World and posed with the joint sliders — no physics, no server, just kinematics.
struct InspectorJoint {
  std::string name;
  raisim::Joint::Type type = raisim::Joint::Type::FIXED;
  int gcOffset = 0;       // start index into the ArticulatedSystem GC vector
  int gcDim = 0;          // 0 (fixed) / 1 (rev/prismatic) / 4 (spherical) / 7 (floating)
  double minLimit = 0.0;  // for revolute/prismatic
  double maxLimit = 0.0;
  bool hasLimits = false;
};

struct InspectorState {
  bool active = false;
  std::string sourceFile;
  std::string lastError;
  raisim::ArticulatedSystem* as = nullptr;   // owned by the local raisim::World
  // Other raisim objects added by an MJCF load (ground plane, extra bodies, etc.) that
  // need to be removed when we close the inspector. Does not include `as`.
  std::vector<raisim::Object*> sideObjects;
  std::vector<double> gc;                    // mirror of the current generalized coordinates
  std::vector<InspectorJoint> joints;
  // True while the user is actively dragging a slider; suppresses auto-connect attempts.
  bool dragging = false;
};

// Sniff the first KiB of a file for the `<mujoco` root marker so we know whether to use
// raisim's URDF path or the MJCF world loader. Tolerates leading XML declarations and
// whitespace; falls back to false on any I/O error.
// Identify MJCF vs URDF by scanning the file for the first non-comment, non-declaration
// root element. The 1 KB short-read used to misclassify files like
// half_cheetah.xml whose <mujoco> tag lives after a long license/header comment
// (the comment + whitespace pushed it past byte 1024). We now read up to 64 KB —
// big enough for any realistic preamble — and skip past <?xml...?> declarations and
// <!-- ... --> comments before sniffing the root tag.
inline bool looksLikeMjcf(const std::string& path) {
  std::ifstream fs(path, std::ios::binary);
  if (!fs) return false;
  std::string buf(65536, '\0');
  fs.read(buf.data(), static_cast<std::streamsize>(buf.size()));
  buf.resize(static_cast<size_t>(fs.gcount()));
  size_t i = 0;
  const size_t n = buf.size();
  while (i < n) {
    while (i < n && std::isspace(static_cast<unsigned char>(buf[i]))) ++i;
    if (i + 1 < n && buf[i] == '<' && buf[i + 1] == '?') {
      const auto end = buf.find("?>", i + 2);
      if (end == std::string::npos) return false;
      i = end + 2;
      continue;
    }
    if (i + 3 < n && buf.compare(i, 4, "<!--") == 0) {
      const auto end = buf.find("-->", i + 4);
      if (end == std::string::npos) return false;
      i = end + 3;
      continue;
    }
    if (i + 8 < n && buf.compare(i, 9, "<!DOCTYPE") == 0) {
      const auto end = buf.find('>', i + 9);
      if (end == std::string::npos) return false;
      i = end + 1;
      continue;
    }
    if (i < n && buf[i] == '<') {
      return buf.compare(i, 7, "<mujoco") == 0;
    }
    ++i;
  }
  return false;
}

struct MotionEstimate {
  bool hasPrevious = false;
  bool valid = false;
  double previousTime = 0.0;
  glm::vec3 previousPosition{0.0f};
  glm::vec4 previousQuat{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 linearVelocity{0.0f};
  float angularSpeed = 0.0f;
};

struct ViewerStats {
  int frames = 0;
  int updates = 0;
  int parseErrors = 0;
  int reconnects = 0;
  int lastPayloadBytes = 0;
  size_t bytes = 0;
  double fps = 0.0;
  double updateHz = 0.0;
  double rxKbps = 0.0;
  int pendingSensorRequests = 0;
  size_t unresolvedAssets = 0;
  std::chrono::steady_clock::time_point windowStart = std::chrono::steady_clock::now();
};

struct PacketSample {
  double timeSeconds = 0.0;
  int bytes = 0;
  bool parsed = false;
  bool replay = false;
  int pendingSensors = 0;
  size_t objects = 0;
  size_t visuals = 0;
  size_t instanced = 0;
  size_t pointClouds = 0;
  size_t unresolvedAssets = 0;
};

struct AssetDiagnostic {
  uint32_t tag = 0;
  int index = 0;
  std::string name;
  std::string meshFile;
  std::string meshPath;
  std::string resourceDir;
  bool resolved = false;
};

struct RecordedFrame {
  uint64_t timeMicros = 0;
  std::vector<char> payload;
};

std::string toLowerAscii(std::string value);
std::string trimAscii(const std::string& value);

std::string shortenPlainLabel(const std::string& value, size_t maxLen) {
  if (value.size() <= maxLen) {
    return value;
  }
  if (maxLen <= 3) {
    return value.substr(0, maxLen);
  }
  return value.substr(0, maxLen - 3) + "...";
}

bool isPathSeparator(char c) {
  return c == '/' || c == '\\';
}

std::vector<std::string> splitPathComponents(const std::string& value) {
  std::vector<std::string> components;
  size_t pos = 0;
  while (pos < value.size()) {
    while (pos < value.size() && isPathSeparator(value[pos])) {
      ++pos;
    }
    const size_t start = pos;
    while (pos < value.size() && !isPathSeparator(value[pos])) {
      ++pos;
    }
    if (start < pos) {
      components.emplace_back(value.substr(start, pos - start));
    }
  }
  return components;
}

std::string joinPathComponentsTail(const std::vector<std::string>& components, size_t tailCount) {
  const size_t first = components.size() - tailCount;
  std::string label;
  for (size_t i = first; i < components.size(); ++i) {
    if (!label.empty()) {
      label += '/';
    }
    label += components[i];
  }
  return label;
}

std::string shortenPurePathLabel(const std::string& value, size_t maxLen) {
  constexpr size_t kMaxVisibleParentDirs = 3;
  const auto components = splitPathComponents(value);
  if (components.empty()) {
    return shortenPlainLabel(value, maxLen);
  }

  const size_t maxTailComponents = kMaxVisibleParentDirs + 1;
  size_t tailCount = std::min(maxTailComponents, components.size());
  const bool hasHiddenParents = components.size() > tailCount;
  auto makeLabel = [&](size_t count) {
    std::string tail = joinPathComponentsTail(components, count);
    if (hasHiddenParents || count < components.size()) {
      return std::string(".../") + tail;
    }
    return value;
  };

  std::string label = makeLabel(tailCount);
  while (label.size() > maxLen && tailCount > 1) {
    --tailCount;
    label = makeLabel(tailCount);
  }
  return shortenPlainLabel(label, maxLen);
}

std::string shortenPathLabel(const std::string& value, size_t maxLen) {
  const size_t firstSep = value.find_first_of("/\\");
  if (firstSep == std::string::npos) {
    return shortenPlainLabel(value, maxLen);
  }

  const bool startsWithPath = firstSep == 0 ||
                              (firstSep == 2 && value.size() > 2 && value[1] == ':') ||
                              value.find_first_of(" \t", 0) > firstSep;
  if (startsWithPath) {
    return shortenPurePathLabel(value, maxLen);
  }

  if (firstSep > 0 && std::isspace(static_cast<unsigned char>(value[firstSep - 1]))) {
    const std::string prefix = value.substr(0, firstSep);
    const size_t pathMaxLen = maxLen > prefix.size() ? maxLen - prefix.size() : 0;
    return prefix + shortenPurePathLabel(value.substr(firstSep), pathMaxLen);
  }

  return shortenPlainLabel(value, maxLen);
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

bool parseDoubleStrict(const char* value, double& out) {
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
  out = std::strtod(value, &end);
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

bool parseWindowSize(const std::string& value, int& width, int& height) {
  const auto sep = value.find_first_of("xX,");
  if (sep == std::string::npos) {
    return false;
  }
  long parsedW = 0;
  long parsedH = 0;
  if (!parseLongStrict(value.substr(0, sep).c_str(), 10, parsedW) ||
      !parseLongStrict(value.substr(sep + 1).c_str(), 10, parsedH)) {
    return false;
  }
  if (parsedW < 320 || parsedH < 240 || parsedW > 16384 || parsedH > 16384) {
    return false;
  }
  width = static_cast<int>(parsedW);
  height = static_cast<int>(parsedH);
  return true;
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

bool parseCameraLookAtText(const std::string& value, glm::vec3& pos, glm::vec3& target) {
  float values[6]{};
  if (!parseFloatListStrict(value.c_str(), values, 6)) {
    return false;
  }
  pos = glm::vec3(values[0], values[1], values[2]);
  target = glm::vec3(values[3], values[4], values[5]);
  return glm::length(target - pos) > 1e-4f;
}

void printUsage(const char* argv0) {
  std::cout
    << "Usage: " << (argv0 ? argv0 : "rayrai_tcp_viewer") << " [options]\n\n"
    << "Options:\n"
    << "  --host HOST                 Server host (default: 127.0.0.1)\n"
    << "  --port PORT                 Server port (default: 8080)\n"
    << "  --connect HOST:PORT         Server endpoint shortcut; use [IPv6]:PORT for IPv6\n"
    << "  --auto-connect              Enable automatic reconnect attempts\n"
    << "  --no-auto-connect           Disable automatic reconnect attempts\n"
    << "  --no-pre-warm               Skip shader pre-warm (faster startup, slower first content)\n"
    << "  --warm-at-startup           Also warm renderer content-frame init (~13s) so the first\n"
    << "                              drag-drop is instant. Off by default; empty-viewer startup\n"
    << "                              stays fast unless this flag is passed.\n"
    << "  --resource-dir PATH         Add a mesh/resource search directory; repeatable\n"
    << "  --window-size WxH           Initial window size, e.g. 1600x900\n"
    << "  --fullscreen                Start fullscreen desktop\n"
    << "  --minimize-panels           Start with overlay panels minimized\n"
    << "  --keep-overlay-open         Disable auto-collapse of the left overlay (doc screenshots)\n"
    << "  --auto-frame                Frame the scene after the first valid update\n"
    << "  --camera-lookat p,t         Six floats: px,py,pz,tx,ty,tz\n"
    << "  --camera-offset x,y,z       Follow target from this offset\n"
    << "  --force-camera-lookat       Reapply --camera-lookat every frame\n"
    << "  --screenshot PATH           Save one PNG after the first rendered frame and exit\n"
    << "  --screenshot-dir PATH       Directory used by F12 and PNG sequence recording\n"
    << "  --record-session PATH       Record raw TCP scene updates for offline replay\n"
    << "  --replay-session PATH       Replay a previously recorded TCP session\n"
    << "  --replay-speed SCALE        Replay speed multiplier (default: 1.0)\n"
    << "  --replay-loop               Loop replay sessions\n"
    << "  --export-scene PATH         Export current scene/object diagnostics as JSON\n"
    << "  --trajectory-csv PATH       Log object poses to CSV while updates arrive\n"
    << "  --server-list PATH          Load extra host:port endpoints from a text file\n"
    << "  --wait-for-server SECONDS   Batch wait limit for initial connection\n"
    << "  --exit-after SECONDS        Exit after the given wall-clock duration\n"
    << "  --help                      Show this help\n"
    << "\nDiscovery:\n"
    << "  The connection dropdown lists compatible LAN RaisimServer beacons only.\n";
}

bool parseProgramOptions(int argc, char** argv, ProgramOptions& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i] ? argv[i] : "";
    const auto requireValue = [&](const char* name) -> const char* {
      if (i + 1 >= argc || !argv[i + 1]) {
        std::cerr << "ERROR: " << name << " requires a value\n";
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      options.printHelp = true;
      return true;
    } else if (arg == "--host") {
      const char* value = requireValue("--host");
      if (!value) return false;
      options.host = value;
    } else if (arg == "--port") {
      const char* value = requireValue("--port");
      if (!value) return false;
      if (!parsePortStrict(value, options.port)) {
        std::cerr << "ERROR: invalid --port value: " << value << "\n";
        return false;
      }
    } else if (arg == "--connect") {
      const char* value = requireValue("--connect");
      if (!value) return false;
      ConnectionEntry entry;
      if (!parseConnectionLabel(value, entry)) {
        std::cerr << "ERROR: invalid --connect value: " << value << "\n";
        return false;
      }
      options.host = entry.host;
      options.port = entry.port;
    } else if (arg == "--auto-connect") {
      options.autoConnect = true;
      options.autoConnectSet = true;
    } else if (arg == "--no-auto-connect") {
      options.autoConnect = false;
      options.autoConnectSet = true;
    } else if (arg == "--resource-dir") {
      const char* value = requireValue("--resource-dir");
      if (!value) return false;
      options.resourceDirs.emplace_back(value);
    } else if (arg == "--window-size") {
      const char* value = requireValue("--window-size");
      if (!value) return false;
      if (!parseWindowSize(value, options.windowWidth, options.windowHeight)) {
        std::cerr << "ERROR: invalid --window-size value: " << value << "\n";
        return false;
      }
    } else if (arg == "--fullscreen") {
      options.fullscreen = true;
    } else if (arg == "--minimize-panels") {
      options.minimizePanels = true;
      options.minimizePanelsSet = true;
    } else if (arg == "--keep-overlay-open") {
      options.keepOverlayOpen = true;
    } else if (arg == "--auto-frame") {
      options.autoFrame = true;
      options.autoFrameSet = true;
    } else if (arg == "--camera-lookat") {
      const char* value = requireValue("--camera-lookat");
      if (!value) return false;
      if (!parseCameraLookAtText(value, options.cameraPos, options.cameraTarget)) {
        std::cerr << "ERROR: invalid --camera-lookat value: " << value << "\n";
        return false;
      }
      options.hasCameraLookAt = true;
    } else if (arg == "--camera-offset") {
      const char* value = requireValue("--camera-offset");
      if (!value) return false;
      if (!parseVec3Text(value, options.targetOffset)) {
        std::cerr << "ERROR: invalid --camera-offset value: " << value << "\n";
        return false;
      }
      options.hasTargetOffset = true;
    } else if (arg == "--force-camera-lookat") {
      options.forceCameraLookAt = true;
    } else if (arg == "--screenshot") {
      const char* value = requireValue("--screenshot");
      if (!value) return false;
      options.screenshotPath = value;
      options.exitAfterScreenshot = true;
    } else if (arg == "--screenshot-dir") {
      const char* value = requireValue("--screenshot-dir");
      if (!value) return false;
      options.screenshotDir = value;
    } else if (arg == "--record-session") {
      const char* value = requireValue("--record-session");
      if (!value) return false;
      options.recordSessionPath = value;
    } else if (arg == "--replay-session") {
      const char* value = requireValue("--replay-session");
      if (!value) return false;
      options.replaySessionPath = value;
      options.autoConnect = false;
      options.autoConnectSet = true;
    } else if (arg == "--replay-speed") {
      const char* value = requireValue("--replay-speed");
      if (!value) return false;
      float speed = 0.0f;
      if (!parseFloatStrict(value, speed) || speed <= 0.0f || speed > 100.0f) {
        std::cerr << "ERROR: invalid --replay-speed value: " << value << "\n";
        return false;
      }
      options.replaySpeed = speed;
    } else if (arg == "--replay-loop") {
      options.replayLoop = true;
    } else if (arg == "--export-scene") {
      const char* value = requireValue("--export-scene");
      if (!value) return false;
      options.exportScenePath = value;
    } else if (arg == "--trajectory-csv") {
      const char* value = requireValue("--trajectory-csv");
      if (!value) return false;
      options.trajectoryCsvPath = value;
    } else if (arg == "--server-list") {
      const char* value = requireValue("--server-list");
      if (!value) return false;
      options.endpointListPath = value;
    } else if (arg == "--no-pre-warm") {
      options.preWarmShaders = false;
    } else if (arg == "--warm-at-startup") {
      options.warmAtStartup = true;
    } else if (arg == "--inspect") {
      const char* value = requireValue("--inspect");
      if (!value) return false;
      options.inspectorPath = value;
      options.autoConnect = false;
      options.autoConnectSet = true;
    } else if (arg == "--inspect-after-frames") {
      const char* value = requireValue("--inspect-after-frames");
      if (!value) return false;
      int n = 0;
      try { n = std::stoi(value); }
      catch (...) { std::cerr << "ERROR: invalid --inspect-after-frames\n"; return false; }
      options.inspectAfterFrames = std::max(0, n);
    } else if (arg == "--inspect-close-after-frames") {
      const char* value = requireValue("--inspect-close-after-frames");
      if (!value) return false;
      int n = 0;
      try { n = std::stoi(value); }
      catch (...) { std::cerr << "ERROR: invalid --inspect-close-after-frames\n"; return false; }
      options.inspectCloseAfterFrames = std::max(0, n);
    } else if (arg == "--inspect-reload") {
      const char* value = requireValue("--inspect-reload");
      if (!value) return false;
      options.inspectReloadPath = value;
    } else if (arg == "--wait-for-server") {
      const char* value = requireValue("--wait-for-server");
      if (!value) return false;
      if (!parseDoubleStrict(value, options.waitForServerSeconds) ||
          options.waitForServerSeconds < 0.0) {
        std::cerr << "ERROR: invalid --wait-for-server value: " << value << "\n";
        return false;
      }
    } else if (arg == "--exit-after") {
      const char* value = requireValue("--exit-after");
      if (!value) return false;
      if (!parseDoubleStrict(value, options.exitAfterSeconds) || options.exitAfterSeconds < 0.0) {
        std::cerr << "ERROR: invalid --exit-after value: " << value << "\n";
        return false;
      }
    } else {
      std::cerr << "ERROR: unknown option: " << arg << "\n";
      return false;
    }
  }
  return true;
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

void sortServerEntries(std::vector<ServerEntry>& servers) {
  std::sort(servers.begin(), servers.end(), [](const ServerEntry& lhs, const ServerEntry& rhs) {
    if (lhs.remoteBeacon != rhs.remoteBeacon) {
      return !lhs.remoteBeacon;
    }
    if (lhs.bindHost != rhs.bindHost) {
      return lhs.bindHost < rhs.bindHost;
    }
    if (lhs.endpoint.port != rhs.endpoint.port) {
      return lhs.endpoint.port < rhs.endpoint.port;
    }
    return lhs.protocol < rhs.protocol;
  });
}

bool isCompatibleDiscoveryVersion(int version) {
  return version == raisin::tcp_viewer::kProtocolVersion;
}

bool parseDiscoveryBeacon(const std::string& payload, int& port, int& version,
                          std::string& hostname,
                          std::unordered_map<std::string, std::string>& metadata) {
  std::istringstream input(payload);
  std::string magic;
  std::string portToken;
  if (!(input >> magic >> portToken)) {
    return false;
  }
  int parsedPort = 0;
  if (magic != kDiscoveryMagic || !parsePortStrict(portToken, parsedPort)) {
    return false;
  }

  metadata.clear();
  std::vector<std::string> legacyHostTokens;
  std::vector<std::string> tokens;
  std::string token;
  while (input >> token) {
    tokens.push_back(std::move(token));
  }

  version = 0;
  size_t firstMetadataToken = 0;
  if (!tokens.empty()) {
    long parsedVersion = 0;
    if (parseLongStrict(tokens.front().c_str(), 10, parsedVersion)) {
      if (parsedVersion < 0 || parsedVersion > std::numeric_limits<int>::max()) {
        return false;
      }
      version = static_cast<int>(parsedVersion);
      firstMetadataToken = 1;
    }
  }

  for (size_t i = firstMetadataToken; i < tokens.size(); ++i) {
    const std::string& token = tokens[i];
    const auto eq = token.find('=');
    if (eq == std::string::npos || eq == 0) {
      legacyHostTokens.push_back(token);
      continue;
    }
    std::string key = token.substr(0, eq);
    std::string value = token.substr(eq + 1);
    metadata[key] = value;
  }

  const auto hostIt = metadata.find("hostname");
  if (hostIt != metadata.end() && !hostIt->second.empty()) {
    hostname = hostIt->second;
  } else if (!legacyHostTokens.empty()) {
    hostname.clear();
    for (const auto& part : legacyHostTokens) {
      if (!hostname.empty()) hostname += " ";
      hostname += part;
    }
  } else {
    hostname.clear();
  }

  port = parsedPort;
  return true;
}

class DiscoveryBeaconReceiver {
 public:
  ~DiscoveryBeaconReceiver() { closeSocket(); }

  bool start(std::string& status) {
#if defined(__linux__) || defined(__APPLE__)
    if (socketFd_ >= 0) {
      status = "LAN beacon listener active";
      return true;
    }

    socketFd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFd_ < 0) {
      status = std::string("LAN beacon listener unavailable: ") + std::strerror(errno);
      return false;
    }

    int opt = 1;
    setsockopt(socketFd_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char*>(&opt), sizeof(opt));
#ifdef SO_REUSEPORT
    setsockopt(socketFd_, SOL_SOCKET, SO_REUSEPORT, reinterpret_cast<char*>(&opt), sizeof(opt));
#endif

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(kDiscoveryPort);
    if (bind(socketFd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
      status = std::string("LAN beacon listener bind failed: ") + std::strerror(errno);
      closeSocket();
      return false;
    }

    const int flags = fcntl(socketFd_, F_GETFL, 0);
    if (flags >= 0) {
      fcntl(socketFd_, F_SETFL, flags | O_NONBLOCK);
    }

    status = "LAN beacon listener active";
    return true;
#else
    status = "LAN beacon listener unavailable on this platform";
    return false;
#endif
  }

  bool poll() {
#if defined(__linux__) || defined(__APPLE__)
    if (socketFd_ < 0) {
      return false;
    }

    bool changed = false;
    const auto now = std::chrono::steady_clock::now();
    while (true) {
      char buffer[1024];
      sockaddr_in from{};
      socklen_t fromLen = sizeof(from);
      const ssize_t count = recvfrom(socketFd_, buffer, sizeof(buffer) - 1, 0,
                                     reinterpret_cast<sockaddr*>(&from), &fromLen);
      if (count < 0) {
        if (errno == EINTR) {
          continue;
        }
        break;
      }
      if (count == 0) {
        break;
      }
      buffer[count] = '\0';

      char sourceHost[INET_ADDRSTRLEN]{};
      if (!inet_ntop(AF_INET, &from.sin_addr, sourceHost, sizeof(sourceHost))) {
        continue;
      }

      int port = 0;
      int version = 0;
      std::string hostname;
      std::unordered_map<std::string, std::string> metadata;
      if (!parseDiscoveryBeacon(buffer, port, version, hostname, metadata) ||
          !isCompatibleDiscoveryVersion(version)) {
        continue;
      }

      const std::string source = sourceHost;
      const std::string key = source + ":" + std::to_string(port);
      auto& record = beacons_[key];
      const bool isNew = record.server.endpoint.host.empty();
      ServerEntry updated;
      updated.endpoint.host = source;
      updated.endpoint.port = port;
      updated.bindHost = source;
      updated.protocol = "raisim beacon";
      updated.metadata = metadata;
      updated.remoteBeacon = true;
      updated.lastSeen = now;
      const auto exeIt = metadata.find("exe");
      const std::string serverName =
        exeIt != metadata.end() && !exeIt->second.empty() ? exeIt->second : "RaisimServer";
      updated.process = hostname.empty() ? serverName : serverName + " on " + hostname;
      if (version > 0) {
        updated.process += " (protocol " + std::to_string(version) + ")";
      }
      changed = changed || isNew ||
                record.server.endpoint.host != updated.endpoint.host ||
                record.server.endpoint.port != updated.endpoint.port ||
                record.server.process != updated.process ||
                record.server.metadata != updated.metadata;
      record.server = updated;
      record.lastSeen = now;
    }

    for (auto it = beacons_.begin(); it != beacons_.end();) {
      if (now - it->second.lastSeen > kDiscoveryBeaconTimeout) {
        it = beacons_.erase(it);
        changed = true;
      } else {
        ++it;
      }
    }
    return changed;
#else
    return false;
#endif
  }

  std::vector<ServerEntry> servers() const {
    std::vector<ServerEntry> entries;
    entries.reserve(beacons_.size());
    for (const auto& item : beacons_) {
      entries.push_back(item.second.server);
    }
    sortServerEntries(entries);
    return entries;
  }

 private:
  struct BeaconRecord {
    ServerEntry server;
    std::chrono::steady_clock::time_point lastSeen;
  };

  void closeSocket() {
#if defined(__linux__) || defined(__APPLE__)
    if (socketFd_ >= 0) {
      close(socketFd_);
      socketFd_ = -1;
    }
#endif
  }

  int socketFd_ = -1;
  std::unordered_map<std::string, BeaconRecord> beacons_;
};

std::string formatServerLabel(const ServerEntry& server) {
  std::string label = formatConnectionLabel(ConnectionEntry{server.bindHost, server.endpoint.port});
  if (server.bindHost != server.endpoint.host) {
    label += " (connect " + formatConnectionLabel(server.endpoint) + ")";
  }
  if (!server.protocol.empty()) {
    label += " [" + server.protocol + "]";
  }
  if (!server.process.empty()) {
    label += " - " + shortenPathLabel(server.process, 72);
  }
  return label;
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
  float parsed = fallback;
  return parseFloatStrict(rawValue.c_str(), parsed) ? parsed : fallback;
}

float readEnvFloatClamped(const char* name, float defaultValue, float minValue, float maxValue) {
  const char* rawValue = std::getenv(name);
  if (!rawValue || !*rawValue) {
    return defaultValue;
  }
  float parsed = defaultValue;
  if (!parseFloatStrict(rawValue, parsed)) {
    return defaultValue;
  }
  return std::clamp(parsed, minValue, maxValue);
}

int parseIntValue(const std::string& rawValue, int fallback) {
  long parsed = 0;
  if (!parseLongStrict(rawValue.c_str(), 10, parsed) ||
      parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max()) {
    return fallback;
  }
  return static_cast<int>(parsed);
}

glm::vec3 parseVec3Value(const std::string& rawValue, const glm::vec3& fallback) {
  glm::vec3 parsed = fallback;
  return parseVec3Text(rawValue, parsed) ? parsed : fallback;
}

glm::vec4 parseVec4Value(const std::string& rawValue, const glm::vec4& fallback) {
  glm::vec4 parsed = fallback;
  return parseVec4Text(rawValue, parsed) ? parsed : fallback;
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

struct GpuQualityInfo {
  std::string vendor;
  std::string renderer;
  std::string version;
  int maxTextureSize = 0;
  int maxSamples = 0;
  int maxTextureImageUnits = 0;
};

struct GpuQualityRecommendation {
  int quality = 1;
  GpuQualityInfo gpu;
};

bool containsToken(const std::string& haystack, const std::string& token) {
  return haystack.find(token) != std::string::npos;
}

bool containsAnyToken(const std::string& haystack, std::initializer_list<const char*> tokens) {
  for (const char* token : tokens) {
    if (token != nullptr && containsToken(haystack, token)) {
      return true;
    }
  }
  return false;
}

int automaticRenderQualityFromGpuInfo(const GpuQualityInfo& info) {
  const std::string gpuText = toLowerAscii(info.vendor + " " + info.renderer + " " + info.version);
  if (containsAnyToken(gpuText, {
        "llvmpipe", "softpipe", "swiftshader", "software rasterizer",
        "microsoft basic render", "mesa offscreen", "swrast"})) {
    return 0;
  }

  int score = 0;
  const bool highEndDiscrete = containsAnyToken(gpuText, {
    "rtx 4090", "rtx 4080", "rtx 4070", "rtx 3090", "rtx 3080", "rtx 3070",
    "rx 7900", "rx 7800", "rx 7700", "rx 6950", "rx 6900", "rx 6800",
    "radeon pro w7", "radeon pro w6"});
  const bool lowEndDiscrete = containsAnyToken(gpuText, {
    "gtx 1050", "gtx 1060", "gtx 1650", "gtx 1660", "mx150", "mx250", "mx350",
    "rx 550", "rx 560", "rx 570", "rx 580"});
  const bool nvidia = containsAnyToken(gpuText, {"nvidia", "geforce", "quadro", "rtx", "gtx"});
  const bool amd = containsAnyToken(gpuText, {"amd", "radeon", "advanced micro devices"});
  const bool intel = containsAnyToken(gpuText, {"intel", "uhd graphics", "iris", "arc"});
  const bool apple = containsAnyToken(gpuText, {"apple", "metal"});

  if (highEndDiscrete) {
    score += 6;
  } else if (lowEndDiscrete) {
    score += 2;
  } else if (nvidia || amd || apple) {
    score += 4;
  } else if (intel) {
    score += containsToken(gpuText, "arc") ? 3 : 2;
  } else {
    score += 1;
  }

  if (info.maxTextureSize >= 32768) {
    score += 2;
  } else if (info.maxTextureSize >= 16384) {
    score += 1;
  } else if (info.maxTextureSize > 0 && info.maxTextureSize < 8192) {
    score -= 1;
  }

  if (info.maxSamples >= 8) {
    score += 1;
  } else if (info.maxSamples > 0 && info.maxSamples < 4) {
    score -= 1;
  }

  if (info.maxTextureImageUnits >= 32) {
    score += 1;
  } else if (info.maxTextureImageUnits > 0 && info.maxTextureImageUnits < 16) {
    score -= 1;
  }

  if (score <= 1) {
    return 0;
  }
  if (score <= 4) {
    return 1;
  }
  if (score <= 8) {
    return 2;
  }
  return 3;
}

std::string glStringValue(gl::GLenum name) {
  const auto* raw = reinterpret_cast<const char*>(gl::glGetString(name));
  return raw != nullptr ? std::string(raw) : std::string();
}

GpuQualityRecommendation recommendRenderQualityForCurrentGpu() {
  GpuQualityRecommendation recommendation;
  recommendation.gpu.vendor = glStringValue(gl::GL_VENDOR);
  recommendation.gpu.renderer = glStringValue(gl::GL_RENDERER);
  recommendation.gpu.version = glStringValue(gl::GL_VERSION);

  gl::GLint value = 0;
  gl::glGetIntegerv(gl::GL_MAX_TEXTURE_SIZE, &value);
  recommendation.gpu.maxTextureSize = static_cast<int>(value);
  value = 0;
  gl::glGetIntegerv(gl::GL_MAX_SAMPLES, &value);
  recommendation.gpu.maxSamples = static_cast<int>(value);
  value = 0;
  gl::glGetIntegerv(gl::GL_MAX_TEXTURE_IMAGE_UNITS, &value);
  recommendation.gpu.maxTextureImageUnits = static_cast<int>(value);

  recommendation.quality = automaticRenderQualityFromGpuInfo(recommendation.gpu);
  return recommendation;
}

int colorModeIndexFromName(const std::string& rawValue, int fallback) {
  const std::string value = toLowerAscii(trimAscii(rawValue));
  if (value == "fast_linear" || value == "fast linear" || value == "linear" || value == "0") return 0;
  if (value == "aces_approx" || value == "aces approx" || value == "aces" || value == "1") return 1;
  if (value == "unreal_preview" || value == "unreal preview" || value == "unreal" || value == "2") return 2;
  return fallback;
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

void recordConnection(
  std::vector<ConnectionEntry>& connections, const std::string& host, int port);
void recordResourceDir(std::vector<std::string>& dirs, const std::string& path);

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

raisin::RayraiWindow::WeatherPreset weatherPresetFromIndex(int preset) {
  switch (std::clamp(preset, 0, 10)) {
    case 0: return raisin::RayraiWindow::WeatherPreset::Clear;
    case 1: return raisin::RayraiWindow::WeatherPreset::Hazy;
    case 2: return raisin::RayraiWindow::WeatherPreset::Overcast;
    case 3: return raisin::RayraiWindow::WeatherPreset::Fog;
    case 4: return raisin::RayraiWindow::WeatherPreset::Rain;
    case 5: return raisin::RayraiWindow::WeatherPreset::HeavyRain;
    case 6: return raisin::RayraiWindow::WeatherPreset::Snow;
    case 7: return raisin::RayraiWindow::WeatherPreset::Storm;
    case 8: return raisin::RayraiWindow::WeatherPreset::NightClear;
    case 9: return raisin::RayraiWindow::WeatherPreset::NightRain;
    case 10:
    default: return raisin::RayraiWindow::WeatherPreset::Custom;
  }
}

raisin::RayraiWindow::WeatherQuality weatherQualityFromIndex(int quality) {
  switch (std::clamp(quality, 0, 3)) {
    case 0: return raisin::RayraiWindow::WeatherQuality::Low;
    case 1: return raisin::RayraiWindow::WeatherQuality::Medium;
    case 3: return raisin::RayraiWindow::WeatherQuality::Ultra;
    case 2:
    default: return raisin::RayraiWindow::WeatherQuality::High;
  }
}

float normalizedDegrees(float degrees) {
  if (!std::isfinite(degrees)) {
    return 0.0f;
  }
  float wrapped = std::fmod(degrees, 360.0f);
  if (wrapped < 0.0f) {
    wrapped += 360.0f;
  }
  return wrapped;
}

glm::vec3 windDirectionFromDegrees(float degrees) {
  const float radians = normalizedDegrees(degrees) * kDegToRad;
  return glm::normalize(glm::vec3(std::cos(radians), std::sin(radians), 0.0f));
}

float windDirectionDegreesFromVector(const glm::vec3& direction) {
  if (glm::dot(direction, direction) <= 1.0e-8f) {
    return 0.0f;
  }
  return normalizedDegrees(std::atan2(direction.y, direction.x) * kRadToDeg);
}

bool weatherDefaultEnabledForQuality(int quality) {
  const int clampedQuality = std::clamp(quality, 0, 4);
  return clampedQuality >= 2;
}

bool highFidelityPbrAllowedForQuality(int quality) {
  const int clampedQuality = std::clamp(quality, 0, 4);
  return clampedQuality >= 2;
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
  settings.skyWindDirectionDeg = normalizedDegrees(settings.skyWindDirectionDeg);
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
  settings.skySunAzimuthDeg = normalizedDegrees(settings.skySunAzimuthDeg);
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
}

std::filesystem::path settingsFilePath() {
  const char* home = std::getenv("HOME");
  if (home && *home) {
    return std::filesystem::path(home) / ".rayrai" / "settings.yaml";
  }
  return std::filesystem::current_path() / ".rayrai" / "settings.yaml";
}

// Forward decl so the parser below can propagate preset cloud values when
// sky_weather_preset is set via TCP / settings file.
void copyWeatherPresetToSettings(ViewerSettings& settings, int preset);

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
  for (const auto& entry : settings.recentConnections) {
    output << "recent_connection: " << formatConnectionLabel(entry) << "\n";
  }
  for (const auto& dir : settings.resourceDirs) {
    output << "resource_dir: " << dir << "\n";
  }
}

std::string findRobotoFontPath(const std::filesystem::path& binaryDir) {
  if (const char* envPath = std::getenv("RAYRAI_TCP_VIEWER_FONT")) {
    if (*envPath && std::filesystem::exists(envPath)) {
      return envPath;
    }
  }

  std::vector<std::filesystem::path> candidates;
  candidates.push_back(binaryDir / kRobotoFontRelativePath);
  candidates.push_back(binaryDir / ".." / kRobotoFontRelativePath);
  candidates.push_back(binaryDir / ".." / ".." / kRobotoFontRelativePath);
  candidates.push_back(binaryDir / ".." / ".." / ".." / kRobotoFontRelativePath);
  candidates.push_back(std::filesystem::current_path() / kRobotoFontRelativePath);
  candidates.push_back(std::filesystem::current_path() / ".." / kRobotoFontRelativePath);

  const std::filesystem::path sourceDir = std::filesystem::path(__FILE__).parent_path();
  if (!sourceDir.empty()) {
    candidates.push_back(sourceDir / ".." / ".." / ".." / kRobotoFontRelativePath);
    candidates.push_back(sourceDir / ".." / ".." / ".." / ".." / kRobotoFontRelativePath);
    candidates.push_back(sourceDir / ".." / ".." / ".." / ".." / "raisim2Lib" / kRobotoFontRelativePath);
  }

  std::error_code ec;
  for (const auto& path : candidates) {
    const std::filesystem::path absolute = std::filesystem::absolute(path, ec).lexically_normal();
    if (!ec && std::filesystem::exists(absolute, ec) && !ec) {
      return absolute.string();
    }
    ec.clear();
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
  float values[3]{};
  if (!parseFloatListStrict(value, values, 3)) {
    return false;
  }
  vec = glm::vec3(values[0], values[1], values[2]);
  return glm::length(vec) > 1e-4f;
}

bool parseCameraLookAtEnv(const char* value, glm::vec3& pos, glm::vec3& target) {
  float values[6]{};
  if (!parseFloatListStrict(value, values, 6)) {
    return false;
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

void frameBounds(raisin::RayraiWindow& viewer, const glm::vec3& minBound, const glm::vec3& maxBound) {
  const glm::vec3 center = 0.5f * (minBound + maxBound);
  const glm::vec3 extent = glm::max(maxBound - minBound, glm::vec3(0.25f));
  const float radius = std::max(0.25f, 0.5f * glm::length(extent));
  const float fovy = glm::radians(std::max(20.0f, viewer.getCamera().zoom));
  const float distance = std::max(1.0f, radius / std::tan(0.5f * fovy) * 1.55f);
  const glm::vec3 viewDir = glm::normalize(glm::vec3(1.55f, -1.85f, 1.05f));
  applyCameraLookAt(viewer.getCamera(), center + viewDir * distance, center);
}

bool frameScene(raisin::RayraiWindow& viewer, const RemoteScene& scene) {
  glm::vec3 minBound, maxBound;
  if (!scene.computeSceneBounds(minBound, maxBound)) {
    return false;
  }
  frameBounds(viewer, minBound, maxBound);
  return true;
}

bool frameSelected(raisin::RayraiWindow& viewer, const VisualEntry* entry) {
  if (!entry) {
    return false;
  }
  const glm::vec3 halfExtent = glm::max(glm::vec3(entry->lastSize) * 0.5f, glm::vec3(0.25f));
  frameBounds(viewer, entry->lastPos - halfExtent, entry->lastPos + halfExtent);
  return true;
}

glm::vec3 normalizedOr(const glm::vec3& value, const glm::vec3& fallback) {
  const float len = glm::length(value);
  if (std::isfinite(len) && len > 1.0e-6f) {
    return value / len;
  }
  const float fallbackLen = glm::length(fallback);
  if (std::isfinite(fallbackLen) && fallbackLen > 1.0e-6f) {
    return fallback / fallbackLen;
  }
  return glm::vec3(1.0f, 0.0f, 0.0f);
}

glm::vec3 mouseForceFromDragPixels(
  const glm::vec3& cameraRight, const glm::vec3& cameraUp, const ImVec2& dragPixels,
  float newtonsPerPixel) {
  const float scale = std::isfinite(newtonsPerPixel) ? std::max(0.0f, newtonsPerPixel) : 0.0f;
  const glm::vec3 right = normalizedOr(cameraRight, glm::vec3(1.0f, 0.0f, 0.0f));
  const glm::vec3 up = normalizedOr(cameraUp, glm::vec3(0.0f, 0.0f, 1.0f));
  return (right * dragPixels.x - up * dragPixels.y) * scale;
}

glm::vec3 mouseForceFromDragPixels(
  const raisin::Camera& camera, const ImVec2& dragPixels, float newtonsPerPixel) {
  return mouseForceFromDragPixels(camera.right, camera.up, dragPixels, newtonsPerPixel);
}

bool projectWorldToViewport(
  const raisin::Camera& camera, const ViewerViewportState& viewport, const glm::vec3& world,
  ImVec2& screen) {
  if (viewport.size.x <= 1.0f || viewport.size.y <= 1.0f) {
    return false;
  }
  const glm::vec4 clip = camera.getProjectionMatrix() * camera.getViewMatrix() * glm::vec4(world, 1.0f);
  if (!std::isfinite(clip.x) || !std::isfinite(clip.y) || !std::isfinite(clip.z) ||
      !std::isfinite(clip.w) || clip.w <= 1.0e-6f) {
    return false;
  }
  const glm::vec3 ndc = glm::vec3(clip) / clip.w;
  if (!std::isfinite(ndc.x) || !std::isfinite(ndc.y) || !std::isfinite(ndc.z)) {
    return false;
  }
  screen.x = viewport.origin.x + (ndc.x * 0.5f + 0.5f) * viewport.size.x;
  screen.y = viewport.origin.y + (0.5f - ndc.y * 0.5f) * viewport.size.y;
  return std::isfinite(screen.x) && std::isfinite(screen.y);
}

float screenDistancePixels(const ImVec2& a, const ImVec2& b) {
  const float dx = a.x - b.x;
  const float dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

std::string formatRulerDistance(float meters) {
  if (!std::isfinite(meters) || meters < 0.0f) {
    return "--";
  }
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.3f m", meters);
  return buffer;
}

std::string formatRulerPoint(const glm::vec3& point) {
  char buffer[128];
  std::snprintf(buffer, sizeof(buffer), "(%.3f, %.3f, %.3f)",
    point.x, point.y, point.z);
  return buffer;
}

bool readRulerWorldPointAtCursor(
  const raisin::Camera& camera, const ViewerViewportState& viewport, glm::vec3& world) {
  const int width = camera.rtWidth();
  const int height = camera.rtHeight();
  const int x = viewport.cursorX;
  const int y = viewport.cursorY;
  if (!viewport.hovered || width <= 0 || height <= 0 || x < 0 || y < 0 || x >= width || y >= height ||
      camera.getSceneFbo() == 0) {
    return false;
  }

  gl::GLint previousReadFramebuffer = 0;
  gl::glGetIntegerv(gl::GL_READ_FRAMEBUFFER_BINDING, &previousReadFramebuffer);
  gl::glBindFramebuffer(gl::GL_READ_FRAMEBUFFER, camera.getSceneFbo());
  float depth = 1.0f;
  const int readY = (height - 1) - y;
  gl::glReadPixels(x, readY, 1, 1, gl::GL_DEPTH_COMPONENT, gl::GL_FLOAT, &depth);
  gl::glBindFramebuffer(gl::GL_READ_FRAMEBUFFER,
    static_cast<gl::GLuint>(previousReadFramebuffer));

  if (!std::isfinite(depth) || depth < 0.0f || depth >= 1.0f) {
    return false;
  }

  const float ndcX = ((static_cast<float>(x) + 0.5f) / static_cast<float>(width)) * 2.0f - 1.0f;
  const float ndcY = 1.0f - ((static_cast<float>(y) + 0.5f) / static_cast<float>(height)) * 2.0f;
  const float ndcZ = depth * 2.0f - 1.0f;
  const glm::vec4 clip(ndcX, ndcY, ndcZ, 1.0f);
  const glm::mat4 invViewProjection = glm::inverse(camera.getProjectionMatrix() * camera.getViewMatrix());
  const glm::vec4 worldH = invViewProjection * clip;
  if (!std::isfinite(worldH.x) || !std::isfinite(worldH.y) || !std::isfinite(worldH.z) ||
      !std::isfinite(worldH.w) || std::abs(worldH.w) <= 1.0e-6f) {
    return false;
  }

  world = glm::vec3(worldH) / worldH.w;
  return std::isfinite(world.x) && std::isfinite(world.y) && std::isfinite(world.z);
}

float mouseForceStartRadiusPixels(
  const raisin::Camera& camera, const ViewerViewportState& viewport,
  const VisualEntry& entry, const glm::vec3& applicationPoint) {
  constexpr float kMinRadius = 24.0f;
  constexpr float kMaxRadius = 96.0f;
  ImVec2 center;
  if (!projectWorldToViewport(camera, viewport, applicationPoint, center)) {
    return kMinRadius;
  }

  const glm::vec3 halfSize = glm::max(glm::vec3(entry.lastSize) * 0.5f, glm::vec3(0.05f));
  const float worldRadius = std::max(0.05f, glm::length(halfSize));
  const glm::vec3 right = normalizedOr(camera.right, glm::vec3(1.0f, 0.0f, 0.0f));
  const glm::vec3 up = normalizedOr(camera.up, glm::vec3(0.0f, 0.0f, 1.0f));

  float radius = kMinRadius;
  ImVec2 edge;
  if (projectWorldToViewport(camera, viewport, applicationPoint + right * worldRadius, edge)) {
    radius = std::max(radius, screenDistancePixels(center, edge));
  }
  if (projectWorldToViewport(camera, viewport, applicationPoint + up * worldRadius, edge)) {
    radius = std::max(radius, screenDistancePixels(center, edge));
  }
  return std::clamp(radius, kMinRadius, kMaxRadius);
}

bool isMouseWithinForceStartRadius(
  const raisin::Camera& camera, const ViewerViewportState& viewport, const VisualEntry& entry,
  const glm::vec3& applicationPoint, const ImVec2& mousePos, ImVec2& anchorScreen) {
  if (!projectWorldToViewport(camera, viewport, applicationPoint, anchorScreen)) {
    return false;
  }
  const float radius = mouseForceStartRadiusPixels(camera, viewport, entry, applicationPoint);
  return screenDistancePixels(mousePos, anchorScreen) <= radius;
}

void drawMouseForcePreview(
  const MouseForceGesture& gesture, const ViewerViewportState& viewport, const raisin::Camera& camera) {
  if (!gesture.active) {
    return;
  }
  ImVec2 origin = gesture.pressMouse;
  projectWorldToViewport(camera, viewport, gesture.applicationPoint, origin);
  const ImVec2 drag(gesture.currentMouse.x - gesture.pressMouse.x,
                    gesture.currentMouse.y - gesture.pressMouse.y);
  const ImVec2 tip(origin.x + drag.x, origin.y + drag.y);
  const float len = std::sqrt(drag.x * drag.x + drag.y * drag.y);

  ImDrawList* drawList = ImGui::GetForegroundDrawList();
  const ImU32 shadow = IM_COL32(5, 8, 12, 190);
  const ImU32 lineColor = IM_COL32(255, 210, 74, 255);
  const ImU32 fillColor = IM_COL32(255, 168, 48, 245);
  drawList->AddCircleFilled(origin, 5.0f, shadow, 24);
  drawList->AddCircleFilled(origin, 3.5f, lineColor, 24);
  if (len >= 2.0f) {
    drawList->AddLine(ImVec2(origin.x + 1.0f, origin.y + 1.0f), ImVec2(tip.x + 1.0f, tip.y + 1.0f),
      shadow, 5.0f);
    drawList->AddLine(origin, tip, lineColor, 3.0f);
    const ImVec2 dir(drag.x / len, drag.y / len);
    const ImVec2 normal(-dir.y, dir.x);
    const float headLen = std::min(22.0f, std::max(10.0f, len * 0.24f));
    const float headHalfWidth = headLen * 0.42f;
    const ImVec2 base(tip.x - dir.x * headLen, tip.y - dir.y * headLen);
    const ImVec2 p1(tip.x, tip.y);
    const ImVec2 p2(base.x + normal.x * headHalfWidth, base.y + normal.y * headHalfWidth);
    const ImVec2 p3(base.x - normal.x * headHalfWidth, base.y - normal.y * headHalfWidth);
    drawList->AddTriangleFilled(ImVec2(p1.x + 1.0f, p1.y + 1.0f), ImVec2(p2.x + 1.0f, p2.y + 1.0f),
      ImVec2(p3.x + 1.0f, p3.y + 1.0f), shadow);
    drawList->AddTriangleFilled(p1, p2, p3, fillColor);
  }

  char label[96];
  std::snprintf(label, sizeof(label), "%.1f N", glm::length(gesture.force));
  const ImVec2 labelPos(tip.x + 10.0f, tip.y - ImGui::GetFontSize() * 0.5f);
  drawList->AddText(ImVec2(labelPos.x + 1.0f, labelPos.y + 1.0f), shadow, label);
  drawList->AddText(labelPos, lineColor, label);
}

void drawRulerOverlay(
  const RulerToolState& ruler, const ViewerViewportState& viewport, const raisin::Camera& camera) {
  if (!ruler.hasA && !ruler.hasB) {
    return;
  }

  ImVec2 screenA;
  ImVec2 screenB;
  const bool visibleA = ruler.hasA && projectWorldToViewport(camera, viewport, ruler.a, screenA);
  const bool visibleB = ruler.hasB && projectWorldToViewport(camera, viewport, ruler.b, screenB);
  if (!visibleA && !visibleB) {
    return;
  }

  ImDrawList* drawList = ImGui::GetForegroundDrawList();
  const ImU32 shadow = IM_COL32(4, 7, 11, 205);
  const ImU32 lineColor = IM_COL32(74, 214, 255, 255);
  const ImU32 pointAColor = IM_COL32(255, 214, 84, 255);
  const ImU32 pointBColor = IM_COL32(120, 255, 166, 255);
  const auto drawEndpoint = [&](const ImVec2& pos, const char* label, ImU32 color) {
    drawList->AddCircleFilled(ImVec2(pos.x + 1.0f, pos.y + 1.0f), 6.0f, shadow, 28);
    drawList->AddCircleFilled(pos, 4.0f, color, 28);
    const ImVec2 textPos(pos.x + 8.0f, pos.y - ImGui::GetFontSize() * 0.5f);
    drawList->AddText(ImVec2(textPos.x + 1.0f, textPos.y + 1.0f), shadow, label);
    drawList->AddText(textPos, color, label);
  };

  if (visibleA && visibleB) {
    const ImVec2 delta(screenB.x - screenA.x, screenB.y - screenA.y);
    const float len = std::max(1.0f, std::sqrt(delta.x * delta.x + delta.y * delta.y));
    const ImVec2 dir(delta.x / len, delta.y / len);
    const ImVec2 normal(-dir.y, dir.x);
    const float tickHalf = 8.0f;
    drawList->AddLine(ImVec2(screenA.x + 1.0f, screenA.y + 1.0f),
      ImVec2(screenB.x + 1.0f, screenB.y + 1.0f), shadow, 5.0f);
    drawList->AddLine(screenA, screenB, lineColor, 2.5f);
    drawList->AddLine(ImVec2(screenA.x - normal.x * tickHalf, screenA.y - normal.y * tickHalf),
      ImVec2(screenA.x + normal.x * tickHalf, screenA.y + normal.y * tickHalf), lineColor, 2.5f);
    drawList->AddLine(ImVec2(screenB.x - normal.x * tickHalf, screenB.y - normal.y * tickHalf),
      ImVec2(screenB.x + normal.x * tickHalf, screenB.y + normal.y * tickHalf), lineColor, 2.5f);

    const std::string label = formatRulerDistance(glm::distance(ruler.a, ruler.b));
    const ImVec2 textSize = ImGui::CalcTextSize(label.c_str());
    const ImVec2 mid((screenA.x + screenB.x) * 0.5f, (screenA.y + screenB.y) * 0.5f);
    const ImVec2 labelPos(mid.x - textSize.x * 0.5f, mid.y - textSize.y - 12.0f);
    const ImVec2 pad(7.0f, 4.0f);
    drawList->AddRectFilled(ImVec2(labelPos.x - pad.x, labelPos.y - pad.y),
      ImVec2(labelPos.x + textSize.x + pad.x, labelPos.y + textSize.y + pad.y),
      IM_COL32(8, 13, 20, 220), 5.0f);
    drawList->AddRect(ImVec2(labelPos.x - pad.x, labelPos.y - pad.y),
      ImVec2(labelPos.x + textSize.x + pad.x, labelPos.y + textSize.y + pad.y),
      IM_COL32(74, 214, 255, 180), 5.0f);
    drawList->AddText(labelPos, lineColor, label.c_str());
  }

  if (visibleA) drawEndpoint(screenA, "A", pointAColor);
  if (visibleB) drawEndpoint(screenB, "B", pointBColor);
}

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

std::filesystem::path timestampedCapturePath(const std::filesystem::path& dir, const char* prefix) {
  const auto now = std::chrono::system_clock::now();
  const std::time_t raw = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &raw);
#else
  localtime_r(&raw, &tm);
#endif
  std::ostringstream name;
  name << (prefix ? prefix : "rayrai") << "_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".png";
  return dir / name.str();
}

bool saveViewerTexturePng(raisin::RayraiWindow& viewer, const std::filesystem::path& path, std::string& status) {
  auto& camera = viewer.getCamera();
  const int width = camera.rtWidth();
  const int height = camera.rtHeight();
  if (width <= 0 || height <= 0) {
    status = "capture failed: invalid render target";
    return false;
  }

  std::error_code ec;
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path(), ec);
    if (ec) {
      status = "capture failed: cannot create directory";
      return false;
    }
  }

  std::vector<unsigned char> rgba(static_cast<size_t>(width) * static_cast<size_t>(height) * 4u);
  gl::GLint previousPackAlignment = 4;
  gl::glGetIntegerv(gl::GL_PACK_ALIGNMENT, &previousPackAlignment);
  gl::glBindTexture(gl::GL_TEXTURE_2D, camera.getFinalTexture());
  gl::glPixelStorei(gl::GL_PACK_ALIGNMENT, 1);
  gl::glGetTexImage(gl::GL_TEXTURE_2D, 0, gl::GL_RGBA, gl::GL_UNSIGNED_BYTE, rgba.data());
  gl::glPixelStorei(gl::GL_PACK_ALIGNMENT, previousPackAlignment);
  gl::glBindTexture(gl::GL_TEXTURE_2D, 0);
  flipRows(rgba, width, height);

  if (!stbi_write_png(path.string().c_str(), width, height, 4, rgba.data(), width * 4)) {
    status = "capture failed: PNG write failed";
    return false;
  }
  status = "saved " + path.string();
  return true;
}

float quaternionAngularSpeed(const glm::vec4& previous, const glm::vec4& current, double dt) {
  if (dt <= 1e-9) {
    return 0.0f;
  }
  const glm::quat q0(previous.w, previous.x, previous.y, previous.z);
  const glm::quat q1(current.w, current.x, current.y, current.z);
  const float dotValue = std::abs(glm::dot(glm::normalize(q0), glm::normalize(q1)));
  const float angle = 2.0f * std::acos(std::clamp(dotValue, 0.0f, 1.0f));
  return angle / static_cast<float>(dt);
}

uint64_t visualMotionKey(uint32_t tag, int index) {
  return (static_cast<uint64_t>(tag) << 32u) | static_cast<uint32_t>(index);
}

void updateMotionEstimate(MotionEstimate& estimate, const VisualEntry& entry, double timeSeconds) {
  if (estimate.hasPrevious) {
    const double dt = timeSeconds - estimate.previousTime;
    if (dt > 1e-9 && dt < 10.0) {
      estimate.linearVelocity = (entry.lastPos - estimate.previousPosition) / static_cast<float>(dt);
      estimate.angularSpeed = quaternionAngularSpeed(estimate.previousQuat, entry.lastQuat, dt);
      estimate.valid = true;
    }
  }
  estimate.previousTime = timeSeconds;
  estimate.previousPosition = entry.lastPos;
  estimate.previousQuat = entry.lastQuat;
  estimate.hasPrevious = true;
}

void updateStatsWindow(ViewerStats& stats, std::chrono::steady_clock::time_point now) {
  const double elapsed = std::chrono::duration<double>(now - stats.windowStart).count();
  if (elapsed < 1.0) {
    return;
  }
  stats.fps = static_cast<double>(stats.frames) / elapsed;
  stats.updateHz = static_cast<double>(stats.updates) / elapsed;
  stats.rxKbps = static_cast<double>(stats.bytes) / 1024.0 / elapsed;
  stats.frames = 0;
  stats.updates = 0;
  stats.bytes = 0;
  stats.windowStart = now;
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

void copyWeatherSettingsToViewerSettings(
  ViewerSettings& settings, const raisin::RayraiWindow::WeatherSettings& weather) {
  settings.skyWeatherPreset = static_cast<int>(weather.preset);
  settings.skyWeatherQuality = static_cast<int>(weather.quality);
  settings.skyWeatherSeed = static_cast<int>(std::max<uint32_t>(weather.seed, 1u));
  settings.skyTimeOfDayHours = weather.timeOfDayHours;
  settings.skyLatitude = weather.latitude;
  settings.skyLongitude = weather.longitude;
  settings.skyYear = weather.year;
  settings.skyMonth = weather.month;
  settings.skyDay = weather.day;
  settings.skyWindDirectionDeg = windDirectionDegreesFromVector(weather.windDirection);
  settings.skyWindSpeed = weather.windSpeed;
  settings.skyCloudCoverage = weather.cloudCoverage;
  settings.skyCloudDensity = weather.cloudDensity;
  settings.skyCloudAltitudeMeters = weather.cloudAltitudeMeters;
  settings.skyCloudThicknessMeters = weather.cloudThicknessMeters;
  settings.skyCloudShadowStrength = weather.cloudShadowStrength;
  settings.skyCloudScale = weather.cloudScale;
  settings.skyCloudAnimationSpeed = weather.cloudAnimationSpeed;
  settings.skyPrecipitationRate = weather.precipitationRate;
  settings.skyRainOcclusionStrength = weather.rainOcclusionStrength;
  settings.skySnowCoverage = weather.snowCoverage;
  settings.skyHumidity = weather.humidity;
  settings.skyWetness = weather.wetness;
  settings.skyWetnessAccumulationEnabled = weather.wetnessAccumulationEnabled &&
    (weather.precipitationRate > 0.0f || weather.wetness > 0.0f || weather.snowCoverage > 0.0f);
  settings.skyWetnessAccumulationRate = weather.wetnessAccumulationRate;
  settings.skyWetnessDryingRate = weather.wetnessDryingRate;
  settings.skyLightningRate = weather.lightningRate;
  settings.skyFogDensity = weather.fogDensity;
  settings.skyVisibilityMeters = weather.visibilityMeters;
  settings.skyFogColor = weather.fogColor;
  settings.skyFogAnisotropy = weather.fogAnisotropy;
  settings.skyAirTurbidity = weather.airTurbidity;
  settings.skyGroundAlbedo = weather.groundAlbedo;
  settings.skyUseExplicitSunAngles = weather.useExplicitSunAngles;
  settings.skySunAzimuthDeg = weather.sunAzimuthDegrees;
  settings.skySunElevationDeg = weather.sunElevationDegrees;
  settings.skySunSize = weather.sunDiskSize;
  settings.skyMoonSize = weather.moonDiskSize;
  settings.skyLensDropletsEnabled = weather.lensDropletsEnabled;
  settings.skyLensDropletStrength = weather.lensDropletStrength;
}

void copyWeatherPresetToSettings(ViewerSettings& settings, int preset) {
  const int clampedPreset = std::clamp(preset, 0, 10);
  settings.skyWeatherPreset = clampedPreset;
  if (clampedPreset == 10) {
    return;
  }
  const bool skyEnabled = settings.skyEnabled;
  const bool weatherEnabled = settings.skyWeatherEnabled;
  copyWeatherSettingsToViewerSettings(
    settings, raisin::RayraiWindow::defaultWeatherSettings(weatherPresetFromIndex(clampedPreset)));
  settings.skyEnabled = skyEnabled;
  settings.skyWeatherEnabled = weatherEnabled;
  settings.skyWeatherPreset = clampedPreset;
}

raisin::RayraiWindow::WeatherSettings weatherSettingsFromViewerSettings(
  const ViewerSettings& settings) {
  auto weather = raisin::RayraiWindow::defaultWeatherSettings(weatherPresetFromIndex(settings.skyWeatherPreset));
  if (settings.skyWeatherPreset == 10) {
    weather.preset = raisin::RayraiWindow::WeatherPreset::Custom;
  }
  weather.enabled = settings.skyEnabled && settings.skyWeatherEnabled &&
    weatherDefaultEnabledForQuality(settings.renderQuality);
  weather.preset = weatherPresetFromIndex(settings.skyWeatherPreset);
  weather.quality = weatherQualityFromIndex(settings.skyWeatherQuality);
  weather.seed = static_cast<uint32_t>(std::max(settings.skyWeatherSeed, 1));
  weather.timeOfDayHours = settings.skyTimeOfDayHours;
  weather.latitude = settings.skyLatitude;
  weather.longitude = settings.skyLongitude;
  weather.year = settings.skyYear;
  weather.month = settings.skyMonth;
  weather.day = settings.skyDay;
  weather.windDirection = windDirectionFromDegrees(settings.skyWindDirectionDeg);
  weather.windSpeed = settings.skyWindSpeed;
  weather.transitionSeconds = 0.0f;
  weather.affectSensors = false;
  // When the user picks a non-Custom weather preset, the preset's own
  // cloud coverage / density / altitude / thickness should win — otherwise an
  // old persisted ViewerSettings (with skyCloudCoverage = 0.05 from before
  // clouds were a thing) silently overrides "Rain" with "almost no clouds".
  const bool useCustomCloudAuthoring = settings.skyWeatherPreset == 10;
  if (useCustomCloudAuthoring) {
    weather.cloudCoverage = settings.skyCloudCoverage;
    weather.cloudDensity = settings.skyCloudDensity;
  }
  // else: weather already holds the preset defaults from defaultWeatherSettings(preset)
  weather.cloudAltitudeMeters = settings.skyCloudAltitudeMeters;
  weather.cloudThicknessMeters = settings.skyCloudThicknessMeters;
  weather.cloudShadowStrength = settings.skyCloudShadowStrength;
  weather.cloudScale = settings.skyCloudScale;
  weather.cloudAnimationSpeed = settings.skyCloudAnimationSpeed;
  weather.precipitationRate = settings.skyPrecipitationRate;
  weather.rainOcclusionStrength = settings.skyRainOcclusionStrength;
  weather.snowCoverage = settings.skySnowCoverage;
  weather.humidity = settings.skyHumidity;
  weather.wetness = settings.skyWetness;
  weather.wetnessAccumulationEnabled = settings.skyWetnessAccumulationEnabled;
  weather.wetnessAccumulationRate = settings.skyWetnessAccumulationRate;
  weather.wetnessDryingRate = settings.skyWetnessDryingRate;
  weather.lightningRate = settings.skyLightningRate;
  weather.fogDensity = settings.skyFogDensity;
  weather.visibilityMeters = settings.skyVisibilityMeters;
  weather.fogColor = settings.skyFogColor;
  weather.fogAnisotropy = settings.skyFogAnisotropy;
  weather.airTurbidity = settings.skyAirTurbidity;
  weather.groundAlbedo = settings.skyGroundAlbedo;
  weather.useExplicitSunAngles = settings.skyUseExplicitSunAngles;
  weather.sunAzimuthDegrees = settings.skySunAzimuthDeg;
  weather.sunElevationDegrees = settings.skySunElevationDeg;
  weather.sunDiskSize = settings.skySunSize;
  weather.moonDiskSize = settings.skyMoonSize;
  weather.lensDropletsEnabled = settings.skyLensDropletsEnabled;
  weather.lensDropletStrength = settings.skyLensDropletStrength;
  return weather;
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
  settings.skyEnabled = true;
  settings.skySunStrength = 1.8f;
  settings.skySunSize = renderSettings.proceduralSkySunSize;
  settings.skyWeatherEnabled = weatherDefaultEnabledForQuality(settings.renderQuality);
  copyWeatherPresetToSettings(settings, 0);
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

bool applyAutomaticRenderQualityIfUnset(ViewerSettings& settings, int recommendedQuality) {
  if (settings.renderQualityUserSet) {
    return false;
  }
  copyRenderDefaultsToSettings(settings, std::clamp(recommendedQuality, 0, 3));
  settings.renderQualityUserSet = false;
  sanitizeViewerSettings(settings);
  return true;
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
  const bool highFidelityPbrAllowed = highFidelityPbrAllowedForQuality(settings.renderQuality);
  renderSettings.highFidelityPbr = highFidelityPbrAllowed && settings.highFidelityPbr;
  if (!highFidelityPbrAllowed) {
    renderSettings.forceSimpleMaterialShading = true;
  }
  renderSettings.pbrToneMapping = settings.pbrToneMapping;
  renderSettings.pbrExposure = settings.pbrExposure;
  renderSettings.pbrEnvironmentMaxLod = settings.pbrEnvironmentMaxLod;
  renderSettings.pbrEnvironmentIntensity = settings.pbrEnvironmentIntensity;
  renderSettings.pbrKeyLightIntensity = settings.pbrKeyLightIntensity;
  renderSettings.proceduralSkyBackgroundEnabled = settings.skyEnabled;
  renderSettings.proceduralSkySunStrength = settings.skySunStrength;
  renderSettings.proceduralSkySunSize = settings.skySunSize;
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
  viewer.setWeatherSettings(weatherSettingsFromViewerSettings(settings));
  // Apply the explicit cloud-quality user choice last so it wins over both
  // the render preset default and the weather-driven promotion. Auto leaves
  // cloudQuality at whatever the preset + weather logic resolved to.
  if (settings.skyCloudQuality != 0) {
    auto rs = viewer.getRenderQualitySettings();
    switch (settings.skyCloudQuality) {
      case 1: rs.cloudQuality = raisin::CloudQuality::Off; break;
      case 2: rs.cloudQuality = raisin::CloudQuality::Texture; break;
      case 3: rs.cloudQuality = raisin::CloudQuality::Volumetric; break;
      default: break;
    }
    // If user picked Off, make sure the layer flag follows; otherwise force
    // the cloud layer on so the chosen quality has something to draw.
    rs.proceduralCloudLayerEnabled = (settings.skyCloudQuality != 1);
    viewer.setRenderQualitySettings(rs);
  }
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

bool isTcpViewerSingleBodyControlType(int objectTypeRaw) {
  if (objectTypeRaw < 0 || objectTypeRaw == 10 || objectTypeRaw == 11) {
    return false;
  }
  switch (static_cast<raisim::ObjectType>(objectTypeRaw)) {
    case raisim::ObjectType::SPHERE:
    case raisim::ObjectType::BOX:
    case raisim::ObjectType::CYLINDER:
    case raisim::ObjectType::CAPSULE:
    case raisim::ObjectType::MESH:
      return true;
    default:
      return false;
  }
}

bool supportsTcpViewerForceControl(const VisualEntry* entry) {
  if (!entry) {
    return false;
  }
  return entry->isArticulated || isTcpViewerSingleBodyControlType(entry->objectTypeRaw);
}

bool supportsTcpViewerPoseControl(const VisualEntry* entry) {
  if (!entry || entry->isCollision || entry->isArticulated) {
    return false;
  }
  return isTcpViewerSingleBodyControlType(entry->objectTypeRaw);
}

const char* tcpViewerJointTypeLabel(int32_t rawType) {
  const auto type = static_cast<raisim::Joint::Type>(rawType);
  switch (type) {
    case raisim::Joint::Type::REVOLUTE: return "rev";
    case raisim::Joint::Type::PRISMATIC: return "pris";
    case raisim::Joint::Type::SPHERICAL: return "sph";
    case raisim::Joint::Type::FLOATING: return "float";
    case raisim::Joint::Type::FIXED: return "fixed";
    default: return "joint";
  }
}

glm::vec4 normalizedWxyz(glm::vec4 quat) {
  const float norm2 = quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z;
  if (!std::isfinite(norm2) || norm2 <= 1e-12f) {
    return glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
  }
  const float invNorm = 1.0f / std::sqrt(norm2);
  return quat * invNorm;
}

void normalizeWxyzSlice(std::vector<float>& values, size_t offset) {
  if (offset + 3 >= values.size()) {
    return;
  }
  glm::vec4 quat(values[offset], values[offset + 1], values[offset + 2], values[offset + 3]);
  quat = normalizedWxyz(quat);
  values[offset] = quat.w;
  values[offset + 1] = quat.x;
  values[offset + 2] = quat.y;
  values[offset + 3] = quat.z;
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

bool objectMatchesFilter(const ObjectListItem& item, const std::string& filterLower) {
  if (filterLower.empty()) {
    return true;
  }
  std::ostringstream haystack;
  haystack << item.name << " " << objectTypeLabel(item.objectTypeRaw) << " tag " << item.tag
           << " index " << item.index << (item.isCollision ? " collision" : " visual");
  return toLowerAscii(haystack.str()).find(filterLower) != std::string::npos;
}

bool objectLessByMode(const ObjectListItem& lhs, const ObjectListItem& rhs, int mode) {
  switch (mode) {
    case 1:
      if (lhs.objectTypeRaw != rhs.objectTypeRaw) return lhs.objectTypeRaw < rhs.objectTypeRaw;
      return lhs.name < rhs.name;
    case 2:
      return lhs.tag < rhs.tag;
    case 3:
      return lhs.index < rhs.index;
    case 0:
    default:
      return lhs.name < rhs.name;
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

std::string jsonEscape(const std::string& value) {
  std::ostringstream out;
  for (unsigned char c : value) {
    switch (c) {
      case '\\': out << "\\\\"; break;
      case '"': out << "\\\""; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c)
              << std::dec << std::setfill(' ');
        } else {
          out << static_cast<char>(c);
        }
    }
  }
  return out.str();
}

std::string csvEscape(const std::string& value) {
  const bool quote = value.find_first_of(",\n\r\"") != std::string::npos;
  if (!quote) {
    return value;
  }
  std::string out = "\"";
  for (char c : value) {
    if (c == '"') out += "\"\"";
    else out += c;
  }
  out += "\"";
  return out;
}

std::filesystem::path timestampedDataPath(const std::filesystem::path& dir, const char* prefix,
                                          const char* extension) {
  const auto now = std::chrono::system_clock::now();
  const std::time_t raw = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#if defined(_WIN32)
  localtime_s(&tm, &raw);
#else
  localtime_r(&raw, &tm);
#endif
  std::ostringstream name;
  name << (prefix ? prefix : "rayrai") << "_" << std::put_time(&tm, "%Y%m%d_%H%M%S")
       << (extension ? extension : "");
  return dir / name.str();
}

template <typename T>
bool writeBinaryPod(std::ofstream& output, const T& value) {
  output.write(reinterpret_cast<const char*>(&value), sizeof(T));
  return static_cast<bool>(output);
}

class SessionRecorder {
 public:
  bool open(const std::filesystem::path& path, std::string& status) {
    close();
    std::error_code ec;
    if (!path.parent_path().empty()) {
      std::filesystem::create_directories(path.parent_path(), ec);
      if (ec) {
        status = "session record failed: cannot create directory";
        return false;
      }
    }
    output_.open(path, std::ios::binary);
    if (!output_) {
      status = "session record failed: cannot open " + path.string();
      return false;
    }
    output_.write(kSessionMagic, std::strlen(kSessionMagic));
    if (!output_) {
      status = "session record failed: cannot write header";
      close();
      return false;
    }
    path_ = path;
    frames_ = 0;
    bytes_ = 0;
    start_ = std::chrono::steady_clock::now();
    status = "recording " + path.string();
    return true;
  }

  void close() {
    if (output_.is_open()) {
      output_.close();
    }
  }

  bool active() const { return output_.is_open(); }
  size_t frameCount() const { return frames_; }
  size_t byteCount() const { return bytes_; }
  std::string pathString() const { return path_.string(); }

  bool record(const std::vector<char>& payload, std::chrono::steady_clock::time_point now,
              std::string& status) {
    if (!active()) {
      return false;
    }
    const auto micros = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(now - start_).count());
    if (payload.size() > std::numeric_limits<uint32_t>::max()) {
      status = "session record failed: payload too large";
      close();
      return false;
    }
    const uint32_t size = static_cast<uint32_t>(payload.size());
    if (!writeBinaryPod(output_, micros) || !writeBinaryPod(output_, size)) {
      status = "session record failed: write error";
      close();
      return false;
    }
    if (!payload.empty()) {
      output_.write(payload.data(), static_cast<std::streamsize>(payload.size()));
    }
    if (!output_) {
      status = "session record failed: write error";
      close();
      return false;
    }
    ++frames_;
    bytes_ += payload.size();
    return true;
  }

 private:
  std::ofstream output_;
  std::filesystem::path path_;
  std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();
  size_t frames_ = 0;
  size_t bytes_ = 0;
};

bool loadSessionFile(const std::filesystem::path& path, std::vector<RecordedFrame>& frames,
                     std::string& status) {
  frames.clear();
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    status = "replay failed: cannot open " + path.string();
    return false;
  }
  std::string magic(std::strlen(kSessionMagic), '\0');
  input.read(magic.data(), static_cast<std::streamsize>(magic.size()));
  if (magic != kSessionMagic) {
    status = "replay failed: unsupported session file";
    return false;
  }
  while (true) {
    RecordedFrame frame;
    uint32_t size = 0;
    input.read(reinterpret_cast<char*>(&frame.timeMicros), sizeof(frame.timeMicros));
    if (!input) {
      if (input.gcount() == 0 && input.eof()) {
        break;
      }
      status = "replay failed: truncated frame header";
      frames.clear();
      return false;
    }
    input.read(reinterpret_cast<char*>(&size), sizeof(size));
    if (!input) {
      status = "replay failed: truncated frame header";
      frames.clear();
      return false;
    }
    if (size > static_cast<uint32_t>(raisin::tcp_viewer::kMaxMessageBytes)) {
      status = "replay failed: frame exceeds max message size";
      frames.clear();
      return false;
    }
    frame.payload.resize(size);
    if (size > 0) {
      input.read(frame.payload.data(), size);
      if (!input) {
        status = "replay failed: truncated frame";
        frames.clear();
        return false;
      }
    }
    frames.push_back(std::move(frame));
  }
  status = "loaded replay " + path.string() + " (" + std::to_string(frames.size()) + " frames)";
  return !frames.empty();
}

std::vector<AssetDiagnostic> collectAssetDiagnostics(const RemoteScene& scene) {
  std::vector<AssetDiagnostic> assets;
  std::unordered_set<uint64_t> seen;
  for (const auto& snapshot : scene.getVisualEntries()) {
    const auto& entry = snapshot.entry;
    if (entry.meshFile.empty()) {
      continue;
    }
    if (!seen.insert(visualMotionKey(snapshot.tag, snapshot.index)).second) {
      continue;
    }
    AssetDiagnostic asset;
    asset.tag = snapshot.tag;
    asset.index = snapshot.index;
    asset.name = entry.objectName.empty() ? scene.getObjectName(snapshot.tag) : entry.objectName;
    asset.meshFile = entry.meshFile;
    asset.meshPath = entry.meshPath;
    asset.resourceDir = entry.resourceDir;
    asset.resolved = !entry.meshPath.empty();
    assets.push_back(std::move(asset));
  }
  std::sort(assets.begin(), assets.end(), [](const AssetDiagnostic& lhs, const AssetDiagnostic& rhs) {
    if (lhs.resolved != rhs.resolved) return !lhs.resolved;
    if (lhs.meshFile != rhs.meshFile) return lhs.meshFile < rhs.meshFile;
    return lhs.tag < rhs.tag;
  });
  return assets;
}

size_t unresolvedAssetCount(const RemoteScene& scene) {
  size_t count = 0;
  std::unordered_set<uint64_t> seen;
  for (const auto& snapshot : scene.getVisualEntries()) {
    const auto& entry = snapshot.entry;
    if (entry.meshFile.empty()) {
      continue;
    }
    if (seen.insert(visualMotionKey(snapshot.tag, snapshot.index)).second && entry.meshPath.empty()) {
      ++count;
    }
  }
  return count;
}

void pushPacketSample(std::deque<PacketSample>& samples, const PacketSample& sample) {
  samples.push_back(sample);
  while (samples.size() > 180) {
    samples.pop_front();
  }
}

std::array<float, kTransferRateGraphBuckets> buildTransferRateGraph(
  const std::deque<PacketSample>& samples, double endTimeSeconds,
  double windowSeconds = kTransferRateGraphWindowSeconds) {
  std::array<float, kTransferRateGraphBuckets> rates{};
  if (samples.empty() || windowSeconds <= 0.0 || !std::isfinite(windowSeconds) ||
      !std::isfinite(endTimeSeconds)) {
    return rates;
  }

  const double bucketSeconds = windowSeconds / static_cast<double>(rates.size());
  if (bucketSeconds <= 0.0) {
    return rates;
  }
  const double startTimeSeconds = endTimeSeconds - windowSeconds;
  for (const auto& sample : samples) {
    if (sample.bytes <= 0 || sample.timeSeconds < startTimeSeconds ||
        sample.timeSeconds > endTimeSeconds) {
      continue;
    }
    size_t bucket = sample.timeSeconds >= endTimeSeconds
      ? rates.size() - 1
      : static_cast<size_t>((sample.timeSeconds - startTimeSeconds) / bucketSeconds);
    bucket = std::min(bucket, rates.size() - 1);
    rates[bucket] += static_cast<float>(static_cast<double>(sample.bytes) / 1024.0 / bucketSeconds);
  }
  return rates;
}

float maxTransferRate(const std::array<float, kTransferRateGraphBuckets>& rates) {
  return *std::max_element(rates.begin(), rates.end());
}

bool exportSceneJson(const std::filesystem::path& path, const RemoteScene& scene,
                     const std::vector<AssetDiagnostic>& assets, std::string& status) {
  std::error_code ec;
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path(), ec);
    if (ec) {
      status = "scene export failed: cannot create directory";
      return false;
    }
  }
  std::ofstream output(path);
  if (!output) {
    status = "scene export failed: cannot open " + path.string();
    return false;
  }
  output << std::setprecision(9);
  output << "{\n";
  output << "  \"world_time\": ";
  if (scene.hasServerWorldTime()) output << scene.getServerWorldTime(); else output << "null";
  output << ",\n  \"counts\": {\n";
  output << "    \"selectable\": " << scene.selectableObjectCount() << ",\n";
  output << "    \"visuals\": " << scene.visualCount() << ",\n";
  output << "    \"instanced\": " << scene.instancedCount() << ",\n";
  output << "    \"point_clouds\": " << scene.pointCloudCount() << "\n  },\n";
  output << "  \"objects\": [\n";
  const auto items = scene.getSelectableObjects();
  bool first = true;
  for (const auto& item : items) {
    if (!item.visual || isContactItem(item)) continue;
    uint32_t tag = 0;
    int index = 0;
    const VisualEntry* entry = nullptr;
    if (!scene.getVisualInfo(item.visual.get(), tag, index, entry) || !entry) continue;
    if (!first) output << ",\n";
    first = false;
    output << "    {\"tag\": " << tag << ", \"index\": " << index
           << ", \"name\": \"" << jsonEscape(item.name.empty() ? entry->objectName : item.name) << "\""
           << ", \"type\": \"" << objectTypeLabel(entry->objectTypeRaw) << "\""
           << ", \"collision\": " << (entry->isCollision ? "true" : "false")
           << ", \"mesh\": \"" << jsonEscape(entry->meshFile) << "\""
           << ", \"mesh_path\": \"" << jsonEscape(entry->meshPath) << "\""
           << ", \"position\": [" << entry->lastPos.x << ", " << entry->lastPos.y << ", " << entry->lastPos.z << "]"
           << ", \"quaternion_wxyz\": [" << entry->lastQuat.w << ", " << entry->lastQuat.x << ", "
           << entry->lastQuat.y << ", " << entry->lastQuat.z << "]}";
  }
  output << "\n  ],\n  \"assets\": [\n";
  for (size_t i = 0; i < assets.size(); ++i) {
    const auto& asset = assets[i];
    output << "    {\"tag\": " << asset.tag << ", \"index\": " << asset.index
           << ", \"name\": \"" << jsonEscape(asset.name)
           << "\", \"mesh\": \"" << jsonEscape(asset.meshFile)
           << "\", \"resource_dir\": \"" << jsonEscape(asset.resourceDir)
           << "\", \"resolved_path\": \"" << jsonEscape(asset.meshPath)
           << "\", \"resolved\": " << (asset.resolved ? "true" : "false") << "}";
    if (i + 1 < assets.size()) output << ",";
    output << "\n";
  }
  output << "  ]\n}\n";
  status = "exported scene " + path.string();
  return true;
}

void writeTrajectoryRows(std::ofstream& output, const RemoteScene& scene, double timeSeconds) {
  if (!output) {
    return;
  }
  output << std::setprecision(9);
  for (const auto& item : scene.getSelectableObjects()) {
    if (!item.visual || isContactItem(item)) continue;
    uint32_t tag = 0;
    int index = 0;
    const VisualEntry* entry = nullptr;
    if (!scene.getVisualInfo(item.visual.get(), tag, index, entry) || !entry) continue;
    const std::string name = item.name.empty() ? entry->objectName : item.name;
    output << timeSeconds << ',' << tag << ',' << index << ',' << csvEscape(name) << ','
           << csvEscape(objectTypeLabel(entry->objectTypeRaw)) << ','
           << entry->lastPos.x << ',' << entry->lastPos.y << ',' << entry->lastPos.z << ','
           << entry->lastQuat.w << ',' << entry->lastQuat.x << ',' << entry->lastQuat.y << ','
           << entry->lastQuat.z << '\n';
  }
}

void loadEndpointList(const std::filesystem::path& path, std::vector<ConnectionEntry>& connections) {
  std::ifstream input(path);
  if (!input) {
    std::cerr << "WARN: failed to open server list " << path << "\n";
    return;
  }
  std::string line;
  while (std::getline(input, line)) {
    const auto comment = line.find('#');
    if (comment != std::string::npos) line.resize(comment);
    line = trimAscii(line);
    if (line.empty()) continue;
    ConnectionEntry entry;
    if (parseConnectionLabel(line, entry)) {
      recordConnection(connections, entry.host, entry.port);
    }
  }
}

void drawSliderInsideLabel(const char* label, const char* valueText, bool disabled) {
  const ImVec2 itemMin = ImGui::GetItemRectMin();
  const ImVec2 itemMax = ImGui::GetItemRectMax();
  const ImGuiStyle& style = ImGui::GetStyle();
  const float padding = style.FramePadding.x + 2.0f;
  const float textHeight = ImGui::GetFontSize();
  const float textY = itemMin.y + (itemMax.y - itemMin.y - textHeight) * 0.5f;
  const float itemWidth = itemMax.x - itemMin.x;
  const ImVec2 labelSize = ImGui::CalcTextSize(label);
  const ImVec2 valueSize = ImGui::CalcTextSize(valueText);
  const bool visuallyDisabled = disabled || style.Alpha < 0.999f;
  const ImU32 textColor = ImGui::GetColorU32(visuallyDisabled ? ImGuiCol_TextDisabled : ImGuiCol_Text);
  const ImU32 shadowColor = ImGui::GetColorU32(ImVec4(0.0f, 0.0f, 0.0f, visuallyDisabled ? 0.16f : 0.34f));
  ImDrawList* drawList = ImGui::GetWindowDrawList();
  const auto drawTextWithShadow = [&](const ImVec2& pos, const char* text) {
    drawList->AddText(ImVec2(pos.x + 1.0f, pos.y + 1.0f), shadowColor, text);
    drawList->AddText(pos, textColor, text);
  };

  drawList->PushClipRect(itemMin, itemMax, true);
  if (labelSize.x + valueSize.x + padding * 3.0f <= itemWidth) {
    drawTextWithShadow(ImVec2(itemMin.x + padding, textY), label);
    drawTextWithShadow(ImVec2(itemMax.x - padding - valueSize.x, textY), valueText);
  } else {
    char combined[128];
    std::snprintf(combined, sizeof(combined), "%s: %s", label, valueText);
    const ImVec2 combinedSize = ImGui::CalcTextSize(combined);
    const float textX = itemMin.x + std::max(padding, (itemWidth - combinedSize.x) * 0.5f);
    drawTextWithShadow(ImVec2(textX, textY), combined);
  }
  drawList->PopClipRect();
}

bool drawInlineLabelSliderFloat(const char* id, const char* label, float* value, float min, float max,
                                const char* format, float itemWidth = 0.0f, bool disabled = false) {
  ImGui::PushID(id ? id : label);
  if (itemWidth > 0.0f) {
    ImGui::PushItemWidth(itemWidth);
  }
  ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
  const bool changed = ImGui::SliderFloat("##slider", value, min, max, format);
  ImGui::PopStyleColor();
  if (itemWidth > 0.0f) {
    ImGui::PopItemWidth();
  }

  char valueBuf[32];
  std::snprintf(valueBuf, sizeof(valueBuf), format, *value);
  drawSliderInsideLabel(label, valueBuf, disabled);
  ImGui::PopID();
  return changed;
}

bool drawInlineLabelSliderInt(const char* id, const char* label, int* value, int min, int max,
                              const char* format = "%d", float itemWidth = 0.0f,
                              bool disabled = false) {
  ImGui::PushID(id ? id : label);
  if (itemWidth > 0.0f) {
    ImGui::PushItemWidth(itemWidth);
  }
  ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
  const bool changed = ImGui::SliderInt("##slider", value, min, max, format);
  ImGui::PopStyleColor();
  if (itemWidth > 0.0f) {
    ImGui::PopItemWidth();
  }

  char valueBuf[32];
  std::snprintf(valueBuf, sizeof(valueBuf), format, *value);
  drawSliderInsideLabel(label, valueBuf, disabled);
  ImGui::PopID();
  return changed;
}

bool drawOverlaySlider(const char* id, const char* label, float* value, float min, float max,
  const char* format, float valueWidth, float itemWidth, bool disabled) {
  static_cast<void>(valueWidth);
  return drawInlineLabelSliderFloat(id, label, value, min, max, format, itemWidth, disabled);
}

struct TcpViewerPoseAssignPriorityLow {};
struct TcpViewerPoseAssignPriorityHigh : TcpViewerPoseAssignPriorityLow {};

template <typename Vec3>
auto setTcpViewerZeroPosition(Vec3& position, TcpViewerPoseAssignPriorityHigh)
  -> decltype(position.x, position.y, position.z, void()) {
  position.x = 0.0f;
  position.y = 0.0f;
  position.z = 0.0f;
}

template <typename Vec3>
auto setTcpViewerZeroPosition(Vec3& position, TcpViewerPoseAssignPriorityLow)
  -> decltype(position[0], position[1], position[2], void()) {
  position[0] = 0.0;
  position[1] = 0.0;
  position[2] = 0.0;
}

template <typename Quat>
auto setTcpViewerIdentityQuaternion(Quat& quaternion, TcpViewerPoseAssignPriorityHigh)
  -> decltype(quaternion.w, quaternion.x, quaternion.y, quaternion.z, void()) {
  quaternion.w = 1.0f;
  quaternion.x = 0.0f;
  quaternion.y = 0.0f;
  quaternion.z = 0.0f;
}

template <typename Quat>
auto setTcpViewerIdentityQuaternion(Quat& quaternion, TcpViewerPoseAssignPriorityLow)
  -> decltype(quaternion[0], quaternion[1], quaternion[2], quaternion[3], void()) {
  quaternion[0] = 1.0;
  quaternion[1] = 0.0;
  quaternion[2] = 0.0;
  quaternion[3] = 0.0;
}

template <typename Pose>
void setTcpViewerIdentityPose(Pose& pose) {
  setTcpViewerZeroPosition(pose.position, TcpViewerPoseAssignPriorityHigh{});
  setTcpViewerIdentityQuaternion(pose.quaternion, TcpViewerPoseAssignPriorityHigh{});
}

enum class TcpViewerIconKind {
  Connect = 0,
  Disconnect,
  Refresh,
  Save,
  Home,
  Focus,
  Camera,
  Folder,
  Export,
  Options,
  Robot,
  Reset,
  Exit,
  Pause,
  Play,
  Step,
  StepFast,
  Count
};

struct TcpViewerIcon {
  unsigned int texture = 0;
  int width = 0;
  int height = 0;

  [[nodiscard]] bool valid() const { return texture != 0 && width > 0 && height > 0; }
};

struct TcpViewerImageTexture {
  unsigned int texture = 0;
  int width = 0;
  int height = 0;
  ImVec2 uvMin{0.0f, 0.0f};
  ImVec2 uvMax{1.0f, 1.0f};

  [[nodiscard]] bool valid() const { return texture != 0 && width > 0 && height > 0; }

  void release() {
    if (texture != 0) {
      gl::glDeleteTextures(1, &texture);
    }
    texture = 0;
    width = 0;
    height = 0;
    uvMin = ImVec2(0.0f, 0.0f);
    uvMax = ImVec2(1.0f, 1.0f);
  }
};

const char* tcpViewerIconFileName(TcpViewerIconKind kind) {
  switch (kind) {
    case TcpViewerIconKind::Connect: return "connect_uicons_sr_plug_connection.png";
    case TcpViewerIconKind::Disconnect: return "disconnect_uicons_sr_link_horizontal_slash.png";
    case TcpViewerIconKind::Refresh: return "refresh_uicons_sr_refresh.png";
    case TcpViewerIconKind::Save: return "save_uicons_sr_disk.png";
    case TcpViewerIconKind::Home: return "home_uicons_sr_house_signal.png";
    case TcpViewerIconKind::Focus: return "focus_uicons_sr_target.png";
    case TcpViewerIconKind::Camera: return "camera_uicons_sr_camera_viewfinder.png";
    case TcpViewerIconKind::Folder: return "folder_uicons_sr_folder_open.png";
    case TcpViewerIconKind::Export: return "export_uicons_sr_file_export.png";
    case TcpViewerIconKind::Options: return "options_uicons_sr_settings_sliders.png";
    case TcpViewerIconKind::Robot: return "robot_uicons_sr_robot.png";
    case TcpViewerIconKind::Reset: return "reset_uicons_sr_rotate_left.png";
    case TcpViewerIconKind::Exit: return "exit_uicons_sr_sign_out_alt.png";
    case TcpViewerIconKind::Pause: return "pause_uicons_sr_pause.png";
    case TcpViewerIconKind::Play: return "play_uicons_sr_play.png";
    case TcpViewerIconKind::Step: return "step_uicons_sr_step_forward.png";
    case TcpViewerIconKind::StepFast: return "step_fast_uicons_sr_forward_fast.png";
    case TcpViewerIconKind::Count: break;
  }
  return "";
}

bool directoryExists(const std::filesystem::path& path) {
  std::error_code ec;
  return std::filesystem::is_directory(path, ec);
}

std::filesystem::path absolutePathNoThrow(const std::filesystem::path& path) {
  std::error_code ec;
  std::filesystem::path absolute = std::filesystem::absolute(path, ec);
  if (ec) {
    return path;
  }
  return absolute.lexically_normal();
}

std::filesystem::path findTcpViewerIconDir(const std::filesystem::path& binaryDir) {
  std::vector<std::filesystem::path> candidates;
  const std::filesystem::path sourceDir = std::filesystem::path(__FILE__).parent_path();
  if (!sourceDir.empty()) {
    candidates.push_back(sourceDir / "../assets/flaticon/tcp_viewer");
  }
  if (!binaryDir.empty()) {
    candidates.push_back(binaryDir / "assets/flaticon/tcp_viewer");
    candidates.push_back(binaryDir / "../assets/flaticon/tcp_viewer");
    candidates.push_back(binaryDir / "../share/rayrai/assets/flaticon/tcp_viewer");
  }
  std::error_code ec;
  const std::filesystem::path cwd = std::filesystem::current_path(ec);
  if (!ec) {
    candidates.push_back(cwd / "visualizer/rayrai/assets/flaticon/tcp_viewer");
    candidates.push_back(cwd / "../visualizer/rayrai/assets/flaticon/tcp_viewer");
    candidates.push_back(cwd / "examples/src/rayrai/assets/flaticon/tcp_viewer");
    candidates.push_back(cwd / "../examples/src/rayrai/assets/flaticon/tcp_viewer");
  }
  for (const auto& candidate : candidates) {
    const std::filesystem::path absolute = absolutePathNoThrow(candidate);
    if (directoryExists(absolute)) {
      return absolute;
    }
  }
  return {};
}

std::filesystem::path findTcpViewerIconDir() {
  return findTcpViewerIconDir({});
}

std::filesystem::path findRaisimLogoPath(const std::filesystem::path& binaryDir) {
  std::vector<std::filesystem::path> candidates;
  const std::filesystem::path sourceDir = std::filesystem::path(__FILE__).parent_path();
  if (!sourceDir.empty()) {
    candidates.push_back(sourceDir / "../../../logo.png");
    candidates.push_back(sourceDir / "../../../../logo.png");
    candidates.push_back(sourceDir / "../../../../docs/logo.png");
  }
  if (!binaryDir.empty()) {
    candidates.push_back(binaryDir / "logo.png");
    candidates.push_back(binaryDir / "../logo.png");
    candidates.push_back(binaryDir / "../../logo.png");
    candidates.push_back(binaryDir / "../share/rayrai/logo.png");
  }
  std::error_code ec;
  const std::filesystem::path cwd = std::filesystem::current_path(ec);
  if (!ec) {
    candidates.push_back(cwd / "logo.png");
    candidates.push_back(cwd / "docs/logo.png");
    candidates.push_back(cwd / "../logo.png");
    candidates.push_back(cwd / "../docs/logo.png");
  }
  for (const auto& candidate : candidates) {
    const std::filesystem::path absolute = absolutePathNoThrow(candidate);
    if (std::filesystem::is_regular_file(absolute, ec) && !ec) {
      return absolute;
    }
    ec.clear();
  }
  return {};
}

bool loadTcpViewerImageTexture(const std::filesystem::path& path, TcpViewerImageTexture& image) {
  if (path.empty()) {
    return false;
  }
  int width = 0;
  int height = 0;
  int channels = 0;
  unsigned char* pixels = stbi_load(path.string().c_str(), &width, &height, &channels, 4);
  if (!pixels || width <= 0 || height <= 0) {
    if (pixels) {
      stbi_image_free(pixels);
    }
    return false;
  }

  int alphaMinX = width;
  int alphaMinY = height;
  int alphaMaxX = -1;
  int alphaMaxY = -1;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const unsigned char alpha = pixels[(y * width + x) * 4 + 3];
      if (alpha <= 8) {
        continue;
      }
      alphaMinX = std::min(alphaMinX, x);
      alphaMinY = std::min(alphaMinY, y);
      alphaMaxX = std::max(alphaMaxX, x);
      alphaMaxY = std::max(alphaMaxY, y);
    }
  }
  ImVec2 uvMin(0.0f, 0.0f);
  ImVec2 uvMax(1.0f, 1.0f);
  if (alphaMaxX >= alphaMinX && alphaMaxY >= alphaMinY) {
    constexpr int cropMarginPixels = 2;
    alphaMinX = std::max(0, alphaMinX - cropMarginPixels);
    alphaMinY = std::max(0, alphaMinY - cropMarginPixels);
    alphaMaxX = std::min(width - 1, alphaMaxX + cropMarginPixels);
    alphaMaxY = std::min(height - 1, alphaMaxY + cropMarginPixels);
    uvMin = ImVec2(static_cast<float>(alphaMinX) / static_cast<float>(width),
      static_cast<float>(alphaMinY) / static_cast<float>(height));
    uvMax = ImVec2(static_cast<float>(alphaMaxX + 1) / static_cast<float>(width),
      static_cast<float>(alphaMaxY + 1) / static_cast<float>(height));
  }

  unsigned int texture = 0;
  gl::glGenTextures(1, &texture);
  if (texture == 0) {
    stbi_image_free(pixels);
    return false;
  }

  gl::GLint previousAlignment = 4;
  gl::glGetIntegerv(gl::GL_UNPACK_ALIGNMENT, &previousAlignment);
  gl::glBindTexture(gl::GL_TEXTURE_2D, texture);
  gl::glPixelStorei(gl::GL_UNPACK_ALIGNMENT, 1);
  gl::glTexImage2D(gl::GL_TEXTURE_2D, 0, static_cast<gl::GLint>(gl::GL_RGBA8), width, height, 0,
    gl::GL_RGBA, gl::GL_UNSIGNED_BYTE, pixels);
  gl::glPixelStorei(gl::GL_UNPACK_ALIGNMENT, previousAlignment);
  gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_MIN_FILTER,
    static_cast<gl::GLint>(gl::GL_LINEAR));
  gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_MAG_FILTER,
    static_cast<gl::GLint>(gl::GL_LINEAR));
  gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_WRAP_S,
    static_cast<gl::GLint>(gl::GL_CLAMP_TO_EDGE));
  gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_WRAP_T,
    static_cast<gl::GLint>(gl::GL_CLAMP_TO_EDGE));
  gl::glBindTexture(gl::GL_TEXTURE_2D, 0);
  stbi_image_free(pixels);

  image.release();
  image.texture = texture;
  image.width = width;
  image.height = height;
  image.uvMin = uvMin;
  image.uvMax = uvMax;
  return true;
}

bool loadTcpViewerIconTexture(const std::filesystem::path& path, TcpViewerIcon& icon) {
  int width = 0;
  int height = 0;
  int channels = 0;
  unsigned char* pixels = stbi_load(path.string().c_str(), &width, &height, &channels, 4);
  if (!pixels || width <= 0 || height <= 0) {
    if (pixels) {
      stbi_image_free(pixels);
    }
    return false;
  }

  for (int i = 0; i < width * height; ++i) {
    unsigned char* pixel = pixels + i * 4;
    if (pixel[3] == 0) {
      continue;
    }
    // Treat icon artwork as an alpha mask so button colors stay consistent
    // across Flaticon styles and remain legible on dark TCP viewer panels.
    pixel[0] = 255;
    pixel[1] = 255;
    pixel[2] = 255;
  }

  unsigned int texture = 0;
  gl::glGenTextures(1, &texture);
  if (texture == 0) {
    stbi_image_free(pixels);
    return false;
  }

  gl::GLint previousAlignment = 4;
  gl::glGetIntegerv(gl::GL_UNPACK_ALIGNMENT, &previousAlignment);
  gl::glBindTexture(gl::GL_TEXTURE_2D, texture);
  gl::glPixelStorei(gl::GL_UNPACK_ALIGNMENT, 1);
  gl::glTexImage2D(gl::GL_TEXTURE_2D, 0, static_cast<gl::GLint>(gl::GL_RGBA8), width, height, 0,
    gl::GL_RGBA, gl::GL_UNSIGNED_BYTE, pixels);
  gl::glPixelStorei(gl::GL_UNPACK_ALIGNMENT, previousAlignment);
  gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_MIN_FILTER,
    static_cast<gl::GLint>(gl::GL_LINEAR));
  gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_MAG_FILTER,
    static_cast<gl::GLint>(gl::GL_LINEAR));
  gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_WRAP_S,
    static_cast<gl::GLint>(gl::GL_CLAMP_TO_EDGE));
  gl::glTexParameteri(gl::GL_TEXTURE_2D, gl::GL_TEXTURE_WRAP_T,
    static_cast<gl::GLint>(gl::GL_CLAMP_TO_EDGE));
  gl::glBindTexture(gl::GL_TEXTURE_2D, 0);
  stbi_image_free(pixels);

  icon.texture = texture;
  icon.width = width;
  icon.height = height;
  return true;
}

struct TcpViewerIcons {
  std::array<TcpViewerIcon, static_cast<size_t>(TcpViewerIconKind::Count)> icons{};

  bool load(const std::filesystem::path& iconDir) {
    if (iconDir.empty()) {
      return false;
    }
    bool loadedAny = false;
    for (size_t i = 0; i < icons.size(); ++i) {
      const auto kind = static_cast<TcpViewerIconKind>(i);
      const std::filesystem::path path = iconDir / tcpViewerIconFileName(kind);
      loadedAny |= loadTcpViewerIconTexture(path, icons[i]);
    }
    return loadedAny;
  }

  void release() {
    for (auto& icon : icons) {
      if (icon.texture != 0) {
        gl::glDeleteTextures(1, &icon.texture);
      }
      icon = {};
    }
  }

  [[nodiscard]] const TcpViewerIcon* get(TcpViewerIconKind kind) const {
    const auto index = static_cast<size_t>(kind);
    if (index >= icons.size() || !icons[index].valid()) {
      return nullptr;
    }
    return &icons[index];
  }
};

ImVec4 tcpViewerIconTint(TcpViewerIconKind kind, bool hovered, bool active) {
  ImVec4 color;
  switch (kind) {
    case TcpViewerIconKind::Connect: color = ImVec4(0.30f, 0.95f, 0.58f, 1.0f); break;
    case TcpViewerIconKind::Disconnect: color = ImVec4(1.00f, 0.44f, 0.38f, 1.0f); break;
    case TcpViewerIconKind::Refresh: color = ImVec4(0.40f, 0.72f, 1.00f, 1.0f); break;
    case TcpViewerIconKind::Save: color = ImVec4(1.00f, 0.72f, 0.26f, 1.0f); break;
    case TcpViewerIconKind::Home: color = ImVec4(0.46f, 0.92f, 0.96f, 1.0f); break;
    case TcpViewerIconKind::Focus: color = ImVec4(0.76f, 0.66f, 1.00f, 1.0f); break;
    case TcpViewerIconKind::Camera: color = ImVec4(0.55f, 0.86f, 1.00f, 1.0f); break;
    case TcpViewerIconKind::Folder: color = ImVec4(1.00f, 0.78f, 0.36f, 1.0f); break;
    case TcpViewerIconKind::Export: color = ImVec4(0.38f, 0.96f, 0.78f, 1.0f); break;
    case TcpViewerIconKind::Options: color = ImVec4(0.93f, 0.72f, 1.00f, 1.0f); break;
    case TcpViewerIconKind::Robot: color = ImVec4(0.55f, 0.85f, 1.00f, 1.0f); break;
    case TcpViewerIconKind::Reset: color = ImVec4(0.40f, 0.72f, 1.00f, 1.0f); break;
    case TcpViewerIconKind::Exit: color = ImVec4(1.00f, 0.55f, 0.42f, 1.0f); break;
    case TcpViewerIconKind::Pause: color = ImVec4(1.00f, 0.78f, 0.36f, 1.0f); break;
    case TcpViewerIconKind::Play: color = ImVec4(0.30f, 0.95f, 0.58f, 1.0f); break;
    case TcpViewerIconKind::Step: color = ImVec4(0.55f, 0.86f, 1.00f, 1.0f); break;
    case TcpViewerIconKind::StepFast: color = ImVec4(0.55f, 0.86f, 1.00f, 1.0f); break;
    case TcpViewerIconKind::Count: color = ImVec4(0.92f, 0.94f, 0.98f, 1.0f); break;
  }
  const float boost = active ? 1.18f : (hovered ? 1.08f : 1.0f);
  color.x = std::min(color.x * boost, 1.0f);
  color.y = std::min(color.y * boost, 1.0f);
  color.z = std::min(color.z * boost, 1.0f);
  return color;
}

ImVec2 iconTextButtonSize(const char* label, ImVec2 requestedSize = ImVec2(0.0f, 0.0f)) {
  const ImGuiStyle& style = ImGui::GetStyle();
  const ImVec2 textSize = ImGui::CalcTextSize(label);
  const float iconSize = std::round(ImGui::GetFontSize() * 0.95f);
  const ImVec2 minSize(
    textSize.x + iconSize + style.ItemInnerSpacing.x + style.FramePadding.x * 2.0f,
    std::max(textSize.y, iconSize) + style.FramePadding.y * 2.0f);
  return ImVec2(
    requestedSize.x > 0.0f ? std::max(requestedSize.x, minSize.x) : minSize.x,
    requestedSize.y > 0.0f ? std::max(requestedSize.y, minSize.y) : minSize.y);
}

bool drawIconTextButton(const TcpViewerIcons& icons, TcpViewerIconKind kind, const char* label,
                        const char* id, ImVec2 requestedSize = ImVec2(0.0f, 0.0f)) {
  const ImVec2 buttonSize = iconTextButtonSize(label, requestedSize);
  const TcpViewerIcon* icon = icons.get(kind);
  ImGui::PushID(id ? id : label);
  if (!icon) {
    const bool pressed = ImGui::Button(label, buttonSize);
    ImGui::PopID();
    return pressed;
  }

  const bool pressed = ImGui::Button("##icon_text_button", buttonSize);
  const bool hovered = ImGui::IsItemHovered();
  const bool active = ImGui::IsItemActive();
  const ImVec2 itemMin = ImGui::GetItemRectMin();
  const ImVec2 itemMax = ImGui::GetItemRectMax();
  const ImVec2 textSize = ImGui::CalcTextSize(label);
  const ImGuiStyle& style = ImGui::GetStyle();
  const float iconSize = std::round(ImGui::GetFontSize() * 0.95f);
  const float contentWidth = iconSize + style.ItemInnerSpacing.x + textSize.x;
  const float centerY = itemMin.y + (itemMax.y - itemMin.y) * 0.5f;
  const float startX = itemMin.x + std::max(style.FramePadding.x,
    ((itemMax.x - itemMin.x) - contentWidth) * 0.5f);
  const ImVec2 iconMin(startX, centerY - iconSize * 0.5f);
  const ImVec2 iconMax(iconMin.x + iconSize, iconMin.y + iconSize);
  const float chipPad = std::max(1.0f, std::round(ImGui::GetFontSize() * 0.10f));
  ImDrawList* drawList = ImGui::GetWindowDrawList();
  const ImTextureID textureId = (ImTextureID)(intptr_t)icon->texture;
  const ImVec4 iconTint = tcpViewerIconTint(kind, hovered, active);
  const ImVec4 chipFill(iconTint.x, iconTint.y, iconTint.z, active ? 0.26f : (hovered ? 0.22f : 0.16f));
  const ImVec4 chipBorder(iconTint.x, iconTint.y, iconTint.z, active ? 0.72f : (hovered ? 0.58f : 0.42f));
  drawList->PushClipRect(itemMin, itemMax, true);
  drawList->AddRectFilled(ImVec2(iconMin.x - chipPad, iconMin.y - chipPad),
    ImVec2(iconMax.x + chipPad, iconMax.y + chipPad),
    ImGui::GetColorU32(chipFill), 3.0f);
  drawList->AddRect(ImVec2(iconMin.x - chipPad, iconMin.y - chipPad),
    ImVec2(iconMax.x + chipPad, iconMax.y + chipPad),
    ImGui::GetColorU32(chipBorder), 3.0f);
  drawList->AddImage(textureId, ImVec2(iconMin.x + 1.0f, iconMin.y + 1.0f),
    ImVec2(iconMax.x + 1.0f, iconMax.y + 1.0f), ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f),
    ImGui::GetColorU32(ImVec4(0.0f, 0.0f, 0.0f, 0.34f)));
  drawList->AddImage(textureId, iconMin, iconMax, ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f),
    ImGui::GetColorU32(iconTint));
  drawList->AddText(ImVec2(iconMax.x + style.ItemInnerSpacing.x, centerY - textSize.y * 0.5f),
    ImGui::GetColorU32(ImGuiCol_Text), label);
  drawList->PopClipRect();
  ImGui::PopID();
  return pressed;
}

// Icon-only variant of drawIconTextButton. The button is a square (button height) with the
// same coloured chip + icon draw as the text variant, no label. Pass a tooltip so users
// can still discover the action on hover.
bool drawIconOnlyButton(const TcpViewerIcons& icons, TcpViewerIconKind kind,
                        const char* tooltip, const char* id) {
  const ImGuiStyle& style = ImGui::GetStyle();
  const float buttonExtent =
      ImGui::GetFontSize() + style.FramePadding.y * 2.0f;
  const ImVec2 buttonSize(buttonExtent, buttonExtent);
  const TcpViewerIcon* icon = icons.get(kind);
  ImGui::PushID(id ? id : tooltip);
  if (!icon) {
    const bool pressed = ImGui::Button(tooltip ? tooltip : "?", buttonSize);
    ImGui::PopID();
    return pressed;
  }

  const bool pressed = ImGui::Button("##icon_only_button", buttonSize);
  const bool hovered = ImGui::IsItemHovered();
  const bool active = ImGui::IsItemActive();
  if (hovered && tooltip && tooltip[0] != '\0') {
    ImGui::SetTooltip("%s", tooltip);
  }
  const ImVec2 itemMin = ImGui::GetItemRectMin();
  const ImVec2 itemMax = ImGui::GetItemRectMax();
  const float iconSize = std::round(ImGui::GetFontSize() * 0.95f);
  const ImVec2 centre((itemMin.x + itemMax.x) * 0.5f, (itemMin.y + itemMax.y) * 0.5f);
  const ImVec2 iconMin(centre.x - iconSize * 0.5f, centre.y - iconSize * 0.5f);
  const ImVec2 iconMax(iconMin.x + iconSize, iconMin.y + iconSize);
  const float chipPad = std::max(1.0f, std::round(ImGui::GetFontSize() * 0.10f));
  ImDrawList* drawList = ImGui::GetWindowDrawList();
  const ImTextureID textureId = (ImTextureID)(intptr_t)icon->texture;
  const ImVec4 iconTint = tcpViewerIconTint(kind, hovered, active);
  const ImVec4 chipFill(iconTint.x, iconTint.y, iconTint.z,
                        active ? 0.26f : (hovered ? 0.22f : 0.16f));
  const ImVec4 chipBorder(iconTint.x, iconTint.y, iconTint.z,
                          active ? 0.72f : (hovered ? 0.58f : 0.42f));
  drawList->PushClipRect(itemMin, itemMax, true);
  drawList->AddRectFilled(ImVec2(iconMin.x - chipPad, iconMin.y - chipPad),
    ImVec2(iconMax.x + chipPad, iconMax.y + chipPad),
    ImGui::GetColorU32(chipFill), 3.0f);
  drawList->AddRect(ImVec2(iconMin.x - chipPad, iconMin.y - chipPad),
    ImVec2(iconMax.x + chipPad, iconMax.y + chipPad),
    ImGui::GetColorU32(chipBorder), 3.0f);
  drawList->AddImage(textureId, ImVec2(iconMin.x + 1.0f, iconMin.y + 1.0f),
    ImVec2(iconMax.x + 1.0f, iconMax.y + 1.0f), ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f),
    ImGui::GetColorU32(ImVec4(0.0f, 0.0f, 0.0f, 0.34f)));
  drawList->AddImage(textureId, iconMin, iconMax, ImVec2(0.0f, 0.0f), ImVec2(1.0f, 1.0f),
    ImGui::GetColorU32(iconTint));
  drawList->PopClipRect();
  ImGui::PopID();
  return pressed;
}

void drawCollapsedLeftPanelLogo(const TcpViewerImageTexture& logo) {
  const bool hasLogo = logo.valid();
  const float fontSize = ImGui::GetFontSize();
  const float outerSize = std::round(fontSize * 3.05f);
  const ImVec2 handleSize = hasLogo
    ? ImVec2(outerSize, outerSize)
    : ImVec2(std::max(8.0f, std::round(fontSize * 0.48f)),
        std::max(28.0f, std::round(fontSize * 1.65f)));
  ImGui::InvisibleButton("##CollapsedLeftPanelHover", handleSize);

  const bool hovered = ImGui::IsItemHovered();
  const ImVec2 itemMin = ImGui::GetItemRectMin();
  const ImVec2 itemMax = ImGui::GetItemRectMax();
  ImDrawList* drawList = ImGui::GetWindowDrawList();

  if (!hasLogo) {
    const float barWidth = std::max(3.0f, std::round(fontSize * 0.16f));
    const float verticalPad = std::max(3.0f, std::round(fontSize * 0.18f));
    const float centerX = (itemMin.x + itemMax.x) * 0.5f;
    const ImVec2 barMin(centerX - barWidth * 0.5f, itemMin.y + verticalPad);
    const ImVec2 barMax(centerX + barWidth * 0.5f, itemMax.y - verticalPad);
    drawList->AddRectFilled(barMin, barMax,
      ImGui::GetColorU32(ImVec4(0.35f, 0.80f, 1.0f, hovered ? 0.70f : 0.48f)),
      barWidth * 0.5f);
    return;
  }

  const float uvWidth = std::max(1.0f, (logo.uvMax.x - logo.uvMin.x) * static_cast<float>(logo.width));
  const float uvHeight = std::max(1.0f, (logo.uvMax.y - logo.uvMin.y) * static_cast<float>(logo.height));
  const float aspect = uvWidth / uvHeight;
  const float maxImageW = outerSize;
  const float maxImageH = outerSize;
  float imageW = maxImageW;
  float imageH = maxImageH;
  if (aspect >= 1.0f) {
    imageH = imageW / aspect;
  } else {
    imageW = imageH * aspect;
  }

  const ImVec2 center((itemMin.x + itemMax.x) * 0.5f, (itemMin.y + itemMax.y) * 0.5f);
  const ImVec2 imageMin(center.x - imageW * 0.5f, center.y - imageH * 0.5f);
  const ImVec2 imageMax(center.x + imageW * 0.5f, center.y + imageH * 0.5f);
  const ImTextureID textureId = (ImTextureID)(intptr_t)logo.texture;
  drawList->AddImage(textureId, imageMin, imageMax, logo.uvMin, logo.uvMax,
    ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, hovered ? 1.0f : 0.94f)));
}

void renderViewer(raisin::RayraiWindow& viewer, SDL_Window* window,
                  bool allowViewportInput = true, bool allowClickSelection = true,
                  ViewerViewportState* viewportState = nullptr) {
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
  if (viewportState) {
    viewportState->origin = windowPos;
    viewportState->size = ImVec2(static_cast<float>(fbW), static_cast<float>(fbH));
    viewportState->hovered = isHovered;
    viewportState->cursorX = cursorX;
    viewportState->cursorY = cursorY;
  }
  viewer.update(fbW, fbH, allowViewportInput ? isHovered : false, cursorX, cursorY,
    allowClickSelection);

  ImGui::End();
  ImGui::PopStyleVar(2);
}

} // namespace

#ifndef RAYRAI_TCP_VIEWER_NO_MAIN
int main(int argc, char* argv[]) {
  std::setlocale(LC_ALL, "C");
  std::locale::global(std::locale::classic());

  ProgramOptions options;
  if (!parseProgramOptions(argc, argv, options)) {
    printUsage(argc > 0 ? argv[0] : "rayrai_tcp_viewer");
    return 2;
  }
  if (options.printHelp) {
    printUsage(argc > 0 ? argv[0] : "rayrai_tcp_viewer");
    return 0;
  }

  std::error_code pathEc;
  const std::filesystem::path argvPath = argc > 0 && argv[0] ? std::filesystem::path(argv[0])
                                                             : std::filesystem::current_path(pathEc);
  std::filesystem::path binaryPath = std::filesystem::absolute(argvPath, pathEc);
  if (pathEc) {
    pathEc.clear();
    binaryPath = std::filesystem::current_path(pathEc);
  }
  const std::filesystem::path binaryDir = pathEc ? std::filesystem::path{} : binaryPath.parent_path();
  const std::string robotoFontPath = findRobotoFontPath(binaryDir);
  const float fontRasterizerDensity = readEnvFloatClamped(
    "RAYRAI_TCP_VIEWER_FONT_DENSITY", kDefaultFontRasterizerDensity, 1.0f, 3.0f);

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
    SDL_WINDOWPOS_CENTERED, options.windowWidth, options.windowHeight,
    SDL_WindowFlags(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI |
                    (options.fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0)));

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

  TcpViewerIcons uiIcons;
  const std::filesystem::path tcpViewerIconDir = findTcpViewerIconDir(binaryDir);
  if (!uiIcons.load(tcpViewerIconDir)) {
    std::cerr << "WARN: TCP viewer icons were not found; falling back to text-only buttons\n";
  }
  TcpViewerImageTexture raisimLogo;
  const std::filesystem::path raisimLogoPath = findRaisimLogoPath(binaryDir);
  if (!loadTcpViewerImageTexture(raisimLogoPath, raisimLogo)) {
    std::cerr << "WARN: Raisim logo was not found; using collapsed-panel fallback handle\n";
  }

  auto world = std::make_shared<raisim::World>();
  // Construct the renderer with the shader binary cache turned on and verbose so
  // we can see whether the slow "pbrMeshHigh" compile is a cache miss or a
  // genuine driver recompile. ThreadingMode default, 1 compile thread, cache
  // enabled, default cache dir ($HOME/.raisim/rayrai), log hits/misses.
  auto viewer = std::make_shared<raisin::RayraiWindow>(
      world, options.windowWidth, options.windowHeight,
      raisin::RayraiWindow::ThreadingMode::SingleThread,
      /*shaderCompileThreadCount=*/1u,
      /*shaderBinaryCacheEnabled=*/true,
      /*shaderBinaryCacheDirectory=*/std::string{},
      /*logShaderBinaryCache=*/true);
  // TCP scene updates need mesh assets available on the first render/export pass.
  viewer->setAsyncMeshLoadingEnabled(false);
  ViewerSettings settings;
  copyRenderDefaultsToSettings(settings, settings.renderQuality);
  settings.cameraSpeed = viewer->getCamera().movementSpeed;
  settings.cameraFovDeg = viewer->getCamera().zoom;
  loadViewerSettings(settings);
  const GpuQualityRecommendation gpuQuality = recommendRenderQualityForCurrentGpu();
  if (applyAutomaticRenderQualityIfUnset(settings, gpuQuality.quality)) {
    std::cerr << "INFO: Auto render quality selected " << qualityName(settings.renderQuality)
              << " for GPU '" << gpuQuality.gpu.renderer << "'\n";
  }

  viewer->setBackgroundColorRgb255({20, 20, 30, 255});
  // viewer->setGroundPatternResourcePath(
  //   raisin::getResourceDirectory("raisin_gui") + "material/checkerboard/checker_gray-01.png");
  viewer->setShowCollisionBodies(false);
  auto& camera = viewer->getCamera();
  camera.nearPlane = 0.01f;
  camera.farPlane = 1000.0f;
  camera.zNear = 0.01f;
  camera.zFar = 1000.0f;
  const char* cameraEnv = std::getenv("RAYRAI_TCP_VIEWER_CAMERA_LOOKAT");
  const bool forceCameraEnv = options.forceCameraLookAt ||
                              std::getenv("RAYRAI_TCP_VIEWER_FORCE_CAMERA_LOOKAT") != nullptr;
  glm::vec3 forcedCameraPos{0.0f};
  glm::vec3 forcedCameraTarget{0.0f};
  bool hasForcedCamera = false;
  if (options.hasCameraLookAt) {
    forcedCameraPos = options.cameraPos;
    forcedCameraTarget = options.cameraTarget;
    hasForcedCamera = true;
  } else {
    hasForcedCamera = parseCameraLookAtEnv(cameraEnv, forcedCameraPos, forcedCameraTarget);
  }
  glm::vec3 forcedTargetOffset{0.0f};
  bool hasForcedTargetOffset = false;
  if (options.hasTargetOffset) {
    forcedTargetOffset = options.targetOffset;
    hasForcedTargetOffset = true;
  } else {
    hasForcedTargetOffset = parseVec3Env(
      std::getenv("RAYRAI_TCP_VIEWER_CAMERA_OFFSET_FROM_TARGET"), forcedTargetOffset);
  }
  if (hasForcedCamera) {
    applyCameraLookAt(camera, forcedCameraPos, forcedCameraTarget);
  } else {
    const glm::vec3 horizonCameraPos(6.0f, -7.0f, 1.6f);
    const glm::vec3 horizonCameraTarget(0.0f, 0.0f, 1.6f);
    applyCameraLookAt(camera, horizonCameraPos, horizonCameraTarget);
  }
  const glm::vec3 defaultCameraPos = camera.getPosition();
  const glm::vec3 defaultCameraTarget = camera.target;

  auto& light = viewer->getLight();
  light.type = raisin::LightType::DIRECTIONAL;
  light.ambient = glm::vec3(0.42f, 0.42f, 0.42f);
  light.diffuse = glm::vec3(1.0f, 1.0f, 1.0f);
  light.specular = glm::vec3(0.22f, 0.22f, 0.22f);
  light.setShadowParams(0.0008f, 0.6f, 1.25f);
  light.setShadowsEnabled(true);
  applyViewerSettings(*viewer, settings);

  // --warm-at-startup: pay the ~13 s of non-shader lazy init up front so that any
  // later drag-drop completes in <50 ms. Off by default — empty-viewer launches
  // (which never need PBR mesh rendering) stay fast.
  if (options.warmAtStartup) {
    static const char* kWarmupUrdf = R"(<?xml version="1.0"?>
<robot name="rayrai_warmup">
  <link name="base">
    <visual><geometry><box size="0.001 0.001 0.001"/></geometry></visual>
    <collision><geometry><box size="0.001 0.001 0.001"/></geometry></collision>
    <inertial><mass value="0.001"/>
      <inertia ixx="1e-9" iyy="1e-9" izz="1e-9" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
</robot>)";
    std::cerr << "[rayrai] --warm-at-startup: warming content-frame init (~13s)\n";
    auto* warmupGround = world->addGround();
    raisim::ArticulatedSystem* warmupAs = nullptr;
    try {
      warmupAs = world->addArticulatedSystem(kWarmupUrdf);
      if (warmupAs) warmupAs->setBasePos(raisim::Vec<3>{0.0, 0.0, -1000.0});
    } catch (...) { warmupAs = nullptr; }
    const auto t0 = std::chrono::steady_clock::now();
    viewer->update(options.windowWidth, options.windowHeight, false, 0, 0, true);
    const auto warmMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - t0).count();
    if (warmupAs) world->removeObject(warmupAs);
    if (warmupGround) world->removeObject(warmupGround);
    viewer->updateObjectLists();
    std::cerr << "[rayrai] content-frame warmup: " << warmMs << " ms\n";
  }

  // Renderer warmup is now scoped to *shader* pre-compile only, which is cheap
  // (~40 ms with the binary cache hit) and fits comfortably in startup. The other
  // content-frame init (IBL convolution / texture pool / FBOs / etc.) is NOT
  // covered by the shader binary cache — it's per-launch state that takes ~13 s.
  // We don't want to pay that at every startup; instead it fires inside the first
  // load (drag-drop OR --inspect FILE) as part of that explicit user action, with
  // a clearly-visible status message so the wait is attributed to the load.
  static const std::vector<std::string> kViewerWarmupShaders = {
    "pbrMeshHigh",
  };
  size_t shaderWarmupIdx = 0;
  bool shaderWarmupActive = options.preWarmShaders;
  const auto shaderWarmupStart = std::chrono::steady_clock::now();
  if (shaderWarmupActive) {
    std::cerr << "[rayrai] background shader warmup started ("
              << kViewerWarmupShaders.size() << " targeted shader"
              << (kViewerWarmupShaders.size() == 1 ? "" : "s") << ")\n";
  }

  TcpClient client;
  RemoteScene scene(viewer);
  scene.setShowCollisionBodies(false);
  scene.setForceTransparent(false);

  char host[256] = "127.0.0.1";
  std::snprintf(host, sizeof(host), "%s", options.host.c_str());
  int port = options.port;
  char portBuf[16];
  std::snprintf(portBuf, sizeof(portBuf), "%d", port);
  std::vector<ConnectionEntry> recentConnections = settings.recentConnections;
  if (!options.endpointListPath.empty()) {
    loadEndpointList(options.endpointListPath, recentConnections);
  }
  std::vector<std::string> resourceDirs = settings.resourceDirs;
  for (const auto& dir : options.resourceDirs) {
    recordResourceDir(resourceDirs, dir);
  }
  for (const auto& dir : resourceDirs) {
    scene.addSearchPath(dir);
  }
  char searchPathBuf[256] = "";
  char screenshotDirBuf[512];
  std::snprintf(screenshotDirBuf, sizeof(screenshotDirBuf), "%s", options.screenshotDir.string().c_str());
  char sessionPathBuf[512];
  const std::filesystem::path defaultSessionPath =
    options.recordSessionPath.empty()
      ? timestampedDataPath(options.screenshotDir, "rayrai_tcp_viewer_session", ".rrtcs")
      : options.recordSessionPath;
  std::snprintf(sessionPathBuf, sizeof(sessionPathBuf), "%s", defaultSessionPath.string().c_str());
  std::string captureStatus;
  std::string sessionStatus;
  bool screenshotAfterFirstScene = !options.screenshotPath.empty();
  bool screenshotRequested = false;
  std::filesystem::path pendingScreenshotPath = options.screenshotPath;
  bool recordPngSequence = false;
  int recordEveryNFrames = 1;
  int recordFrameIndex = 0;
  SessionRecorder sessionRecorder;
  std::vector<RecordedFrame> replayFrames;
  bool replayMode = false;
  bool replayPaused = false;
  bool replayStep = false;
  size_t replayIndex = 0;
  float replaySpeed = options.replaySpeed;
  auto replayStart = std::chrono::steady_clock::now();
  uint64_t replayBaseMicros = 0;
  std::ofstream trajectoryCsv;
  std::deque<PacketSample> packetSamples;
  std::vector<AssetDiagnostic> assetDiagnostics;
  bool assetDiagnosticsDirty = true;
  bool exportScenePending = !options.exportScenePath.empty();
  int frameSerial = 0;
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
  const bool defaultAutoConnect = options.autoConnectSet ? options.autoConnect :
                                  readEnvBool("RAYRAI_TCP_VIEWER_AUTO_CONNECT", true);
  const bool envMinimizePanels = options.minimizePanelsSet ? options.minimizePanels :
                                 std::getenv("RAYRAI_TCP_VIEWER_MINIMIZE_PANELS") != nullptr;
  const bool envAutoFrame = options.autoFrameSet ? options.autoFrame :
                            std::getenv("RAYRAI_TCP_VIEWER_AUTO_FRAME") != nullptr;
  bool autoFrameApplied = false;
  bool overlayMinimized = envMinimizePanels;
  bool overlayCollapsedHoveredLastFrame = false;
  auto overlayLastInteractionTime = std::chrono::steady_clock::now();
  bool detailMinimized = envMinimizePanels;
  std::shared_ptr<raisin::CoordinateFrame> worldFrame;
  bool awaitingResponse = false;
  bool awaitingSensorAck = false;
  bool autoConnect = defaultAutoConnect;

  // Sim control: queue of requests flushed onto the next update frame.
  std::vector<raisin::tcp_viewer::SimControlRequest> pendingControlRequests;
  bool simPaused = false;
  glm::vec3 controlForce(0.0f, 0.0f, 20.0f);
  glm::vec3 controlTorque(0.0f, 0.0f, 1.0f);
  glm::vec3 controlPointOffset(0.0f);
  int controlBodyIdx = 0;
  bool controlBodyFollowsSelection = true;
  uint32_t controlSelectionTag = 0;
  int controlSelectionIndex = -1;
  glm::vec3 controlPosePosition(0.0f);
  glm::vec4 controlPoseQuat(0.0f, 0.0f, 0.0f, 1.0f);
  uint32_t controlPoseTag = 0;
  bool controlPoseInitialized = false;
  std::vector<float> controlGc;
  uint32_t controlGcTag = 0;
  bool controlGcDirty = false;
  bool mouseForceEnabled = true;
  float mouseForceScale = 1.0f;
  MouseForceGesture mouseForce;
  RulerToolState ruler;
  ViewerViewportState viewportState;

  // These were declared further down before the inspector was added; pull them up so the
  // load/close lambdas below can capture them.
  bool requestFrameScene = false;
  bool requestFrameSelected = false;

  // Local AS inspector mode (drag-drop URDF/MJCF while disconnected).
  InspectorState inspector;
  auto closeInspector = [&]() {
    if (!inspector.active) return;
    // Drop any selection that points at our AS first, so the renderer doesn't keep
    // a dangling pointer when it next iterates targetVisual_ against the world list.
    viewer->setTargetVisual(nullptr);
    // Remove the AS first so any contact/constraint references on side objects clear.
    if (inspector.as) {
      world->removeObject(inspector.as);
    }
    // MJCF can also add ground planes, lights, mocap bodies, etc. Tear them down too.
    for (raisim::Object* ob : inspector.sideObjects) {
      if (ob) world->removeObject(ob);
    }
    // Critical: tell the renderer its objectList_ cache is now stale. updateWeather()
    // and other per-frame helpers walk that cache directly and would dereference the
    // freed raisim::Object pointers we just deleted. Without this, the next frame
    // segfaults inside RaisimObject::configureGroundMaterial (or any other place that
    // touches the wrapper's stored raisim::Object*).
    viewer->updateObjectLists();
    inspector = InspectorState{};
    lastStatus = "disconnected";
  };
  auto loadAsInspector = [&](const std::string& path) -> bool {
    namespace fs = std::filesystem;
    inspector.lastError.clear();
    if (path.empty() || !fs::exists(path)) {
      inspector.lastError = "file not found: " + path;
      return false;
    }

    // Close any previously-loaded inspector model before replacing it.
    closeInspector();

    const auto tStart = std::chrono::steady_clock::now();
    const bool isMjcf = looksLikeMjcf(path);

    // Snapshot the object list so we can attribute any new objects to this load.
    std::vector<raisim::Object*> before(world->getObjList().begin(), world->getObjList().end());

    raisim::ArticulatedSystem* as = nullptr;
    try {
      if (isMjcf) {
        // loadMjcfFile populates the world; we pick the first ArticulatedSystem it added.
        world->loadMjcfFile(path);
      } else {
        as = world->addArticulatedSystem(path);
      }
    } catch (const std::exception& e) {
      inspector.lastError = std::string("load failed: ") + e.what();
      return false;
    } catch (...) {
      inspector.lastError = "load failed (unknown exception)";
      return false;
    }

    // Diff before/after to collect the new objects.
    std::vector<raisim::Object*> newObjects;
    {
      std::unordered_set<raisim::Object*> beforeSet(before.begin(), before.end());
      for (raisim::Object* ob : world->getObjList()) {
        if (beforeSet.find(ob) == beforeSet.end()) newObjects.push_back(ob);
      }
    }
    if (isMjcf) {
      for (raisim::Object* ob : newObjects) {
        if (!as && ob->getObjectType() == raisim::ObjectType::ARTICULATED_SYSTEM) {
          as = dynamic_cast<raisim::ArticulatedSystem*>(ob);
        }
      }
    }
    if (!as) {
      // No AS produced — tear down whatever was added so the world stays clean.
      for (raisim::Object* ob : newObjects) {
        if (ob) world->removeObject(ob);
      }
      inspector.lastError = isMjcf ? "MJCF contained no articulated system"
                                   : "URDF parse failed (see console)";
      return false;
    }

    inspector = InspectorState{};
    inspector.active = true;
    inspector.sourceFile = path;
    inspector.as = as;
    for (raisim::Object* ob : newObjects) {
      if (ob != as) inspector.sideObjects.push_back(ob);
    }
    inspector.as->setName("inspector_" + fs::path(path).stem().string());

    const int gcDim = inspector.as->getGeneralizedCoordinateDim();
    inspector.gc.assign(gcDim, 0.0);
    {
      const raisim::VecDyn& q = inspector.as->getGeneralizedCoordinate();
      const int copyN = std::min<int>(gcDim, static_cast<int>(q.size()));
      for (int i = 0; i < copyN; ++i) inspector.gc[i] = q[i];
    }

    // Walk all joints (including FIXED ones at the root) so we get the correct GC offset
    // for each movable joint. movableJointNames is in joint order but skips FIXED joints;
    // we re-pair them with their owning joint index using the same trick the server uses
    // when the first joint is a FIXED ground attachment.
    const auto& movableNames = inspector.as->getMovableJointNames();
    const auto& jointLimits = inspector.as->getJointLimits();
    const size_t numJoints = inspector.as->getNumberOfJoints();
    int gcCursor = 0;
    size_t movableNameIdx = 0;
    for (size_t j = 0; j < numJoints; ++j) {
      const raisim::Joint::Type type = inspector.as->getJointType(j);
      int dofThisJoint = 0;
      switch (type) {
        case raisim::Joint::Type::FIXED: dofThisJoint = 0; break;
        case raisim::Joint::Type::REVOLUTE:
        case raisim::Joint::Type::PRISMATIC: dofThisJoint = 1; break;
        case raisim::Joint::Type::SPHERICAL: dofThisJoint = 4; break;
        case raisim::Joint::Type::FLOATING: dofThisJoint = 7; break;
      }
      if (type == raisim::Joint::Type::FIXED) {
        continue; // FIXED joints don't appear in movableJointNames and consume no GC.
      }
      InspectorJoint info;
      info.type = type;
      info.gcOffset = gcCursor;
      info.gcDim = dofThisJoint;
      info.name = (movableNameIdx < movableNames.size())
                      ? movableNames[movableNameIdx++]
                      : ("joint_" + std::to_string(j));
      if (j < jointLimits.size() && info.gcDim == 1) {
        info.minLimit = jointLimits[j][0];
        info.maxLimit = jointLimits[j][1];
        info.hasLimits = (info.maxLimit > info.minLimit);
      }
      inspector.joints.push_back(std::move(info));
      gcCursor += dofThisJoint;
    }

    const auto tEnd = std::chrono::steady_clock::now();
    const auto loadMs = std::chrono::duration_cast<std::chrono::milliseconds>(tEnd - tStart).count();
    awaitingResponse = false;
    awaitingSensorAck = false;
    lastStatus = "inspector: " + fs::path(path).filename().string() +
                 " (" + std::to_string(loadMs) + " ms)";
    // Frame the camera on the loaded AS directly. We can't rely on requestFrameScene
    // here because that path goes through RemoteScene::computeSceneBounds, which is
    // empty in inspector mode (the AS lives in the local raisim::World, not RemoteScene).
    {
      raisim::Vec<3> centerVec;
      try {
        centerVec = inspector.as->getCOM();
      } catch (...) {
        centerVec = {0.0, 0.0, 0.5};
      }
      const glm::vec3 center(static_cast<float>(centerVec[0]),
                             static_cast<float>(centerVec[1]),
                             static_cast<float>(centerVec[2]));
      const glm::vec3 halfExtent(1.0f, 1.0f, 0.8f);
      frameBounds(*viewer, center - halfExtent, center + halfExtent);
    }
    std::cerr << "[inspector] loaded " << path
              << " in " << loadMs << " ms"
              << " (gcDim=" << inspector.gc.size()
              << " joints=" << inspector.joints.size() << ")\n";
    return true;
  };
  float uiScale = settings.uiScale;
  float defaultUiScale = 1.0f;
  bool uiScaleInitialized = false;
  bool uiScaleUserSet = settings.uiScaleUserSet;
  float appliedUiScale = 0.0f;
  bool baseStyleCaptured = false;
  ImGuiStyle baseStyle;
  ImVec2 lastDisplaySize(0.0f, 0.0f);
  bool settingsDirty = !options.resourceDirs.empty();
  bool settingsApplied = false;
  bool settingsSavePending = false;
  auto lastSettingsDirtyTime = std::chrono::steady_clock::now();
  // requestFrameScene / requestFrameSelected are declared earlier so the inspector
  // load lambda can frame the scene; only requestResetCamera lives here.
  bool requestResetCamera = false;
  bool groupObjectsByType = false;
  bool hideCollisionObjects = false;
  int objectSortMode = 0;
  char objectFilterBuf[160] = "";
  std::array<CameraBookmark, 4> cameraBookmarks;
  std::unordered_map<uint64_t, MotionEstimate> motionEstimates;
  DiscoveryBeaconReceiver beaconReceiver;
  std::string discoveryStatus;
  beaconReceiver.start(discoveryStatus);
  std::vector<ServerEntry> discoveredServers = beaconReceiver.servers();
  ViewerStats stats;
  const auto steadyStart = std::chrono::steady_clock::now();
  auto nextAutoConnectAttempt = std::chrono::steady_clock::now();
  auto clearSceneState = [&]() {
    scene.clear();
    motionEstimates.clear();
    assetDiagnostics.clear();
    assetDiagnosticsDirty = true;
    stats.pendingSensorRequests = 0;
    stats.unresolvedAssets = 0;
  };
  auto connectToEndpoint = [&](const ConnectionEntry& endpoint, bool verbose,
                               const char* connectingStatus, const char* failureStatus) -> bool {
    if (inspector.active) {
      lastStatus = "close inspector to connect";
      return false;
    }
    lastStatus = connectingStatus;
    if (client.connectTo(endpoint.host, endpoint.port, verbose, kConnectTimeoutMs)) {
      lastStatus = "waiting for scene";
      awaitingResponse = false;
      awaitingSensorAck = false;
      std::snprintf(host, sizeof(host), "%s", endpoint.host.c_str());
      port = endpoint.port;
      std::snprintf(portBuf, sizeof(portBuf), "%d", port);
      recordConnection(recentConnections, endpoint.host, endpoint.port);
      settingsDirty = true;
      stats.reconnects++;
      return true;
    }
    lastStatus = failureStatus;
    return false;
  };

  auto applyScenePayload = [&](const std::vector<char>& payload, bool fromReplay,
                              std::chrono::steady_clock::time_point sampleNow,
                              std::vector<PendingSensorUpdate>& pending) -> bool {
    BufferReader reader(payload);
    scene.setVerbose(verboseParsing);
    const bool parsedOk = scene.applyResponse(reader, pending);
    const bool disconnectRequested = scene.consumeDisconnectRequested();
    bool ok = parsedOk && !disconnectRequested;
    if (disconnectRequested) {
      lastStatus = fromReplay ? "replay protocol disconnect" : "protocol error (disconnect)";
    } else if (!parsedOk) {
      lastStatus = fromReplay ? "replay parse error" : "parse error (dropped update)";
      stats.parseErrors++;
    } else {
      lastStatus = fromReplay ? "replay" : "connected";
      if (screenshotAfterFirstScene) {
        screenshotRequested = true;
        screenshotAfterFirstScene = false;
      }
      if (envAutoFrame && !autoFrameApplied && frameScene(*viewer, scene)) {
        autoFrameApplied = true;
      }
    }

    if (ok) {
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
    }

    assetDiagnosticsDirty = true;
    stats.pendingSensorRequests = static_cast<int>(pending.size());
    stats.unresolvedAssets = unresolvedAssetCount(scene);
    const double sampleTime = std::chrono::duration<double>(sampleNow - steadyStart).count();
    PacketSample sample;
    sample.timeSeconds = sampleTime;
    sample.bytes = static_cast<int>(payload.size());
    sample.parsed = ok;
    sample.replay = fromReplay;
    sample.pendingSensors = static_cast<int>(pending.size());
    sample.objects = scene.selectableObjectCount();
    sample.visuals = scene.visualCount();
    sample.instanced = scene.instancedCount();
    sample.pointClouds = scene.pointCloudCount();
    sample.unresolvedAssets = stats.unresolvedAssets;
    pushPacketSample(packetSamples, sample);
    if (ok) {
      const double worldTime = scene.hasServerWorldTime() ? scene.getServerWorldTime() : sampleTime;
      if (trajectoryCsv) {
        writeTrajectoryRows(trajectoryCsv, scene, worldTime);
      }
      for (const auto& item : scene.getSelectableObjects()) {
        if (isContactItem(item) || !item.visual) {
          continue;
        }
        uint32_t motionTag = 0;
        int motionIndex = 0;
        const VisualEntry* motionEntry = nullptr;
        if (scene.getVisualInfo(item.visual.get(), motionTag, motionIndex, motionEntry) && motionEntry) {
          updateMotionEstimate(motionEstimates[visualMotionKey(motionTag, motionIndex)], *motionEntry, worldTime);
        }
      }
    }
    return ok;
  };
  if (!options.replaySessionPath.empty()) {
    replayMode = loadSessionFile(options.replaySessionPath, replayFrames, sessionStatus);
    replayPaused = false;
    replayStart = std::chrono::steady_clock::now();
    replayBaseMicros = replayFrames.empty() ? 0 : replayFrames.front().timeMicros;
    autoConnect = false;
    lastStatus = replayMode ? "replay" : "replay load failed";
  }
  if (!options.recordSessionPath.empty()) {
    sessionRecorder.open(options.recordSessionPath, sessionStatus);
  }
  if (!options.trajectoryCsvPath.empty()) {
    std::error_code ec;
    if (!options.trajectoryCsvPath.parent_path().empty()) {
      std::filesystem::create_directories(options.trajectoryCsvPath.parent_path(), ec);
    }
    trajectoryCsv.open(options.trajectoryCsvPath);
    if (trajectoryCsv) {
      trajectoryCsv << "time,tag,index,name,type,x,y,z,qw,qx,qy,qz\n";
    } else {
      std::cerr << "WARN: failed to open trajectory CSV " << options.trajectoryCsvPath << "\n";
    }
  }
  light.direction = lightDirectionFromYawPitch(lightYawDeg, lightPitchDeg);

  // --inspect FILE was passed on the CLI: load the model as if drag-dropped. When
  // --inspect-after-frames N is also set, defer the load until the main loop has
  // ticked N frames (lets the headless harness measure drag-drop latency after
  // the background shader warmup has run).
  if (!options.inspectorPath.empty() && options.inspectAfterFrames < 0) {
    if (!loadAsInspector(options.inspectorPath.string())) {
      std::cerr << "ERROR: --inspect failed: " << inspector.lastError << "\n";
    }
  }

  while (!quit && !gSignalQuit.load(std::memory_order_relaxed)) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT)
        quit = true;
      if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE &&
          event.window.windowID == SDL_GetWindowID(window))
        quit = true;
      if (event.type == SDL_DROPFILE && event.drop.file) {
        // Only accept drops while disconnected — this is the local AS inspector mode.
        // Once a server connection is active, we don't want to confuse the streamed
        // scene by spawning local objects into the same world.
        const std::string droppedPath(event.drop.file);
        SDL_free(event.drop.file);
        std::cerr << "[inspector] drop received: " << droppedPath << "\n";
        if (client.isConnected()) {
          std::cerr << "[inspector] ignored — connected; disconnect first\n";
          lastStatus = "drop ignored while connected — disconnect first";
        } else {
          if (loadAsInspector(droppedPath)) {
            autoConnect = false; // don't fight the user by reconnecting underneath
          } else {
            std::cerr << "[inspector] load failed: " << inspector.lastError << "\n";
            lastStatus = inspector.lastError.empty()
                            ? "drop failed"
                            : ("drop failed: " + inspector.lastError);
          }
        }
      }
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
      fontConfig.OversampleH = 3;
      fontConfig.OversampleV = 2;
      fontConfig.PixelSnapH = false;
      fontConfig.RasterizerDensity = std::max(fontRasterizerDensity,
        std::clamp(std::max(io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y), 1.0f, 2.0f));
      fontConfig.RasterizerMultiply = 1.04f;
      io.Fonts->Clear();
      io.Fonts->TexDesiredWidth = 2048;
      io.Fonts->TexGlyphPadding = 2;
      ImFont* uiFont = nullptr;
      if (!robotoFontPath.empty()) {
        uiFont = io.Fonts->AddFontFromFileTTF(robotoFontPath.c_str(), fontSize, &fontConfig);
      }
      static bool fontSelectionLogged = false;
      if (!fontSelectionLogged) {
        if (uiFont) {
          std::cerr << "INFO: TCP viewer font " << robotoFontPath
                    << " size_px=" << fontSize
                    << " rasterizer_density=" << fontConfig.RasterizerDensity << "\n";
        } else {
          std::cerr << "WARN: TCP viewer Roboto font not found; using ImGui default font\n";
        }
        fontSelectionLogged = true;
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

    if (!io.WantTextInput) {
      if (ImGui::IsKeyPressed(ImGuiKey_F, false)) requestFrameScene = true;
      if (ImGui::IsKeyPressed(ImGuiKey_C, false)) requestFrameSelected = true;
      if (ImGui::IsKeyPressed(ImGuiKey_R, false)) requestResetCamera = true;
      if (ImGui::IsKeyPressed(ImGuiKey_F12, false)) screenshotRequested = true;
      if (ImGui::IsKeyPressed(ImGuiKey_F11, false)) {
        const Uint32 flags = SDL_GetWindowFlags(window);
        const bool isFullscreen = (flags & SDL_WINDOW_FULLSCREEN_DESKTOP) != 0;
        SDL_SetWindowFullscreen(window, isFullscreen ? 0 : SDL_WINDOW_FULLSCREEN_DESKTOP);
      }
    }

    constexpr float menuBarHeight = 0.0f;

    const auto now = std::chrono::steady_clock::now();
    const double wallElapsed = std::chrono::duration<double>(now - steadyStart).count();
    if (options.exitAfterSeconds > 0.0 && wallElapsed >= options.exitAfterSeconds) {
      quit = true;
    }
    if (options.waitForServerSeconds > 0.0 && !replayMode && !client.isConnected() &&
        wallElapsed >= options.waitForServerSeconds) {
      lastStatus = "wait-for-server timed out";
      quit = true;
    }
    if (beaconReceiver.poll()) {
      discoveredServers = beaconReceiver.servers();
    }
    if (replayMode && !replayFrames.empty()) {
      if (replayIndex < replayFrames.size() && (replayStep || !replayPaused)) {
        const uint64_t targetMicros = replayStep
          ? replayFrames[replayIndex].timeMicros
          : replayBaseMicros + static_cast<uint64_t>(
              std::chrono::duration<double, std::micro>(now - replayStart).count() * replaySpeed);
        while (replayIndex < replayFrames.size() && replayFrames[replayIndex].timeMicros <= targetMicros) {
          const auto& frame = replayFrames[replayIndex];
          stats.lastPayloadBytes = static_cast<int>(frame.payload.size());
          stats.bytes += frame.payload.size();
          stats.updates++;
          std::vector<PendingSensorUpdate> pending;
          applyScenePayload(frame.payload, true, now, pending);
          ++replayIndex;
          if (replayStep) break;
        }
        replayStep = false;
        if (replayIndex >= replayFrames.size()) {
          if (options.replayLoop) {
            replayIndex = 0;
            replayStart = now;
            replayBaseMicros = replayFrames.front().timeMicros;
            clearSceneState();
          } else {
            replayPaused = true;
            replayIndex = replayFrames.size();
          }
        }
      } else {
        replayStep = false;
      }
    }
    if (!replayMode && autoConnect && !inspector.active && !client.isConnected() &&
        now >= nextAutoConnectAttempt) {
      ConnectionEntry endpoint;
      if (!normalizeConnectionEndpoint(host, port, endpoint)) {
        lastStatus = "invalid endpoint";
      } else {
        connectToEndpoint(endpoint, false, "auto-connecting", "auto-connect failed");
      }
      nextAutoConnectAttempt = now + kAutoConnectInterval;
    }

    uint32_t requestedTag = 0;
    int requestedIndex = 0;
    const VisualEntry* requestedEntry = nullptr;
    scene.getVisualInfo(viewer->getTargetVisual(), requestedTag, requestedIndex, requestedEntry);
    if (requestedEntry && (requestedEntry->shape == raisim::Shape::Ground ||
                            requestedEntry->shape == raisim::Shape::HeightMap)) {
      viewer->setTargetVisual(nullptr);
      requestedTag = 0;
      requestedIndex = 0;
      requestedEntry = nullptr;
    }
    if (requestedEntry &&
        (controlSelectionTag != requestedTag || controlSelectionIndex != requestedIndex)) {
      controlSelectionTag = requestedTag;
      controlSelectionIndex = requestedIndex;
      controlBodyIdx = std::max(0, requestedEntry->localBodyIdx);
      controlBodyFollowsSelection = true;
      controlPosePosition = requestedEntry->lastPos;
      controlPoseQuat = requestedEntry->lastQuat;
      controlPoseTag = requestedTag;
      controlPoseInitialized = true;
      controlGc.clear();
      controlGcTag = 0;
      controlGcDirty = false;
    } else if (!requestedEntry) {
      controlSelectionTag = 0;
      controlSelectionIndex = -1;
      controlGcDirty = false;
    }
    scene.setSelectionTag(hasForcedTargetOffset ? 0 : requestedTag);
    const auto setRulerEndpoint = [&](int endpoint, const glm::vec3& point, std::string label) {
      label = trimAscii(label);
      if (label.empty()) {
        label = "scene point";
      }
      if (endpoint == 0) {
        ruler.hasA = true;
        ruler.a = point;
        ruler.aLabel = std::move(label);
        ruler.nextPoint = 1;
      } else {
        ruler.hasB = true;
        ruler.b = point;
        ruler.bLabel = std::move(label);
        ruler.nextPoint = 0;
      }
    };
    const auto appendRulerPoint = [&](const glm::vec3& point, const std::string& label) {
      if (!ruler.hasA || (ruler.hasA && ruler.hasB) || ruler.nextPoint == 0) {
        ruler.hasA = true;
        ruler.a = point;
        ruler.aLabel = trimAscii(label).empty() ? "scene point" : trimAscii(label);
        ruler.hasB = false;
        ruler.bLabel.clear();
        ruler.nextPoint = 1;
        lastStatus = "ruler point A set";
      } else {
        setRulerEndpoint(1, point, label);
        lastStatus = "ruler point B set";
      }
    };
    const auto rulerSelectionLabel = [&](uint32_t tag, int index, const VisualEntry* entry) {
      std::string label;
      if (entry) {
        label = entry->objectName.empty() ? scene.getObjectName(tag) : entry->objectName;
        if (label.empty()) {
          label = objectTypeLabel(entry->objectTypeRaw);
        }
      }
      if (label.empty()) {
        label = "tag " + std::to_string(tag) + ":" + std::to_string(index);
      }
      return label;
    };
    const auto setRulerEndpointFromSelection = [&](int endpoint, uint32_t tag, int index,
                                                   const VisualEntry* entry) {
      if (!entry) {
        return false;
      }
      setRulerEndpoint(endpoint, entry->lastPos, rulerSelectionLabel(tag, index, entry));
      lastStatus = endpoint == 0 ? "ruler point A set" : "ruler point B set";
      return true;
    };
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
          if (!sendUpdateRequest(client, updateRequestTag, pendingControlRequests)) {
            if (!client.lastIoWouldBlock()) {
              lastStatus = "connection lost";
              networkFailed = true;
            }
          } else {
            awaitingResponse = true;
            pendingControlRequests.clear();
            if (mouseForce.active) {
              mouseForce.pendingRequestIndex = std::numeric_limits<size_t>::max();
            }
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
            stats.lastPayloadBytes = static_cast<int>(payload.size());
            stats.bytes += payload.size();
            stats.updates++;
            if (sessionRecorder.active()) {
              sessionRecorder.record(payload, now, sessionStatus);
            }
            std::vector<PendingSensorUpdate> pending;
            const bool parsedOk = applyScenePayload(payload, false, now, pending);
            if (!parsedOk) {
              networkFailed = lastStatus.find("disconnect") != std::string::npos;
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
        clearSceneState();
      }
    }

    if (requestFrameScene) {
      frameScene(*viewer, scene);
      requestFrameScene = false;
    }
    if (requestFrameSelected) {
      uint32_t actionTag = 0;
      int actionIndex = 0;
      const VisualEntry* actionEntry = nullptr;
      scene.getVisualInfo(viewer->getTargetVisual(), actionTag, actionIndex, actionEntry);
      frameSelected(*viewer, actionEntry);
      requestFrameSelected = false;
    }
    if (requestResetCamera) {
      viewer->setTargetVisual(nullptr);
      applyCameraLookAt(viewer->getCamera(), defaultCameraPos, defaultCameraTarget);
      requestResetCamera = false;
    }

    if (exportScenePending && scene.selectableObjectCount() > 0) {
      if (assetDiagnostics.empty()) {
        assetDiagnostics = collectAssetDiagnostics(scene);
      }
      exportSceneJson(options.exportScenePath, scene, assetDiagnostics, captureStatus);
      exportScenePending = false;
    }

    if (showWorldFrame) {
      if (!worldFrame) {
        worldFrame = viewer->addCoordinateFrame("world_frame");
      }
      if (worldFrame) {
        worldFrame->poses.resize(1);
        setTcpViewerIdentityPose(worldFrame->poses[0]);
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
      settings.recentConnections = recentConnections;
      settings.resourceDirs = resourceDirs;
      applyViewerSettings(*viewer, settings);
      settingsApplied = true;
      if (settingsDirty) {
        settingsSavePending = true;
        lastSettingsDirtyTime = now;
        settingsDirty = false;
      }
    }
    if (settingsSavePending && now - lastSettingsDirtyTime >= kSettingsSaveDebounce) {
      saveViewerSettings(settings);
      settingsSavePending = false;
    }
    const bool weatherControlsLight = settings.skyEnabled && settings.skyWeatherEnabled &&
      weatherDefaultEnabledForQuality(settings.renderQuality);
    if (weatherControlsLight) {
      viewer->updateWeather(static_cast<double>(std::max(0.0f, io.DeltaTime)));
    } else {
      auto& lightRef = viewer->getLight();
      lightRef.type = raisin::LightType::DIRECTIONAL;
      lightRef.ambient = settings.mainLightAmbient * ambientStrength;
      lightRef.diffuse = settings.mainLightDiffuse * lightStrength;
      lightRef.specular = settings.mainLightSpecular * lightStrength;
      lightRef.direction = lightDirectionFromYawPitch(lightYawDeg, lightPitchDeg);
      lightRef.setShadowsEnabled(settings.shadowsEnabled);
      lightRef.setShadowResolution(settings.shadowResolution);
      lightRef.setShadowParams(settings.shadowBias, settings.shadowStrength, settings.shadowPcfRadius);
    }

    scene.updateContactVisuals(
      showContactPoints, contactPointSize, showContactForces, contactForceSize);

    const bool canQueueSimControl = client.isConnected() && scene.serverSupportsSimControl();
    const bool mouseForceCanStart = !ruler.enabled && mouseForceEnabled && canQueueSimControl &&
      requestedEntry && supportsTcpViewerForceControl(requestedEntry);
    const bool mouseForceShortcutActive = mouseForceCanStart && io.KeyShift;
    const bool mouseForceSuppressViewportInput = mouseForce.active ||
      (mouseForceShortcutActive && io.MouseDown[ImGuiMouseButton_Left]);
    viewportState = ViewerViewportState{};

    const auto tFrameStart = std::chrono::steady_clock::now();
    const bool rulerCapturesViewportInput = ruler.enabled && !mouseForce.active;
    renderViewer(*viewer, window, !mouseForceSuppressViewportInput && !rulerCapturesViewportInput,
      !mouseForce.active && !rulerCapturesViewportInput, &viewportState);

    if (ruler.enabled && !mouseForce.active && viewportState.hovered &&
        ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
      glm::vec3 rulerPoint(0.0f);
      if (readRulerWorldPointAtCursor(viewer->getCamera(), viewportState, rulerPoint)) {
        appendRulerPoint(rulerPoint, "scene point");
      } else {
        lastStatus = "ruler pick missed";
      }
    }

    if (!mouseForce.active && mouseForceShortcutActive && viewportState.hovered &&
        ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      const glm::vec3 applicationPoint = requestedEntry->lastPos + controlPointOffset;
      ImVec2 anchorScreen;
      if (isMouseWithinForceStartRadius(viewer->getCamera(), viewportState, *requestedEntry,
            applicationPoint, io.MousePos, anchorScreen)) {
        mouseForce.active = true;
        mouseForce.tag = requestedTag;
        mouseForce.index = requestedIndex;
        mouseForce.localBodyIdx = std::max(0, controlBodyIdx);
        mouseForce.applicationPoint = applicationPoint;
        mouseForce.force = glm::vec3(0.0f);
        mouseForce.pressMouse = anchorScreen;
        mouseForce.currentMouse = io.MousePos;
      }
    }
    const auto cancelPendingMouseForce = [&]() {
      const size_t idx = mouseForce.pendingRequestIndex;
      if (idx != std::numeric_limits<size_t>::max() && idx < pendingControlRequests.size()) {
        pendingControlRequests.erase(pendingControlRequests.begin() + static_cast<long>(idx));
      }
      mouseForce.pendingRequestIndex = std::numeric_limits<size_t>::max();
    };
    const auto queueOrUpdateMouseForce = [&]() {
      raisin::tcp_viewer::SimControlRequest r;
      r.type = raisin::tcp_viewer::ClientRequestType::CR_APPLY_FORCE;
      r.visTag = mouseForce.tag;
      r.localBodyIdx = std::max(0, mouseForce.localBodyIdx);
      r.vec3a = mouseForce.applicationPoint;
      r.vec3b = mouseForce.force;

      const size_t idx = mouseForce.pendingRequestIndex;
      if (idx != std::numeric_limits<size_t>::max() && idx < pendingControlRequests.size()) {
        auto& pending = pendingControlRequests[idx];
        if (pending.type == raisin::tcp_viewer::ClientRequestType::CR_APPLY_FORCE &&
            pending.visTag == mouseForce.tag) {
          pending = r;
          return;
        }
      }
      pendingControlRequests.push_back(r);
      mouseForce.pendingRequestIndex = pendingControlRequests.size() - 1;
    };

    if (mouseForce.active) {
      mouseForce.currentMouse = io.MousePos;
      const ImVec2 dragPixels(mouseForce.currentMouse.x - mouseForce.pressMouse.x,
                              mouseForce.currentMouse.y - mouseForce.pressMouse.y);
      mouseForce.force = mouseForceFromDragPixels(viewer->getCamera(), dragPixels, mouseForceScale);
      controlForce = mouseForce.force;
      const float dragLen = std::sqrt(dragPixels.x * dragPixels.x + dragPixels.y * dragPixels.y);
      const bool mouseButtonDown = io.MouseDown[ImGuiMouseButton_Left];
      const bool shouldApplyMouseForce = mouseButtonDown && dragLen >= 4.0f &&
        glm::length(mouseForce.force) > 1.0e-4f && client.isConnected();
      if (shouldApplyMouseForce) {
        queueOrUpdateMouseForce();
        lastStatus = "mouse force applying";
      } else {
        cancelPendingMouseForce();
      }
      if (!mouseButtonDown) {
        cancelPendingMouseForce();
        mouseForce = MouseForceGesture{};
      }
    }
    drawMouseForcePreview(mouseForce, viewportState, viewer->getCamera());
    if (ruler.enabled) {
      drawRulerOverlay(ruler, viewportState, viewer->getCamera());
    }

    // Headless test harness uses --inspect-close-after-frames; surface per-frame
    // render times so close/reload regressions are easy to spot in CI output.
    if (options.inspectCloseAfterFrames >= 0) {
      const auto frameMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - tFrameStart).count();
      // Filter out the chatter: only log fast warmup-period frames if they're slow.
      // The post-load frames are always logged so close/reload regressions show up.
      const bool inspectorFocus = inspector.active ||
                                  frameSerial < 5 ||
                                  frameSerial == options.inspectAfterFrames ||
                                  frameSerial == options.inspectAfterFrames + 1 ||
                                  frameMs > 50;
      if (inspectorFocus) {
        std::cerr << "[inspect-close-test] frame=" << frameSerial << " renderViewer="
                  << frameMs << "ms\n";
      }
    }
    // Diagnostic: at frame 0 (empty viewer) and frame just before the inspect load,
    // print how many shaders the renderer has actually compiled so we can tell what
    // the lazy path needs vs what background warmup adds.
    if (options.inspectCloseAfterFrames >= 0 &&
        (frameSerial == 1 ||
         frameSerial == options.inspectAfterFrames + 1)) {
      auto d = viewer->shaderWarmupDiagnostics();
      std::cerr << "[shader-diag] frame=" << frameSerial
                << " registered=" << d.shaderProgramCount
                << " linked=" << d.linkedProgramCount;
      auto names = viewer->linkedShaderNames();
      std::cerr << " {";
      for (size_t i = 0; i < names.size(); ++i) {
        std::cerr << (i ? ", " : "") << names[i];
      }
      std::cerr << "}\n";
    }
    // Background shader warmup: compile one targeted shader per frame so the viewer
    // stays interactive while heavy programs (pbrMeshHigh, etc.) build up.
    if (shaderWarmupActive) {
      if (shaderWarmupIdx >= kViewerWarmupShaders.size()) {
        shaderWarmupActive = false;
        const auto totalMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - shaderWarmupStart).count();
        std::cerr << "[rayrai] shader warmup complete: " << totalMs << " ms wall time\n";
      } else {
        const std::string& name = kViewerWarmupShaders[shaderWarmupIdx++];
        const long long ms = viewer->compileShaderByName(name);
        std::cerr << "[rayrai] compiled '" << name << "' in " << ms << " ms\n";
      }
    }
    frameSerial++;
    stats.frames++;
    // Headless deferred --inspect: load only after warmup has had time to tick.
    if (options.inspectAfterFrames >= 0 && !options.inspectorPath.empty() &&
        !inspector.active && frameSerial == options.inspectAfterFrames) {
      std::cerr << "[inspect-after-frames] triggering inspect load at frame "
                << frameSerial << "\n";
      const auto loadStart = std::chrono::steady_clock::now();
      if (!loadAsInspector(options.inspectorPath.string())) {
        std::cerr << "ERROR: deferred --inspect failed: " << inspector.lastError << "\n";
      } else {
        const auto loadMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::steady_clock::now() - loadStart).count();
        std::cerr << "[inspect-after-frames] load returned in " << loadMs << " ms\n";
      }
    }
    // Headless close-inspector reproducer. After the chosen number of frames, click
    // the close path; after another batch, quit. Lets CI catch close-inspector segfaults.
    if (options.inspectCloseAfterFrames >= 0 && inspector.active &&
        frameSerial == options.inspectCloseAfterFrames) {
      std::cerr << "[inspect-close-test] closing inspector at frame " << frameSerial << "\n";
      closeInspector();
    }
    // After close, if --inspect-reload was passed, trigger a second load so we can
    // observe the post-close first-frame cost (should be ~zero because shaders cached).
    if (options.inspectCloseAfterFrames >= 0 && !inspector.active &&
        !options.inspectReloadPath.empty() &&
        frameSerial == options.inspectCloseAfterFrames + 2) {
      std::cerr << "[inspect-close-test] reloading "
                << options.inspectReloadPath.string() << "\n";
      loadAsInspector(options.inspectReloadPath.string());
      options.inspectReloadPath.clear();  // one-shot
    }
    if (options.inspectCloseAfterFrames >= 0 && !inspector.active &&
        options.inspectReloadPath.empty() &&
        frameSerial >= options.inspectCloseAfterFrames * 2 + 2) {
      std::cerr << "[inspect-close-test] exiting after post-close frames\n";
      quit = true;
    }
    if (screenshotRequested) {
      const std::filesystem::path captureDir(screenshotDirBuf);
      const std::filesystem::path path = pendingScreenshotPath.empty()
        ? timestampedCapturePath(captureDir, "rayrai_tcp_viewer")
        : pendingScreenshotPath;
      saveViewerTexturePng(*viewer, path, captureStatus);
      pendingScreenshotPath.clear();
      screenshotRequested = false;
      if (options.exitAfterScreenshot) {
        quit = true;
      }
    }
    if (recordPngSequence && frameSerial % std::max(1, recordEveryNFrames) == 0) {
      std::ostringstream frameName;
      frameName << "rayrai_tcp_viewer_frame_" << std::setw(6) << std::setfill('0')
                << recordFrameIndex++ << ".png";
      saveViewerTexturePng(*viewer, std::filesystem::path(screenshotDirBuf) / frameName.str(), captureStatus);
    }
    updateStatsWindow(stats, now);

    const auto drawViewOptions = [&]() {
      ImGui::SeparatorText("Interface");
      ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10.0f);
      if (drawInlineLabelSliderFloat("ui_scale_panel", "UI Scale", &uiScale, 0.8f, 2.6f, "%.2f")) {
        uiScaleUserSet = true;
        settings.uiScale = uiScale;
        settings.uiScaleUserSet = true;
        settingsDirty = true;
      }
      if (drawIconTextButton(uiIcons, TcpViewerIconKind::Options, "Reset Scale", "reset_screen_scale")) {
        uiScale = defaultUiScale;
        uiScaleUserSet = false;
        settings.uiScale = uiScale;
        settings.uiScaleUserSet = false;
        settingsDirty = true;
      }
      if (ImGui::Checkbox("Show collapsed logo", &settings.showCollapsedLogo)) {
        settingsDirty = true;
      }

      ImGui::SeparatorText("Camera");
      if (drawIconTextButton(uiIcons, TcpViewerIconKind::Home, "Frame Scene", "view_frame_scene")) {
        requestFrameScene = true;
      }
      ImGui::SameLine();
      if (drawIconTextButton(uiIcons, TcpViewerIconKind::Focus, "Frame Selected", "view_frame_selected")) {
        requestFrameSelected = true;
      }
      ImGui::SameLine();
      if (drawIconTextButton(uiIcons, TcpViewerIconKind::Refresh, "Reset Camera", "view_reset_camera")) {
        requestResetCamera = true;
      }

      ImGui::SeparatorText("Bookmarks");
      for (int i = 0; i < static_cast<int>(cameraBookmarks.size()); ++i) {
        ImGui::PushID(i);
        const std::string setLabel = "Set " + std::to_string(i + 1);
        if (drawIconTextButton(uiIcons, TcpViewerIconKind::Save, setLabel.c_str(), "set_bookmark")) {
          cameraBookmarks[static_cast<size_t>(i)].valid = true;
          cameraBookmarks[static_cast<size_t>(i)].position = viewer->getCamera().getPosition();
          cameraBookmarks[static_cast<size_t>(i)].target = viewer->getCamera().target;
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!cameraBookmarks[static_cast<size_t>(i)].valid);
        const std::string restoreLabel = "Restore " + std::to_string(i + 1);
        if (drawIconTextButton(uiIcons, TcpViewerIconKind::Focus, restoreLabel.c_str(), "restore_bookmark")) {
          const auto& bookmark = cameraBookmarks[static_cast<size_t>(i)];
          applyCameraLookAt(viewer->getCamera(), bookmark.position, bookmark.target);
        }
        ImGui::EndDisabled();
        ImGui::PopID();
      }

      ImGui::SeparatorText("Window");
      if (drawIconTextButton(uiIcons, TcpViewerIconKind::Options, "Toggle Fullscreen", "toggle_fullscreen")) {
        const Uint32 flags = SDL_GetWindowFlags(window);
        const bool isFullscreen = (flags & SDL_WINDOW_FULLSCREEN_DESKTOP) != 0;
        SDL_SetWindowFullscreen(window, isFullscreen ? 0 : SDL_WINDOW_FULLSCREEN_DESKTOP);
      }
    };

    const auto drawCaptureOptions = [&]() {
      ImGui::SeparatorText("Screenshots");
      ImGui::SetNextItemWidth(360.0f);
      ImGui::InputText("##ScreenshotDirectory", screenshotDirBuf, sizeof(screenshotDirBuf));
      if (drawIconTextButton(uiIcons, TcpViewerIconKind::Camera, "Screenshot", "capture_screenshot")) {
        screenshotRequested = true;
      }
      ImGui::TextUnformatted("PNG Sequence");
      ImGui::Checkbox("##PngSequence", &recordPngSequence);
      ImGui::BeginDisabled(!recordPngSequence);
      drawInlineLabelSliderInt("png_sequence_every_n_frames", "Every N frames", &recordEveryNFrames, 1, 120);
      ImGui::EndDisabled();

      ImGui::SeparatorText("Session");
      ImGui::SetNextItemWidth(360.0f);
      ImGui::InputText("##SessionFile", sessionPathBuf, sizeof(sessionPathBuf));
      if (!sessionRecorder.active()) {
        if (drawIconTextButton(uiIcons, TcpViewerIconKind::Save, "Start TCP Recording", "start_tcp_recording")) {
          sessionRecorder.open(sessionPathBuf, sessionStatus);
        }
      } else {
        if (drawIconTextButton(uiIcons, TcpViewerIconKind::Disconnect, "Stop TCP Recording", "stop_tcp_recording")) {
          sessionRecorder.close();
          sessionStatus = "recorded " + std::to_string(sessionRecorder.frameCount()) +
                          " frames to " + sessionRecorder.pathString();
        }
        ImGui::TextDisabled("%zu frames | %.1f MiB", sessionRecorder.frameCount(),
          static_cast<double>(sessionRecorder.byteCount()) / (1024.0 * 1024.0));
      }

      if (replayMode) {
        ImGui::SeparatorText("Replay");
        if (drawIconTextButton(uiIcons, TcpViewerIconKind::Refresh,
              replayPaused ? "Resume Replay" : "Pause Replay", "toggle_replay")) {
          replayPaused = !replayPaused;
          replayStart = std::chrono::steady_clock::now();
          replayBaseMicros = replayIndex < replayFrames.size() ? replayFrames[replayIndex].timeMicros
                                                                : replayFrames.back().timeMicros;
        }
        ImGui::SameLine();
        if (drawIconTextButton(uiIcons, TcpViewerIconKind::Focus, "Step", "step_replay")) {
          replayPaused = true;
          replayStep = true;
        }
        if (drawIconTextButton(uiIcons, TcpViewerIconKind::Home, "Restart Replay", "restart_replay")) {
          clearSceneState();
          replayIndex = 0;
          replayStart = std::chrono::steady_clock::now();
          replayBaseMicros = replayFrames.empty() ? 0 : replayFrames.front().timeMicros;
          replayPaused = false;
        }
        drawInlineLabelSliderFloat("replay_speed", "Replay speed", &replaySpeed, 0.05f, 8.0f, "%.2f");
        ImGui::TextDisabled("frame %zu / %zu", std::min(replayIndex, replayFrames.size()), replayFrames.size());
      }
      if (!captureStatus.empty()) {
        ImGui::TextDisabled("%s", shortenPathLabel(captureStatus, 80).c_str());
      }
      if (!sessionStatus.empty()) {
        ImGui::TextDisabled("%s", shortenPathLabel(sessionStatus, 90).c_str());
      }
    };

    const auto drawRenderingOptions = [&]() {
      bool changed = false;
      bool detailChanged = false;
      const ImGuiStyle& renderStyle = ImGui::GetStyle();
      const float colorSwatchWidth = std::round(ImGui::GetFontSize() * 4.2f);
      const auto comboWidthFor = [&](const char* const* items, int itemCount) {
        float maxTextWidth = 0.0f;
        for (int i = 0; i < itemCount; ++i) {
          maxTextWidth = std::max(maxTextWidth, ImGui::CalcTextSize(items[i]).x);
        }
        return std::ceil(maxTextWidth + renderStyle.FramePadding.x * 2.0f +
                         renderStyle.ItemInnerSpacing.x + ImGui::GetFrameHeight());
      };
      const auto drawBackgroundColorPicker = [&]() {
        glm::vec4 color = settings.backgroundColorRgb255 / 255.0f;
        color.r = std::clamp(color.r, 0.0f, 1.0f);
        color.g = std::clamp(color.g, 0.0f, 1.0f);
        color.b = std::clamp(color.b, 0.0f, 1.0f);
        color.a = std::clamp(color.a, 0.0f, 1.0f);
        bool colorChanged = false;
        ImGui::PushID("BackgroundColor");
        const ImGuiColorEditFlags colorFlags = ImGuiColorEditFlags_DisplayRGB |
          ImGuiColorEditFlags_Float | ImGuiColorEditFlags_PickerHueBar |
          ImGuiColorEditFlags_AlphaBar | ImGuiColorEditFlags_AlphaPreviewHalf;
        if (ImGui::ColorButton("##swatch", ImVec4(color.r, color.g, color.b, color.a),
              colorFlags, ImVec2(colorSwatchWidth, ImGui::GetFrameHeight()))) {
          ImGui::OpenPopup("picker");
        }
        if (ImGui::BeginPopup("picker")) {
          colorChanged |= ImGui::ColorPicker4("##picker", &color.x, colorFlags);
          ImGui::EndPopup();
        }
        ImGui::PopID();
        if (colorChanged) {
          settings.backgroundColorRgb255 = glm::vec4(
            std::clamp(color.r, 0.0f, 1.0f) * 255.0f,
            std::clamp(color.g, 0.0f, 1.0f) * 255.0f,
            std::clamp(color.b, 0.0f, 1.0f) * 255.0f,
            std::clamp(color.a, 0.0f, 1.0f) * 255.0f);
        }
        return colorChanged;
      };
      const auto drawLightColorPicker = [&](const char* label, const char* id, glm::vec3& color) {
        ImGui::TextUnformatted(label);
        bool colorChanged = false;
        ImGui::PushID(id);
        const ImGuiColorEditFlags colorFlags = ImGuiColorEditFlags_DisplayRGB |
          ImGuiColorEditFlags_Float | ImGuiColorEditFlags_PickerHueBar |
          ImGuiColorEditFlags_HDR;
        if (ImGui::ColorButton("##swatch", ImVec4(color.r, color.g, color.b, 1.0f),
              colorFlags, ImVec2(colorSwatchWidth, ImGui::GetFrameHeight()))) {
          ImGui::OpenPopup("picker");
        }
        if (ImGui::BeginPopup("picker")) {
          colorChanged |= ImGui::ColorPicker3("##picker", &color.x, colorFlags);
          ImGui::EndPopup();
        }
        ImGui::PopID();
        return colorChanged;
      };
      ImGui::SeparatorText("Quality");
      int quality = std::clamp(settings.renderQuality, 0, 4);
      constexpr const char* qualityItems[] = {"Fast", "Balanced", "High", "Ultra", "Custom"};
      if (ImGui::Combo("##RenderQuality", &quality, qualityItems, IM_ARRAYSIZE(qualityItems))) {
        settings.renderQualityUserSet = true;
        if (quality == 4) {
          settings.renderQuality = quality;
        } else {
          copyRenderDefaultsToSettings(settings, quality);
          settings.renderQualityUserSet = true;
        }
        changed = true;
      }

      ImGui::SeparatorText("Background");
      detailChanged |= drawBackgroundColorPicker();

      ImGui::SeparatorText("Sky");
      detailChanged |= ImGui::Checkbox("Enabled", &settings.skyEnabled);
      const bool weatherAllowed = weatherDefaultEnabledForQuality(settings.renderQuality);
      if (!weatherAllowed) {
        settings.skyWeatherEnabled = false;
      }
      ImGui::BeginDisabled(!settings.skyEnabled);
      ImGui::BeginDisabled(!weatherAllowed);
      detailChanged |= ImGui::Checkbox("Weather model", &settings.skyWeatherEnabled);
      ImGui::EndDisabled();
      if (weatherAllowed && settings.skyWeatherEnabled) {
        constexpr const char* weatherPresetItems[] = {
          "Clear", "Hazy", "Overcast", "Fog", "Rain", "Heavy Rain",
          "Snow", "Storm", "Night Clear", "Night Rain", "Custom"};
        int weatherPreset = std::clamp(settings.skyWeatherPreset, 0, 10);
        ImGui::TextUnformatted("Weather");
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 9.0f);
        if (ImGui::Combo("##SkyWeatherPreset", &weatherPreset, weatherPresetItems,
              IM_ARRAYSIZE(weatherPresetItems))) {
          copyWeatherPresetToSettings(settings, weatherPreset);
          detailChanged = true;
        }
        constexpr const char* weatherQualityItems[] = {"Low", "Medium", "High", "Ultra"};
        int weatherQuality = std::clamp(settings.skyWeatherQuality, 0, 3);
        ImGui::TextUnformatted("Quality");
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 7.0f);
        if (ImGui::Combo("##SkyWeatherQuality", &weatherQuality, weatherQualityItems,
              IM_ARRAYSIZE(weatherQualityItems))) {
          settings.skyWeatherQuality = weatherQuality;
          detailChanged = true;
        }
        if (ImGui::TreeNodeEx("Time & Sun", ImGuiTreeNodeFlags_DefaultOpen)) {
          detailChanged |= drawInlineLabelSliderFloat("render_sky_time_of_day",
            "Time", &settings.skyTimeOfDayHours, 0.0f, 24.0f, "%.2f h");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_latitude",
            "Latitude", &settings.skyLatitude, -89.9f, 89.9f, "%.1f");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_longitude",
            "Longitude", &settings.skyLongitude, -180.0f, 180.0f, "%.1f");
          detailChanged |= drawInlineLabelSliderInt("render_sky_year",
            "Year", &settings.skyYear, 1900, 2500);
          detailChanged |= drawInlineLabelSliderInt("render_sky_month",
            "Month", &settings.skyMonth, 1, 12);
          detailChanged |= drawInlineLabelSliderInt("render_sky_day",
            "Day", &settings.skyDay, 1, 31);
          detailChanged |= ImGui::Checkbox("Explicit sun", &settings.skyUseExplicitSunAngles);
          ImGui::BeginDisabled(!settings.skyUseExplicitSunAngles);
          detailChanged |= drawInlineLabelSliderFloat("render_sky_sun_azimuth",
            "Azimuth", &settings.skySunAzimuthDeg, 0.0f, 360.0f, "%.1f");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_sun_elevation",
            "Elevation", &settings.skySunElevationDeg, -8.0f, 89.0f, "%.1f");
          ImGui::EndDisabled();
          detailChanged |= drawInlineLabelSliderFloat("render_sky_sun_size",
            "Sun size", &settings.skySunSize, 0.001f, 0.08f, "%.3f");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_moon_size",
            "Moon size", &settings.skyMoonSize, 0.001f, 0.08f, "%.3f");
          ImGui::TreePop();
        }
        if (ImGui::TreeNodeEx("Clouds", ImGuiTreeNodeFlags_DefaultOpen)) {
          // Cloud quality: "Auto" picks per render preset (Fast/Balanced → Texture,
          // High/Ultra → Volumetric) and follows the active weather preset; Off
          // disables clouds entirely; Texture/Volumetric force a specific path.
          constexpr const char* cloudQualityItems[] = {"Auto", "Off", "Texture", "Volumetric"};
          int cloudQ = std::clamp(settings.skyCloudQuality, 0, 3);
          ImGui::TextUnformatted("Quality");
          ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8.0f);
          if (ImGui::Combo("##SkyCloudQuality", &cloudQ, cloudQualityItems,
                IM_ARRAYSIZE(cloudQualityItems))) {
            settings.skyCloudQuality = cloudQ;
            detailChanged = true;
          }
          detailChanged |= drawInlineLabelSliderFloat("render_sky_cloud_coverage",
            "Coverage", &settings.skyCloudCoverage, 0.0f, 1.0f, "%.2f");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_cloud_density",
            "Density", &settings.skyCloudDensity, 0.0f, 1.0f, "%.2f");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_cloud_altitude",
            "Altitude", &settings.skyCloudAltitudeMeters, 20.0f, 12000.0f, "%.0f m");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_cloud_thickness",
            "Thickness", &settings.skyCloudThicknessMeters, 1.0f, 4000.0f, "%.0f m");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_cloud_shadow",
            "Shadow", &settings.skyCloudShadowStrength, 0.0f, 1.0f, "%.2f");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_cloud_scale",
            "Scale", &settings.skyCloudScale, 0.01f, 2.0f, "%.2f");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_cloud_animation",
            "Animation", &settings.skyCloudAnimationSpeed, 0.0f, 200.0f, "%.1f");
          ImGui::TreePop();
        }
        if (ImGui::TreeNodeEx("Atmosphere", ImGuiTreeNodeFlags_DefaultOpen)) {
          detailChanged |= drawInlineLabelSliderFloat("render_sky_air_turbidity",
            "Turbidity", &settings.skyAirTurbidity, 1.0f, 12.0f, "%.2f");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_visibility",
            "Visibility", &settings.skyVisibilityMeters, 1.0f, 100000.0f, "%.0f m");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_fog_density",
            "Fog", &settings.skyFogDensity, 0.0f, 1.0f, "%.4f");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_fog_anisotropy",
            "Fog phase", &settings.skyFogAnisotropy, -0.85f, 0.85f, "%.2f");
          ImGui::TextUnformatted("Fog color");
          detailChanged |= ImGui::ColorEdit3("##SkyFogColor", &settings.skyFogColor.x);
          detailChanged |= drawInlineLabelSliderFloat("render_sky_ground_albedo",
            "Ground albedo", &settings.skyGroundAlbedo, 0.0f, 1.0f, "%.2f");
          ImGui::TreePop();
        }
        if (ImGui::TreeNodeEx("Weather Effects", ImGuiTreeNodeFlags_DefaultOpen)) {
          detailChanged |= drawInlineLabelSliderFloat("render_sky_precipitation",
            "Precip", &settings.skyPrecipitationRate, 0.0f, 1.0f, "%.2f");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_rain_occlusion",
            "Rain occlusion", &settings.skyRainOcclusionStrength, 0.0f, 1.0f, "%.2f");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_snow",
            "Snow", &settings.skySnowCoverage, 0.0f, 1.0f, "%.2f");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_humidity",
            "Humidity", &settings.skyHumidity, 0.0f, 1.0f, "%.2f");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_wetness",
            "Wetness", &settings.skyWetness, 0.0f, 1.0f, "%.2f");
          detailChanged |= ImGui::Checkbox("Accumulate wetness", &settings.skyWetnessAccumulationEnabled);
          ImGui::BeginDisabled(!settings.skyWetnessAccumulationEnabled);
          detailChanged |= drawInlineLabelSliderFloat("render_sky_wetness_accum_rate",
            "Wet gain", &settings.skyWetnessAccumulationRate, 0.0f, 4.0f, "%.2f");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_wetness_dry_rate",
            "Drying", &settings.skyWetnessDryingRate, 0.0f, 4.0f, "%.2f");
          ImGui::EndDisabled();
          detailChanged |= drawInlineLabelSliderFloat("render_sky_lightning",
            "Lightning", &settings.skyLightningRate, 0.0f, 16.0f, "%.2f");
          detailChanged |= ImGui::Checkbox("Lens droplets", &settings.skyLensDropletsEnabled);
          ImGui::BeginDisabled(!settings.skyLensDropletsEnabled);
          detailChanged |= drawInlineLabelSliderFloat("render_sky_lens_droplet_strength",
            "Droplets", &settings.skyLensDropletStrength, 0.0f, 1.0f, "%.2f");
          ImGui::EndDisabled();
          ImGui::TreePop();
        }
        if (ImGui::TreeNodeEx("Wind", ImGuiTreeNodeFlags_DefaultOpen)) {
          detailChanged |= drawInlineLabelSliderFloat("render_sky_wind_direction",
            "Direction", &settings.skyWindDirectionDeg, 0.0f, 360.0f, "%.1f");
          detailChanged |= drawInlineLabelSliderFloat("render_sky_wind_speed",
            "Speed", &settings.skyWindSpeed, 0.0f, 80.0f, "%.1f m/s");
          detailChanged |= drawInlineLabelSliderInt("render_sky_seed",
            "Seed", &settings.skyWeatherSeed, 1, 1000000);
          ImGui::TreePop();
        }
      } else {
        detailChanged |= drawInlineLabelSliderFloat("render_sky_sun_strength",
          "Sun", &settings.skySunStrength, 0.0f, 8.0f, "%.2f");
        detailChanged |= drawInlineLabelSliderFloat("render_sky_sun_size",
          "Sun size", &settings.skySunSize, 0.001f, 0.08f, "%.3f");
      }
      ImGui::EndDisabled();

      ImGui::SeparatorText("Camera");
      detailChanged |= drawInlineLabelSliderFloat("render_move_speed", "Move speed", &settings.cameraSpeed, 0.1f, 30.0f, "%.1f");
      detailChanged |= drawInlineLabelSliderFloat("render_fov_deg", "FOV (deg)", &settings.cameraFovDeg, 20.0f, 100.0f, "%.1f");
      detailChanged |= drawInlineLabelSliderFloat("render_near_clip", "Near clip", &settings.cameraNear, 0.001f, 1.0f, "%.3f");
      detailChanged |= drawInlineLabelSliderFloat("render_far_clip", "Far clip", &settings.cameraFar, 10.0f, 5000.0f, "%.0f");

      ImGui::SeparatorText("Light");
      ImGui::BeginDisabled(settings.skyEnabled && settings.skyWeatherEnabled);
      detailChanged |= drawInlineLabelSliderFloat("render_light_yaw", "Yaw (deg)", &settings.lightYawDeg, -180.0f, 180.0f, "%.1f");
      detailChanged |= drawInlineLabelSliderFloat("render_light_pitch", "Pitch (deg)", &settings.lightPitchDeg, -89.0f, 89.0f, "%.1f");
      detailChanged |= drawInlineLabelSliderFloat("render_key_strength", "Key strength", &settings.lightStrength, 0.0f, 2.0f, "%.2f");
      detailChanged |= drawInlineLabelSliderFloat("render_ambient", "Ambient", &settings.ambientStrength, 0.0f, 2.0f, "%.2f");
      detailChanged |= drawLightColorPicker("Ambient", "LightAmbientColor", settings.mainLightAmbient);
      detailChanged |= drawLightColorPicker("Diffuse", "LightDiffuseColor", settings.mainLightDiffuse);
      detailChanged |= drawLightColorPicker("Specular", "LightSpecularColor", settings.mainLightSpecular);
      ImGui::EndDisabled();
      detailChanged |= ImGui::Checkbox("Fill/rim lights", &settings.addViewerFillLights);

      ImGui::SeparatorText("Shadows");
      detailChanged |= ImGui::Checkbox("Enabled", &settings.shadowsEnabled);
      int shadowResolutionIndex =
        settings.shadowResolution <= 1024 ? 0 : settings.shadowResolution <= 2048 ? 1 :
        settings.shadowResolution <= 4096 ? 2 : 3;
      constexpr const char* shadowResolutionItems[] = {"1024", "2048", "4096", "8192"};
      ImGui::TextUnformatted("Shadow map");
      ImGui::SetNextItemWidth(comboWidthFor(shadowResolutionItems, IM_ARRAYSIZE(shadowResolutionItems)));
      if (ImGui::Combo("##ShadowMap", &shadowResolutionIndex, shadowResolutionItems,
            IM_ARRAYSIZE(shadowResolutionItems))) {
        constexpr int values[] = {1024, 2048, 4096, 8192};
        settings.shadowResolution = values[shadowResolutionIndex];
        detailChanged = true;
      }
      detailChanged |= drawInlineLabelSliderFloat("render_shadow_bias", "Bias", &settings.shadowBias, 0.0f, 0.01f, "%.5f");
      detailChanged |= drawInlineLabelSliderFloat("render_shadow_strength", "Strength", &settings.shadowStrength, 0.0f, 1.0f, "%.2f");
      detailChanged |= drawInlineLabelSliderFloat("render_shadow_pcf_radius", "PCF", &settings.shadowPcfRadius, 0.0f, 4.0f, "%.2f");
      detailChanged |= drawInlineLabelSliderFloat("render_shadow_ortho_half_size", "Ortho half", &settings.shadowOrthoHalfSize, 1.0f, 100.0f, "%.1f");
      detailChanged |= drawInlineLabelSliderFloat("render_shadow_near", "Near", &settings.shadowNear, 0.01f, 10.0f, "%.2f");
      detailChanged |= drawInlineLabelSliderFloat("render_shadow_far", "Far", &settings.shadowFar, 1.0f, 250.0f, "%.1f");
      detailChanged |= drawInlineLabelSliderFloat("render_shadow_center_offset", "Center offset", &settings.shadowCenterOffset, 0.0f, 80.0f, "%.1f");
      detailChanged |= ImGui::Checkbox("Update every frame", &settings.updateShadowsEveryFrame);
      detailChanged |= drawInlineLabelSliderInt("render_shadowed_light_budget", "Light budget", &settings.shadowedLightBudget, 0, 8);
      detailChanged |= drawInlineLabelSliderInt("render_point_shadow_lights", "Point lights", &settings.maxPointShadowLights, 0, 8);
      detailChanged |= drawInlineLabelSliderFloat("render_additional_shadow_resolution_scale",
        "Extra map scale", &settings.additionalShadowResolutionScale, 0.05f, 2.0f, "%.2f");
      detailChanged |= drawInlineLabelSliderFloat("render_point_shadow_resolution_scale",
        "Point map scale", &settings.pointShadowResolutionScale, 0.05f, 2.0f, "%.2f");
      detailChanged |= drawInlineLabelSliderInt("render_min_additional_resolution",
        "Min map size", &settings.minAdditionalShadowResolution, 64, 2048);
      detailChanged |= ImGui::Checkbox("Auto imported light", &settings.autoSelectImportedShadowLight);

      ImGui::SeparatorText("Post");
      detailChanged |= drawInlineLabelSliderFloat("render_fog_density", "Fog", &settings.fogDensity, 0.0f, 0.08f, "%.4f");
      detailChanged |= drawInlineLabelSliderFloat("render_gamma", "Gamma", &settings.gamma, 0.5f, 2.5f, "%.2f");
      int colorMode = std::clamp(settings.colorMode, 0, 2);
      constexpr const char* colorModeItems[] = {"Fast Linear", "ACES Approx", "Unreal Preview"};
      ImGui::TextUnformatted("Color mode");
      ImGui::SetNextItemWidth(comboWidthFor(colorModeItems, IM_ARRAYSIZE(colorModeItems)));
      if (ImGui::Combo("##ColorMode", &colorMode, colorModeItems, IM_ARRAYSIZE(colorModeItems))) {
        settings.colorMode = colorMode;
        detailChanged = true;
      }
      detailChanged |= ImGui::Checkbox("FXAA", &settings.fxaaEnabled);
      detailChanged |= ImGui::Checkbox("Bloom", &settings.bloomEnabled);
      ImGui::BeginDisabled(!settings.bloomEnabled);
      detailChanged |= drawInlineLabelSliderFloat("render_bloom_threshold", "Threshold", &settings.bloomThreshold, 0.0f, 4.0f, "%.2f");
      detailChanged |= drawInlineLabelSliderFloat("render_bloom_strength", "Strength", &settings.bloomStrength, 0.0f, 2.0f, "%.2f");
      detailChanged |= drawInlineLabelSliderFloat("render_bloom_radius", "Radius", &settings.bloomRadius, 0.0f, 12.0f, "%.1f");
      detailChanged |= drawInlineLabelSliderFloat("render_bloom_knee", "Knee", &settings.bloomKnee, 0.0f, 1.0f, "%.2f");
      detailChanged |= drawInlineLabelSliderInt("render_bloom_quality", "Quality", &settings.bloomQuality, 0, 3);
      ImGui::EndDisabled();
      detailChanged |= ImGui::Checkbox("Screen-space AO", &settings.screenSpaceAoEnabled);
      ImGui::BeginDisabled(!settings.screenSpaceAoEnabled);
      detailChanged |= drawInlineLabelSliderFloat("render_ao_radius", "Radius", &settings.screenSpaceAoRadius, 0.05f, 10.0f, "%.2f");
      detailChanged |= drawInlineLabelSliderFloat("render_ao_strength", "Strength", &settings.screenSpaceAoStrength, 0.0f, 4.0f, "%.2f");
      detailChanged |= drawInlineLabelSliderFloat("render_ao_bias", "Bias", &settings.screenSpaceAoBias, 0.0f, 0.25f, "%.3f");
      ImGui::EndDisabled();
      detailChanged |= ImGui::Checkbox("Opaque depth prepass", &settings.opaqueDepthPrepass);
      detailChanged |= ImGui::Checkbox("Depth of field", &settings.depthOfFieldEnabled);
      ImGui::BeginDisabled(!settings.depthOfFieldEnabled);
      detailChanged |= drawInlineLabelSliderFloat("render_dof_focus_distance",
        "Focus distance", &settings.depthOfFieldFocusDistance, 0.05f, 30.0f, "%.2f");
      detailChanged |= drawInlineLabelSliderFloat("render_dof_focus_range",
        "Focus range", &settings.depthOfFieldFocusRange, 1.0f, 100.0f, "%.1f");
      detailChanged |= drawInlineLabelSliderFloat("render_dof_max_blur_radius",
        "Max blur radius", &settings.depthOfFieldMaxRadius, 0.0f, 8.0f, "%.2f");
      ImGui::EndDisabled();

      ImGui::SeparatorText("PBR");
      detailChanged |= ImGui::Checkbox("High fidelity", &settings.highFidelityPbr);
      detailChanged |= ImGui::Checkbox("Tone mapping", &settings.pbrToneMapping);
      detailChanged |= drawInlineLabelSliderFloat("render_pbr_exposure", "Exposure", &settings.pbrExposure, 0.1f, 4.0f, "%.2f");
      detailChanged |= drawInlineLabelSliderFloat("render_pbr_environment_max_lod", "Environment LOD", &settings.pbrEnvironmentMaxLod, 0.0f, 12.0f, "%.1f");
      detailChanged |= drawInlineLabelSliderFloat("render_pbr_environment_intensity", "Environment", &settings.pbrEnvironmentIntensity, 0.0f, 4.0f, "%.2f");
      detailChanged |= drawInlineLabelSliderFloat("render_pbr_key_intensity", "Key", &settings.pbrKeyLightIntensity, 0.0f, 4.0f, "%.2f");

      ImGui::SeparatorText("Ground");
      detailChanged |= ImGui::Checkbox("Reflective checkerboard", &settings.reflectiveGround);
      ImGui::BeginDisabled(!settings.reflectiveGround);
      detailChanged |= drawInlineLabelSliderFloat("render_ground_roughness",
        "Roughness", &settings.reflectiveGroundRoughness, 0.02f, 1.0f, "%.2f");
      detailChanged |= drawInlineLabelSliderFloat("render_ground_metallic",
        "Metallic", &settings.reflectiveGroundMetallic, 0.0f, 1.0f, "%.2f");
      ImGui::EndDisabled();

      ImGui::SeparatorText("Advanced");
      detailChanged |= ImGui::Checkbox("Sort transparent", &settings.sortTransparentInstances);
      detailChanged |= drawInlineLabelSliderInt("render_additional_lights_per_frame",
        "Lights per frame", &settings.maxAdditionalLightsPerFrame, 0, 16);
      detailChanged |= drawInlineLabelSliderFloat("render_min_light_influence",
        "Min influence", &settings.minAdditionalLightInfluence, 0.0f, 1.0f, "%.3f");

      if (drawIconTextButton(uiIcons, TcpViewerIconKind::Refresh, "Reset Rendering Settings", "reset_rendering_settings")) {
        ViewerSettings defaults;
        applyAutomaticRenderQualityIfUnset(defaults, gpuQuality.quality);
        defaults.uiScale = settings.uiScale;
        defaults.uiScaleUserSet = settings.uiScaleUserSet;
        defaults.showCollapsedLogo = settings.showCollapsedLogo;
        settings = defaults;
        changed = true;
      }

      changed |= detailChanged;
      if (changed) {
        if (detailChanged) {
          settings.renderQuality = 4;
          settings.renderQualityUserSet = true;
        }
        cameraSpeed = settings.cameraSpeed;
        lightYawDeg = settings.lightYawDeg;
        lightPitchDeg = settings.lightPitchDeg;
        lightStrength = settings.lightStrength;
        ambientStrength = settings.ambientStrength;
        settingsDirty = true;
      }
    };

    const ImVec2 overlayBase(12.0f, 12.0f + menuBarHeight);
    const ImVec2 overlayPos(overlayBase.x + overlayOffset.x, overlayBase.y + overlayOffset.y);
    const float collapsedPanelPadding = std::max(2.0f, std::round(ImGui::GetFontSize() * 0.18f));
    const bool collapsedLogoVisible = overlayMinimized && settings.showCollapsedLogo && raisimLogo.valid();
    // Inspector mode is a modal alternative to the TCP client UI — hide the entire
    // left overlay (connection / render / objects / diagnostics tabs) while it's
    // active so the user isn't tempted to interact with mutually-exclusive state.
    const bool overlayVisible = !inspector.active;
    bool overlayHovered = false;
    if (overlayVisible) {
    ImGui::SetNextWindowBgAlpha(collapsedLogoVisible ? 0.90f : 0.5f);
    ImGui::SetNextWindowPos(overlayPos, ImGuiCond_Always);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,
      overlayMinimized ? ImVec2(collapsedPanelPadding, collapsedPanelPadding) : ImVec2(12.0f, 10.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, ImGui::GetStyle().WindowBorderSize);
    const int overlayColorPushCount = collapsedLogoVisible ? 2 : 0;
    if (collapsedLogoVisible) {
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.96f, 0.97f, 0.99f, 1.0f));
      ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.86f, 0.88f, 0.92f, 0.95f));
    }
    const ImGuiWindowFlags overlayFlags =
      ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
      ImGuiWindowFlags_NoNavFocus;
    if (ImGui::Begin("Raisim TCP##Overlay", nullptr, overlayFlags)) {
      overlayHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
      const bool overlayHoverShouldOpen = overlayMinimized && overlayHovered &&
                                          !overlayCollapsedHoveredLastFrame;
      const ImGuiStyle& style = ImGui::GetStyle();
      if (overlayHoverShouldOpen) {
        overlayMinimized = false;
        overlayLastInteractionTime = now;
      }

      if (overlayMinimized) {
        drawCollapsedLeftPanelLogo(collapsedLogoVisible ? raisimLogo : TcpViewerImageTexture{});
      } else if (ImGui::BeginTabBar("##LeftTabs")) {
        if (ImGui::BeginTabItem("Connection")) {
          ImGui::PushStyleColor(ImGuiCol_TextDisabled, ImVec4(0.66f, 0.71f, 0.78f, 1.0f));
          ConnectionEntry current;
          current.host = host;
          current.port = port;
          std::string preview = formatConnectionLabel(current);
          if (preview.empty()) {
            preview = "set host:port";
          }
          const float comboLabelWidth = ImGui::CalcTextSize(preview.c_str()).x;
          const float minHostTextWidth = ImGui::CalcTextSize("255.255.255.255").x;
          const float minPortTextWidth = ImGui::CalcTextSize("0").x;
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
              int parsed = port;
              if (parsePortStrict(portBuf, parsed)) {
                port = parsed;
              }
            }
            ImGui::SeparatorText("Remote server");
            ImGui::TextDisabled("Enter any DNS name or IP address and port above.");
            if (drawIconTextButton(uiIcons, TcpViewerIconKind::Save, "Save Endpoint", "save_endpoint")) {
              ConnectionEntry endpoint;
              if (normalizeConnectionEndpoint(host, port, endpoint)) {
                std::snprintf(host, sizeof(host), "%s", endpoint.host.c_str());
                port = endpoint.port;
                std::snprintf(portBuf, sizeof(portBuf), "%d", port);
                recordConnection(recentConnections, endpoint.host, endpoint.port);
                settingsDirty = true;
              } else {
                lastStatus = "invalid endpoint";
              }
            }
            ImGui::SeparatorText("Detected RaisimServer beacons");
            if (!discoveryStatus.empty()) {
              ImGui::TextDisabled("%s; showing protocol %d only", discoveryStatus.c_str(),
                raisin::tcp_viewer::kProtocolVersion);
            }
            if (drawIconTextButton(uiIcons, TcpViewerIconKind::Refresh, "Refresh", "refresh_servers")) {
              beaconReceiver.poll();
              discoveredServers = beaconReceiver.servers();
            }
            bool shownDetectedServer = false;
            for (const auto& server : discoveredServers) {
              shownDetectedServer = true;
              const std::string label = formatServerLabel(server);
              if (ImGui::Selectable(label.c_str())) {
                std::snprintf(host, sizeof(host), "%s", server.endpoint.host.c_str());
                port = server.endpoint.port;
                std::snprintf(portBuf, sizeof(portBuf), "%d", port);
                if (!client.isConnected()) {
                  connectToEndpoint(server.endpoint, true, "connecting", "connect failed");
                }
                ImGui::CloseCurrentPopup();
              }
            }
            if (!shownDetectedServer) {
              ImGui::TextDisabled("No compatible RaisimServer beacons detected");
            }
            ImGui::SeparatorText("Saved / recent endpoints");
            if (recentConnections.empty()) {
              ImGui::TextDisabled("No saved or recent endpoints");
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
            ImGui::BeginDisabled(inspector.active);
            if (drawIconTextButton(uiIcons, TcpViewerIconKind::Connect, "Connect", "connect")) {
              ConnectionEntry endpoint;
              if (!normalizeConnectionEndpoint(host, port, endpoint)) {
                lastStatus = "invalid endpoint";
              } else {
                connectToEndpoint(endpoint, true, "connecting", "connect failed");
              }
            }
            ImGui::EndDisabled();
            if (inspector.active) {
              ImGui::SameLine();
              ImGui::TextDisabled("(inspector active — close to connect)");
            }
          } else {
            if (drawIconTextButton(uiIcons, TcpViewerIconKind::Disconnect, "Disconnect", "disconnect")) {
              client.disconnect();
              awaitingResponse = false;
              awaitingSensorAck = false;
              lastStatus = "disconnected";
              clearSceneState();
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
          ImGui::TextColored(statusColor, "Status: %s", lastStatus.c_str());
          ImGui::SameLine();
          ImGui::TextUnformatted(worldText);
          ImGui::TextDisabled("Heightmap colors: server color map");
          ImGui::TextDisabled("FPS %.1f | updates %.1f Hz", stats.fps, stats.updateHz);
          ImGui::TextDisabled("Objects %zu | visuals %zu | instanced %zu | point clouds %zu",
            scene.selectableObjectCount(), scene.visualCount(), scene.instancedCount(), scene.pointCloudCount());
          ImGui::TextDisabled("Assets unresolved %zu | sensor requests %d | session %s",
            stats.unresolvedAssets, stats.pendingSensorRequests,
            sessionRecorder.active() ? "recording" : (replayMode ? "replay" : "live"));
          if (drawIconTextButton(uiIcons, TcpViewerIconKind::Home, "Frame Scene", "frame_scene")) requestFrameScene = true;
          ImGui::SameLine();
          if (drawIconTextButton(uiIcons, TcpViewerIconKind::Focus, "Frame Selected", "frame_selected")) requestFrameSelected = true;
          ImGui::SameLine();
          if (drawIconTextButton(uiIcons, TcpViewerIconKind::Camera, "Screenshot", "screenshot")) screenshotRequested = true;
          if (!captureStatus.empty()) {
            ImGui::TextDisabled("%s", shortenPathLabel(captureStatus, 70).c_str());
          }

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
                  setTcpViewerIdentityPose(worldFrame->poses[0]);
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
            const float buttonWidth = iconTextButtonSize("Add").x;
            const float spacing = ImGui::GetStyle().ItemInnerSpacing.x;
            const float inputWidth = std::max(0.0f, rowWidth - buttonWidth - spacing);
            ImGui::SetNextItemWidth(inputWidth);
            ImGui::InputTextWithHint(
              "##resource_dir_input", "path/to/resources", searchPathBuf, sizeof(searchPathBuf));
            ImGui::SameLine(0.0f, spacing);
            if (drawIconTextButton(uiIcons, TcpViewerIconKind::Folder, "Add", "resource_add", ImVec2(buttonWidth, 0.0f))) {
              std::string path(searchPathBuf);
              if (!path.empty()) {
                scene.addSearchPath(path);
                recordResourceDir(resourceDirs, path);
                settingsDirty = true;
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
                    settingsDirty = true;
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
          ImGui::PopStyleColor();
          ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Options")) {
          drawViewOptions();
          ImGui::Separator();
          drawCaptureOptions();
          ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Render")) {
          drawRenderingOptions();
          ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Object")) {
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
          auto items = scene.getSelectableObjects();
          const std::string filterLower = toLowerAscii(trimAscii(objectFilterBuf));
          items.erase(std::remove_if(items.begin(), items.end(), [&](const ObjectListItem& item) {
            return isContactItem(item) || (hideCollisionObjects && item.isCollision) ||
                   !objectMatchesFilter(item, filterLower);
          }), items.end());
          std::sort(items.begin(), items.end(), [&](const ObjectListItem& lhs, const ObjectListItem& rhs) {
            if (groupObjectsByType && lhs.objectTypeRaw != rhs.objectTypeRaw) {
              return lhs.objectTypeRaw < rhs.objectTypeRaw;
            }
            return objectLessByMode(lhs, rhs, objectSortMode);
          });

          const float objectFilterWidth = 260.0f;
          const float objectSortWidth = 110.0f;
          const float objectListWidth = objectFilterWidth + style.ItemSpacing.x + objectSortWidth +
                                        style.ItemInnerSpacing.x + ImGui::CalcTextSize("Sort").x;
          ImGui::SetNextItemWidth(objectFilterWidth);
          ImGui::InputTextWithHint("##ObjectFilter", "filter name, type, tag", objectFilterBuf,
            sizeof(objectFilterBuf));
          ImGui::SameLine();
          constexpr const char* sortItems[] = {"Name", "Type", "Tag", "Index"};
          ImGui::SetNextItemWidth(objectSortWidth);
          ImGui::Combo("Sort", &objectSortMode, sortItems, IM_ARRAYSIZE(sortItems));
          ImGui::Checkbox("Group by type", &groupObjectsByType);
          ImGui::SameLine();
          ImGui::Checkbox("Hide collisions", &hideCollisionObjects);
          ImGui::TextDisabled("%zu shown / %zu selectable | visuals %zu",
            items.size(), scene.selectableObjectCount(), scene.visualCount());
          ImGui::TextDisabled("instanced %zu | point clouds %zu",
            scene.instancedCount(), scene.pointCloudCount());

          if (items.empty()) {
            ImGui::TextDisabled("No matching objects");
          } else {
            const ImU32 typeColor = ImGui::GetColorU32(ImVec4(0.35f, 0.8f, 1.0f, 1.0f));
            const float listHeight = ImGui::GetFontSize() * 14.0f;
            const auto drawObjectRow = [&](const ObjectListItem& item) {
              const char* typeName = objectTypeLabel(item.objectTypeRaw);
              const bool selected = hasSelected && item.tag == selectedTag && item.index == selectedIndex;
              const std::string id =
                "##obj_" + std::to_string(item.tag) + "_" + std::to_string(item.index);
              if (ImGui::Selectable(id.c_str(), selected, ImGuiSelectableFlags_SpanAllColumns)) {
                if (viewer) {
                  viewer->setTargetVisual(item.visual.get());
                }
              }
              if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("tag %u, index %d%s", item.tag, item.index,
                  item.isCollision ? ", collision" : "");
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
                ImGui::GetColorU32(item.isCollision ? ImGuiCol_TextDisabled : ImGuiCol_Text),
                nameText.c_str());
            };

            if (ImGui::BeginChild("##ObjectList", ImVec2(objectListWidth, listHeight), true)) {
              if (groupObjectsByType) {
                int currentType = std::numeric_limits<int>::min();
                for (const auto& item : items) {
                  if (item.objectTypeRaw != currentType) {
                    currentType = item.objectTypeRaw;
                    ImGui::SeparatorText(objectTypeLabel(currentType));
                  }
                  drawObjectRow(item);
                }
              } else {
                for (const auto& item : items) {
                  drawObjectRow(item);
                }
              }
              ImGui::EndChild();
            }
          }

          ImGui::SeparatorText("Ruler");
          ImGui::Checkbox("Ruler", &ruler.enabled);
          ImGui::SameLine();
          ImGui::TextDisabled("next %s", (!ruler.hasA || (ruler.hasA && ruler.hasB) ||
            ruler.nextPoint == 0) ? "A" : "B");
          ImGui::BeginDisabled(!hasSelected);
          if (drawIconTextButton(uiIcons, TcpViewerIconKind::Focus, "Set A", "ruler_set_a")) {
            setRulerEndpointFromSelection(0, selectedTag, selectedIndex, selectedEntry);
          }
          ImGui::SameLine();
          if (drawIconTextButton(uiIcons, TcpViewerIconKind::Focus, "Set B", "ruler_set_b")) {
            setRulerEndpointFromSelection(1, selectedTag, selectedIndex, selectedEntry);
          }
          ImGui::EndDisabled();
          ImGui::SameLine();
          ImGui::BeginDisabled(!ruler.hasA && !ruler.hasB);
          if (drawIconTextButton(uiIcons, TcpViewerIconKind::Reset, "Clear", "ruler_clear")) {
            ruler = RulerToolState{};
            lastStatus = "ruler cleared";
          }
          ImGui::EndDisabled();
          const std::string pointAText = ruler.hasA ? formatRulerPoint(ruler.a) : "--";
          const std::string pointBText = ruler.hasB ? formatRulerPoint(ruler.b) : "--";
          ImGui::TextDisabled("A %s", pointAText.c_str());
          if (ruler.hasA && !ruler.aLabel.empty()) {
            ImGui::SameLine();
            ImGui::TextDisabled("%s", shortenPathLabel(ruler.aLabel, 34).c_str());
          }
          ImGui::TextDisabled("B %s", pointBText.c_str());
          if (ruler.hasB && !ruler.bLabel.empty()) {
            ImGui::SameLine();
            ImGui::TextDisabled("%s", shortenPathLabel(ruler.bLabel, 34).c_str());
          }
          if (ruler.hasA && ruler.hasB) {
            const std::string distanceText = formatRulerDistance(glm::distance(ruler.a, ruler.b));
            ImGui::Text("Distance %s", distanceText.c_str());
          }

          // ----- Simulation control (requires server SIM_CONTROL feature + CAP_SIM_CONTROL) -----
          ImGui::SeparatorText("Simulation");
          const bool serverSimControl = scene.serverSupportsSimControl();
          const bool canControlSim = client.isConnected() && serverSimControl;
          ImGui::BeginDisabled(!canControlSim);
          using raisin::tcp_viewer::ClientRequestType;
          // Pause / Resume toggle — icon-only with hover tooltip.
          if (simPaused) {
            if (drawIconOnlyButton(uiIcons, TcpViewerIconKind::Play,
                                   "Resume simulation", "sim_resume")) {
              raisin::tcp_viewer::SimControlRequest r;
              r.type = ClientRequestType::CR_RESUME;
              pendingControlRequests.push_back(r);
              simPaused = false;
            }
          } else {
            if (drawIconOnlyButton(uiIcons, TcpViewerIconKind::Pause,
                                   "Pause simulation", "sim_pause")) {
              raisin::tcp_viewer::SimControlRequest r;
              r.type = ClientRequestType::CR_PAUSE;
              pendingControlRequests.push_back(r);
              simPaused = true;
            }
          }
          // Step buttons — auto-pause first if running, then queue the step(s).
          auto queueStep = [&](int n) {
            raisin::tcp_viewer::SimControlRequest r;
            r.type = ClientRequestType::CR_STEP_N;
            r.stepCount = n;
            pendingControlRequests.push_back(r);
            if (!simPaused) {
              raisin::tcp_viewer::SimControlRequest p;
              p.type = ClientRequestType::CR_PAUSE;
              pendingControlRequests.push_back(p);
              simPaused = true;
            }
          };
          ImGui::SameLine();
          if (drawIconOnlyButton(uiIcons, TcpViewerIconKind::Step,
                                 "Step 1 frame", "sim_step")) {
            queueStep(1);
          }
          ImGui::SameLine();
          if (drawIconOnlyButton(uiIcons, TcpViewerIconKind::StepFast,
                                 "Step 10 frames", "sim_step10")) {
            queueStep(10);
          }
          ImGui::EndDisabled();
          if (!client.isConnected()) {
            ImGui::TextDisabled("Sim control: disconnected");
          } else if (!serverSimControl) {
            ImGui::TextDisabled("Sim control: server does not advertise SIM_CONTROL");
          }

          ImGui::SeparatorText("Selected control");
          const SelectedObjectInfo& controlInfo = scene.getSelectedInfo();
          const bool hasControlSelection = requestedEntry != nullptr && requestedTag != 0;
          if (!hasControlSelection) {
            ImGui::TextDisabled("Select an object to apply controls");
          } else {
            const bool forceSupported = supportsTcpViewerForceControl(requestedEntry);
            const bool poseSupported = supportsTcpViewerPoseControl(requestedEntry);
            const bool gcSupported = requestedEntry->isArticulated;
            const int selectedBodyIdx = requestedEntry->isArticulated ?
              std::max(0, requestedEntry->localBodyIdx) : 0;
            if (controlBodyFollowsSelection) {
              controlBodyIdx = selectedBodyIdx;
            }

            std::string selectedName = requestedEntry->objectName.empty() ?
              scene.getObjectName(requestedTag) : requestedEntry->objectName;
            if (selectedName.empty()) selectedName = objectTypeLabel(requestedEntry->objectTypeRaw);
            ImGui::TextDisabled("%s | tag %u | body %d", selectedName.c_str(),
              requestedTag, selectedBodyIdx);

            const float controlVecWidth = std::round(ImGui::GetFontSize() * 12.5f);
            const auto drawVec3Control = [&](const char* label, const char* id, glm::vec3& value,
                                             float speed) {
              ImGui::TextUnformatted(label);
              ImGui::SetNextItemWidth(controlVecWidth);
              return ImGui::DragFloat3(id, &value.x, speed, -100000.0f, 100000.0f, "%.3g");
            };
            const auto syncPoseFromSelection = [&]() {
              controlPosePosition = requestedEntry->lastPos;
              controlPoseQuat = requestedEntry->lastQuat;
              controlPoseTag = requestedTag;
              controlPoseInitialized = true;
            };

            if (requestedEntry->isArticulated) {
              if (ImGui::Checkbox("Body follows selection", &controlBodyFollowsSelection)) {
                if (controlBodyFollowsSelection) controlBodyIdx = selectedBodyIdx;
              }
              ImGui::BeginDisabled(controlBodyFollowsSelection);
              ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5.0f);
              ImGui::InputInt("Body", &controlBodyIdx);
              if (controlBodyIdx < 0) controlBodyIdx = 0;
              ImGui::EndDisabled();
            }

            ImGui::BeginDisabled(!canControlSim || !forceSupported);
            ImGui::Checkbox("Shift-drag force", &mouseForceEnabled);
            ImGui::BeginDisabled(!mouseForceEnabled);
            drawInlineLabelSliderFloat("mouse_force_scale", "Mouse scale", &mouseForceScale,
              0.05f, 50.0f, "%.2f N/px");
            ImGui::EndDisabled();
            drawVec3Control("Force", "##selected_force", controlForce, 0.25f);
            drawVec3Control("Point offset", "##selected_force_offset", controlPointOffset, 0.01f);
            if (drawIconTextButton(uiIcons, TcpViewerIconKind::Focus, "Apply Force", "apply_selected_force")) {
              raisin::tcp_viewer::SimControlRequest r;
              r.type = ClientRequestType::CR_APPLY_FORCE;
              r.visTag = requestedTag;
              r.localBodyIdx = std::max(0, controlBodyIdx);
              r.vec3a = requestedEntry->lastPos + controlPointOffset;
              r.vec3b = controlForce;
              pendingControlRequests.push_back(r);
              lastStatus = "force queued";
            }
            drawVec3Control("Torque", "##selected_torque", controlTorque, 0.05f);
            if (drawIconTextButton(uiIcons, TcpViewerIconKind::Focus, "Apply Torque", "apply_selected_torque")) {
              raisin::tcp_viewer::SimControlRequest r;
              r.type = ClientRequestType::CR_APPLY_TORQUE;
              r.visTag = requestedTag;
              r.localBodyIdx = std::max(0, controlBodyIdx);
              r.vec3a = controlTorque;
              pendingControlRequests.push_back(r);
              lastStatus = "torque queued";
            }
            ImGui::EndDisabled();
            if (!forceSupported) {
              ImGui::TextDisabled("Force/torque: unsupported object type");
            }

            if (poseSupported) {
              if (!controlPoseInitialized || controlPoseTag != requestedTag) {
                syncPoseFromSelection();
              }
              if (ImGui::TreeNodeEx("Pose", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::BeginDisabled(!canControlSim);
                if (drawIconTextButton(uiIcons, TcpViewerIconKind::Refresh, "Sync Pose", "sync_selected_pose")) {
                  syncPoseFromSelection();
                }
                drawVec3Control("Position", "##selected_pose_position", controlPosePosition, 0.01f);
                ImGui::TextUnformatted("Quaternion WXYZ");
                float poseQuatWxyz[4] = {controlPoseQuat.w, controlPoseQuat.x,
                  controlPoseQuat.y, controlPoseQuat.z};
                ImGui::SetNextItemWidth(controlVecWidth);
                if (ImGui::DragFloat4("##selected_pose_quat", poseQuatWxyz, 0.005f,
                      -1.0f, 1.0f, "%.3f")) {
                  controlPoseQuat.w = poseQuatWxyz[0];
                  controlPoseQuat.x = poseQuatWxyz[1];
                  controlPoseQuat.y = poseQuatWxyz[2];
                  controlPoseQuat.z = poseQuatWxyz[3];
                }
                if (drawIconTextButton(uiIcons, TcpViewerIconKind::Save, "Set Pose", "set_selected_pose")) {
                  raisin::tcp_viewer::SimControlRequest r;
                  r.type = ClientRequestType::CR_SET_POSE;
                  r.visTag = requestedTag;
                  r.vec3a = controlPosePosition;
                  r.quat = normalizedWxyz(controlPoseQuat);
                  controlPoseQuat = r.quat;
                  pendingControlRequests.push_back(r);
                  lastStatus = "pose queued";
                }
                ImGui::EndDisabled();
                ImGui::TreePop();
              }
            }

            if (gcSupported) {
              const bool hasGc = controlInfo.valid && controlInfo.isArticulated &&
                controlInfo.tag == requestedTag && !controlInfo.generalizedCoordinates.empty();
              if (!hasGc) {
                ImGui::TextDisabled("Joint controls: waiting for selected object data");
              } else {
                if (controlGcTag != requestedTag || (!controlGcDirty &&
                    controlGc.size() != controlInfo.generalizedCoordinates.size())) {
                  controlGc = controlInfo.generalizedCoordinates;
                  controlGcTag = requestedTag;
                  controlGcDirty = false;
                }
                if (ImGui::TreeNodeEx("Generalized coordinates", ImGuiTreeNodeFlags_DefaultOpen)) {
                  ImGui::BeginDisabled(!canControlSim);
                  if (drawIconTextButton(uiIcons, TcpViewerIconKind::Refresh, "Sync GC", "sync_selected_gc")) {
                    controlGc = controlInfo.generalizedCoordinates;
                    controlGcTag = requestedTag;
                    controlGcDirty = false;
                  }
                  for (size_t i = 0; i < controlInfo.jointNames.size(); ++i) {
                    const int32_t offset = i < controlInfo.jointGcOffsets.size() ?
                      controlInfo.jointGcOffsets[i] : -1;
                    const int32_t dim = i < controlInfo.jointGcDims.size() ?
                      controlInfo.jointGcDims[i] : 0;
                    const int32_t type = i < controlInfo.jointTypes.size() ?
                      controlInfo.jointTypes[i] : int32_t(raisim::Joint::Type::FIXED);
                    ImGui::PushID(static_cast<int>(i));
                    ImGui::Text("%s [%s]", controlInfo.jointNames[i].c_str(),
                      tcpViewerJointTypeLabel(type));
                    const bool validSlice = offset >= 0 &&
                      static_cast<size_t>(offset + std::max<int32_t>(1, dim) - 1) < controlGc.size();
                    if (!validSlice || dim == 0) {
                      ImGui::TextDisabled("not editable");
                    } else if (dim == 1) {
                      ImGui::SetNextItemWidth(controlVecWidth);
                      if (ImGui::DragFloat("##joint_q", &controlGc[static_cast<size_t>(offset)],
                            0.005f, -1000.0f, 1000.0f, "%.5g")) {
                        controlGcDirty = true;
                      }
                    } else if (dim == 4) {
                      ImGui::SetNextItemWidth(controlVecWidth);
                      if (ImGui::DragFloat4("##joint_quat", &controlGc[static_cast<size_t>(offset)],
                            0.005f, -1.0f, 1.0f, "%.3f")) {
                        controlGcDirty = true;
                      }
                    } else if (dim == 7) {
                      ImGui::TextUnformatted("Position");
                      ImGui::SetNextItemWidth(controlVecWidth);
                      if (ImGui::DragFloat3("##floating_pos", &controlGc[static_cast<size_t>(offset)],
                            0.01f, -1000.0f, 1000.0f, "%.4g")) {
                        controlGcDirty = true;
                      }
                      ImGui::TextUnformatted("Quaternion WXYZ");
                      ImGui::SetNextItemWidth(controlVecWidth);
                      if (ImGui::DragFloat4("##floating_quat", &controlGc[static_cast<size_t>(offset + 3)],
                            0.005f, -1.0f, 1.0f, "%.3f")) {
                        controlGcDirty = true;
                      }
                    }
                    ImGui::PopID();
                  }
                  if (drawIconTextButton(uiIcons, TcpViewerIconKind::Robot, "Set GC", "set_selected_gc")) {
                    for (size_t i = 0; i < controlInfo.jointTypes.size(); ++i) {
                      const auto type = static_cast<raisim::Joint::Type>(controlInfo.jointTypes[i]);
                      const int32_t offset = i < controlInfo.jointGcOffsets.size() ?
                        controlInfo.jointGcOffsets[i] : -1;
                      if (offset < 0) continue;
                      if (type == raisim::Joint::Type::SPHERICAL) {
                        normalizeWxyzSlice(controlGc, static_cast<size_t>(offset));
                      } else if (type == raisim::Joint::Type::FLOATING) {
                        normalizeWxyzSlice(controlGc, static_cast<size_t>(offset + 3));
                      }
                    }
                    raisin::tcp_viewer::SimControlRequest r;
                    r.type = ClientRequestType::CR_SET_GC;
                    r.visTag = requestedTag;
                    r.gc = controlGc;
                    pendingControlRequests.push_back(std::move(r));
                    controlGcDirty = false;
                    lastStatus = "gc queued";
                  }
                  ImGui::EndDisabled();
                  ImGui::TreePop();
                }
              }
            }
          }

          ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Diagnostics")) {
          constexpr float packetColumnWidths[] = {62.0f, 62.0f, 46.0f, 34.0f, 44.0f, 44.0f, 44.0f, 44.0f};
          float packetTableWidth = ImGui::GetStyle().ScrollbarSize + ImGui::GetStyle().CellPadding.x * 16.0f;
          for (const float width : packetColumnWidths) {
            packetTableWidth += width;
          }
          const float diagnosticsContentWidth = std::max(packetTableWidth,
            iconTextButtonSize("Refresh Assets").x + style.ItemSpacing.x +
              iconTextButtonSize("Export Scene JSON").x);

          ImGui::SeparatorText("Security");
          ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + diagnosticsContentWidth);
          ImGui::TextDisabled("TCP viewer traffic is plain and unauthenticated; use SSH/VPN/TLS proxy for untrusted networks.");
          ImGui::PopTextWrapPos();
          ImGui::SeparatorText("Data Transfer");
          const double diagnosticsNowSeconds = std::chrono::duration<double>(now - steadyStart).count();
          const auto transferRates = buildTransferRateGraph(packetSamples, diagnosticsNowSeconds);
          const float peakTransferRate = maxTransferRate(transferRates);
          const float graphMax = std::max(1.0f, peakTransferRate * 1.15f);
          char transferOverlay[96];
          std::snprintf(transferOverlay, sizeof(transferOverlay), "current %.1f KiB/s | peak %.1f KiB/s",
            stats.rxKbps, peakTransferRate);
          ImGui::PlotLines("##DataTransferRate", transferRates.data(),
            static_cast<int>(transferRates.size()), 0, transferOverlay, 0.0f, graphMax,
            ImVec2(diagnosticsContentWidth, ImGui::GetFontSize() * 6.0f));

          ImGui::SeparatorText("Packets");
          ImGui::TextDisabled("Recent %zu packets | parse errors %d | RX %.1f KiB/s", packetSamples.size(),
            stats.parseErrors, stats.rxKbps);
          const float packetHeight = ImGui::GetFontSize() * 8.0f;
          if (ImGui::BeginChild("##PacketHistory", ImVec2(diagnosticsContentWidth, packetHeight), true)) {
            if (ImGui::BeginTable("##packet_table", 8,
                  ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingFixedFit)) {
              ImGui::TableSetupColumn("t", ImGuiTableColumnFlags_WidthFixed, 62.0f);
              ImGui::TableSetupColumn("bytes", ImGuiTableColumnFlags_WidthFixed, 62.0f);
              ImGui::TableSetupColumn("src", ImGuiTableColumnFlags_WidthFixed, 46.0f);
              ImGui::TableSetupColumn("ok", ImGuiTableColumnFlags_WidthFixed, 34.0f);
              ImGui::TableSetupColumn("obj", ImGuiTableColumnFlags_WidthFixed, 44.0f);
              ImGui::TableSetupColumn("vis", ImGuiTableColumnFlags_WidthFixed, 44.0f);
              ImGui::TableSetupColumn("sens", ImGuiTableColumnFlags_WidthFixed, 44.0f);
              ImGui::TableSetupColumn("miss", ImGuiTableColumnFlags_WidthFixed, 44.0f);
              ImGui::TableHeadersRow();
              for (auto it = packetSamples.rbegin(); it != packetSamples.rend(); ++it) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::Text("%.2f", it->timeSeconds);
                ImGui::TableSetColumnIndex(1); ImGui::Text("%d", it->bytes);
                ImGui::TableSetColumnIndex(2); ImGui::TextUnformatted(it->replay ? "replay" : "live");
                ImGui::TableSetColumnIndex(3); ImGui::TextUnformatted(it->parsed ? "yes" : "no");
                ImGui::TableSetColumnIndex(4); ImGui::Text("%zu", it->objects);
                ImGui::TableSetColumnIndex(5); ImGui::Text("%zu", it->visuals);
                ImGui::TableSetColumnIndex(6); ImGui::Text("%d", it->pendingSensors);
                ImGui::TableSetColumnIndex(7); ImGui::Text("%zu", it->unresolvedAssets);
              }
              ImGui::EndTable();
            }
            ImGui::EndChild();
          }

          ImGui::SeparatorText("Assets");
          if (drawIconTextButton(uiIcons, TcpViewerIconKind::Refresh, "Refresh Assets", "refresh_assets")) {
            assetDiagnostics = collectAssetDiagnostics(scene);
            assetDiagnosticsDirty = false;
            stats.unresolvedAssets = unresolvedAssetCount(scene);
          }
          ImGui::SameLine();
          if (drawIconTextButton(uiIcons, TcpViewerIconKind::Export, "Export Scene JSON", "export_scene_json")) {
            const std::filesystem::path path = options.exportScenePath.empty()
              ? timestampedDataPath(std::filesystem::path(screenshotDirBuf), "rayrai_tcp_viewer_scene", ".json")
              : options.exportScenePath;
            if (assetDiagnosticsDirty || assetDiagnostics.empty()) {
              assetDiagnostics = collectAssetDiagnostics(scene);
              assetDiagnosticsDirty = false;
            }
            exportSceneJson(path, scene, assetDiagnostics, captureStatus);
          }
          ImGui::TextDisabled("%zu assets | %zu unresolved", assetDiagnostics.size(), stats.unresolvedAssets);
          const float assetHeight = ImGui::GetFontSize() * 9.0f;
          if (ImGui::BeginChild("##AssetDiagnostics", ImVec2(diagnosticsContentWidth, assetHeight), true)) {
            if (assetDiagnostics.empty()) {
              ImGui::TextDisabled("No mesh assets observed yet");
            } else if (ImGui::BeginTable("##asset_table", 5,
                         ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingStretchProp)) {
              ImGui::TableSetupColumn("state", ImGuiTableColumnFlags_WidthFixed, 70.0f);
              ImGui::TableSetupColumn("tag", ImGuiTableColumnFlags_WidthFixed, 52.0f);
              ImGui::TableSetupColumn("object");
              ImGui::TableSetupColumn("mesh");
              ImGui::TableSetupColumn("resolved/resource");
              ImGui::TableHeadersRow();
              for (const auto& asset : assetDiagnostics) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextColored(asset.resolved ? ImVec4(0.35f, 0.85f, 0.45f, 1.0f)
                                                   : ImVec4(1.0f, 0.45f, 0.25f, 1.0f),
                  "%s", asset.resolved ? "resolved" : "missing");
                ImGui::TableSetColumnIndex(1); ImGui::Text("%u", asset.tag);
                ImGui::TableSetColumnIndex(2); ImGui::TextUnformatted(shortenPathLabel(asset.name, 32).c_str());
                ImGui::TableSetColumnIndex(3); ImGui::TextUnformatted(shortenPathLabel(asset.meshFile, 44).c_str());
                ImGui::TableSetColumnIndex(4);
                const std::string pathLabel = asset.resolved ? asset.meshPath : asset.resourceDir;
                ImGui::TextUnformatted(shortenPathLabel(pathLabel, 56).c_str());
              }
              ImGui::EndTable();
            }
            ImGui::EndChild();
          }

          ImGui::SeparatorText("Server Metadata");
          bool shownMetadata = false;
          for (const auto& server : discoveredServers) {
            if (!server.remoteBeacon || server.metadata.empty()) continue;
            shownMetadata = true;
            if (ImGui::TreeNode(formatConnectionLabel(server.endpoint).c_str())) {
              for (const auto& item : server.metadata) {
                ImGui::TextDisabled("%s: %s", item.first.c_str(), item.second.c_str());
              }
              ImGui::TreePop();
            }
          }
          if (!shownMetadata) {
            ImGui::TextDisabled("No beacon metadata received yet");
          }
          ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
      }
    }
    ImGui::End();
    if (overlayColorPushCount > 0) {
      ImGui::PopStyleColor(overlayColorPushCount);
    }
    ImGui::PopStyleVar(2);
    const bool overlayInteractionActive = ImGui::IsAnyItemActive() ||
                                          ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopup);
    if (!overlayMinimized) {
      if (overlayHovered || overlayInteractionActive || options.keepOverlayOpen) {
        overlayLastInteractionTime = now;
      } else if (now - overlayLastInteractionTime >= kOverlayAutoCollapseDelay) {
        overlayMinimized = true;
      }
    }
    overlayCollapsedHoveredLastFrame = overlayMinimized && overlayHovered;
    } // end if (overlayVisible)

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
            ImGui::TextUnformatted("Body");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(indexColor, "%d", selectedEntry->localBodyIdx);

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

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Color");
            ImGui::TableSetColumnIndex(1);
            ImGui::TextColored(metaColor, "%.2f %.2f %.2f %.2f", selectedEntry->lastColor.r,
              selectedEntry->lastColor.g, selectedEntry->lastColor.b, selectedEntry->lastColor.a);

            const auto motionIt = motionEstimates.find(visualMotionKey(selectedTag, selectedIndex));
            if (motionIt != motionEstimates.end() && motionIt->second.valid) {
              const auto& motion = motionIt->second;
              ImGui::TableNextRow();
              ImGui::TableSetColumnIndex(0);
              ImGui::TextUnformatted("Lin vel");
              ImGui::TableSetColumnIndex(1);
              ImGui::TextColored(metaColor, "%.3f %.3f %.3f", motion.linearVelocity.x,
                motion.linearVelocity.y, motion.linearVelocity.z);

              ImGui::TableNextRow();
              ImGui::TableSetColumnIndex(0);
              ImGui::TextUnformatted("Speed");
              ImGui::TableSetColumnIndex(1);
              ImGui::TextColored(metaColor, "%.3f m/s", glm::length(motion.linearVelocity));

              ImGui::TableNextRow();
              ImGui::TableSetColumnIndex(0);
              ImGui::TextUnformatted("Angular");
              ImGui::TableSetColumnIndex(1);
              ImGui::TextColored(metaColor, "%.3f rad/s", motion.angularSpeed);
            }

            if (!selectedEntry->resourceDir.empty()) {
              ImGui::TableNextRow();
              ImGui::TableSetColumnIndex(0);
              ImGui::TextUnformatted("Resource");
              ImGui::TableSetColumnIndex(1);
              ImGui::TextColored(metaColor, "%s", shortenPathLabel(selectedEntry->resourceDir, 56).c_str());
            }

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

    // ----- Local AS inspector window — styled to match the left "Raisim TCP" panel -----
    if (inspector.active && inspector.as) {
      // Mirror the left overlay's positioning/style so the inspector visually replaces it
      // (the left overlay is hidden while inspector.active).
      const ImVec2 inspectorBase(12.0f, 12.0f + menuBarHeight);
      // Both width and height auto-fit the joint list. Width is locked to whatever
      // the longest joint label needs plus the slider budget. Height grows with the
      // joint count, but is capped at displaySize so the window never pushes
      // off-screen — only then does a vertical scrollbar appear.
      const ImGuiStyle& imStyle = ImGui::GetStyle();
      const float namePixelBudget = [&]() {
        float w = ImGui::CalcTextSize("Articulated System Inspector").x;
        for (const auto& j : inspector.joints) {
          // The label row prints "<name>  [type]"; account for the type tag width too.
          const std::string row = j.name + "  [floating]";
          w = std::max(w, ImGui::CalcTextSize(row.c_str()).x);
        }
        return w;
      }();
      const float sliderBudget = 220.0f;   // room for the DragFloat/SliderFloat widgets
      const float wantedWidth = std::min(
          displaySize.x - inspectorBase.x - 20.0f,
          namePixelBudget + sliderBudget + imStyle.WindowPadding.x * 2.0f + 24.0f);
      const float maxHeight = std::max(240.0f, displaySize.y - inspectorBase.y - 16.0f);

      ImGui::SetNextWindowBgAlpha(0.5f);
      ImGui::SetNextWindowPos(inspectorBase, ImGuiCond_Always);
      // Lock width via constraints (min==max) and let height auto-fit up to maxHeight.
      // AlwaysAutoResize keeps the window snug around its content every frame; the
      // ScrollY flag turns scroll on only when content exceeds the height cap.
      ImGui::SetNextWindowSizeConstraints(ImVec2(wantedWidth, 0.0f),
                                          ImVec2(wantedWidth, maxHeight));
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));
      ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, ImGui::GetStyle().WindowBorderSize);
      const ImGuiWindowFlags inspectorFlags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNavFocus;
      if (ImGui::Begin("Raisim Inspector##Overlay", nullptr, inspectorFlags)) {
        ImGui::PushStyleColor(ImGuiCol_TextDisabled, ImVec4(0.66f, 0.71f, 0.78f, 1.0f));
        // Title row — mirrors the "Status" line in the connection panel.
        const std::filesystem::path src(inspector.sourceFile);
        if (const TcpViewerIcon* robotIcon = uiIcons.get(TcpViewerIconKind::Robot)) {
          const float lineH = ImGui::GetFontSize() + 4.0f;
          ImGui::Image(reinterpret_cast<ImTextureID>(uint64_t(robotIcon->texture)),
                       ImVec2(lineH, lineH),
                       ImVec2(0, 0), ImVec2(1, 1),
                       tcpViewerIconTint(TcpViewerIconKind::Robot, false, false));
          ImGui::SameLine();
        }
        ImGui::TextColored(ImVec4(0.55f, 0.85f, 1.0f, 1.0f), "Articulated System Inspector");
        ImGui::TextUnformatted(src.filename().string().c_str());
        ImGui::TextDisabled("%s", shortenPathLabel(inspector.sourceFile, 60).c_str());
        ImGui::TextDisabled("DoF %d  GC %zu  Joints %zu",
                            int(inspector.as->getDOF()),
                            inspector.gc.size(),
                            inspector.joints.size());
        if (!lastStatus.empty()) {
          ImGui::TextDisabled("Status: %s", lastStatus.c_str());
        }
        ImGui::Separator();
        if (drawIconTextButton(uiIcons, TcpViewerIconKind::Reset, "Reset pose", "reset_pose")) {
          std::fill(inspector.gc.begin(), inspector.gc.end(), 0.0);
          // For multi-DoF joints with a quaternion component, set qw=1 (identity rotation).
          for (const auto& j : inspector.joints) {
            if (j.type == raisim::Joint::Type::FLOATING && j.gcOffset + 3 < int(inspector.gc.size())) {
              inspector.gc[j.gcOffset + 3] = 1.0; // qw for floating: pos(3) + quat(wxyz)
            } else if (j.type == raisim::Joint::Type::SPHERICAL &&
                       j.gcOffset < int(inspector.gc.size())) {
              inspector.gc[j.gcOffset] = 1.0; // qw for spherical
            }
          }
          Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd>(
              inspector.gc.data(), Eigen::Index(inspector.gc.size()));
          inspector.as->setGeneralizedCoordinate(q);
        }
        ImGui::SameLine();
        if (drawIconTextButton(uiIcons, TcpViewerIconKind::Exit, "Close inspector",
                               "close_inspector")) {
          closeInspector();
        } else {
          ImGui::Separator();
          bool gcChanged = false;
          inspector.dragging = false;
          for (size_t ji = 0; ji < inspector.joints.size(); ++ji) {
            const auto& j = inspector.joints[ji];
            ImGui::PushID(int(ji));
            const char* typeLabel = "?";
            switch (j.type) {
              case raisim::Joint::Type::FIXED: typeLabel = "fixed"; break;
              case raisim::Joint::Type::REVOLUTE: typeLabel = "rev"; break;
              case raisim::Joint::Type::PRISMATIC: typeLabel = "pris"; break;
              case raisim::Joint::Type::SPHERICAL: typeLabel = "sph  quat"; break;
              case raisim::Joint::Type::FLOATING: typeLabel = "float  pos  quat"; break;
            }
            ImGui::Text("%s  [%s]", j.name.c_str(), typeLabel);
            if (j.type == raisim::Joint::Type::REVOLUTE ||
                j.type == raisim::Joint::Type::PRISMATIC) {
              float v = float(inspector.gc[j.gcOffset]);
              const bool useSlider = j.hasLimits;
              bool changed = false;
              if (useSlider) {
                changed = ImGui::SliderFloat("##v", &v, float(j.minLimit), float(j.maxLimit),
                                             "%.4f");
              } else {
                changed = ImGui::DragFloat("##v", &v, 0.01f, 0.0f, 0.0f, "%.4f");
              }
              if (changed) {
                inspector.gc[j.gcOffset] = v;
                gcChanged = true;
              }
              if (ImGui::IsItemActive()) inspector.dragging = true;
            } else if (j.type == raisim::Joint::Type::FLOATING) {
              float pos[3] = {float(inspector.gc[j.gcOffset]),
                              float(inspector.gc[j.gcOffset + 1]),
                              float(inspector.gc[j.gcOffset + 2])};
              if (ImGui::DragFloat3("##float_pos", pos, 0.05f)) {
                inspector.gc[j.gcOffset] = pos[0];
                inspector.gc[j.gcOffset + 1] = pos[1];
                inspector.gc[j.gcOffset + 2] = pos[2];
                gcChanged = true;
              }
              if (ImGui::IsItemActive()) inspector.dragging = true;
              float quat[4] = {float(inspector.gc[j.gcOffset + 3]),
                               float(inspector.gc[j.gcOffset + 4]),
                               float(inspector.gc[j.gcOffset + 5]),
                               float(inspector.gc[j.gcOffset + 6])};
              if (ImGui::DragFloat4("##float_quat", quat, 0.01f, -1.0f, 1.0f)) {
                const double n = std::sqrt(double(quat[0]) * quat[0] + double(quat[1]) * quat[1] +
                                           double(quat[2]) * quat[2] + double(quat[3]) * quat[3]);
                const double inv = n > 1e-6 ? 1.0 / n : 1.0;
                inspector.gc[j.gcOffset + 3] = quat[0] * inv;
                inspector.gc[j.gcOffset + 4] = quat[1] * inv;
                inspector.gc[j.gcOffset + 5] = quat[2] * inv;
                inspector.gc[j.gcOffset + 6] = quat[3] * inv;
                gcChanged = true;
              }
              if (ImGui::IsItemActive()) inspector.dragging = true;
            } else if (j.type == raisim::Joint::Type::SPHERICAL) {
              float quat[4] = {float(inspector.gc[j.gcOffset]),
                               float(inspector.gc[j.gcOffset + 1]),
                               float(inspector.gc[j.gcOffset + 2]),
                               float(inspector.gc[j.gcOffset + 3])};
              if (ImGui::DragFloat4("##sph_quat", quat, 0.01f, -1.0f, 1.0f)) {
                const double n = std::sqrt(double(quat[0]) * quat[0] + double(quat[1]) * quat[1] +
                                           double(quat[2]) * quat[2] + double(quat[3]) * quat[3]);
                const double inv = n > 1e-6 ? 1.0 / n : 1.0;
                inspector.gc[j.gcOffset] = quat[0] * inv;
                inspector.gc[j.gcOffset + 1] = quat[1] * inv;
                inspector.gc[j.gcOffset + 2] = quat[2] * inv;
                inspector.gc[j.gcOffset + 3] = quat[3] * inv;
                gcChanged = true;
              }
              if (ImGui::IsItemActive()) inspector.dragging = true;
            }
            ImGui::PopID();
          }
          if (gcChanged) {
            Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd>(
                inspector.gc.data(), Eigen::Index(inspector.gc.size()));
            inspector.as->setGeneralizedCoordinate(q);
          }
        }
        ImGui::PopStyleColor();
      }
      ImGui::End();
      ImGui::PopStyleVar(2);
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
  }

  if (settingsDirty || settingsSavePending) {
    settings.recentConnections = recentConnections;
    settings.resourceDirs = resourceDirs;
    saveViewerSettings(settings);
    settingsDirty = false;
    settingsSavePending = false;
  }

  raisimLogo.release();
  uiIcons.release();
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
#endif  // RAYRAI_TCP_VIEWER_NO_MAIN
