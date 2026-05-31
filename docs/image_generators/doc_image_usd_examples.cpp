// Renders the USD examples used by the OpenUSD documentation.
//
// The scenes use the same assets as examples/src/server/robots:
// - nvidia_usd_robots: three Isaac Sim robot USD articulations.
// - shadow_hand_usd_cube: ShadowHand loaded through World(.usd) with one
//   native RaiSim cube above the hand.

#include "doc_image_common.hpp"

#include <array>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr int kWidth = 1280;
constexpr int kHeight = 720;

struct UsdRobot {
  const char* name;
  const char* path;
  std::array<double, 3> basePosition;
};

const std::array<UsdRobot, 3> kRobotAssets = {{
    {"iRobot Create 3",
     "isaac/Robots/iRobot/Create3/create_3.usd",
     {-0.65, -0.95, 0.18}},
    {"AWS Robomaker Jetbot",
     "isaac/Robots/NVIDIA/Robomaker/aws_robomaker_jetbot.usd",
     {0.35, -0.9, 0.12}},
    {"Isaac Sim Ant",
     "isaac/Robots/IsaacSim/Ant/ant.usd",
     {1.45, 0.1, 0.55}},
}};

const std::array<raisim::Vec<4>, 3> kRobotColors = {{
    [] {
      raisim::Vec<4> color;
      color << 0.78, 0.80, 0.83, 1.0;
      return color;
    }(),
    [] {
      raisim::Vec<4> color;
      color << 0.18, 0.50, 0.95, 1.0;
      return color;
    }(),
    [] {
      raisim::Vec<4> color;
      color << 0.95, 0.38, 0.05, 1.0;
      return color;
    }(),
}};

std::filesystem::path canonicalIfExists(const std::filesystem::path& path) {
  std::error_code ec;
  const auto absolute = std::filesystem::absolute(path, ec);
  const auto normalized = ec ? path : std::filesystem::weakly_canonical(absolute, ec);
  return ec ? absolute.lexically_normal() : normalized;
}

std::filesystem::path findRscDir() {
  const auto sourceRoot =
      std::filesystem::path(__FILE__).parent_path() / ".." / "..";
  const std::vector<std::filesystem::path> candidates = {
      sourceRoot / "rsc",
      std::filesystem::current_path() / ".." / ".." / ".." / "rsc",
      std::filesystem::current_path() / ".." / ".." / "rsc",
      std::filesystem::current_path() / "rsc",
  };

  for (const auto& candidate : candidates) {
    const auto rscDir = canonicalIfExists(candidate);
    if (std::filesystem::exists(rscDir / "isaac" / "Robots")) {
      return rscDir;
    }
  }

  std::fprintf(stderr, "doc_image: could not locate rsc directory for USD examples\n");
  doc_image::finishAndExit(1);
}

void configureRenderer(raisin::RayraiWindow& viewer) {
  const auto preset = raisin::RayraiWindow::RenderQualityPreset::Balanced;
  auto quality = raisin::RayraiWindow::defaultRenderQualitySettings(preset);
  quality.colorMode = raisin::ViewerColorMode::FastLinear;
  quality.pbrToneMapping = false;
  doc_image::applyCommonSceneOptions(quality, preset);
  quality.mainLightDirection = glm::normalize(glm::vec3(-0.35f, 0.45f, -1.0f));
  quality.mainLightDiffuse = glm::vec3(1.0f, 0.96f, 0.9f) * 1.5f;
  viewer.setRenderQualitySettings(quality);
}

void tintVisuals(raisim::ArticulatedSystem* system,
                 const raisim::Vec<4>& color) {
  for (auto& visual : system->getVisOb()) {
    visual.color = color;
    visual.material.clear();
  }
  for (auto& visual : system->getVisColOb()) {
    visual.color = color;
    visual.material.clear();
  }
}

void captureNvidiaRobots(const std::filesystem::path& outputDir,
                         const std::filesystem::path& rscDir) {
  auto world = std::make_shared<raisim::World>();
  world->setTimeStep(1.0 / 500.0);

  for (std::size_t i = 0; i < kRobotAssets.size(); ++i) {
    const auto& robotAsset = kRobotAssets[i];
    const auto usdPath = rscDir / robotAsset.path;
    auto* robot = world->addUsdArticulatedSystem(usdPath.string());
    if (robot == nullptr) {
      throw std::runtime_error(std::string("Failed to import ") +
                               robotAsset.name + " from " + usdPath.string());
    }
    if (robot->getCollisionBodies().empty()) {
      throw std::runtime_error(std::string("USD robot has no supported collision bodies: ") +
                               robotAsset.name);
    }
    robot->setBasePos({robotAsset.basePosition[0],
                       robotAsset.basePosition[1],
                       robotAsset.basePosition[2]});
    robot->setBaseOrientation(raisim::Mat<3, 3>::getIdentity());
    tintVisuals(robot, kRobotColors[i]);
  }

  if (world->getObjectCount() < kRobotAssets.size()) {
    throw std::runtime_error("Expected 3 USD robots, but imported " +
                             std::to_string(world->getObjectCount()));
  }

  auto* ground = world->addGround(0.0, "ground");
  ground->setAppearance("checkerboard");

  raisin::RayraiWindow viewer(world, kWidth, kHeight);
  viewer.setShowCollisionBodies(true);
  configureRenderer(viewer);
  doc_image::setCameraLookAt(viewer.getCamera(),
                             glm::vec3(2.3f, -3.2f, 1.35f),
                             glm::vec3(0.35f, -0.55f, 0.34f),
                             /*horizontalFovDeg=*/45.0f);

  raisin::RayraiWindow::RenderOverrides overrides;
  overrides.doShadows = true;

  const auto path = outputDir / "rayrai_usd_nvidia_robots.png";
  if (!doc_image::captureScene(viewer, kWidth, kHeight, path, 2, overrides))
    doc_image::finishAndExit(1);
  std::printf("doc_image: wrote %s\n", path.string().c_str());
}

void captureShadowHandCube(const std::filesystem::path& outputDir,
                           const std::filesystem::path& rscDir) {
  const auto usdPath =
      rscDir / "isaac/Robots/ShadowRobot/ShadowHand/shadow_hand.usd";
  auto world = std::make_shared<raisim::World>(usdPath.string());
  world->setTimeStep(1.0 / 500.0);

  auto* hand =
      dynamic_cast<raisim::ArticulatedSystem*>(world->getObject("shadow_hand"));
  if (hand == nullptr) {
    throw std::runtime_error("Failed to import ShadowHand from " + usdPath.string());
  }
  if (hand->getCollisionBodies().empty()) {
    throw std::runtime_error("Imported ShadowHand has no collision bodies");
  }
  if (hand->getVisOb().empty()) {
    throw std::runtime_error("Imported ShadowHand has no visual meshes");
  }

  hand->setBasePos({0.0, 0.0, 0.5});
  hand->setBaseOrientation(raisim::Mat<3, 3>::getIdentity());
  raisim::Vec<4> handColor;
  handColor << 0.86, 0.70, 0.48, 1.0;
  tintVisuals(hand, handColor);

  auto* ground = world->addGround(0.0, "ground");
  ground->setAppearance("checkerboard");

  auto* cube = world->addBox(0.065, 0.065, 0.065, 0.18, "cube");
  cube->setName("dexterous_cube");
  cube->setPosition(0.0, -0.39, 1.05);
  cube->setAppearance("0.18,0.55,0.95,1");

  raisin::RayraiWindow viewer(world, kWidth, kHeight);
  viewer.setShowCollisionBodies(true);
  configureRenderer(viewer);
  doc_image::setCameraLookAt(viewer.getCamera(),
                             glm::vec3(0.02f, -0.48f, 1.85f),
                             glm::vec3(0.0f, -0.26f, 0.50f),
                             /*horizontalFovDeg=*/28.0f);

  raisin::RayraiWindow::RenderOverrides overrides;
  overrides.doShadows = true;

  const auto path = outputDir / "rayrai_usd_shadow_hand_cube.png";
  if (!doc_image::captureScene(viewer, kWidth, kHeight, path, 2, overrides))
    doc_image::finishAndExit(1);
  std::printf("doc_image: wrote %s\n", path.string().c_str());
}
}  // namespace

int main(int argc, char** argv) {
  const auto outputDir = doc_image::resolveOutputDir(argc, argv);
  doc_image::OffscreenContext gl;
  if (!gl.init("doc_image_usd_examples")) doc_image::finishAndExit(1);

  const auto rscDir = findRscDir();
  const auto activationKey = rscDir / "activation.raisim";
  if (std::filesystem::exists(activationKey))
    raisim::World::setActivationKey(activationKey.string());

  try {
    captureNvidiaRobots(outputDir, rscDir);
    captureShadowHandCube(outputDir, rscDir);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "doc_image: %s\n", e.what());
    doc_image::finishAndExit(1);
  }

  doc_image::finishAndExit(0);
}
