// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rayrai_tcp_viewer_hint.hpp"

#include <array>
#include <iostream>
#include <string>

namespace {

struct AssetSpec {
  const char* key;
  const char* label;
  const char* usdPath;
  double baseHeight;
  std::array<double, 3> cameraPosition;
  std::array<double, 3> cameraTarget;
};

const std::array<AssetSpec, 3> kAssets = {{
    {"create3",
     "iRobot Create 3",
     "/isaac/Robots/iRobot/Create3/create_3.usd",
     0.18,
     {1.2, -1.0, 0.65},
     {0.0, 0.0, 0.18}},
    {"jetbot",
     "AWS Robomaker Jetbot",
     "/isaac/Robots/NVIDIA/Robomaker/aws_robomaker_jetbot.usd",
     0.12,
     {0.9, -0.9, 0.55},
     {0.0, 0.0, 0.15}},
    {"ant",
     "Isaac Sim Ant",
     "/isaac/Robots/IsaacSim/Ant/ant.usd",
     0.55,
     {2.0, -2.0, 1.1},
     {0.0, 0.0, 0.45}},
}};

bool isArgument(const char* value, const char* expected) {
  while (*value != '\0' && *expected != '\0' && *value == *expected) {
    ++value;
    ++expected;
  }
  return *value == '\0' && *expected == '\0';
}

bool startsWith(const char* value, const char* prefix) {
  while (*prefix != '\0') {
    if (*value != *prefix)
      return false;
    ++value;
    ++prefix;
  }
  return true;
}

bool hasArgument(int argc, char* argv[], const char* expected) {
  for (int i = 1; i < argc; i++) {
    if (isArgument(argv[i], expected))
      return true;
  }
  return false;
}

void printAssetList() {
  std::cout << "Available NVIDIA USD assets:\n";
  for (const auto& asset : kAssets)
    std::cout << "  " << asset.key << " - " << asset.label << "\n";
}

const AssetSpec* findAsset(const std::string& key) {
  for (const auto& asset : kAssets) {
    if (key == asset.key)
      return &asset;
  }
  return nullptr;
}

const AssetSpec* parseAsset(int argc, char* argv[]) {
  for (int i = 1; i < argc; i++) {
    if (isArgument(argv[i], "--asset") && i + 1 < argc)
      return findAsset(argv[i + 1]);

    constexpr const char* prefix = "--asset=";
    if (startsWith(argv[i], prefix))
      return findAsset(argv[i] + std::string(prefix).size());
  }

  return &kAssets.front();
}

raisim::ArticulatedSystem* findFirstArticulatedSystem(raisim::World& world) {
  for (auto* object : world.getObjList()) {
    if (auto* articulatedSystem = dynamic_cast<raisim::ArticulatedSystem*>(object))
      return articulatedSystem;
  }
  return nullptr;
}

void placeRobot(raisim::ArticulatedSystem& robot, const AssetSpec& asset) {
  robot.setBasePos({0.0, 0.0, asset.baseHeight});
  robot.setBaseOrientation(raisim::Mat<3, 3>::getIdentity());
  for (auto& collisionBody : robot.getCollisionBodies())
    collisionBody.setMaterial("nvidia_usd_robot");
}

}  // namespace

int main(int argc, char* argv[]) {
  if (hasArgument(argc, argv, "--list-assets")) {
    printAssetList();
    return 0;
  }

  const auto* asset = parseAsset(argc, argv);
  if (asset == nullptr) {
    std::cerr << "Unknown --asset value.\n";
    printAssetList();
    return 1;
  }

  const bool headlessTest = hasArgument(argc, argv, "--headless-test");
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  const auto rscPath = (binaryPath.getDirectory() + "/rsc").getString();
  raisim::World::setActivationKey(rscPath + "/activation.raisim");

  raisim::World world(rscPath + asset->usdPath);
  world.setTimeStep(1.0 / 500.0);

  auto* robot = findFirstArticulatedSystem(world);
  if (robot == nullptr) {
    std::cerr << asset->label << " USD did not import as a RaiSim articulated system.\n";
    return 2;
  }

  if (robot->getCollisionBodies().empty()) {
    std::cerr << asset->label << " USD imported without supported collision bodies.\n";
    return 3;
  }

  placeRobot(*robot, *asset);

  auto* ground = world.addGround(0.0, "ground");
  ground->setAppearance("checkerboard");
  world.setMaterialPairProp("ground", "nvidia_usd_robot", 0.8, 0.0, 0.001);

  if (headlessTest) {
    for (int i = 0; i < 8; i++)
      world.integrate();
    std::cout << asset->key
              << " dof=" << robot->getDOF()
              << " gc=" << robot->getGeneralizedCoordinateDim()
              << " bodies=" << robot->getBodyNames().size()
              << " collisions=" << robot->getCollisionBodies().size()
              << "\n";
    return 0;
  }

  raisim::RaisimServer server(&world);
  server.launchServer();
  raisim_examples::warnIfNoClientConnected(server);
  server.focusOn(robot);
  server.setCameraPositionAndLookAt(
      {asset->cameraPosition[0], asset->cameraPosition[1], asset->cameraPosition[2]},
      {asset->cameraTarget[0], asset->cameraTarget[1], asset->cameraTarget[2]});

  for (int i = 0; i < 2000000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))
    server.integrateWorldThreadSafe();
  }

  server.killServer();
  return 0;
}
