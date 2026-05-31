// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rayrai_tcp_viewer_hint.hpp"

#include <array>
#include <stdexcept>
#include <string>

namespace {

struct UsdRobot {
  const char* name;
  const char* path;
  std::array<double, 3> basePosition;
};

const std::array<UsdRobot, 3> robotAssets = {{
    {"iRobot Create 3",
     "/isaac/Robots/iRobot/Create3/create_3.usd",
     {-1.4, 0.0, 0.18}},
    {"AWS Robomaker Jetbot",
     "/isaac/Robots/NVIDIA/Robomaker/aws_robomaker_jetbot.usd",
     {0.0, 0.0, 0.12}},
    {"Isaac Sim Ant",
     "/isaac/Robots/IsaacSim/Ant/ant.usd",
     {1.6, 0.0, 0.55}},
}};

}  // namespace

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  const auto rscPath = (binaryPath.getDirectory() + "/rsc").getString();
  raisim::World::setActivationKey(rscPath + "/activation.raisim");

  raisim::World world;
  world.setTimeStep(1.0 / 500.0);

  raisim::ArticulatedSystem* firstRobot = nullptr;
  for (const auto& robotAsset : robotAssets) {
    auto* robot = world.addUsdArticulatedSystem(rscPath + robotAsset.path);
    if (robot == nullptr) {
      throw std::runtime_error(std::string("Failed to import ") + robotAsset.name +
                               " from " + robotAsset.path);
    }
    if (robot->getCollisionBodies().empty()) {
      throw std::runtime_error(std::string("USD robot has no supported collision bodies: ") +
                               robotAsset.name);
    }

    robot->setBasePos({robotAsset.basePosition[0],
                       robotAsset.basePosition[1],
                       robotAsset.basePosition[2]});
    robot->setBaseOrientation(raisim::Mat<3, 3>::getIdentity());
    for (auto& collisionBody : robot->getCollisionBodies()) {
      collisionBody.setMaterial("nvidia_usd_robot");
    }

    if (firstRobot == nullptr) {
      firstRobot = robot;
    }
  }
  if (world.getObjectCount() < robotAssets.size()) {
    throw std::runtime_error("Expected 3 USD robots, but imported " +
                             std::to_string(world.getObjectCount()));
  }

  auto* ground = world.addGround(0.0, "ground");
  ground->setAppearance("checkerboard");
  world.setMaterialPairProp("ground", "nvidia_usd_robot", 0.8, 0.0, 0.001);

  raisim::RaisimServer server(&world);
  server.launchServer();
  raisim_examples::warnIfNoClientConnected(server);
  server.focusOn(firstRobot);
  server.setCameraPositionAndLookAt({3.0, -3.4, 1.5}, {0.1, 0.0, 0.3});

  for (int i = 0; i < 2000000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))
    server.integrateWorldThreadSafe();
  }

  server.killServer();
  return 0;
}
