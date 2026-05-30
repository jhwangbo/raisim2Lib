// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rayrai_tcp_viewer_hint.hpp"

#include <string>

namespace {

bool isArgument(const char* value, const char* expected) {
  while (*value != '\0' && *expected != '\0' && *value == *expected) {
    ++value;
    ++expected;
  }
  return *value == '\0' && *expected == '\0';
}

bool hasArgument(int argc, char* argv[], const char* expected) {
  for (int i = 1; i < argc; i++) {
    if (isArgument(argv[i], expected))
      return true;
  }
  return false;
}

raisim::Box* addStaticProxyBox(raisim::World& world, const char* name,
                               double xLength, double yLength, double zLength,
                               double x, double y, double z) {
  auto* box = world.addBox(xLength, yLength, zLength, 1.0, "shadow_hand");
  box->setName(name);
  box->setBodyType(raisim::BodyType::STATIC);
  box->setPosition(x, y, z);
  box->setAppearance("0.72,0.72,0.76,1.0");
  return box;
}

void addTcpSafeShadowHandProxy(raisim::World& world) {
  addStaticProxyBox(world, "shadow_hand_proxy_forearm", 0.16, 0.30, 0.10,
                    0.00, -0.16, 0.50);
  addStaticProxyBox(world, "shadow_hand_proxy_palm", 0.22, 0.10, 0.12,
                    0.00, -0.33, 0.57);

  addStaticProxyBox(world, "shadow_hand_proxy_ff", 0.028, 0.17, 0.034,
                    -0.078, -0.43, 0.65);
  addStaticProxyBox(world, "shadow_hand_proxy_mf", 0.028, 0.18, 0.034,
                    -0.026, -0.44, 0.66);
  addStaticProxyBox(world, "shadow_hand_proxy_rf", 0.028, 0.17, 0.034,
                    0.026, -0.43, 0.65);
  addStaticProxyBox(world, "shadow_hand_proxy_lf", 0.028, 0.15, 0.034,
                    0.078, -0.42, 0.64);
  addStaticProxyBox(world, "shadow_hand_proxy_thumb", 0.13, 0.035, 0.040,
                    0.13, -0.36, 0.56);
}

void syncVisualToCube(const raisim::Box* cube, raisim::Visuals* visual) {
  visual->setPosition(cube->getPosition());
  visual->setOrientation(cube->getQuaternion());
}

}  // namespace

int main(int argc, char* argv[]) {
  const bool headlessTest = hasArgument(argc, argv, "--headless-test");
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  const auto rscPath = (binaryPath.getDirectory() + "/rsc").getString();
  raisim::World::setActivationKey(rscPath + "/activation.raisim");

  const auto shadowHandUsd =
      rscPath + "/isaac/Robots/ShadowRobot/ShadowHand/shadow_hand.usd";
  const auto dexCubeMesh =
      rscPath + "/isaac/Props/Blocks/DexCube/dex_cube_textured.obj";

  raisim::World world(shadowHandUsd);
  world.setTimeStep(1.0 / 500.0);

  auto* ground = world.addGround(0.0, "ground");
  ground->setAppearance("checkerboard");

  auto* hand = dynamic_cast<raisim::ArticulatedSystem*>(world.getObject("shadow_hand"));
  if (hand) {
    hand->setBasePos({0.0, 0.0, 0.5});
    hand->setBaseOrientation(raisim::Mat<3, 3>::getIdentity());
    for (auto& collisionBody : hand->getCollisionBodies())
      collisionBody.setMaterial("shadow_hand");
    if (hand->getCollisionBodies().empty() && !headlessTest)
      addTcpSafeShadowHandProxy(world);
  }

  constexpr double cubeSize = 0.065;
  auto* cube = world.addBox(cubeSize, cubeSize, cubeSize, 0.18, "cube");
  cube->setName("dexterous_cube");
  cube->setPosition(0.0, -0.39, 0.6);
  cube->setAppearance("0.18,0.55,0.95,0.0");

  world.setMaterialPairProp("ground", "cube", 0.8, 0.0, 0.001);
  world.setMaterialPairProp("shadow_hand", "cube", 1.1, 0.0, 0.001);

  if (headlessTest) {
    for (int i = 0; i < 8; i++)
      world.integrate();
    return 0;
  }

  raisim::RaisimServer server(&world);
  auto* cubeVisual = server.addVisualMesh("dexterous_cube_textured",
                                          dexCubeMesh,
                                          raisim::Vec<3>{cubeSize, cubeSize, cubeSize});
  syncVisualToCube(cube, cubeVisual);

  auto* goal = server.addVisualBox("goal_cube", cubeSize, cubeSize, cubeSize,
                                   0.2, 1.0, 0.45, 0.35);
  goal->setPosition(0.0, -0.39, 0.78);

  server.launchServer();
  raisim_examples::warnIfNoClientConnected(server);
  server.focusOn(cube);
  server.setCameraPositionAndLookAt({1.1, -1.15, 0.95}, {0.0, -0.25, 0.55});

  for (int i = 0; i < 2000000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))
    server.integrateWorldThreadSafe();
    server.lockVisualizationServerMutex();
    syncVisualToCube(cube, cubeVisual);
    server.unlockVisualizationServerMutex();
  }

  server.killServer();
  return 0;
}
