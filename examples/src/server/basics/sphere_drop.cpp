// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rayrai_tcp_viewer_hint.hpp"

#include <string>
#include <vector>

int main() {
  raisim::World world;
  world.setTimeStep(0.002);

  auto ground = world.addGround();
  ground->setName("ground");
  ground->setAppearance("grid");

  constexpr int spheresPerAxis = 10;
  constexpr double radius = 0.08;
  constexpr double mass = 0.1;
  constexpr double spacing = radius * 2.6;
  constexpr double firstHeight = 1.5;

  std::vector<raisim::Sphere*> spheres;
  spheres.reserve(spheresPerAxis * spheresPerAxis * spheresPerAxis);

  const double originOffset = -0.5 * spacing * static_cast<double>(spheresPerAxis - 1);

  for (int i = 0; i < spheresPerAxis; i++) {
    for (int j = 0; j < spheresPerAxis; j++) {
      for (int k = 0; k < spheresPerAxis; k++) {
        auto* sphere = world.addSphere(radius, mass);
        sphere->setName("sphere_" + std::to_string(spheres.size()));
        sphere->setAppearance((i + j + k) % 2 == 0 ? "red" : "blue");

        const double layerOffset = (k % 2 == 0) ? 0.0 : 0.25 * radius;
        sphere->setPosition(
            originOffset + spacing * i + layerOffset,
            originOffset + spacing * j - layerOffset,
            firstHeight + spacing * k);
        spheres.push_back(sphere);
      }
    }
  }

  raisim::RaisimServer server(&world);
  server.launchServer();
  raisim_examples::warnIfNoClientConnected(server);
  server.setCameraPositionAndLookAt({4.0, -5.0, 3.0}, {0.0, 0.0, 1.2});

  while (true) {
    RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
