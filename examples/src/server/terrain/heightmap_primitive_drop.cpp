// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include <string>

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rayrai_tcp_viewer_hint.hpp"

int main() {
  raisim::World world;
  world.setTimeStep(0.002);

  raisim::TerrainProperties terrainProperties;
  terrainProperties.frequency = 0.18;
  terrainProperties.zScale = 2.5;
  terrainProperties.xSize = 24.0;
  terrainProperties.ySize = 24.0;
  terrainProperties.xSamples = 80;
  terrainProperties.ySamples = 80;
  terrainProperties.fractalOctaves = 4;
  terrainProperties.fractalLacunarity = 2.0;
  terrainProperties.fractalGain = 0.3;
  terrainProperties.heightOffset = -0.5;

  auto* heightMap = world.addHeightMap(0.0, 0.0, terrainProperties);
  heightMap->setName("terrain");
  heightMap->setAppearance("soil1");

  constexpr int gridSize = 12;
  constexpr int layers = 3;
  constexpr double spacing = 1.2;
  constexpr double initialHeight = 8.0;

  for (int layer = 0; layer < layers; ++layer) {
    for (int row = 0; row < gridSize; ++row) {
      for (int column = 0; column < gridSize; ++column) {
        raisim::SingleBodyObject* object = nullptr;
        const int shape = (layer + row + column) % 4;

        switch (shape) {
          case 0:
            object = world.addBox(0.55, 0.55, 0.55, 1.0);
            object->setAppearance("blue");
            break;
          case 1:
            object = world.addSphere(0.3, 1.0);
            object->setAppearance("red");
            break;
          case 2:
            object = world.addCapsule(0.25, 0.5, 1.0);
            object->setAppearance("green");
            break;
          default:
            object = world.addCylinder(0.3, 0.55, 1.0);
            object->setAppearance("0.8, 0.65, 0.2, 1.0");
            break;
        }

        const double x = spacing * (column - 0.5 * (gridSize - 1));
        const double y = spacing * (row - 0.5 * (gridSize - 1));
        const double heightJitter = 0.08 * ((7 * row + 3 * column) % 5);
        const double z = initialHeight + 1.1 * layer + heightJitter;

        object->setName("primitive_" + std::to_string(layer) + "_" +
                        std::to_string(row) + "_" + std::to_string(column));
        object->setPosition(x, y, z);
      }
    }
  }

  raisim::RaisimServer server(&world);
  server.launchServer();

  raisim_examples::warnIfNoClientConnected(server);
  while (true) {
    RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
