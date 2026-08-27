// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rayrai_tcp_viewer_hint.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  raisim::World world;
  world.setTimeStep(0.002);
  world.setERP(0.0, 0.0);

  /// blocky height map: 500x500 samples over 40m x 40m (0.08m sample spacing).
  /// Every 5x5 patch of samples shares one height, giving 100x100 flat tiles of
  /// 0.4m x 0.4m. Tile heights are sampled uniformly from -0.15m to 0.15m.
  constexpr size_t samples = 500;
  constexpr double mapSize = 40.0;
  constexpr size_t blockSize = 5;
  constexpr size_t blocks = samples / blockSize;
  constexpr double heightRange = 0.15;

  std::mt19937 generator(2024);
  std::uniform_real_distribution<double> heightDistribution(-heightRange,
                                                            heightRange);

  std::vector<double> blockHeights(blocks * blocks);
  for (auto& height : blockHeights) height = heightDistribution(generator);

  std::vector<double> height(samples * samples);
  for (size_t yIndex = 0; yIndex < samples; ++yIndex) {
    for (size_t xIndex = 0; xIndex < samples; ++xIndex) {
      const size_t block =
          (yIndex / blockSize) * blocks + (xIndex / blockSize);
      height[yIndex * samples + xIndex] = blockHeights[block];
    }
  }

  auto* heightMap = world.addHeightMap(samples, samples, mapSize, mapSize, 0.0,
                                       0.0, height);
  heightMap->setName("blocky_terrain");
  heightMap->setAppearance("soil1");

  /// monkey mesh used for one fifth of the dropped bodies
  const std::string sep = raisim::Path::separator();
  std::string monkeyPath = binaryPath.getDirectory().getString() + sep + "rsc" +
                           sep + "monkey" + sep + "monkey.obj";
  raisim::Path::replaceAntiSeparatorWithSeparator(monkeyPath);

  /// drop 400 bodies (20x20 grid) above the tiles: box, sphere, capsule,
  /// cylinder, and monkey mesh, 80 of each
  constexpr int gridSize = 20;
  constexpr double spacing = 0.9;
  constexpr double dropHeight = 3.0;

  int meshCount = 0;
  for (int row = 0; row < gridSize; ++row) {
    for (int column = 0; column < gridSize; ++column) {
      raisim::SingleBodyObject* object = nullptr;
      const int shape = (row + 2 * column) % 5;

      switch (shape) {
        case 0:
          object = world.addBox(0.35, 0.35, 0.35, 1.0);
          object->setAppearance("blue");
          break;
        case 1:
          object = world.addSphere(0.2, 1.0);
          object->setAppearance("red");
          break;
        case 2:
          object = world.addCapsule(0.15, 0.35, 1.0);
          object->setAppearance("green");
          break;
        case 3:
          object = world.addCylinder(0.2, 0.35, 1.0);
          object->setAppearance("0.8, 0.65, 0.2, 1.0");
          break;
        default: {
          /// a convex hull keeps 80 mesh bodies cheap to load and to collide
          const double scale = 0.13 + 0.01 * (meshCount % 4);
          object = world.addMesh(monkeyPath, 1.0, scale, "",
                                 raisim::MeshCollisionMode::CONVEX_HULL);
          object->setAppearance("0.75, 0.45, 0.25, 1.0");

          /// tilt each monkey differently so that they do not land identically
          const double angle = 0.37 * meshCount;
          object->setOrientation(std::cos(0.5 * angle),
                                 std::sin(0.5 * angle) * std::cos(0.7 * meshCount),
                                 std::sin(0.5 * angle) * std::sin(0.7 * meshCount),
                                 0.0);
          ++meshCount;
          break;
        }
      }

      const double x = spacing * (column - 0.5 * (gridSize - 1));
      const double y = spacing * (row - 0.5 * (gridSize - 1));
      const double z = heightMap->getHeight(x, y) + dropHeight +
                       0.05 * ((7 * row + 3 * column) % 5);

      object->setName((shape == 4 ? "monkey_" : "primitive_") +
                      std::to_string(row) + "_" + std::to_string(column));
      object->setPosition(x, y, z);
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
