// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include <cmath>
#include <string>
#include <vector>

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rayrai_tcp_viewer_hint.hpp"

int main(int argc, char* argv[]) {

    auto binaryPath = raisim::Path::setFromArgv(argv[0]);
    (void)binaryPath;

    constexpr size_t X_SAMPLES = 300;
    constexpr size_t Y_SAMPLES = 300;
    constexpr double X_SIZE = 45.0;
    constexpr double Y_SIZE = 45.0;

    raisim::World world;
    world.setTimeStep(0.0025);

    std::vector<double> terrainHeights(X_SAMPLES * Y_SAMPLES, 0.0);
    std::vector<raisim::ColorRGB> terrainColors(X_SAMPLES * Y_SAMPLES, {0, 0, 0});

    for (size_t i = 0; i < X_SAMPLES; i++) {
      for (size_t j = 0; j < Y_SAMPLES; j++) {
        const double x = static_cast<double>(i) / X_SAMPLES;
        const double y = static_cast<double>(j) / Y_SAMPLES;
        const double h = 4.0 * std::sin(3.0 * x + 0.8) + 3.0 * std::cos(5.0 * y + 1.1) +
                         2.2 * std::sin(2.0 * (x + y) + 0.2) +
                         1.4 * std::sin(11.0 * x + 2.3) * std::cos(9.0 * y + 0.7) +
                         0.8 * std::sin(23.0 * x + 1.9) * std::sin(19.0 * y + 3.1) +
                         0.5 * std::cos(37.0 * (x - y) + 0.5) +
                         0.3 * std::sin(61.0 * x + 0.9) * std::cos(53.0 * y + 2.1);

        terrainHeights[i * Y_SAMPLES + j] = h + 5.0;

        const uint8_t r = static_cast<uint8_t>(80 + 80 * (0.5 + 0.5 * std::sin(4.0 * x)));
        const uint8_t g = static_cast<uint8_t>(100 + 90 * (0.5 + 0.5 * std::cos(4.0 * y + 0.4)));
        const uint8_t b = static_cast<uint8_t>(140 + 70 * (0.5 + 0.5 * std::sin(3.0 * (x + y))));
        terrainColors[i * Y_SAMPLES + j] = {r, g, b};
      }
    }

    auto terrain = world.addHeightMap(X_SAMPLES, Y_SAMPLES, X_SIZE, Y_SIZE, 0.0, 0.0, terrainHeights);
    terrain->setName("drop_heightmap");
    terrain->setColor(terrainColors);

    raisim::RaisimServer server(&world);
    server.launchServer();

    size_t dropped = 0;
    constexpr size_t targetDropCount = 800;
    constexpr double spacingX = X_SIZE / 16.0;
    constexpr double spacingY = Y_SIZE / 16.0;
    constexpr double startX = -X_SIZE * 0.5 + spacingX * 0.5;
    constexpr double startY = -Y_SIZE * 0.5 + spacingY * 0.5;

    int i = 0;
    while (true) {
      RS_TIMED_LOOP(int(world.getTimeStep() * 1e6));
      server.integrateWorldThreadSafe([&]() {
        if (dropped < targetDropCount && (i % 3 == 0)) {
          for (int k = 0; k < 6; k++) {
            if (dropped >= targetDropCount) break;

            const size_t row = dropped / 16;
            const size_t col = dropped % 16;
            const double x = startX + spacingX * static_cast<double>(col) + 0.02 * (dropped % 10);
            const double y = startY + spacingY * static_cast<double>(row % 16) + 0.02 * ((dropped % 7) - 3);
            const double z = 19.5 + 0.03 * (dropped / 80);

            raisim::SingleBodyObject* obj = nullptr;
            switch (dropped % 4) {
              case 0:
                obj = world.addBox(0.18, 0.18, 0.18, 1.0);
                obj->setAppearance("0.2, 0.5, 1.0, 1.0");
                break;
              case 1:
                obj = world.addSphere(0.12, 1.0);
                obj->setAppearance("1.0, 0.2, 0.2, 1.0");
                break;
              case 2:
                obj = world.addCapsule(0.10, 0.30, 1.0);
                obj->setAppearance("0.2, 1.0, 0.4, 1.0");
                break;
              default:
                obj = world.addCylinder(0.09, 0.28, 1.0);
                obj->setAppearance("1.0, 0.8, 0.2, 1.0");
                break;
            }

            obj->setName("drop_" + std::to_string(dropped));
            obj->setPosition(x, y, z);
            obj->setVelocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            ++dropped;
          }
        }
        ++i;
      });
    }

    return 0;
}
