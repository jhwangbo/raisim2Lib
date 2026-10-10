// Copyright (c) 2026 Raion Robotics Inc. All rights reserved.
#include <algorithm>
#include <array>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "rayrai/example_common.hpp"
#include "rayrai/Camera.hpp"
#include "raisim/World.hpp"

namespace {
constexpr size_t kXSamples = 201;
constexpr size_t kYSamples = 101;
constexpr int kStepsPerFrame = 8;

struct Options {
  bool headless = false;
  bool hidden = false;
  int steps = -1;
  std::string activationKey;
  std::string exportPath;
};

bool parseOptions(int argc, char* argv[], Options& options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--headless")
      options.headless = true;
    else if (arg == "--hidden")
      options.hidden = true;
    else if (arg.rfind("--steps=", 0) == 0)
      options.steps = std::max(1, std::stoi(arg.substr(8)));
    else if (arg.rfind("--activation-key=", 0) == 0)
      options.activationKey = arg.substr(17);
    else if (arg.rfind("--export=", 0) == 0)
      options.exportPath = arg.substr(9);
    else {
      std::fprintf(stderr, "Usage: %s [--headless|--hidden] [--steps=N] "
                           "[--export=DIR] [--activation-key=FILE]\n", argv[0]);
      return false;
    }
  }
  if (options.headless && options.steps < 0)
    options.steps = 3000;
  return true;
}

void dragBlocks(raisim::World& world, const std::array<raisim::Box*, 3>& blocks) {
  for (auto* block : blocks) {
    // Apply a speed controller; the terrain solver supplies the contact friction.
    const double velocity = block->getLinearVelocity()[0];
    const double target = block->getPosition()[0] < 2.8 ? 0.4 : 0.0;
    const double force = target > 0.0
      ? 6.0 * block->getMass(0) + 100.0 * (target - velocity)
      : -100.0 * velocity;
    block->setExternalForce(0, raisim::Vec<3>{force, 0.0, 0.0});
  }
  world.integrate();
}
} // namespace

int main(int argc, char* argv[]) {
  Options options;
  if (!parseOptions(argc, argv, options))
    return 1;
  if (!options.activationKey.empty())
    raisim::World::setActivationKey(options.activationKey);

  auto world = std::make_shared<raisim::World>();
  world->setTimeStep(0.002);

  raisim::DeformableTerrain::Material material;
  material.bekkerKphi = 4.0e4;
  material.sinkageExponent = 1.1;
  material.damping = 2000.0;
  material.cohesion = 100.0;

  auto* soil = world->addDeformableTerrain(kXSamples, kYSamples, 8.0, 4.0, 0.0, 0.0,
    std::vector<double>(kXSamples * kYSamples, 0.0), material);
  soil->setName("soil");
  soil->setAppearance("0.49 0.325 0.188 1");
  soil->setColor(std::vector<raisim::ColorRGB>(kXSamples * kYSamples, {125, 83, 48}));

  constexpr const char* appearances[] = {"0.2 0.5 0.9 1", "0.9 0.5 0.1 1", "0.8 0.2 0.2 1"};
  std::array<raisim::Box*, 3> blocks;
  for (size_t i = 0; i < blocks.size(); ++i) {
    blocks[i] = world->addBox(0.3, 0.3, 0.2, 10.0 * (i + 1));
    blocks[i]->setPosition(-2.8, 0.8 * (double(i) - 1.0), 0.1);
    blocks[i]->setName("block_" + std::to_string(i));
    blocks[i]->setAppearance(appearances[i]);
  }

  // Settle under gravity before dragging to distinguish weight loading from impact.
  for (int i = 0; i < 1000; ++i)
    world->integrate();

  ExampleApp app;
  std::shared_ptr<raisin::RayraiWindow> viewer;
  if (!options.headless) {
    if (!app.init("Mesh soil: 10 / 20 / 30 kg blocks", 1280, 720, !options.hidden))
      return 1;
    viewer = std::make_shared<raisin::RayraiWindow>(world, 1280, 720);
    auto& camera = viewer->getCamera();
    camera.position = {4.0f, -5.0f, 3.0f};
    camera.yaw = 140.0f;
    camera.pitch = -25.0f;
  }

  int step = 0;
  while (options.steps < 0 || step < options.steps) {
    if (!options.headless) {
      app.processEvents();
      if (app.quit)
        break;
    }

    int batch = options.headless ? 1 : kStepsPerFrame;
    if (options.steps >= 0)
      batch = std::min(batch, options.steps - step);
    for (int i = 0; i < batch; ++i, ++step)
      dragBlocks(*world, blocks);

    if (!options.headless) {
      app.beginFrame();
      app.renderViewer(*viewer);
      app.endFrame();
    }
  }

  if (!options.exportPath.empty()) {
    std::filesystem::create_directories(options.exportPath);
    world->exportToXml(options.exportPath, "world.xml");
  }

  bool finite = true;
  for (auto* block : blocks) {
    finite = finite && block->getPosition().allFinite();
    std::printf("%s: x=%.3f m, bottom=%.4f m\n", block->getName().c_str(),
      block->getPosition()[0], block->getPosition()[2] - 0.1);
  }
  const auto& stats = soil->getSoilStepStats();
  std::printf("mesh: %zu vertices; last step: %zu contact rows, %zu changed vertices\n",
    kXSamples * kYSamples, stats.contacts, stats.updatedVertices);

  viewer.reset();
  if (!options.headless)
    app.shutdown();
  return finite ? 0 : 1;
}
