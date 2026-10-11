// Copyright (c) 2026 Raion Robotics Inc. All rights reserved.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "example_resources.hpp"
#include "rayrai_loading_progress.hpp"
#include "deformable_terrain_scene.hpp"
#include "../worlds/offroad_buggy_vehicle.hpp"
#include "../worlds/offroad_buggy_visuals.hpp"
#include "raisim/World.hpp"
#include "rayrai/Camera.hpp"
#include "rayrai/example_common.hpp"

namespace {
constexpr size_t kXSamples = 201;
constexpr size_t kYSamples = 76;
constexpr int kStepsPerFrame = 16;
constexpr double kTimeStep = 0.001;
constexpr const char* kWheelNames[] = {"fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel"};

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
      std::fprintf(stderr,
                   "Usage: %s [--headless|--hidden] [--steps=N] "
                   "[--export=DIR] [--activation-key=FILE]\n",
                   argv[0]);
      return false;
    }
  }
  if (options.headless && options.steps < 0) options.steps = 3000;
  return true;
}

void driveCar(raisim::World& world, offroad::Buggy& buggy) {
  offroad::DriverInput input;
  const auto* car = buggy.system();
  if (car->getGeneralizedCoordinate()[0] < 4.5) {
    const double speed = car->getGeneralizedVelocity()[0];
    input.throttle = std::clamp(0.20 + 0.60 * (0.8 - speed), 0.0, 0.65);
    input.brake = std::clamp(0.50 * (speed - 0.8), 0.0, 1.0);
  } else {
    input.brake = 1.0;
  }
  buggy.applyControls(input, kTimeStep);
  world.integrate();
  buggy.accumulate(kTimeStep);
}
}  // namespace

int main(int argc, char* argv[]) {
  Options options;
  if (!parseOptions(argc, argv, options)) return 1;
  if (!options.activationKey.empty()) raisim::World::setActivationKey(options.activationKey);

  auto world = std::make_shared<raisim::World>();
  world->setTimeStep(kTimeStep);

  raisim::DeformableTerrain::Material material;
  material.bekkerKphi = 2.0e6;
  material.sinkageExponent = 1.1;
  material.elasticStiffness = 2.0e7;
  material.damping = 4.0e4;
  material.cohesion = 100.0;

  auto* soil = world->addDeformableTerrain(kXSamples, kYSamples, 16.0, 6.0, 0.0, 0.0, std::vector<double>(kXSamples * kYSamples, 0.0), material);
  soil->setName("soil");
  soil->setAppearance("0.49 0.325 0.188 1");
  soil->setColor(std::vector<raisim::ColorRGB>(kXSamples * kYSamples, {125, 83, 48}));

  offroad::Buggy buggy(*world, exampleRscPath(argv[0], "offroad_buggy/buggy.urdf"));
  buggy.place(-5.5, 0.0, 0.0, [](double, double) { return 0.0; });
  auto* car = buggy.system();
  car->setName("terrain_car");

  ExampleApp app;
  RayraiLoadingProgress loading("Loading buggy assets");
  std::shared_ptr<raisin::RayraiWindow> viewer;
  std::unique_ptr<offroad::SuspensionVisuals> suspension;
  if (!options.headless) {
    if (!app.init("Wheeled car on deformable mesh soil", 1280, 720, !options.hidden)) return 1;
    ImGui::GetIO().IniFilename = nullptr;
    viewer = std::make_shared<raisin::RayraiWindow>(world, 1280, 720);
    viewer->setAsyncMeshLoadingEnabled(true);
    deformable_terrain::configureLighting(*viewer);
    suspension = std::make_unique<offroad::SuspensionVisuals>(*car);
    auto& camera = viewer->getCamera();
    const auto position = car->getBasePosition();
    camera.position = {float(position[0]) + 4.5f, -5.5f, 2.8f};
    camera.yaw = 140.0f;
    camera.pitch = -18.0f;
    camera.setTargetObject(car);
    camera.setCameraFixedDistance(true);
    loading.update(viewer->pendingAsyncMeshLoadCount());
  }

  // The first render discovers the world meshes and starts their asynchronous imports.
  bool sceneReady = options.headless;
  int step = 0;
  while (options.steps < 0 || step < options.steps) {
    if (!options.headless) {
      app.processEvents();
      if (app.quit) break;
    }

    if (sceneReady) {
      int batch = kStepsPerFrame;
      if (options.steps >= 0) batch = std::min(batch, options.steps - step);
      for (int i = 0; i < batch; ++i, ++step) driveCar(*world, buggy);
      buggy.updateTelemetry(batch * kTimeStep);
    }

    if (!options.headless) {
      suspension->update();
      app.beginFrame();
      app.renderViewer(*viewer);
      const bool wasLoading = loading.visible();
      loading.update(viewer->pendingAsyncMeshLoadCount());
      sceneReady = !loading.visible();
      if (wasLoading && !loading.visible())
        std::printf("assets: %zu meshes ready; simulation time=%.6f s\n", loading.total, world->getWorldTime());
      loading.draw();
      app.endFrame();
    }
  }

  if (!options.exportPath.empty()) {
    std::filesystem::create_directories(options.exportPath);
    world->exportToXml(options.exportPath, "world.xml");
  }

  bool finite = car->getGeneralizedCoordinate().e().allFinite() && car->getGeneralizedVelocity().e().allFinite();
  const auto& state = soil->getSoilState();
  for (const auto* values : {&state.sinkage, &state.plasticSinkage, &state.shear})
    finite = finite && std::all_of(values->begin(), values->end(), [](double value) { return std::isfinite(value); });
  std::printf("simulation: %d steps, time=%.6f s\n", step, world->getWorldTime());
  std::printf("car: x=%.3f m, speed=%.3f m/s\n", car->getGeneralizedCoordinate()[0], car->getGeneralizedVelocity()[0]);
  for (size_t i = 0; i < std::size(kWheelNames); ++i) {
    const auto& corner = buggy.telemetry().corners[i];
    std::printf("%s: speed=%.3f rad/s, shock=%.5f m, load=%.1f N\n", kWheelNames[i], corner.wheelSpeed, corner.shockExtension, corner.wheelLoad);
  }
  std::printf("suspension: maximum loop error=%.8f m\n", buggy.telemetry().maxLoopError);
  const auto& stats = soil->getSoilStepStats();
  const double rut = *std::max_element(state.plasticSinkage.begin(), state.plasticSinkage.end());
  std::printf("mesh: %zu vertices; last step: %zu contact rows, %zu changed vertices; maximum permanent rut=%.4f m\n",
              kXSamples * kYSamples, stats.contacts, stats.updatedVertices, rut);

  suspension.reset();
  viewer.reset();
  if (!options.headless) app.shutdown();
  return finite ? 0 : 1;
}
