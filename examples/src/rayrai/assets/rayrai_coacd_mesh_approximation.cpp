#include <iostream>
#include <string>
#include <vector>

#include "rayrai/example_common.hpp"
#include "rayrai_example_compat.hpp"
#include "rayrai_example_resources.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  ExampleApp app;
  if (!app.init("rayrai_coacd_mesh_approximation", 1600, 900))
    return -1;

  auto world = std::make_shared<raisim::World>();
  world->setGravity({0.0, 0.0, 0.0});

  auto viewer = std::make_shared<raisin::RayraiWindow>(world, 1600, 900);
  viewer->setRenderQualitySettings(raisin::RayraiWindow::defaultRenderQualitySettings(
    raisin::RayraiWindow::RenderQualityPreset::Balanced));
  raisim_examples::setRayraiBackgroundColorRgb255(*viewer, {24, 26, 30, 255});
  viewer->setFogDensity(0.0f);
  viewer->setShadowOrtho(8.0f, 0.1f, 40.0f);
  viewer->setShadowCenterOffset(4.0f);

  raisim::CoacdOptions options;
  options.threshold = 0.04;
  options.maxConvexHull = 12;
  options.preprocess = "off";
  options.sampleResolution = 1200;
  options.mctsNodes = 12;
  options.mctsIteration = 80;
  options.mctsMaxDepth = 3;
  options.merge = true;
  options.maxConvexHullVertex = 64;
  options.seed = 7;

  const std::vector<std::string> meshFiles = {
    "ycb/002_master_chef_can/google_16k/textured_vhacd.obj",
    "ycb/007_tuna_fish_can/google_16k/textured.obj",
    "ycb/012_strawberry/google_16k/textured.obj",
    "ycb/013_apple/google_16k/textured.obj",
    "ycb/017_orange/google_16k/textured.obj"
  };

  const double meshScale = 3.0;
  const double columnOffset = 0.45;
  const double rowSpacing = 0.72;
  const double startY = 1.45;

  for (size_t row = 0; row < meshFiles.size(); ++row) {
    const std::string path = rayraiRscPath(argv[0], meshFiles[row]);
    const double y = startY - static_cast<double>(row) * rowSpacing;

    auto* original = world->addMesh(path, 1.0, meshScale, "",
                                    raisim::MeshCollisionMode::ORIGINAL_MESH,
                                    1, 0);
    original->setName("original_" + std::to_string(row));
    original->setPosition(-columnOffset, y, 0.8);
    original->setBodyType(raisim::BodyType::STATIC);
    original->setAppearance("0.72,0.76,0.82,1.0");

    auto* coacd = world->addMesh(path, 1.0, meshScale, "",
                                 raisim::MeshCollisionMode::CONVEXIFY,
                                 1, 0, options);
    coacd->setName("coacd_" + std::to_string(row));
    coacd->setPosition(columnOffset, y, 0.8);
    coacd->setBodyType(raisim::BodyType::STATIC);
    coacd->setAppearance("0.95,0.52,0.24,0.78");

    std::cout << meshFiles[row] << ": addMesh generated "
              << coacd->getCoacdConvexParts().size()
              << " CoACD collision parts" << std::endl;
  }

  auto& camera = viewer->getCamera();
  camera.target = {0.0f, 0.0f, 0.8f};
  camera.position = {0.0f, -3.4f, 3.0f};
  camera.yaw = 90.0f;
  camera.pitch = -36.0f;
  camera.setCameraFixedTarget(true);
  camera.setCameraFixedDistance(true);

  while (!app.quit) {
    app.processEvents();
    if (app.quit)
      break;

    world->integrate();
    app.beginFrame();
    app.renderViewer(*viewer);
    app.endFrame();
  }

  viewer.reset();
  app.shutdown();
  return 0;
}
