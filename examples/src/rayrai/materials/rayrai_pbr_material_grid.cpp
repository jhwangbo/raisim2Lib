#include <cmath>
#include <memory>

#include "example_resources.hpp"
#include "raisim/World.hpp"
#include "rayrai/PhysicsSnapshot.hpp"
#include "rayrai/example_common.hpp"
#include "rayrai_example_compat.hpp"

int main(int argc, char* argv[]) {
  ExampleApp app;
  if (!app.init("rayrai_pbr_material_grid", 1280, 720)) return -1;

  auto world = std::make_shared<raisim::World>();
  world->addGround();

  // The renderer owns an empty world; physics reaches it only through copied snapshots.
  raisin::PhysicsSnapshotCapture snapshotCapture;
  auto viewer = std::make_shared<raisin::RayraiWindow>(std::make_shared<raisim::World>(), 1280, 720);
  viewer->setRenderQualitySettings(raisin::RayraiWindow::defaultRenderQualitySettings(raisin::RayraiWindow::RenderQualityPreset::High));
  raisim_examples::setRayraiBackgroundColorRgb255(*viewer, {24, 26, 30, 255});
  viewer->setFogDensity(0.0f);
  viewer->setShadowOrtho(8.0f, 0.1f, 30.0f);
  viewer->setShadowCenterOffset(3.0f);

  raisim_examples::addRayraiPbrSceneLights(*viewer);

  // Khronos glTF Sample Assets: MetalRoughSpheres.
  // The model contains a grid of spheres spanning metallic and roughness values.
  const std::string spheresPath = exampleRscPath(argv[0], "rayrai/pbr/MetalRoughSpheres/glTF/MetalRoughSpheres.gltf");
  auto spheres = viewer->addVisualMesh("metal_rough_spheres", spheresPath, 0.75, 0.75, 0.75, 1.0f, 1.0f, 1.0f, 1.0f);
  spheres->setPosition(0.0, 0.0, 1.0);
  spheres->setOrientation(0.7071068, 0.0, 0.0, 0.7071068);
  spheres->setDetectable(true);

  viewer->getCamera().position = {3.5f, -5.0f, 2.8f};
  viewer->getCamera().yaw = 126.0f;
  viewer->getCamera().pitch = -26.0f;

  while (!app.quit) {
    app.processEvents();
    if (app.quit) break;

    world->integrate();
    // CPU-only copy immediately after physics; apply on the thread owning the GL context.
    const auto snapshot = snapshotCapture.capture(*world);
    const double t = world->getWorldTime();
    const double a = 0.15 * t;
    spheres->setOrientation(std::cos(a), 0.0, 0.0, std::sin(a));

    viewer->applyPhysicsSnapshot(snapshot);

    app.beginFrame();
    app.renderViewer(*viewer);
    app.endFrame();
  }

  viewer.reset();
  app.shutdown();
  return 0;
}
