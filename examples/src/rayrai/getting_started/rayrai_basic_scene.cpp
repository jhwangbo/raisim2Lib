#include <memory>
#include <string>

#include "example_resources.hpp"
#include "raisim/World.hpp"
#include "rayrai/PhysicsSnapshot.hpp"
#include "rayrai/example_common.hpp"
#include "rayrai_example_compat.hpp"

int main(int argc, char* argv[]) {
  ExampleApp app;
  if (!app.init("rayrai_example_charging_station", 1280, 720)) return -1;

  auto world = std::make_shared<raisim::World>();
  world->addGround();

  const std::string urdfPath = exampleRscPath(argv[0], "go1/go1.urdf");
  raisim::ArticulatedSystemOption options;
  options.convexifyCollisionMeshes = true;
  auto robot = world->addArticulatedSystem(urdfPath, "", {}, raisim::CollisionGroup(1), raisim::CollisionGroup(-1), options);
  if (robot) {
    robot->setGeneralizedCoordinate({0, 0, 0.32, 1.0, 0.0, 0.0, 0.0, 0, 0.67, -1.3, 0, 0.67, -1.3, 0, 0.67, -1.3, 0, 0.67, -1.3});
  }

  // The renderer owns an empty world; physics reaches it only through copied snapshots.
  raisin::PhysicsSnapshotCapture snapshotCapture;
  auto viewer = std::make_shared<raisin::RayraiWindow>(std::make_shared<raisim::World>(), 1280, 720);
  viewer->setRenderQualitySettings(raisin::RayraiWindow::defaultRenderQualitySettings(raisin::RayraiWindow::RenderQualityPreset::Balanced));
  raisim_examples::setRayraiBackgroundColorRgb255(*viewer, {30, 30, 40, 255});
  viewer->setGroundPatternResourcePath(exampleRscPath(argv[0], "minitaur/vision/checker_blue.png"));

  raisim_examples::addRayraiBasicSceneLights(*viewer);

  auto& camera = viewer->getCamera();
  camera.position = {0.0f, 1.55f, 0.75f};
  camera.yaw = -90.0f;
  camera.pitch = -14.0f;

  while (!app.quit) {
    app.processEvents();

    world->integrate();
    // With separate threads, integrate() and capture() share the physics lock. Release it
    // before handing this immutable snapshot to the renderer. applyPhysicsSnapshot() and
    // rendering use only the window lock on the thread owning the GL context.
    const auto snapshot = snapshotCapture.capture(*world);

    viewer->applyPhysicsSnapshot(snapshot);

    app.beginFrame();
    app.renderViewer(*viewer);
    app.endFrame();
  }

  viewer.reset();
  app.shutdown();
  return 0;
}
