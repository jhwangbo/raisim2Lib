#include <memory>

#include <glm/glm.hpp>

#include "raisim/World.hpp"
#include "rayrai/PhysicsSnapshot.hpp"
#include "rayrai/example_common.hpp"
#include "rayrai_example_compat.hpp"

int main() {
  ExampleApp app;
  if (!app.init("rayrai_swept_ccd", 1280, 720)) return -1;

  auto world = std::make_shared<raisim::World>();
  world->setTimeStep(0.02);
  world->setGravity({0.0, 0.0, -9.81});

  auto contactSettings = world->getContactSettings();
  contactSettings.sweptCcdEnabled = true;
  contactSettings.sweptCcdMinSpeed = 8.0;
  contactSettings.sweptCcdSpeculativeMargin = 1e-4;
  world->setContactSettings(contactSettings);

  auto* ground = world->addGround();
  ground->setAppearance("checkerboard");

  auto* fastSphere = world->addSphere(0.08, 1.0);
  fastSphere->setName("swept_ccd_sphere");
  fastSphere->setAppearance("0.95,0.34,0.22,1.0");

  auto* referencePost = world->addBox(0.04, 0.04, 3.0, 1.0);
  referencePost->setBodyType(raisim::BodyType::STATIC);
  referencePost->setPosition(0.45, 0.0, 1.5);
  referencePost->setAppearance("0.25,0.30,0.36,0.6");

  // The renderer owns an empty world; physics reaches it only through copied snapshots.
  raisin::PhysicsSnapshotCapture snapshotCapture;
  auto viewer = std::make_shared<raisin::RayraiWindow>(std::make_shared<raisim::World>(), 1280, 720);
  viewer->setRenderQualitySettings(raisin::RayraiWindow::defaultRenderQualitySettings(raisin::RayraiWindow::RenderQualityPreset::Balanced));
  raisim_examples::setRayraiBackgroundColorRgb255(*viewer, {24, 26, 30, 255});
  raisim_examples::addRayraiBasicSceneLights(*viewer);

  auto& camera = viewer->getCamera();
  camera.target = {0.0f, 0.0f, 1.1f};
  camera.position = {2.3f, -3.0f, 1.9f};
  camera.yaw = 128.0f;
  camera.pitch = -20.0f;
  camera.setCameraFixedTarget(true);
  camera.setCameraFixedDistance(true);

  int step = 0;
  while (!app.quit) {
    app.processEvents();
    if (app.quit) break;

    if (step % 120 == 0) {
      fastSphere->setPosition(0.0, 0.0, 3.0);
      fastSphere->setVelocity(0.0, 0.0, -35.0, 0.0, 0.0, 0.0);
      fastSphere->setAppearance("0.95,0.34,0.22,1.0");
    }

    world->integrate();

    if (fastSphere->getPosition()[2] < 0.08) {
      fastSphere->setAppearance("1.0,0.0,0.0,1.0");
    }

    const auto snapshot = snapshotCapture.capture(*world);
    viewer->applyPhysicsSnapshot(snapshot);

    app.beginFrame();
    app.renderViewer(*viewer);
    app.endFrame();
    ++step;
  }

  viewer.reset();
  app.shutdown();
  return 0;
}
