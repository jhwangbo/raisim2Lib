#include <memory>

#include <glm/glm.hpp>

#include "rayrai/example_common.hpp"
#include "rayrai_example_compat.hpp"
#include "raisim/World.hpp"

int main() {
  ExampleApp app;
  if (!app.init("rayrai_runtime_scene_editing", 1280, 720))
    return -1;

  auto world = std::make_shared<raisim::World>();
  world->setTimeStep(0.005);
  world->setGravity({0.0, 0.0, -9.81});

  auto* ground = world->addGround();
  ground->setAppearance("checkerboard");

  auto* source = world->addBox(0.28, 0.28, 0.28, 1.0);
  source->setName("source_box");
  source->setPosition(-0.55, 0.0, 1.1);
  source->setAppearance("0.20,0.56,0.95,1.0");

  raisim::World::SingleBodySnapshot sourceSnapshot;
  world->captureSingleBodySnapshot(source, sourceSnapshot);

  auto* clone = world->cloneSingleBodyObject(source, "cloned_box");
  if (clone) {
    clone->setPosition(0.0, 0.0, 1.45);
    clone->setAppearance("0.95,0.52,0.24,1.0");
  }

  auto* filtered = world->addSphere(0.18, 1.0, "default",
                                    raisim::CollisionGroup(1),
                                    raisim::CollisionGroup(-1));
  filtered->setName("filter_sphere");
  filtered->setPosition(0.58, 0.0, 1.25);
  filtered->setAppearance("0.24,0.84,0.38,1.0");
  const auto filteredId = filtered->getId();

  auto viewer = std::make_shared<raisin::RayraiWindow>(world, 1280, 720);
  viewer->setRenderQualitySettings(raisin::RayraiWindow::defaultRenderQualitySettings(
    raisin::RayraiWindow::RenderQualityPreset::Balanced));
  raisim_examples::setRayraiBackgroundColorRgb255(*viewer, {24, 26, 30, 255});
  raisim_examples::addRayraiBasicSceneLights(*viewer);

  auto& camera = viewer->getCamera();
  camera.target = {0.0f, 0.0f, 0.75f};
  camera.position = {2.7f, -3.2f, 2.2f};
  camera.yaw = 132.0f;
  camera.pitch = -26.0f;
  camera.setCameraFixedTarget(true);
  camera.setCameraFixedDistance(true);

  bool filterDisabled = false;
  int step = 0;
  while (!app.quit) {
    app.processEvents();
    if (app.quit)
      break;

    if (step % 240 == 0) {
      filterDisabled = !filterDisabled;
      auto* object = dynamic_cast<raisim::SingleBodyObject*>(world->getObjectById(filteredId));
      if (object) {
        world->setObjectCollisionFilter(object,
                                        raisim::CollisionGroup(1),
                                        filterDisabled ? raisim::CollisionGroup(0)
                                                       : raisim::CollisionGroup(-1));
        object->setAppearance(filterDisabled ? "0.45,0.45,0.48,0.38"
                                             : "0.24,0.84,0.38,1.0");
        object->setPosition(0.58, 0.0, 1.25);
        object->setVelocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      }
    }

    if (step % 480 == 120) {
      world->restoreSingleBodySnapshot(source, sourceSnapshot);
      source->setPosition(-0.55, 0.0, 1.1);
      source->setVelocity(0.65, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    if (step % 720 == 360 && clone) {
      world->removeObject(clone);
      clone = nullptr;
    } else if (step % 720 == 540 && !clone) {
      clone = world->cloneSingleBodyObject(source, "cloned_box");
      if (clone) {
        clone->setPosition(0.0, 0.0, 1.45);
        clone->setVelocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        clone->setAppearance("0.95,0.52,0.24,1.0");
      }
    }

    world->integrate();

    app.beginFrame();
    app.renderViewer(*viewer);
    app.endFrame();
    ++step;
  }

  viewer.reset();
  app.shutdown();
  return 0;
}
