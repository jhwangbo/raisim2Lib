#include <memory>
#include <string>

#include "rayrai/example_common.hpp"
#include "rayrai_example_resources.hpp"
#include "rayrai_example_compat.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  ExampleApp app;
  if (!app.init("rayrai_example_charging_station", 1280, 720))
    return -1;

  auto world = std::make_shared<raisim::World>();
  world->addGround();

  const std::string urdfPath = rayraiRscPath(argv[0], "go1/go1.urdf");
  auto robot = world->addArticulatedSystem(urdfPath);
  if (robot) {
    robot->setGeneralizedCoordinate({0, 0, 0.32, 1.0, 0.0, 0.0, 0.0, 0, 0.67, -1.3, 0, 0.67, -1.3, 0,
      0.67, -1.3, 0, 0.67, -1.3});
  }

  auto viewer = std::make_shared<raisin::RayraiWindow>(world, 1280, 720);
  viewer->setRenderQualitySettings(raisin::RayraiWindow::defaultRenderQualitySettings(
    raisin::RayraiWindow::RenderQualityPreset::Balanced));
  raisim_examples::setRayraiBackgroundColorRgb255(*viewer, {30, 30, 40, 255});
  viewer->setGroundPatternResourcePath(
    rayraiRscPath(argv[0], "minitaur/vision/checker_blue.png"));

  raisim_examples::addRayraiBasicSceneLights(*viewer);

  auto& camera = viewer->getCamera();
  camera.position = {0.0f, 1.55f, 0.75f};
  camera.yaw = -90.0f;
  camera.pitch = -14.0f;

  while (!app.quit) {
    app.processEvents();

    world->integrate();

    app.beginFrame();
    app.renderViewer(*viewer);
    app.endFrame();
  }

  viewer.reset();
  app.shutdown();
  return 0;
}
