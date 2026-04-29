#include <cmath>
#include <memory>

#include "rayrai/example_common.hpp"
#include "rayrai_example_resources.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  ExampleApp app;
  if (!app.init("rayrai_pbr_texture_maps", 1280, 720))
    return -1;

  auto world = std::make_shared<raisim::World>();
  world->addGround();

  auto viewer = std::make_shared<raisin::RayraiWindow>(world, 1280, 720);
  viewer->setBackgroundColor({22, 24, 28, 255});
  viewer->setFogDensity(0.0f);
  viewer->setShadowOrtho(7.0f, 0.1f, 30.0f);
  viewer->setShadowCenterOffset(3.0f);

  // Khronos glTF Sample Assets: BoomBox.
  // This is a textured core glTF PBR asset packed in a binary glTF.
  // It is useful for checking imported base color, metallic-roughness, and emissive-style details.
  const std::string boomboxPath = rayraiRscPath(argv[0], "rayrai/pbr/BoomBox/glTF/BoomBox.gltf");
  const auto boombox = viewer->addVisualMesh("boombox", boomboxPath,
    1.0, 1.0, 1.0, 1.0f, 1.0f, 1.0f, 1.0f);
  boombox->setPosition(0.0, 0.0, 1.0);
  boombox->setOrientation(0.7071068, 0.0, 0.0, 0.7071068);
  boombox->setDetectable(true);

  auto& camera = viewer->getCamera();
  camera.target = {0.0f, 0.0f, 1.0f};
  camera.position = {0.08f, -0.16f, 1.04f};
  camera.yaw = 116.6f;
  camera.pitch = -12.6f;
  camera.setCameraFixedTarget(true);
  camera.setCameraFixedDistance(true);

  while (!app.quit) {
    app.processEvents();
    if (app.quit)
      break;

    world->integrate();
    const double t = world->getWorldTime();
    const double a = 0.35 * t;
    boombox->setOrientation(std::cos(a), 0.0, 0.0, std::sin(a));

    app.beginFrame();
    app.renderViewer(*viewer);
    app.endFrame();
  }

  viewer.reset();
  app.shutdown();
  return 0;
}
