#include <cmath>
#include <memory>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "rayrai/example_common.hpp"
#include "rayrai_example_compat.hpp"
#include "raisim/World.hpp"

int main() {
  ExampleApp app;
  if (!app.init("rayrai_quality_lighting", 1280, 720))
    return -1;

  auto world = std::make_shared<raisim::World>();
  world->addGround();

  auto* redBox = world->addBox(0.55, 0.55, 0.55, 1.0);
  redBox->setPosition(-1.15, -0.1, 0.28);
  redBox->setBodyType(raisim::BodyType::STATIC);
  redBox->setAppearance("red");

  auto* blueSphere = world->addSphere(0.34, 1.0);
  blueSphere->setPosition(-0.15, 0.35, 0.35);
  blueSphere->setBodyType(raisim::BodyType::STATIC);
  blueSphere->setAppearance("blue");

  auto* greenCylinder = world->addCylinder(0.24, 0.8, 1.0);
  greenCylinder->setPosition(0.85, -0.25, 0.40);
  greenCylinder->setBodyType(raisim::BodyType::STATIC);
  greenCylinder->setAppearance("green");

  auto viewer = std::make_shared<raisin::RayraiWindow>(world, 1280, 720);
  auto quality = raisin::RayraiWindow::defaultRenderQualitySettings(
    raisin::RayraiWindow::RenderQualityPreset::Ultra);
  quality.reflectiveGround = true;
  quality.fxaaEnabled = true;
  quality.depthOfFieldEnabled = true;
  quality.depthOfFieldFocusDistance = 5.0f;
  quality.depthOfFieldFocusRange = 8.0f;
  quality.depthOfFieldMaxRadius = 1.25f;
  quality.addViewerFillLights = false;
  quality.shadowOrthoHalfSize = 9.0f;
  quality.shadowCenterOffset = 4.0f;
  viewer->setRenderQualitySettings(quality);
  raisim_examples::setRayraiBackgroundColorRgb255(*viewer, {32, 35, 46, 255});

  viewer->clearAdditionalLights();

  raisin::RayraiWindow::AdditionalLight point;
  point.type = raisin::LightType::POINT;
  point.position = glm::vec3(-1.7f, -1.2f, 2.3f);
  point.diffuse = glm::vec3(0.95f, 0.35f, 0.18f);
  point.specular = glm::vec3(0.18f, 0.08f, 0.04f);
  point.linear = 0.08f;
  point.quadratic = 0.025f;
  viewer->addAdditionalLight(point);

  raisin::RayraiWindow::AdditionalLight spot;
  spot.type = raisin::LightType::SPOT;
  spot.position = glm::vec3(1.8f, -1.6f, 2.6f);
  spot.direction = glm::normalize(glm::vec3(-1.4f, 1.0f, -1.8f));
  spot.diffuse = glm::vec3(0.18f, 0.42f, 1.0f);
  spot.specular = glm::vec3(0.08f, 0.12f, 0.22f);
  spot.spotInnerCos = std::cos(glm::radians(14.0f));
  spot.spotOuterCos = std::cos(glm::radians(28.0f));
  spot.linear = 0.06f;
  spot.quadratic = 0.018f;
  viewer->addAdditionalLight(spot);

  raisin::RayraiWindow::AdditionalLight area;
  area.type = raisin::LightType::AREA;
  area.position = glm::vec3(0.1f, 1.8f, 2.1f);
  area.diffuse = glm::vec3(0.55f, 0.65f, 0.42f);
  area.specular = glm::vec3(0.08f);
  area.radius = 1.4f;
  area.areaSize = glm::vec2(1.8f, 0.9f);
  area.linear = 0.04f;
  area.quadratic = 0.012f;
  viewer->addAdditionalLight(area);

  auto& camera = viewer->getCamera();
  camera.position = {2.4f, -3.2f, 1.85f};
  camera.yaw = 126.0f;
  camera.pitch = -24.0f;
  camera.zoom = 34.0f;

  while (!app.quit) {
    app.processEvents();
    if (app.quit)
      break;

    const double t = world->getWorldTime();
    point.position = glm::vec3(-1.7f + 0.35f * std::sin(t), -1.2f, 2.3f);
    spot.direction = glm::normalize(glm::vec3(-1.4f + 0.25f * std::sin(0.7 * t), 1.0f, -1.8f));
    viewer->clearAdditionalLights();
    viewer->addAdditionalLight(point);
    viewer->addAdditionalLight(spot);
    viewer->addAdditionalLight(area);

    world->integrate();
    app.beginFrame();
    app.renderViewer(*viewer);
    app.endFrame();
  }

  viewer.reset();
  app.shutdown();
  return 0;
}
