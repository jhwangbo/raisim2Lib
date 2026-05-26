// Demonstrates additional lights: a warm spot, a cool fill, and an area-style
// soft light, all illuminating the same row of primitives.

#include "doc_image_common.hpp"

#include <cmath>

namespace {
constexpr int kWidth = 1280;
constexpr int kHeight = 720;
}  // namespace

int main(int argc, char** argv) {
  const auto outputDir = doc_image::resolveOutputDir(argc, argv);
  doc_image::OffscreenContext gl;
  if (!gl.init("doc_image_lights")) doc_image::finishAndExit(1);

  auto world = std::make_shared<raisim::World>();
  raisin::RayraiWindow viewer(world, kWidth, kHeight);

  const auto preset = raisin::RayraiWindow::RenderQualityPreset::Ultra;
  auto q = raisin::RayraiWindow::defaultRenderQualitySettings(preset);
  q.colorMode = raisin::ViewerColorMode::AcesApprox;
  q.pbrToneMapping = true;
  doc_image::applyCommonSceneOptions(q, preset);
  q.addViewerFillLights = false;            // make the additional lights stand out
  q.mainLightAmbient = glm::vec3(0.05f);    // dark ambient so spot lights pop
  q.mainLightDiffuse = glm::vec3(0.25f);
  q.bloomEnabled = true;
  q.bloomStrength = 0.18f;
  q.maxAdditionalLightsPerFrame = 6;
  q.maxPointShadowLights = 3;
  viewer.setRenderQualitySettings(q);
  viewer.setBackgroundColorRgb255({14, 18, 28, 255});

  world->addGround();

  // Row of grey columns to receive the lights.
  for (int i = -2; i <= 2; ++i) {
    auto* c = world->addBox(0.4, 0.4, 1.2, 1.0);
    c->setPosition(i * 0.8, 0.0, 0.6);
    c->setBodyType(raisim::BodyType::STATIC);
    c->setAppearance("0.62,0.62,0.62,1");
  }

  // Warm spotlight from the left.
  raisin::RayraiWindow::AdditionalLight warmSpot;
  warmSpot.type = raisin::LightType::SPOT;
  warmSpot.position = glm::vec3(-2.2f, -1.8f, 2.6f);
  warmSpot.direction = glm::normalize(glm::vec3(0.9f, 0.7f, -1.0f));
  warmSpot.diffuse = glm::vec3(2.4f, 1.6f, 0.9f);
  warmSpot.spotInnerCos = std::cos(glm::radians(14.0f));
  warmSpot.spotOuterCos = std::cos(glm::radians(34.0f));
  warmSpot.castsShadows = true;
  viewer.addAdditionalLight(warmSpot);

  // Cool fill point light from the right.
  raisin::RayraiWindow::AdditionalLight coolFill;
  coolFill.type = raisin::LightType::POINT;
  coolFill.position = glm::vec3(2.2f, -1.4f, 1.8f);
  coolFill.diffuse = glm::vec3(0.40f, 0.62f, 1.10f);
  coolFill.constant = 1.0f; coolFill.linear = 0.20f; coolFill.quadratic = 0.04f;
  viewer.addAdditionalLight(coolFill);

  // Soft area light from above behind the row, with shadows off.
  raisin::RayraiWindow::AdditionalLight rim;
  rim.type = raisin::LightType::AREA;
  rim.position = glm::vec3(0.0f, 1.6f, 2.4f);
  rim.direction = glm::normalize(glm::vec3(0.0f, -1.0f, -0.6f));
  rim.diffuse = glm::vec3(0.55f, 0.92f, 0.55f);
  rim.areaSize = glm::vec2(1.8f, 0.8f);
  rim.castsShadows = false;
  viewer.addAdditionalLight(rim);

  doc_image::setCameraLookAt(viewer.getCamera(),
                             glm::vec3(0.0f, -3.4f, 1.6f),
                             glm::vec3(0.0f, 0.0f, 0.6f),
                             /*horizontalFovDeg=*/65.0f);

  raisin::RayraiWindow::RenderOverrides overrides;
  overrides.doShadows = true;

  const auto path = outputDir / "rayrai_lights.png";
  if (!doc_image::captureScene(viewer, kWidth, kHeight, path, 2, overrides))
    doc_image::finishAndExit(1);
  std::printf("doc_image: wrote %s\n", path.string().c_str());
  doc_image::finishAndExit(0);
}
