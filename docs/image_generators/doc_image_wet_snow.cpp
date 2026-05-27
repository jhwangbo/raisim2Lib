// Renders a scene with weatherWetMaterialEnabled cranked up so the surface
// response (darkened albedo, drop in roughness, rain ripples) is visible.

#include "doc_image_common.hpp"

namespace {
constexpr int kWidth = 1280;
constexpr int kHeight = 720;
}  // namespace

int main(int argc, char** argv) {
  const auto outputDir = doc_image::resolveOutputDir(argc, argv);
  doc_image::OffscreenContext gl;
  if (!gl.init("doc_image_wet_snow")) doc_image::finishAndExit(1);

  auto world = std::make_shared<raisim::World>();
  raisin::RayraiWindow viewer(world, kWidth, kHeight);

  const auto preset = raisin::RayraiWindow::RenderQualityPreset::Ultra;
  auto q = raisin::RayraiWindow::defaultRenderQualitySettings(preset);
  q.colorMode = raisin::ViewerColorMode::AcesApprox;
  q.pbrToneMapping = true;
  doc_image::applyCommonSceneOptions(q, preset);
  // Override the helper's defaults: wet scene wants a glossier ground.
  q.reflectiveGround = true;
  q.reflectiveGroundRoughness = 0.22f;
  q.reflectiveGroundMetallic = 0.35f;
  q.weatherWetMaterialEnabled = true;
  q.weatherWetness = 0.85f;
  q.weatherPuddleStrength = 0.55f;
  q.weatherRainRippleStrength = 0.55f;
  q.weatherRainRippleScale = 18.0f;
  q.weatherWetAlbedoDarkening = 0.40f;
  q.weatherWetRoughnessScale = 0.25f;
  q.weatherWetSpecularBoost = 0.50f;
  q.bloomEnabled = true;
  q.bloomStrength = 0.10f;
  viewer.setRenderQualitySettings(q);
  viewer.setBackgroundColorRgb255({26, 32, 44, 255});

  // Apply matching rain weather so the diffuse, IBL contribution, and tone
  // curve all sit in the same brightness range.
  auto weather = raisin::RayraiWindow::defaultWeatherSettings(
      raisin::RayraiWindow::WeatherPreset::HeavyRain);
  weather.enabled = true;
  weather.timeOfDayHours = 15.0f;
  weather.affectSensors = false;
  viewer.setWeatherSettings(weather);
  for (int i = 0; i < 60; ++i) viewer.updateWeather(1.0 / 60.0);

  world->addGround();

  // Cluster of props that benefit from wet shading.
  auto* boxA = world->addBox(0.5, 0.5, 0.4, 1.0);
  boxA->setPosition(-0.7, 0.0, 0.2);
  boxA->setBodyType(raisim::BodyType::STATIC);
  boxA->setAppearance("0.95,0.43,0.12,1");

  auto* boxB = world->addBox(0.35, 0.35, 0.6, 1.0);
  boxB->setPosition(0.4, -0.6, 0.3);
  boxB->setBodyType(raisim::BodyType::STATIC);
  boxB->setAppearance("0.18,0.46,0.74,1");

  auto* ball = world->addSphere(0.28, 1.0);
  ball->setPosition(0.0, 0.7, 0.28);
  ball->setBodyType(raisim::BodyType::STATIC);
  ball->setAppearance("0.92,0.86,0.34,1");

  doc_image::setCameraLookAt(viewer.getCamera(),
                             glm::vec3(2.6f, 2.4f, 1.2f),
                             glm::vec3(0.0f, 0.0f, 0.25f),
                             /*horizontalFovDeg=*/60.0f);

  raisin::RayraiWindow::RenderOverrides overrides;
  overrides.doShadows = true;

  const auto path = outputDir / "rayrai_wet_material.png";
  if (!doc_image::captureScene(viewer, kWidth, kHeight, path, 2, overrides))
    doc_image::finishAndExit(1);
  std::printf("doc_image: wrote %s\n", path.string().c_str());
  doc_image::finishAndExit(0);
}
