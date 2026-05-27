// Renders the "Example: custom visuals + background color" scene: a ground
// plane plus an authored marker prop on a tinted dark background.

#include "doc_image_common.hpp"

namespace {
constexpr int kWidth = 1280;
constexpr int kHeight = 720;
}  // namespace

int main(int argc, char** argv) {
  const auto outputDir = doc_image::resolveOutputDir(argc, argv);
  doc_image::OffscreenContext gl;
  if (!gl.init("doc_image_custom_visuals")) doc_image::finishAndExit(1);

  auto world = std::make_shared<raisim::World>();
  raisin::RayraiWindow viewer(world, kWidth, kHeight);

  const auto preset = raisin::RayraiWindow::RenderQualityPreset::High;
  auto quality = raisin::RayraiWindow::defaultRenderQualitySettings(preset);
  quality.colorMode = raisin::ViewerColorMode::AcesApprox;
  quality.pbrToneMapping = true;
  doc_image::applyCommonSceneOptions(quality, preset);
  viewer.setRenderQualitySettings(quality);

  world->addGround();

  // Match the snippet: a 0.4 x 0.2 x 0.1 marker box in saturated amber.
  auto* marker = world->addBox(0.4, 0.2, 0.1, 1.0);
  marker->setPosition(1.0, 0.0, 0.3);
  marker->setBodyType(raisim::BodyType::STATIC);
  marker->setAppearance("0.9,0.6,0.1,1");

  // A pair of secondary props for visual reference (the snippet mentions
  // them indirectly through "background color + custom visuals").
  auto* prop = world->addCapsule(0.18, 0.3, 1.0);
  prop->setPosition(-0.7, 0.2, 0.28);
  prop->setBodyType(raisim::BodyType::STATIC);
  prop->setAppearance("0.2,0.5,0.9,1");

  doc_image::setCameraLookAt(viewer.getCamera(),
                             glm::vec3(2.6f, 2.2f, 1.4f),
                             glm::vec3(0.0f, 0.0f, 0.25f),
                             /*horizontalFovDeg=*/60.0f);

  raisin::RayraiWindow::RenderOverrides overrides;
  overrides.doShadows = true;

  const auto path = outputDir / "rayrai_custom_visuals.png";
  if (!doc_image::captureScene(viewer, kWidth, kHeight, path, 2, overrides))
    doc_image::finishAndExit(1);
  std::printf("doc_image: wrote %s\n", path.string().c_str());
  doc_image::finishAndExit(0);
}
