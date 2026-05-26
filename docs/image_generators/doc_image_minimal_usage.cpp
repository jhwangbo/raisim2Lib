// Renders the canonical "Minimal usage" scene shown in the Rayrai overview:
// a coloured sphere sitting just above a ground plane, with default Balanced
// preset and a neutral background.

#include "doc_image_common.hpp"

namespace {
constexpr int kWidth = 1280;
constexpr int kHeight = 720;
}  // namespace

int main(int argc, char** argv) {
  const auto outputDir = doc_image::resolveOutputDir(argc, argv);
  doc_image::OffscreenContext gl;
  if (!gl.init("doc_image_minimal_usage")) doc_image::finishAndExit(1);

  auto world = std::make_shared<raisim::World>();
  raisin::RayraiWindow viewer(world, kWidth, kHeight);

  const auto preset = raisin::RayraiWindow::RenderQualityPreset::Balanced;
  auto quality = raisin::RayraiWindow::defaultRenderQualitySettings(preset);
  quality.colorMode = raisin::ViewerColorMode::AcesApprox;
  quality.pbrToneMapping = true;
  doc_image::applyCommonSceneOptions(quality, preset);
  viewer.setRenderQualitySettings(quality);

  world->addGround();

  // Matches the minimal-usage snippet: a sphere of radius 0.2 at (0, 0, 1)
  // in saturated red.
  auto* sphere = world->addSphere(0.2, 1.0);
  sphere->setPosition(0.0, 0.0, 1.0);
  sphere->setBodyType(raisim::BodyType::STATIC);
  sphere->setAppearance("0.9,0.2,0.2,1");

  doc_image::setCameraLookAt(viewer.getCamera(),
                             glm::vec3(2.4f, 2.4f, 1.6f),
                             glm::vec3(0.0f, 0.0f, 0.9f),
                             /*horizontalFovDeg=*/60.0f);

  raisin::RayraiWindow::RenderOverrides overrides;
  overrides.doShadows = true;

  const auto path = outputDir / "rayrai_minimal_usage.png";
  if (!doc_image::captureScene(viewer, kWidth, kHeight, path, 2, overrides))
    doc_image::finishAndExit(1);
  std::printf("doc_image: wrote %s\n", path.string().c_str());
  doc_image::finishAndExit(0);
}
