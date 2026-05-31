// Renders the "Minimal Example, Top To Bottom" scene shown in the .rscene
// File documentation page: ground, one dynamic box, a single directional
// light, and a perspective camera. The scene is authored directly with
// raisim + rayrai (instead of going through raisim_engine2's loader) but
// mirrors the records in the .rscene snippet 1:1.

#include "doc_image_common.hpp"

namespace {
constexpr int kWidth = 1280;
constexpr int kHeight = 720;
}  // namespace

int main(int argc, char** argv) {
  const auto outputDir = doc_image::resolveOutputDir(argc, argv);
  doc_image::OffscreenContext gl;
  if (!gl.init("doc_image_rscene_minimal")) doc_image::finishAndExit(1);

  auto world = std::make_shared<raisim::World>();
  raisin::RayraiWindow viewer(world, kWidth, kHeight);

  const auto preset = raisin::RayraiWindow::RenderQualityPreset::Balanced;
  auto quality = raisin::RayraiWindow::defaultRenderQualitySettings(preset);
  quality.colorMode = raisin::ViewerColorMode::AcesApprox;
  quality.pbrToneMapping = true;
  doc_image::applyCommonSceneOptions(quality, preset);
  // Mirror the .rscene's `light /World/MainLight ... intensity 3.5
  // type=directional color=1,0.97,0.92 shadows=true shadowResolution=2048`.
  quality.mainLightDirection = glm::normalize(glm::vec3(-0.3f, 0.4f, -1.0f));
  quality.mainLightDiffuse = glm::vec3(1.0f, 0.97f, 0.92f) * 1.6f;
  viewer.setRenderQualitySettings(quality);

  // `object /World/Ground ground ... 20 20 ... mat_box`.
  world->addGround();

  // `object /World/Props/CrateA box 0 0 0.5 1 0 0 0 1 1 1 ... mat_box dynamic`.
  // The mat_box record has albedo 0.7,0.3,0.2 and roughness 0.6, which the
  // setAppearance string preserves.
  auto* crate = world->addBox(1.0, 1.0, 1.0, 1.0);
  crate->setPosition(0.0, 0.0, 0.5);
  crate->setBodyType(raisim::BodyType::STATIC);
  crate->setAppearance("0.7,0.3,0.2,1");

  // `camera /World/Cam 3 -5 2 ...` — same eye point and FOV.
  doc_image::setCameraLookAt(viewer.getCamera(),
                             glm::vec3(3.0f, -5.0f, 2.0f),
                             glm::vec3(0.0f, 0.0f, 0.5f),
                             /*horizontalFovDeg=*/60.0f);

  raisin::RayraiWindow::RenderOverrides overrides;
  overrides.doShadows = true;

  const auto path = outputDir / "rayrai_rscene_minimal.png";
  if (!doc_image::captureScene(viewer, kWidth, kHeight, path, 2, overrides))
    doc_image::finishAndExit(1);
  std::printf("doc_image: wrote %s\n", path.string().c_str());
  doc_image::finishAndExit(0);
}
