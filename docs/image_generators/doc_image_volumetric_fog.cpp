// Renders a scene with volumetric fog + height fog + light shafts.

#include "doc_image_common.hpp"

namespace {
constexpr int kWidth = 1280;
constexpr int kHeight = 720;
}  // namespace

int main(int argc, char** argv) {
  const auto outputDir = doc_image::resolveOutputDir(argc, argv);
  doc_image::OffscreenContext gl;
  if (!gl.init("doc_image_volumetric_fog")) doc_image::finishAndExit(1);

  auto world = std::make_shared<raisim::World>();
  raisin::RayraiWindow viewer(world, kWidth, kHeight);

  const auto preset = raisin::RayraiWindow::RenderQualityPreset::Ultra;
  auto q = raisin::RayraiWindow::defaultRenderQualitySettings(preset);
  q.colorMode = raisin::ViewerColorMode::AcesApprox;
  q.pbrToneMapping = true;
  doc_image::applyCommonSceneOptions(q, preset);
  q.fogDensity = 0.02f;
  q.heightFogEnabled = true;
  q.heightFogDensity = 0.05f;
  q.heightFogBaseHeight = 0.0f;
  q.heightFogFalloff = 0.35f;
  q.volumetricFogEnabled = true;
  q.volumetricFogDensity = 0.035f;
  q.volumetricFogColor = glm::vec3(0.80f, 0.85f, 0.95f);
  q.volumetricFogAnisotropy = 0.45f;
  q.volumetricLightingEnabled = true;
  q.volumetricLightStrength = 1.0f;
  q.volumetricLightDecay = 0.95f;
  q.volumetricLightSamples = 48;
  q.lightShaftsEnabled = true;
  q.lightShaftsStrength = 1.4f;
  q.lightShaftsDecay = 0.97f;
  q.bloomEnabled = true;
  q.bloomStrength = 0.18f;
  viewer.setRenderQualitySettings(q);
  viewer.setBackgroundColorRgb255({22, 28, 40, 255});

  world->addGround();

  // A row of pillars to slice the volumetric light into shafts.
  for (int i = -3; i <= 3; ++i) {
    auto* pillar = world->addBox(0.3, 0.3, 3.0, 1.0);
    pillar->setPosition(i * 1.1, 2.5, 1.5);
    pillar->setBodyType(raisim::BodyType::STATIC);
    pillar->setAppearance("0.40,0.42,0.45,1");
  }

  // A small subject in the front.
  auto* subject = world->addSphere(0.35, 1.0);
  subject->setPosition(0.0, 0.5, 0.35);
  subject->setBodyType(raisim::BodyType::STATIC);
  subject->setAppearance("0.85,0.65,0.20,1");

  doc_image::setCameraLookAt(viewer.getCamera(),
                             glm::vec3(2.4f, -1.4f, 1.4f),
                             glm::vec3(0.0f, 2.0f, 0.6f),
                             /*horizontalFovDeg=*/65.0f);

  raisin::RayraiWindow::RenderOverrides overrides;
  overrides.doShadows = true;

  const auto path = outputDir / "rayrai_volumetric_fog.png";
  if (!doc_image::captureScene(viewer, kWidth, kHeight, path, 2, overrides))
    doc_image::finishAndExit(1);
  std::printf("doc_image: wrote %s\n", path.string().c_str());
  doc_image::finishAndExit(0);
}
