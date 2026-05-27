// Renders a cinematic composition: three props at increasing depth with DoF
// focused on the middle one, plus bloom / vignette / chromatic aberration /
// film grain / lens flare / letterbox.

#include "doc_image_common.hpp"

namespace {
constexpr int kWidth = 1280;
constexpr int kHeight = 720;
}  // namespace

int main(int argc, char** argv) {
  const auto outputDir = doc_image::resolveOutputDir(argc, argv);
  doc_image::OffscreenContext gl;
  if (!gl.init("doc_image_cinematic")) doc_image::finishAndExit(1);

  auto world = std::make_shared<raisim::World>();
  raisin::RayraiWindow viewer(world, kWidth, kHeight);

  const auto preset = raisin::RayraiWindow::RenderQualityPreset::Ultra;
  auto q = raisin::RayraiWindow::defaultRenderQualitySettings(preset);
  q.colorMode = raisin::ViewerColorMode::AcesApprox;
  q.pbrToneMapping = true;
  doc_image::applyCommonSceneOptions(q, preset);
  q.viewerColorGradePreset = raisin::ColorGradePreset::Cinematic;
  q.viewerColorGradeStrength = 0.85f;
  q.bloomEnabled = true;
  q.bloomThreshold = 1.10f;
  q.bloomStrength = 0.22f;
  q.depthOfFieldEnabled = true;
  q.depthOfFieldFocusDistance = 4.0f;  // metres from camera
  q.depthOfFieldFocusRange = 0.9f;
  q.depthOfFieldMaxRadius = 1.3f;
  q.viewerVignetteStrength = 0.40f;
  q.viewerChromaticAberrationStrength = 0.10f;
  q.viewerFilmGrainStrength = 0.05f;
  q.lensFlareGhostStrength = 0.45f;
  q.lensFlareStreakStrength = 0.30f;
  q.starburstEnabled = true;
  q.starburstStrength = 0.40f;
  q.letterboxEnabled = true;
  q.letterboxAspect = 2.35f;
  viewer.setRenderQualitySettings(q);
  viewer.setBackgroundColorRgb255({18, 22, 30, 255});

  world->addGround();

  // Three props along the +y axis: near (1.6 m), focus (4 m), far (8 m).
  auto* near = world->addBox(0.4, 0.4, 0.5, 1.0);
  near->setPosition(0.0, 1.6, 0.25);
  near->setBodyType(raisim::BodyType::STATIC);
  near->setAppearance("0.95,0.43,0.12,1");  // saturated orange

  auto* mid = world->addSphere(0.4, 1.0);
  mid->setPosition(0.0, 4.0, 0.4);
  mid->setBodyType(raisim::BodyType::STATIC);
  mid->setAppearance("0.92,0.92,0.96,1");   // near-white sphere (catches bloom)

  auto* far = world->addCapsule(0.25, 0.5, 1.0);
  far->setPosition(0.0, 8.0, 0.5);
  far->setBodyType(raisim::BodyType::STATIC);
  far->setAppearance("0.18,0.46,0.74,1");

  // A bright emissive marker behind the focus to drive bloom / lens flare.
  auto* glow = world->addSphere(0.18, 1.0);
  glow->setPosition(0.9, 4.0, 1.4);
  glow->setBodyType(raisim::BodyType::STATIC);
  glow->setAppearance("3.0,2.4,1.8,1");     // > 1.0 luminance -> bloom

  doc_image::setCameraLookAt(viewer.getCamera(),
                             glm::vec3(0.0f, 0.0f, 0.9f),
                             glm::vec3(0.0f, 4.0f, 0.4f),
                             /*horizontalFovDeg=*/55.0f);

  raisin::RayraiWindow::RenderOverrides overrides;
  overrides.doShadows = true;

  const auto path = outputDir / "rayrai_cinematic.png";
  if (!doc_image::captureScene(viewer, kWidth, kHeight, path, 2, overrides))
    doc_image::finishAndExit(1);
  std::printf("doc_image: wrote %s\n", path.string().c_str());
  doc_image::finishAndExit(0);
}
