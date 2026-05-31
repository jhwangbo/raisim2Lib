// Renders the atmospheric / decal record kinds from the .rscene File
// documentation page: a `local_fog` volume tinted to evoke evening haze, plus
// a `projected_decal` of a coloured emissive sign on the floor. The
// surrounding scene is the warehouse pallet + crate layout so the new
// effects read clearly.

#include "doc_image_common.hpp"

#include <rayrai/SceneEffects.hpp>
#include <rayrai/Weather.hpp>

namespace {
constexpr int kWidth = 1280;
constexpr int kHeight = 720;
}  // namespace

int main(int argc, char** argv) {
  const auto outputDir = doc_image::resolveOutputDir(argc, argv);
  doc_image::OffscreenContext gl;
  if (!gl.init("doc_image_rscene_atmosphere")) doc_image::finishAndExit(1);

  auto world = std::make_shared<raisim::World>();
  raisin::RayraiWindow viewer(world, kWidth, kHeight);

  const auto preset = raisin::RayraiWindow::RenderQualityPreset::High;
  auto quality = raisin::RayraiWindow::defaultRenderQualitySettings(preset);
  quality.colorMode = raisin::ViewerColorMode::AcesApprox;
  quality.pbrToneMapping = true;
  doc_image::applyCommonSceneOptions(quality, preset);
  // Slightly dim main light so the local fog has visible weight.
  quality.mainLightDirection = glm::normalize(glm::vec3(-0.4f, 0.3f, -1.0f));
  quality.mainLightDiffuse = glm::vec3(1.0f, 0.86f, 0.66f) * 1.2f;
  quality.mainLightAmbient = glm::vec3(0.20f, 0.22f, 0.26f);
  // Enable volumetric fog so LocalFogVolume actually composits.
  quality.volumetricFogEnabled = true;
  quality.volumetricFogDensity = 0.18f;
  quality.volumetricLightStrength = 0.6f;
  viewer.setRenderQualitySettings(quality);

  world->addGround();

  // Pallet + crates so the fog has surfaces to wrap around.
  auto* pallet = world->addBox(2.4, 1.6, 0.12, 1.0);
  pallet->setPosition(0.0, 0.0, 0.06);
  pallet->setBodyType(raisim::BodyType::STATIC);
  pallet->setAppearance("0.45,0.46,0.48,1");

  auto* crateA = world->addBox(1.0, 0.8, 0.8, 1.0);
  crateA->setPosition(-0.5, 0.0, 0.52);
  crateA->setBodyType(raisim::BodyType::STATIC);
  crateA->setAppearance("0.95,0.45,0.10,1");

  auto* crateB = world->addBox(0.6, 0.6, 0.6, 1.0);
  crateB->setPosition(0.7, 0.2, 0.42);
  crateB->setBodyType(raisim::BodyType::STATIC);
  crateB->setAppearance("0.86,0.34,0.18,1");

  // `local_fog /World/Effects/EveningHaze ... color=0.45,0.55,0.65 density=0.6`
  // Two volumes: a big body of evening haze, and a denser puff near the
  // pallet so the local-fog effect is unambiguous on screen.
  raisin::LocalFogVolume haze;
  haze.center = glm::vec3(0.0f, 0.5f, 0.9f);
  haze.radius = 3.5f;
  haze.color = glm::vec3(0.62f, 0.70f, 0.82f);
  haze.density = 0.9f;
  haze.edgeFade = 0.35f;
  haze.noiseScale = 0.8f;
  haze.noiseStrength = 0.6f;
  viewer.addLocalFogVolume(haze);

  raisin::LocalFogVolume puff;
  puff.center = glm::vec3(-0.8f, -0.4f, 0.5f);
  puff.radius = 1.4f;
  puff.color = glm::vec3(0.80f, 0.70f, 0.55f);  // warm dust
  puff.density = 1.8f;
  puff.edgeFade = 0.45f;
  puff.noiseScale = 1.6f;
  puff.noiseStrength = 0.75f;
  viewer.addLocalFogVolume(puff);

  // `projected_decal /World/Decals/FloorWarning ...` projected onto the floor.
  raisin::ProjectedDecal decal;
  decal.center = glm::vec3(0.6f, 1.6f, 0.03f);
  decal.halfExtents = glm::vec3(1.4f, 1.0f, 0.4f);
  decal.color = glm::vec4(1.0f, 0.78f, 0.18f, 1.0f);   // warning yellow
  decal.edgeFade = 0.06f;
  decal.normalFade = 0.5f;
  decal.albedoMix = 1.0f;
  decal.emissionEnergy = 3.0f;                         // emissive so it reads
                                                       // through fog
  viewer.addProjectedDecal(decal);

  doc_image::setCameraLookAt(viewer.getCamera(),
                             glm::vec3(2.4f, -3.3f, 1.5f),
                             glm::vec3(0.0f, 0.2f, 0.5f),
                             /*horizontalFovDeg=*/55.0f);

  raisin::RayraiWindow::RenderOverrides overrides;
  overrides.doShadows = true;

  const auto path = outputDir / "rayrai_rscene_atmosphere.png";
  if (!doc_image::captureScene(viewer, kWidth, kHeight, path, 2, overrides))
    doc_image::finishAndExit(1);
  std::printf("doc_image: wrote %s\n", path.string().c_str());
  doc_image::finishAndExit(0);
}
