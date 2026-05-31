// Renders a `terrain_region` example: a procedural Perlin heightmap with a
// couple of crates on top, matching the doc's terrain_region snippet
// (procedural source kind with fractal octaves, lacunarity, gain). The
// heightmap is generated through raisim's TerrainProperties Perlin path,
// which is what `.rscene` stores via `sourceKind=procedural` plus the
// `procedural*` keys.

#include "doc_image_common.hpp"

#include <raisim/Terrain.hpp>

namespace {
constexpr int kWidth = 1280;
constexpr int kHeight = 720;
}  // namespace

int main(int argc, char** argv) {
  const auto outputDir = doc_image::resolveOutputDir(argc, argv);
  doc_image::OffscreenContext gl;
  if (!gl.init("doc_image_rscene_terrain")) doc_image::finishAndExit(1);

  auto world = std::make_shared<raisim::World>();
  raisin::RayraiWindow viewer(world, kWidth, kHeight);

  const auto preset = raisin::RayraiWindow::RenderQualityPreset::High;
  auto quality = raisin::RayraiWindow::defaultRenderQualitySettings(preset);
  quality.colorMode = raisin::ViewerColorMode::AcesApprox;
  quality.pbrToneMapping = true;
  doc_image::applyCommonSceneOptions(quality, preset);
  quality.mainLightDirection = glm::normalize(glm::vec3(-0.3f, 0.4f, -1.0f));
  quality.mainLightDiffuse = glm::vec3(1.0f, 0.96f, 0.86f) * 1.4f;
  viewer.setRenderQualitySettings(quality);

  // `terrain_region /World/Terrain/Field 41 41 12 12 0 0 0 sourceKind=procedural
  //  proceduralFrequency=0.18 proceduralZScale=0.9 proceduralOctaves=5 ...`.
  raisim::TerrainProperties tp(/*frequency=*/0.18,
                               /*xSize=*/12.0,
                               /*ySize=*/12.0,
                               /*zScale=*/0.9,
                               /*xSamples=*/41,
                               /*ySamples=*/41,
                               /*fractalOctaves=*/5,
                               /*fractalLacunarity=*/2.0,
                               /*fractalGain=*/0.5,
                               /*stepSize=*/0.0,
                               /*heightOffset=*/0.0,
                               /*seed=*/5489u);
  auto* terrain = world->addHeightMap(/*centerX=*/0.0, /*centerY=*/0.0, tp);
  terrain->setAppearance("0.55,0.50,0.42,1");  // dry-earth tone, matches
                                               // terrain_texture slot=1 Rock.

  // A few crates on the terrain to give scale and contact-shadow contrast.
  auto* crateA = world->addBox(0.7, 0.7, 0.7, 1.0);
  crateA->setPosition(-1.4, 0.6, 1.0);
  crateA->setBodyType(raisim::BodyType::STATIC);
  crateA->setAppearance("0.95,0.45,0.10,1");

  auto* crateB = world->addBox(0.6, 0.6, 0.6, 1.0);
  crateB->setPosition(1.3, -0.4, 0.95);
  crateB->setBodyType(raisim::BodyType::STATIC);
  crateB->setAppearance("0.86,0.34,0.18,1");

  auto* boulder = world->addSphere(0.4, 1.0);
  boulder->setPosition(0.0, 1.6, 1.2);
  boulder->setBodyType(raisim::BodyType::STATIC);
  boulder->setAppearance("0.55,0.55,0.55,1");

  doc_image::setCameraLookAt(viewer.getCamera(),
                             glm::vec3(5.2f, -6.0f, 4.2f),
                             glm::vec3(0.0f, 0.0f, 0.5f),
                             /*horizontalFovDeg=*/55.0f);

  raisin::RayraiWindow::RenderOverrides overrides;
  overrides.doShadows = true;

  const auto path = outputDir / "rayrai_rscene_terrain.png";
  if (!doc_image::captureScene(viewer, kWidth, kHeight, path, 2, overrides))
    doc_image::finishAndExit(1);
  std::printf("doc_image: wrote %s\n", path.string().c_str());
  doc_image::finishAndExit(0);
}
