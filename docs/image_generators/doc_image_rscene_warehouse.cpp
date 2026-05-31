// Renders a richer warehouse-style scene that exercises several .rscene
// record kinds described in the .rscene File documentation page: a ground
// with a non-default material, two dynamic crates with a distinct safety-
// orange material, a warm spot light (matching the warehouse_scene.rscene
// `light /World/Lights/Key ... type=spot color=1,0.86,0.64 coneAngle=30`),
// and the procedural rayrai sky from the `environment backgroundMode=
// rayrai_sky` + `weather preset=clear` records.

#include "doc_image_common.hpp"

#include <cmath>

namespace {
constexpr int kWidth = 1280;
constexpr int kHeight = 720;

// Compose a yaw (about Z) into a unit quaternion (w,x,y,z).
inline raisim::Vec<4> yawQuat(double degrees) {
  const double half = degrees * 0.5 * M_PI / 180.0;
  raisim::Vec<4> q;
  q[0] = std::cos(half);
  q[1] = 0.0;
  q[2] = 0.0;
  q[3] = std::sin(half);
  return q;
}
}  // namespace

int main(int argc, char** argv) {
  const auto outputDir = doc_image::resolveOutputDir(argc, argv);
  doc_image::OffscreenContext gl;
  if (!gl.init("doc_image_rscene_warehouse")) doc_image::finishAndExit(1);

  auto world = std::make_shared<raisim::World>();
  raisin::RayraiWindow viewer(world, kWidth, kHeight);

  const auto preset = raisin::RayraiWindow::RenderQualityPreset::High;
  auto quality = raisin::RayraiWindow::defaultRenderQualitySettings(preset);
  quality.colorMode = raisin::ViewerColorMode::AcesApprox;
  quality.pbrToneMapping = true;
  doc_image::applyCommonSceneOptions(quality, preset);

  // `light /World/Lights/Key -0.4 0.5 -1 4 type=spot color=1,0.86,0.64`.
  quality.mainLightDirection = glm::normalize(glm::vec3(-0.4f, 0.5f, -1.0f));
  quality.mainLightDiffuse = glm::vec3(1.0f, 0.86f, 0.64f) * 2.6f;
  quality.mainLightAmbient = glm::vec3(0.18f, 0.20f, 0.24f);

  // Warm key spot + cool indirect fill so the orange crates separate from
  // the grey floor and we don't crush the back of the scene.
  quality.addViewerFillLights = false;
  quality.maxAdditionalLightsPerFrame = 4;
  viewer.setRenderQualitySettings(quality);

  raisin::RayraiWindow::AdditionalLight coolFill;
  coolFill.type = raisin::LightType::POINT;
  coolFill.position = glm::vec3(-2.5f, 2.5f, 2.0f);
  coolFill.diffuse = glm::vec3(0.35f, 0.50f, 0.85f) * 2.0f;
  coolFill.constant = 1.0f;
  coolFill.linear = 0.10f;
  coolFill.quadratic = 0.02f;
  viewer.addAdditionalLight(coolFill);

  // `object /World/Ground ground ... mat_floor` — grey floor at z=0.
  world->addGround();
  // Ground appearance comes from the renderer; the procedural sky provides
  // the rest of the IBL via doc_image::captureScene's IBL bake step.

  // Two crates matching CrateA/CrateB in warehouse_scene.rscene.
  //   CrateA: pos (-0.6, 0.2, 0.6), yaw ~8 deg, mat_safety (orange).
  //   CrateB: pos (0.7, -0.3, 1.1), yaw ~-18 deg, mat_safety.
  // The .rscene's box positional fields encode the half-extents as
  // 0.7 0.5 0.45; we recreate with full extents 1.4 x 1.0 x 0.9.
  auto* crateA = world->addBox(1.4, 1.0, 0.9, 1.0);
  crateA->setPosition(-0.6, 0.2, 0.6);
  crateA->setOrientation(yawQuat(8.0));
  crateA->setBodyType(raisim::BodyType::STATIC);
  crateA->setAppearance("0.95,0.45,0.10,1");  // mat_safety albedo.

  auto* crateB = world->addBox(1.4, 1.0, 0.9, 1.0);
  crateB->setPosition(0.7, -0.3, 1.1);
  crateB->setOrientation(yawQuat(-18.0));
  crateB->setBodyType(raisim::BodyType::STATIC);
  crateB->setAppearance("0.95,0.45,0.10,1");

  // A third smaller crate to fill the foreground.
  auto* crateC = world->addBox(0.7, 0.7, 0.7, 1.0);
  crateC->setPosition(1.6, 0.6, 0.35);
  crateC->setOrientation(yawQuat(22.0));
  crateC->setBodyType(raisim::BodyType::STATIC);
  crateC->setAppearance("0.95,0.45,0.10,1");

  // A grey pallet-like slab to break up the orange.
  auto* pallet = world->addBox(2.4, 1.6, 0.12, 1.0);
  pallet->setPosition(0.0, 0.0, 0.06);
  pallet->setBodyType(raisim::BodyType::STATIC);
  pallet->setAppearance("0.45,0.46,0.48,1");

  // `camera /World/Cameras/Editor 3 -5 2.2 ... 52` — same eye + FOV.
  doc_image::setCameraLookAt(viewer.getCamera(),
                             glm::vec3(3.5f, -5.0f, 2.4f),
                             glm::vec3(0.0f, 0.0f, 0.55f),
                             /*horizontalFovDeg=*/52.0f);

  raisin::RayraiWindow::RenderOverrides overrides;
  overrides.doShadows = true;

  const auto path = outputDir / "rayrai_rscene_warehouse.png";
  if (!doc_image::captureScene(viewer, kWidth, kHeight, path, 2, overrides))
    doc_image::finishAndExit(1);
  std::printf("doc_image: wrote %s\n", path.string().c_str());
  doc_image::finishAndExit(0);
}
