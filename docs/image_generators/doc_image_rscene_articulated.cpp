// Renders an articulated robot loaded from URDF on the warehouse-style
// ground, illustrating the `articulated_resource` + `articulated` records in
// the .rscene File documentation page. The robot pose mimics what the
// .rscene's `generalizedCoordinate=...` field would set on load.

#include "doc_image_common.hpp"

#include <filesystem>

namespace {
constexpr int kWidth = 1280;
constexpr int kHeight = 720;
}  // namespace

int main(int argc, char** argv) {
  const auto outputDir = doc_image::resolveOutputDir(argc, argv);
  doc_image::OffscreenContext gl;
  if (!gl.init("doc_image_rscene_articulated")) doc_image::finishAndExit(1);

  auto world = std::make_shared<raisim::World>();
  raisin::RayraiWindow viewer(world, kWidth, kHeight);

  const auto preset = raisin::RayraiWindow::RenderQualityPreset::Fast;
  auto quality = raisin::RayraiWindow::defaultRenderQualitySettings(preset);
  quality.colorMode = raisin::ViewerColorMode::FastLinear;
  quality.forceSimpleMaterialShading = true;
  quality.pbrToneMapping = false;
  doc_image::applyCommonSceneOptions(quality, preset);
  quality.mainLightDirection = glm::normalize(glm::vec3(-0.3f, 0.4f, -1.0f));
  quality.mainLightDiffuse = glm::vec3(1.0f, 0.97f, 0.92f) * 1.6f;
  viewer.setRenderQualitySettings(quality);

  // `object /World/Ground ground ...`
  world->addGround();

  // `articulated_resource anymal ... rsc/anymal/urdf/anymal.urdf` ⇒
  // `articulated /World/Robots/ANYmal`. Resolve the URDF relative to the
  // repo so the generator works whether run from the build tree or the
  // install tree.
  const auto candidatePaths = std::vector<std::filesystem::path>{
      std::filesystem::path("../../../rsc/anymal/urdf/anymal.urdf"),
      std::filesystem::path("../../rsc/anymal/urdf/anymal.urdf"),
      std::filesystem::path("rsc/anymal/urdf/anymal.urdf"),
      std::filesystem::path("/home/jemin/workspace/raisim2Lib/rsc/anymal/urdf/anymal.urdf"),
  };
  std::filesystem::path urdfPath;
  for (const auto& p : candidatePaths) {
    if (std::filesystem::exists(p)) {
      urdfPath = std::filesystem::canonical(p);  // raisim's URDF reader
                                                 // rejects relative paths.
      break;
    }
  }
  if (urdfPath.empty()) {
    std::fprintf(stderr, "doc_image: could not locate anymal.urdf\n");
    doc_image::finishAndExit(1);
  }

  auto* anymal = world->addArticulatedSystem(urdfPath.string());
  // Standing pose: floating-base position + identity quat + 12 joint angles
  // (hip_aa, hip_fe, knee × 4 legs). Mirrors `generalizedCoordinate=` on the
  // `articulated` record.
  Eigen::VectorXd q(anymal->getGeneralizedCoordinateDim());
  q << 0.0, 0.0, 0.54,                 // floating-base position
       1.0, 0.0, 0.0, 0.0,             // floating-base quat (w,x,y,z)
       0.03,  0.4, -0.8,               // LF leg
      -0.03,  0.4, -0.8,               // RF leg
       0.03, -0.4,  0.8,               // LH leg
      -0.03, -0.4,  0.8;               // RH leg
  anymal->setGeneralizedCoordinate(q);

  // A couple of static props so the robot has spatial context. Placed
  // wide of the trunk so they don't occlude the ANYmal body.
  auto* boxA = world->addBox(0.3, 0.3, 0.3, 1.0);
  boxA->setPosition(-1.1, 0.7, 0.15);
  boxA->setBodyType(raisim::BodyType::STATIC);
  boxA->setAppearance("0.86,0.34,0.18,1");

  auto* boxB = world->addBox(0.35, 0.35, 0.35, 1.0);
  boxB->setPosition(1.1, -0.7, 0.175);
  boxB->setBodyType(raisim::BodyType::STATIC);
  boxB->setAppearance("0.45,0.46,0.48,1");

  doc_image::setCameraLookAt(viewer.getCamera(),
                             glm::vec3(1.15f, -1.75f, 0.9f),
                             glm::vec3(0.0f, 0.0f, 0.34f),
                             /*horizontalFovDeg=*/48.0f);

  raisin::RayraiWindow::RenderOverrides overrides;
  overrides.doShadows = true;

  const auto path = outputDir / "rayrai_rscene_articulated.png";
  if (!doc_image::captureScene(viewer, kWidth, kHeight, path, 2, overrides))
    doc_image::finishAndExit(1);
  std::printf("doc_image: wrote %s\n", path.string().c_str());
  doc_image::finishAndExit(0);
}
