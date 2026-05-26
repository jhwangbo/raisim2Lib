// Demonstrates the SSR + SSAO + contact-shadow stack with a glossy ground
// plane and a small set of props.

#include "doc_image_common.hpp"

namespace {
constexpr int kWidth = 1280;
constexpr int kHeight = 720;
}  // namespace

int main(int argc, char** argv) {
  const auto outputDir = doc_image::resolveOutputDir(argc, argv);
  doc_image::OffscreenContext gl;
  if (!gl.init("doc_image_ssr_ssao")) doc_image::finishAndExit(1);

  auto world = std::make_shared<raisim::World>();
  raisin::RayraiWindow viewer(world, kWidth, kHeight);

  const auto preset = raisin::RayraiWindow::RenderQualityPreset::Ultra;
  auto q = raisin::RayraiWindow::defaultRenderQualitySettings(preset);
  q.colorMode = raisin::ViewerColorMode::AcesApprox;
  q.pbrToneMapping = true;
  doc_image::applyCommonSceneOptions(q, preset);
  // SSR scenes get an extra-glossy ground so reflections are obvious.
  q.reflectiveGround = true;
  q.reflectiveGroundRoughness = 0.18f;
  q.reflectiveGroundMetallic = 0.55f;
  q.ssrEnabled = true;
  q.ssrStrength = 0.9f;
  q.ssrSteps = 32;
  q.ssrMaxDistance = 8.0f;
  q.screenSpaceAoEnabled = true;
  q.screenSpaceAoStrength = 0.7f;
  q.screenSpaceAoRadius = 1.4f;
  q.screenSpaceAoRadiusWorldSpace = true;
  q.screenSpaceAoSamples = 16;
  q.screenSpaceAoDenoiseEnabled = true;
  q.contactShadowsEnabled = true;
  q.contactShadowsLength = 0.20f;
  q.contactShadowsStrength = 0.7f;
  q.bloomEnabled = true;
  q.bloomStrength = 0.14f;
  viewer.setRenderQualitySettings(q);
  viewer.setBackgroundColorRgb255({28, 32, 42, 255});

  world->addGround();

  // Cluster of props near the floor to give SSR something to reflect.
  struct Spec { double x, y, z, r, g, b; std::string kind; };
  const std::vector<Spec> props = {
    {-0.8, 0.0, 0.25, 0.95, 0.43, 0.12, "box"},
    {-0.1, -0.5, 0.20, 0.18, 0.46, 0.74, "box"},
    {0.7, -0.1, 0.30, 0.92, 0.86, 0.34, "box"},
    {0.2, 0.6, 0.18, 0.66, 0.36, 0.86, "sphere"},
    {-0.6, 0.6, 0.18, 0.85, 0.85, 0.92, "sphere"},
  };
  for (size_t i = 0; i < props.size(); ++i) {
    const auto& p = props[i];
    if (p.kind == "box") {
      auto* b = world->addBox(0.4, 0.4, p.z * 2.0, 1.0);
      b->setPosition(p.x, p.y, p.z);
      b->setBodyType(raisim::BodyType::STATIC);
      b->setAppearance(std::to_string(p.r) + "," + std::to_string(p.g) + "," +
                       std::to_string(p.b) + ",1");
    } else {
      auto* s = world->addSphere(0.22, 1.0);
      s->setPosition(p.x, p.y, p.z);
      s->setBodyType(raisim::BodyType::STATIC);
      s->setAppearance(std::to_string(p.r) + "," + std::to_string(p.g) + "," +
                       std::to_string(p.b) + ",1");
    }
  }

  doc_image::setCameraLookAt(viewer.getCamera(),
                             glm::vec3(2.4f, 2.6f, 1.2f),
                             glm::vec3(0.0f, 0.0f, 0.25f),
                             /*horizontalFovDeg=*/60.0f);

  raisin::RayraiWindow::RenderOverrides overrides;
  overrides.doShadows = true;

  const auto path = outputDir / "rayrai_ssr_ssao.png";
  if (!doc_image::captureScene(viewer, kWidth, kHeight, path, 2, overrides))
    doc_image::finishAndExit(1);
  std::printf("doc_image: wrote %s\n", path.string().c_str());
  doc_image::finishAndExit(0);
}
