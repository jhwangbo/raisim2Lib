#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "rayrai/example_common.hpp"
#include "rayrai_example_compat.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  ExampleApp app;
  if (!app.init("rayrai_visual_asset_support", 1280, 720))
    return -1;

  const std::filesystem::path binaryDir = std::filesystem::absolute(argv[0]).parent_path();
  const std::filesystem::path rscDir = binaryDir / "rsc";
  const std::filesystem::path anymalUrdf = rscDir / "anymal_c" / "urdf" / "anymal.urdf";
  const std::filesystem::path ycbDir = rscDir / "ycb";

  raisim::World::setActivationKey((rscDir / "activation.raisim").string());

  auto world = std::make_shared<raisim::World>();
  world->setTimeStep(0.002);
  auto* ground = world->addGround();
  ground->setName("ground");

  auto* robot = world->addArticulatedSystem(anymalUrdf.string());
  robot->setName("anymal_c_textured_urdf");

  Eigen::VectorXd q0(robot->getGeneralizedCoordinateDim());
  q0 << 0.0, 0.0, 0.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8,
    -0.03, -0.4, 0.8;
  robot->setGeneralizedCoordinate(q0);
  robot->setGeneralizedVelocity(Eigen::VectorXd::Zero(robot->getDOF()));

  struct TexturedObject {
    raisim::ArticulatedSystem* object;
    Eigen::Vector3d position;
  };
  std::vector<TexturedObject> texturedObjects;
  const std::vector<std::string> ycbUrdfs = {
    "002_master_chef_can.urdf",
    "007_tuna_fish_can.urdf",
    "012_strawberry.urdf",
    "013_apple.urdf",
  };
  const std::vector<Eigen::Vector3d> ycbPositions = {
    {0.70, -0.26, 0.10},
    {0.76, -0.07, 0.08},
    {0.78, 0.12, 0.07},
    {0.72, 0.30, 0.08},
  };
  for (size_t i = 0; i < ycbUrdfs.size(); ++i) {
    auto* object = world->addArticulatedSystem((ycbDir / ycbUrdfs[i]).string());
    object->setName("textured_ycb_" + std::to_string(i));
    object->setBasePos(ycbPositions[i]);
    object->setGeneralizedVelocity(Eigen::VectorXd::Zero(object->getDOF()));
    texturedObjects.push_back({object, ycbPositions[i]});
  }

  auto viewer = std::make_shared<raisin::RayraiWindow>(world, 1280, 720);
  viewer->setRenderQualitySettings(raisin::RayraiWindow::defaultRenderQualitySettings(
    raisin::RayraiWindow::RenderQualityPreset::High));
  raisim_examples::setRayraiBackgroundColorRgb255(*viewer, {22, 24, 28, 255});
  raisim_examples::addRayraiPbrSceneLights(*viewer);
  viewer->setFogDensity(0.0f);
  viewer->setShadowOrtho(5.5f, 0.05f, 18.0f);
  viewer->setShadowCenterOffset(2.0f);

  auto& camera = viewer->getCamera();
  camera.target = {0.64f, -0.03f, 0.22f};
  camera.position = {1.32f, -1.02f, 0.40f};
  camera.yaw = 124.5f;
  camera.pitch = -10.5f;
  camera.setCameraFixedTarget(true);
  camera.setCameraFixedDistance(true);

  while (!app.quit) {
    app.processEvents();
    if (app.quit)
      break;

    const double t = world->getWorldTime();
    Eigen::VectorXd q = q0;
    q[3] = std::cos(0.12 * std::sin(0.5 * t));
    q[6] = std::sin(0.12 * std::sin(0.5 * t));
    robot->setGeneralizedCoordinate(q);
    robot->setGeneralizedVelocity(Eigen::VectorXd::Zero(robot->getDOF()));
    for (auto& textured : texturedObjects) {
      textured.object->setBasePos(textured.position);
      textured.object->setGeneralizedVelocity(Eigen::VectorXd::Zero(textured.object->getDOF()));
    }

    world->integrate();

    app.beginFrame();
    app.renderViewer(*viewer);
    app.endFrame();
  }

  viewer.reset();
  app.shutdown();
  return 0;
}
