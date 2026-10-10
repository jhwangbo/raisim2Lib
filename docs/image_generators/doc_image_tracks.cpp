// Renders the tracked robot of the templated_tracked_robot example for the Tracks page and the
// example's page: the robot climbing a step with its flippers raised, and the example's scene.

#include "doc_image_common.hpp"

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {

constexpr int kWidth = 960;
constexpr int kHeight = 600;

/// the example's robot with the example's parameters
raisim::ArticulatedSystem* addTrackedRobot(raisim::World& world, const std::filesystem::path& rsc) {
  const std::vector<raisim::World::ParameterContainer> params = {
      {"rho", "500"}, {"base_length", "1.0"}, {"base_width", "0.15"}, {"wheel_y_offset", "0.14"},
      {"track_y_offset", "0.24"}, {"wheel_width", "0.08"}, {"wheel_radius", "0.17775"},
      {"flipper_radius", "0.16"}, {"track_thickness", "0.015"}};
  auto* robot = world.addArticulatedSystem((rsc / "templatedTrackedRobot/trackedTemplate.urdf").string(), params);
  robot->setBasePos({0., 0., 0.25});
  // the example's controller: sprockets at 5 rad/s, flippers raised 0.6 rad
  Eigen::VectorXd kp = Eigen::VectorXd::Zero(robot->getDOF()), kd = kp, velocity = kp;
  Eigen::VectorXd position = Eigen::VectorXd::Zero(robot->getGeneralizedCoordinateDim());
  for (const char* sprocket : {"left_sprocket", "right_sprocket"}) {
    kd[robot->getGeneralizedVelocityIndex(sprocket)] = 50.;
    velocity[robot->getGeneralizedVelocityIndex(sprocket)] = 5.;
  }
  for (const char* flipper : {"left_flipper_joint", "right_flipper_joint"}) {
    kp[robot->getGeneralizedVelocityIndex(flipper)] = 250.;
    kd[robot->getGeneralizedVelocityIndex(flipper)] = 50.;
    position[robot->getJoint(flipper).getIdxInGeneralizedCoordinate()] = -0.6;
  }
  robot->setPdGains(kp, kd);
  robot->setPdTarget(position, velocity);
  return robot;
}

struct Renderer {
  std::shared_ptr<raisim::World> world = std::make_shared<raisim::World>();
  std::unique_ptr<raisin::RayraiWindow> window;

  Renderer() {
    world->setTimeStep(0.001);
    world->addGround(0., "ground")->setAppearance("checkerboard");
    window = std::make_unique<raisin::RayraiWindow>(world, kWidth, kHeight);
    const auto preset = raisin::RayraiWindow::RenderQualityPreset::High;
    auto quality = raisin::RayraiWindow::defaultRenderQualitySettings(preset);
    quality.colorMode = raisin::ViewerColorMode::AcesApprox;
    quality.pbrToneMapping = true;
    doc_image::applyCommonSceneOptions(quality, preset);
    window->setRenderQualitySettings(quality);
  }

  raisim::Box* step(double x, double height) {
    auto* box = world->addBox(1., 3., height, 100.);
    box->setPosition(x, 0., 0.5 * height);
    box->setBodyType(raisim::BodyType::STATIC);
    box->setAppearance("0.42 0.45 0.5 1");
    return box;
  }

  bool capture(const std::filesystem::path& path, const glm::vec3& eye, const glm::vec3& target, float fov) {
    doc_image::setCameraLookAt(window->getCamera(), eye, target, fov);
    raisin::RayraiWindow::RenderOverrides overrides;
    overrides.doShadows = true;
    if (!doc_image::captureScene(*window, kWidth, kHeight, path, 2, overrides)) return false;
    std::printf("doc_image: wrote %s\n", path.string().c_str());
    return true;
  }
};

/// drives the robot at a 0.3 m step until its flippers are on top and its front is lifted
bool renderClimb(const std::filesystem::path& out, const std::filesystem::path& rsc) {
  Renderer renderer;
  renderer.step(1.75, 0.3);
  auto* robot = addTrackedRobot(*renderer.world, rsc);
  for (int i = 0; i < 4000 && robot->getBasePosition()[0] < 0.95; ++i) renderer.world->integrate();
  const auto base = robot->getBasePosition();
  const glm::vec3 target(float(base[0]) + 0.15f, float(base[1]), float(base[2]) + 0.05f);
  return renderer.capture(out / "tracked_robot_climb.png", target + glm::vec3(0.9f, -2.1f, 0.75f), target, 40.f);
}

/// the example's scene: the robot in front of its row of steps
bool renderExample(const std::filesystem::path& out, const std::filesystem::path& rsc) {
  Renderer renderer;
  for (int i = 0; i < 5; ++i) renderer.step(2. + 2. * i, i % 2 == 0 ? 0.3 : 0.2);
  auto* robot = addTrackedRobot(*renderer.world, rsc);
  for (int i = 0; i < 300; ++i) renderer.world->integrate();
  const auto base = robot->getBasePosition();
  const glm::vec3 target(float(base[0]) + 1.7f, float(base[1]), 0.2f);
  return renderer.capture(out / "templated_tracked_robot.png", target + glm::vec3(-2.3f, -3.9f, 2.1f), target, 45.f);
}

}  // namespace

int main(int argc, char** argv) {
  const auto outputDir = doc_image::resolveOutputDir(argc, argv);
  // the images go to rsc/docs/image/rayrai; the models are in rsc/ (or the second argument)
  const auto rsc = argc >= 3 ? std::filesystem::path(argv[2])
                             : std::filesystem::weakly_canonical(outputDir / ".." / ".." / "..");
  if (std::filesystem::exists(rsc / "activation.raisim"))
    raisim::World::setActivationKey((rsc / "activation.raisim").string());
  doc_image::OffscreenContext gl;
  if (!gl.init("doc_image_tracks")) doc_image::finishAndExit(1);
  const bool ok = renderClimb(outputDir, rsc) && renderExample(outputDir, rsc);
  doc_image::finishAndExit(ok ? 0 : 1);
}
