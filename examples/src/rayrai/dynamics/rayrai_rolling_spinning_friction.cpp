#include <algorithm>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "rayrai/example_common.hpp"
#include "rayrai_example_compat.hpp"
#include "raisim/World.hpp"

namespace {

struct DemoBody {
  raisim::SingleBodyObject* object = nullptr;
  std::string activeAppearance;
  bool sleeping = false;
  bool sphereShape = false;
  int x = 0;
  int y = 0;
};

int readIntArg(int argc, char** argv, const char* name, int fallback) {
  const std::string prefix = std::string(name) + "=";
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == name && i + 1 < argc)
      return std::max(1, std::atoi(argv[++i]));
    if (arg.rfind(prefix, 0) == 0)
      return std::max(1, std::atoi(arg.c_str() + prefix.size()));
  }
  return fallback;
}

void resetBody(raisim::World& world, DemoBody& body, int grid) {
  const double spacing = 1.1;
  const double origin = -0.5 * spacing * double(grid - 1);
  const int x = body.x;
  const int y = body.y;

  if (body.sphereShape) {
    body.object->setPosition(origin + spacing * x, origin + spacing * y, 0.25);
    body.object->setOrientation(1.0, 0.0, 0.0, 0.0);
    body.object->setVelocity(1.7 + 0.08 * x, 0.4 + 0.05 * y, 0.0, 0.0, -11.0, 18.0);
  } else if ((x & 1) == 0) {
    // Cylinder axis lies along world X, so angular velocity about X visibly rolls it along Y.
    body.object->setOrientation(0.7071067812, 0.0, 0.7071067812, 0.0);
    body.object->setPosition(origin + spacing * x, origin + spacing * y, 0.22);
    body.object->setVelocity(0.15 + 0.04 * x, 1.9 + 0.07 * y, 0.0, 14.0, 0.0, 0.0);
  } else {
    // Cylinder axis lies along world Y, so angular velocity about Y visibly rolls it along X.
    body.object->setOrientation(0.7071067812, -0.7071067812, 0.0, 0.0);
    body.object->setPosition(origin + spacing * x, origin + spacing * y, 0.22);
    body.object->setVelocity(1.9 + 0.07 * x, 0.15 + 0.04 * y, 0.0, 0.0, -14.0, 0.0);
  }

  body.sleeping = false;
  body.object->setAppearance(body.activeAppearance);
  world.wakeObject(body.object);
}

std::vector<DemoBody> createBodies(raisim::World& world, int grid) {
  std::vector<DemoBody> bodies;
  bodies.reserve(static_cast<size_t>(grid * grid));

  for (int y = 0; y < grid; ++y) {
    for (int x = 0; x < grid; ++x) {
      const bool sphereShape = ((x + y) & 1) == 0;
      DemoBody body;
      body.sphereShape = sphereShape;
      body.x = x;
      body.y = y;
      body.activeAppearance = sphereShape ? "0.95, 0.42, 0.12, 1.0" : "0.15, 0.70, 0.28, 1.0";
      if (sphereShape) {
        body.object = world.addSphere(0.25, 1.0, "body");
      } else {
        body.object = world.addCylinder(0.22, 0.55, 1.0, "body");
      }
      body.object->setAppearance(body.activeAppearance);
      bodies.push_back(body);
      resetBody(world, bodies.back(), grid);
    }
  }

  return bodies;
}

void resetDemo(raisim::World& world, std::vector<DemoBody>& bodies, int grid) {
  world.setWorldTime(0.0);
  for (auto& body : bodies)
    resetBody(world, body, grid);
  world.wakeAll();
}

void updateSleepingAppearance(raisim::World& world, std::vector<DemoBody>& bodies) {
  for (auto& body : bodies) {
    const bool sleeping = world.isObjectSleeping(body.object);
    if (sleeping != body.sleeping) {
      body.object->setAppearance(sleeping ? "0.20, 0.55, 1.00, 1.0" : body.activeAppearance);
      body.sleeping = sleeping;
    }
  }
}

void computeMeanSpeeds(const std::vector<DemoBody>& bodies, double& linearSpeed, double& angularSpeed) {
  linearSpeed = 0.0;
  angularSpeed = 0.0;
  if (bodies.empty())
    return;

  for (const auto& body : bodies) {
    linearSpeed += body.object->getLinearVelocity().norm();
    angularSpeed += body.object->getAngularVelocity().norm();
  }

  linearSpeed /= double(bodies.size());
  angularSpeed /= double(bodies.size());
}

}  // namespace

int main(int argc, char* argv[]) {
  const int grid = readIntArg(argc, argv, "--grid", 10);
  const int cycleSteps = readIntArg(argc, argv, "--steps", 2500);
  const int stepsPerFrame = readIntArg(argc, argv, "--steps-per-frame", 16);
  const int holdFrames = readIntArg(argc, argv, "--hold-frames", 120);

  ExampleApp app;
  if (!app.init("rayrai_rolling_spinning_friction", 1280, 720))
    return -1;

  auto world = std::make_shared<raisim::World>();
  world->setTimeStep(0.001);
  world->setERP(0.0, 0.0);
  world->setContactSolverParam(1.0, 1.0, 1.0, 120, 1e-10);
  world->setGravity({0.0, 0.0, -9.81});
  world->setSleepingEnabled(true);
  world->setSleepingParameters(0.012, 0.03, 25);
  auto* ground = world->addGround(0.0, "ground");
  ground->setAppearance("checkerboard");
  world->setMaterialPairProp("ground",
                             "body",
                             1.0,
                             0.0,
                             0.0,
                             1.0,
                             1e-3,
                             0.12,
                             0.08);

  std::vector<DemoBody> bodies = createBodies(*world, grid);

  auto viewer = std::make_shared<raisin::RayraiWindow>(world, 1280, 720);
  viewer->setRenderQualitySettings(raisin::RayraiWindow::defaultRenderQualitySettings(
    raisin::RayraiWindow::RenderQualityPreset::Balanced));
  raisim_examples::setRayraiBackgroundColorRgb255(*viewer, {24, 26, 30, 255});
  raisim_examples::addRayraiBasicSceneLights(*viewer);

  auto& camera = viewer->getCamera();
  camera.position = glm::vec3(6.0f, -8.5f, 5.0f);
  camera.target = glm::vec3(0.0f, 0.0f, 0.25f);
  camera.yaw = 125.0f;
  camera.pitch = -31.0f;
  camera.zoom = 42.0f;
  camera.zNear = 0.03f;
  camera.zFar = 35.0f;
  camera.setCameraFixedTarget(true);
  camera.setCameraFixedDistance(true);
  camera.update(false);

  int stepInCycle = 0;
  int heldFrames = 0;
  std::size_t contactTotal = 0;
  std::size_t lastContacts = 0;
  double meanLinearSpeed = 0.0;
  double meanAngularSpeed = 0.0;

  while (!app.quit) {
    app.processEvents();
    if (app.quit)
      break;

    if (stepInCycle < cycleSteps) {
      const int frameSteps = std::min(stepsPerFrame, cycleSteps - stepInCycle);
      for (int i = 0; i < frameSteps; ++i) {
        world->integrate();
        lastContacts = world->getContactProblem()->size();
        contactTotal += lastContacts;
        ++stepInCycle;
      }
      heldFrames = 0;
    } else if (++heldFrames >= holdFrames) {
      resetDemo(*world, bodies, grid);
      stepInCycle = 0;
      heldFrames = 0;
      contactTotal = 0;
      lastContacts = 0;
    }

    updateSleepingAppearance(*world, bodies);
    computeMeanSpeeds(bodies, meanLinearSpeed, meanAngularSpeed);

    app.beginFrame();
    app.renderViewer(*viewer);

    ImGui::SetNextWindowPos(ImVec2(12, 12), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.72f);
    ImGui::Begin("Rolling friction", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
                   ImGuiWindowFlags_NoCollapse);
    ImGui::Text("step %d / %d", stepInCycle, cycleSteps);
    ImGui::Text("bodies: %zu", bodies.size());
    ImGui::Text("contacts: %zu", lastContacts);
    ImGui::Text("average contacts: %.2f",
                stepInCycle > 0 ? double(contactTotal) / double(stepInCycle) : 0.0);
    ImGui::Text("mean linear speed: %.3f m/s", meanLinearSpeed);
    ImGui::Text("mean angular speed: %.3f rad/s", meanAngularSpeed);
    ImGui::End();

    app.endFrame();
  }

  viewer.reset();
  app.shutdown();
  return 0;
}
