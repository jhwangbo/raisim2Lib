// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rayrai_tcp_viewer_hint.hpp"

#include <Eigen/Core>

#include <sstream>
#include <stdexcept>
#include <string>

namespace {

constexpr double kCubeSize = 0.065;
constexpr double kCubeMass = 0.18;
constexpr double kCubeDropX = 0.0;
constexpr double kCubeDropY = -0.39;
constexpr double kCubeDropHeight = 1.05;
constexpr double kInteractiveGcUpdateTolerance = 1e-7;

raisim::ArticulatedSystem* findShadowHand(raisim::World& world) {
  return dynamic_cast<raisim::ArticulatedSystem*>(world.getObject("shadow_hand"));
}

std::string describeImportedObjects(raisim::World& world) {
  std::ostringstream stream;
  stream << "object count: " << world.getObjList().size();
  if (!world.getObjList().empty()) {
    stream << ", imported objects:";
    for (auto* object : world.getObjList())
      stream << " '" << object->getName() << "'";
  }
  return stream.str();
}

void setNominalPdControl(raisim::ArticulatedSystem* hand) {
  const raisim::VecDyn nominalConfig = hand->getGeneralizedCoordinate();
  Eigen::VectorXd velocityTarget = Eigen::VectorXd::Zero(hand->getDOF());
  Eigen::VectorXd pGain = Eigen::VectorXd::Constant(hand->getDOF(), 80.0);
  Eigen::VectorXd dGain = Eigen::VectorXd::Constant(hand->getDOF(), 2.0);

  hand->getSprings().clear();
  hand->setGeneralizedVelocity(velocityTarget);
  hand->setGeneralizedForce(Eigen::VectorXd::Zero(hand->getDOF()));
  hand->setPdGains(pGain, dGain);
  hand->setPdTarget(nominalConfig.e(), velocityTarget);
}

}  // namespace

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  const auto rscPath = (binaryPath.getDirectory() + "/rsc").getString();
  raisim::World::setActivationKey(rscPath + "/activation.raisim");

  const auto shadowHandUsd =
      rscPath + "/isaac/Robots/ShadowRobot/ShadowHand/shadow_hand.usd";

  raisim::World world(shadowHandUsd);
  world.setTimeStep(1.0 / 500.0);

  auto* hand = findShadowHand(world);
  if (hand == nullptr) {
    throw std::runtime_error(
        "Failed to load ShadowHand from USD '" + shadowHandUsd +
        "': no ArticulatedSystem named 'shadow_hand' was imported (" +
        describeImportedObjects(world) + ").");
  }

  hand->setBasePos({0.0, 0.0, 0.5});
  hand->setBaseOrientation(raisim::Mat<3, 3>::getIdentity());
  setNominalPdControl(hand);
  for (auto& collisionBody : hand->getCollisionBodies())
    collisionBody.setMaterial("shadow_hand");
  if (hand->getCollisionBodies().empty()) {
    throw std::runtime_error(
        "Failed to load ShadowHand from USD '" + shadowHandUsd +
        "': imported ArticulatedSystem has no collision bodies.");
  }
  if (hand->getVisOb().empty()) {
    throw std::runtime_error(
        "Failed to load ShadowHand from USD '" + shadowHandUsd +
        "': imported ArticulatedSystem has no visual meshes.");
  }

  auto* ground = world.addGround(0.0, "ground");
  ground->setAppearance("checkerboard");

  auto* cube = world.addBox(kCubeSize, kCubeSize, kCubeSize, kCubeMass, "cube");
  cube->setName("dexterous_cube");
  cube->setPosition(kCubeDropX, kCubeDropY, kCubeDropHeight);
  cube->setAppearance("0.18,0.55,0.95,0.0");

  world.setMaterialPairProp("ground", "cube", 0.8, 0.0, 0.001);
  world.setMaterialPairProp("shadow_hand", "cube", 1.1, 0.0, 0.001);

  raisim::RaisimServer server(&world);

  server.launchServer();
  raisim_examples::warnIfNoClientConnected(server);
  server.focusOn(cube);
  server.setCameraPositionAndLookAt({1.2, -1.3, 1.15}, {0.0, -0.33, 0.72});

  const Eigen::VectorXd zeroVelocityTarget =
      Eigen::VectorXd::Zero(hand->getDOF());
  Eigen::VectorXd pausedGcSnapshot(hand->getGeneralizedCoordinateDim());
  bool pausedGcSnapshotValid = false;

  for (;;) {
    RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))
    server.integrateWorldThreadSafe([&]() {
      if (!server.isSimulationPaused()) {
        pausedGcSnapshotValid = false;
        return;
      }

      const Eigen::VectorXd currentGc = hand->getGeneralizedCoordinate().e();
      if (!pausedGcSnapshotValid) {
        pausedGcSnapshot = currentGc;
        pausedGcSnapshotValid = true;
        return;
      }

      if ((currentGc - pausedGcSnapshot).cwiseAbs().maxCoeff() >
          kInteractiveGcUpdateTolerance) {
        hand->setGeneralizedVelocity(zeroVelocityTarget);
        hand->setPdTarget(currentGc, zeroVelocityTarget);
        pausedGcSnapshot = currentGc;
      }
    });
  }

  server.killServer();
  return 0;
}
