#include <cmath>
#include <filesystem>
#include <iostream>

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rayrai_tcp_viewer_hint.hpp"

int main(int argc, char* argv[]) {
  const std::filesystem::path binaryDir = std::filesystem::absolute(argv[0]).parent_path();
  const std::string rscPath = (binaryDir / "rsc").string();
  raisim::World::setActivationKey(rscPath + "/activation.raisim");

  raisim::World world(rscPath + "/mjcf/gymnasium/walker2d.xml");

  auto* object = world.getObject("torso");
  auto* walker = dynamic_cast<raisim::ArticulatedSystem*>(object);
  if (!walker) {
    std::cerr << "Failed to find MJCF articulated system 'torso'\n";
    return 1;
  }

  std::cout << "Loaded Gymnasium Walker2d MJCF\n";
  std::cout << "DOF: " << walker->getDOF() << "\n";
  std::cout << "generalized coordinate dim: " << walker->getGeneralizedCoordinateDim() << "\n";
  std::cout << "collision bodies: " << walker->getCollisionBodies().size() << "\n";

  walker->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

  raisim::RaisimServer server(&world);
  server.launchServer();
  raisim_examples::warnIfNoClientConnected(server);
  server.focusOn(walker);

  size_t step = 0;
  while (true) {
    RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))

    Eigen::VectorXd tau = Eigen::VectorXd::Zero(walker->getDOF());
    const double phase = double(step) * world.getTimeStep();
    for (int i = 3; i < walker->getDOF(); ++i)
      tau[i] = 20.0 * std::sin(phase * 2.5 + 0.5 * double(i));

    walker->setGeneralizedForce(tau);
    server.integrateWorldThreadSafe();
    ++step;
  }

  server.killServer();
  return 0;
}
