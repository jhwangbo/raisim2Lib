#include <cmath>
#include <filesystem>
#include <iostream>

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rayrai_tcp_viewer_hint.hpp"
#include "example_resources.hpp"

int main(int argc, char* argv[]) {
  raisim::World::setActivationKey(exampleRscPath(argv[0], "activation.raisim"));

  raisim::World world(exampleRscPath(argv[0], "mjcf/gymnasium/hopper.xml"));

  auto* object = world.getObject("torso");
  auto* hopper = dynamic_cast<raisim::ArticulatedSystem*>(object);
  if (!hopper) {
    std::cerr << "Failed to find MJCF articulated system 'torso'\n";
    return 1;
  }

  std::cout << "Loaded Gymnasium Hopper MJCF\n";
  std::cout << "DOF: " << hopper->getDOF() << "\n";
  std::cout << "generalized coordinate dim: " << hopper->getGeneralizedCoordinateDim() << "\n";
  std::cout << "collision bodies: " << hopper->getCollisionBodies().size() << "\n";

  hopper->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

  raisim::RaisimServer server(&world);
  server.launchServer();
  raisim_examples::warnIfNoClientConnected(server);
  server.focusOn(hopper);

  size_t step = 0;
  while (true) {
    RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))

    Eigen::VectorXd tau = Eigen::VectorXd::Zero(hopper->getDOF());
    const double phase = double(step) * world.getTimeStep();
    for (int i = 3; i < hopper->getDOF(); ++i)
      tau[i] = 25.0 * std::sin(phase * 3.0 + double(i));

    hopper->setGeneralizedForce(tau);
    server.integrateWorldThreadSafe();
    ++step;
  }

  server.killServer();
  return 0;
}
