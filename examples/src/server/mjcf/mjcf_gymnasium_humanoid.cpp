#include <filesystem>
#include <iostream>

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rayrai_tcp_viewer_hint.hpp"

int main(int argc, char* argv[]) {
  const std::filesystem::path binaryDir = std::filesystem::absolute(argv[0]).parent_path();
  const std::string rscPath = (binaryDir / "rsc").string();
  raisim::World::setActivationKey(rscPath + "/activation.raisim");

  raisim::World world(rscPath + "/mjcf/gymnasium/humanoid.xml");

  auto* object = world.getObject("torso");
  auto* humanoid = dynamic_cast<raisim::ArticulatedSystem*>(object);
  if (!humanoid) {
    std::cerr << "Failed to find MJCF articulated system 'torso'\n";
    return 1;
  }

  std::cout << "Loaded Gymnasium Humanoid MJCF\n";
  std::cout << "DOF: " << humanoid->getDOF() << "\n";
  std::cout << "generalized coordinate dim: " << humanoid->getGeneralizedCoordinateDim() << "\n";
  std::cout << "collision bodies: " << humanoid->getCollisionBodies().size() << "\n";

  Eigen::VectorXd gc = humanoid->getGeneralizedCoordinate().e();
  Eigen::VectorXd gv = Eigen::VectorXd::Zero(humanoid->getDOF());
  if (gc.size() >= 24) {
    gc[0] = 0.0;
    gc[1] = 0.0;
    gc[2] = 2.4;
    gc[3] = 0.9914449;
    gc[4] = 0.0;
    gc[5] = 0.1305262;
    gc[6] = 0.0;

    gc[7] = 0.15;
    gc[8] = -0.25;
    gc[9] = 0.12;
    gc[10] = -0.18;
    gc[11] = 0.22;
    gc[12] = -0.45;
    gc[13] = 0.35;
    gc[14] = 0.18;
    gc[15] = -0.2;
    gc[16] = -0.35;
    gc[17] = -0.55;
    gc[18] = 0.35;
    gc[19] = -0.4;
    gc[20] = 0.3;
    gc[21] = -0.45;
    gc[22] = 0.4;
    gc[23] = -0.25;

    humanoid->setGeneralizedCoordinate(gc);
    humanoid->setGeneralizedVelocity(gv);
    std::cout << "initial base height: " << gc[2] << "\n";
    std::cout << "drop clearance above standing pose: about 1 m\n";
  }

  humanoid->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);

  raisim::RaisimServer server(&world);
  server.launchServer();
  raisim_examples::warnIfNoClientConnected(server);
  server.focusOn(humanoid);

  size_t step = 0;
  while (true) {
    RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))

    humanoid->setGeneralizedForce(Eigen::VectorXd::Zero(humanoid->getDOF()));
    server.integrateWorldThreadSafe();
    ++step;
  }

  server.killServer();
  return 0;
}
