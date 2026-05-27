// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.
//
// sim_control_demo
// ----------------
// Demonstrates interactive sim control from the rayrai TCP viewer:
//   * pause / resume / step from the viewer's Control tab
//   * apply external forces / torques from the viewer (CR_APPLY_FORCE / CR_APPLY_TORQUE)
//   * set pose or generalized coordinates from the viewer (CR_SET_POSE / CR_SET_GC)
//   * loopback-only bind (default), opt-in remote access
//
// There is no authentication — any client that can reach the port has full
// control. Keep the default loopback bind unless you trust every host on the
// reachable network.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rayrai_tcp_viewer_hint.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  // Resolve the binary directory directly from argv to avoid setFromArgv mangling
  // paths containing digits.
  std::string argv0 = argv[0];
  std::string binaryDir = argv0.substr(0, argv0.find_last_of('/'));

  raisim::World world;
  world.setTimeStep(0.003);
  world.addGround();

  // A simple PD-controlled ANYmal so CR_SET_GC has something interesting to drive.
  auto anymal = world.addArticulatedSystem(
      binaryDir + "/rsc/anymal/urdf/anymal.urdf");
  Eigen::VectorXd nominal(anymal->getGeneralizedCoordinateDim()),
      target(anymal->getDOF());
  nominal << 0, 0, 0.54, 1, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03,
      -0.4, 0.8, -0.03, -0.4, 0.8;
  target.setZero();
  Eigen::VectorXd pGain(anymal->getDOF()), dGain(anymal->getDOF());
  pGain.tail(12).setConstant(200.0);
  dGain.tail(12).setConstant(10.0);
  anymal->setGeneralizedCoordinate(nominal);
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));
  anymal->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  anymal->setPdGains(pGain, dGain);
  anymal->setPdTarget(nominal, target);
  anymal->setName("anymal");

  // A free-falling box the user can poke around with the force-application UI.
  auto pokeBox = world.addBox(0.4, 0.4, 0.4, 5.0);
  pokeBox->setPosition(2.0, 0.0, 1.0);
  pokeBox->setName("poke_box");
  pokeBox->setAppearance("orange");

  raisim::RaisimServer server(&world);
  // Defaults: bound to 127.0.0.1. Opt in to external bind explicitly if needed:
  //   server.setBindLoopbackOnly(false);  // listen on all interfaces
  server.launchServer();
  raisim_examples::warnIfNoClientConnected(server);

  // Optional: pause at startup so the viewer can attach before anything happens.
  // server.pauseSimulation();

  for (int i = 0;; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))
    // The viewer can pause/step this loop and push forces/poses; no extra
    // bookkeeping needed in the example code.
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
