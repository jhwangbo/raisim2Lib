// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rayrai_tcp_viewer_hint.hpp"
#include "example_resources.hpp"

int main(int argc, char* argv[]) {

  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.0001);

  /// create objects
  world.addGround();
  auto revAndPrisSpringAndDamper = world.addArticulatedSystem(exampleRscPath(argv[0], "springDamper/cartpole.urdf"));
  auto ballSpringAndDamper = world.addArticulatedSystem(exampleRscPath(argv[0], "springDamper/chainSpringed.urdf"));

  revAndPrisSpringAndDamper->setName("rev_pris_joint");
  ballSpringAndDamper->setName("ball_joint");

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  raisim_examples::warnIfNoClientConnected(server);
  server.focusOn(revAndPrisSpringAndDamper);

  for (int i=0; i<2000000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
