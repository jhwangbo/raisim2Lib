// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include <iostream>
#if WIN32
#include <timeapi.h>
#endif

int main(int, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "/rsc/activation.raisim");

  const std::string xmlScript = "";

  if (xmlScript.empty()) {
    std::cout << "drop in the script in rsc/xmlScripts to simulate" << std::endl;
    return 0;
  }

  raisim::World world(binaryPath.getDirectory() + "/rsc/xmlScripts/" + xmlScript);
  raisim::RaisimServer server(&world);
  server.launchServer();
  for (int i=0; i<10000000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
