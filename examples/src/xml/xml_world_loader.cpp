// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include <iostream>
#if WIN32
#include <timeapi.h>
#endif

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey(binaryPath.getDirectory() + "/rsc/activation.raisim");

  if (argc > 1 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
    std::cout << "Usage: " << argv[0] << " [XML_WORLD]\n"
              << "Loads objects/SingleBodies.xml by default. Relative paths are also searched "
                 "under rsc/xmlScripts."
              << std::endl;
    return 0;
  }

  const std::string xmlScript = argc > 1 ? argv[1] : "objects/SingleBodies.xml";
  raisim::Path xmlPath(xmlScript);
  if (!xmlPath.fileExists()) {
    xmlPath = binaryPath.getDirectory() + "/rsc/xmlScripts/" + xmlScript;
  }
  if (!xmlPath.fileExists()) {
    std::cerr << "Could not find XML world: " << xmlScript << std::endl;
    return 1;
  }

  std::cout << "Loading RaiSim world: " << xmlPath.getString() << std::endl;
  raisim::World world(xmlPath.getString());
  raisim::RaisimServer server(&world);
  server.launchServer();
  for (int i=0; i<10000000; i++) {
    RS_TIMED_LOOP(int(world.getTimeStep()*1e6))
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
