#pragma once

#include <string>
#include <vector>

#include "raisim/Path.hpp"

inline std::string rayraiRscPath(char* argv0, const std::string& relative) {
  auto binaryPath = raisim::Path::setFromArgv(argv0);
  const std::string sep = raisim::Path::separator();
  const std::string binaryDir = binaryPath.getDirectory().getString();
  const std::vector<std::string> candidates = {
    binaryDir + sep + "rsc" + sep + relative,
    binaryDir + sep + ".." + sep + "rsc" + sep + relative,
    binaryDir + sep + ".." + sep + ".." + sep + "rsc" + sep + relative,
    std::string("rsc") + sep + relative,
    std::string("..") + sep + "rsc" + sep + relative,
    std::string("..") + sep + ".." + sep + "rsc" + sep + relative,
  };

  for (const auto& candidate : candidates) {
    raisim::Path path(candidate);
    if (path.fileExists() || path.directoryExists())
      return path.getString();
  }

  return raisim::Path(candidates.front()).getString();
}
