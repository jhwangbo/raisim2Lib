#pragma once

#include <string>
#include <vector>

#include "raisim/Path.hpp"

/// Path of rsc/<relative>, a file or folder in the examples' resource
/// directory. CMake copies rsc next to the example executables (bin/rsc on
/// Windows), or into examples/rsc when the examples folder is configured on
/// its own. It is also found one or two folders above the executables, or
/// relative to the working directory. Returns the first candidate when none
/// exists, so the caller's error names it.
inline std::string exampleRscPath(char* argv0, const std::string& relative) {
  const std::string sep = raisim::Path::separator();
  const std::string binaryDir = raisim::Path::setFromArgv(argv0).getDirectory().getString();
  const std::vector<std::string> candidates = {
    binaryDir + sep + "rsc" + sep + relative,
    binaryDir + sep + "examples" + sep + "rsc" + sep + relative,
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
