// Copyright (c) 2026 Raion Robotics Inc. All rights reserved.
#include "TcpViewerInspectorLoad.hpp"

#include <algorithm>
#include <functional>
#include <stdexcept>

#include "raisim/World.hpp"
#include "raisim/raisim_message_logger.hpp"
#include "TcpViewerSimulation.hpp"

namespace raisin::tcp_viewer {

InspectorModel loadInspectorModel(raisim::World& world, const std::string& path) {
  InspectorModel model;
  model.mjcf = xmlRootElement(path) == "mujoco";
  std::vector<raisim::Object*> before(world.getObjList().begin(), world.getObjList().end());
  std::sort(before.begin(), before.end(), std::less<>());
  const auto added = [&] {
    std::vector<raisim::Object*> objects;
    for (raisim::Object* object : world.getObjList())
      if (!std::binary_search(before.begin(), before.end(), object, std::less<>())) objects.push_back(object);
    return objects;
  };
  // Articulated systems go first, as when the inspector closes a model, so
  // their references to the other objects clear before those go.
  const auto remove = [&](std::vector<raisim::Object*> objects) {
    std::stable_partition(objects.begin(), objects.end(), [](raisim::Object* object) {
      return object->getObjectType() == raisim::ObjectType::ARTICULATED_SYSTEM;
    });
    for (raisim::Object* object : objects) world.removeObject(object);
  };
  try {
    // RaiSim's fatal errors would otherwise end the viewer.
    const raisim::ScopedRaiSimFatalCallback fatal(
        [] { throw std::runtime_error("RaiSim could not load the file; see the console"); });
    if (model.mjcf) {
      world.loadMjcfFile(path);
    } else {
      raisim::ArticulatedSystemOption options;
      options.convexifyCollisionMeshes = true;
      model.system = world.addArticulatedSystem(path, "", {}, raisim::CollisionGroup(1),
                                                raisim::CollisionGroup(-1), options);
    }
  } catch (...) {
    // A file that fails part way, such as an MJCF file whose world geoms
    // loaded before one of its bodies failed, keeps none of what it added.
    remove(added());
    throw;
  }
  const auto objects = added();
  // An MJCF file's model is the first articulated system it added.
  for (raisim::Object* object : objects)
    if (!model.system && object->getObjectType() == raisim::ObjectType::ARTICULATED_SYSTEM)
      model.system = static_cast<raisim::ArticulatedSystem*>(object);
  if (!model.system) {
    remove(objects);
    return model;
  }
  for (raisim::Object* object : objects)
    if (object != model.system) model.sideObjects.push_back(object);
  return model;
}

} // namespace raisin::tcp_viewer
