#pragma once
// The race venue of rayrai_offroad_buggy around the circuit of
// offroad_buggy_scene.hpp: the start/finish gantry, barriers along the start
// straight, tyre walls and hay bales on the outside of the corners, cones at
// the apexes, marker posts along the edges in between, the pit lane with
// canopies and the teams' gear, the paddock, and the spectator area with
// bleachers, fence and flags.
//
// Fixed props get static colliders from their .rasset files. Hay bales, cones
// and marker posts are dynamic, so the car can knock them over: a bale is a
// box, a cone or post the convex hull of a small proxy mesh, each drawn with
// its visual mesh at the body's pose. They sleep as static bodies while the
// car is away (updateLooseProps()). The cones, tool carts and crates are
// lighter copies of the rsc/city and rsc/warehouse props.
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "offroad_buggy_scene.hpp"
#include "raisim/Rasset.hpp"
#include "raisim/World.hpp"
#include "raisim/object/singleBodies/Mesh.hpp"
#include "raisim/object/terrain/HeightMap.hpp"

namespace offroad {

/// Instances of one visual mesh. Plain-coloured meshes (the generated gantry, canopies and flags,
/// the city cars) are drawn as single visuals: rayrai's instanced batches would tint their
/// untextured materials with the instance colour.
struct PropSet {
  std::filesystem::path mesh;
  bool instanced = true;
  std::vector<Placement> at;
};

/// A dynamic prop: a body hidden from rayrai, drawn with a visual mesh whose origin is its base.
struct LooseProp {
  raisim::SingleBodyObject* body;
  std::filesystem::path mesh;
  double lift;      // height of the body's origin above the base, in the body frame [m]
  double rest = 0;  // time it has been at rest [s]
};

struct Venue {
  std::vector<PropSet> props;
  std::vector<LooseProp> loose;  // hay bales, cones and marker posts
  size_t colliders = 0;
  int tyreStacks = 0, bales = 0, cones = 0, markers = 0, pitBoxes = 0;
};

namespace venue {
// Sizes of the prepared props [m].
constexpr double kTyreStack = 0.63;  // diameter
constexpr double kBaleWidth = 0.46, kBaleLength = 0.96, kBaleHeight = 0.33;
constexpr double kWaterBarrier = 1.98, kFence = 2.0;
constexpr double kGantrySpan = 12.6, kCanopy = 3.0;
// Masses of the dynamic props [kg].
constexpr double kBaleMass = 22.0, kConeMass = 2.5, kMarkerMass = 3.0;
// Loose props wake within this distance of the car's centre [m]: the car's corners reach 2 m, and it
// covers 0.3 m between two updates at full speed.
constexpr double kWake = 4.5;
}  // namespace venue

/// Friction of the contact materials: tyres on the ground, rock, wood, barriers, hay and plastic;
/// the body slides; hay bales and cones on the ground and on each other.
inline void setMaterials(raisim::World& world) {
  world.setMaterialPairProp("tire", "terrain", 0.85, 0.0, 0.0);
  world.setMaterialPairProp("tire", "rock", 0.9, 0.0, 0.0);
  world.setMaterialPairProp("tire", "wood", 0.7, 0.0, 0.0);
  world.setMaterialPairProp("tire", "barrier", 0.6, 0.1, 0.0);
  world.setMaterialPairProp("tire", "hay", 0.6, 0.0, 0.0);
  world.setMaterialPairProp("tire", "plastic", 0.5, 0.1, 0.0);
  world.setMaterialPairProp("body", "terrain", 0.5, 0.0, 0.0);
  world.setMaterialPairProp("body", "rock", 0.4, 0.0, 0.0);
  world.setMaterialPairProp("body", "wood", 0.4, 0.1, 0.0);
  world.setMaterialPairProp("body", "barrier", 0.3, 0.2, 0.0);
  world.setMaterialPairProp("body", "hay", 0.5, 0.0, 0.0);
  world.setMaterialPairProp("body", "plastic", 0.3, 0.2, 0.0);
  for (const char* loose : {"hay", "plastic"}) {
    world.setMaterialPairProp(loose, "terrain", 0.8, 0.0, 0.0);
    world.setMaterialPairProp(loose, "rock", 0.7, 0.0, 0.0);
    world.setMaterialPairProp(loose, "barrier", 0.6, 0.0, 0.0);
  }
  world.setMaterialPairProp("hay", "hay", 0.9, 0.0, 0.0);
  world.setMaterialPairProp("hay", "plastic", 0.6, 0.0, 0.0);
  world.setMaterialPairProp("plastic", "plastic", 0.5, 0.0, 0.0);
}

/// Build the venue, add its colliders and dynamic props to the world and mark it in occupied.
/// rsc is the folder with offroad_buggy, city and warehouse. Deterministic.
inline Venue buildVenue(raisim::World& world, const raisim::HeightMap& terrain, const TerrainInfo& info, const std::filesystem::path& rsc, PlacementGrid& occupied) {
  using namespace venue;
  const TrackLocator track(info);
  const auto props = rsc / "offroad_buggy" / "props";
  const auto city = rsc / "city", warehouse = rsc / "warehouse" / "props";
  std::mt19937 rng(41);
  auto unit = [&] { return double(rng()) / 4294967296.0; };
  Venue venue;
  std::map<std::string, size_t> sets;
  std::map<std::string, raisim::Rasset> assets;

  auto record = [&](const std::filesystem::path& mesh, bool instanced, const Placement& p) {
    auto [it, added] = sets.try_emplace(mesh.string(), venue.props.size());
    if (added) venue.props.push_back({mesh, instanced, {}});
    venue.props[it->second].at.push_back(p);
  };
  auto lowest = [&](double x, double y, double radius) {
    double low = terrain.getHeight(x, y);
    for (int i = 0; i < 8 && radius > 0; ++i) {
      const double a = i * M_PI / 4;
      low = std::min(low, terrain.getHeight(x + radius * std::cos(a), y + radius * std::sin(a)));
    }
    return low;
  };
  // A static prop from its .rasset, standing on the lowest ground under its footprint radius.
  auto fixed = [&](const std::filesystem::path& rasset, double x, double y, double yaw, double radius, bool instanced = true, double z = std::numeric_limits<double>::quiet_NaN()) {
    auto it = assets.find(rasset.string());
    if (it == assets.end()) it = assets.emplace(rasset.string(), raisim::Rasset::load(rasset)).first;
    if (std::isnan(z)) z = lowest(x, y, radius * 0.7);
    venue.colliders += it->second.addStaticColliders(world, {x, y, z}, yaw, 1.0, "", "barrier").size();
    record(it->second.visualMesh, instanced, {x, y, z, yaw, 1.0, 0});
    occupied.add(x, y, radius);
  };
  // A visual-only prop (small gear in the pits).
  auto decor = [&](const std::filesystem::path& mesh, double x, double y, double yaw, double z = std::numeric_limits<double>::quiet_NaN()) {
    record(mesh, true, {x, y, std::isnan(z) ? terrain.getHeight(x, y) : z, yaw, 1.0, 0});
  };
  // Dynamic props stand on the local slope of the ground, tilted onto its normal.
  auto onGround = [&](double x, double y, double yaw) {
    const double e = 0.3;
    const Eigen::Vector3d normal(terrain.getHeight(x - e, y) - terrain.getHeight(x + e, y), terrain.getHeight(x, y - e) - terrain.getHeight(x, y + e), 2 * e);
    return Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal.normalized()) * Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  };
  auto slopeAt = [&](double x, double y) { return std::hypot(terrain.getHeight(x + 0.3, y) - terrain.getHeight(x - 0.3, y), terrain.getHeight(x, y + 0.3) - terrain.getHeight(x, y - 0.3)) / 0.6; };
  // A body whose base (lift below its origin) is height above the ground at (x, y).
  auto loose = [&](raisim::SingleBodyObject* body, const std::filesystem::path& mesh, double lift, double x, double y, double yaw, double height) {
    const Eigen::Quaterniond q = onGround(x, y, yaw);
    body->setPosition(Eigen::Vector3d(x, y, terrain.getHeight(x, y)) + q * Eigen::Vector3d(0, 0, height + lift + 0.004));
    body->setOrientation(q);
    body->setAppearance("hidden");
    body->setBodyType(raisim::BodyType::STATIC);  // asleep until the car comes close
    venue.loose.push_back({body, mesh, lift});
    occupied.add(x, y, 0.4);
  };
  auto bale = [&](double x, double y, double yaw, double height) {
    loose(world.addBox(kBaleWidth, kBaleLength, kBaleHeight, kBaleMass, "hay"), props / "hay_bales" / "square.gltf", kBaleHeight / 2, x, y, yaw, height);
    ++venue.bales;
  };
  // Cones and marker posts stand on heavy rubber bases: their centre of mass is low, not at the middle
  // of the bounding box that RaiSim would assume for a mesh.
  auto baseHeavy = [&](const std::filesystem::path& hull, double mass, double comHeight, double radius, double height) {
    raisim::Mat<3, 3> inertia;
    inertia.setZero();
    inertia(0, 0) = inertia(1, 1) = mass * (3 * radius * radius + height * height) / 24;
    inertia(2, 2) = mass * radius * radius / 2;
    raisim::Vec<3> com;
    com = {0, 0, comHeight};
    return world.addMesh(hull.string(), mass, inertia, com, 1.0, "plastic", raisim::MeshCollisionMode::CONVEX_HULL);
  };
  auto cone = [&](double x, double y, double yaw) {
    loose(baseHeavy(props / "traffic_cone" / "collision.obj", kConeMass, 0.08, 0.15, 0.53), props / "traffic_cone" / "model.gltf", 0.0, x, y, yaw, 0.0);
    ++venue.cones;
  };
  auto marker = [&](double x, double y) {
    loose(baseHeavy(props / "marker_post" / "collision.obj", kMarkerMass, 0.06, 0.13, 1.0), props / "marker_post" / "model.gltf", 0.0, x, y, 0.0, 0.0);
    ++venue.markers;
  };
  auto along = [&](double station, double lateral, double& heading) { return track.offset(station, lateral, &heading); };
  const double w = info.halfWidth;

  // Start/finish gantry across the track; the straight is level, so it stands on the track height.
  double heading = 0;
  const Eigen::Vector2d gate = along(info.startLine, 0, heading);
  fixed(props / "start_gantry" / "model.rasset", gate.x(), gate.y(), heading, 0.6, false,
        std::min(lowest(gate.x() - std::sin(heading) * kGantrySpan / 2, gate.y() + std::cos(heading) * kGantrySpan / 2, 0.5),
                 lowest(gate.x() + std::sin(heading) * kGantrySpan / 2, gate.y() - std::cos(heading) * kGantrySpan / 2, 0.5)));
  occupied.add(gate.x(), gate.y(), kGantrySpan / 2 + 1);

  // The start straight: water barriers on both sides, the fence behind them on the spectators'
  // side, and a gap into the pit lane at the end of the pit wall.
  auto clearOfGate = [&](double s, double margin) { return std::abs(s - info.startLine) > margin; };
  const double s0 = info.straightStart + 6, s1 = info.straightEnd - 4;
  for (double s = s0; s + kWaterBarrier < s1; s += kWaterBarrier) {
    if (!clearOfGate(s + kWaterBarrier / 2, 1.8)) continue;
    const Eigen::Vector2d p = along(s + kWaterBarrier / 2, w + 1.3, heading);
    const bool red = int(std::round((s - s0) / kWaterBarrier)) % 2 == 0;
    fixed(props / "water_barrier" / (red ? "red.rasset" : "white.rasset"), p.x(), p.y(), heading, 0.5);
  }
  for (double s = s0 + 2; s + kFence < s1 + 4; s += kFence) {
    const Eigen::Vector2d p = along(s + kFence / 2, w + 3.6, heading);
    fixed(props / "chainlink_fence" / "model.rasset", p.x(), p.y(), heading, 0.3);
  }
  for (double s = s0; s + kWaterBarrier < s1 - 22; s += kWaterBarrier) {
    if (!clearOfGate(s + kWaterBarrier / 2, 1.8)) continue;
    const Eigen::Vector2d p = along(s + kWaterBarrier / 2, -(w + 1.3), heading);
    const bool red = int(std::round((s - s0) / kWaterBarrier)) % 2 == 1;
    fixed(props / "water_barrier" / (red ? "red.rasset" : "white.rasset"), p.x(), p.y(), heading, 0.5);
  }
  // Feather flags between the barriers and the fence, their printed side towards the oncoming cars.
  for (double s = s0 + 5; s < s1; s += 13) {
    const Eigen::Vector2d p = along(s, w + 3.0, heading);
    if (!clearOfGate(s, 3)) continue;
    const bool orange = int((s - s0) / 13) % 2 == 0;
    record(props / "feather_flag" / (orange ? "orange.gltf" : "blue.gltf"), false, {p.x(), p.y(), terrain.getHeight(p.x(), p.y()), heading - M_PI / 2 - 0.35, 1.0, 0});
  }

  // Corners, found from the curvature of the centre line: a tyre wall on the outside of the tight
  // ones, hay bales on the outside of the others, and three cones on the apex.
  const size_t n = info.centre.size();
  const double step = info.centreStep;
  std::vector<std::array<size_t, 2>> corners;
  for (size_t i = 0; i < n; ++i) {
    const bool in = std::abs(info.curvature[i]) > 1 / 40.0, before = std::abs(info.curvature[(i + n - 1) % n]) > 1 / 40.0;
    if (in && !before) corners.push_back({i, i});
    if (in && !corners.empty()) corners.back()[1] = i;
  }
  for (const auto& [first, last] : corners) {
    size_t apex = first;
    for (size_t i = first; i <= last; ++i)
      if (std::abs(info.curvature[i]) > std::abs(info.curvature[apex])) apex = i;
    const double kApex = info.curvature[apex], side = kApex > 0 ? 1.0 : -1.0;  // +1: turning left
    const bool tight = std::abs(kApex) > 1 / 30.0;
    const double begin = first * step - 4, end = last * step + 4;
    const double lateral = -side * (w + (tight ? 2.3 : 2.0));  // on the outside
    for (double s = begin; s < end;) {
      const double grow = 1 + std::abs(track.curvature(s) * lateral);  // outside arc per station
      const Eigen::Vector2d p = along(s, lateral, heading);
      if (tight) {
        fixed(props / "tyre_stack" / "model.rasset", p.x(), p.y(), unit() * 2 * M_PI, kTyreStack / 2);
        ++venue.tyreStacks;
        s += (kTyreStack + 0.01) / grow;
      } else {
        // Square bales lengthwise along the edge, two high around the apex (staggered), where the
        // ground is gentle enough to hold them.
        if (slopeAt(p.x(), p.y()) < 0.3) {
          bale(p.x(), p.y(), heading + M_PI / 2 + 0.06 * (unit() - 0.5), 0.0);
          if (std::abs(s - apex * step) < 8) {
            const Eigen::Vector2d q = along(s + 0.5 * kBaleLength / grow, lateral, heading);
            bale(q.x(), q.y(), heading + M_PI / 2, kBaleHeight + 0.01);
          }
        }
        s += (kBaleLength + 0.04) / grow;
      }
    }
    for (int c = -1; c <= 1; ++c) {
      const Eigen::Vector2d p = along(apex * step + 3.0 * c, side * (w + 0.4), heading);
      cone(p.x(), p.y(), unit() * 2 * M_PI);
    }
  }

  // Marker posts every 12 m along both edges, where no barrier, tyre wall or bale line is.
  auto dressed = [&](double s) {
    if (s > info.straightStart - 4 && s < info.straightEnd + 4) return true;
    for (double d = -8; d <= 8; d += 2)
      if (std::abs(track.curvature(s + d)) > 1 / 40.0) return true;
    return false;
  };
  for (double s = info.straightEnd + 10; s < info.length + info.straightStart - 6; s += 12) {
    if (dressed(s)) continue;
    for (double side : {-1.0, 1.0}) {
      const Eigen::Vector2d p = along(s, side * (w + 0.9), heading);
      if (slopeAt(p.x(), p.y()) < 0.35) marker(p.x(), p.y());
    }
  }

  // Pit lane: a row of canopies facing the track, each with a team's gear.
  const Area& pit = info.pit;
  const std::array<const char*, 3> colours = {"blue.rasset", "red.rasset", "white.rasset"};
  venue.pitBoxes = int((pit.x1 - pit.x0 - 8) / 12);
  for (int b = 0; b < venue.pitBoxes; ++b) {
    const double cx = pit.x0 + 4 + kCanopy / 2 + b * 12, cy = pit.y1 - 4.0;
    fixed(props / "canopy" / colours[b % 3], cx, cy, 0, kCanopy / 2, false);
    // Back of the box: tool cart, spare tyres and crates; front: chairs, fuel and a generator.
    fixed(props / "tool_cart" / "model.rasset", cx - 0.6, cy - 1.0, M_PI + 0.1 * (unit() - 0.5), 0.7);
    fixed(props / "tyre_stack" / "model.rasset", cx + 1.0, cy - 1.0, unit() * 2 * M_PI, 0.32);
    if (b % 2 == 0) fixed(props / "tyre_stack" / "model.rasset", cx + 1.0, cy - 0.35, unit() * 2 * M_PI, 0.32);
    decor(props / "plastic_crate_03" / "model.gltf", cx - 1.2, cy + 0.2, 1.4 + 0.2 * unit());
    decor(props / "plastic_crate_03" / "model.gltf", cx - 1.2, cy + 0.2, 1.5 + 0.2 * unit(), terrain.getHeight(cx - 1.2, cy + 0.2) + 0.266);
    for (int k = 0; k < 2; ++k) decor(props / "plastic_monobloc_chair_01" / "model.gltf", cx - 0.4 + 0.8 * k, cy + 0.9, M_PI / 2 + 0.5 * (unit() - 0.5));
    decor(props / "metal_jerrycan" / "model.gltf", cx + 1.25, cy + 0.8, 0.3 * unit());
    decor(props / "metal_jerrycan" / "model.gltf", cx + 1.25, cy + 1.2, 0.3 * unit());
    if (b % 3 != 1) fixed(props / "portable_generator" / "model.rasset", cx + 2.4, cy + 1.0, unit() * 2 * M_PI, 0.5);
    if (b % 3 == 1) fixed(warehouse / "propane_tank" / "model.rasset", cx + 2.0, cy + 1.2, 0, 0.2);
    fixed(warehouse / (b % 2 ? "Barrel_01" : "barrel_03") / "model.rasset", cx - 2.2, cy - 1.2, unit() * 2 * M_PI, 0.32);
  }
  // Paddock behind the pits: the teams' cars and a stack of round bales.
  const std::array<std::array<const char*, 2>, 4> cars = {
      {{"fairheaven_lt80", "model.rasset"}, {"kiri86", "model.rasset"}, {"fairheaven_sw84", "model.rasset"}, {"fairheaven_lt80", "model_paint1.rasset"}}};
  for (int c = 0; c < 4; ++c) fixed(city / "cars" / cars[c][0] / cars[c][1], pit.x0 + 10 + c * 15 + 3 * unit(), pit.y0 + 5 + unit(), M_PI / 2 + 0.25 * (unit() - 0.5), 2.6, false);
  for (int k = 0; k < 3; ++k) fixed(props / "hay_bales" / "round.rasset", pit.x1 - 4 - 1.6 * k, pit.y0 + 3, M_PI / 2, 0.8);

  // Spectators: bleachers facing the track, the fence in front of them, toilets and picnic tables.
  const Area& stand = info.spectators;
  const double rowY = stand.y0 + 6.5;  // behind the fence
  for (int k = 0; k < 7; ++k) {
    const double x = stand.x0 + 8 + k * 4.35 + (k >= 4 ? 2.0 : 0.0);
    fixed(props / "bleacher" / "model.rasset", x, rowY, 0, 2.2);
  }
  for (int k = 0; k < 4; ++k) fixed(props / "porta_potty" / "model.rasset", stand.x1 - 2 - 1.25 * k, stand.y1 - 1.5, M_PI, 0.6);
  for (int k = 0; k < 3; ++k) {
    const double x = stand.x0 + 1.5, y = stand.y0 + 6.0 + 3.6 * k;
    fixed(props / "wooden_picnic_table" / "model.rasset", x + 0.4 * unit(), y, M_PI / 2 + 0.2 * (unit() - 0.5), 1.4);
  }
  for (int k = 0; k < 6; ++k) decor(props / "plastic_monobloc_chair_01" / "model.gltf", stand.x0 + 5 + 1.1 * k + 0.3 * unit(), stand.y1 - 1.6 - 0.8 * unit(), unit() * 2 * M_PI);
  for (int k = 0; k < 4; ++k) {
    const double x = stand.x0 + 6 + k * 16;
    record(props / "feather_flag" / (k % 2 ? "orange.gltf" : "blue.gltf"), false, {x, stand.y1 + 1.0, terrain.getHeight(x, stand.y1 + 1.0), -M_PI / 2 + 0.4, 1.0, 0});
  }
  // Keep the trees and boulders off the pads.
  for (const Area* area : {&info.pit, &info.spectators})
    for (double x = area->x0; x <= area->x1; x += 4)
      for (double y = area->y0; y <= area->y1; y += 4) occupied.add(x, y, 3.0);
  return venue;
}

/// Loose props sleep as static bodies until the car comes within kWake, and fall asleep again once
/// they have been at rest for half a second away from it. Each dynamic prop resting on the height
/// map costs RaiSim several contacts per step, which a hundred of them would make the bulk of the
/// simulation time.
inline void updateLooseProps(Venue& venue, const Eigen::Vector3d& car, double dt) {
  for (auto& prop : venue.loose) {
    const double distance = (prop.body->getPosition() - car).head<2>().norm();
    if (prop.body->getBodyType() == raisim::BodyType::STATIC) {
      if (distance < venue::kWake) {
        prop.body->setBodyType(raisim::BodyType::DYNAMIC);
        prop.rest = 0;
      }
      continue;
    }
    const bool still = prop.body->getLinearVelocity().norm() < 0.03 && prop.body->getAngularVelocity().norm() < 0.08;
    prop.rest = still ? prop.rest + dt : 0.0;
    if (distance > venue::kWake + 2 && prop.rest > 0.5) prop.body->setBodyType(raisim::BodyType::STATIC);
  }
}

}  // namespace offroad
