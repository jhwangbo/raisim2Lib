// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "rayrai_tcp_viewer_hint.hpp"

namespace {

constexpr size_t kRayCount = 4096;
constexpr double kRayLength = 45.0;
constexpr double kRayRadius = 0.012;
constexpr double kHitRadius = 0.09;
constexpr double kScanPeriod = 0.1;
constexpr double kPi = 3.14159265358979323846;

struct RayVisual {
  Eigen::Vector3d midpoint;
  Eigen::Vector4d orientation;
  Eigen::Vector3d scale;
  float colorWeight;
};

std::vector<Eigen::Vector3d> makeUniformDirections(size_t count) {
  std::vector<Eigen::Vector3d> directions;
  directions.reserve(count);

  // A Fibonacci sphere distributes directions evenly without over-sampling the poles.
  constexpr double goldenAngle = kPi * (3.0 - 2.23606797749978969641);
  for (size_t i = 0; i < count; ++i) {
    const double z = 1.0 - 2.0 * (static_cast<double>(i) + 0.5) /
                               static_cast<double>(count);
    const double radialDistance = std::sqrt(std::max(0.0, 1.0 - z * z));
    const double azimuth = goldenAngle * static_cast<double>(i);
    directions.emplace_back(radialDistance * std::cos(azimuth),
                            radialDistance * std::sin(azimuth),
                            z);
  }

  return directions;
}

Eigen::Vector4d alignCylinderWith(const Eigen::Vector3d& direction) {
  const Eigen::Quaterniond quaternion =
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), direction);
  return {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
}

void addArena(raisim::World& world) {
  constexpr double halfExtent = 25.0;
  constexpr double wallThickness = 0.5;
  constexpr double wallHeight = 12.0;

  auto ground = world.addGround();
  ground->setAppearance("grid");

  auto negativeXWall = world.addBox(wallThickness, 2.0 * halfExtent, wallHeight, 1.0);
  auto positiveXWall = world.addBox(wallThickness, 2.0 * halfExtent, wallHeight, 1.0);
  auto negativeYWall = world.addBox(2.0 * halfExtent, wallThickness, wallHeight, 1.0);
  auto positiveYWall = world.addBox(2.0 * halfExtent, wallThickness, wallHeight, 1.0);

  negativeXWall->setPosition(-halfExtent, 0.0, wallHeight * 0.5);
  positiveXWall->setPosition(halfExtent, 0.0, wallHeight * 0.5);
  negativeYWall->setPosition(0.0, -halfExtent, wallHeight * 0.5);
  positiveYWall->setPosition(0.0, halfExtent, wallHeight * 0.5);

  for (auto* wall : {negativeXWall, positiveXWall, negativeYWall, positiveYWall}) {
    wall->setBodyType(raisim::BodyType::STATIC);
    wall->setAppearance("0.32, 0.35, 0.40, 1.0");
  }

  // Offset pillars produce hit points at a useful range before rays reach the walls.
  constexpr size_t pillarCount = 20;
  for (size_t i = 0; i < pillarCount; ++i) {
    const double angle = 2.0 * kPi * static_cast<double>(i) /
                         static_cast<double>(pillarCount);
    const double radius = 9.0 + 4.0 * static_cast<double>(i % 3);
    const double height = 3.0 + static_cast<double>(i % 5);
    auto pillar = world.addCylinder(0.65, height, 1.0);
    pillar->setBodyType(raisim::BodyType::STATIC);
    pillar->setPosition(radius * std::cos(angle), radius * std::sin(angle), height * 0.5);
    pillar->setAppearance("0.18, 0.45, 0.70, 1.0");
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  raisim::Path::setFromArgv(argv[0]);

  raisim::World world;
  world.setTimeStep(0.002);
  addArena(world);

  const Eigen::Vector3d raySource(0.0, 0.0, 6.0);
  const auto baseDirections = makeUniformDirections(kRayCount);

  raisim::RaisimServer server(&world);
  auto sourceVisual = server.addVisualSphere("ray_source", 0.35, 1.0, 0.75, 0.05, 1.0);
  sourceVisual->setPosition(raySource);

  auto rayVisuals = server.addInstancedVisuals(
      "ray_cylinders",
      raisim::Shape::Cylinder,
      {kRayRadius, kRayRadius, 1.0},
      {0.05, 0.55, 1.0, 0.35},
      {0.15, 0.95, 1.0, 0.35});
  auto hitVisuals = server.addInstancedVisuals(
      "ray_hit_spheres",
      raisim::Shape::Sphere,
      {kHitRadius, kHitRadius, kHitRadius},
      {1.0, 0.85, 0.05, 1.0},
      {1.0, 0.15, 0.05, 1.0});

  std::vector<RayVisual> rays;
  std::vector<Eigen::Vector3d> hitPoints;
  std::vector<float> hitColorWeights;
  rays.reserve(kRayCount);
  hitPoints.reserve(kRayCount);
  hitColorWeights.reserve(kRayCount);

  auto updateScan = [&](double scanAngle) {
    rays.clear();
    hitPoints.clear();
    hitColorWeights.clear();

    const Eigen::AngleAxisd scanRotation(scanAngle, Eigen::Vector3d(1.0, 1.0, 0.5).normalized());
    for (const auto& baseDirection : baseDirections) {
      const Eigen::Vector3d direction = scanRotation * baseDirection;
      const auto& collisions = world.rayTest(raySource, direction, kRayLength);

      Eigen::Vector3d endpoint = raySource + direction * kRayLength;
      const bool hit = collisions.size() > 0;
      if (hit)
        endpoint = collisions[0].getPosition();

      const double rayDistance = (endpoint - raySource).norm();
      const float distanceWeight = static_cast<float>(rayDistance / kRayLength);
      rays.push_back({(raySource + endpoint) * 0.5,
                      alignCylinderWith(direction),
                      {1.0, 1.0, rayDistance},
                      distanceWeight});

      if (hit) {
        hitPoints.push_back(endpoint);
        hitColorWeights.push_back(distanceWeight);
      }
    }

    server.lockVisualizationServerMutex();
    rayVisuals->clearInstances();
    hitVisuals->clearInstances();
    for (const auto& ray : rays)
      rayVisuals->addInstance(ray.midpoint, ray.orientation, ray.scale, ray.colorWeight);
    for (size_t i = 0; i < hitPoints.size(); ++i)
      hitVisuals->addInstance(hitPoints[i], hitColorWeights[i]);
    server.unlockVisualizationServerMutex();
  };

  updateScan(0.0);
  std::cout << "Large-scale ray test: " << kRayCount << " rays, "
            << hitPoints.size() << " initial hits" << std::endl;

  server.launchServer();
  raisim_examples::warnIfNoClientConnected(server);

  double elapsedSinceScan = 0.0;
  double scanAngle = 0.0;
  while (true) {
    RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))
    server.integrateWorldThreadSafe();

    elapsedSinceScan += world.getTimeStep();
    if (elapsedSinceScan >= kScanPeriod) {
      elapsedSinceScan -= kScanPeriod;
      scanAngle += 0.025;
      updateScan(scanAngle);
    }
  }

  server.killServer();
}
