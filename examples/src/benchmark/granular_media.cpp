// ANYmal standing on native granular media.

#include "benchmarkCommon.hpp"

#include <raisim/RaisimServer.hpp>
#include <raisim/World.hpp>
#include <raisim/object/granular/GranularSystem.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

int parseServerPort(int argc, char** argv) {
  int port = 8080;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    const std::string prefix = "--port=";
    if (arg.rfind(prefix, 0) == 0) {
      port = std::max(1, std::atoi(arg.c_str() + prefix.size()));
    }
  }
  return port;
}

void buildAnymalDefaults(raisim::ArticulatedSystem* anymal,
                         Eigen::VectorXd& gc,
                         Eigen::VectorXd& gv,
                         Eigen::VectorXd& pgain,
                         Eigen::VectorXd& dgain,
                         Eigen::VectorXd& velTarget) {
  const size_t gcDim = anymal->getGeneralizedCoordinateDim();
  const size_t dof = anymal->getDOF();
  gc.setZero(static_cast<int>(gcDim));
  gv.setZero(static_cast<int>(dof));
  pgain.setZero(static_cast<int>(dof));
  dgain.setZero(static_cast<int>(dof));
  velTarget.setZero(static_cast<int>(dof));

  const double jointDefaults[12] = {
      0.03, 0.4, -0.8,
      -0.03, 0.4, -0.8,
      0.03, -0.4, 0.8,
      -0.03, -0.4, 0.8
  };

  if (gcDim >= 7 && dof >= 6) {
    gc[0] = 0.0;
    gc[1] = 0.0;
    gc[2] = 0.54;
    gc[3] = 1.0;
    gc[4] = 0.0;
    gc[5] = 0.0;
    gc[6] = 0.0;
    const size_t jointCount = std::min<size_t>(12, gcDim - 7);
    for (size_t i = 0; i < jointCount; ++i) {
      gc[7 + static_cast<int>(i)] = jointDefaults[i];
    }
  } else {
    const size_t jointCount = std::min<size_t>(12, gcDim);
    for (size_t i = 0; i < jointCount; ++i) {
      gc[static_cast<int>(i)] = jointDefaults[i];
    }
  }

  const size_t gainCount = std::min<size_t>(12, dof);
  for (size_t i = 0; i < gainCount; ++i) {
    pgain[static_cast<int>(dof - gainCount + i)] = 200.0;
    dgain[static_cast<int>(dof - gainCount + i)] = 10.0;
  }
}

void makeGranularBed(int resolution,
                     int layers,
                     double radius,
                     double spacing,
                     double bedLength,
                     double bedWidth,
                     std::vector<raisim::Vec<3>>& positions,
                     std::vector<double>& radii,
                     std::vector<uint8_t>& fixed) {
  const double staggerPad = 0.5 * radius;
  const int fitNx = std::max(1, static_cast<int>((bedLength - 2.0 * radius - staggerPad) / spacing) + 1);
  const int fitNy = std::max(1, static_cast<int>((bedWidth - 2.0 * radius - staggerPad) / spacing) + 1);
  const int targetNx = std::max(4, resolution * 3 + 6);
  const int targetNy = std::max(4, resolution * 2 + 4);
  const int nx = std::max(1, std::min(fitNx, targetNx));
  const int ny = std::max(1, std::min(fitNy, targetNy));
  const int clampedLayers = std::max(2, layers);
  const int maxParticles = nx * ny * clampedLayers;
  positions.clear();
  radii.clear();
  fixed.clear();
  positions.reserve(static_cast<size_t>(std::min(nx * ny * clampedLayers, maxParticles)));
  radii.reserve(positions.capacity());
  fixed.reserve(positions.capacity());

  const double x0 = -0.5 * spacing * static_cast<double>(nx - 1);
  const double y0 = -0.5 * spacing * static_cast<double>(ny - 1);
  for (int z = 0; z < clampedLayers; ++z) {
    for (int y = 0; y < ny; ++y) {
      for (int x = 0; x < nx; ++x) {
        const double staggerX = ((y + z) & 1) ? 0.5 * radius : 0.0;
        const double staggerY = (z & 1) ? 0.5 * radius : 0.0;
        positions.push_back({x0 + spacing * static_cast<double>(x) + staggerX,
                             y0 + spacing * static_cast<double>(y) + staggerY,
                             radius + spacing * static_cast<double>(z)});
        radii.push_back(radius);
        fixed.push_back(z == 0 ? 1u : 0u);
      }
    }
  }
}

double computeTopSurface(const raisim::GranularSystem* grains) {
  double top = 0.0;
  const auto& positions = grains->getPositions();
  const auto& radii = grains->getRadii();
  for (size_t i = 0; i < positions.size(); ++i) {
    top = std::max(top, positions[i][2] + radii[i]);
  }
  return top;
}

void syncParticleVisuals(raisim::InstancedVisuals* visual, const raisim::GranularSystem* grains) {
  if (!visual || !grains) return;
  const auto& positions = grains->getPositions();
  const auto& velocities = grains->getVelocities();
  for (size_t i = 0; i < positions.size(); ++i) {
    visual->setPosition(i, Eigen::Vector3d(positions[i][0], positions[i][1], positions[i][2]));
    visual->setColorWeight(i, static_cast<float>(std::min(1.0, velocities[i].norm() / 1.0)));
  }
}

void addPhysicalContainer(raisim::World& world, double bedLength, double bedWidth, double bedHeight) {
  constexpr double thickness = 0.04;
  constexpr const char* halfTransparent = "0.2, 0.35, 0.7, 0.5";
  auto* floor = world.addBox(bedLength + 2.0 * thickness, bedWidth + 2.0 * thickness, thickness, 1.0);
  floor->setName("granular_anymal_floor_collision");
  floor->setAppearance(halfTransparent);
  floor->setPosition(0.0, 0.0, -0.5 * thickness);
  floor->setBodyType(raisim::BodyType::STATIC);

  auto* left = world.addBox(thickness, bedWidth + 2.0 * thickness, bedHeight, 1.0);
  left->setName("granular_anymal_left_wall_collision");
  left->setAppearance(halfTransparent);
  left->setPosition(-0.5 * bedLength - 0.5 * thickness, 0.0, 0.5 * bedHeight);
  left->setBodyType(raisim::BodyType::STATIC);

  auto* right = world.addBox(thickness, bedWidth + 2.0 * thickness, bedHeight, 1.0);
  right->setName("granular_anymal_right_wall_collision");
  right->setAppearance(halfTransparent);
  right->setPosition(0.5 * bedLength + 0.5 * thickness, 0.0, 0.5 * bedHeight);
  right->setBodyType(raisim::BodyType::STATIC);

  auto* back = world.addBox(bedLength, thickness, bedHeight, 1.0);
  back->setName("granular_anymal_back_wall_collision");
  back->setAppearance(halfTransparent);
  back->setPosition(0.0, -0.5 * bedWidth - 0.5 * thickness, 0.5 * bedHeight);
  back->setBodyType(raisim::BodyType::STATIC);

  auto* front = world.addBox(bedLength, thickness, bedHeight, 1.0);
  front->setName("granular_anymal_front_wall_collision");
  front->setAppearance(halfTransparent);
  front->setPosition(0.0, 0.5 * bedWidth + 0.5 * thickness, 0.5 * bedHeight);
  front->setBodyType(raisim::BodyType::STATIC);
}

} // namespace

int run_granular_media_example(int argc, char** argv) {
  const int serverPort = parseServerPort(argc, argv);
  int steps = 240;
  int resolution = 7;
  int layers = 7;
  int substeps = 2;
  int settleSteps = 500;
  int robotSettleSteps = 250;
  double radius = 0.022;
  double spacing = 0.047;
  double bedLength = 1.8;
  double bedWidth = 1.25;
  double stiffness = 3.5e4;
  double damping = 35.0;
  double tangentialStiffness = 1.75e4;
  double tangentialDamping = 8.0;
  double friction = 0.85;
  double rollingFriction = 0.03;
  double baseClearance = 0.02;
  double maxAllowedSpeed = 6.0;
  bool hertz = false;
  bool stepsExplicit = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if ((arg == "--steps" || arg == "--loops") && i + 1 < argc) {
      steps = std::max(1, std::atoi(argv[++i]));
      stepsExplicit = true;
    } else if (arg.rfind("--steps=", 0) == 0) {
      steps = std::max(1, std::atoi(arg.c_str() + 8));
      stepsExplicit = true;
    } else if ((arg == "--resolution" || arg == "--res") && i + 1 < argc) {
      resolution = std::max(2, std::atoi(argv[++i]));
    } else if (arg.rfind("--resolution=", 0) == 0) {
      resolution = std::max(2, std::atoi(arg.c_str() + 13));
    } else if (arg == "--layers" && i + 1 < argc) {
      layers = std::max(2, std::atoi(argv[++i]));
    } else if (arg.rfind("--layers=", 0) == 0) {
      layers = std::max(2, std::atoi(arg.c_str() + 9));
    } else if (arg == "--substeps" && i + 1 < argc) {
      substeps = std::max(1, std::atoi(argv[++i]));
    } else if (arg.rfind("--substeps=", 0) == 0) {
      substeps = std::max(1, std::atoi(arg.c_str() + 11));
    } else if (arg == "--settle-steps" && i + 1 < argc) {
      settleSteps = std::max(0, std::atoi(argv[++i]));
    } else if (arg.rfind("--settle-steps=", 0) == 0) {
      settleSteps = std::max(0, std::atoi(arg.c_str() + 15));
    } else if (arg == "--robot-settle-steps" && i + 1 < argc) {
      robotSettleSteps = std::max(0, std::atoi(argv[++i]));
    } else if (arg.rfind("--robot-settle-steps=", 0) == 0) {
      robotSettleSteps = std::max(0, std::atoi(arg.c_str() + 21));
    } else if (arg == "--radius" && i + 1 < argc) {
      radius = std::max(1.0e-5, std::atof(argv[++i]));
    } else if (arg.rfind("--radius=", 0) == 0) {
      radius = std::max(1.0e-5, std::atof(arg.c_str() + 9));
    } else if (arg == "--spacing" && i + 1 < argc) {
      spacing = std::max(2.0 * radius, std::atof(argv[++i]));
    } else if (arg.rfind("--spacing=", 0) == 0) {
      spacing = std::max(2.0 * radius, std::atof(arg.c_str() + 10));
    } else if (arg == "--bed-length" && i + 1 < argc) {
      bedLength = std::max(4.0 * spacing, std::atof(argv[++i]));
    } else if (arg.rfind("--bed-length=", 0) == 0) {
      bedLength = std::max(4.0 * spacing, std::atof(arg.c_str() + 13));
    } else if (arg == "--bed-width" && i + 1 < argc) {
      bedWidth = std::max(4.0 * spacing, std::atof(argv[++i]));
    } else if (arg.rfind("--bed-width=", 0) == 0) {
      bedWidth = std::max(4.0 * spacing, std::atof(arg.c_str() + 12));
    } else if (arg == "--stiffness" && i + 1 < argc) {
      stiffness = std::max(0.0, std::atof(argv[++i]));
    } else if (arg.rfind("--stiffness=", 0) == 0) {
      stiffness = std::max(0.0, std::atof(arg.c_str() + 12));
    } else if (arg == "--damping" && i + 1 < argc) {
      damping = std::max(0.0, std::atof(argv[++i]));
    } else if (arg.rfind("--damping=", 0) == 0) {
      damping = std::max(0.0, std::atof(arg.c_str() + 10));
    } else if (arg == "--tangential-stiffness" && i + 1 < argc) {
      tangentialStiffness = std::max(0.0, std::atof(argv[++i]));
    } else if (arg.rfind("--tangential-stiffness=", 0) == 0) {
      tangentialStiffness = std::max(0.0, std::atof(arg.c_str() + 23));
    } else if (arg == "--tangential-damping" && i + 1 < argc) {
      tangentialDamping = std::max(0.0, std::atof(argv[++i]));
    } else if (arg.rfind("--tangential-damping=", 0) == 0) {
      tangentialDamping = std::max(0.0, std::atof(arg.c_str() + 21));
    } else if (arg == "--friction" && i + 1 < argc) {
      friction = std::max(0.0, std::atof(argv[++i]));
    } else if (arg.rfind("--friction=", 0) == 0) {
      friction = std::max(0.0, std::atof(arg.c_str() + 11));
    } else if (arg == "--rolling-friction" && i + 1 < argc) {
      rollingFriction = std::max(0.0, std::atof(argv[++i]));
    } else if (arg.rfind("--rolling-friction=", 0) == 0) {
      rollingFriction = std::max(0.0, std::atof(arg.c_str() + 19));
    } else if (arg == "--base-clearance" && i + 1 < argc) {
      baseClearance = std::max(0.0, std::atof(argv[++i]));
    } else if (arg.rfind("--base-clearance=", 0) == 0) {
      baseClearance = std::max(0.0, std::atof(arg.c_str() + 17));
    } else if (arg == "--max-allowed-speed" && i + 1 < argc) {
      maxAllowedSpeed = std::max(0.0, std::atof(argv[++i]));
    } else if (arg.rfind("--max-allowed-speed=", 0) == 0) {
      maxAllowedSpeed = std::max(0.0, std::atof(arg.c_str() + 20));
    } else if (arg == "--hertz") {
      hertz = true;
    }
  }

  const std::filesystem::path binaryDir = std::filesystem::absolute(argv[0]).parent_path();
  const std::filesystem::path rscDir = std::filesystem::exists(binaryDir / "rsc")
                                          ? (binaryDir / "rsc")
                                          : (binaryDir / "../../rsc");
  const std::string rscPath = std::filesystem::weakly_canonical(rscDir).string();
  raisim::World::setActivationKey(rscPath + "/activation.raisim");

  raisim::World world;
  world.setSleepingEnabled(false);
  world.setGravity({0.0, 0.0, -9.81});
  world.setTimeStep(0.001);
  const double containerHeight = 0.38;
  addPhysicalContainer(world, bedLength, bedWidth, containerHeight);

  raisim::GranularSystem::Material material;
  material.density = 1600.0;
  material.normalStiffness = stiffness;
  material.normalDamping = damping;
  material.tangentialStiffness = tangentialStiffness;
  material.tangentialDamping = tangentialDamping;
  material.friction = friction;
  material.rollingFriction = rollingFriction;
  material.normalContactModel = hertz ? raisim::GranularSystem::NormalContactModel::HERTZ
                                      : raisim::GranularSystem::NormalContactModel::LINEAR;
  material.substeps = substeps;
  material.maxSpeed = 10.0;
  material.maxAngularSpeed = 100.0;

  std::vector<raisim::Vec<3>> positions;
  std::vector<double> radii;
  std::vector<uint8_t> fixed;
  makeGranularBed(resolution, layers, radius, spacing, bedLength, bedWidth, positions, radii, fixed);
  auto* grains = world.addGranularParticles(positions, radii, material);
  grains->setName("granular_anymal_bed");
  for (size_t i = 0; i < fixed.size(); ++i) {
    if (fixed[i]) {
      grains->setParticleFixed(i, true);
    }
  }

  for (int i = 0; i < settleSteps; ++i) {
    world.integrate();
  }

  auto* anymal = world.addArticulatedSystem(rscPath + "/anymal/urdf/anymal.urdf");
  Eigen::VectorXd gc, gv, pgain, dgain, velTarget;
  buildAnymalDefaults(anymal, gc, gv, pgain, dgain, velTarget);
  const double bedSurface = computeTopSurface(grains);
  if (gc.size() >= 3) {
    gc[2] += bedSurface + baseClearance;
  }
  anymal->setState(gc, gv);
  anymal->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  anymal->setPdGains(pgain, dgain);
  anymal->setPdTarget(gc, velTarget);
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));

  for (int i = 0; i < robotSettleSteps; ++i) {
    world.integrate();
  }

  raisim::RaisimServer server(&world);
  auto* particleVisuals = server.addInstancedVisuals(
      "granular_anymal_particles",
      raisim::Shape::Sphere,
      raisim::Vec<3>{radius, radius, radius},
      raisim::Vec<4>{0.92, 0.70, 0.34, 1.0},
      raisim::Vec<4>{0.16, 0.45, 0.95, 1.0});
  particleVisuals->resize(grains->getNumParticles());
  syncParticleVisuals(particleVisuals, grains);
  server.launchServer(serverPort);
  std::cout << "[granular_media] server on port " << serverPort
            << ". Connect Rayrai/RaiSim visualizer." << std::endl;

  const auto begin = std::chrono::steady_clock::now();
  int stepsRun = 0;
  bool interactiveStarted = false;
  const bool interactiveViz = !stepsExplicit;
  uint64_t totalParticleContacts = 0;
  uint64_t totalBoundaryContacts = 0;
  uint64_t totalRigidContacts = 0;
  uint64_t totalRigidCandidatePairs = 0;
  uint64_t totalAnymalContacts = 0;
  double maxPenetration = 0.0;
  double maxSpeed = 0.0;
  bool unstable = false;
  for (;;) {
    if (interactiveViz && !interactiveStarted) {
      if (!server.isConnected()) {
        server.waitForNewClients(10000);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      interactiveStarted = true;
    } else if (!interactiveViz && stepsRun >= steps) {
      break;
    }

    const auto stepBegin = std::chrono::steady_clock::now();
    server.integrateWorldThreadSafe();
    server.lockVisualizationServerMutex();
    syncParticleVisuals(particleVisuals, grains);
    server.unlockVisualizationServerMutex();
    if (server.isConnected()) {
      const auto stepElapsed = std::chrono::steady_clock::now() - stepBegin;
      const auto stepBudget = std::chrono::duration<double>(world.getTimeStep());
      if (stepElapsed < stepBudget) {
        std::this_thread::sleep_for(stepBudget - stepElapsed);
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const auto& stepStats = grains->getLastStepStats();
    totalParticleContacts += stepStats.particleContacts;
    totalBoundaryContacts += stepStats.boundaryContacts;
    totalRigidContacts += stepStats.rigidContacts;
    totalRigidCandidatePairs += stepStats.rigidCandidatePairs;
    totalAnymalContacts += anymal->getContacts().size();
    maxPenetration = std::max(maxPenetration, stepStats.maxPenetration);
    maxSpeed = std::max(maxSpeed, stepStats.maxSpeed);
    if (maxAllowedSpeed > 0.0 && stepStats.maxSpeed > maxAllowedSpeed) {
      unstable = true;
    }
    for (const auto& position : grains->getPositions()) {
      if (!std::isfinite(position[0]) || !std::isfinite(position[1]) || !std::isfinite(position[2])) {
        unstable = true;
        break;
      }
    }
    ++stepsRun;
  }
  const auto end = std::chrono::steady_clock::now();

  raisim::Vec<3> basePosition;
  anymal->getPosition(0, basePosition);
  const int divisor = std::max(1, stepsRun);
  raisim::print_timediff("Granular media", divisor, begin, end);
  std::cout << "resolution=" << resolution
            << " particles=" << grains->getNumParticles()
            << " fixed_particles=" << std::count(fixed.begin(), fixed.end(), uint8_t(1))
            << " steps=" << stepsRun
            << " settle_steps=" << settleSteps
            << " robot_settle_steps=" << robotSettleSteps
            << " layers=" << layers
            << " substeps=" << substeps
            << " radius=" << radius
            << " spacing=" << spacing
            << " bed_length=" << bedLength
            << " bed_width=" << bedWidth
            << " stiffness=" << stiffness
            << " damping=" << damping
            << " tangential_stiffness=" << tangentialStiffness
            << " tangential_damping=" << tangentialDamping
            << " friction=" << friction
            << " rolling_friction=" << rollingFriction
            << " base_clearance=" << baseClearance
            << " bed_surface=" << bedSurface
            << " normal_model=" << (hertz ? "hertz" : "linear")
            << " avg_anymal_contacts=" << static_cast<double>(totalAnymalContacts) / divisor
            << " avg_particle_contacts=" << static_cast<double>(totalParticleContacts) / divisor
            << " avg_boundary_contacts=" << static_cast<double>(totalBoundaryContacts) / divisor
            << " avg_rigid_contacts=" << static_cast<double>(totalRigidContacts) / divisor
            << " avg_rigid_candidate_pairs=" << static_cast<double>(totalRigidCandidatePairs) / divisor
            << " max_penetration=" << maxPenetration
            << " max_speed=" << maxSpeed
            << " unstable=" << (unstable ? 1 : 0)
            << " final_base_z=" << basePosition[2]
            << std::endl;

  server.killServer();
  return unstable ? 2 : 0;
}

#ifndef RAISIM_BENCHMARK_UNIFIED
int main(int argc, char* argv[]) { return run_granular_media_example(argc, argv); }
#endif
