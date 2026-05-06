// This file is part of RaiSim. You must obtain a valid license from RaiSim Tech
// Inc. prior to usage.

#include "benchmarkCommon.hpp"

#include <Eigen/Dense>
#include <raisim/World.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace {

raisim::Path findRscDir(const raisim::Path& binaryDir) {
  const std::vector<std::string> suffixes = {
      "/rsc",
      "/../rsc",
      "/../../rsc",
      "/../../../rsc",
  };

  for (const auto& suffix : suffixes) {
    raisim::Path candidate(binaryDir.getString() + suffix);
    if (candidate.directoryExists()) return candidate;
  }

  std::cerr << "[anymal_standing_benchmark] Could not find `rsc` directory. Tried:";
  for (const auto& suffix : suffixes) std::cerr << " " << (binaryDir.getString() + suffix);
  std::cerr << std::endl;
  return raisim::Path("");
}

int parseStepsArg(int argc, char** argv, int defaultSteps) {
  int steps = defaultSteps;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if ((arg == "--steps" || arg == "--loops") && i + 1 < argc) {
      steps = std::max(1, std::atoi(argv[++i]));
    } else if (arg.rfind("--steps=", 0) == 0) {
      steps = std::max(1, std::atoi(arg.c_str() + 8));
    } else if (arg.rfind("--loops=", 0) == 0) {
      steps = std::max(1, std::atoi(arg.c_str() + 8));
    }
  }
  return steps;
}

bool wantsFastMode(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--fast") == 0 ||
        std::strcmp(argv[i], "--quick") == 0) {
      return true;
    }
  }
  return false;
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

}  // namespace

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  const raisim::Path binaryDir = binaryPath.getDirectory();
  const raisim::Path rscDir = findRscDir(binaryDir);
  if (rscDir.getString().empty()) return 1;

  raisim::World::setActivationKey(rscDir.getString() + "/activation.raisim");

  const char* anymalStepsEnv = std::getenv("ANYMAL_STEPS");
  int steps = anymalStepsEnv ? std::atoi(anymalStepsEnv) : 1000000;
  if (wantsFastMode(argc, argv)) {
    steps = std::min(steps, 20000);
  }
  steps = parseStepsArg(argc, argv, steps);

  const bool phaseProfile = []() {
    const char* env = std::getenv("RAISIM_ANYMAL_PHASE_PROFILE");
    return env && std::strcmp(env, "1") == 0;
  }();

  raisim::World world;
  world.setSleepingEnabled(false);
  world.setTimeStep(0.002);
  world.addGround();

  auto anymal = world.addArticulatedSystem(rscDir.getString() + "/anymal/urdf/anymal.urdf");
  Eigen::VectorXd jointConfig, jointVel, jointPgain, jointDgain, jointVelocityTarget;
  buildAnymalDefaults(anymal, jointConfig, jointVel, jointPgain, jointDgain, jointVelocityTarget);
  anymal->setState(jointConfig, jointVel);
  anymal->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  anymal->setPdGains(jointPgain, jointDgain);
  anymal->setPdTarget(jointConfig, jointVelocityTarget);
  anymal->setGeneralizedForce(Eigen::VectorXd::Zero(anymal->getDOF()));

  std::size_t contactTotal = 0;
  std::uint64_t integrate1Ns = 0;
  std::uint64_t integrate2Ns = 0;
  const auto begin = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) {
    if (phaseProfile) {
      const auto p1Begin = std::chrono::steady_clock::now();
      world.integrate1();
      const auto p1End = std::chrono::steady_clock::now();
      const auto p2Begin = p1End;
      world.integrate2();
      const auto p2End = std::chrono::steady_clock::now();
      integrate1Ns += static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(p1End - p1Begin).count());
      integrate2Ns += static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(p2End - p2Begin).count());
    } else {
      world.integrate();
    }
    contactTotal += world.getContactProblem()->size();
  }
  const auto end = std::chrono::steady_clock::now();
  raisim::print_timediff("ANYmal standing (raisim)", steps, begin, end);
  const double avgContacts =
      steps > 0 ? static_cast<double>(contactTotal) / static_cast<double>(steps) : 0.0;
  std::cout << "ANYmal avg contacts (raisim): " << avgContacts << std::endl;
  if (phaseProfile && steps > 0) {
    std::cout << "[anymal_phase_profile][raisim] integrate1_us_per_step="
              << (static_cast<double>(integrate1Ns) / 1e3 / static_cast<double>(steps))
              << " integrate2_us_per_step="
              << (static_cast<double>(integrate2Ns) / 1e3 / static_cast<double>(steps))
              << std::endl;
  }

  return 0;
}
