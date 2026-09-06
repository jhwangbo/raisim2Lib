// Copyright (c) 2026 Raion Robotics Inc. All rights reserved.
#include "TcpViewerSimulation.hpp"
#include "raisim/RaisimServer.hpp"
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>
#if defined(_WIN32)
#include <shellapi.h>
#else
#include <poll.h>
#include <unistd.h>
#endif

namespace raisin::tcp_viewer {
namespace {
volatile std::sig_atomic_t stopRequested = 0;
void stopWorker(int) { stopRequested = 1; }
bool parentClosedPipe() {
#if defined(_WIN32)
  DWORD bytes = 0;
  return !PeekNamedPipe(GetStdHandle(STD_INPUT_HANDLE), nullptr, 0, nullptr, &bytes, nullptr);
#else
  pollfd pipe{STDIN_FILENO, POLLIN | POLLHUP, 0};
  return poll(&pipe, 1, 0) > 0 && (pipe.revents & (POLLHUP | POLLERR | POLLNVAL));
#endif
}
#if !defined(_WIN32)
void closeInheritedDescriptors() {
  // posix_spawn may inherit viewer sockets owned by third-party code. Release
  // them before loading the world; only stdin and the redirected log are needed.
  std::vector<int> descriptors;
  std::error_code error;
#if defined(__APPLE__)
  const char* directory = "/dev/fd";
#else
  const char* directory = "/proc/self/fd";
#endif
  for (const auto& entry : std::filesystem::directory_iterator(directory, error)) {
    try {
      const int fd = std::stoi(entry.path().filename().string());
      if (fd > STDERR_FILENO) descriptors.push_back(fd);
    } catch (...) {}
  }
  for (int fd : descriptors) close(fd);
}
#endif
} // namespace

int runXmlSimulationWorker(int argc, char** argv) {
  namespace fs = std::filesystem;
  if (argc != 5) return 2;
  fs::path xml(argv[2]), ready(argv[3]), key(argv[4]);
#if defined(_WIN32)
  int count = 0;
  if (auto** wide = CommandLineToArgvW(GetCommandLineW(), &count)) {
    if (count == 5) { xml = wide[2]; ready = wide[3]; key = wide[4]; }
    LocalFree(wide);
  }
#else
  closeInheritedDescriptors();
#endif
  std::signal(SIGINT, stopWorker);
  std::signal(SIGTERM, stopWorker);
  raisim::ScopedRaiSimFatalCallback exceptions([] { throw std::runtime_error("RaiSim failed to load or simulate the world; see process log"); });
  try {
    if (!key.empty()) raisim::World::setActivationKey(key.string());
    std::string error;
    if (classifyDroppedScene(xml, error) != DroppedSceneKind::World)
      throw std::runtime_error(error.empty() ? "Expected a RaiSim world XML" : error);
    raisim::World world(xml.string());
    const double dt = world.getTimeStep();
    if (!std::isfinite(dt) || dt <= 0.) throw std::runtime_error("World timestep must be positive and finite");
    if (parentClosedPipe() || stopRequested) return 0;
    raisim::RaisimServer server(&world);
    server.setBindLoopbackOnly(true);
#if defined(_WIN32)
    const unsigned long processId = GetCurrentProcessId();
#else
    const unsigned long processId = static_cast<unsigned long>(getpid());
#endif
    const int requestedPort = 20000 + static_cast<int>(processId % 20000);
    server.launchServer(requestedPort);
    try {
      if (server.getPort() < requestedPort) throw std::runtime_error("Cannot bind the local simulation server");
      const fs::path temporary = ready.string() + ".tmp";
      std::ofstream out(temporary);
      out << server.getPort() << '\n'; out.close();
      if (!out) throw std::runtime_error("Cannot publish simulation endpoint");
      fs::rename(temporary, ready);
      std::cout << "Loaded " << xml.string() << " on 127.0.0.1:" << server.getPort() << std::endl;
      using Clock = std::chrono::steady_clock;
      const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(dt));
      auto next = Clock::now();
      bool connectedOnce = false;
      while (!stopRequested && !parentClosedPipe()) {
        connectedOnce = connectedOnce || server.isConnected();
        // Preserve the initial state while the viewer initializes its first frame.
        if (connectedOnce) server.integrateWorldThreadSafe();
        next += period;
        const auto now = Clock::now();
        if (next < now - std::chrono::milliseconds(100)) next = now;
        // Check lifetime signals frequently even for a large authored timestep.
        while (!stopRequested && !parentClosedPipe() && Clock::now() < next)
          std::this_thread::sleep_until(std::min(next, Clock::now() + std::chrono::milliseconds(10)));
      }
    } catch (...) { server.killServer(); throw; }
    server.killServer();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    std::ofstream(ready.parent_path() / "error.txt") << error.what() << '\n';
    return 1;
  }
}
} // namespace raisin::tcp_viewer
