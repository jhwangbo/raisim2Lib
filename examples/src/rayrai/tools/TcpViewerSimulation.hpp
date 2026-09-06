// Copyright (c) 2026 Raion Robotics Inc. All rights reserved.
#pragma once
#include <filesystem>
#include <memory>
#include <string>

namespace raisin::tcp_viewer {
enum class DroppedSceneKind { World, Robot, Invalid };
DroppedSceneKind classifyDroppedScene(const std::filesystem::path& path, std::string& error);

// Owns only the process launched for a dropped world. Poll from the UI thread;
// loading and physics run in the child, without blocking the viewer's event loop.
class LocalSimulation {
 public:
  enum class State { Idle, Starting, Running, Failed };
  LocalSimulation();
  ~LocalSimulation();
  LocalSimulation(const LocalSimulation&) = delete;
  LocalSimulation& operator=(const LocalSimulation&) = delete;
  bool start(const std::filesystem::path& executable, const std::filesystem::path& xml,
             const std::filesystem::path& activationKey, std::string& error);
  void poll();
  void stop();
  State state() const;
  bool active() const;
  int port() const;
  const std::string& error() const;
  const std::filesystem::path& worldPath() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Internal mode of the same executable; runs before any SDL/OpenGL initialization.
int runXmlSimulationWorker(int argc, char** argv);
std::filesystem::path viewerExecutablePath(const char* argv0);
} // namespace raisin::tcp_viewer
