// Copyright (c) 2026 Raion Robotics Inc.
// All rights reserved.

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace raisin::tcp_viewer
{

inline constexpr int kDiscoveryPort = 59312;
inline constexpr const char* kDiscoveryMagic = "RAISIM_TCP_DISCOVERY_V1";
inline constexpr std::chrono::seconds kDiscoveryBeaconTimeout{8};

struct DiscoveredServer {
  std::string endpointHost;
  int endpointPort = 0;
  std::string bindHost;
  std::string process;
  std::string protocol;
  std::unordered_map<std::string, std::string> metadata;
  bool remoteBeacon = false;
  std::chrono::steady_clock::time_point lastSeen;
};

bool isCompatibleDiscoveryVersion(int version);
bool parseDiscoveryBeacon(const std::string& payload, int& port, int& version,
                          std::string& hostname,
                          std::unordered_map<std::string, std::string>& metadata);

class DiscoveryBeaconReceiver {
 public:
  DiscoveryBeaconReceiver();
  ~DiscoveryBeaconReceiver();

  DiscoveryBeaconReceiver(const DiscoveryBeaconReceiver&) = delete;
  DiscoveryBeaconReceiver& operator=(const DiscoveryBeaconReceiver&) = delete;

  bool start(std::string& status);
  bool poll();
  std::vector<DiscoveredServer> servers() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace raisin::tcp_viewer
