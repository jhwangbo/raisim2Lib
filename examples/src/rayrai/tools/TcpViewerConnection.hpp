// Copyright (c) 2026 Raion Robotics Inc. All rights reserved.
#pragma once
#include "TcpViewerSimulation.hpp"
#include "rayrai/RaisimTcpCommon.hpp"

namespace raisin::tcp_viewer {
enum class ConnectionTarget { ExternalServer, LocalSimulation };

// A user-requested connection replaces the owned simulation, even if the new
// server is unavailable. Only the internal startup path preserves the child.
inline bool connectViewerEndpoint(TcpClient& client, LocalSimulation& simulation,
                                  bool& connectingLocalSimulation, ConnectionTarget target,
                                  const std::string& host, int port, bool verbose, int timeoutMs) {
  if (target == ConnectionTarget::ExternalServer) {
    connectingLocalSimulation = false;
    client.disconnect();
    simulation.stop();
  }
  const bool connected = client.connectTo(host, port, verbose, timeoutMs);
  if (target == ConnectionTarget::LocalSimulation) connectingLocalSimulation = !connected;
  return connected;
}
} // namespace raisin::tcp_viewer
