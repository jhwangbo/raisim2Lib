// Copyright (c) 2026 Raion Robotics Inc.
// All rights reserved.

#pragma once

#include <cfloat>
#include <filesystem>
#include <string>
#include <vector>

#include <imgui/imgui.h>

#include "rayrai/RaisimTcpCommon.hpp"
#include "rayrai/RsceneVisuals.hpp"
#include "rayrai/TcpSceneSync.hpp"
#include "TcpViewerSceneLoading.hpp"

namespace raisin
{
class RayraiWindow;

namespace tcp_viewer
{
class RemoteScene;

/**
 * The server's RaiSim Engine scene in one viewer pane: finding or downloading its files
 * (SceneSync), the prompt that asks before a download, and the scene's visuals, lights and
 * render settings once applied. The server streams the physics bodies; the scene adds what
 * rayrai draws for it locally (raisin::rayrai_internal::applyRscene()). The scene's meshes load
 * in the background while the pane shows their progress (SceneMeshLoading).
 */
class ViewerSceneFiles {
 public:
  /// @param cacheRoot Scene directory (SceneSync::Options::cacheRoot); empty uses
  ///        defaultSceneDirectory(), ~/.raisim/scenes.
  explicit ViewerSceneFiles(std::filesystem::path cacheRoot = {});

  /// The viewer's resource directories, searched for the scene's files.
  void setResourceDirs(const std::vector<std::string>& dirs);
  /// Disconnect: removes the applied scene from @p viewer and forgets it.
  void reset(RayraiWindow& viewer, RemoteScene& scene);

  /// Scene-file block of a live response (not of a replayed one) from @p server. A server on
  /// this computer, such as a --simulate child, names this computer's files, so they load in
  /// place without a prompt (SceneSync::setServerPathsTrusted()). The socket's peer address
  /// decides it (TcpClient::peerIsLoopback()), not the host name the user typed.
  void receive(SceneFileBlock&& block, const TcpClient& server) {
    sync_.setServerPathsTrusted(server.peerIsLoopback());
    sync_.receive(std::move(block));
  }
  /// Requests for the next update.
  void appendRequests(std::vector<ClientRequest>& requests) { sync_.appendRequests(requests); }
  /**
   * Advances the background work, applies a ready scene and loads its meshes; call it once
   * per frame.
   * @return True in the frame the scene was applied.
   */
  bool update(RayraiWindow& viewer, RemoteScene& scene);

  /// True while the scene's render settings, lights and camera are in effect; the viewer
  /// then does not drive the main light and camera lens from its own settings.
  bool applied() const { return applied_; }
  /// True once after the scene's lights and render settings were removed from the viewer;
  /// the pane then re-applies its own.
  bool takeViewerSettingsReset() {
    const bool reset = viewerSettingsReset_;
    viewerSettingsReset_ = false;
    return reset;
  }
  const SceneSync& sync() const { return sync_; }
  /// False while the scene is on its way (the server lists its files, the viewer checks,
  /// downloads, loads or applies them, or its meshes load); a screenshot waits for it.
  /// True without a scene, at the download prompt, after a decline or a failure.
  bool settled(const RayraiWindow& viewer) const;

  /// Draws the download prompt or progress, or the progress of the scene's meshes, at the top
  /// of the pane (drawSceneOverlay()), clear of a panel along its left edge that ends at
  /// @p panelRight, such as the viewer's left overlay.
  void drawOverlay(const ImVec2& paneOrigin, const ImVec2& paneSize, int paneId,
                   float panelRight = -FLT_MAX);

 private:
  /// Applies a ready scene. @return True when it was applied.
  bool apply(RayraiWindow& viewer, RemoteScene& scene);
  void remove(RayraiWindow& viewer);

  SceneSync sync_;
  RsceneVisuals visuals_;
  bool applied_ = false;
  /// applyRscene() ran, possibly failing part way; the viewer holds scene state.
  bool touched_ = false;
  bool viewerSettingsReset_ = false;
  bool errorDismissed_ = false;
  SceneMeshLoading meshLoading_;
};

} // namespace tcp_viewer
} // namespace raisin
