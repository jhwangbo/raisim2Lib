// Copyright (c) 2026 Raion Robotics Inc.
// All rights reserved.

#pragma once

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>

#include <imgui/imgui.h>

#include "rayrai/RayraiWindow.hpp"
#include "rayrai/TcpSceneSync.hpp"

namespace raisin
{
namespace tcp_viewer
{

/**
 * Fills progress bars with the viewer theme's accent, the colour of slider handles and check
 * marks, while it lives. The theme's own progress-bar colour (ImGuiCol_PlotHistogram) is a pure
 * red that matches nothing else in the viewer.
 */
class SceneProgressBarStyle {
 public:
  SceneProgressBarStyle() {
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImGui::GetStyleColorVec4(ImGuiCol_SliderGrab));
  }
  ~SceneProgressBarStyle() { ImGui::PopStyleColor(); }
  SceneProgressBarStyle(const SceneProgressBarStyle&) = delete;
  SceneProgressBarStyle& operator=(const SceneProgressBarStyle&) = delete;
};

/**
 * Narrows the pane area in which the scene overlays (download prompt and progress, mesh
 * progress) are centred at the top so that they clear a panel along the pane's left edge,
 * such as the viewer's left overlay, whose right edge is @p panelRight. The area stays as it
 * is when an overlay centred in it already clears the panel. In a pane too narrow for both, the
 * overlay keeps a width of 12 font sizes and covers part of the panel.
 */
inline void clearSceneOverlayArea(ImVec2& origin, ImVec2& size, float panelRight) {
  const float margin = ImGui::GetFontSize();
  const float width = std::min(size.x - 2.0f * margin, margin * 30.0f);
  if (origin.x + 0.5f * (size.x - width) >= panelRight + margin) return;
  const float right = origin.x + size.x;
  const float left = std::max(origin.x, std::min(panelRight, right - margin * 14.0f));
  size.x = right - left;
  origin.x = left;
}

/**
 * Begins a scene overlay window centred at the top of the area; call ImGui::End() whatever it
 * returns.
 * @param contentWidth Set to the width text and progress bars may take inside the window.
 */
inline bool beginSceneOverlay(const char* name, int paneId, const ImVec2& origin,
                              const ImVec2& size, float backgroundAlpha, float& contentWidth) {
  const float margin = ImGui::GetFontSize();
  const float width = std::max(std::min(size.x - 2.0f * margin, margin * 30.0f), 1.0f);
  contentWidth = std::max(width - 2.0f * ImGui::GetStyle().WindowPadding.x, 1.0f);
  ImGui::SetNextWindowPos(ImVec2(origin.x + size.x * 0.5f, origin.y + margin), ImGuiCond_Always,
                          ImVec2(0.5f, 0.0f));
  ImGui::SetNextWindowSizeConstraints(ImVec2(0.0f, 0.0f), ImVec2(width, FLT_MAX));
  ImGui::SetNextWindowBgAlpha(backgroundAlpha);
  const std::string id = std::string(name) + "##pane" + std::to_string(paneId);
  constexpr ImGuiWindowFlags kFlags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
    ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize |
    ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
    ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoMove;
  return ImGui::Begin(id.c_str(), nullptr, kFlags);
}

/**
 * Loads the meshes of the server's scene in the background while the pane shows their progress.
 *
 * The TCP viewer loads mesh files synchronously, so a streamed body is drawn with its mesh in the
 * frame it appears. Applying a scene that way stalls the pane until every mesh is ready (seconds
 * for a large scene) with nothing drawn. update() applies the scene with asynchronous loading
 * instead and finishes the meshes that are ready for up to kFrameBudget per frame. Between its
 * calls, meshes requested by anything else still load synchronously
 * (RayraiWindow::SynchronousMeshLoadingScope) while the scene's keep loading in the background.
 * Once none is pending, the viewer is back to plain synchronous loading.
 *
 * Inline, so code that compiles the viewer's frame loop against a prebuilt rayrai gets it too.
 */
class SceneMeshLoading {
 public:
  /// Time spent per frame finishing ready meshes while a scene's meshes load.
  static constexpr std::chrono::milliseconds kFrameBudget{8};

  SceneMeshLoading() = default;
  SceneMeshLoading(const SceneMeshLoading&) = delete;
  SceneMeshLoading& operator=(const SceneMeshLoading&) = delete;

  /**
   * Call once per frame, with @p applyScene applying the scene when it is ready
   * (ViewerSceneFiles::update()): runs it with asynchronous mesh loading, then finishes ready
   * meshes for up to kFrameBudget.
   * @return What applyScene returned: true in the frame the scene was applied.
   */
  template <class ApplyScene>
  bool update(RayraiWindow& viewer, ApplyScene&& applyScene) {
    synchronous_.reset();
    viewer.setAsyncMeshLoadingEnabled(true);
    const bool applied = applyScene();
    if (applied) {
      shown_ = true;
      peakPending_ = 0;
      progress_ = 0.0f;
      start_ = Clock::now();
    }
    const auto frameStart = Clock::now();
    pending_ = viewer.pendingAsyncMeshLoadCount();
    while (pending_ > 0 && Clock::now() - frameStart < kFrameBudget) {
      viewer.pollAsyncMeshLoads(1);
      pending_ = viewer.pendingAsyncMeshLoadCount();
    }
    if (pending_ == 0) {
      viewer.setAsyncMeshLoadingEnabled(false);
      shown_ = false;
      return applied;
    }
    synchronous_.emplace(viewer);
    // Pending imports and LOD jobs; a finished import can start a LOD job, so the count may
    // rise. The bar never moves back.
    peakPending_ = std::max(peakPending_, pending_);
    progress_ = std::max(progress_,
      1.0f - static_cast<float>(pending_) / static_cast<float>(peakPending_));
    return applied;
  }

  /// Hides the progress of a scene that was removed (disconnect); its meshes finish in update().
  void hide() { shown_ = false; }

  /// True while meshes load in the background.
  bool loading() const { return pending_ > 0; }
  /// True while the progress of an applied scene's meshes is shown.
  bool shown() const { return shown_; }
  /// Share of the scene's mesh jobs finished, in [0, 1]; it only grows while they load.
  float progress() const { return progress_; }

  /// Draws the progress at the top of the area, where drawSceneOverlay() shows the download.
  void drawOverlay(const ImVec2& origin, const ImVec2& size, int paneId,
                   const std::string& sceneName) const {
    if (!shown_) return;
    float contentWidth = 0.0f;
    if (beginSceneOverlay("Scene meshes", paneId, origin, size, 0.85f, contentWidth)) {
      ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + contentWidth);
      ImGui::Text("Loading the meshes of %s", sceneName.empty() ? "the scene" : sceneName.c_str());
      ImGui::PopTextWrapPos();
      char label[32];
      std::snprintf(label, sizeof(label), "%.0f%% (%.0f s)", 100.0f * progress_,
                    std::chrono::duration<double>(Clock::now() - start_).count());
      SceneProgressBarStyle style;
      ImGui::ProgressBar(progress_, ImVec2(contentWidth, 0.0f), label);
    }
    ImGui::End();
  }

 private:
  using Clock = std::chrono::steady_clock;

  /// Keeps other mesh requests synchronous between update() calls while the scene's load.
  std::optional<RayraiWindow::SynchronousMeshLoadingScope> synchronous_;
  size_t pending_ = 0;
  size_t peakPending_ = 0;
  float progress_ = 0.0f;
  bool shown_ = false;
  Clock::time_point start_;
};

/// Bytes as "12.3 MB".
inline std::string sceneOverlayMegabytes(uint64_t bytes) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.1f MB", static_cast<double>(bytes) / (1024.0 * 1024.0));
  return text;
}

/**
 * Draws the scene's overlay at the top of the pane: the download prompt, the download progress,
 * the loading of the scene file, a failure, the bar of a declined scene, or else the progress of
 * the scene's meshes. It stays clear of a panel along the pane's left edge whose right edge is
 * @p panelRight (clearSceneOverlayArea()).
 * @param errorDismissed The pane's flag for a dismissed failure; set when the user dismisses it.
 */
inline void drawSceneOverlay(SceneSync& sync, bool& errorDismissed,
                             const SceneMeshLoading& meshLoading, ImVec2 origin, ImVec2 size,
                             int paneId, float panelRight) {
  using State = SceneSync::State;
  clearSceneOverlayArea(origin, size, panelRight);
  const State state = sync.state();
  const bool prompt = state == State::AwaitingConsent;
  const bool busy = state == State::Downloading || state == State::Finishing;
  const bool failed = state == State::Failed && !errorDismissed;
  if (!prompt && !busy && !failed && state != State::Declined) {
    meshLoading.drawOverlay(origin, size, paneId, sync.sceneName());
    return;
  }
  float contentWidth = 0.0f;
  const float alpha = state == State::Declined ? 0.45f : 0.85f;
  if (!beginSceneOverlay("Scene files", paneId, origin, size, alpha, contentWidth)) {
    ImGui::End();
    return;
  }
  ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + contentWidth);
  const std::string scene = sync.sceneName().empty() ? "the scene" : sync.sceneName();
  if (prompt) {
    ImGui::TextUnformatted("Download the server's scene?");
    ImGui::Separator();
    ImGui::Text("%s offers %zu file%s (%s). Accept to check local files and download missing "
                "resources.", scene.c_str(), sync.missingFileCount(),
                sync.missingFileCount() == 1 ? "" : "s",
                sceneOverlayMegabytes(sync.missingBytes()).c_str());
    // The copy is staged in the scene directory.
    ImGui::TextDisabled("They are saved in %s",
      raisim::server::scene_files::pathUtf8(sync.copyDirectory().parent_path()).c_str());
    ImGui::TextDisabled("Add a resource directory that holds them to skip the download.");
    if (ImGui::Button("Download")) sync.acceptDownload();
    ImGui::SameLine();
    if (ImGui::Button("Not now")) sync.declineDownload();
  } else if (state == State::Downloading) {
    const uint64_t total = std::max<uint64_t>(1, sync.missingBytes());
    ImGui::Text("Downloading %s", scene.c_str());
    const std::string progress = sceneOverlayMegabytes(sync.downloadedBytes()) + " / " +
      sceneOverlayMegabytes(sync.missingBytes());
    {
      SceneProgressBarStyle style;
      ImGui::ProgressBar(static_cast<float>(static_cast<double>(sync.downloadedBytes()) / total),
                         ImVec2(contentWidth, 0.0f), progress.c_str());
    }
    if (ImGui::Button("Cancel")) sync.cancelDownload();
  } else if (state == State::Finishing) {
    ImGui::Text("Loading %s...", scene.c_str());
  } else if (failed) {
    ImGui::TextUnformatted("Scene files");
    ImGui::Separator();
    ImGui::TextUnformatted(sync.error().c_str());
    if (ImGui::Button("Dismiss")) errorDismissed = true;
  } else {
    // Declined: the scene stays without its visuals until the user asks for it.
    ImGui::Text("%s: visuals not downloaded", scene.c_str());
    ImGui::SameLine();
    if (ImGui::SmallButton("Download")) sync.acceptDownload();
  }
  ImGui::PopTextWrapPos();
  ImGui::End();
}

} // namespace tcp_viewer
} // namespace raisin
