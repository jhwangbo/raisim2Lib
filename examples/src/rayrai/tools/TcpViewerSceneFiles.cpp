// Copyright (c) 2026 Raion Robotics Inc.
// All rights reserved.

#include "TcpViewerSceneFiles.hpp"

#include <exception>
#include <utility>

#include "rayrai/RaisimTcpCommon.hpp"
#include "rayrai/RayraiWindow.hpp"
#include "rayrai/Visuals.hpp"

namespace raisin
{
namespace tcp_viewer
{

namespace
{

SceneSync::Options syncOptions(std::filesystem::path cacheRoot) {
  SceneSync::Options options;
  options.cacheRoot = std::move(cacheRoot);
  return options;
}

} // namespace

ViewerSceneFiles::ViewerSceneFiles(std::filesystem::path cacheRoot)
  : sync_(syncOptions(std::move(cacheRoot))) {}

void ViewerSceneFiles::setResourceDirs(const std::vector<std::string>& dirs) {
  sync_.setResourceDirs({dirs.begin(), dirs.end()});
}

void ViewerSceneFiles::reset(RayraiWindow& viewer, RemoteScene& scene) {
  remove(viewer);
  scene.setServerPathMap({});
  sync_.reset();
  errorDismissed_ = false;
}

bool ViewerSceneFiles::update(RayraiWindow& viewer, RemoteScene& scene) {
  return meshLoading_.update(viewer, [&] { return apply(viewer, scene); });
}

bool ViewerSceneFiles::apply(RayraiWindow& viewer, RemoteScene& scene) {
  sync_.poll();
  if (sync_.state() != SceneSync::State::Ready) return false;
  // The sync starts over when the server stops and resumes sharing; names must stay unique.
  remove(viewer);
  // The scene's render settings replace the removed ones right away.
  viewerSettingsReset_ = false;
  touched_ = true;
  try {
    // The server's scene is not the viewer's world scene, so it is applied here. On failure,
    // applyRscene() removes the visuals it created before it throws.
    visuals_ = rayrai_internal::applyRscene(*sync_.scene(), viewer,
                                            rsceneRenderSettings(*sync_.scene()));
  } catch (const std::exception& error) {
    remove(viewer);
    sync_.markFailed(std::string("The scene could not be shown: ") + error.what());
    return false;
  }
  // Bodies the server streams name their mesh by its server path.
  scene.setServerPathMap(sync_.pathMap());
  sync_.markApplied();
  applied_ = true;
  return true;
}

bool ViewerSceneFiles::settled(const RayraiWindow& viewer) const {
  switch (sync_.state()) {
    case SceneSync::State::Idle:
    case SceneSync::State::Declined:
    case SceneSync::State::Failed:
      return true;
    case SceneSync::State::Applied:
      return viewer.pendingAsyncMeshLoadCount() == 0;
    default:
      return false;
  }
}

void ViewerSceneFiles::remove(RayraiWindow& viewer) {
  for (const auto& visual : visuals_.objects)
    if (visual) viewer.removeVisualObject(visual->getName());
  for (const auto* batches : {&visuals_.instancedVisuals, &visuals_.foliage})
    for (const auto& batch : *batches)
      if (batch) viewer.removeInstancedVisuals(batch->getName());
  visuals_ = {};
  applied_ = false;
  meshLoading_.hide();
  if (!touched_) return;
  touched_ = false;
  viewerSettingsReset_ = true;
  // What applyRscene() set on the viewer; the pane re-applies its own settings afterwards.
  viewer.clearAdditionalLights();
  viewer.clearLocalFogVolumes();
  viewer.clearProjectedDecals();
  viewer.clearIrradianceVolumes();
  viewer.clearReflectionProbes();
  viewer.clearEnvironmentBackground();
  viewer.clearWeatherSkyEnvironment();
  viewer.setHeightmapPatternResourcePath("");
  viewer.setHeightmapNormalResourcePath("");
  viewer.setHeightmapHeightResourcePath("");
  viewer.clearHeightmapSplatLayer();
}

void ViewerSceneFiles::drawOverlay(const ImVec2& paneOrigin, const ImVec2& paneSize, int paneId,
                                   float panelRight) {
  drawSceneOverlay(sync_, errorDismissed_, meshLoading_, paneOrigin, paneSize, paneId, panelRight);
}

} // namespace tcp_viewer
} // namespace raisin
