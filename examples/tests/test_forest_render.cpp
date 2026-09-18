#include <chrono>
#include <cmath>
#include <iostream>
#include "forest_viewer.hpp"
#include "forest_loading.hpp"
#include "forest_shadow_check.hpp"

// Bounded GPU checks and timing belong here, not in the interactive example.
int main(int argc, char** argv) {
  try {
    int frames=3;
    for (int i=1;i<argc;++i) {
      const std::string arg=argv[i];
      if (arg=="--hidden") continue;
      if (arg=="--frames" && i+1<argc) frames=std::stoi(argv[++i]);
      else throw std::runtime_error("Usage: forest_render_test [--hidden] [--frames N]");
    }
    if (frames<=0) throw std::runtime_error("Frame count must be positive");
    ExampleApp app;
    if (!app.init("Forest GPU test",1280,800,false)) return 1;
    ImGui::GetIO().IniFilename=nullptr;
    SDL_GL_SetSwapInterval(0);
    {
      const auto setupStart=std::chrono::steady_clock::now();
      ForestViewer forest(FOREST_ASSET_DIR,1280,800);
      std::cout << "Setup seconds: " << std::chrono::duration<double>(std::chrono::steady_clock::now()-setupStart).count() << std::endl;
      auto& viewer=*forest.viewer;
      ForestLoadingProgress loading;
      loading.update(viewer.pendingAsyncMeshLoadCount());
      app.beginFrame();
      const bool showedProgress=loading.draw();
      app.endFrame();
      constexpr int warmup=60;
      int rendered=0;
      double loadingSeconds=0;
      const auto loadingStart=std::chrono::steady_clock::now();
      size_t loadingUpdates=0;
      double longestLoadingUpdate=0;
      auto start=std::chrono::steady_clock::now();
      while (!app.quit && rendered<warmup+frames) {
        const auto updateStart=std::chrono::steady_clock::now();
        app.processEvents();
        if (app.quit) break;
        const bool ready=!loading.visible();
        if (ready) for (int step=0;step<8;++step) forest.world->integrate();
        app.beginFrame();
        app.renderViewer(viewer);
        loading.update(viewer.pendingAsyncMeshLoadCount());
        loading.draw();
        app.endFrame();
        gl::glFinish();
        if (!ready) {
          ++loadingUpdates;
          longestLoadingUpdate=std::max(longestLoadingUpdate,
            std::chrono::duration<double>(std::chrono::steady_clock::now()-updateStart).count());
          continue;
        }
        if (!rendered)
          loadingSeconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-loadingStart).count();
        if (++rendered==warmup) start=std::chrono::steady_clock::now();
      }
      if (rendered!=warmup+frames || !viewer.asyncMeshLoadingEnabled() ||
          !showedProgress || loading.visible())
        throw std::runtime_error("Async rendering lifecycle did not complete");
      std::cout << "Loading seconds: " << loadingSeconds << '\n';
      std::cout << "Loading updates: " << loadingUpdates
                << ", longest update seconds: " << longestLoadingUpdate << '\n';
      if (loadingUpdates<16 || longestLoadingUpdate>1.0)
        throw std::runtime_error("Loading blocked event/render processing for more than one second");
      const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      std::cout << "Completed-frame FPS: " << frames/seconds << '\n';
      for (const auto& group:{forest.foliage,forest.rockVisuals})
        for (const auto& visual:group)
          if (!visual->sourceMeshCountForDiagnostics() || !visual->castsShadows())
            throw std::runtime_error("Forest asset failed to load or cast shadows");
      const auto atmosphere=viewer.weatherDiagnostics();
      const auto& quality=viewer.getRenderQualitySettings();
      // Check the applied weather/render path: light haze at distance, with no
      // volumetric pass or loss of close-range visibility.
      if (!atmosphere.heightFogActive || !quality.heightFogEnabled ||
          !quality.fogColorOverrideEnabled || quality.heightFogDensity<=0.f ||
          atmosphere.visibilityTransmittance100m<.65f ||
          atmosphere.visibilityTransmittance100m>.95f ||
          std::exp(-quality.heightFogDensity*5.f)<.97f ||
          quality.volumetricFogEnabled)
        throw std::runtime_error("Forest haze must be subtle and active after rendering");
      std::cout << "Haze transmittance at 100 m: "
                << atmosphere.visibilityTransmittance100m << '\n';
#ifdef FOREST_VERIFY_SHADOWS
      checkForestShadows(app,viewer,forest.foliage);
#endif
    }
    app.shutdown();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
