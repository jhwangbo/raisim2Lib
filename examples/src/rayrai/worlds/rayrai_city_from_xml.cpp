// The rayrai_city world, loaded from a RaiSim XML world file. The XML file
// (rsc/xmlScripts/rscene/anymal_in_city.xml) describes only the ANYmal C, its
// initial state and its joint PD gains, and includes the city's .rscene file
// for everything else, so the robot and the scene stay in separate files.
#include "example_resources.hpp"
#include "rayrai/example_common.hpp"
#include "rayrai_loading_progress.hpp"

int main(int argc, char* argv[]) {
  auto world = std::make_shared<raisim::World>(exampleRscPath(argv[0], "xmlScripts/rscene/anymal_in_city.xml"));
  ExampleApp app;
  if (!app.init("RaiSim world from XML", 1280, 800)) return 1;
  {
    // The window draws every body of the world, the robot included, and applies the
    // included scene: sky, sun, instanced buildings, visual meshes and the saved camera.
    raisin::RayraiWindow viewer(world, 1280, 800);
    // Meshes load asynchronously by default, so this line changes nothing; it shows the
    // switch. setAsyncMeshLoadingEnabled(false) loads the scene's meshes before it returns.
    viewer.setAsyncMeshLoadingEnabled(true);
    RayraiLoadingProgress loading("Loading city assets");
    loading.update(viewer.pendingAsyncMeshLoadCount());
    while (!app.quit) {
      app.processEvents();
      // Physics starts once the meshes are loaded: eight 2 ms steps per frame.
      if (!loading.visible())
        for (int step = 0; step < 8; ++step) world->integrate();
      app.beginFrame();
      app.renderViewer(viewer);
      loading.update(viewer.pendingAsyncMeshLoadCount());
      loading.draw();
      app.endFrame();
    }
  }  // Release graphics resources before closing the OpenGL context.
  app.shutdown();
  return 0;
}
