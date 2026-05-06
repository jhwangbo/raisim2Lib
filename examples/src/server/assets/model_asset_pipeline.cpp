#include <filesystem>
#include <fstream>
#include <iostream>

#include "raisim/Path.hpp"
#include "raisim/World.hpp"
#include "raisim/object/singleBodies/Mesh.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  const std::string rscPath =
      (binaryPath.getDirectory() + "/rsc").getString();
  raisim::World::setActivationKey(rscPath + "/activation.raisim");

  const std::filesystem::path outputDir =
      std::filesystem::temp_directory_path() / "raisim_model_asset_pipeline_example";
  std::filesystem::create_directories(outputDir);

  const std::filesystem::path sourceMesh = outputDir / "source_tetra.obj";
  {
    std::ofstream out(sourceMesh);
    out << "v 0 0 0\n"
        << "v 1 0 0\n"
        << "v 0 1 0\n"
        << "v 0 0 1\n"
        << "f 1 2 3\n"
        << "f 1 2 4\n"
        << "f 1 3 4\n"
        << "f 2 3 4\n";
  }

  raisim::Mesh::PreprocessOptions preprocess;
  preprocess.cacheDirectory = (outputDir / "cache").string();
  const auto result = raisim::Mesh::preprocessMesh(sourceMesh.string(), preprocess);

  raisim::World world;
  world.setSleepingEnabled(false);
  auto* mesh = world.addMesh(result.outputPath, 1.0, 1.0, "default",
                             raisim::MeshCollisionMode::ORIGINAL_MESH);
  mesh->setName("preprocessed_tetra");
  mesh->setPosition(0.0, 0.0, 1.0);

  const auto exportedMeshes =
      world.exportMeshAssetsToObj((outputDir / "exported_meshes").string(), "scene");
  world.exportToXml((outputDir / "scene.xml").string());

  std::cout << "preprocessed mesh: " << result.outputPath << "\n";
  std::cout << "cache hit: " << (result.cacheHit ? "true" : "false") << "\n";
  for (const auto& meshPath : exportedMeshes) {
    std::cout << "exported mesh: " << meshPath << "\n";
  }
  std::cout << "exported world: " << (outputDir / "scene.xml").string() << "\n";
  return 0;
}
