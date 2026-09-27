#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include "forest_scene.hpp"
#include "raisim/Rasset.hpp"

namespace {
std::unique_ptr<raisim::World> makeWorld(bool colliders) {
  auto world = std::make_unique<raisim::World>();
  auto* terrain = forest::addTerrain(*world);
  const auto plants = forest::scatter(*terrain);
  forest::addObjects(*world, *terrain, plants);
  const auto rocks = forest::scatterRocks(*terrain, plants);
  if (colliders) {
    const std::filesystem::path root = FOREST_ASSET_DIR;
    const std::array<const char*, 3> treeNames = {
      "pine_sapling_small", "fir_sapling", "tree_small_02"};
    std::array<raisim::Rasset, 3> trees;
    std::array<raisim::Rasset, 6> stones;
    for (size_t i = 0; i < trees.size(); ++i)
      trees[i] = raisim::Rasset::load(root/treeNames[i]/"model.rasset");
    for (size_t i = 0; i < stones.size(); ++i)
      stones[i] = raisim::Rasset::load(root/("rock_moss_" + std::to_string(i))/"model.rasset");
    for (const auto& p : plants) if (p.type < 3)
      trees[p.type].addStaticColliders(*world, {p.x, p.y, p.z}, p.yaw, p.scale);
    for (const auto& p : rocks)
      stones[p.type].addStaticColliders(*world, {p.x, p.y, p.z}, p.yaw, p.scale);
  }
  for (size_t i = 0; i < 24; ++i) {
    const auto& p = rocks[i];
    auto* sphere = world->addSphere(.16, 1);
    sphere->setPosition(p.x, p.y, p.z + 1.2);
  }
  return world;
}

double measure(raisim::World& world, int steps) {
  for (int i = 0; i < 100; ++i) world.integrate();
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) world.integrate();
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count() / steps;
}
} // namespace

int main(int argc, char** argv) try {
  const int steps = argc == 2 ? std::atoi(argv[1]) : 1000;
  if (steps <= 0 || argc > 2) throw std::runtime_error("Usage: forest_collision_benchmark [steps]");
  auto baseline = makeWorld(false);
  auto forest = makeWorld(true);
  const auto baselineMs = measure(*baseline, steps);
  const auto forestMs = measure(*forest, steps);
  std::cout << "Bodies: " << baseline->getObjList().size() << " baseline, "
            << forest->getObjList().size() << " with tree/rock proxies\n"
            << "Physics milliseconds/step: " << baselineMs << " baseline, "
            << forestMs << " with proxies\n";
  return 0;
} catch (const std::exception& error) {
  std::cerr << error.what() << '\n'; return 1;
}
