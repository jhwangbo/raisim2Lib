#include "tendon_example_runner.hpp"

using namespace raisim_examples::tendons;
namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void compare(Scene& original, Scene& restored) {
  for (const auto* object:original.world->getObjList()) {
    const auto* copy=restored.world->getObject(object->getName());
    require(copy!=nullptr,"export lost an object");
    Vec<3> a,b; object->getPosition(0,a); copy->getPosition(0,b);
    require((a.e()-b.e()).norm()<1e-6,"export changed an object/base position");
  }
  for (const auto& tendon:original.world->getTendons()) {
    const auto* copy=restored.world->getTendon(tendon->getName());
    require(copy!=nullptr,"export lost a tendon");
    require(std::abs(copy->getLength()-tendon->getLength())<1e-6,"export changed a tendon trajectory");
  }
}
}
int main(int argc, char** argv) {
  try {
    const auto options=parseOptions(argc,argv,Kind::All);
    if (!options.key.empty()) World::setActivationKey(options.key);
    const auto stamp=std::chrono::steady_clock::now().time_since_epoch().count();
    const auto directory=std::filesystem::temp_directory_path()/
      ("raisim-tendon-roundtrip-"+std::to_string(stamp));
    require(std::filesystem::create_directory(directory),"cannot create export test directory");
    struct Cleanup {
      std::filesystem::path directory;
      ~Cleanup() { std::error_code error; std::filesystem::remove_all(directory,error); }
    } cleanup{directory};
    for (auto kind:{Kind::Elastic,Kind::Pulley,Kind::Coupling,Kind::All}) {
      Scene original(kind),restored(kind);
      const auto path=directory/kindName(kind)/"tendon world.xml";
      exportScene(original,path.string());
      restored.world=std::make_shared<World>(path.string());
      require(original.world->getObjList().size()==restored.world->getObjList().size(),"object count changed");
      require(original.world->getTendonCouplings().size()==restored.world->getTendonCouplings().size(),
              "coupling count changed");
      compare(original,restored);
      for (int step=0;step<1200;++step) { original.step(); restored.step(); }
      original.verify(1200); restored.verify(1200); compare(original,restored);
    }
    std::cout << "All four exported scenes preserve placement and controlled trajectories.\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
