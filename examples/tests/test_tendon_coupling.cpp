#include "tendon_example_runner.hpp"

using namespace raisim_examples::tendons;
int main(int argc, char** argv) {
  try {
    const auto options=parseOptions(argc,argv,Kind::Coupling);
    if (!options.key.empty()) World::setActivationKey(options.key);
    Scene scene(Kind::Coupling);
    auto* a=scene.world->getTendon("joint_a");
    auto* b=scene.world->getTendon("joint_b");
    if (!a || !b || a->getType()!=Tendon::Type::Spatial || b->getType()!=Tendon::Type::Spatial)
      throw std::runtime_error("coupling example must couple two drawable spatial tendons");
    const auto& coupling=scene.world->getTendonCouplings().at(0);
    if (coupling->getFirst()!=b || coupling->getSecond()!=a)
      throw std::runtime_error("visible cables must participate in the coupling");
    double motion=0.,force=0.;
    for (int step=0;step<6000;++step) {
      scene.step();
      motion=std::max(motion,std::abs(a->getLength()-a->getReferenceLength()));
      force=std::max(force,std::abs(coupling->getForce()));
      if (std::abs((b->getLength()-b->getReferenceLength())+
                    0.65*(a->getLength()-a->getReferenceLength()))>0.001)
        throw std::runtime_error("visible cable lengths violated their coupling");
      if (step%200==0) for (auto* tendon:{a,b}) {
        tendon->updateGeometry(true);
        double length=0.;
        for (const auto& segment:tendon->getVisualSegments())
          length+=(segment.end.e()-segment.start.e()).norm();
        if (tendon->getVisualSegments().empty() || std::abs(length-tendon->getLength())>1e-10)
          throw std::runtime_error("coupled cable drawing must follow its physical route");
      }
    }
    scene.verify(6000);
    if (motion<0.05 || force<0.1)
      throw std::runtime_error("visible coupling must transmit forces and produce cable motion");
    std::cout << "Visible spatial coupling verified: motion=" << motion << " force=" << force << '\n';
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
