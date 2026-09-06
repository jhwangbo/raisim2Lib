// A weighted joint actuator moves a two-link mechanism. Two visible spatial
// cables are coupled by delta(LB) = -0.65 * delta(LA); the joint-angle ratio
// varies with cable geometry. See include/tendon_scenes.hpp::addJointStation.
#include "tendon_example_runner.hpp"
int main(int argc,char** argv) {
  return raisim_examples::tendons::runServerExample(argc,argv,raisim_examples::tendons::Kind::Coupling);
}
