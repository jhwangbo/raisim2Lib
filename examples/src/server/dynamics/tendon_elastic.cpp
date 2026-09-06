// Compare lightly and heavily damped pull-only springs, with armature,
// dry friction, and independent hard length limits. Scene construction is in
// include/tendon_scenes.hpp::addElasticStation.
#include "tendon_example_runner.hpp"
int main(int argc,char** argv) {
  return raisim_examples::tendons::runServerExample(argc,argv,raisim_examples::tendons::Kind::Elastic);
}
