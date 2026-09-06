// Cylinder wrapping and sphere wrapping with a 2:1 pulley branch. Each pair of
// loads is connected by a length-limited cable and driven by a fixed-tendon servo.
// See include/tendon_scenes.hpp::addPulleyStation for the routing and controls.
#include "tendon_example_runner.hpp"
int main(int argc,char** argv) {
  return raisim_examples::tendons::runServerExample(argc,argv,raisim_examples::tendons::Kind::Pulley);
}
