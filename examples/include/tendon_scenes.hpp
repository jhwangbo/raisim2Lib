// This file is part of RaiSim. A valid RaiSim license is required.
#pragma once

#include "raisim/World.hpp"
#include "tendon_models.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace raisim_examples::tendons {
using namespace raisim;
using E=Tendon::PathElement;
constexpr double pi=3.14159265358979323846;
inline const Vec<4> blue{0.08,0.46,0.86,1.}, gold{1.,0.58,0.08,1.};

inline void support(World& world, double x, double y, double width) {
  auto* foot=world.addBox(width,1.0,0.12,1.);
  foot->setBodyType(BodyType::STATIC); foot->setPosition(x,y,0.06); foot->setAppearance("0.14,0.18,0.23,1");
  auto* post=world.addBox(0.12,0.12,2.9,1.);
  post->setBodyType(BodyType::STATIC); post->setPosition(x,y+0.35,1.55); post->setAppearance("0.24,0.29,0.35,1");
  auto* beam=world.addBox(width,0.12,0.12,1.);
  beam->setBodyType(BodyType::STATIC); beam->setPosition(x,y+0.35,2.96); beam->setAppearance("0.24,0.29,0.35,1");
}
struct Drive { std::string tendon; double amplitude, phase; };
inline void addPulleyStation(World& world, double x, double y, bool divided,
                      const std::string& name, std::vector<Drive>& drives) {
  support(world,x+0.15,y,2.3);
  auto* robot=world.addArticulatedSystem(pulleyModel(name,divided,x,y),".");
  robot->setName(name);
  const size_t leftBody=robot->getBodyIdx(name+"_left_body"), rightBody=robot->getBodyIdx(name+"_right_body");
  SingleBodyObject* guide=divided ? static_cast<SingleBodyObject*>(world.addSphere(0.48,1.)) :
                                  static_cast<SingleBodyObject*>(world.addCylinder(0.48,0.22,1.));
  guide->setName(name+"_guide"); guide->setPosition(x,y,2.65); guide->setBodyType(BodyType::STATIC);
  guide->setOrientation(std::sqrt(0.5),-std::sqrt(0.5),0.,0.); guide->setAppearance("0.28,0.35,0.44,1");
  auto wrap=divided ? E::sphere({guide},0.49) : E::cylinder({guide},0.49);
  wrap.withSideSite({nullptr,0,{x,y,3.4}});
  std::vector<E> path{E::via({robot,leftBody,{0.,0.,0.22}}),wrap};
  if (divided) {
    path.push_back(E::via({nullptr,0,{x+0.48,y,1.5}}));
    path.push_back(E::pulley(2.));
    path.push_back(E::via({nullptr,0,{x+1.05,y,2.94}}));
  }
  path.push_back(E::via({robot,rightBody,{0.,0.,0.22}}));
  auto* cable=world.addSpatialTendon(name+"_cable",path);
  auto properties=cable->getProperties(); properties.upperLimit=cable->getLength();
  properties.damping=0.3; properties.armature=0.02; properties.width=0.012; properties.color=gold;
  cable->setProperties(properties);
  auto* motor=world.addFixedTendon(name+"_motor",{{robot,name+"_left",1.}});
  Tendon::Drive drive; drive.positionGain=450.; drive.velocityGain=45.; motor->setDrive(drive);
  drives.push_back({motor->getName(),divided ? 0.28 : 0.42,divided ? pi/2. : 0.});
}
inline void addJointStation(World& world, double x, double y, const std::string& name, std::vector<Drive>& drives) {
  support(world,x,y,2.3);
  auto* robot=world.addArticulatedSystem(jointModel(name,x,y),".");
  robot->setName(name);
  Tendon::Properties p; p.armature=0.06; p.damping=0.8;
  auto* transmission=world.addFixedTendon(name+"_transmission",{
      {robot,name+"_shoulder",1.},{robot,name+"_elbow",0.25}},p);
  Tendon::Drive drive; drive.positionGain=160.; drive.velocityGain=18.; transmission->setDrive(drive);
  drives.push_back({transmission->getName(),0.35,0.});

  // Couple actual cable lengths so the mechanism has physical routes to draw.
  // Both routes sit just in front of the links, with visible anchor brackets.
  // Mounts illustrate the fixed joint support without adding pivot contacts.
  auto* anchor=world.addSphere(0.032,1.,"default",0,0);
  anchor->setName(name+"_anchor"); anchor->setBodyType(BodyType::STATIC);
  anchor->setPosition(x+0.65,y-0.16,2.6); anchor->setAppearance("1,0.42,0.08,1");
  auto* bracket=world.addBox(0.65,0.05,0.05,1.,"default",0,0);
  bracket->setBodyType(BodyType::STATIC); bracket->setPosition(x+0.325,y-0.16,2.6);
  bracket->setAppearance("0.36,0.42,0.48,1");
  auto* standOff=world.addBox(0.05,0.51,0.05,1.,"default",0,0);
  standOff->setBodyType(BodyType::STATIC); standOff->setPosition(x,y+0.095,2.6);
  standOff->setAppearance("0.36,0.42,0.48,1");
  const auto upper=robot->getBodyIdx(name+"_upper"), lower=robot->getBodyIdx(name+"_lower");
  Tendon::Properties cable; cable.width=0.018; cable.color={1.,0.42,0.08,1.};
  auto* a=world.addSpatialTendon(name+"_a",{
      E::via({anchor}),E::via({robot,upper,{0.,-0.16,-0.7}})},cable);
  cable.color={0.06,0.9,0.75,1.};
  auto* b=world.addSpatialTendon(name+"_b",{
      E::via({robot,upper,{0.55,-0.16,-1.05}}),E::via({robot,lower,{0.,-0.16,-0.65}})},cable);
  TendonCoupling::Properties coupling; coupling.coefficients={0.,-0.65,0.,0.,0.};
  world.addTendonCoupling(name+"_coupling",b,a,coupling);
}

// Two identical suspended loads show how damping changes an elastic tendon.
// springLower=0 makes the spring pull-only; upperLimit is a separate hard bound.
inline void addElasticStation(World& world, double x) {
  support(world,x,0.,2.3);
  for (int i=0;i<2;++i) {
    const double offset=i==0 ? -0.65 : 0.65;
    const std::string name=i==0 ? "elastic_light_damping" : "elastic_heavy_damping";
    auto* load=world.addSphere(0.17,1.);
    load->setName(name+"_load"); load->setPosition(x+offset+0.2,0.,1.05);
    load->setVelocity(0.,0.25,0.,0.,0.,0.);
    load->setAppearance(i==0 ? "0.08,0.46,0.86,1" : "1,0.58,0.08,1");
    Tendon::Properties p;
    p.springLower=0.; p.springUpper=1.1; p.stiffness=70.;
    p.damping=i==0 ? 0.8 : 7.; p.frictionLoss=0.05; p.armature=0.04;
    p.upperLimit=2.15; p.width=0.012; p.color=i==0 ? blue : gold;
    world.addSpatialTendon(name,{
      E::via({nullptr,0,{x+offset,0.,2.94}}), E::via({load})},p);
  }
}

enum class Kind { Elastic, Pulley, Coupling, All };
inline const char* kindName(Kind kind) {
  switch (kind) {
    case Kind::Elastic: return "elastic";
    case Kind::Pulley: return "pulley";
    case Kind::Coupling: return "coupling";
    case Kind::All: return "all";
  }
  return "all";
}
inline Kind parseKind(const std::string& value) {
  for (auto kind:{Kind::Elastic,Kind::Pulley,Kind::Coupling,Kind::All})
    if (value==kindName(kind)) return kind;
  throw std::invalid_argument("scene must be elastic, pulley, coupling, or all");
}

struct Scene {
  std::shared_ptr<World> world=std::make_shared<World>();
  std::vector<Drive> drives;
  Kind kind;
  double maximumLimitError=0., maximumCouplingError=0., maximumMotion=0.;
  explicit Scene(Kind selected) : kind(selected) {
    world->setTimeStep(0.001);
    world->setContactSolverParam(1.,1.,1.,150,1e-12);
    world->addGround()->setName("floor");
    if (kind==Kind::Elastic || kind==Kind::All) addElasticStation(*world,kind==Kind::All ? -5. : 0.);
    if (kind==Kind::Pulley || kind==Kind::All) {
      addPulleyStation(*world,-1.6,0.,false,"cylinder",drives);
      addPulleyStation(*world,1.4,0.,true,"sphere_2to1",drives);
    }
    if (kind==Kind::Coupling || kind==Kind::All)
      addJointStation(*world,kind==Kind::All ? 4.8 : 0.,0.,"joint",drives);
  }
  void updateControls() {
    const double time=world->getWorldTime();
    for (const auto& motor:drives) {
      // Name lookup also handles a viewer removing an attached object/tendon.
      auto* tendon=world->getTendon(motor.tendon);
      if (!tendon) continue;
      auto command=tendon->getDrive();
      command.targetLength=(1.-std::exp(-2.*time))*motor.amplitude*std::sin(2.*pi*time/4.+motor.phase);
      tendon->setDrive(command);
    }
  }
  void observe() {
    for (const auto& tendon:world->getTendons()) {
      tendon->updateGeometry();
      const auto& p=tendon->getProperties();
      if (!std::isfinite(tendon->getLength()) || !std::isfinite(tendon->getVelocity()) ||
          !std::isfinite(tendon->getForce())) throw std::runtime_error("non-finite tendon state");
      maximumLimitError=std::max({maximumLimitError,p.lowerLimit-tendon->getLength(),tendon->getLength()-p.upperLimit});
      maximumMotion=std::max(maximumMotion,std::abs(tendon->getLength()-tendon->getReferenceLength()));
    }
    for (const auto& coupling:world->getTendonCouplings()) {
      const auto* second=coupling->getSecond();
      const double x=second ? second->getLength()-second->getReferenceLength() : 0.;
      const auto& c=coupling->getProperties().coefficients;
      const double target=((((c[4]*x+c[3])*x+c[2])*x+c[1])*x+c[0]);
      const auto* first=coupling->getFirst();
      maximumCouplingError=std::max(maximumCouplingError,
        std::abs(first->getLength()-first->getReferenceLength()-target));
    }
  }
  void step() { updateControls(); world->integrate(); observe(); }
  void verify(int steps) const {
    if (maximumLimitError>0.01 || maximumCouplingError>0.001)
      throw std::runtime_error("tendon constraint error exceeded the example tolerance");
    if (steps>=1000 && maximumMotion<0.01)
      throw std::runtime_error("the example did not produce tendon-driven motion");
  }
};
}  // namespace raisim_examples::tendons
