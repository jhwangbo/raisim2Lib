// Copyright (c) 2026 Raion Robotics Inc. All rights reserved.
#pragma once
#include "rayrai/OpenGLMesh.hpp"
#include <cmath>
#include <vector>

namespace raisin::glass_example {
struct Geometry {
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  std::shared_ptr<OpenGLMesh> cpuMesh() const {
    return OpenGLMesh::createCpuGeometry(vertices,indices,{},glm::vec4(1),false);
  }
  std::shared_ptr<std::vector<std::shared_ptr<OpenGLMesh>>> meshes() const {
    auto result=std::make_shared<std::vector<std::shared_ptr<OpenGLMesh>>>();
    result->push_back(std::make_shared<OpenGLMesh>(vertices,indices)); return result;
  }
  void quad(const std::array<glm::vec3,4>& p,const std::array<glm::vec3,4>& n) {
    const auto base=static_cast<unsigned int>(vertices.size());
    for(int i=0;i<4;++i) { Vertex v; v.Position=p[i]; v.Normal=n[i]; vertices.push_back(v); }
    for(auto i:{0u,1u,2u,0u,2u,3u}) indices.push_back(base+i);
  }
};
inline Geometry box(glm::vec3 lo=glm::vec3(-.5f),glm::vec3 hi=glm::vec3(.5f),bool reverse=false) {
  Geometry g;
  for(int axis=0;axis<3;++axis) for(int sign:{-1,1}) {
    glm::vec3 n(0),u(0); n[axis]=float(sign); u[(axis+1)%3]=1;
    const auto v=glm::cross(n,u); const auto center=(lo+hi)*.5f,extent=(hi-lo)*.5f;
    const auto c=center+n*extent;
    std::array<glm::vec3,4> p{c-u*extent-v*extent,c+u*extent-v*extent,c+u*extent+v*extent,c-u*extent+v*extent};
    if(reverse) { std::swap(p[1],p[3]); n=-n; }
    g.quad(p,{n,n,n,n});
  }
  return g;
}
inline Geometry lathe(const std::vector<glm::vec2>& profile,int segments=64) {
  Geometry g;
  for(size_t j=0;j<profile.size();++j) {
    const auto a=profile[j],b=profile[(j+1)%profile.size()];
    if(a.x==0 && b.x==0) continue;
    const auto n=glm::normalize(glm::vec2(b.y-a.y,a.x-b.x));
    for(int i=0;i<segments;++i) {
      const float u=6.28318530718f*i/segments,w=6.28318530718f*((i+1)%segments)/segments;
      const auto p=[&](glm::vec2 rz,float angle) { return glm::vec3(rz.x*std::cos(angle),rz.x*std::sin(angle),rz.y); };
      const auto normal=[&](float angle) { return glm::vec3(n.x*std::cos(angle),n.x*std::sin(angle),n.y); };
      g.quad({p(a,u),p(a,w),p(b,w),p(b,u)},{normal(u),normal(w),normal(w),normal(u)});
    }
  }
  return g;
}
inline Geometry cup(float radius=.8f,float height=1.8f,float wall=.1f) {
  return lathe({{0,0},{radius,0},{radius,height},{radius-wall,height},{radius-wall,wall},{0,wall}});
}
inline Geometry torus(float major=.8f,float minor=.25f,int rings=64,int sides=24) {
  Geometry g;
  const auto normal=[](float u,float v) { return glm::vec3(std::cos(u)*std::cos(v),std::sin(u)*std::cos(v),std::sin(v)); };
  const auto point=[&](float u,float v) { return glm::vec3(major*std::cos(u),major*std::sin(u),0)+minor*normal(u,v); };
  for(int i=0;i<rings;++i) for(int j=0;j<sides;++j) {
    const float u=6.28318530718f*i/rings,w=6.28318530718f*((i+1)%rings)/rings;
    const float v=6.28318530718f*j/sides,x=6.28318530718f*((j+1)%sides)/sides;
    g.quad({point(u,v),point(w,v),point(w,x),point(u,x)},
           {normal(u,v),normal(w,v),normal(w,x),normal(u,x)});
  }
  return g;
}
} // namespace raisin::glass_example
