#include <array>
#include <iostream>
#include <stdexcept>
#include "forest_scene.hpp"
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
int main() {
  try {
    raisim::World world;
    auto* terrain = forest::addTerrain(world);
    const auto plants = forest::scatter(*terrain);
    std::array<int,10> counts{};
    std::array<std::array<int,100>,3> grassCells{};
    std::array<int,8> grassTrailBands{};
    std::array<int,4> grassEdges{};
    int grassInClearing=0;
    double low=1e9, high=-1e9;
    // Ensure indexing agrees with the collision surface at off-axis grid points.
    for (int y=0;y<forest::samples;y+=7) for (int x=0;x<forest::samples;x+=11) {
      const double px=-40+x*.5,py=-40+y*.5,h=terrain->getHeight(px,py);
      require(std::abs(h-forest::elevation(px,py))<1e-8,"Terrain grid indexing mismatch");
      low=std::min(low,h);high=std::max(high,h);
    }
    require(high-low>7,"Terrain lacks relief");
    for (const auto& p:plants) {
      ++counts.at(p.type);
      require(std::isfinite(p.z) && p.scale>0,"Invalid plant transform");
      require(std::abs(p.z-terrain->getHeight(p.x,p.y))<1e-10,"Floating plant root");
      require(std::abs(p.x)<=forest::extent/2 && std::abs(p.y)<=forest::extent/2,
              "Plant root outside terrain");
      if (p.type<3) {
        require(std::abs(p.x-forest::trail(p.y))>=3,"Tree blocks corridor");
        require(std::hypot(p.x,p.y+9)>=4,"Tree blocks physics clearing");
      } else if (p.type<6) {
        const double u=(p.x+forest::extent/2)/forest::extent;
        const double v=(p.y+forest::extent/2)/forest::extent;
        ++grassCells[p.type-3].at(int(v*10)*10+int(u*10));
        if (std::abs(p.x-forest::trail(p.y))<1.8) ++grassTrailBands.at(int(v*8));
        if (std::hypot(p.x,p.y+9)<4) ++grassInClearing;
        grassEdges[0]+=p.x < -forest::extent/2+1;
        grassEdges[1]+=p.x > forest::extent/2-1;
        grassEdges[2]+=p.y < -forest::extent/2+1;
        grassEdges[3]+=p.y > forest::extent/2-1;
      }
    }
    require(counts[0]==880 && counts[1]==880 && counts[2]==128,"Missing trees");
    for(int i=3;i<6;++i) require(counts[i]==18000,"Missing dense grass type");
    for(int i=6;i<10;++i) require(counts[i]==1800,"Missing ground-cover type");
    for (const auto& cells:grassCells) for (int count:cells)
      require(count>=50,"Grass does not cover the whole heightmap");
    for (int count:grassTrailBands) require(count>=100,"Bare section of former trail");
    for (int count:grassEdges) require(count>=100,"Grass missing at terrain edge");
    require(grassInClearing>=100,"Grass missing around physics objects");
    for (size_t i=0;i<plants.size() && plants[i].type<3;++i)
      for (size_t j=0;j<i;++j)
        require(std::hypot(plants[i].x-plants[j].x,plants[i].y-plants[j].y)>=.8,
                "Dense forest trunks overlap");
    const auto rocks=forest::scatterRocks(*terrain,plants);
    require(rocks.size()==180,"Missing rocks");
    std::array<int,6> rockCounts{};
    for (size_t i=0;i<rocks.size();++i) {
      const auto& p=rocks[i]; ++rockCounts.at(p.type);
      require(std::isfinite(p.z) && p.scale>=.35 && p.scale<=1.2,"Invalid rock transform");
      require(p.z<terrain->getHeight(p.x,p.y),"Floating rock base");
      require(std::abs(p.x-forest::trail(p.y))>=2.1+1.2*p.scale,"Rock blocks trail");
      require(std::hypot(p.x,p.y+9)>=4+p.scale,"Rock blocks clearing");
      for (const auto& tree:plants) if (tree.type<3)
        require(std::hypot(p.x-tree.x,p.y-tree.y)>=p.scale+.25,"Rock intersects tree trunk");
      for (size_t j=0;j<i;++j)
        require(std::hypot(p.x-rocks[j].x,p.y-rocks[j].y)>=p.scale+rocks[j].scale,"Rocks overlap");
    }
    for (int count:rockCounts) require(count==30,"Missing rock variant");
    forest::addObjects(world,*terrain);
    require(world.getObjList().size()==13,"Missing physics objects");
    for(int i=0;i<1000;++i) world.integrate();
    for(auto* object:world.getObjList()) {
      if (object == terrain) continue;
      raisim::Vec<3> pos; object->getPosition(0,pos);
      require(std::isfinite(pos[2]),"Physics became non-finite");
      require(pos[2]>=terrain->getHeight(pos[0],pos[1])-.7,"Object fell through terrain");
    }
    std::cout << "Validated " << plants.size() << " grounded plants and 2 seconds of physics\n";
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
