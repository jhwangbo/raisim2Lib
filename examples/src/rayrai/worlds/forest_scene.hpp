#pragma once
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>
#include "raisim/World.hpp"
#include "raisim/object/terrain/HeightMap.hpp"

namespace forest {
constexpr int samples = 161;
constexpr double extent = 80;
inline double trail(double y) { return 3.5 * std::sin(y * .10); }
inline double elevation(double x, double y) {
  const double hills = 3.4 * std::sin(x * .065) * std::cos(y * .055)
    + 1.8 * std::sin(y * .12 + x * .035) + .38 * std::sin(x * .32) * std::cos(y * .27);
  const double gully = 1.8 * std::exp(-std::pow((x - 13 - 4 * std::sin(y * .08)) / 5, 2));
  return hills - gully;
}
inline raisim::HeightMap* addTerrain(raisim::World& world) {
  std::vector<double> heights;
  std::vector<raisim::ColorRGB> colors;
  for (int y = 0; y < samples; ++y) for (int x = 0; x < samples; ++x) {
    const double px = -extent / 2 + extent * x / (samples - 1);
    const double py = -extent / 2 + extent * y / (samples - 1);
    heights.push_back(elevation(px, py));
    colors.push_back({105,119,78});
  }
  auto* terrain = world.addHeightMap(samples, samples, extent, extent, 0, 0, heights);
  terrain->setName("Forest terrain");
  terrain->setColor(colors);
  return terrain;
}
struct Plant { double x, y, z, scale, yaw; int type; };
inline std::vector<Plant> scatter(const raisim::HeightMap& terrain) {
  std::mt19937 rng(42);
  // Explicit conversion keeps the scatter identical across standard libraries.
  auto unit = [&] { return double(rng()) / 4294967296.0; };
  std::vector<Plant> plants;
  for (int type = 0; type < 10; ++type) {
    const bool tree = type < 3;
    const int count = type < 2 ? 880 : type == 2 ? 128 : type < 6 ? 18000 : 1800;
    for (int n = 0, attempts = 0; n < count && attempts < count * 100; ++attempts) {
      // Low vegetation covers the full terrain, including the open tree corridor.
      const double span = tree ? 74 : extent;
      double x = (unit() - .5) * span, y = (unit() - .5) * span;
      if (tree && (std::abs(x - trail(y)) < 3.0 || std::hypot(x, y + 9) < 4)) continue;
      // Let crowns overlap while keeping neighboring trunks separate.
      if (tree && std::any_of(plants.begin(), plants.end(), [&](const Plant& p) {
            return p.type < 3 && std::hypot(x-p.x,y-p.y) < .8;
          })) continue;
      const double z = terrain.getHeight(x, y);
      if (tree) {
        const double slope = std::hypot(terrain.getHeight(x+.25,y)-terrain.getHeight(x-.25,y),
                                       terrain.getHeight(x,y+.25)-terrain.getHeight(x,y-.25)) / .5;
        if (slope > .65) continue;
      }
      const double base = type < 2 ? 4.0 : type == 2 ? 1.5 : type == 3 ? 3 : type == 4 ? 3.5 : 1.0;
      plants.push_back({x,y,z,base*(.8+.4*unit()),unit()*6.283185307179586,type});
      ++n;
    }
  }
  return plants;
}
// Six normalized rock meshes, each contained in a horizontal radius of one metre.
inline std::vector<Plant> scatterRocks(const raisim::HeightMap& terrain,
                                     const std::vector<Plant>& plants) {
  std::mt19937 rng(93);
  auto unit = [&] { return double(rng()) / 4294967296.0; };
  std::vector<Plant> rocks;
  for (int attempts = 0; rocks.size() < 180 && attempts < 18000; ++attempts) {
    const double y = (unit()-.5)*72;
    const double x = attempts%2 ? (unit()-.5)*72
      : trail(y)+(unit()<.5 ? -1 : 1)*(2.8+unit()*2.2);
    const double scale = .35 + .85*unit();
    if (std::abs(x-trail(y)) < 2.1+1.2*scale || std::hypot(x,y+9) < 4+scale) continue;
    if (std::any_of(plants.begin(),plants.end(),[&](const Plant& p) {
          return p.type < 3 && std::hypot(x-p.x,y-p.y) < scale+.25;
        })) continue;
    if (std::any_of(rocks.begin(),rocks.end(),[&](const Plant& p) {
          return std::hypot(x-p.x,y-p.y) < scale+p.scale;
        })) continue;
    // Embed the rounded underside using terrain samples around its contact patch.
    double low = terrain.getHeight(x,y), high = low;
    for (int i=0;i<8;++i) {
      const double a = i*6.283185307179586/8;
      const double z = terrain.getHeight(x+.35*scale*std::cos(a),y+.35*scale*std::sin(a));
      low=std::min(low,z); high=std::max(high,z);
    }
    if (high-low > .35*scale) continue;
    rocks.push_back({x,y,low-.06*scale,scale,unit()*6.283185307179586,int(rocks.size()%6)});
  }
  return rocks;
}
inline void addObjects(raisim::World& world, const raisim::HeightMap& terrain) {
  world.setTimeStep(.002);
  for (int i = 0; i < 6; ++i) {
    const double x = (i % 3 - 1) * 1.15, y = -9 + (i / 3) * 1.4;
    auto* box = world.addBox(.65,.65,.65,3);
    box->setPosition(x,y,terrain.getHeight(x,y)+1.4);
    box->setAppearance(i % 2 ? "0.75,0.22,0.08,1" : "0.15,0.35,0.7,1");
    box->setName("Falling crate " + std::to_string(i));
    auto* ball = world.addSphere(.3,1);
    ball->setPosition(x,y+3,terrain.getHeight(x,y+3)+2.0);
    ball->setAppearance("0.85,0.65,0.12,1");
    ball->setName("Rolling ball " + std::to_string(i));
  }
}
} // namespace forest
