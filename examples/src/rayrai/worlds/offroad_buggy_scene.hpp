#pragma once
// Terrain and scenery of rayrai_offroad_buggy: the eroded height map with its
// race circuit (rsc/offroad_buggy/terrain, written by
// examples/tools/generate_offroad_terrain.py), and trees, boulders, fallen
// trunks, a rock garden on the track and grass placed on it. Trees, boulders
// and stones get static colliders from their .rasset files; grass is visual
// only. The race venue around the track is in offroad_buggy_venue.hpp.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "raisim/Rasset.hpp"
#include "raisim/World.hpp"
#include "raisim/object/terrain/HeightMap.hpp"
#include "stb/stb_image.h"

namespace offroad {

struct TrackFeature {
  double start = 0, length = 0;
  std::string name;
};

/// An axis-aligned rectangle of the ground [m].
struct Area {
  double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
  bool contains(double x, double y, double margin = 0) const {
    return x > x0 - margin && x < x1 + margin && y > y0 - margin && y < y1 + margin;
  }
};

struct TerrainInfo {
  double extent = 0, heightMin = 0, heightMax = 0, halfWidth = 0, length = 0;
  int samples = 0;
  Eigen::Vector3d start = Eigen::Vector3d::Zero();  // x, y, heading
  double startLine = 0;                             // station of the start/finish line
  double straightStart = 0, straightEnd = 0, straightHeight = 0;
  Area pit, spectators;                             // level pads beside the straight
  std::vector<TrackFeature> features;
  std::vector<Eigen::Vector3d> centre;  // every 2 m: x, y, track height
  std::vector<double> curvature;        // signed (positive to the left) [1/m], at the centre points
  double centreStep = 2.0;
};

inline TerrainInfo readTerrainInfo(const std::filesystem::path& file) {
  std::ifstream in(file);
  if (!in) throw std::runtime_error("Cannot read " + file.string());
  TerrainInfo info;
  std::string line, key;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream s(line);
    s >> key;
    if (key == "extent") s >> info.extent;
    else if (key == "samples") s >> info.samples;
    else if (key == "height") s >> info.heightMin >> info.heightMax;
    else if (key == "track") s >> info.halfWidth >> info.length;
    else if (key == "start") s >> info.start.x() >> info.start.y() >> info.start.z();
    else if (key == "start_line") s >> info.startLine;
    else if (key == "straight") s >> info.straightStart >> info.straightEnd >> info.straightHeight;
    else if (key == "pit") s >> info.pit.x0 >> info.pit.y0 >> info.pit.x1 >> info.pit.y1;
    else if (key == "spectators") s >> info.spectators.x0 >> info.spectators.y0 >> info.spectators.x1 >> info.spectators.y1;
    else if (key == "feature") {
      TrackFeature f;
      s >> f.start >> f.length;
      std::getline(s >> std::ws, f.name);
      info.features.push_back(f);
    } else if (key == "centre") {
      Eigen::Vector3d p;
      double k = 0;
      s >> p.x() >> p.y() >> p.z() >> k;
      info.centre.push_back(p);
      info.curvature.push_back(k);
    }
  }
  if (info.samples < 2 || info.centre.empty()) throw std::runtime_error("Incomplete " + file.string());
  return info;
}

/// A single-channel image covering the terrain (row 0 at the lowest y), sampled bilinearly.
struct GroundMap {
  int width = 0, height = 0;
  std::vector<unsigned char> pixels;
  double extent = 1;

  double at(double x, double y) const {
    const double u = std::clamp((x / extent + 0.5) * (width - 1), 0.0, width - 1.001);
    const double v = std::clamp((y / extent + 0.5) * (height - 1), 0.0, height - 1.001);
    const int i = int(u), j = int(v);
    const double fu = u - i, fv = v - j;
    auto p = [&](int a, int b) { return pixels[size_t(b) * width + a] / 255.0; };
    return (p(i, j) * (1 - fu) + p(i + 1, j) * fu) * (1 - fv) + (p(i, j + 1) * (1 - fu) + p(i + 1, j + 1) * fu) * fv;
  }
};

/// The surface layers of the terrain (generate_offroad_terrain.py): bare ground (else grass),
/// rock, loose dirt (else the packed racing surface) and gravel, and the wear of the racing line.
struct Surface {
  GroundMap bare, rock, loose, gravel, wear;
};

inline std::vector<GroundMap> readGroundMaps(const std::filesystem::path& file, double extent, int channels) {
  int w = 0, h = 0, stored = 0;
  unsigned char* data = stbi_load(file.string().c_str(), &w, &h, &stored, channels);
  if (!data) throw std::runtime_error("Cannot read " + file.string());
  std::vector<GroundMap> maps(channels);
  for (int c = 0; c < channels; ++c) {
    maps[c].width = w;
    maps[c].height = h;
    maps[c].extent = extent;
    maps[c].pixels.resize(size_t(w) * h);
    for (size_t i = 0; i < maps[c].pixels.size(); ++i) maps[c].pixels[i] = data[i * channels + c];
  }
  stbi_image_free(data);
  return maps;
}

inline Surface readSurface(const std::filesystem::path& dir, double extent) {
  auto layers = readGroundMaps(dir / "surface.png", extent, 4);
  Surface surface{layers[0], layers[1], layers[2], layers[3], readGroundMaps(dir / "wear.png", extent, 1)[0]};
  return surface;
}

/// Heights of the terrain grid (row-major, rows along y from its lowest value), in metres.
inline std::vector<double> readHeights(const std::filesystem::path& dir, const TerrainInfo& info) {
  int w = 0, h = 0, channels = 0;
  const auto file = dir / "height.png";
  unsigned short* data = stbi_load_16(file.string().c_str(), &w, &h, &channels, 1);
  if (!data || w != info.samples || h != info.samples) throw std::runtime_error("Cannot read " + file.string());
  std::vector<double> heights(size_t(w) * h);
  const double scale = (info.heightMax - info.heightMin) / 65535.0;
  for (size_t i = 0; i < heights.size(); ++i) heights[i] = info.heightMin + data[i] * scale;
  stbi_image_free(data);
  return heights;
}

inline raisim::HeightMap* addTerrain(raisim::World& world, const TerrainInfo& info, const std::vector<double>& heights) {
  auto* terrain = world.addHeightMap(info.samples, info.samples, info.extent, info.extent, 0, 0, heights, "terrain");
  terrain->setName("terrain");
  return terrain;
}

/// Meadow tint per height-map sample (linear RGB around 1): patches of lush and dry grass that
/// break up the texture tiling, and darker soil in the hollows.
inline std::vector<Eigen::Vector3f> terrainColours(const raisim::HeightMap& terrain, const TerrainInfo& info,
                                                   const Surface& surface) {
  const int n = info.samples;
  std::vector<Eigen::Vector3f> colours(size_t(n) * n);
  auto wave = [](double x, double y, double k, double phase) {
    return std::sin(x * k + phase) * std::cos(y * k * 0.83 - phase * 1.7);
  };
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const double x = -info.extent / 2 + info.extent * i / (n - 1);
      const double y = -info.extent / 2 + info.extent * j / (n - 1);
      const double patch = 0.5 * wave(x, y, 0.045, 0.3) + 0.3 * wave(x + 13, y - 7, 0.11, 1.9) +
                           0.2 * wave(x - 5, y + 21, 0.27, 4.1);
      const double hollow = std::clamp((terrain.getHeight(x + 2, y) + terrain.getHeight(x - 2, y) +
                                        terrain.getHeight(x, y + 2) + terrain.getHeight(x, y - 2)) / 4 -
                                       terrain.getHeight(x, y), -0.3, 0.3);
      const double d = surface.bare.at(x, y);
      // Lush patches are greener and darker, dry ones yellower and brighter.
      const double dry = std::clamp(0.5 + 0.6 * patch, 0.0, 1.0) * (1 - d);
      const double shade = 1.0 - 0.8 * std::max(0.0, hollow);
      colours[size_t(j) * n + i] = Eigen::Vector3f(float((0.72 + 0.34 * dry) * shade), float((0.86 + 0.14 * dry) * shade),
                                                   float((0.66 + 0.10 * dry) * shade));
    }
  return colours;
}

/// Uniform-grid broad phase for placement tests.
class PlacementGrid {
 public:
  explicit PlacementGrid(double cell) : cell_(cell) {}
  void add(double x, double y, double radius) {
    cells_[key(x, y)].push_back({x, y, radius});
    maxRadius_ = std::max(maxRadius_, radius);
  }
  bool clear(double x, double y, double radius) const {
    const double reach = radius + maxRadius_;
    const int r = int(std::ceil(reach / cell_));
    const auto cx = long(std::floor(x / cell_)), cy = long(std::floor(y / cell_));
    for (long i = cx - r; i <= cx + r; ++i)
      for (long j = cy - r; j <= cy + r; ++j) {
        auto it = cells_.find(pack(i, j));
        if (it == cells_.end()) continue;
        for (const auto& p : it->second)
          if (std::hypot(p[0] - x, p[1] - y) < radius + p[2]) return false;
      }
    return true;
  }

 private:
  static long long pack(long i, long j) { return (long long)(i) * 1000003LL + j; }
  long long key(double x, double y) const { return pack(long(std::floor(x / cell_)), long(std::floor(y / cell_))); }
  double cell_, maxRadius_ = 0;
  std::unordered_map<long long, std::vector<std::array<double, 3>>> cells_;
};

/// Distance to the track centre line and the station of the closest point.
class TrackLocator {
 public:
  explicit TrackLocator(const TerrainInfo& info) : info_(info) {
    for (size_t i = 0; i < info.centre.size(); ++i) bins_[key(info.centre[i].x(), info.centre[i].y())].push_back(i);
  }
  /// @return {distance [m], station [m]}; distance is large far from the track.
  std::array<double, 2> locate(double x, double y) const {
    double best = 1e9, station = 0;
    const auto cx = long(std::floor(x / 8.0)), cy = long(std::floor(y / 8.0));
    for (long i = cx - 2; i <= cx + 2; ++i)
      for (long j = cy - 2; j <= cy + 2; ++j) {
        auto it = bins_.find((long long)(i) * 1000003LL + j);
        if (it == bins_.end()) continue;
        for (size_t k : it->second) {
          // Distance to the segment from point k to point k + 1.
          const Eigen::Vector2d a = info_.centre[k].head<2>(), b = info_.centre[(k + 1) % info_.centre.size()].head<2>();
          const Eigen::Vector2d p(x, y), ab = b - a;
          const double t = std::clamp((p - a).dot(ab) / ab.squaredNorm(), 0.0, 1.0);
          const double d = (a + t * ab - p).norm();
          if (d < best) {
            best = d;
            station = (k + t) * info_.centreStep;
          }
        }
      }
    return {best, station};
  }
  /// Point and unit direction of the centre line at a station.
  void at(double station, Eigen::Vector3d& point, Eigen::Vector2d& direction) const {
    const size_t n = info_.centre.size();
    double s = std::fmod(station, n * info_.centreStep);
    if (s < 0) s += n * info_.centreStep;
    const size_t k = size_t(s / info_.centreStep) % n;
    const double t = s / info_.centreStep - std::floor(s / info_.centreStep);
    const auto& a = info_.centre[k];
    const auto& b = info_.centre[(k + 1) % n];
    point = a + t * (b - a);
    direction = (b - a).head<2>().normalized();
  }
  /// Signed curvature (positive to the left) at a station [1/m].
  double curvature(double station) const {
    const size_t n = info_.centre.size();
    double s = std::fmod(station, n * info_.centreStep);
    if (s < 0) s += n * info_.centreStep;
    const size_t k = size_t(s / info_.centreStep) % n;
    const double t = s / info_.centreStep - std::floor(s / info_.centreStep);
    return (1 - t) * info_.curvature[k] + t * info_.curvature[(k + 1) % n];
  }
  /// Ground point at a station and a lateral offset (positive to the left), and the heading there.
  Eigen::Vector2d offset(double station, double lateral, double* heading = nullptr) const {
    Eigen::Vector3d p;
    Eigen::Vector2d d;
    at(station, p, d);
    if (heading) *heading = std::atan2(d.y(), d.x());
    return p.head<2>() + lateral * Eigen::Vector2d(-d.y(), d.x());
  }

 private:
  long long key(double x, double y) const {
    return (long long)(std::floor(x / 8.0)) * 1000003LL + (long long)std::floor(y / 8.0);
  }
  const TerrainInfo& info_;
  std::unordered_map<long long, std::vector<size_t>> bins_;
};

/// One placed instance of a scenery asset.
struct Placement {
  double x, y, z, yaw, scale;
  int type;
};

struct Scenery {
  std::vector<Placement> trees, rocks, stones, logs, grass;
  size_t colliders = 0;
};

// Asset folders in rsc/forest and rsc/offroad_buggy/props.
inline const std::array<const char*, 3> kTreeNames = {"pine_sapling_small", "fir_sapling", "tree_small_02"};
inline const std::array<const char*, 5> kGrassNames = {"grass_medium_01", "grass_medium_02", "dandelion_01",
                                                       "periwinkle_plant", "fern_02"};
constexpr int kMossRocks = 6;
/// Rock types: the six mossy rocks of rsc/forest (normalised to a horizontal radius of 1 m), then the
/// sun-baked boulders of the venue.
inline const std::array<const char*, 2> kBoulderNames = {"boulder_01", "namaqualand_boulder_02"};
constexpr int kRockTypes = kMossRocks + 2;
/// Horizontal radius of each rock type at scale 1 [m].
constexpr std::array<double, kRockTypes> kRockRadius = {1, 1, 1, 1, 1, 1, 0.95, 1.3};
/// Small stones: the rock garden and the verges (rsc/offroad_buggy/props).
inline const std::array<const char*, 2> kStoneNames = {"rock_07", "rock_09"};

inline std::filesystem::path rockFolder(const std::filesystem::path& rsc, int type) {
  return type < kMossRocks ? rsc / "forest" / ("rock_moss_" + std::to_string(type))
                           : rsc / "offroad_buggy" / "props" / kBoulderNames[type - kMossRocks];
}

/// Place trees, boulders, fallen trunks, the rock garden, verge stones and grass around the circuit,
/// away from the venue already in occupied, and add the static colliders of the trees, boulders,
/// trunks and rock-garden stones. Deterministic for a fixed seed.
inline Scenery placeScenery(raisim::World& world, const raisim::HeightMap& terrain, const TerrainInfo& info,
                            const Surface& surface, const std::filesystem::path& rsc, double grassDensity,
                            PlacementGrid& occupied) {
  std::mt19937 rng(7);
  auto unit = [&] { return double(rng()) / 4294967296.0; };
  const TrackLocator track(info);
  const double half = info.extent / 2 - 3;
  auto slope = [&](double x, double y) {
    return std::hypot(terrain.getHeight(x + 0.5, y) - terrain.getHeight(x - 0.5, y),
                      terrain.getHeight(x, y + 0.5) - terrain.getHeight(x, y - 0.5));
  };
  auto onPad = [&](double x, double y, double margin) {
    return info.pit.contains(x, y, margin) || info.spectators.contains(x, y, margin);
  };
  Scenery scenery;
  const auto forest = rsc / "forest", props = rsc / "offroad_buggy" / "props";

  // Trees: groves in the hills (clustered by a smooth density field) and loners in the meadow.
  std::array<raisim::Rasset, 3> treeAssets;
  for (int t = 0; t < 3; ++t) treeAssets[t] = raisim::Rasset::load(forest / kTreeNames[t] / "model.rasset");
  auto grove = [](double x, double y) {
    return 0.5 + 0.5 * std::sin(x * 0.035 + 1.3) * std::cos(y * 0.041 - 0.4) + 0.25 * std::sin(x * 0.09 - y * 0.07);
  };
  for (int attempts = 0; scenery.trees.size() < 480 && attempts < 60000; ++attempts) {
    const double x = (unit() * 2 - 1) * half, y = (unit() * 2 - 1) * half;
    if (track.locate(x, y)[0] < info.halfWidth + 7.0 || onPad(x, y, 6.0)) continue;
    if (unit() > std::clamp(grove(x, y), 0.04, 1.0)) continue;
    if (slope(x, y) > 0.7) continue;
    // Mostly broadleaf trees, with young pines and firs in between.
    const double kind = unit();
    const int type = kind < 0.6 ? 2 : kind < 0.85 ? 1 : 0;
    const double scale = (type == 2 ? 1.7 : 3.4) * (0.75 + 0.5 * unit());
    if (!occupied.clear(x, y, type == 2 ? 1.6 : 1.1)) continue;
    occupied.add(x, y, type == 2 ? 1.6 : 1.1);
    scenery.trees.push_back({x, y, terrain.getHeight(x, y) - 0.05, unit() * 2 * M_PI, scale, type});
  }
  for (const auto& p : scenery.trees)
    scenery.colliders += treeAssets[p.type].addStaticColliders(world, {p.x, p.y, p.z}, p.yaw, p.scale, "", "wood").size();

  // Boulders: mossy ones in the meadow, sun-baked ones mostly near the track.
  rng.seed(23);
  std::array<raisim::Rasset, kRockTypes> rockAssets;
  for (int t = 0; t < kRockTypes; ++t) rockAssets[t] = raisim::Rasset::load(rockFolder(rsc, t) / "model.rasset");
  auto lowest = [&](double x, double y, double radius) {
    double low = terrain.getHeight(x, y);
    for (int i = 0; i < 8; ++i) {
      const double a = i * M_PI / 4;
      low = std::min(low, terrain.getHeight(x + radius * std::cos(a), y + radius * std::sin(a)));
    }
    return low;
  };
  for (int attempts = 0; scenery.rocks.size() < 240 && attempts < 40000; ++attempts) {
    const double x = (unit() * 2 - 1) * half, y = (unit() * 2 - 1) * half;
    const double distance = track.locate(x, y)[0];
    if (onPad(x, y, 3.0)) continue;
    const double r = unit();
    const int type = distance < 30 ? (r < 0.55 ? kMossRocks : r < 0.8 ? kMossRocks + 1 : int(rng() % kMossRocks))
                                   : (r < 0.8 ? int(rng() % kMossRocks) : kMossRocks + int(rng() % 2));
    const double scale = type < kMossRocks ? 0.3 + 1.5 * unit() * unit() : 0.55 + 0.75 * unit();
    const double radius = kRockRadius[type] * scale;
    if (distance < info.halfWidth + 3.0 + radius) continue;
    if (!occupied.clear(x, y, radius + 0.2)) continue;
    occupied.add(x, y, radius);
    scenery.rocks.push_back({x, y, lowest(x, y, 0.6 * radius) - 0.12 * radius, unit() * 2 * M_PI, scale, type});
  }
  for (const auto& p : scenery.rocks)
    scenery.colliders += rockAssets[p.type].addStaticColliders(world, {p.x, p.y, p.z}, p.yaw, p.scale, "", "rock").size();

  // Fallen trunks at the edges of the groves.
  const auto logAsset = raisim::Rasset::load(props / "dead_tree_trunk_02" / "model.rasset");
  for (int attempts = 0; scenery.logs.size() < 10 && attempts < 20000; ++attempts) {
    const double x = (unit() * 2 - 1) * half, y = (unit() * 2 - 1) * half;
    if (track.locate(x, y)[0] < info.halfWidth + 8.0 || onPad(x, y, 6.0)) continue;
    if (grove(x, y) < 0.55 || slope(x, y) > 0.35 || !occupied.clear(x, y, 2.4)) continue;
    occupied.add(x, y, 2.2);
    const double scale = 0.8 + 0.4 * unit();
    scenery.logs.push_back({x, y, lowest(x, y, 1.6 * scale) - 0.08, unit() * 2 * M_PI, scale, 0});
  }
  for (const auto& p : scenery.logs)
    scenery.colliders += logAsset.addStaticColliders(world, {p.x, p.y, p.z}, p.yaw, p.scale, "", "wood").size();

  // Rock garden: stones 12 to 20 cm high scattered over the track, which the car drives over at
  // walking pace. Its own random sequence keeps it fixed when the other scenery changes.
  rng.seed(11);
  const auto stoneAsset = raisim::Rasset::load(props / kStoneNames[0] / "model.rasset");
  for (const auto& f : info.features)
    if (f.name == "rock garden")
      for (double s = f.start + 3; s < f.start + f.length - 2; s += 1.0 + 1.0 * unit()) {
        const double lateral = (unit() * 2 - 1) * (info.halfWidth - 0.7);
        const Eigen::Vector2d p = track.offset(s, lateral);
        const double scale = 0.85 + 0.55 * unit();
        if (!occupied.clear(p.x(), p.y(), 0.2 * scale + 0.25)) continue;
        occupied.add(p.x(), p.y(), 0.2 * scale);
        scenery.stones.push_back({p.x(), p.y(), lowest(p.x(), p.y(), 0.1 * scale) - 0.035 * scale, unit() * 2 * M_PI,
                                  scale, 0});
      }
  for (const auto& p : scenery.stones)
    scenery.colliders += stoneAsset.addStaticColliders(world, {p.x, p.y, p.z}, p.yaw, p.scale, "", "rock").size();
  // Loose stones on the verges and berms, too small to matter to the car (visual only).
  rng.seed(29);
  for (int i = 0; i < 360; ++i) {
    const double s = unit() * info.length;
    const double side = unit() < 0.5 ? -1 : 1;
    const Eigen::Vector2d p = track.offset(s, side * (info.halfWidth + 0.2 + 3.0 * unit() * unit()));
    if (onPad(p.x(), p.y(), 1.0)) continue;
    const int type = unit() < 0.5 ? 0 : 1;
    const double scale = type == 0 ? 0.5 + 0.6 * unit() : 0.8 + 1.4 * unit();
    scenery.stones.push_back({p.x(), p.y(), terrain.getHeight(p.x(), p.y()) - 0.01, unit() * 2 * M_PI, scale, type});
  }

  // Grass and flowers: dense in the meadow, thinning out on bare ground, none on the track.
  rng.seed(37);
  const int count = int(170000 * grassDensity);
  for (int attempts = 0; int(scenery.grass.size()) < count && attempts < count * 4; ++attempts) {
    const double x = (unit() * 2 - 1) * half, y = (unit() * 2 - 1) * half;
    const double d = surface.bare.at(x, y);
    if (unit() < d * 1.6 || info.pit.contains(x, y, 1.0)) continue;
    if (track.locate(x, y)[0] < info.halfWidth + 0.3) continue;
    const double r = unit();
    const int type = r < 0.48 ? 0 : r < 0.84 ? 1 : r < 0.9 ? 2 : r < 0.94 ? 3 : 4;
    // Scale factors from the sizes of the Poly Haven models (tufts of 0.15 to 0.4 m).
    const double base = 1.1 * (type == 0 ? 2.4 : type == 1 ? 1.1 : type == 2 ? 1.3 : 1.0);
    scenery.grass.push_back({x, y, terrain.getHeight(x, y) - 0.02, unit() * 2 * M_PI, base * (0.7 + 0.6 * unit()), type});
  }
  return scenery;
}

}  // namespace offroad
