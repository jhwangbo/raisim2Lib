// Copyright (c) 2026 Raion Robotics Inc. All rights reserved.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <glm/glm.hpp>
#include "raisim/World.hpp"

namespace offroad {

inline glm::vec3 springColour(double compression) {
  static const std::array<std::pair<double, glm::vec3>, 5> stops = {
      {{0.00, {0.10f, 0.35f, 1.00f}}, {0.38, {0.05f, 0.85f, 0.30f}}, {0.62, {1.00f, 0.86f, 0.05f}}, {0.80, {1.00f, 0.42f, 0.02f}}, {1.00, {0.95f, 0.02f, 0.02f}}}};
  compression = std::clamp(compression, 0.0, 1.0);
  for (size_t i = 1; i < stops.size(); ++i)
    if (compression <= stops[i].first) {
      const float t = float((compression - stops[i - 1].first) / (stops[i].first - stops[i - 1].first));
      return glm::mix(stops[i - 1].second, stops[i].second, t);
    }
  return stops.back().second;
}

inline glm::vec4 toGlmQuat(const Eigen::Matrix3d& rotation) {
  const Eigen::Quaterniond q(rotation);
  return {float(q.w()), float(q.x()), float(q.y()), float(q.z())};
}

/// Animate the visuals already defined in buggy.urdf; no extra scene objects are created.
class SuspensionVisuals {
 public:
  explicit SuspensionVisuals(raisim::ArticulatedSystem& car) : car_(car) {
    constexpr const char* names[] = {"fl", "fr", "rl", "rr"};
    for (size_t i = 0; i < springs_.size(); ++i) {
      const std::string name = names[i];
      const size_t body = car.getBodyIdx(name + "_shock_body");
      coordinates_[i] = car.getJoint(name + "_shock").getIdxInGeneralizedCoordinate();
      const auto& limits = car.getJointLimits()[car.getGeneralizedVelocityIndex(name + "_shock")];
      limits_[i] = {limits[0], limits[1]};
      for (auto& visual : car.getVisOb())
        if (visual.localIdx == body && std::filesystem::path(visual.fileName).filename() == "coil_spring.gltf") springs_[i] = &visual;
      if (!springs_[i]) throw std::runtime_error("Missing URDF coil spring on " + name);
      if (i >= 2) {
        const size_t side = i - 2;
        wheels_[side] = car.getBodyIdx(name + "_wheel");
        for (auto& visual : car.getVisOb()) {
          if (visual.material == name + "_shaft") shafts_[side] = &visual;
          if (visual.material == name + "_inner_boot") boots_[side][0] = &visual;
          if (visual.material == name + "_outer_boot") boots_[side][1] = &visual;
        }
        if (!shafts_[side] || !boots_[side][0] || !boots_[side][1]) throw std::runtime_error("Missing URDF drive shaft on " + name);
      }
    }
    update();
  }

  void update() {
    const auto& q = car_.getGeneralizedCoordinate();
    constexpr double upperSeat = -0.1028, lowerSeat = -0.3357;
    for (size_t i = 0; i < springs_.size(); ++i) {
      const double scale = (upperSeat - lowerSeat + q[coordinates_[i]]) / (upperSeat - lowerSeat);
      auto& spring = *springs_[i];
      spring.scale[2] = spring.visShapeParam[2] = scale;
      spring.offset[2] = upperSeat * (1.0 - scale);
      const auto colour = springColour((limits_[i][1] - q[coordinates_[i]]) / (limits_[i][1] - limits_[i][0]));
      spring.color << colour.r, colour.g, colour.b, 1.0;
    }
    raisim::Mat<3, 3> chassisRotation;
    raisim::Vec<3> chassisPosition;
    car_.getBodyPose(0, chassisRotation, chassisPosition);
    for (size_t i = 0; i < shafts_.size(); ++i) {
      raisim::Mat<3, 3> rotation;
      raisim::Vec<3> position;
      car_.getBodyPose(wheels_[i], rotation, position);
      const double side = i == 0 ? 1.0 : -1.0;
      const Eigen::Vector3d inner(-1.3571, 0.30 * side, 0.0);
      const Eigen::Vector3d outer = chassisRotation.e().transpose() * (position.e() - chassisPosition.e()) -
                                    chassisRotation.e().transpose() * rotation.e().col(1) * (0.115 * side);
      const Eigen::Vector3d axis = outer - inner;
      const double length = axis.norm();
      const Eigen::Vector3d direction = axis / length;
      const Eigen::Matrix3d orientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), direction).toRotationMatrix();
      shafts_[i]->offset = (0.5 * (inner + outer)).eval();
      shafts_[i]->rot = orientation;
      shafts_[i]->scale[2] = shafts_[i]->visShapeParam[2] = length - 0.1;
      for (size_t end = 0; end < 2; ++end) {
        const Eigen::Vector3d centre = end == 0 ? Eigen::Vector3d(inner + direction * 0.045) : Eigen::Vector3d(outer - direction * 0.045);
        boots_[i][end]->offset = centre;
        boots_[i][end]->rot = orientation;
      }
    }
  }

 private:
  raisim::ArticulatedSystem& car_;
  std::array<raisim::VisObject*, 4> springs_{};
  std::array<size_t, 4> coordinates_{};
  std::array<std::array<double, 2>, 4> limits_{};
  std::array<raisim::VisObject*, 2> shafts_{};
  std::array<std::array<raisim::VisObject*, 2>, 2> boots_{};
  std::array<size_t, 2> wheels_{};
};

}  // namespace offroad
