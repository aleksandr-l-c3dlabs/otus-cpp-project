#pragma once

#include <eigen3/Eigen/Core>

#include "ray.h"

namespace rtr {

struct AABB {
  Eigen::Vector3f min;
  Eigen::Vector3f max;

  AABB()
      : min(Eigen::Vector3f::Constant(std::numeric_limits<float>::max())),
        max(Eigen::Vector3f::Constant(-std::numeric_limits<float>::max())) {}
  AABB(const Eigen::Vector3f& min_, const Eigen::Vector3f& max_)
      : min(min_), max(max_) {}

  void expand(const AABB& other) {
    min = min.cwiseMin(other.min);
    max = max.cwiseMax(other.max);
  }

  bool intersect(const Ray& ray, float t_min, float t_max) const;

  void clear() {
    min = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    max = Eigen::Vector3f::Constant(-std::numeric_limits<float>::max());
  }
};

}  // namespace rtr