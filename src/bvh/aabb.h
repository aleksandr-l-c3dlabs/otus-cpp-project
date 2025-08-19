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

  bool intersect(const Ray& ray, float t_min, float t_max) const {
    for (int i = 0; i < 3; ++i) {
      float inv_dir = 1.0f / ray.direction[i];
      float t0 = (min[i] - ray.origin[i]) * inv_dir;
      float t1 = (max[i] - ray.origin[i]) * inv_dir;
      if (inv_dir < 0.0f)
        std::swap(t0, t1);
      t_min = std::max(t0, t_min);
      t_max = std::min(t1, t_max);
      if (t_max <= t_min)
        return false;
    }
    return true;
  }

  void clear() {
    min = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    max = Eigen::Vector3f::Constant(-std::numeric_limits<float>::max());
  }
};

}  // namespace rtr