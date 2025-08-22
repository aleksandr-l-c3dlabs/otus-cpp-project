#include "aabb.h"

namespace rtr {

bool AABB::intersect(const Ray& ray, float t_min, float t_max) const {
  float t0 = t_min;
  float t1 = t_max;

  for (int i = 0; i < 3; ++i) {
    if (std::fabs(ray.direction[i]) < std::numeric_limits<float>::epsilon()) {
      if (ray.origin[i] < min[i] || ray.origin[i] > max[i]) {
        return false;
      }
      continue;
    }

    float inv_dir = 1.0f / ray.direction[i];
    float t_near = (min[i] - ray.origin[i]) * inv_dir;
    float t_far = (max[i] - ray.origin[i]) * inv_dir;

    if (t_near > t_far) {
      std::swap(t_near, t_far);
    }

    t0 = std::max(t_near, t0);
    t1 = std::min(t_far, t1);

    if (t0 > t1) {
      return false;
    }
  }

  return t0 <= t_max && t1 >= t_min;
}

}  // namespace rtr