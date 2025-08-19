#pragma once

#include <eigen3/Eigen/Core>
#include <memory>

#include "material.h"

namespace rtr {

using namespace Eigen;

struct Light {
  Vector3f position;
  Vector3f intensity;
};

struct Ray {
  Vector3f origin;
  Vector3f direction;
  Ray(const Vector3f& o, const Vector3f& d)
      : origin(o), direction(d.normalized()) {}
};

struct HitRecord {
  float t = std::numeric_limits<float>::max();
  Vector3f point;
  Vector3f normal;
  Vector2f tex_coord;
  std::shared_ptr<Material> material;
  bool front_face;

  inline void set_face_normal(const Ray& ray, const Vector3f& outward_normal) {
    front_face = ray.direction.dot(outward_normal) < 0;
    normal = front_face ? outward_normal : -outward_normal;
  }
};

}  // namespace rtr