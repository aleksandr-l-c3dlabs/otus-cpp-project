#pragma once

#include <eigen3/Eigen/Core>
#include <memory>
#include <vector>

#include "aabb.h"
#include "camera.h"
#include "material.h"
#include "model.h"
#include "ray.h"

namespace rtr {

class RayTracer {
 public:
  using Vector3f = Eigen::Vector3f;
  using Vector2f = Eigen::Vector2f;

  RayTracer(std::shared_ptr<const Model> model,
            std::shared_ptr<const Camera> camera,
            const Vector3f& bg_color = Vector3f(0.898f, 0.95687f, 1.0f))
      : model_(model), camera_(camera), background_color_(bg_color) {}

  void add_light(const Light& light) { lights_.push_back(light); }
  [[nodiscard]] const std::vector<Light> get_lights() const { return lights_; }
  void remove_light(size_t index);
  void remove_light(const Light& light);

  [[nodiscard]] Vector3f trace_pixel(float u, float v, int max_depth = 5);

  void build_bvh();
  [[nodiscard]] AABB get_root_bbox() const { return bbox_; };

 private:
  [[nodiscard]] Ray generate_ray(float u, float v) const;

  [[nodiscard]] Vector3f trace_ray(const Ray& ray, int depth);

  bool hit_model(const Ray& ray, float t_min, float t_max,
                 HitRecord& rec) const;

  bool hit_triangle(const Ray& ray, const PackedVertex& v0,
                    const PackedVertex& v1, const PackedVertex& v2, float t_min,
                    float t_max, HitRecord& rec) const;

  [[nodiscard]] Vector3f calculate_lighting(const HitRecord& rec);

 private:
  std::shared_ptr<const Model> model_;
  std::shared_ptr<const Camera> camera_;
  Vector3f background_color_;
  std::vector<Light> lights_;
  AABB bbox_;
};

inline bool operator==(const rtr::Light& left, const rtr::Light& right) {
  return left.position == right.position && left.intensity == right.intensity;
}

}  // namespace rtr
