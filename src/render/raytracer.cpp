#include "raytracer.h"

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <limits>
#include <numbers>

namespace rtr {

using Vector3f = Eigen::Vector3f;

/// @brief Offsetting the origin of rays prevents self-intersections
constexpr float bias = 0.001f;

// reflection
Vector3f reflect(const Vector3f& incident, const Vector3f& normal) {
  return incident - 2 * incident.dot(normal) * normal;
}

// refraction
Vector3f refract(const Vector3f& incident, const Vector3f& normal,
                 float ior_ratio) {
  float cos_theta = std::min(-incident.dot(normal), 1.0f);
  Vector3f r_out_perp = ior_ratio * (incident + cos_theta * normal);
  Vector3f r_out_parallel =
      -std::sqrt(std::abs(1.0f - r_out_perp.squaredNorm())) * normal;
  return r_out_perp + r_out_parallel;
}

Vector3f texture_color(const Vector3f& color, std::shared_ptr<Image> texture,
                       const HitRecord& rec) {
  if (!texture)
    return color;
  return texture->sample(rec.tex_coord.x(), rec.tex_coord.y());
}

void RayTracer::remove_light(size_t index) {
  lights_.erase(lights_.begin() + index);
}

void RayTracer::remove_light(const Light& light) {
  lights_.erase(std::remove(lights_.begin(), lights_.end(), light),
                lights_.end());
}

void RayTracer::build_bvh() {
  for (const auto& mesh : model_->get_meshes()) {
    if (mesh.bvh) {
      AABB mesh_bbox = mesh.bvh->get_root_bbox();
      bbox_.expand(mesh_bbox);
    }
  }
}

Vector3f RayTracer::trace_pixel(float u, float v, int max_depth) {
  Ray ray = generate_ray(u, v);
  return trace_ray(ray, max_depth);
}

Ray RayTracer::generate_ray(float u, float v) const {
  Vector3f origin = camera_->get_position();
  Vector3f direction = camera_->generate_ray(u, v);
  return Ray(origin, direction);
}

Vector3f RayTracer::trace_ray(const Ray& ray, int depth) {
  if (depth <= 0) {
    return Vector3f::Zero();
  }

  HitRecord rec;
  if (!hit_model(ray, bias, std::numeric_limits<float>::max(), rec)) {
    return background_color_;
  }

  Vector3f color_from_emission = rec.material->emission;
  Vector3f color_from_reflection = Vector3f::Zero();
  Vector3f color_from_refraction = Vector3f::Zero();

  if (rec.material->reflectivity > 0) {
    Vector3f reflected_dir = reflect(ray.direction, rec.normal).normalized();
    Vector3f reflected_origin = rec.point + rec.normal * bias;
    Ray reflected_ray(reflected_origin, reflected_dir);
    color_from_reflection =
        trace_ray(reflected_ray, depth - 1) * rec.material->reflectivity;
  }

  if (rec.material->transparency > 0) {
    float refraction_ratio =
        rec.front_face ? (1.0f / rec.material->ior) : rec.material->ior;
    Vector3f refracted_dir =
        refract(ray.direction, rec.normal, refraction_ratio);
    if (refracted_dir.norm() > 0) {
      Vector3f refracted_origin = rec.point - rec.normal * bias;
      Ray refracted_ray(refracted_origin, refracted_dir);
      color_from_refraction =
          trace_ray(refracted_ray, depth - 1) * rec.material->transparency;
    }
  }

  Vector3f direct_lighting = calculate_lighting(rec);
  return color_from_emission +
         (1.0f - rec.material->reflectivity - rec.material->transparency) *
             direct_lighting +
         color_from_reflection + color_from_refraction;
}

bool RayTracer::hit_model(const Ray& ray, float t_min, float t_max,
                          HitRecord& rec) const {
  if (!bbox_.intersect(ray, t_min, t_max))
    return false;

  HitRecord temp_rec;
  float closest_so_far = t_max;
  bool hit_anything = false;

  for (const auto& mesh : model_->get_meshes()) {
    auto material_ptr = mesh.material.lock();
    if (!material_ptr)
      continue;

    const auto& indices =
        mesh.bvh ? mesh.bvh->get_intersect_indices(ray, t_min, t_max)
                 : IntersectIndices(mesh.indices);

    for (size_t i = 0; i < indices.size(); i += 3) {
      const auto& v0 = mesh.vertexes[indices[i]];
      const auto& v1 = mesh.vertexes[indices[i + 1]];
      const auto& v2 = mesh.vertexes[indices[i + 2]];

      if (hit_triangle(ray, v0, v1, v2, t_min, closest_so_far, temp_rec)) {
        closest_so_far = temp_rec.t;
        rec = temp_rec;
        rec.material = material_ptr;
        hit_anything = true;
      }
    }
  }

  return hit_anything;
}

bool RayTracer::hit_triangle(const Ray& ray, const PackedVertex& v0,
                             const PackedVertex& v1, const PackedVertex& v2,
                             float t_min, float t_max, HitRecord& rec) const {
  const auto p0 = v0.get_position();
  const auto p1 = v1.get_position();
  const auto p2 = v2.get_position();

  // Möller–Trumbore intersection algorithm
  const Vector3f e1 = p1 - p0;
  const Vector3f e2 = p2 - p0;
  const Vector3f h = ray.direction.cross(e2);
  const float a = e1.dot(h);

  if (a > -std::numeric_limits<float>::epsilon() &&
      a < std::numeric_limits<float>::epsilon()) {
    return false;  // The ray is parallel to the triangle
  }

  const float f = 1.0f / a;
  const Vector3f s = ray.origin - p0;
  const float u = f * s.dot(h);

  if (u < 0.0f || u > 1.0f) {
    return false;
  }

  const Vector3f q = s.cross(e1);
  const float v = f * ray.direction.dot(q);

  if (v < 0.0f || u + v > 1.0f) {
    return false;
  }

  const float t = f * e2.dot(q);

  if (t > t_min && t < t_max) {
    rec.t = t;
    rec.point = ray.origin + t * ray.direction;

    // Normal interpolation
    float w = 1.0f - u - v;
    rec.normal =
        (w * v0.get_normal() + u * v1.get_normal() + v * v2.get_normal())
            .normalized();
    rec.set_face_normal(ray, rec.normal);

    // Texture coordinate interpolation
    rec.tex_coord =
        w * v0.get_texcoord() + u * v1.get_texcoord() + v * v2.get_texcoord();

    return true;
  }

  return false;
}

/// @brief Light calculation by Phong method
/// @param rec
/// @return
Vector3f RayTracer::calculate_lighting(const HitRecord& rec) {
  Vector3f ambient =
      texture_color(rec.material->ambient, rec.material->ambient_texture, rec);
  Vector3f diffuse = Vector3f::Zero();
  Vector3f specular = Vector3f::Zero();

  Vector3f view_dir = (camera_->get_position() - rec.point).normalized();

  for (const auto& light : lights_) {
    Vector3f light_dir = light.position - rec.point;
    float distance = light_dir.norm();
    light_dir = light_dir.normalized();
    float attenuation = 1.0 / (distance * distance);

    // diffuse
    float diff = std::max(light_dir.dot(rec.normal), 0.0f);
    diffuse += attenuation * diff *
               light.intensity.cwiseProduct(texture_color(
                   rec.material->diffuse, rec.material->diffuse_texture, rec));

    // reflection
    Vector3f reflect_dir = reflect(-light_dir, rec.normal);
    float spec =
        pow(std::max(view_dir.dot(reflect_dir), 0.0f), rec.material->shininess);
    specular += attenuation * spec *
                light.intensity.cwiseProduct(rec.material->specular);
  }

  return ambient + diffuse + specular;
}

}  // namespace rtr
