#include "reflect.h"

using namespace Eigen;

namespace rtr {

Vector3f reflect(const Vector3f& incident, const Vector3f& normal) {
  float incident_len2 = incident.squaredNorm();
  float normal_len2 = normal.squaredNorm();

  if (incident_len2 < 1e-12f || normal_len2 < 1e-12f) {
    return Vector3f::Zero();
  }

  float dot_product = incident.dot(normal);
  return incident - (2.0f * dot_product / normal_len2) * normal;
}

Vector3f refract(const Vector3f& incident, const Vector3f& normal,
                 float ior_ratio) {
  float incident_len2 = incident.squaredNorm();
  float normal_len2 = normal.squaredNorm();

  if (incident_len2 < 1e-12f || normal_len2 < 1e-12f ||
      std::fabs(ior_ratio) < 1e-12f) {
    return Vector3f::Zero();
  }

  Vector3f incident_dir = incident / std::sqrt(incident_len2);
  Vector3f normal_dir = normal / std::sqrt(normal_len2);

  float cos_theta = incident_dir.dot(normal_dir);
  if (cos_theta > 0.0f) {
    normal_dir = -normal_dir;
    cos_theta = -cos_theta;
  }

  float k = 1.0f - ior_ratio * ior_ratio * (1.0f - cos_theta * cos_theta);
  if (k < 0.0f) {
    return Vector3f::Zero();
  }

  return ior_ratio * incident_dir +
         (ior_ratio * cos_theta - std::sqrt(k)) * normal_dir;
}

}  // namespace rtr