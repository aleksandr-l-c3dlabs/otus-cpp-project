#pragma once

#include <eigen3/Eigen/Core>
#include <memory>

#include "image.h"

namespace rtr {

struct Material {
  Eigen::Vector3f ambient;
  Eigen::Vector3f diffuse;
  Eigen::Vector3f specular;
  Eigen::Vector3f transmittance;
  Eigen::Vector3f emission;
  float ior;
  float shininess;
  float transparency;
  float reflectivity = 0.f;

  std::shared_ptr<Image> diffuse_texture;
  std::shared_ptr<Image> ambient_texture;
};

}  // namespace rtr