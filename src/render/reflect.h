#pragma once

#include <eigen3/Eigen/Core>

namespace rtr {

// reflection
Eigen::Vector3f reflect(const Eigen::Vector3f& incident,
                        const Eigen::Vector3f& normal);

// refraction
Eigen::Vector3f refract(const Eigen::Vector3f& incident,
                        const Eigen::Vector3f& normal, float ior_ratio);

}  // namespace rtr