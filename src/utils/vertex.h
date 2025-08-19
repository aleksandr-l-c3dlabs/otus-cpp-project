#pragma once

#include <array>
#include <eigen3/Eigen/Core>
#include <type_traits>

namespace rtr {

struct PackedVertex {
  std::array<float, 3> position;
  std::array<float, 3> normal;
  std::array<float, 2> texcoord;

  bool operator==(const PackedVertex&) const = default;

  [[nodiscard]] Eigen::Vector3f get_position() const {
    return {position[0], position[1], position[2]};
  }
  [[nodiscard]] Eigen::Vector3f get_normal() const {
    return {normal[0], normal[1], normal[2]};
  }
  [[nodiscard]] Eigen::Vector2f get_texcoord() const {
    return {texcoord[0], texcoord[1]};
  }
};

}  // namespace rtr

namespace std {
template <>
struct hash<rtr::PackedVertex> {
  size_t operator()(const rtr::PackedVertex& v) const {
    size_t h = 0;
    auto hash_combine = [&h](auto val) {
      h ^= hash<std::decay_t<decltype(val)>>{}(val) + 0x9e3779b9 + (h << 6) +
           (h >> 2);
    };
    for (float f : v.position)
      hash_combine(f);
    for (float f : v.normal)
      hash_combine(f);
    for (float f : v.texcoord)
      hash_combine(f);
    return h;
  }
};
}  // namespace std