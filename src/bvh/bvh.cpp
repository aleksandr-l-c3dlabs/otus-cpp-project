#include "bvh.h"

#include <algorithm>
#include <execution>
#include <future>
#include <numeric>

namespace rtr {

Vector3f min_point(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3) {
  return v1.cwiseMin(v2).cwiseMin(v3);
}

Vector3f max_point(const Vector3f& v1, const Vector3f& v2, const Vector3f& v3) {
  return v1.cwiseMax(v2).cwiseMax(v3);
}

AABB compute_bbox(const std::array<Eigen::Vector3f, 3>& p) {
  return AABB(min_point(p[0], p[1], p[2]), max_point(p[0], p[1], p[2]));
}

AABB compute_bbox(const std::vector<BVHTriangle>& triangles) {
  AABB bbox;
  for (const auto& triangle : triangles) {
    bbox.expand(triangle.bbox);
  }
  return bbox;
}

std::vector<size_t> join_indices(const std::vector<BVHTriangle>& triangles) {
  std::vector<size_t> result;
  result.reserve(3 * triangles.size());

  for (const auto& triangle : triangles) {
    result.insert(result.end(), triangle.indices.begin(),
                  triangle.indices.end());
  }

  return std::move(result);
}

size_t IntersectIndices::size() const {
  if (offsets.empty())
    return 0;
  return *offsets.rbegin() + (*indices_ptrs.rbegin())->size();
}

void IntersectIndices::update_offsets() {
  offsets.clear();
  size_t current = 0;
  for (const auto& ptr : indices_ptrs) {
    offsets.push_back(current);
    current += ptr->size();
  }
}

size_t IntersectIndices::operator[](size_t global_index) const {
  auto it = std::upper_bound(offsets.begin(), offsets.end(), global_index);
  size_t segment = std::distance(offsets.begin(), it) - 1;
  return (*indices_ptrs[segment])[global_index - offsets[segment]];
}

BVHAccel::BVHAccel(const std::vector<PackedVertex>& vertices,
                   const std::vector<size_t>& indices) {
  TriangleVector triangles;
  triangles.reserve(indices.size() / 3);
  for (size_t i = 0; i < indices.size(); i += 3) {
    auto i1 = indices[i];
    auto i2 = indices[i + 1];
    auto i3 = indices[i + 2];

    std::array<Eigen::Vector3f, 3> points{vertices[i1].get_position(),
                                          vertices[i2].get_position(),
                                          vertices[i3].get_position()};

    triangles.push_back(BVHTriangle{std::array<size_t, 3>{i1, i2, i3}, points,
                                    (points[0] + points[1] + points[2]) / 3.f,
                                    compute_bbox(points)});
  }
  root_ = build_node(triangles);
}
IntersectIndices BVHAccel::get_intersect_indices(const Ray& ray, float t_min,
                                                 float t_max) const {
  return get_intersect_node_indices(root_.get(), ray, t_min, t_max);
}

std::unique_ptr<BVHNode> BVHAccel::build_node(TriangleVector& triangles,
                                              int depth) {
  auto node = std::make_unique<BVHNode>();
  node->bbox = compute_bbox(triangles);

  if (triangles.size() <= 4 || depth > 20) {
    node->triangle_indices = join_indices(triangles);
    node->is_leaf = true;
    return node;
  }

  Eigen::Vector3f extent = node->bbox.max - node->bbox.min;
  int axis = (extent[0] > extent[1] && extent[0] > extent[2]) ? 0
             : (extent[1] > extent[2])                        ? 1
                                                              : 2;

  auto comparator = [axis](const BVHTriangle& a, const BVHTriangle& b) {
    return a.center[axis] < b.center[axis];
  };
  std::sort(std::execution::par, triangles.begin(), triangles.end(),
            comparator);

  auto mid = triangles.begin() + triangles.size() / 2;
  TriangleVector left_triangles(triangles.begin(), mid);
  TriangleVector right_triangles(mid, triangles.end());

  auto left_future = std::async(std::launch::async, [&]() {
    return build_node(left_triangles, depth + 1);
  });
  auto right_future = std::async(std::launch::async, [&]() {
    return build_node(right_triangles, depth + 1);
  });

  node->left = std::move(left_future.get());
  node->right = std::move(right_future.get());

  return node;
}

IntersectIndices BVHAccel::get_intersect_node_indices(const BVHNode* node,
                                                      const Ray& ray,
                                                      float t_min,
                                                      float t_max) const {
  if (!node->bbox.intersect(ray, t_min, t_max))
    return {};

  IntersectIndices result;

  if (node->is_leaf) {
    result.add(node->triangle_indices);
    return result;
  }

  auto left = get_intersect_node_indices(node->left.get(), ray, t_min, t_max);
  auto right = get_intersect_node_indices(node->right.get(), ray, t_min, t_max);

  result.indices_ptrs.reserve(left.indices_ptrs.size() +
                              right.indices_ptrs.size());
  result.indices_ptrs.insert(result.indices_ptrs.end(),
                             left.indices_ptrs.begin(),
                             left.indices_ptrs.end());
  result.indices_ptrs.insert(result.indices_ptrs.end(),
                             right.indices_ptrs.begin(),
                             right.indices_ptrs.end());
  result.update_offsets();
  return result;
}

}  // namespace rtr