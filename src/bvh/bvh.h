#pragma once

#include <array>
#include <eigen3/Eigen/Core>
#include <memory>
#include <vector>

#include "aabb.h"
#include "vertex.h"

namespace rtr {

struct BVHNode {
  AABB bbox;
  std::unique_ptr<BVHNode> left;
  std::unique_ptr<BVHNode> right;
  std::vector<size_t> triangle_indices;
  bool is_leaf = false;
};

struct BVHTriangle {
  std::array<size_t, 3> indices;
  std::array<Eigen::Vector3f, 3> vertexes;
  Eigen::Vector3f center;
  AABB bbox;
};

struct IntersectIndices {
  std::vector<const std::vector<size_t>*> indices_ptrs;
  std::vector<size_t> offsets;

  IntersectIndices() = default;
  IntersectIndices(const std::vector<size_t>& indices)
      : indices_ptrs{&indices} {
    update_offsets();
  }
  IntersectIndices(std::initializer_list<const std::vector<size_t>*> lists)
      : indices_ptrs(lists) {
    update_offsets();
  }

  void add(const std::vector<size_t>& vec) {
    indices_ptrs.push_back(&vec);
    update_offsets();
  }
  void update_offsets();
  [[nodiscard]] bool empty() const { return indices_ptrs.empty(); }
  [[nodiscard]] size_t size() const;
  [[nodiscard]] size_t operator[](size_t global_index) const;
};

class BVHAccel {
 private:
  using TriangleVector = std::vector<BVHTriangle>;

 public:
  BVHAccel(const std::vector<PackedVertex>& vertices,
           const std::vector<size_t>& indices);

  [[nodiscard]] IntersectIndices get_intersect_indices(const Ray& ray,
                                                       float t_min,
                                                       float t_max) const;

  [[nodiscard]] AABB get_root_bbox() const { return root_->bbox; }

 private:
  std::unique_ptr<BVHNode> build_node(TriangleVector& triangles, int depth = 0);

  IntersectIndices get_intersect_node_indices(const BVHNode* node,
                                              const Ray& ray, float t_min,
                                              float t_max) const;

 private:
  std::unique_ptr<BVHNode> root_;
};

}  // namespace rtr