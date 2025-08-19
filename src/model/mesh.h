#pragma once

#include <memory>
#include <vector>

#include "bvh.h"
#include "material.h"
#include "vertex.h"

namespace rtr {

struct Mesh {
  std::vector<PackedVertex> vertexes;
  std::vector<size_t> indices;
  std::weak_ptr<Material> material;
  std::shared_ptr<BVHAccel> bvh;
};

}  // namespace rtr