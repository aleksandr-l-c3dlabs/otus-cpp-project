#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <vector>

#include "material.h"
#include "mesh.h"

namespace fs = std::filesystem;

namespace rtr {

class Model {
 public:
  Model() {}
  ~Model() = default;

  [[nodiscard]] const std::vector<Mesh>& get_meshes() const { return meshes; }

  [[nodiscard]] static std::optional<Model> import(const fs::path& path);

 private:
  std::vector<Mesh> meshes;
  std::vector<std::shared_ptr<Material>> materials;
};

}  // namespace rtr