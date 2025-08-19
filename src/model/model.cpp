#include "model.h"

#include <stb_image.h>
#include <tiny_obj_loader.h>
#include <algorithm>
#include <execution>
#include <iostream>
#include <mutex>
#include <unordered_map>

#define TINYOBJLOADER_IMPLEMENTATION

namespace rtr {

std::shared_ptr<Image> load_texture(const fs::path& base, fs::path tex) {
  std::shared_ptr<Image> result;

  {
    std::string s = tex.string();
    std::replace(s.begin(), s.end(), '\\', '/');
    tex = s;
  }

  fs::path path = tex;
  while (!fs::exists(path) && !tex.empty()) {
    fs::path rel = tex.relative_path();
    path = base / rel;
    tex = rel.lexically_relative(*rel.begin());
  }

  if (!path.empty()) {
    std::string texture_path = path.string();
    int w, h, channels;
    unsigned char* data = stbi_load(texture_path.c_str(), &w, &h, &channels, 3);

    if (data) {
      result = std::make_shared<Image>(w, h, data);
      stbi_image_free(data);
    } else {
      std::cerr << "ERR: Failed to load texture: " << texture_path << "\n";
    }
  }

  return result;
}

std::optional<Model> Model::import(const fs::path& path) {
  tinyobj::ObjReaderConfig reader_config;
  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(path.string(), reader_config)) {
    std::cerr << "ERR: " << reader.Error() << std::endl;
    return {};
  }

  if (!reader.Warning().empty()) {
    std::cout << "WARN: " << reader.Warning() << std::endl;
  }

  Model model;

  auto attrib = reader.GetAttrib();

  // Materials
  for (const auto& mat : reader.GetMaterials()) {
    auto material = std::make_shared<Material>(Material{
        {mat.ambient[0], mat.ambient[1], mat.ambient[2]},
        {mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]},
        {mat.specular[0], mat.specular[1], mat.specular[2]},
        {mat.transmittance[0], mat.transmittance[1], mat.transmittance[2]},
        {mat.emission[0], mat.emission[1], mat.emission[2]},
        mat.ior,
        mat.shininess,
        1 - mat.dissolve,
    });

    // reflectivity prop
    if (mat.illum == 5 || mat.illum == 7) {  // metal
      material->reflectivity = 0.8f;
    } else if (material->specular.norm() > 0.8f &&
               mat.shininess > 50.0f) {  // mirror
      material->reflectivity = 0.9f;
    } else if (material->transparency > 0.1f && mat.ior > 1.2f) {  // glass
      material->reflectivity = 0.1f;
    } else {  // default
      material->reflectivity =
          std::max({mat.specular[0], mat.specular[1], mat.specular[2]});
    }

    // textures
    material->ambient_texture =
        load_texture(path.parent_path(), mat.ambient_texname);
    material->diffuse_texture =
        load_texture(path.parent_path(), mat.diffuse_texname);

    model.materials.push_back(material);
  }

  std::unordered_map<int, std::vector<tinyobj::index_t>> material_triangles;
  for (const auto& shape : reader.GetShapes()) {
    size_t index_offset = 0;
    for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++) {
      int mat_id = std::max(0, shape.mesh.material_ids[f]);  // default to 0
      size_t fv = shape.mesh.num_face_vertices[f];

      if (fv == 3) {
        for (size_t v = 0; v < 3; v++) {
          material_triangles[mat_id].push_back(
              shape.mesh.indices[index_offset + v]);
        }
      } else {
        std::cout << "WARN: Face with unsupported vertices: " << fv
                  << std::endl;
      }
      index_offset += fv;
    }
  }

  model.meshes.reserve(material_triangles.size());
  for (const auto& [mat_id, indices] : material_triangles) {
    Mesh mesh;
    if (mat_id < model.materials.size()) {
      mesh.material = model.materials[mat_id];
    }

    std::mutex mesh_mutex;
    std::unordered_map<PackedVertex, size_t> unique_vertices;
    mesh.indices.reserve(indices.size());

    const size_t block_size = 1024;
    const size_t blocks = (indices.size() + block_size - 1) / block_size;
    std::vector<size_t> blocks_vec(blocks);
    std::iota(blocks_vec.begin(), blocks_vec.end(), 0);

    const auto& local_indices = indices;
    const auto& local_attrib = attrib;

    std::for_each(
        std::execution::par, blocks_vec.begin(), blocks_vec.end(),
        [&](size_t block) {
          size_t start = block * block_size;
          size_t end = std::min(start + block_size, local_indices.size());

          for (size_t i = start; i < end; i++) {
            const auto& idx = local_indices[i];

            PackedVertex vertex;
            memcpy(vertex.position.data(),
                   &local_attrib.vertices[3 * idx.vertex_index], 12);
            if (idx.normal_index >= 0)
              memcpy(vertex.normal.data(),
                     &local_attrib.normals[3 * idx.normal_index], 12);
            if (idx.texcoord_index >= 0)
              memcpy(vertex.texcoord.data(),
                     &local_attrib.texcoords[2 * idx.texcoord_index], 8);

            {
              std::lock_guard<std::mutex> lock(mesh_mutex);

              if (auto it = unique_vertices.find(vertex);
                  it != unique_vertices.end()) {
                mesh.indices.push_back(it->second);
              } else {
                uint32_t new_idx = mesh.vertexes.size();
                mesh.vertexes.push_back(vertex);
                unique_vertices[vertex] = new_idx;
                mesh.indices.push_back(new_idx);
              }
            }
          }
        });

    mesh.bvh = std::make_shared<BVHAccel>(mesh.vertexes, mesh.indices);

    model.meshes.emplace_back(std::move(mesh));
  }

  return model;
}

}  // namespace rtr