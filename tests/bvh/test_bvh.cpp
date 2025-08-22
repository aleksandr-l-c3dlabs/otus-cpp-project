#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include "bvh.h"
#include "ray.h"
#include "vertex.h"

using namespace rtr;

class BVHTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Простые тестовые вершины и индексы
    vertices = {
        {{0, 0, 0}, {0, 1, 0}, {0, 0}},  // v0
        {{1, 0, 0}, {0, 1, 0}, {1, 0}},  // v1
        {{0, 1, 0}, {0, 1, 0}, {0, 1}},  // v2
        {{2, 0, 0}, {0, 1, 0}, {0, 0}},  // v3
        {{0, 2, 0}, {0, 1, 0}, {0, 0}}   // v4
    };

    indices = {0, 1, 2, 1, 3, 4};  // Два треугольника
  }

  std::vector<PackedVertex> vertices;
  std::vector<size_t> indices;
};

// Базовое построение BVH
TEST_F(BVHTest, ConstructionWithValidData) {
  EXPECT_NO_THROW({
    BVHAccel bvh(vertices, indices);
    EXPECT_FALSE(bvh.get_root_bbox().min.isApprox(bvh.get_root_bbox().max));
  });
}

// Пустые данные
TEST_F(BVHTest, ConstructionWithEmptyData) {
  std::vector<PackedVertex> empty_vertices;
  std::vector<size_t> empty_indices;

  EXPECT_NO_THROW({
    BVHAccel bvh(empty_vertices, empty_indices);
    // BBOX должен быть валидным даже для пустых данных
    auto bbox = bvh.get_root_bbox();
    EXPECT_TRUE(bbox.min.x() > bbox.max.x());
  });
}

// Вырожденные треугольники
TEST_F(BVHTest, ConstructionWithDegenerateTriangles) {
  std::vector<PackedVertex> deg_vertices = {
      {{0, 0, 0}, {0, 1, 0}, {0, 0}},
      {{0, 0, 0}, {0, 1, 0}, {0, 0}},  // Та же точка
      {{0, 0, 0}, {0, 1, 0}, {0, 0}}   // Та же точка
  };
  std::vector<size_t> deg_indices = {0, 1, 2};

  EXPECT_NO_THROW({ BVHAccel bvh(deg_vertices, deg_indices); });
}

// Тестирование пересечений
TEST_F(BVHTest, IntersectionQueries) {
  BVHAccel bvh(vertices, indices);
  Ray ray({0.5f, 0.5f, -1.0f}, {0.0f, 0.0f, 1.0f});  // Вниз через центр

  auto intersect_indices = bvh.get_intersect_indices(ray, 0.0f, 10.0f);
  EXPECT_FALSE(intersect_indices.empty());
}

// Луч мимо сцены
TEST_F(BVHTest, RayMissesEverything) {
  BVHAccel bvh(vertices, indices);
  Ray ray({10.0f, 10.0f, 10.0f}, {1.0f, 0.0f, 0.0f});  // Далеко от сцены

  auto intersect_indices = bvh.get_intersect_indices(ray, 0.0f, 100.0f);
  EXPECT_TRUE(intersect_indices.empty());
}

// Краевые случаи с t_range
TEST_F(BVHTest, IntersectionWithTRangeLimits) {
  BVHAccel bvh(vertices, indices);
  Ray ray({0.5f, 0.5f, -2.0f}, {0.0f, 0.0f, 1.0f});

  // Пересечение при t=2.0, должно быть найдено
  auto indices1 = bvh.get_intersect_indices(ray, 1.5f, 2.5f);
  auto indices2 = bvh.get_intersect_indices(ray, 3.0f, 4.0f);  // После
  auto indices3 = bvh.get_intersect_indices(ray, 0.0f, 1.0f);  // До

  EXPECT_FALSE(indices1.empty());
  EXPECT_TRUE(indices2.empty());
  EXPECT_TRUE(indices3.empty());
}