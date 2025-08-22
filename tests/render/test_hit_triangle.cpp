#include <gtest/gtest.h>
#include <memory>
#include "raytracer.h"
#include "vertex.h"

using namespace rtr;
using namespace Eigen;

class TestRayTracer : public RayTracer {
 public:
  TestRayTracer(std::shared_ptr<const Model> model,
                std::shared_ptr<const Camera> camera,
                const Vector3f& bg_color = Vector3f(0.898f, 0.95687f, 1.0f))
      : RayTracer(model, camera, bg_color) {}
  ~TestRayTracer() = default;

  [[nodiscard]] static bool hit_triangle(const Ray& ray, const PackedVertex& v0,
                                         const PackedVertex& v1,
                                         const PackedVertex& v2, float t_min,
                                         float t_max, HitRecord& rec) {
    return RayTracer::hit_triangle(ray, v0, v1, v2, t_min, t_max, rec);
  }
};

class TriangleIntersectionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Простой треугольник в плоскости XY
    v0 = {{0, 0, 0}, {0, 0, 1}, {0, 0}};
    v1 = {{1, 0, 0}, {0, 0, 1}, {1, 0}};
    v2 = {{0, 1, 0}, {0, 0, 1}, {0, 1}};

    // Луч через центр треугольника
    ray_through_center = std::make_unique<Ray>(Vector3f{0.3f, 0.3f, -1.0f},
                                               Vector3f{0.0f, 0.0f, 1.0f});

    // Луч мимо треугольника
    ray_miss = std::make_unique<Ray>(Vector3f{2.0f, 2.0f, -1.0f},
                                     Vector3f{0.0f, 0.0f, 1.0f});
  }

  PackedVertex v0, v1, v2;
  std::unique_ptr<Ray> ray_through_center;
  std::unique_ptr<Ray> ray_miss;
};

// Базовое пересечение
TEST_F(TriangleIntersectionTest, BasicIntersection) {
  HitRecord rec;
  bool result = TestRayTracer::hit_triangle(*ray_through_center.get(), v0, v1,
                                            v2, 0.0f, 10.0f, rec);

  EXPECT_TRUE(result);
  EXPECT_GT(rec.t, 0.0f);
  EXPECT_LT(rec.t, 10.0f);
  EXPECT_TRUE(rec.normal.norm() > 0.9f);  // Нормализованная нормаль
}

// Промах
TEST_F(TriangleIntersectionTest, RayMissesTriangle) {
  HitRecord rec;
  bool result = TestRayTracer::hit_triangle(*ray_miss.get(), v0, v1, v2, 0.0f,
                                            10.0f, rec);
  EXPECT_FALSE(result);
}

// Вырожденный треугольник (все точки совпадают)
TEST_F(TriangleIntersectionTest, DegenerateTriangle) {
  PackedVertex degenerate = {{0, 0, 0}, {0, 0, 1}, {0, 0}};
  HitRecord rec;

  bool result =
      TestRayTracer::hit_triangle(*ray_through_center.get(), degenerate,
                                  degenerate, degenerate, 0.0f, 10.0f, rec);
  EXPECT_FALSE(result);  // Не должно быть пересечения
}

// Луч параллелен плоскости треугольника
TEST_F(TriangleIntersectionTest, RayParallelToTriangle) {
  Ray parallel_ray({0.3f, 0.3f, 1.0f},
                   {1.0f, 0.0f, 0.0f});  // Параллелен плоскости XY
  HitRecord rec;

  bool result =
      TestRayTracer::hit_triangle(parallel_ray, v0, v1, v2, 0.0f, 10.0f, rec);
  EXPECT_FALSE(result);
}

// Луч начинается внутри треугольника
TEST_F(TriangleIntersectionTest, DISABLED_RayStartsInsideTriangle) {
  Ray ray_from_inside({0.3f, 0.3f, 0.0f}, {0.0f, 0.0f, 1.0f});
  HitRecord rec;

  bool result = TestRayTracer::hit_triangle(ray_from_inside, v0, v1, v2, 0.0f,
                                            10.0f, rec);
  EXPECT_TRUE(result);
  EXPECT_NEAR(rec.t, 0.0f, 1e-5f);  // t должно быть близко к 0
}

// Ограничения по t_range
TEST_F(TriangleIntersectionTest, TRangeLimits) {
  HitRecord rec;

  // Пересечение при t=1.0, должно быть найдено
  bool result1 = TestRayTracer::hit_triangle(*ray_through_center.get(), v0, v1,
                                             v2, 0.5f, 1.5f, rec);
  bool result2 = TestRayTracer::hit_triangle(*ray_through_center.get(), v0, v1,
                                             v2, 2.0f, 3.0f, rec);
  bool result3 = TestRayTracer::hit_triangle(*ray_through_center.get(), v0, v1,
                                             v2, 0.0f, 0.5f, rec);

  EXPECT_TRUE(result1);
  EXPECT_FALSE(result2);
  EXPECT_FALSE(result3);
}