#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include "aabb.h"
#include "ray.h"

using namespace rtr;
using namespace Eigen;

class AABBIntersectTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Стандартный AABB от (0,0,0) до (1,1,1)
    standard_bbox = AABB(Vector3f(0, 0, 0), Vector3f(1, 1, 1));

    // AABB в отрицательных координатах
    negative_bbox = AABB(Vector3f(-2, -2, -2), Vector3f(-1, -1, -1));

    // Большой AABB
    large_bbox = AABB(Vector3f(-5, -5, -5), Vector3f(5, 5, 5));
  }

  AABB standard_bbox;
  AABB negative_bbox;
  AABB large_bbox;
};

// Тест 1: Луч проходит через центр AABB
TEST_F(AABBIntersectTest, RayThroughCenter) {
  Ray ray(Vector3f(0.5f, -1.0f, 0.5f), Vector3f(0.0f, 1.0f, 0.0f));
  EXPECT_TRUE(standard_bbox.intersect(ray, 0.0f, 100.0f));
}

// Тест 2: Луч проходит через угол AABB
TEST_F(AABBIntersectTest, RayThroughCorner) {
  Ray ray(Vector3f(-1.0f, -1.0f, -1.0f),
          Vector3f(1.0f, 1.0f, 1.0f).normalized());
  EXPECT_TRUE(standard_bbox.intersect(ray, 0.0f, 100.0f));
}

// Тест 3: Луч параллелен одной из осей и пересекает AABB
TEST_F(AABBIntersectTest, RayParallelToAxisIntersects) {
  Ray ray_x(Vector3f(0.5f, -1.0f, 0.5f), Vector3f(0.0f, 1.0f, 0.0f));
  Ray ray_y(Vector3f(-1.0f, 0.5f, 0.5f), Vector3f(1.0f, 0.0f, 0.0f));
  Ray ray_z(Vector3f(0.5f, 0.5f, -1.0f), Vector3f(0.0f, 0.0f, 1.0f));

  EXPECT_TRUE(standard_bbox.intersect(ray_x, 0.0f, 100.0f));
  EXPECT_TRUE(standard_bbox.intersect(ray_y, 0.0f, 100.0f));
  EXPECT_TRUE(standard_bbox.intersect(ray_z, 0.0f, 100.0f));
}

// Тест 4: Луч проходит мимо AABB
TEST_F(AABBIntersectTest, RayMissesAABB) {
  Ray ray(Vector3f(2.0f, 2.0f, 2.0f), Vector3f(1.0f, 0.0f, 0.0f));
  EXPECT_FALSE(standard_bbox.intersect(ray, 0.0f, 100.0f));
}

// Тест 5: Луч начинается внутри AABB
TEST_F(AABBIntersectTest, RayStartsInsideAABB) {
  Ray ray(Vector3f(0.5f, 0.5f, 0.5f), Vector3f(1.0f, 0.0f, 0.0f));
  EXPECT_TRUE(standard_bbox.intersect(ray, 0.0f, 100.0f));
}

// Тест 6: Луч начинается на границе AABB
TEST_F(AABBIntersectTest, RayStartsOnBoundary) {
  Ray ray1(Vector3f(0.0f, 0.5f, 0.5f), Vector3f(1.0f, 0.0f, 0.0f));
  Ray ray2(Vector3f(1.0f, 0.5f, 0.5f), Vector3f(-1.0f, 0.0f, 0.0f));

  EXPECT_TRUE(standard_bbox.intersect(ray1, 0.0f, 100.0f));
  EXPECT_TRUE(standard_bbox.intersect(ray2, 0.0f, 100.0f));
}

// Тест 7: Луч направлен away from AABB
TEST_F(AABBIntersectTest, DISABLED_RayAwayFromAABB) {
  Ray ray(Vector3f(0.5f, 0.5f, 0.5f), Vector3f(0.0f, -1.0f, 0.0f));
  EXPECT_TRUE(
      standard_bbox.intersect(ray, 0.0f, 0.5f));  // Должен пересекать при t=0
  EXPECT_FALSE(standard_bbox.intersect(ray, 0.1f, 100.0f));  // Но не дальше
}

// Тест 8: Луч касается AABB (касательное пересечение)
TEST_F(AABBIntersectTest, RayGrazesAABB) {
  // Луч проходит точно по грани
  Ray ray(Vector3f(0.0f, -1.0f, 0.5f), Vector3f(0.0f, 1.0f, 0.0f));
  EXPECT_TRUE(standard_bbox.intersect(ray, 0.0f, 100.0f));
}

// Тест 9: Отрицательный AABB
TEST_F(AABBIntersectTest, DISABLED_NegativeCoordinatesAABB) {
  Ray ray1(Vector3f(-1.5f, -1.5f, -1.5f),
           Vector3f(1.0f, 1.0f, 1.0f).normalized());
  Ray ray2(Vector3f(0.0f, 0.0f, 0.0f),
           Vector3f(-1.0f, -1.0f, -1.0f).normalized());

  EXPECT_TRUE(negative_bbox.intersect(ray1, 0.0f, 100.0f));
  EXPECT_FALSE(negative_bbox.intersect(ray2, 0.1f, 100.0f));
}

// Тест 10: Большой AABB
TEST_F(AABBIntersectTest, LargeAABB) {
  Ray ray(Vector3f(-10.0f, 0.0f, 0.0f), Vector3f(1.0f, 0.0f, 0.0f));
  EXPECT_TRUE(large_bbox.intersect(ray, 0.0f, 100.0f));
}

// Тест 11: Ограничение по t_min/t_max
TEST_F(AABBIntersectTest, DISABLED_TRangeLimits) {
  Ray ray(Vector3f(0.5f, -2.0f, 0.5f), Vector3f(0.0f, 1.0f, 0.0f));

  // Пересечение происходит при t=1.0 (от y=-2 до y=0)
  EXPECT_TRUE(standard_bbox.intersect(ray, 0.5f, 1.5f));   // В диапазоне
  EXPECT_FALSE(standard_bbox.intersect(ray, 2.0f, 3.0f));  // После диапазона
  EXPECT_FALSE(standard_bbox.intersect(ray, 0.0f, 0.5f));  // До диапазона
}

// Тест 12: Вырожденный AABB (точка)
TEST_F(AABBIntersectTest, DegenerateAABBPoint) {
  AABB point_bbox(Vector3f(0.5f, 0.5f, 0.5f), Vector3f(0.5f, 0.5f, 0.5f));
  Ray ray1(Vector3f(0.5f, 0.5f, 0.5f), Vector3f(1.0f, 0.0f, 0.0f));
  Ray ray2(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f).normalized());

  EXPECT_TRUE(point_bbox.intersect(ray1, 0.0f, 100.0f));
  EXPECT_TRUE(point_bbox.intersect(ray2, 0.0f, 100.0f));
}

// Тест 13: Вырожденный AABB (линия)
TEST_F(AABBIntersectTest, DegenerateAABBLine) {
  AABB line_bbox(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 0.0f, 0.0f));
  Ray ray1(Vector3f(0.5f, 0.0f, 0.0f), Vector3f(1.0f, 0.0f, 0.0f));
  Ray ray2(Vector3f(0.5f, 1.0f, 0.0f), Vector3f(0.0f, -1.0f, 0.0f));
  Ray ray3(Vector3f(0.5f, 1.0f, 1.0f), Vector3f(0.0f, -1.0f, 0.0f));

  EXPECT_TRUE(line_bbox.intersect(ray1, 0.0f, 100.0f));
  EXPECT_TRUE(line_bbox.intersect(ray2, 0.0f, 100.0f));
  EXPECT_FALSE(line_bbox.intersect(ray3, 0.0f, 100.0f));
}

// Тест 14: Луч с отрицательным направлением
TEST_F(AABBIntersectTest, RayWithNegativeDirection) {
  Ray ray1(Vector3f(0.5f, 2.0f, 0.5f), Vector3f(0.0f, -1.0f, 0.0f));
  Ray ray2(Vector3f(2.0f, 0.5f, 0.5f), Vector3f(-1.0f, 0.0f, 0.0f));
  Ray ray3(Vector3f(0.5f, 0.5f, 2.0f), Vector3f(0.0f, 0.0f, -1.0f));

  EXPECT_TRUE(standard_bbox.intersect(ray1, 0.0f, 100.0f));
  EXPECT_TRUE(standard_bbox.intersect(ray2, 0.0f, 100.0f));
  EXPECT_TRUE(standard_bbox.intersect(ray3, 0.0f, 100.0f));
}

// Тест 15: Луч с нулевым направлением (должен обрабатываться корректно)
TEST_F(AABBIntersectTest, RayWithZeroDirection) {
  Ray ray(Vector3f(0.5f, 0.5f, 0.5f), Vector3f(0.0f, 0.0f, 0.0f));
  // Поведение зависит от реализации - может вернуть true или false
  // Главное - не должно быть crash
  EXPECT_NO_THROW(standard_bbox.intersect(ray, 0.0f, 100.0f));
}

// Тест 16: Бесконечный AABB (крайний случай)
TEST_F(AABBIntersectTest, InfiniteAABB) {
  AABB infinite_bbox(Vector3f(-std::numeric_limits<float>::max(),
                              -std::numeric_limits<float>::max(),
                              -std::numeric_limits<float>::max()),
                     Vector3f(std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max()));

  Ray ray(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 0.0f, 0.0f));
  EXPECT_TRUE(infinite_bbox.intersect(ray, 0.0f, 100.0f));
}

// Тест 17: Очень маленький t_range
TEST_F(AABBIntersectTest, DISABLED_VerySmallTRange) {
  Ray ray(Vector3f(0.5f, -1.0f, 0.5f), Vector3f(0.0f, 1.0f, 0.0f));

  // Пересечение происходит при t=1.0
  EXPECT_TRUE(standard_bbox.intersect(ray, 0.999f, 1.001f));
  EXPECT_FALSE(standard_bbox.intersect(ray, 1.001f, 2.0f));
  EXPECT_FALSE(standard_bbox.intersect(ray, 0.0f, 0.999f));
}

// Тест 18: AABB с нулевой шириной по одной оси (плоскость XY)
TEST_F(AABBIntersectTest, ZeroWidthZ_AABB) {
  AABB plane_bbox(Vector3f(0.0f, 0.0f, 0.5f), Vector3f(1.0f, 1.0f, 0.5f));

  // Луч перпендикулярный плоскости
  Ray ray1(Vector3f(0.5f, 0.5f, -1.0f), Vector3f(0.0f, 0.0f, 1.0f));
  Ray ray2(Vector3f(0.5f, 0.5f, 2.0f), Vector3f(0.0f, 0.0f, -1.0f));

  // Луч параллельный плоскости
  Ray ray3(Vector3f(0.5f, -1.0f, 0.5f), Vector3f(0.0f, 1.0f, 0.0f));
  Ray ray4(Vector3f(-1.0f, 0.5f, 0.5f), Vector3f(1.0f, 0.0f, 0.0f));

  // Луч под углом к плоскости
  Ray ray5(Vector3f(0.5f, 0.5f, -1.0f),
           Vector3f(0.1f, 0.1f, 1.0f).normalized());

  EXPECT_TRUE(plane_bbox.intersect(ray1, 0.0f, 100.0f));
  EXPECT_TRUE(plane_bbox.intersect(ray2, 0.0f, 100.0f));
  EXPECT_TRUE(plane_bbox.intersect(ray3, 0.0f, 100.0f));
  EXPECT_TRUE(plane_bbox.intersect(ray4, 0.0f, 100.0f));
  EXPECT_TRUE(plane_bbox.intersect(ray5, 0.0f, 100.0f));
}

// Тест 19: AABB с нулевой шириной по двум осям (линия по Z)
TEST_F(AABBIntersectTest, ZeroWidthXY_AABB) {
  AABB line_bbox(Vector3f(0.5f, 0.5f, 0.0f), Vector3f(0.5f, 0.5f, 1.0f));

  // Луч вдоль линии
  Ray ray1(Vector3f(0.5f, 0.5f, -1.0f), Vector3f(0.0f, 0.0f, 1.0f));
  Ray ray2(Vector3f(0.5f, 0.5f, 2.0f), Vector3f(0.0f, 0.0f, -1.0f));

  // Луч перпендикулярный линии
  Ray ray3(Vector3f(0.0f, 0.5f, 0.5f), Vector3f(1.0f, 0.0f, 0.0f));
  Ray ray4(Vector3f(0.5f, 0.0f, 0.5f), Vector3f(0.0f, 1.0f, 0.0f));

  // Луч под углом к линии
  Ray ray5(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f).normalized());

  EXPECT_TRUE(line_bbox.intersect(ray1, 0.0f, 100.0f));
  EXPECT_TRUE(line_bbox.intersect(ray2, 0.0f, 100.0f));
  EXPECT_TRUE(line_bbox.intersect(ray3, 0.0f, 100.0f));
  EXPECT_TRUE(line_bbox.intersect(ray4, 0.0f, 100.0f));
  EXPECT_TRUE(line_bbox.intersect(ray5, 0.0f, 100.0f));
}

// Тест 20: AABB с нулевой шириной по всем осям (точка)
TEST_F(AABBIntersectTest, ZeroWidthAll_AABB) {
  AABB point_bbox(Vector3f(0.5f, 0.5f, 0.5f), Vector3f(0.5f, 0.5f, 0.5f));

  // Лучи через точку в разных направлениях
  Ray ray1(Vector3f(0.5f, 0.5f, -1.0f), Vector3f(0.0f, 0.0f, 1.0f));
  Ray ray2(Vector3f(-1.0f, 0.5f, 0.5f), Vector3f(1.0f, 0.0f, 0.0f));
  Ray ray3(Vector3f(0.5f, -1.0f, 0.5f), Vector3f(0.0f, 1.0f, 0.0f));
  Ray ray4(Vector3f(0.0f, 0.0f, 0.0f), Vector3f(1.0f, 1.0f, 1.0f).normalized());

  // Лучи мимо точки
  Ray ray5(Vector3f(0.6f, 0.5f, -1.0f), Vector3f(0.0f, 0.0f, 1.0f));
  Ray ray6(Vector3f(0.5f, 0.6f, -1.0f), Vector3f(0.0f, 0.0f, 1.0f));

  EXPECT_TRUE(point_bbox.intersect(ray1, 0.0f, 100.0f));
  EXPECT_TRUE(point_bbox.intersect(ray2, 0.0f, 100.0f));
  EXPECT_TRUE(point_bbox.intersect(ray3, 0.0f, 100.0f));
  EXPECT_TRUE(point_bbox.intersect(ray4, 0.0f, 100.0f));
  EXPECT_FALSE(point_bbox.intersect(ray5, 0.0f, 100.0f));
  EXPECT_FALSE(point_bbox.intersect(ray6, 0.0f, 100.0f));
}

// Тест 21: AABB с нулевой шириной и отрицательными координатами
TEST_F(AABBIntersectTest, ZeroWidthNegativeCoords_AABB) {
  AABB negative_plane(Vector3f(-1.0f, -1.0f, 0.0f), Vector3f(1.0f, 1.0f, 0.0f));

  Ray ray1(Vector3f(0.0f, 0.0f, -1.0f), Vector3f(0.0f, 0.0f, 1.0f));
  Ray ray2(Vector3f(0.0f, 0.0f, 1.0f), Vector3f(0.0f, 0.0f, -1.0f));
  Ray ray3(Vector3f(-2.0f, 0.0f, 0.0f), Vector3f(1.0f, 0.0f, 0.0f));

  EXPECT_TRUE(negative_plane.intersect(ray1, 0.0f, 100.0f));
  EXPECT_TRUE(negative_plane.intersect(ray2, 0.0f, 100.0f));
  EXPECT_TRUE(negative_plane.intersect(ray3, 0.0f, 100.0f));
}

// Тест 22: AABB с очень маленькой (но не нулевой) шириной
TEST_F(AABBIntersectTest, VerySmallWidth_AABB) {
  const float epsilon = std::numeric_limits<float>::epsilon();
  AABB tiny_bbox(Vector3f(0.5f, 0.5f, 0.5f),
                 Vector3f(0.5f + epsilon, 0.5f + epsilon, 0.5f + epsilon));

  Ray ray1(Vector3f(0.5f, 0.5f, -1.0f), Vector3f(0.0f, 0.0f, 1.0f));
  Ray ray2(Vector3f(0.5f + 2 * epsilon, 0.5f, -1.0f),
           Vector3f(0.0f, 0.0f, 1.0f));

  EXPECT_TRUE(tiny_bbox.intersect(ray1, 0.0f, 100.0f));
  EXPECT_FALSE(tiny_bbox.intersect(ray2, 0.0f, 100.0f));
}

// Тест 23: Смешанные нулевые и ненулевые ширины
TEST_F(AABBIntersectTest, MixedZeroWidth_AABB) {
  // Нулевая ширина по X, нормальная по Y и Z
  AABB mixed_bbox1(Vector3f(0.5f, 0.0f, 0.0f), Vector3f(0.5f, 1.0f, 1.0f));

  // Нулевая ширина по Y, нормальная по X и Z
  AABB mixed_bbox2(Vector3f(0.0f, 0.5f, 0.0f), Vector3f(1.0f, 0.5f, 1.0f));

  Ray ray1(Vector3f(0.5f, 0.5f, 0.5f), Vector3f(1.0f, 0.0f, 0.0f));
  Ray ray2(Vector3f(0.5f, 0.5f, 0.5f), Vector3f(0.0f, 1.0f, 0.0f));
  Ray ray3(Vector3f(0.5f, 0.5f, 0.5f), Vector3f(0.0f, 0.0f, 1.0f));

  EXPECT_TRUE(mixed_bbox1.intersect(ray1, 0.0f, 100.0f));  // По X - попадание
  EXPECT_TRUE(mixed_bbox1.intersect(ray2, 0.0f, 100.0f));  // По Y - попадание
  EXPECT_TRUE(mixed_bbox1.intersect(ray3, 0.0f, 100.0f));  // По Z - попадание

  EXPECT_TRUE(mixed_bbox2.intersect(ray1, 0.0f, 100.0f));  // По X - попадание
  EXPECT_TRUE(mixed_bbox2.intersect(ray2, 0.0f, 100.0f));  // По Y - попадание
  EXPECT_TRUE(mixed_bbox2.intersect(ray3, 0.0f, 100.0f));  // По Z - попадание
}

// Тест 24: Луч параллельный плоскости нулевой ширины
TEST_F(AABBIntersectTest, RayParallelToZeroWidthPlane) {
  AABB plane_bbox(Vector3f(0.0f, 0.0f, 0.5f), Vector3f(1.0f, 1.0f, 0.5f));

  // Лучи параллельные плоскости
  Ray ray1(Vector3f(0.5f, -1.0f, 0.5f), Vector3f(0.0f, 1.0f, 0.0f));
  Ray ray2(Vector3f(-1.0f, 0.5f, 0.5f), Vector3f(1.0f, 0.0f, 0.0f));
  Ray ray3(Vector3f(0.5f, 0.5f, 0.6f),
           Vector3f(0.0f, 1.0f, 0.0f));  // Выше плоскости

  EXPECT_TRUE(plane_bbox.intersect(ray1, 0.0f, 100.0f));
  EXPECT_TRUE(plane_bbox.intersect(ray2, 0.0f, 100.0f));
  EXPECT_FALSE(
      plane_bbox.intersect(ray3, 0.0f, 100.0f));  // Не должен пересекать
}