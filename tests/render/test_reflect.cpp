#include <gtest/gtest.h>
#include <cmath>
#include "ray.h"
#include "reflect.h"

using namespace rtr;
using namespace Eigen;

class ReflectionRefractionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Стандартные векторы для тестов
    normal = Vector3f(0.0f, 1.0f, 0.0f).normalized();     // Вверх
    incident = Vector3f(1.0f, -1.0f, 0.0f).normalized();  // Под углом 45°
  }

  Vector3f normal;
  Vector3f incident;
};

// Отражение под разными углами
TEST_F(ReflectionRefractionTest, ReflectionVariousAngles) {
  // Перпендикулярное падение
  Vector3f reflected1 = reflect(Vector3f(0.0f, -1.0f, 0.0f), normal);
  EXPECT_TRUE(reflected1.isApprox(Vector3f(0.0f, 1.0f, 0.0f), 1e-5f));

  // Касательное падение
  Vector3f reflected2 = reflect(Vector3f(1.0f, 0.0f, 0.0f), normal);
  EXPECT_TRUE(reflected2.isApprox(Vector3f(1.0f, 0.0f, 0.0f), 1e-5f));

  // Под углом 45°
  Vector3f reflected3 = reflect(incident, normal);
  EXPECT_NEAR(reflected3.dot(normal), incident.dot(-normal),
              1e-5f);  // Угол сохранен
}

// Преломление под разными углами
TEST_F(ReflectionRefractionTest, DISABLED_RefractionVariousAngles) {
  // Перпендикулярное падение (воздух -> стекло)
  Vector3f refracted1 =
      refract(Vector3f(0.0f, -1.0f, 0.0f), normal, 1.0f / 1.5f);
  EXPECT_TRUE(refracted1.isApprox(Vector3f(0.0f, -1.0f, 0.0f),
                                  1e-5f));  // Направление сохранено

  // Полное внутреннее отражение
  Vector3f refracted2 =
      refract(Vector3f(0.0f, 1.0f, 0.0f), normal, 1.5f / 1.0f);
  EXPECT_NEAR(refracted2.norm(), 0, 1e-5f);  // Должен вернуть нулевой вектор
}

// Краевые случаи
TEST_F(ReflectionRefractionTest, DISABLED_EdgeCases) {
  // Нулевые векторы
  Vector3f result1 = reflect(Vector3f::Zero(), normal);
  Vector3f result2 = refract(Vector3f::Zero(), normal, 1.5f);
  EXPECT_NEAR(result1.norm(), 0, 1e-5f);
  EXPECT_NEAR(result2.norm(), 0, 1e-5f);

  // Нулевая нормаль
  Vector3f result3 = reflect(incident, Vector3f::Zero());
  Vector3f result4 = refract(incident, Vector3f::Zero(), 1.5f);
  EXPECT_NEAR(result3.norm(), 0, 1e-5f);
  EXPECT_NEAR(result4.norm(), 0, 1e-5f);

  // Коэффициент преломления 1.0
  Vector3f result5 = refract(incident, normal, 1.0f);
  EXPECT_NEAR(result5.norm(), 0, 1e-5f);
}

// Сохранение энергии (длина вектора)
TEST_F(ReflectionRefractionTest, DISABLED_EnergyConservation) {
  Vector3f reflected = reflect(incident, normal);
  Vector3f refracted = refract(incident, normal, 1.0f / 1.5f);

  EXPECT_NEAR(reflected.norm(), 1.0f, 1e-5f);  // Отражение сохраняет длину
  if (refracted.norm() > 1e-5f) {  // Если не полное внутреннее отражение
    EXPECT_NEAR(refracted.norm(), 1.0f, 1e-5f);  // Преломление тоже
  }
}