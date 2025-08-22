#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include <memory>

#include "material.h"
#include "ray.h"
#include "raytracer.h"

using namespace rtr;
using namespace Eigen;

class TestRayTracer : public RayTracer {
 public:
  TestRayTracer(std::shared_ptr<const Model> model,
                std::shared_ptr<const Camera> camera,
                const Vector3f& bg_color = Vector3f(0.898f, 0.95687f, 1.0f))
      : RayTracer(model, camera, bg_color) {}
  ~TestRayTracer() = default;

  [[nodiscard]] Vector3f calculate(const HitRecord& rec) {
    return calculate_lighting(rec);
  }
};

class CalculateLightingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Создаем базовый материал
    material = std::make_shared<Material>();
    material->ambient = Vector3f(0.1f, 0.1f, 0.1f);
    material->diffuse = Vector3f(0.8f, 0.8f, 0.8f);
    material->specular = Vector3f(0.5f, 0.5f, 0.5f);
    material->shininess = 32.0f;

    // Базовая запись попадания
    hit.t = 1.0f;
    hit.point = Vector3f(0.0f, 0.0f, 0.0f);
    hit.normal = Vector3f(0.0f, 1.0f, 0.0f);  // Нормаль направлена вверх
    hit.material = material;
    hit.front_face = true;

    // Создаем RayTracer с пустыми зависимостями
    model = std::make_shared<const Model>();
    camera = std::make_shared<const Camera>();
    rayTracer = std::make_unique<TestRayTracer>(model, camera);
  }

  std::shared_ptr<Material> material;
  HitRecord hit;
  std::shared_ptr<const Model> model;
  std::shared_ptr<const Camera> camera;
  std::unique_ptr<TestRayTracer> rayTracer;
};

// Тест 1: Освещение без источников света (только ambient)
TEST_F(CalculateLightingTest, NoLightsReturnsAmbientOnly) {
  Vector3f result = rayTracer->calculate(hit);

  // Должен вернуться только ambient компонент
  EXPECT_FLOAT_EQ(result.x(), material->ambient.x());
  EXPECT_FLOAT_EQ(result.y(), material->ambient.y());
  EXPECT_FLOAT_EQ(result.z(), material->ambient.z());
}

// Тест 2: Один источник света прямо над поверхностью
TEST_F(CalculateLightingTest, SingleLightDirectlyAbove) {
  Light light;
  light.position = Vector3f(0.0f, 2.0f, 0.0f);  // Прямо над точкой
  light.intensity = Vector3f(1.0f, 1.0f, 1.0f);

  rayTracer->add_light(light);

  Vector3f result = rayTracer->calculate(hit);

  // Должен быть хороший diffuse компонент (нормаль и свет в одном направлении)
  EXPECT_GT(result.x(), material->ambient.x());
  EXPECT_GT(result.y(), material->ambient.y());
  EXPECT_GT(result.z(), material->ambient.z());

  // Проверяем, что результат в допустимом диапазоне
  EXPECT_GE(result.x(), 0.0f);
  EXPECT_GE(result.y(), 0.0f);
  EXPECT_GE(result.z(), 0.0f);
  EXPECT_LE(result.x(), 1.0f);
  EXPECT_LE(result.y(), 1.0f);
  EXPECT_LE(result.z(), 1.0f);
}

// Тест 3: Источник света под поверхностью (не должен влиять)
TEST_F(CalculateLightingTest, LightBelowSurfaceNoContribution) {
  Light light;
  light.position = Vector3f(0.0f, -2.0f, 0.0f);  // Под поверхностью
  light.intensity = Vector3f(1.0f, 1.0f, 1.0f);

  rayTracer->add_light(light);

  Vector3f result = rayTracer->calculate(hit);

  // Должен вернуться только ambient (свет под поверхностью не влияет)
  EXPECT_FLOAT_EQ(result.x(), material->ambient.x());
  EXPECT_FLOAT_EQ(result.y(), material->ambient.y());
  EXPECT_FLOAT_EQ(result.z(), material->ambient.z());
}

// Тест 4: Несколько источников света
TEST_F(CalculateLightingTest, MultipleLightsAdditive) {
  Light light1;
  light1.position = Vector3f(2.0f, 2.0f, 0.0f);
  light1.intensity = Vector3f(0.5f, 0.5f, 0.5f);

  Light light2;
  light2.position = Vector3f(-2.0f, 2.0f, 0.0f);
  light2.intensity = Vector3f(0.3f, 0.3f, 0.3f);

  rayTracer->add_light(light1);
  rayTracer->add_light(light2);

  Vector3f result = rayTracer->calculate(hit);

  // Должен быть больше чем только ambient
  EXPECT_GT(result.x(), material->ambient.x());
  EXPECT_GT(result.y(), material->ambient.y());
  EXPECT_GT(result.z(), material->ambient.z());
}

// Тест 5: Источник света с разными цветовыми компонентами
TEST_F(CalculateLightingTest, ColoredLightAffectsResult) {
  Light light;
  light.position = Vector3f(0.0f, 2.0f, 0.0f);
  light.intensity = Vector3f(1.0f, 0.5f, 0.2f);  // Красный доминирует

  rayTracer->add_light(light);

  Vector3f result = rayTracer->calculate(hit);

  // Красная компонента должна быть самой сильной
  EXPECT_GT(result.x(), result.y());
  EXPECT_GT(result.x(), result.z());

  // Синяя компонента должна быть самой слабой
  EXPECT_LT(result.z(), result.x());
  EXPECT_LT(result.z(), result.y());
}

// Тест 6: Проверка затухания с расстоянием
TEST_F(CalculateLightingTest, DistanceAttenuation) {
  Light closeLight;
  closeLight.position = Vector3f(0.0f, 1.0f, 0.0f);  // Близко
  closeLight.intensity = Vector3f(1.0f, 1.0f, 1.0f);

  Light farLight;
  farLight.position = Vector3f(0.0f, 10.0f, 0.0f);  // Далеко
  farLight.intensity = Vector3f(1.0f, 1.0f, 1.0f);

  rayTracer->add_light(closeLight);
  Vector3f closeResult = rayTracer->calculate(hit);

  rayTracer->remove_light(closeLight);
  rayTracer->add_light(farLight);
  Vector3f farResult = rayTracer->calculate(hit);

  // Близкий источник должен давать больше света
  EXPECT_GT(closeResult.norm(), farResult.norm());
}

// Тест 7: Проверка specular отражения
TEST_F(CalculateLightingTest, SpecularReflectionPresent) {
  // Настраиваем материал для хорошего specular отражения
  material->specular = Vector3f(1.0f, 1.0f, 1.0f);
  material->shininess = 128.0f;

  // Размещаем свет так, чтобы получить specular отражение
  Light light;
  light.position = Vector3f(2.0f, 2.0f, 2.0f);
  light.intensity = Vector3f(1.0f, 1.0f, 1.0f);

  rayTracer->add_light(light);

  Vector3f result = rayTracer->calculate(hit);

  // Должен быть заметный вклад specular
  EXPECT_GT(result.norm(), material->ambient.norm() + 0.1f);
}

// Тест 8: Нормаль, направленная от света
TEST_F(CalculateLightingTest, NormalAwayFromLightMinimalContribution) {
  // Поворачиваем нормаль от света
  hit.normal = Vector3f(0.0f, -1.0f, 0.0f);

  Light light;
  light.position = Vector3f(0.0f, 2.0f, 0.0f);
  light.intensity = Vector3f(1.0f, 1.0f, 1.0f);

  rayTracer->add_light(light);

  Vector3f result = rayTracer->calculate(hit);

  // Должен быть только ambient (нормаль направлена от света)
  EXPECT_NEAR(result.x(), material->ambient.x(), 0.001f);
  EXPECT_NEAR(result.y(), material->ambient.y(), 0.001f);
  EXPECT_NEAR(result.z(), material->ambient.z(), 0.001f);
}

// Тест 9: Материал с нулевым diffuse (только ambient + specular)
TEST_F(CalculateLightingTest, ZeroDiffuseMaterial) {
  material->diffuse = Vector3f::Zero();
  material->specular = Vector3f(1.0f, 1.0f, 1.0f);

  Light light;
  light.position = Vector3f(2.0f, 2.0f, 0.0f);
  light.intensity = Vector3f(1.0f, 1.0f, 1.0f);

  rayTracer->add_light(light);

  Vector3f result = rayTracer->calculate(hit);

  // Должен быть ambient и возможно specular, но не diffuse
  EXPECT_GE(result.x(), material->ambient.x());
  EXPECT_GE(result.y(), material->ambient.y());
  EXPECT_GE(result.z(), material->ambient.z());
}

// Тест 10: Очень тусклый свет
TEST_F(CalculateLightingTest, VeryDimLight) {
  Light light;
  light.position = Vector3f(0.0f, 2.0f, 0.0f);
  light.intensity = Vector3f(0.01f, 0.01f, 0.01f);  // Очень тусклый

  rayTracer->add_light(light);

  Vector3f result = rayTracer->calculate(hit);

  // Результат должен быть чуть больше ambient
  EXPECT_GT(result.x(), material->ambient.x());
  EXPECT_GT(result.y(), material->ambient.y());
  EXPECT_GT(result.z(), material->ambient.z());

  // Но все еще очень близко к ambient
  EXPECT_LT(result.x(), material->ambient.x() + 0.1f);
  EXPECT_LT(result.y(), material->ambient.y() + 0.1f);
  EXPECT_LT(result.z(), material->ambient.z() + 0.1f);
}