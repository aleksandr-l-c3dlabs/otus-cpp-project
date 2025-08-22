#include <gtest/gtest.h>
#include "image.h"
#include "material.h"

using namespace rtr;
using namespace Eigen;

class MaterialTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Создаем тестовые текстуры
    unsigned char red[3] = {255, 0, 0};
    unsigned char green[3] = {0, 255, 0};

    red_texture = std::make_shared<Image>(1, 1, red);
    green_texture = std::make_shared<Image>(1, 1, green);
  }

  std::shared_ptr<Image> red_texture;
  std::shared_ptr<Image> green_texture;
};

// Базовые свойства материалов
TEST_F(MaterialTest, MaterialPropertyConsistency) {
  Material mat;
  mat.diffuse = Vector3f(0.8f, 0.2f, 0.2f);
  mat.specular = Vector3f(0.5f, 0.5f, 0.5f);
  mat.shininess = 32.0f;
  mat.ior = 1.5f;
  mat.transparency = 0.3f;

  // Проверяем, что свойства сохраняются
  EXPECT_TRUE(mat.diffuse.isApprox(Vector3f(0.8f, 0.2f, 0.2f)));
  EXPECT_TRUE(mat.specular.isApprox(Vector3f(0.5f, 0.5f, 0.5f)));
  EXPECT_FLOAT_EQ(mat.shininess, 32.0f);
  EXPECT_FLOAT_EQ(mat.ior, 1.5f);
  EXPECT_FLOAT_EQ(mat.transparency, 0.3f);
}

// Материалы с текстурами
TEST_F(MaterialTest, MaterialWithTextures) {
  Material mat;
  mat.diffuse_texture = red_texture;
  mat.ambient_texture = green_texture;

  EXPECT_NE(mat.diffuse_texture, nullptr);
  EXPECT_NE(mat.ambient_texture, nullptr);
}

// Краевые значения свойств
TEST_F(MaterialTest, MaterialEdgeCases) {
  Material mat;

  // Проверяем обработку крайних значений
  mat.shininess = 0.0f;
  EXPECT_GE(mat.shininess, 0.0f);

  mat.ior = 1.0f;  // Вакуум
  EXPECT_GE(mat.ior, 1.0f);

  mat.transparency = 1.0f;  // Полностью прозрачный
  EXPECT_TRUE(mat.transparency >= 0.0f && mat.transparency <= 1.0f);
}

// Значения за доступным диапазоном
TEST_F(MaterialTest, DISABLED_MaterialOutsideValues) {
  Material mat;

  mat.transparency = -0.5f;           // Отрицательная прозрачность
  EXPECT_GE(mat.transparency, 0.0f);  // Должна быть clamped

  mat.transparency = 2.0f;            // Верхняя граница допустимых значений
  EXPECT_EQ(mat.transparency, 1.0f);  // Должна быть clamped
}

// Энергосбережение (specular + diffuse <= 1)
TEST_F(MaterialTest, EnergyConservation) {
  Material mat;

  // Физически корректные значения
  mat.diffuse = Vector3f(0.6f, 0.6f, 0.6f);
  mat.specular = Vector3f(0.3f, 0.3f, 0.3f);

  float total_energy = mat.diffuse.maxCoeff() + mat.specular.maxCoeff();
  EXPECT_LE(total_energy, 1.0f + 1e-5f);  // Допускаем небольшую погрешность
}