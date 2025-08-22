#include <gtest/gtest.h>
#include "image.h"

using namespace rtr;
using namespace Eigen;

class ImageTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Создаем тестовое изображение 2x2
    unsigned char data[12] = {
        255, 0,   0,    // красный
        0,   255, 0,    // зеленый
        0,   0,   255,  // синий
        255, 255, 255   // белый
    };
    test_image = std::make_unique<Image>(2, 2, data);
  }

  std::unique_ptr<Image> test_image;
};

// Базовый sampling
TEST_F(ImageTest, BasicSampling) {
  // Центр пикселя (0,0) - синий
  Vector3f color = test_image->sample(0.25f, 0.25f);
  EXPECT_LT(color.x(), 0.1f);  // R ~0.0
  EXPECT_LT(color.y(), 0.1f);  // G ~0.0
  EXPECT_GT(color.z(), 0.9f);  // B ~1.0
}

// Sampling за границами (должен wrap around)
TEST_F(ImageTest, OutOfBoundsSampling) {
  // Координаты за пределами [0,1]
  Vector3f color1 = test_image->sample(1.25f, 0.25f);   // x > 1
  Vector3f color2 = test_image->sample(-0.25f, 0.25f);  // x < 0
  Vector3f color3 = test_image->sample(0.25f, 1.25f);   // y > 1
  Vector3f color4 = test_image->sample(0.25f, -0.25f);  // y < 0

  // Все должны вернуть валидные цвета (wrap around)
  EXPECT_TRUE(color1.x() >= 0.0f && color1.x() <= 1.0f);
  EXPECT_TRUE(color2.x() >= 0.0f && color2.x() <= 1.0f);
  EXPECT_TRUE(color3.x() >= 0.0f && color3.x() <= 1.0f);
  EXPECT_TRUE(color4.x() >= 0.0f && color4.x() <= 1.0f);
}

// Граничные значения
TEST_F(ImageTest, BoundarySampling) {
  // Точно на границах
  Vector3f color1 = test_image->sample(0.0f, 0.0f);
  Vector3f color2 = test_image->sample(1.0f, 1.0f);
  Vector3f color3 = test_image->sample(0.999f, 0.999f);

  EXPECT_TRUE(color1.x() >= 0.0f && color1.x() <= 1.0f);
  EXPECT_TRUE(color2.x() >= 0.0f && color2.x() <= 1.0f);
  EXPECT_TRUE(color3.x() >= 0.0f && color3.x() <= 1.0f);
}

// Особые случаи координат
TEST_F(ImageTest, SpecialCoordinateValues) {
  // NaN, Infinity, очень большие/малые значения
  Vector3f color1 = test_image->sample(NAN, 0.5f);
  Vector3f color2 = test_image->sample(INFINITY, 0.5f);
  Vector3f color3 = test_image->sample(-INFINITY, 0.5f);
  Vector3f color4 = test_image->sample(1e10f, 0.5f);
  Vector3f color5 = test_image->sample(-1e10f, 0.5f);

  // Должны возвращать валидные цвета (защита от крешей)
  for (const auto& color : {color1, color2, color3, color4, color5}) {
    EXPECT_TRUE(color.x() >= 0.0f && color.x() <= 1.0f);
    EXPECT_TRUE(color.y() >= 0.0f && color.y() <= 1.0f);
    EXPECT_TRUE(color.z() >= 0.0f && color.z() <= 1.0f);
  }
}