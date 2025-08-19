#pragma once

#include <algorithm>
#include <eigen3/Eigen/Core>
#include <vector>

namespace rtr {

struct Image {
  int width, height;
  std::vector<unsigned char> data;

  Image(int w, int h, unsigned char* src)
      : width(w), height(h), data(src, src + w * h * 3) {}

  Eigen::Vector3f sample(float u, float v) const {
    u = u - floor(u);
    v = v - floor(v);

    int x = std::clamp(int(u * width), 0, width - 1);
    int y = std::clamp(int((1 - v) * height), 0, height - 1);  // Flip Y

    int idx = (y * width + x) * 3;
    return {data[idx] / 255.0f, data[idx + 1] / 255.0f, data[idx + 2] / 255.0f};
  }
};

}  // namespace rtr