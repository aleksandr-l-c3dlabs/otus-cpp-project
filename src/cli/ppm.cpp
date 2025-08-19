#include "ppm.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rtr {

/// @brief Gamma correction
/// @param linear Linear color parameter
float toSRGB(float linear) {
  return (linear <= 0.0031308f)
             ? linear * 12.92f
             : 1.055f * std::pow(linear, 1.0f / 2.4f) - 0.055f;
}

void to_byte_color(const float color[3], uint8_t bytes[3]) {
  float r = std::clamp(color[0], 0.0f, 1.0f);
  float g = std::clamp(color[1], 0.0f, 1.0f);
  float b = std::clamp(color[2], 0.0f, 1.0f);

  bytes[0] = static_cast<uint8_t>(r * 255.0f + 0.5f);
  bytes[1] = static_cast<uint8_t>(g * 255.0f + 0.5f);
  bytes[2] = static_cast<uint8_t>(b * 255.0f + 0.5f);
}

void ppm_export(std::ostream& stream, const FrameBuffer& buffer) {
  stream << "P6\n"
         << buffer.get_width() << " " << buffer.get_height() << "\n255\n";

  uint8_t bytes[3];
  for (const auto& color : buffer) {
    to_byte_color(color, bytes);
    stream << bytes[0] << bytes[1] << bytes[2];
  }
}

}  // namespace rtr