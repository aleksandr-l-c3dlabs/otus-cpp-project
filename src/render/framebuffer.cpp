#include "framebuffer.h"

namespace rtr {

FrameBuffer::FrameBuffer() : width_(0), height_(0), buffer_(nullptr) {}

FrameBuffer::FrameBuffer(size_t width, size_t height, float def)
    : width_(width), height_(height) {
  buffer_ = new float[3 * width * height]{def};
}

FrameBuffer::~FrameBuffer() {
  if (buffer_ != nullptr) {
    delete[] buffer_;
  }
}

void FrameBuffer::set_point(size_t x, size_t y, const vec3& color) {
  if (x < width_ && y < height_) {
    auto offset = 3 * (x + y * width_);
    for (size_t i = 0; i < 3; i++) {
      buffer_[offset + i] = color[i];
    }
  }
}

}  // namespace rtr