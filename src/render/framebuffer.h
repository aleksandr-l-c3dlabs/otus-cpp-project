#pragma once

#include <cstddef>
#include <iterator>

namespace rtr {

using vec3 = float[3];

class FrameBuffer {
 public:
  FrameBuffer();
  FrameBuffer(size_t width, size_t height, float def = 0);
  ~FrameBuffer();

  size_t get_width() const { return width_; }
  size_t get_height() const { return height_; }
  void set_point(size_t x, size_t y, const vec3& color);

 public:
  class Iterator {
   private:
    float* ptr;

   public:
    using iterator_category = std::input_iterator_tag;
    using value_type = vec3;
    using difference_type = std::ptrdiff_t;
    using pointer = vec3*;
    using reference = vec3&;

    Iterator(float* p) : ptr(p) {}

    reference operator*() const { return *reinterpret_cast<vec3*>(ptr); }

    Iterator& operator++() {
      ptr += 3;
      return *this;
    }

    Iterator operator++(int) {
      Iterator tmp = *this;
      ptr += 3;
      return tmp;
    }

    bool operator==(const Iterator& other) const { return ptr == other.ptr; }

    bool operator!=(const Iterator& other) const { return ptr != other.ptr; }
  };

  Iterator begin() const { return Iterator(buffer_); }

  Iterator end() const { return Iterator(buffer_ + 3 * width_ * height_); }

 private:
  size_t width_;
  size_t height_;
  float* buffer_;
};

}  // namespace rtr