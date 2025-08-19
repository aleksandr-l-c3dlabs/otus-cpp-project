#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include "aabb.h"
#include "camera.h"
#include "framebuffer.h"
#include "model.h"
#include "raytracer.h"

namespace rtr {

class Renderer {
 public:
  using ProgressCallback = std::function<void(float)>;

  Renderer(std::shared_ptr<const Model> model,
           std::shared_ptr<const Camera> camera, size_t width, size_t height);

  void render(int num_threads = std::thread::hardware_concurrency(),
              ProgressCallback callback = nullptr);

  [[nodiscard]] const FrameBuffer& get_frame_buffer() const {
    return frame_buffer_;
  }
  [[nodiscard]] float get_progress() const { return progress_; }
  [[nodiscard]] AABB get_root_bbox() const {
    return ray_tracer_.get_root_bbox();
  };

 private:
  RayTracer ray_tracer_;
  FrameBuffer frame_buffer_;
  std::mutex progress_mutex_;
  float progress_;
};

}  // namespace rtr