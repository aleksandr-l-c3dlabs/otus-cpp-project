#include "renderer.h"

#include <atomic>
#include <vector>

namespace rtr {

constexpr size_t tile_size = 32;
constexpr float pixel_bias = 0.5f;

Renderer::Renderer(std::shared_ptr<const Model> model,
                   std::shared_ptr<const Camera> camera, size_t width,
                   size_t height)
    : ray_tracer_(model, camera),
      frame_buffer_(width, height),
      progress_(0.0f) {
  ray_tracer_.build_bvh();

  // automatic lights

  auto bbox = ray_tracer_.get_root_bbox();
  Vector3f center = (bbox.min + bbox.max) * 0.5f;
  Vector3f size = bbox.max - bbox.min;
  float max_dim = std::max({size.x(), size.y(), size.z()});
  float base_intensity = max_dim * 5.0f;

  // 1. Key Light (основной)
  ray_tracer_.add_light(
      {center + Vector3f(max_dim * 2.0f, max_dim * 1.5f, -max_dim * 2.0f),
       Vector3f(base_intensity, base_intensity, base_intensity * 0.9f)});

  // 2. Fill Light (заполняющий)
  ray_tracer_.add_light(
      {center + Vector3f(-max_dim * 1.5f, max_dim * 0.5f, -max_dim * 1.5f),
       Vector3f(base_intensity * 0.4f, base_intensity * 0.4f,
                base_intensity * 0.5f)});

  // 3. Rim Light (контровой)
  ray_tracer_.add_light(
      {center + Vector3f(0.0f, max_dim * 1.2f, max_dim * 2.0f),
       Vector3f(base_intensity * 0.1f, base_intensity * 0.1f,
                base_intensity * 0.2f)});
}

void Renderer::render(int num_threads, ProgressCallback callback) {
  // Tiling
  std::vector<std::pair<size_t, size_t>> tiles;
  for (int y = 0; y < frame_buffer_.get_height(); y += tile_size) {
    for (int x = 0; x < frame_buffer_.get_width(); x += tile_size) {
      tiles.emplace_back(x, y);
    }
  }

  // Progress
  std::atomic<int> tiles_completed{0};
  const int total_tiles = tiles.size();
  progress_ = 0.0f;

  // thread function
  auto render_tile = [&](size_t start_x, size_t start_y) {
    const int end_x = std::min(start_x + tile_size, frame_buffer_.get_width());
    const int end_y = std::min(start_y + tile_size, frame_buffer_.get_height());

    for (int y = start_y; y < end_y; ++y) {
      for (int x = start_x; x < end_x; ++x) {
        float u = (x + pixel_bias) / frame_buffer_.get_width();
        float v = (y + pixel_bias) / frame_buffer_.get_height();

        auto pixel = ray_tracer_.trace_pixel(u, v);
        frame_buffer_.set_point(x, y, {pixel[0], pixel[1], pixel[2]});
      }
    }

    int completed = ++tiles_completed;
    if (callback) {
      std::lock_guard<std::mutex> lock(progress_mutex_);
      progress_ = static_cast<float>(completed) / total_tiles;
      callback(progress_);
    }
  };

  // thread parallel
  std::vector<std::thread> threads;
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([&, i]() {
      for (int tile_idx = i; tile_idx < total_tiles; tile_idx += num_threads) {
        const auto& [x, y] = tiles[tile_idx];
        render_tile(x, y);
      }
    });
  }

  for (auto& thread : threads) {
    thread.join();
  }

  progress_ = 1.0f;
  if (callback) {
    callback(1.0f);
  }
}

}  // namespace rtr