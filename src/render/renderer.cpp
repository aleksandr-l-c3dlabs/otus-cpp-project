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

  ray_tracer_.add_light(
      {{2.0f, -4.0f, 3.0f}, {1.5f, 1.5f, 1.5f}});  // Key light
  ray_tracer_.add_light(
      {{-1.0f, -1.0f, 2.0f}, {0.7f, 0.7f, 0.7f}});  // Fill light
  ray_tracer_.add_light(
      {{0.5f, 2.0f, 2.5f}, {0.9f, 0.9f, 0.9f}});  // Back light
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