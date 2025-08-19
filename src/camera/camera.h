#pragma once

#include <eigen3/Eigen/Core>

namespace rtr {

class Camera {
 public:
  using Vector3f = Eigen::Vector3f;

  Camera(const Vector3f& position = Vector3f::Zero(),
         const Vector3f& look_at = Vector3f(0.0f, 0.0f, -1.0f),
         const Vector3f& up = Vector3f(0.0f, 1.0f, 0.0f), float fov = 60.0f,
         float aspect_ratio = 16.0f / 9.0f, float near_plane = 0.1f,
         float far_plane = 1000.0f);

  [[nodiscard]] Vector3f generate_ray(float u, float v) const;

  void move_forward(float distance);
  void move_backward(float distance);
  void move_right(float distance);
  void move_left(float distance);
  void move_up(float distance);
  void move_down(float distance);

  void rotate(float yaw_offset, float pitch_offset);
  void rotate_around_point(const Vector3f& point, float angle_degrees,
                           const Vector3f& axis);
  void zoom_to_fit();

  void set_position(const Vector3f& position);
  void set_look_at(const Vector3f& look_at);
  void set_up(const Vector3f& up);
  void set_fov(float fov);
  void set_aspect_ratio(float aspect_ratio);

  [[nodiscard]] const Vector3f& get_position() const { return position_; }
  [[nodiscard]] const Vector3f& get_forward() const { return forward_; }
  [[nodiscard]] const Vector3f& get_right() const { return right_; }
  [[nodiscard]] const Vector3f& get_up() const { return up_; }
  [[nodiscard]] float get_yaw() const { return yaw_; }
  [[nodiscard]] float get_pitch() const { return pitch_; }

 private:
  void update_vectors();

  Vector3f position_;
  Vector3f look_at_;
  Vector3f world_up_;
  Vector3f forward_ = Vector3f::UnitZ();
  Vector3f right_ = Vector3f::UnitX();
  Vector3f up_ = Vector3f::UnitY();

  // Euler angles
  float yaw_;
  float pitch_;

  float fov_;
  float aspect_ratio_;
  float near_;
  float far_;
  float scale_;
};

}  // namespace rtr