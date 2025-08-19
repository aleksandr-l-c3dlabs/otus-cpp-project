#include "camera.h"

#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <numbers>

namespace rtr {

using Vector3f = Eigen::Vector3f;
using Matrix3f = Eigen::Matrix3f;
using Quaternionf = Eigen::Quaternionf;

Camera::Camera(const Vector3f& position, const Vector3f& look_at,
               const Vector3f& up, float fov, float aspect_ratio,
               float near_plane, float far_plane)
    : position_(position),
      look_at_(look_at),
      world_up_(up),
      fov_(fov),
      aspect_ratio_(aspect_ratio),
      near_(near_plane),
      far_(far_plane),
      yaw_(-90.0f),
      pitch_(0.0f) {
  update_vectors();
}

Vector3f Camera::generate_ray(float u, float v) const {
  float x = (2.0f * u - 1.0f) * aspect_ratio_ * scale_;
  float y = (1.0f - 2.0f * v) * scale_;
  Vector3f ray_direction = x * right_ + y * up_ + forward_;
  return ray_direction.normalized();
}

void Camera::move_forward(float distance) {
  position_ += forward_ * distance;
  look_at_ = position_ + forward_;
}

void Camera::move_backward(float distance) {
  move_forward(-distance);
}

void Camera::move_right(float distance) {
  position_ += right_ * distance;
  look_at_ = position_ + forward_;
}

void Camera::move_left(float distance) {
  move_right(-distance);
}

void Camera::move_up(float distance) {
  position_ += up_ * distance;
  look_at_ = position_ + forward_;
}

void Camera::move_down(float distance) {
  move_up(-distance);
}

void Camera::rotate(float yaw_offset, float pitch_offset) {
  yaw_ += yaw_offset;
  pitch_ += pitch_offset;

  pitch_ = std::clamp(pitch_, -89.0f, 89.0f);

  Vector3f new_forward;
  new_forward.x() = cos(yaw_ * std::numbers::pi_v<float> / 180.0f) *
                    cos(pitch_ * std::numbers::pi_v<float> / 180.0f);
  new_forward.y() = sin(pitch_ * std::numbers::pi_v<float> / 180.0f);
  new_forward.z() = sin(yaw_ * std::numbers::pi_v<float> / 180.0f) *
                    cos(pitch_ * std::numbers::pi_v<float> / 180.0f);

  forward_ = new_forward.normalized();
  look_at_ = position_ + forward_;

  right_ = forward_.cross(world_up_).normalized();
  up_ = right_.cross(forward_).normalized();
}

void Camera::rotate_around_point(const Vector3f& point, float angle_degrees,
                                 const Vector3f& axis) {
  Quaternionf q = Quaternionf(Eigen::AngleAxisf(
      angle_degrees * std::numbers::pi_v<float> / 180.0f, axis.normalized()));

  Vector3f dir = position_ - point;
  dir = q * dir;
  position_ = point + dir;

  forward_ = (q * forward_).normalized();
  look_at_ = position_ + forward_;

  update_vectors();
}

void Camera::zoom_to_fit(const AABB& bbox) {
  Vector3f forward = get_forward();
  Vector3f up = get_up();
  Vector3f center = (bbox.min + bbox.max) * 0.5f;
  Vector3f size = bbox.max - bbox.min;

  float max_dim = std::max({size.x(), size.y(), size.z()});
  float fov_rad = fov_ * std::numbers::pi_v<float> / 180.0f;
  float distance = (max_dim * 0.5f) / std::tan(fov_rad * 0.5f) * 1.5f;

  Vector3f new_position = center - forward * distance;
  set_position(new_position);
}

void Camera::set_position(const Vector3f& position) {
  position_ = position;
  look_at_ = position_ + forward_;
}

void Camera::set_look_at(const Vector3f& look_at) {
  look_at_ = look_at;
  update_vectors();
}

void Camera::set_up(const Vector3f& up) {
  world_up_ = up;
  update_vectors();
}

void Camera::set_fov(float fov) {
  fov_ = fov;
  update_vectors();
}

void Camera::set_aspect_ratio(float aspect_ratio) {
  aspect_ratio_ = aspect_ratio;
  update_vectors();
}

void Camera::update_vectors() {
  if ((look_at_ - position_).squaredNorm() <
      std::numeric_limits<float>::epsilon()) {
    look_at_ = position_ +
               (look_at_.isZero() ? Vector3f(0.0f, 0.0f, -1.0f) : look_at_);
  }

  // If look_at was changed directly, update yaw and pitch
  if (!forward_.isApprox((look_at_ - position_).normalized())) {
    forward_ = (look_at_ - position_).normalized();

    pitch_ = asin(forward_.y()) * 180.0f / std::numbers::pi_v<float>;
    yaw_ =
        atan2(forward_.z(), forward_.x()) * 180.0f / std::numbers::pi_v<float>;
  }

  right_ = forward_.cross(world_up_).normalized();
  up_ = right_.cross(forward_).normalized();

  scale_ = std::tan(fov_ * 0.5f * std::numbers::pi_v<float> / 180.0f);
}

}  // namespace rtr