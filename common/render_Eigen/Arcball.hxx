////////////////////////////////////////////////////////////////////
//
// $Id: Arcball.hxx 2025/07/05 14:35:01 kanai Exp 
//
// Copyright (c) Takashi Kanai
// Released under the MIT license
//
////////////////////////////////////////////////////////////////////

#ifndef _ARCBALL_HXX
#define _ARCBALL_HXX 1

#include "envDep.h"
#include "mydef.h"

#include <cmath>

#include "myEigen.hxx"

class Arcball {
 public:
  // Constants for magic numbers
  static constexpr float DEFAULT_WHEEL_SCALE = 0.1f;
  static constexpr float MOVE_SENSITIVITY = 200.0f;
  static constexpr float ZOOM_SENSITIVITY = 0.01f;
  static constexpr float MAG_OBJECT_SCALE = 2.0f;

  Arcball() { init(); };
  ~Arcball() = default;

  void init() {
    const Eigen::Quaternionf qOne = Eigen::Quaternionf::Identity();
    center_ = qNow_ = qDown_ = qOne;
    mNow_ = Eigen::Matrix4f::Identity();
    mag_ = 1.0f;
    magObject_ = 1.0f;
    wheelScale_ = DEFAULT_WHEEL_SCALE;
    seezo_ = 0.0f;
    scrn_x_ = scrn_y_ = 0;
    offset_ = Eigen::Vector3f::Zero();
  };

  Eigen::Quaternionf mouse_on_sphere(const int x, const int y, const int x0,
                                     const int y0) const {
    Eigen::Quaternionf sphere(
        0.0f, static_cast<float>(x - x0) / static_cast<float>(radius_),
        -static_cast<float>(y - y0) / static_cast<float>(radius_), 0.0f);
    const float mag = sphere.x() * sphere.x() + sphere.y() * sphere.y();
    if (mag > 1.0f) {
      const float scale = 1.0f / std::sqrt(mag);
      sphere.x() *= scale;
      sphere.y() *= scale;
    } else {
      sphere.z() = std::sqrt(1.0f - mag);
    }
    return sphere;
  };

  void setDrag() {
    qDrag_ = Eigen::Quaternionf(
        vFrom_.x() * vTo_.x() + vFrom_.y() * vTo_.y() + vFrom_.z() * vTo_.z(),
        vFrom_.y() * vTo_.z() - vFrom_.z() * vTo_.y(),
        vFrom_.z() * vTo_.x() - vFrom_.x() * vTo_.z(),
        vFrom_.x() * vTo_.y() - vFrom_.y() * vTo_.x());
  };

  void setArc() {
    const float s =
        std::sqrt(qDrag_.x() * qDrag_.x() + qDrag_.y() * qDrag_.y());
    if (s == 0.0f) {
      vrFrom_ = Eigen::Quaternionf(0.0f, 0.0f, 1.0f, 0.0f);
    } else {
      vrFrom_ = Eigen::Quaternionf(0.0f, -qDrag_.y() / s, qDrag_.x() / s, 0.0f);
    }
    vrTo_ = Eigen::Quaternionf(
        vrTo_.w(), qDrag_.w() * vrFrom_.x() - qDrag_.z() * vrFrom_.y(),
        qDrag_.w() * vrFrom_.y() + qDrag_.z() * vrFrom_.x(),
        qDrag_.x() * vrFrom_.y() - qDrag_.y() * vrFrom_.x());
    if (qDrag_.w() < 0.0f) {
      vrFrom_ = Eigen::Quaternionf(0.0f, -vrFrom_.x(), -vrFrom_.y(), 0.0f);
    }
  };

  // Getters and setters with proper const qualification
  void radius(const float r) { radius_ = r; }
  float radius() const { return radius_; }

  void vFrom(const Eigen::Quaternionf& vf) { vFrom_ = vf; }
  void vTo(const Eigen::Quaternionf& vt) { vTo_ = vt; }
  void qDown(const Eigen::Quaternionf& q) { qDown_ = q; }
  void mDown(const Eigen::Matrix4f& m) { mDown_ = m; }

  const Eigen::Quaternionf& center() const { return center_; }
  const Eigen::Quaternionf& qNow() const { return qNow_; }
  const Eigen::Quaternionf& qDown() const { return qDown_; }
  const Eigen::Quaternionf& qDrag() const { return qDrag_; }
  const Eigen::Quaternionf& vFrom() const { return vFrom_; }
  const Eigen::Quaternionf& vTo() const { return vTo_; }
  const Eigen::Quaternionf& vrFrom() const { return vrFrom_; }
  const Eigen::Quaternionf& vrTo() const { return vrTo_; }
  const Eigen::Matrix4f& mNow() const { return mNow_; }
  const Eigen::Matrix4f& mDown() const { return mDown_; }

  void setOffset(const Eigen::Vector3f& p) { offset_ = p; };

  void setMagObject(const float f) { magObject_ = f / MAG_OBJECT_SCALE; };

  const Eigen::Vector3f& offset() const { return offset_; };
  float seezo() const { return seezo_; };
  void setSeezo(const float z) { seezo_ = z; };

  void setScrnXY(const int x, const int y) {
    scrn_x_ = x;
    scrn_y_ = y;
  };
  int scrn_x() const { return scrn_x_; };
  int scrn_y() const { return scrn_y_; };

  void setHalfWHL(const int w, const int h) {
    halfW_ = w;
    halfH_ = h;
    radius_ = std::sqrt(static_cast<float>(halfW_ * halfW_ + halfH_ * halfH_));
  };
  int halfW() const { return halfW_; };
  int halfH() const { return halfH_; };

  void updateRotate(const int dx, const int dy) {
    setDrag();
    qNow_ = qDrag_ * qDown_;
    setArc();
    Eigen::Quaternionf qc = qNow();
    qc.conjugate();
    mNow_.block(0, 0, 3, 3) = qc.toRotationMatrix();
  };

  void updateMove(const int x, const int y, const int ox, const int oy) {
    const float dx =
        static_cast<float>(x - ox) / MOVE_SENSITIVITY / mag_ * magObject_;
    const float dy =
        static_cast<float>(-y + oy) / MOVE_SENSITIVITY / mag_ * magObject_;
    const Eigen::Vector3f p = mNow_.block<3, 2>(0, 0) * Eigen::Vector2f(dx, dy);
    offset_ -= p;
  };

  void updateZoom(const int x, const int y, const int ox, const int oy) {
    seezo_ += static_cast<float>(x - ox) * ZOOM_SENSITIVITY / mag_ * magObject_;
  };

  void updateWheelZoom(const float x) { seezo_ -= wheelScale_ * x; };

 private:
  Eigen::Quaternionf center_;
  float radius_;
  Eigen::Quaternionf qNow_, qDown_, qDrag_;
  Eigen::Quaternionf vFrom_, vTo_, vrFrom_, vrTo_;
  Eigen::Matrix4f mNow_, mDown_;

  Eigen::Vector3f offset_;
  float mag_;
  float magObject_;
  float wheelScale_;
  float seezo_;

  int scrn_x_, scrn_y_;  // screen coordinates
  int halfW_, halfH_;    // half size of window
};

#endif  // _ARCBALL_HXX
