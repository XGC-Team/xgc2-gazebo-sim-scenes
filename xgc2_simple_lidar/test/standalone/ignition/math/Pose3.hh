// Minimal ignition-math6 stand-in for compiling scan_projection.hpp without
// Gazebo. Quaternion construction, Euler/Normalize, Inverse, the Hamilton
// product and RotateVector repeat ignition-math 6 operation for operation, so
// a test that compares the projection with Pose3d arithmetic checks the same
// floating-point result here as against the library.
#pragma once

#include <cmath>

namespace ignition {
namespace math {

inline bool equal(double a, double b, double epsilon = 1e-6) {
    return std::abs(a - b) <= epsilon;
}

class Vector3d {
  public:
    Vector3d() = default;
    Vector3d(double x, double y, double z) : x_(x), y_(y), z_(z) {}
    double X() const { return x_; }
    double Y() const { return y_; }
    double Z() const { return z_; }
    Vector3d operator+(const Vector3d& other) const { return {x_ + other.x_, y_ + other.y_, z_ + other.z_}; }
    Vector3d operator-(const Vector3d& other) const { return {x_ - other.x_, y_ - other.y_, z_ - other.z_}; }
    Vector3d operator*(double scale) const { return {x_ * scale, y_ * scale, z_ * scale}; }
    double Distance(const Vector3d& other) const {
        const Vector3d d = *this - other;
        return std::sqrt(d.x_ * d.x_ + d.y_ * d.y_ + d.z_ * d.z_);
    }

  private:
    double x_{0}, y_{0}, z_{0};
};

class Quaterniond {
  public:
    Quaterniond() = default;
    Quaterniond(double w, double x, double y, double z) : w_(w), x_(x), y_(y), z_(z) {}
    Quaterniond(double roll, double pitch, double yaw) {
        const double phi = roll / 2.0, the = pitch / 2.0, psi = yaw / 2.0;
        w_ = std::cos(phi) * std::cos(the) * std::cos(psi) + std::sin(phi) * std::sin(the) * std::sin(psi);
        x_ = std::sin(phi) * std::cos(the) * std::cos(psi) - std::cos(phi) * std::sin(the) * std::sin(psi);
        y_ = std::cos(phi) * std::sin(the) * std::cos(psi) + std::sin(phi) * std::cos(the) * std::sin(psi);
        z_ = std::cos(phi) * std::cos(the) * std::sin(psi) - std::sin(phi) * std::sin(the) * std::cos(psi);
        Normalize();
    }
    double W() const { return w_; }
    double X() const { return x_; }
    double Y() const { return y_; }
    double Z() const { return z_; }
    void Normalize() {
        const double s = std::sqrt(w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_);
        if (equal(s, 0.0)) {
            w_ = 1.0;
            x_ = y_ = z_ = 0.0;
        } else {
            w_ /= s;
            x_ /= s;
            y_ /= s;
            z_ /= s;
        }
    }
    Quaterniond Inverse() const {
        Quaterniond q(w_, x_, y_, z_);
        const double s = q.w_ * q.w_ + q.x_ * q.x_ + q.y_ * q.y_ + q.z_ * q.z_;
        if (equal(s, 0.0)) {
            q.w_ = 1.0;
            q.x_ = q.y_ = q.z_ = 0.0;
        } else {
            q.w_ = q.w_ / s;
            q.x_ = -q.x_ / s;
            q.y_ = -q.y_ / s;
            q.z_ = -q.z_ / s;
        }
        return q;
    }
    Quaterniond operator*(const Quaterniond& q) const {
        return {w_ * q.w_ - x_ * q.x_ - y_ * q.y_ - z_ * q.z_, w_ * q.x_ + x_ * q.w_ + y_ * q.z_ - z_ * q.y_,
                w_ * q.y_ - x_ * q.z_ + y_ * q.w_ + z_ * q.x_, w_ * q.z_ + x_ * q.y_ - y_ * q.x_ + z_ * q.w_};
    }
    Vector3d RotateVector(const Vector3d& v) const {
        Quaterniond tmp(0.0, v.X(), v.Y(), v.Z());
        tmp = (*this) * (tmp * Inverse());
        return {tmp.x_, tmp.y_, tmp.z_};
    }

  private:
    double w_{1}, x_{0}, y_{0}, z_{0};
};

class Pose3d {
  public:
    Pose3d() = default;
    Pose3d(double x, double y, double z, double roll, double pitch, double yaw)
        : position_(x, y, z), rotation_(roll, pitch, yaw) {}
    Pose3d(const Vector3d& position, const Quaterniond& rotation) : position_(position), rotation_(rotation) {}
    const Vector3d& Pos() const { return position_; }
    const Quaterniond& Rot() const { return rotation_; }

  private:
    Vector3d position_;
    Quaterniond rotation_;
};

} // namespace math
} // namespace ignition
