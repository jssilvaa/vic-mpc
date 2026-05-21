#pragma once

#include <cassert>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>

namespace robot {

class FPSTracker {
 public:
  explicit FPSTracker(const std::string& name, double alpha = 0.1)
      : initialized_(false), sampleCount_(0), alpha_(alpha), fps_(0.0), name_(name) {
    assert(alpha > 0 && alpha <= 1);
    lastTimePoint_ = std::chrono::steady_clock::now();
  }

  ~FPSTracker() = default;

  void tick() {
    auto now = std::chrono::steady_clock::now();
    double deltaTime = std::chrono::duration<double>(now - lastTimePoint_).count();
    lastTimePoint_ = now;

    ++sampleCount_;
    double currentFPS = 1.0 / deltaTime;

    if (initialized_) {
      fps_ = alpha_ * currentFPS + (1.0 - alpha_) * fps_;
    } else {
      fps_ = currentFPS;
      initialized_ = true;
    }
  }

  void reset() { initialized_ = false; }

  void print() const { std::cerr << "FPS [" << name_ << "]: " << static_cast<int>(fps_) << std::endl; }

  double alpha() const { return alpha_; }
  double fps() const { return fps_; }
  size_t sampleCount() const { return sampleCount_; }

 private:
  bool initialized_;
  size_t sampleCount_;

  double alpha_;
  double fps_;

  std::chrono::time_point<std::chrono::steady_clock> lastTimePoint_;

  std::string name_;
};

}  // namespace robot
