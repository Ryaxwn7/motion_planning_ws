#include "felzenszwalb_edt_2d.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace graph_planner {

namespace {

constexpr double kLarge = 1e20;
const double kInf = std::numeric_limits<double>::infinity();

}  // namespace

FelzenszwalbEdt2D::FelzenszwalbEdt2D()
    : width_(0), height_(0), size_(0), resolution_(0.0), initialized_(false) {}

void FelzenszwalbEdt2D::reset(const int width, const int height, const double resolution) {
  width_ = std::max(0, width);
  height_ = std::max(0, height);
  size_ = width_ * height_;
  resolution_ = std::max(0.0, resolution);
  initialized_ = false;
  occupancy_.assign(static_cast<std::size_t>(std::max(0, size_)), 0);
  distance_map_.assign(static_cast<std::size_t>(std::max(0, size_)), kInf);
}

bool FelzenszwalbEdt2D::initialized() const { return initialized_; }
int FelzenszwalbEdt2D::width() const { return width_; }
int FelzenszwalbEdt2D::height() const { return height_; }
int FelzenszwalbEdt2D::size() const { return size_; }
double FelzenszwalbEdt2D::resolution() const { return resolution_; }

const std::vector<uint8_t>& FelzenszwalbEdt2D::occupancy() const { return occupancy_; }

double FelzenszwalbEdt2D::getDistance(const int idx) const {
  if (idx < 0 || idx >= size_) {
    return kInf;
  }
  return distance_map_[static_cast<std::size_t>(idx)];
}

const std::vector<double>& FelzenszwalbEdt2D::getDistanceMap() const { return distance_map_; }

double FelzenszwalbEdt2D::intersection(const std::vector<double>& input, const int i, const int q) {
  const double fi = input[static_cast<std::size_t>(i)];
  const double fq = input[static_cast<std::size_t>(q)];
  return ((fq + static_cast<double>(q * q)) - (fi + static_cast<double>(i * i))) /
         (2.0 * static_cast<double>(q - i));
}

void FelzenszwalbEdt2D::computeSquaredDistanceTransform1D(
    const std::vector<double>& input,
    std::vector<double>* output) {
  const int n = static_cast<int>(input.size());
  if (n <= 0 || output == nullptr) {
    return;
  }

  std::vector<int> v(static_cast<std::size_t>(n), 0);
  std::vector<double> z(static_cast<std::size_t>(n + 1), 0.0);
  int k = 0;
  v[0] = 0;
  z[0] = -kLarge;
  z[1] = kLarge;

  for (int q = 1; q < n; ++q) {
    double s = intersection(input, v[static_cast<std::size_t>(k)], q);
    while (k > 0 && s <= z[static_cast<std::size_t>(k)]) {
      --k;
      s = intersection(input, v[static_cast<std::size_t>(k)], q);
    }
    ++k;
    v[static_cast<std::size_t>(k)] = q;
    z[static_cast<std::size_t>(k)] = s;
    z[static_cast<std::size_t>(k + 1)] = kLarge;
  }

  output->assign(static_cast<std::size_t>(n), kLarge);
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[static_cast<std::size_t>(k + 1)] < static_cast<double>(q)) {
      ++k;
    }
    const int vk = v[static_cast<std::size_t>(k)];
    const double diff = static_cast<double>(q - vk);
    (*output)[static_cast<std::size_t>(q)] =
        diff * diff + input[static_cast<std::size_t>(vk)];
  }
}

void FelzenszwalbEdt2D::computeFromOccupancy(const std::vector<uint8_t>& binary_occ) {
  if (size_ <= 0 || static_cast<int>(binary_occ.size()) != size_) {
    initialized_ = false;
    occupancy_.assign(static_cast<std::size_t>(std::max(0, size_)), 0);
    distance_map_.assign(static_cast<std::size_t>(std::max(0, size_)), kInf);
    return;
  }

  occupancy_ = binary_occ;
  std::vector<double> column_pass(static_cast<std::size_t>(size_), kLarge);

  for (int x = 0; x < width_; ++x) {
    std::vector<double> input(static_cast<std::size_t>(height_), kLarge);
    std::vector<double> output;
    for (int y = 0; y < height_; ++y) {
      const int idx = y * width_ + x;
      if (occupancy_[static_cast<std::size_t>(idx)] != 0) {
        input[static_cast<std::size_t>(y)] = 0.0;
      }
    }
    computeSquaredDistanceTransform1D(input, &output);
    for (int y = 0; y < height_; ++y) {
      column_pass[static_cast<std::size_t>(y * width_ + x)] =
          output[static_cast<std::size_t>(y)];
    }
  }

  const double resolved_leaf = (resolution_ > 0.0) ? resolution_ : 1.0;
  distance_map_.assign(static_cast<std::size_t>(size_), kInf);
  for (int y = 0; y < height_; ++y) {
    std::vector<double> input(static_cast<std::size_t>(width_), kLarge);
    std::vector<double> output;
    for (int x = 0; x < width_; ++x) {
      input[static_cast<std::size_t>(x)] =
          column_pass[static_cast<std::size_t>(y * width_ + x)];
    }
    computeSquaredDistanceTransform1D(input, &output);
    for (int x = 0; x < width_; ++x) {
      const int idx = y * width_ + x;
      const double sq_dist = output[static_cast<std::size_t>(x)];
      if (sq_dist >= kLarge * 0.5) {
        distance_map_[static_cast<std::size_t>(idx)] = kInf;
      } else {
        distance_map_[static_cast<std::size_t>(idx)] = std::sqrt(sq_dist) * resolved_leaf;
      }
    }
  }

  initialized_ = true;
}

}  // namespace graph_planner
