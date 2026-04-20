#include "incremental_esdf_2d.h"

#include <algorithm>
#include <cmath>
#include <functional>

namespace graph_planner {

namespace {

const double kInf = std::numeric_limits<double>::infinity();

const int kNeighborOffsets[8][2] = {
    {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
    {0, 1},   {1, -1}, {1, 0},  {1, 1},
};

}  // namespace

IncrementalEsdf2D::IncrementalEsdf2D()
    : width_(0),
      height_(0),
      size_(0),
      resolution_(0.0),
      initialized_(false),
      distance_cache_dirty_(true) {}

void IncrementalEsdf2D::reset(int width, int height, double resolution) {
  width_ = std::max(0, width);
  height_ = std::max(0, height);
  size_ = width_ * height_;
  resolution_ = std::max(0.0, resolution);
  resetState();
}

void IncrementalEsdf2D::resetState() {
  initialized_ = false;
  occupancy_.assign(static_cast<std::size_t>(std::max(0, size_)), 0);
  source_idx_.assign(static_cast<std::size_t>(std::max(0, size_)), -1);
  dist_sq_cells_.assign(static_cast<std::size_t>(std::max(0, size_)), kInf);
  distance_map_cache_.assign(static_cast<std::size_t>(std::max(0, size_)), kInf);
  distance_cache_dirty_ = true;
}

bool IncrementalEsdf2D::initialized() const { return initialized_; }
int IncrementalEsdf2D::width() const { return width_; }
int IncrementalEsdf2D::height() const { return height_; }
int IncrementalEsdf2D::size() const { return size_; }
double IncrementalEsdf2D::resolution() const { return resolution_; }

const std::vector<uint8_t>& IncrementalEsdf2D::occupancy() const { return occupancy_; }

bool IncrementalEsdf2D::isValidIndex(const int idx) const {
  return idx >= 0 && idx < size_;
}

double IncrementalEsdf2D::computeDistanceSqCells(const int idx, const int source_idx) const {
  if (!isValidIndex(idx) || !isValidIndex(source_idx) || width_ <= 0) {
    return kInf;
  }
  const int x = idx % width_;
  const int y = idx / width_;
  const int sx = source_idx % width_;
  const int sy = source_idx / width_;
  const double dx = static_cast<double>(x - sx);
  const double dy = static_cast<double>(y - sy);
  return dx * dx + dy * dy;
}

void IncrementalEsdf2D::pushCell(
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>>& queue,
    const int idx,
    const int source_idx) {
  if (!isValidIndex(idx) || !isValidIndex(source_idx)) {
    return;
  }
  queue.push(QueueEntry{computeDistanceSqCells(idx, source_idx), idx, source_idx});
}

void IncrementalEsdf2D::processLowerQueue(
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>>& queue) {
  while (!queue.empty()) {
    const QueueEntry entry = queue.top();
    queue.pop();

    if (!isValidIndex(entry.idx) || !isValidIndex(entry.source_idx)) {
      continue;
    }
    if (source_idx_[static_cast<std::size_t>(entry.idx)] != entry.source_idx) {
      continue;
    }
    const double current_dist = dist_sq_cells_[static_cast<std::size_t>(entry.idx)];
    if (!std::isfinite(current_dist) || std::abs(current_dist - entry.dist_sq) > 1e-9) {
      continue;
    }

    const int x = entry.idx % width_;
    const int y = entry.idx / width_;
    for (const auto& offset : kNeighborOffsets) {
      const int nx = x + offset[0];
      const int ny = y + offset[1];
      if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
        continue;
      }
      const int nidx = ny * width_ + nx;
      const double cand_dist = computeDistanceSqCells(nidx, entry.source_idx);
      if ((source_idx_[static_cast<std::size_t>(nidx)] < 0) ||
          (cand_dist + 1e-9 < dist_sq_cells_[static_cast<std::size_t>(nidx)])) {
        source_idx_[static_cast<std::size_t>(nidx)] = entry.source_idx;
        dist_sq_cells_[static_cast<std::size_t>(nidx)] = cand_dist;
        queue.push(QueueEntry{cand_dist, nidx, entry.source_idx});
      }
    }
  }
}

void IncrementalEsdf2D::initializeFromOccupancy(const std::vector<uint8_t>& binary_occ) {
  if (size_ <= 0 || static_cast<int>(binary_occ.size()) != size_) {
    resetState();
    return;
  }

  occupancy_ = binary_occ;
  source_idx_.assign(binary_occ.size(), -1);
  dist_sq_cells_.assign(binary_occ.size(), kInf);

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> lower_queue;
  for (int idx = 0; idx < size_; ++idx) {
    if (occupancy_[static_cast<std::size_t>(idx)] == 0) {
      continue;
    }
    source_idx_[static_cast<std::size_t>(idx)] = idx;
    dist_sq_cells_[static_cast<std::size_t>(idx)] = 0.0;
    lower_queue.push(QueueEntry{0.0, idx, idx});
  }

  processLowerQueue(lower_queue);
  initialized_ = true;
  distance_cache_dirty_ = true;
}

void IncrementalEsdf2D::markForRaise(
    const int idx,
    std::queue<int>& queue,
    std::vector<uint8_t>& enqueued) {
  if (!isValidIndex(idx)) {
    return;
  }
  const std::size_t uidx = static_cast<std::size_t>(idx);
  if (enqueued[uidx] != 0) {
    return;
  }
  enqueued[uidx] = 1;
  queue.push(idx);
}

void IncrementalEsdf2D::updateFromDiff(
    const std::vector<int>& inserted,
    const std::vector<int>& deleted) {
  if (!initialized_) {
    return;
  }
  if (size_ <= 0) {
    return;
  }

  std::vector<uint8_t> deleted_source_mask(static_cast<std::size_t>(size_), 0);
  for (const int idx : deleted) {
    if (!isValidIndex(idx)) {
      continue;
    }
    deleted_source_mask[static_cast<std::size_t>(idx)] = 1;
    occupancy_[static_cast<std::size_t>(idx)] = 0;
  }
  for (const int idx : inserted) {
    if (!isValidIndex(idx)) {
      continue;
    }
    occupancy_[static_cast<std::size_t>(idx)] = 1;
  }

  std::queue<int> raise_queue;
  std::vector<uint8_t> raise_enqueued(static_cast<std::size_t>(size_), 0);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> lower_queue;

  for (const int idx : deleted) {
    if (!isValidIndex(idx)) {
      continue;
    }
    if (source_idx_[static_cast<std::size_t>(idx)] >= 0 &&
        deleted_source_mask[static_cast<std::size_t>(source_idx_[static_cast<std::size_t>(idx)])] != 0) {
      source_idx_[static_cast<std::size_t>(idx)] = -1;
      dist_sq_cells_[static_cast<std::size_t>(idx)] = kInf;
      markForRaise(idx, raise_queue, raise_enqueued);
    }
  }

  while (!raise_queue.empty()) {
    const int idx = raise_queue.front();
    raise_queue.pop();

    const int x = idx % width_;
    const int y = idx / width_;
    for (const auto& offset : kNeighborOffsets) {
      const int nx = x + offset[0];
      const int ny = y + offset[1];
      if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
        continue;
      }
      const int nidx = ny * width_ + nx;
      const int nsrc = source_idx_[static_cast<std::size_t>(nidx)];
      if (nsrc >= 0 && deleted_source_mask[static_cast<std::size_t>(nsrc)] != 0) {
        source_idx_[static_cast<std::size_t>(nidx)] = -1;
        dist_sq_cells_[static_cast<std::size_t>(nidx)] = kInf;
        markForRaise(nidx, raise_queue, raise_enqueued);
      } else if (nsrc >= 0) {
        lower_queue.push(QueueEntry{dist_sq_cells_[static_cast<std::size_t>(nidx)], nidx, nsrc});
      }
    }
  }

  for (const int idx : inserted) {
    if (!isValidIndex(idx)) {
      continue;
    }
    source_idx_[static_cast<std::size_t>(idx)] = idx;
    dist_sq_cells_[static_cast<std::size_t>(idx)] = 0.0;
    lower_queue.push(QueueEntry{0.0, idx, idx});
  }

  processLowerQueue(lower_queue);
  distance_cache_dirty_ = true;
}

double IncrementalEsdf2D::getDistance(const int idx) const {
  if (!isValidIndex(idx)) {
    return kInf;
  }
  const double dist_sq = dist_sq_cells_[static_cast<std::size_t>(idx)];
  if (!std::isfinite(dist_sq)) {
    return kInf;
  }
  return std::sqrt(dist_sq) * resolution_;
}

void IncrementalEsdf2D::updateDistanceCache() const {
  if (!distance_cache_dirty_) {
    return;
  }
  distance_map_cache_.assign(static_cast<std::size_t>(std::max(0, size_)), kInf);
  for (int idx = 0; idx < size_; ++idx) {
    const double dist_sq = dist_sq_cells_[static_cast<std::size_t>(idx)];
    if (!std::isfinite(dist_sq)) {
      distance_map_cache_[static_cast<std::size_t>(idx)] = kInf;
      continue;
    }
    distance_map_cache_[static_cast<std::size_t>(idx)] = std::sqrt(dist_sq) * resolution_;
  }
  distance_cache_dirty_ = false;
}

const std::vector<double>& IncrementalEsdf2D::getDistanceMap() const {
  updateDistanceCache();
  return distance_map_cache_;
}

}  // namespace graph_planner
