#ifndef INCREMENTAL_ESDF_2D_H
#define INCREMENTAL_ESDF_2D_H

#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

namespace graph_planner {

class IncrementalEsdf2D {
public:
  IncrementalEsdf2D();

  void reset(int width, int height, double resolution);
  void initializeFromOccupancy(const std::vector<uint8_t>& binary_occ);
  void updateFromDiff(const std::vector<int>& inserted, const std::vector<int>& deleted);

  bool initialized() const;
  int width() const;
  int height() const;
  int size() const;
  double resolution() const;

  double getDistance(int idx) const;
  const std::vector<double>& getDistanceMap() const;
  const std::vector<uint8_t>& occupancy() const;

private:
  struct QueueEntry {
    double dist_sq = 0.0;
    int idx = -1;
    int source_idx = -1;

    bool operator>(const QueueEntry& other) const {
      if (dist_sq != other.dist_sq) {
        return dist_sq > other.dist_sq;
      }
      if (idx != other.idx) {
        return idx > other.idx;
      }
      return source_idx > other.source_idx;
    }
  };

  void resetState();
  bool isValidIndex(int idx) const;
  void pushCell(
      std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>>& queue,
      int idx,
      int source_idx);
  void processLowerQueue(
      std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>>& queue);
  void markForRaise(
      int idx,
      std::queue<int>& queue,
      std::vector<uint8_t>& enqueued);
  double computeDistanceSqCells(int idx, int source_idx) const;
  void updateDistanceCache() const;

  int width_;
  int height_;
  int size_;
  double resolution_;
  bool initialized_;

  std::vector<uint8_t> occupancy_;
  std::vector<int> source_idx_;
  std::vector<double> dist_sq_cells_;

  mutable std::vector<double> distance_map_cache_;
  mutable bool distance_cache_dirty_;
};

}  // namespace graph_planner

#endif  // INCREMENTAL_ESDF_2D_H
