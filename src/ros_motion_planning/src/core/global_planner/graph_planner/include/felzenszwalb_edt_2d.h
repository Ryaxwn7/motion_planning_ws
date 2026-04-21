#ifndef FELZENSZWALB_EDT_2D_H
#define FELZENSZWALB_EDT_2D_H

#include <cstdint>
#include <vector>

namespace graph_planner {

class FelzenszwalbEdt2D {
public:
  FelzenszwalbEdt2D();

  void reset(int width, int height, double resolution);
  void computeFromOccupancy(const std::vector<uint8_t>& binary_occ);

  bool initialized() const;
  int width() const;
  int height() const;
  int size() const;
  double resolution() const;

  const std::vector<uint8_t>& occupancy() const;
  const std::vector<double>& getDistanceMap() const;
  double getDistance(int idx) const;

private:
  static double intersection(const std::vector<double>& input, int i, int q);
  static void computeSquaredDistanceTransform1D(
      const std::vector<double>& input,
      std::vector<double>* output);

  int width_;
  int height_;
  int size_;
  double resolution_;
  bool initialized_;

  std::vector<uint8_t> occupancy_;
  std::vector<double> distance_map_;
};

}  // namespace graph_planner

#endif  // FELZENSZWALB_EDT_2D_H
