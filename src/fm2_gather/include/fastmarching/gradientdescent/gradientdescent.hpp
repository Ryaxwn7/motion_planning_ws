#ifndef GRADIENTDESCENT_H_
#define GRADIENTDESCENT_H_

#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include "../ndgridmap/ndgridmap.hpp"
#include "../fmm/fmdata/fmcell.h"

// TODO: check if points fall in obstacles, points in the borders, etc.

template <typename T> double sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

template <class grid_t> class GradientDescent {
    private:
        static constexpr size_t ndims_ = grid_t::getNDims();

        // Short-hand.
        typedef typename std::array<int, ndims_> Coord;
        typedef typename std::array<double, ndims_> Point;
        typedef typename std::vector <Point> Path;

        static bool isValidIndex(const grid_t& grid, const int idx) {
            return idx >= 0 && idx < grid.size();
        }

        static bool isValidCoord(const Coord& coord, const Coord& dimsize) {
            for (size_t i = 0; i < ndims_; ++i) {
                if (coord[i] < 0 || coord[i] >= dimsize[i]) {
                    return false;
                }
            }
            return true;
        }

        static bool isInteriorCoord(const Coord& coord, const Coord& dimsize) {
            for (size_t i = 0; i < ndims_; ++i) {
                if (coord[i] <= 0 || coord[i] >= (dimsize[i] - 1)) {
                    return false;
                }
            }
            return true;
        }

        static double sanitizeGradient(double grad) {
            if (std::isinf(grad)) {
                return sgn<double>(grad);
            }
            if (!std::isfinite(grad)) {
                return 0.0;
            }
            return grad;
        }

        static double sampleVelocity(const std::vector<double>& velocity_map, const int idx) {
            if (idx < 0 || idx >= static_cast<int>(velocity_map.size())) {
                return 0.0;
            }
            return velocity_map[idx];
        }

    public:
        GradientDescent() {}; // Default constructor not used.
        virtual ~GradientDescent() {};

       /**
        * Computes the path from the given index to a minimum (the one
        * gradient descent choses) and extract the velocity in every point.
        * The T class chosen must be an nDGridMap or similar whose Cell
        * element should be inherited from Cell class.
        *
        * Simple gradient approximation is used: dimension 0: gx = f((x-1,y)+f(x+1,y))/2
        * dimension 1: gy = f((x,y-1)+f(x,y+1))/2
        * and so on.
        *
        * No checks are done (points in the borders, points in obstacles...).
        *
        * Saves the velocities profile of the path extracting the velocity value of each cell.
        *
        * The included scripts will parse the saved path.
        *
        * IMPORTANT NOTE: both minimum and initial index cannot be in the
        * border of the map. This situation is not checked. We recommend to set a 1 pixel
        * black border around the map image.
        *
        * @param grid the grid to apply gradient descent on.
        * @param idx index of the initial path point.
        * @param path the resulting path (output).
        * @param the velocities profile of the path (output).
        * @param the step size to be applied.
        */
      static void apply
      (grid_t & grid, int &  idx, Path & path, std::vector <double> & path_velocity, const double step = 1) {

          if (grid.size() <= 0 || !isValidIndex(grid, idx)) {
              return;
          }

          Coord current_coord;
          Point current_point;
          Coord dimsize = grid.getDimSizes();

          std::array<int, ndims_-1> d_; //  Same as nDGridMap class auxiliar array d_.
          d_[0] = dimsize[0];
          for (int i = 1; i < ndims_; ++i)
              d_[i] = dimsize[i]*d_[i-1];

          grid.idx2coord(idx, current_coord);
          if (!isValidCoord(current_coord, dimsize)) {
              return;
          }
          std::copy_n( current_coord.begin(), ndims_, current_point.begin() ); // Cast to int.
          path.push_back(current_point);

          std::array<double, ndims_> grads;
          const std::size_t max_iterations = std::max<std::size_t>(1, static_cast<std::size_t>(grid.size()) * 4);
          std::size_t iteration = 0;

          while(iteration++ < max_iterations) {
              if (!isValidIndex(grid, idx) || !isInteriorCoord(current_coord, dimsize)) {
                  break;
              }
              const double arrival_time = grid[idx].getArrivalTime();
              if (!std::isfinite(arrival_time) ||
                  std::abs(arrival_time) <= std::numeric_limits<double>::epsilon()) {
                  break;
              }
              // Every iteration the gradient is computed for all dimensions. If is infinite, we convert it to 1 (keeping the sign).
              // The static_cast are necessary because the conversion between coordinate (we check the value in coordinates) and points
              // (the path is composed by continuous points).

              // First dimension done apart.
              grads[0] = sanitizeGradient(- grid[idx-1].getValue()/2 + grid[idx+1].getValue()/2);
              double max_grad = std::abs(grads[0]);

              for (int i = 1; i < ndims_; ++i) {
                  grads[i] = sanitizeGradient(- grid[idx-d_[i-1]].getValue()/2 + grid[idx+d_[i-1]].getValue()/2);
                  if (std::abs(max_grad) < std::abs(grads[i]))
                      max_grad = std::abs(grads[i]);
              }

              if (max_grad <= std::numeric_limits<double>::epsilon()) {
                  break;
              }

              // Updating points
              Coord next_coord = current_coord;
              Point next_point = current_point;
              for (int i = 0; i < ndims_; ++i) {
                  // Moving the point in dim i.
                  next_point[i] = current_point[i] - step*grads[i]/max_grad;
                  next_coord[i] = next_point[i];
              }

              if (!isValidCoord(next_coord, dimsize)) {
                  break;
              }

              int next_idx = idx;
              grid.coord2idx(next_coord,next_idx);
              path.push_back(next_point);
              path_velocity.push_back(grid[idx].getVelocity());
              current_point = next_point;
              current_coord = next_coord;
              const int prev_idx = idx;
              idx = next_idx;
              if (idx == prev_idx) {
                  break;
              }
          }
          //Adding exactly the last point at the end.
          if (isValidIndex(grid, idx)) {
              grid.idx2coord(idx, current_coord);
              if (isValidCoord(current_coord, dimsize)) {
                  std::copy_n( current_coord.begin(), ndims_, current_point.begin() ); // Cast to double.
                  path.push_back(current_point);
                  path_velocity.push_back(grid[idx].getVelocity());
              }
          }
      }

      /**
       * Computes the path from the given index to a minimum (the one
       * gradient descent choses) and extract the velocity in every point.
       * The T class chosen must be an nDGridMap or similar whose Cell
       * element should be inherited from Cell class.
       *
       * The gradient is applied on the FM2 Directional values.
       *
       * Simple gradient approximation is used: dimension 0: gx = f((x-1,y)+f(x+1,y))/2
       * dimension 1: gy = f((x,y-1)+f(x,y+1))/2
       * and so on.
       *
       * No checks are done (points in the borders, points in obstacles...).
       *
       * Saves the velocities profile of the path extracting the velocity value of each cell.
       *
       * The included scripts will parse the saved path.
       *
       * IMPORTANT NOTE: both minimum and initial index cannot be in the
       * border of the map. This situation is not checked. We recommend to set a 1 pixel
       * black border around the map image.
       *
       * @param grid the grid to apply gradient descent on.
       * @param idx index of the initial path point.
       * @param path the resulting path (output).
       * @param the velocities profile of the path (output).
       * @param the step size to be applied.
       */

      static void apply_directional
      (grid_t & grid, int &  idx, Path & path, std::vector <double> velocity_map, std::vector <double> & path_velocity, const double step = 1) {

          if (grid.size() <= 0 || !isValidIndex(grid, idx)) {
              return;
          }

          Coord current_coord;
          Point current_point;
          Coord dimsize = grid.getDimSizes();

          std::array<int, ndims_-1> d_; //  Same as nDGridMap class auxiliar array d_.
          d_[0] = dimsize[0];
          for (int i = 1; i < ndims_; ++i)
              d_[i] = dimsize[i]*d_[i-1];

          grid.idx2coord(idx, current_coord);
          if (!isValidCoord(current_coord, dimsize)) {
              return;
          }
          std::copy_n( current_coord.begin(), ndims_, current_point.begin() ); // Cast to int.
          path.push_back(current_point);

          std::array<double, ndims_> grads;
          const std::size_t max_iterations = std::max<std::size_t>(1, static_cast<std::size_t>(grid.size()) * 4);
          std::size_t iteration = 0;

          while(iteration++ < max_iterations) {
              if (!isValidIndex(grid, idx) || !isInteriorCoord(current_coord, dimsize)) {
                  break;
              }
              const double arrival_time = grid[idx].getArrivalTime();
              if (!std::isfinite(arrival_time) ||
                  std::abs(arrival_time) <= std::numeric_limits<double>::epsilon()) {
                  break;
              }

              // Every iteration the gradient is computed for all dimensions. If is infinite, we convert it to 1 (keeping the sign).
              // The static_cast are necessary because the conversion between coordinate (we check the value in coordinates) and points
              // (the path is composed by continuous points).

              // First dimension done apart.
              grads[0] = sanitizeGradient(- grid[idx-1].getDirectionalTime()/2 + grid[idx+1].getDirectionalTime()/2);
              double max_grad = std::abs(grads[0]);

              for (int i = 1; i < ndims_; ++i) {
                  grads[i] = sanitizeGradient(- grid[idx-d_[i-1]].getDirectionalTime()/2 + grid[idx+d_[i-1]].getDirectionalTime()/2);
                  if (std::abs(max_grad) < std::abs(grads[i]))
                      max_grad = std::abs(grads[i]);
              }

              if (max_grad <= std::numeric_limits<double>::epsilon()) {
                  break;
              }

              // Updating points
              Coord next_coord = current_coord;
              Point next_point = current_point;
              for (int i = 0; i < ndims_; ++i) {
                  // Moving the point in dim i.
                  next_point[i] = current_point[i] - step*grads[i]/max_grad;
                  next_coord[i] = next_point[i];
              }

              if (!isValidCoord(next_coord, dimsize)) {
                  break;
              }

              int next_idx = idx;
              grid.coord2idx(next_coord,next_idx);
              path.push_back(next_point);
              path_velocity.push_back(sampleVelocity(velocity_map, idx));
              current_point = next_point;
              current_coord = next_coord;
              const int prev_idx = idx;
              idx = next_idx;
              if (idx == prev_idx) {
                  break;
              }
          }
          //Adding exactly the last point at the end.
          if (isValidIndex(grid, idx)) {
              grid.idx2coord(idx, current_coord);
              if (isValidCoord(current_coord, dimsize)) {
                  std::copy_n( current_coord.begin(), ndims_, current_point.begin() ); // Cast to double.
                  path.push_back(current_point);
                  path_velocity.push_back(sampleVelocity(velocity_map, idx));
              }
          }
      }

    protected:
};


#endif /* GRADIENTDESCENT_H_*/
