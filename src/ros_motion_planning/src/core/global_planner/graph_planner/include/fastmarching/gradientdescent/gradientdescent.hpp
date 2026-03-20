#ifndef GRADIENTDESCENT_H_
#define GRADIENTDESCENT_H_

#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
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
      static bool apply(grid_t & grid, int &  idx, Path & path, std::vector <double> & path_velocity, const double step = 1) {

          Coord current_coord;
          Point current_point;
          Coord dimsize = grid.getDimSizes();
          int ncells = grid.size();

          std::array<int, ndims_-1> d_; //  Same as nDGridMap class auxiliar array d_.
          d_[0] = dimsize[0];
          for (int i = 1; i < ndims_; ++i)
              d_[i] = dimsize[i]*d_[i-1];

          grid.idx2coord(idx, current_coord);
        //   std::cout <<"[gradient descent-72]idx:" << idx << " coord: " << current_coord[0] << " " << current_coord[1] << std::endl;
          std::copy_n( current_coord.begin(), ndims_, current_point.begin() ); // Cast to int.
          path.push_back(current_point);
        //   std::cout <<"[gradient descent-75]current_point: " << current_point[0] << " " << current_point[1] << std::endl;
          std::array<double, ndims_> grads;

          while(idx>= 0 && idx<ncells  && (idx+1)>=0 && (idx+1) < ncells  && grid[idx].getArrivalTime() != 0) 
          {
              // Every iteration the gradient is computed for all dimensions. If is infinite, we convert it to 1 (keeping the sign).
              // The static_cast are necessary because the conversion between coordinate (we check the value in coordinates) and points
              // (the path is composed by continuous points).
              // First dimension done apart.
            //   std::cout <<"[gradient descent-80]idx:" << idx << " grid[idx-1].getValue()/2 " << grid[idx-1].getValue()/2 << "grid[idx+1].getValue()/2 " << grid[idx+1].getValue()/2 << std::endl;
            
            if(idx==0)
                {
                  return false;
                    // grads[0] = - grid[idx].getValue()/2 + grid[idx+1].getValue()/2;
                }
              else
                {
                    grads[0] = - grid[idx-1].getValue()/2 + grid[idx+1].getValue()/2;
                }
              if (isinf(grads[0]))
                 {
                    // std::cout << "[gradient descent-87]Gradient is infinite. Setting it to 1." << std::endl;
                    grads[0] = sgn<double>(grads[0]);
                    // std::cout << "[gradient descent-89]grads[0] " << grads[0] << std::endl;
                 }
              double max_grad = std::abs(grads[0]);
            
              for (int i = 1; i < ndims_; ++i) {
                  if(idx-d_[i-1]<0 || idx+d_[i-1]>ncells)
                  {
                    return false;
                  }
                  grads[i] = - grid[idx-d_[i-1]].getValue()/2 + grid[idx+d_[i-1]].getValue()/2;
                  if (isinf(grads[i]))
                      grads[i] = sgn<double>(grads[i]);
                  if (std::abs(max_grad) < std::abs(grads[i]))
                      max_grad = grads[i];
              }

              // Updating points
              for (int i = 0; i < ndims_; ++i) {
                  // Moving the point in dim i.
                //   std::cout <<"[gradient descent-100]  i:"<< i <<  "current_point: " << current_point[0] << " " << current_point[1] <<"grads[i]: " << grads[i] << " max_grad: " << max_grad << std::endl;
                  current_point[i] = current_point[i] - step*grads[i]/std::abs(max_grad);
                //   std::cout <<"[gradient descent-101] current_point: " << current_point[0] << " " << current_point[1] << std::endl;
                  current_coord[i] = current_point[i];
              }
              path.push_back(current_point);
              path_velocity.push_back(grid[idx].getVelocity());
              grid.coord2idx(current_coord,idx);
              if(idx < 0 || idx >= grid.size())
              {
                //   std::cout << "[In while loop]End of the path reached. The cell_id = " << idx << "is null." << std::endl;
                //   std::cout << "[In while loop]The coords is: "  << current_coord[0] << " " << current_coord[1] << " "<< std::endl;
                  break;
              }
          }
          if(idx-1 < 0 || idx > ncells  || (idx+1)<0 || (idx+1) >= ncells)
            {
                // std::cout << "[Out of while loop]End of the path reached. The cell_id = " << (idx+1) << " is null." << std::endl;
                return false;
            }
          //Adding exactly the last point at the end.
          grid.idx2coord(idx, current_coord);
          std::copy_n( current_coord.begin(), ndims_, current_point.begin() ); // Cast to double.
          path.push_back(current_point);
          path_velocity.push_back(grid[idx].getVelocity());
          return true;
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

      static void apply_directional(grid_t & grid, int &  idx, Path & path, std::vector <double> velocity_map, std::vector <double> & path_velocity, const double step = 1) {

          Coord current_coord;
          Point current_point;
          Coord dimsize = grid.getDimSizes();

          std::array<int, ndims_-1> d_; //  Same as nDGridMap class auxiliar array d_.
          d_[0] = dimsize[0];
          for (int i = 1; i < ndims_; ++i)
              d_[i] = dimsize[i]*d_[i-1];

          grid.idx2coord(idx, current_coord);
          std::copy_n( current_coord.begin(), ndims_, current_point.begin() ); // Cast to int.
          path.push_back(current_point);

          std::array<double, ndims_> grads;

          while(grid[idx].getArrivalTime() != 0) {

              // Every iteration the gradient is computed for all dimensions. If is infinite, we convert it to 1 (keeping the sign).
              // The static_cast are necessary because the conversion between coordinate (we check the value in coordinates) and points
              // (the path is composed by continuous points).

              // First dimension done apart.
              grads[0] = - grid[idx-1].getDirectionalTime()/2 + grid[idx+1].getDirectionalTime()/2;
              if (isinf(grads[0]))
                  grads[0] = sgn<double>(grads[0]);
              double max_grad = std::abs(grads[0]);

              for (int i = 1; i < ndims_; ++i) {
                  grads[i] = - grid[idx-d_[i-1]].getDirectionalTime()/2 + grid[idx+d_[i-1]].getDirectionalTime()/2;
                  if (isinf(grads[i]))
                      grads[i] = sgn<double>(grads[i]);
                  if (std::abs(max_grad) < std::abs(grads[i]))
                      max_grad = grads[i];
              }

              // Updating points
              for (int i = 0; i < ndims_; ++i) {
                  // Moving the point in dim i.
                  current_point[i] = current_point[i] - step*grads[i]/std::abs(max_grad);
                  current_coord[i] = current_point[i];
              }

              path.push_back(current_point);
              path_velocity.push_back(velocity_map[idx]);
              grid.coord2idx(current_coord,idx);
              if(idx < 0 || idx >= grid.size())
              {
                //   std::cout << "End of the path reached. The cell_id = " << idx << " is null." << std::endl;
              }
          }
          //Adding exactly the last point at the end.
          grid.idx2coord(idx, current_coord);
          std::copy_n( current_coord.begin(), ndims_, current_point.begin() ); // Cast to double.
          path.push_back(current_point);
          path_velocity.push_back(velocity_map[idx]);
      }

    protected:
};


#endif /* GRADIENTDESCENT_H_*/
