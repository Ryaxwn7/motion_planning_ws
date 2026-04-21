/*! \file fastmarching2.hpp
    \brief Templated class which computes the Fast Marching Square (FM2).

    It uses as a main container the nDGridMap class. The nDGridMap type T
    has to be an FMCell or something inherited from it.

    The leafsize of the grid map is ignored since it has to be >=1 and that
    depends on the units employed.

    The type of the heap introduced is very important for the behaviour of the
    algorithm. The following heaps are provided:

    - FMDaryHeap wrap for the Boost D_ary heap (generalization of binary heaps).
    * Set by default if no other heap is specified. The arity has been set to 2
    * (binary heap) since it has been tested to be the more efficient in this algorithm.
    - FMFibHeap wrap for the Boost Fibonacci heap.
    - FMPriorityQueue wrap to the std::PriorityQueue class. This heap implies the implementation
    * of the Simplified FMM (SFMM) method, done automatically because of the FMPriorityQueue::increase implementation.
    *
    @par External documentation:
        FMM2:
          J.V. Gómez, A. Lumbier, S. Garrido and L. Moreno, The Path to Efficiency: Fast Marching Methods in Path Planning.

    Copyright (C) 2014 Javier V. Gomez and Jose Pardeiro
    www.javiervgomez.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.*/

#ifndef FASTMARCHING2_H_
#define FASTMARCHING2_H_

#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <array>
#include <limits>

#include "../fmm/fastmarching.hpp"
#include "../gradientdescent/gradientdescent.hpp"
#include "../fmm/fmdata/fmpriorityqueue.hpp"

template < class grid_t, class heap_t = FMPriorityQueue<FMCell>  >  class FastMarching2 {

    public:
        typedef std::vector< std::array< double, grid_t::getNDims() > > path_t;

        FastMarching2 <grid_t, heap_t> () {
            goal_idx_ = -1;
            velocity_scale_ = 1.0;
            velocity_alpha_ = 0.2;
            velocity_dmax_ = -1.0;
            robot_radius_ = 0.0;
            velocity_mode_ = 0;
            velocity_sigmoid_k_ = 0.15;
            velocity_sigmoid_b_ = 0.0;
            distance_field_backend_ = 0;
        };

        virtual ~FastMarching2 <grid_t, heap_t> () {};

         /**
          * Sets the input grid in which operations will be performed.
          *
          * @param g input grid map.
          */
        virtual void setEnvironment
        (grid_t * g) {
            grid_ = g;
        }

        /**
         * Sets the initial points by the indices in the nDGridMap and
         * computes the initialization of the Fast Marching Square calling
         * the init() function. When the wave front reaches goal_idx, the propagation
         * stops.
         *
         * @param initial_point contains the index of the initial point of the query.
         *
         * @param fmm2_sources contains the indices of the initial points corresponding to all black cells.
         *
         * @param goal_point contains the index of the goal point.
         *
         * @see init()
         */
        virtual void setInitialAndGoalPoints
        (const std::vector<int> & initial_point, const std::vector<int> & fmm2_sources, const int goal_idx) {
            initial_point_ = initial_point;
            fmm2_sources_ = fmm2_sources;
            goal_idx_ = goal_idx;
        }

        /**
         * Sets the initial points by the indices in the nDGridMap and
         * computes the initialization of the Fast Marching Method calling
         * the init() function.
         *
         * By default, the goal point is not set, so it will expand the second wave
         * throughout the whole map.
         *
         * @param init_point contains the indices of the init points.
         *
         * @param fmm2_sources contains the indices of the initial points corresponding to all black cells.
         *
         * @see init()
         */
        virtual void setInitialPoints
        (const std::vector<int> & init_points, const std::vector<int> & fmm2_sources) {
            setInitialAndGoalPoints(init_points, fmm2_sources, goal_idx_);
        }

        // 与 fm2_gather_ws 一致的速度场参数接口。
        virtual void setVelocityScale(const double scale) {
            if (scale > 0.0) {
                velocity_scale_ = scale;
            }
        }

        virtual void setVelocityProfile(const double alpha, const double dmax) {
            velocity_alpha_ = alpha;
            velocity_dmax_ = dmax;
        }

        virtual void setRobotRadius(const double radius) {
            if (radius >= 0.0) {
                robot_radius_ = radius;
            }
        }

        // 速度映射模式：0=线性，1=Sigmoid
        virtual void setVelocityMode(const int mode) {
            velocity_mode_ = (mode == 1) ? 1 : 0;
        }

        // Sigmoid 速度映射参数
        virtual void setVelocitySigmoid(const double k, const double b) {
            velocity_sigmoid_k_ = k;
            velocity_sigmoid_b_ = b;
        }

        // 距离场后端：0=FMM，1=二维精确欧氏距离变换（EDT/ESDF-style）
        virtual void setDistanceFieldBackend(const int backend) {
            distance_field_backend_ = (backend == 1) ? 1 : 0;
        }

        // 兼容旧接口：近似映射到新参数。
        virtual void setVelocityMappingParams(const double v_min,
                                              const double v_max,
                                              const double d0) {
            if (v_max > 0.0) {
                setVelocityScale(v_max);
                const double alpha = (v_min >= 0.0) ? (v_min / v_max) : velocity_alpha_;
                setVelocityProfile(alpha, d0);
            }
        }

        virtual std::vector<double> getVelocityMap() const {
            std::vector<double> velocities;
            if (!grid_) return velocities;
            
            velocities.reserve(grid_->size());
            for (int i = 0; i < grid_->size(); ++i) {
                velocities.push_back(grid_->getCell(i).getVelocity());
            }
            return velocities;
        }

        /**
         * Sets velocity map values to the grid.
         * @param velocityMap vector of velocity values matching grid size.
         * @note The input vector size must exactly match grid size.
         */
        virtual void setVelocityMap(const std::vector<double>& velocityMap) {
            if (!grid_ || velocityMap.size() != grid_->size()) return;
            
            for (int i = 0; i < velocityMap.size(); ++i) {
                grid_->getCell(i).setVelocity(velocityMap[i]);
            }
        }

        /**
         * Main Fast Marching Square Function with velocity saturation. It requires to call first the setInitialAndGoalPoints() function.
         *
         * @param maxDistance saturation distance (relative, where 1 means maximum distance). If this value is -1 (default) the velocities map is not saturated.
         *
         * @see setInitialPoints()
         */

        /**
         * Main Fast Marching Square Function with velocity saturation. It requires to call first the setInitialAndGoalPoints() function.
         *
         * @param maxDistance saturation distance (relative, where 1 means maximum distance). If this value is -1 (default) the velocities map is not saturated.
         *
         * @see setInitialPoints()
         */
        virtual void computeFM2 //有目标点的传统fm2路径规划
        (const float maxDistance = -1) {
            maxDistance_ = maxDistance;
            if (maxDistance != -1)
                computeVelocitiesMap(true);
            else
                computeVelocitiesMap();
            // According to the theoretical basis the wave is expanded from the goal point to the initial point.
            std::vector <int> wave_init;
            wave_init.push_back(goal_idx_);
            int wave_goal = initial_point_[0];

            FastMarching< grid_t, heap_t> fmm;
            fmm.setEnvironment(grid_);
            fmm.setInitialAndGoalPoints(wave_init, wave_goal);
            fmm.computeFM();
        }

        //添加机器人初始位置，计算机器人时间图, 没有目标点
        virtual void computeFM2_gather 
        (const std::vector<int> init_points, const float maxDistance = -1) {
            maxDistance_ = maxDistance;
            if (maxDistance != -1)
                computeVelocitiesMap(true);
            else
                computeVelocitiesMap();


            FastMarching< grid_t, heap_t> fmm;
            fmm.setEnvironment(grid_);
            fmm.setInitialPoints(init_points);
            fmm.computeFM();
        }

        virtual void computeFM2_gather_v 
        (const std::vector<int> init_points, const std::vector<double>& velocityMap,const float maxDistance = -1 ) {
            maxDistance_ = maxDistance;
            // if (maxDistance != -1)
            //     computeVelocitiesMap(true);
            // else
            //     computeVelocitiesMap();
            setVelocityMap(velocityMap);
            // According to the theoretical basis the wave is expanded from the goal point to the initial point.

            FastMarching< grid_t, heap_t> fmm;
            fmm.setEnvironment(grid_);
            fmm.setInitialPoints(init_points);
            fmm.computeFM();
        }

        virtual void computeFM2_velocity   
        (const float maxDistance = -1) {
            maxDistance_ = maxDistance;
            if (maxDistance != -1)
                computeVelocitiesMap(true);
            else
                computeVelocitiesMap();
        }
        /**
         * Computes the path from the previous given goal index to the minimum
         * of the times of arrival map. According to the theoretical basis the 
         * wave is expanded from the goal point to the initial point. For these 
         * reasons the gradient must to be applied from the initial point.
         *
         * No checks are done (points in the borders, points in obstacles...).
         *
         * The included scripts will parse the saved path.
         *
         * @param path the resulting path (output).
         *
         * @param velocity the resulting path (output).
         */
        virtual void computePath
        (path_t * p, std::vector <double> * path_velocity) {
            path_t* path_ = p;
            constexpr int ndims = 2;

            GradientDescent< nDGridMap<FMCell, ndims> > grad;
            grad.apply(*grid_,initial_point_[0],*path_, *path_velocity);
        }

        /**
         * Computes the path from the given goal index to the minimum
         * of the times of arrival map.
         *
         * No checks are done (points in the borders, points in obstacles...).
         *
         * The included scripts will parse the saved path.
         *
         * @param path the resulting path (output).
         *
         * @param velocity the resulting path (output).
         *
         * @param goal_idx index of the goal point, where gradient descent will start.
         */
        virtual void computePath
        (path_t * p, std::vector <double> * path_velocity, int goal_idx) {
            path_t* path_ = p;
            constexpr int ndims = 2;

            GradientDescent< nDGridMap<FMCell, ndims> > grad;
            grad.apply(*grid_,goal_idx,*path_, *path_velocity);
        }

    private:

        /**
         * Computes the velocities map of the FM2 algorithm.
         *
         * @param saturate select if the potential is saturated according to maxDistance_ .
         */
        void computeVelocitiesMap
        (bool saturate = false) {
            if (distance_field_backend_ == 1) {
                computeObstacleDistanceMapEsdf();
            } else {
                computeObstacleDistanceMapFmm();
            }

            const double vmax = (velocity_scale_ > 0.0) ? velocity_scale_ : 1.0;
            const double dr = (robot_radius_ > 0.0) ? robot_radius_ : 0.0;

            double dmax = velocity_dmax_;
            if (saturate && maxDistance_ > 0.0) {
                dmax = maxDistance_;
            }
            if (dmax <= 0.0) {
                dmax = 0.0;
                for (int i = 0; i < grid_->size(); ++i) {
                    if (!grid_->getCell(i).getOccupancy()) {
                        continue;
                    }
                    const double dist = grid_->getCell(i).getValue();
                    if (std::isfinite(dist) && dist > dmax) {
                        dmax = dist;
                    }
                }
            }
            if (dmax <= dr) {
                const double leaf = grid_->getLeafSize();
                dmax = dr + ((leaf > 0.0) ? leaf : 1.0);
            }

            double alpha = velocity_alpha_;
            if (alpha < 0.0) {
                alpha = 0.0;
            } else if (alpha > 1.0) {
                alpha = 1.0;
            }
            const double v_min = alpha * vmax;

            for (int i = 0; i < grid_->size(); i++) {
                if (!grid_->getCell(i).getOccupancy()) {
                    grid_->getCell(i).setVelocity(0.0);
                } else {
                    const double dist = grid_->getCell(i).getValue();
                    if (!std::isfinite(dist) || dist <= dr) {
                        grid_->getCell(i).setVelocity(0.0);
                    } else if (dist < dmax) {
                        if (velocity_mode_ == 1) {
                            const double k = velocity_sigmoid_k_;
                            const double b = velocity_sigmoid_b_;
                            const double e = 1.0 + std::exp(-k * (dist - dr) + b);
                            grid_->getCell(i).setVelocity(vmax / e);
                        } else {
                            const double ratio = (dist - dr) / (dmax - dr);
                            const double vel = v_min + (vmax - v_min) * ratio;
                            grid_->getCell(i).setVelocity(vel);
                        }
                    } else {
                        if (velocity_mode_ == 1) {
                            const double k = velocity_sigmoid_k_;
                            const double b = velocity_sigmoid_b_;
                            const double e = 1.0 + std::exp(-k * (dist - dr) + b);
                            grid_->getCell(i).setVelocity(vmax / e);
                        } else {
                            grid_->getCell(i).setVelocity(vmax);
                        }
                    }
                }


                // Restarting grid values for second wave expasion.
                grid_->getCell(i).setValue(std::numeric_limits<double>::infinity());
                grid_->getCell(i).setState(FMState::OPEN);
            }
        }

        void computeObstacleDistanceMapFmm() {
            FastMarching< grid_t, heap_t> fmm;
            fmm.setEnvironment(grid_);
            fmm.setInitialPoints(fmm2_sources_);
            fmm.computeFM();
        }

        static double computeSquaredDistanceIntersection(const std::vector<double>& input,
                                                         const int i,
                                                         const int q) {
            const double fi = input[static_cast<std::size_t>(i)];
            const double fq = input[static_cast<std::size_t>(q)];
            return ((fq + static_cast<double>(q * q)) - (fi + static_cast<double>(i * i))) /
                   (2.0 * static_cast<double>(q - i));
        }

        static void computeSquaredDistanceTransform1D(const std::vector<double>& input,
                                                      std::vector<double>* output) {
            const int n = static_cast<int>(input.size());
            if (n <= 0 || output == nullptr) {
                return;
            }

            constexpr double kLarge = 1e20;
            std::vector<int> v(static_cast<std::size_t>(n), 0);
            std::vector<double> z(static_cast<std::size_t>(n + 1), 0.0);
            int k = 0;
            v[0] = 0;
            z[0] = -kLarge;
            z[1] = kLarge;

            for (int q = 1; q < n; ++q) {
                double s = computeSquaredDistanceIntersection(input, v[static_cast<std::size_t>(k)], q);
                while (k > 0 && s <= z[static_cast<std::size_t>(k)]) {
                    --k;
                    s = computeSquaredDistanceIntersection(input, v[static_cast<std::size_t>(k)], q);
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

        void computeObstacleDistanceMapEsdf() {
            const auto dims = grid_->getDimSizes();
            const int width = static_cast<int>(dims[0]);
            const int height = static_cast<int>(dims[1]);
            if (width <= 0 || height <= 0) {
                return;
            }

            constexpr double kLarge = 1e20;
            const std::size_t total = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
            std::vector<double> column_pass(total, kLarge);

            for (int x = 0; x < width; ++x) {
                std::vector<double> input(static_cast<std::size_t>(height), kLarge);
                std::vector<double> output;
                for (int y = 0; y < height; ++y) {
                    const int idx = y * width + x;
                    if (grid_->getCell(idx).isOccupied()) {
                        input[static_cast<std::size_t>(y)] = 0.0;
                    }
                }
                computeSquaredDistanceTransform1D(input, &output);
                for (int y = 0; y < height; ++y) {
                    column_pass[static_cast<std::size_t>(y * width + x)] =
                        output[static_cast<std::size_t>(y)];
                }
            }

            const double leaf = grid_->getLeafSize();
            const double resolved_leaf = (leaf > 0.0) ? leaf : 1.0;

            for (int y = 0; y < height; ++y) {
                std::vector<double> input(static_cast<std::size_t>(width), kLarge);
                std::vector<double> output;
                for (int x = 0; x < width; ++x) {
                    input[static_cast<std::size_t>(x)] =
                        column_pass[static_cast<std::size_t>(y * width + x)];
                }
                computeSquaredDistanceTransform1D(input, &output);
                for (int x = 0; x < width; ++x) {
                    const int idx = y * width + x;
                    const double squared_distance = output[static_cast<std::size_t>(x)];
                    if (squared_distance >= kLarge * 0.5) {
                        grid_->getCell(idx).setValue(std::numeric_limits<double>::infinity());
                    } else {
                        grid_->getCell(idx).setValue(std::sqrt(squared_distance) * resolved_leaf);
                    }
                }
            }
        }

    protected:
        grid_t* grid_; /*!< Main container. */

        std::vector<int> fmm2_sources_;  /*!< Wave propagation sources for the Fast Marching Square. */
        std::vector<int> initial_point_; /*!< Initial point for the Fast Marching Square. */
        int goal_idx_; /*!< Goal point for the Fast Marching Square. */

        double maxDistance_; /*!< Distance value to saturate the first potential. */
        double velocity_scale_;
        double velocity_alpha_;
        double velocity_dmax_;
        double robot_radius_;
        int velocity_mode_;
        double velocity_sigmoid_k_;
        double velocity_sigmoid_b_;
        int distance_field_backend_;
};

#endif /* FASTMARCHING2_H_*/
