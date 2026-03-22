/*! \file gridplotter.hpp
    \brief Auxiliar class which helps to visualise Fast Marching steps and results.
    
    It is based on the CImg library, therefore it has to be accessible.
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
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef GRIDPLOTTER_H_
#define GRIDPLOTTER_H_

#include <string>
#include <array>
#include <cstdlib>
#include <filesystem>

#include "../ndgridmap/ndgridmap.hpp"
#define cimg_use_png
#include "../thirdparty/CImg.h"
#include <memory>

using namespace cimg_library;

typedef typename std::array<int, 2> Coord2D;
typedef typename std::array<double, 2> Point2D;
typedef typename std::vector <Point2D> Path2D;
typedef typename std::vector <std::shared_ptr<Path2D>> Paths2D;

// TODO: include checks which ensure that the grids are adecuate for the functions used.
class GridPlotter {
    public:
        GridPlotter() {};
        virtual ~GridPlotter() {};

        static std::string getDebugOutputDir() {
            namespace fs = std::filesystem;
            const char* custom = std::getenv("FM2_GATHER_DEBUG_DIR");
            fs::path base;
            if (custom != nullptr && custom[0] != '\0') {
                base = custom;
            } else {
                const char* ros_home = std::getenv("ROS_HOME");
                if (ros_home != nullptr && ros_home[0] != '\0') {
                    base = fs::path(ros_home) / "fm2_gather_debug";
                } else {
                    const char* home = std::getenv("HOME");
                    if (home != nullptr && home[0] != '\0') {
                        base = fs::path(home) / ".ros" / "fm2_gather_debug";
                    } else {
                        base = fs::path("/tmp") / "fm2_gather_debug";
                    }
                }
            }
            std::error_code ec;
            fs::create_directories(base, ec);
            return base.string();
        }

        static std::string makeDebugFilepath(const std::string& filename) {
            namespace fs = std::filesystem;
            return (fs::path(getDebugOutputDir()) / filename).string();
        }


        /**
         * Plots the initial binary map included in a given grid. It is based on the
         * nDGridMap::getOccupancy() which has to be bool valued. This function has to be
         * overloaded in another occupancy type is being used.
         * 
         * Should be used only in 2D grids.
         * 
         *  The Y dimension flipping is because nDGridMap works in X-Y coordinates, not in image indices as CImg.
         * 
         * IMPORTANT NOTE: no type-checkings are done. T type has to be Cell or any class with bool getOccupancy() method.
         * 
         * @param grid 2D nDGridmap
         * @param flipY true: flips the Y dimension. 0 does not flip.
         */
        template<class T, size_t ndims> 
        static void plotMap
        (nDGridMap<T, ndims> & grid,  std::string title = "Grid map",const bool flipY = 1) {
            // TODO: image checking: B/W, correct reading, etc.
            std::array<int,2> d = grid.getDimSizes();
            CImg<bool> img(d[0],d[1],1,1,0);
            if (flipY)
                // Filling the image flipping Y dim. We want now top left to be the (0,0).
                cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getOccupancy(); }	
            else 
                cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getOccupancy(); }

            img.display("Grid map", false);
            CImg<unsigned char> img_normalized(img.get_resize(-100,-100,1,1)*255);
            std::string filename = makeDebugFilepath(title + ".png");
            img_normalized.save_png(filename.c_str());
        }

        template<class T, size_t ndims> 
        static void plotMap
        (nDGridMap<T, ndims> & grid,const bool flipY = 1) {
            // TODO: image checking: B/W, correct reading, etc.
            std::array<int,2> d = grid.getDimSizes();
            CImg<bool> img(d[0],d[1],1,1,0);
            if (flipY)
                // Filling the image flipping Y dim. We want now top left to be the (0,0).
                cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getOccupancy(); }	
            else 
                cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getOccupancy(); }

            img.display("Grid map", false);
            CImg<unsigned char> img_normalized(img.get_resize(-100,-100,1,1)*255);
            std::string filename = makeDebugFilepath("map.png");
            img_normalized.save_png(filename.c_str());
        }

       /**
         * Plots the value map included in a given grid. It is based on the
         * nDGridMap::getValue() which has to be float valued. This function has to be
         * overloaded in another value type is being used.
         * 
         * Should be used only in 2D grids.
         * 
         * The Y dimension flipping is because nDGridMap works in X-Y coordinates, not in image indices as CImg.
         * 
         * IMPORTANT NOTE: no type-checkings are done. T type has to be Cell or any class with bool getValue() method.
         * 
         * @param grid 2D nDGridmap
         * @param flipY true: flips the Y dimension. 0 does not flip.
         */
        // template<class T, size_t ndims = 2> 
        // static void plotArrivalTimes
        // (nDGridMap<T, ndims> & grid, const bool flipY = true) {
        //     std::array<int,2> d = grid.getDimSizes();
        //     double max_val = grid.getMaxValue();
        //     // double avg_val = grid.getAvgValue();
        //     // double thereshold = 2*avg_val; // TODO: make this a parameter
        //     CImg<double> img(d[0],d[1],1,1,0);
        //     if (flipY) 
        //         // Filling the image flipping Y dim. We want now top left to be the (0,0).
        //         cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getValue()/max_val*255; }
        //     else 
        //         cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getValue()/max_val*255; }	
        //     // if (flipY) 
        //         // Filling the image flipping Y dim. We want now top left to be the (0,0).
        //         // cimg_forXY(img,x,y) { img(x,y) = (grid[img.width()*(img.height()-y-1)+x].getValue()>thereshold)? thereshold : grid[img.width()*(img.height()-y-1)+x ].getValue()/thereshold*255; }
        //     // else 
        //         // cimg_forXY(img,x,y) { img(x,y) = (grid[img.width()*y+x].getValue()>thereshold)? thereshold : grid[img.width()*y+x].getValue()/thereshold*255; }	
                
        //     img.map( CImg<float>::jet_LUT256() );
        //     img.display("Grid values", false);	
        //     img.save_png("/home/bsrl-ubuntu/new_ws/src/fm2_gather/data/arrival_times.png");

        // }

        template<class T, size_t ndims = 2> 
        static void plotArrivalTimes(
            nDGridMap<T, ndims>& grid, 
            std::string title = "Arrival_times", 
            const bool flipY = true, 
            double upper_percentile = 0.95) {

            std::array<int, 2> d = grid.getDimSizes();
            int width = d[0], height = d[1];
            
            // 1. 收集所有非零值（避免零值干扰分位数计算）
            std::vector<double> non_zero_values;
            for (int i = 0; i < width * height; ++i) {
                double val = grid[i].getValue();
                if (val > 0.0 && std::isfinite(val)) non_zero_values.push_back(val);
            }

            // 2. 处理全零数据的特殊情况
            double max_val = 0.0;
            if (!non_zero_values.empty()) {
                if (upper_percentile >= 1.0) {
                    // 直接取最大值
                    max_val = *std::max_element(non_zero_values.begin(), non_zero_values.end());
                } else {
                    // 计算分位数
                    size_t n = non_zero_values.size();
                    size_t k = static_cast<size_t>(upper_percentile * (n - 1));
                    std::nth_element(non_zero_values.begin(), non_zero_values.begin() + k, non_zero_values.end());
                    max_val = non_zero_values[k];
                    std::cout << "Upper percentile: " << upper_percentile << ", value: " << max_val << std::endl;
                }
            } else {
                // 全零数据警告
                std::cout << "Warning: All grid values are zero." << std::endl;
                max_val = 1.0; // 避免除零错误
            }

            // 3. 创建图像并填充数据（确保索引正确）
            CImg<double> img(width, height, 1, 1, 0);
            auto map_value = [max_val](double val) {
                val = std::min(val, max_val);
                return (max_val > 0) ? (val / max_val * 255) : 0.0;
            };

            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    // 计算实际网格索引
                    int grid_y = flipY ? (height - y - 1) : y;
                    int grid_idx = grid_y * width + x; // 确保行优先存储
                    double val = grid[grid_idx].getValue();
                    if(isinf(val) || val == 0.0)
                        {
                            img(x, y) = 0.0;
                        }
                    else
                        {
                            img(x, y) = map_value(val);
                        }
                }
            }

            // 4. 应用颜色映射并保存
            img.map(CImg<float>::jet_LUT256());
            img.display("Grid values", false);
            
            std::string filename = makeDebugFilepath(title + ".png");
            img.save_png(filename.c_str());
        }
        // template<class T, size_t ndims = 2> 
        // static void plotVelocities
        // (nDGridMap<T, ndims> & grid, const bool flipY = true) {
        //     std::array<int,2> d = grid.getDimSizes();
        //     double max_val = grid.getMaxValue();
        //     // double avg_val = grid.getAvgValue();
        //     // double thereshold = 2*avg_val; // TODO: make this a parameter
        //     CImg<double> img(d[0],d[1],1,1,0);
        //     if (flipY) 
        //         // Filling the image flipping Y dim. We want now top left to be the (0,0).
        //         cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getVelocity()/max_val*255; }
        //     else 
        //         cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getVelocity()/max_val*255; }	
        //     // if (flipY) 
        //         // Filling the image flipping Y dim. We want now top left to be the (0,0).
        //         // cimg_forXY(img,x,y) { img(x,y) = (grid[img.width()*(img.height()-y-1)+x].getValue()>thereshold)? thereshold : grid[img.width()*(img.height()-y-1)+x ].getValue()/thereshold*255; }
        //     // else 
        //         // cimg_forXY(img,x,y) { img(x,y) = (grid[img.width()*y+x].getValue()>thereshold)? thereshold : grid[img.width()*y+x].getValue()/thereshold*255; }	
                
        //     img.map( CImg<float>::jet_LUT256() );
        //     img.display("Grid values", false);	
            
        // }

        /**
         * Plots the initial binary map included in a given grid and the given path. It is based on the
         * nDGridMap::getOccupancy() which has to be bool valued. This function has to be
         * overloaded in another occupancy type is being used.
         *
         * Should be used only in 2D grids.
         *
         *  The Y dimension flipping is because nDGridMap works in X-Y coordinates, not in image indices as CImg.
         *
         * IMPORTANT NOTE: no type-checkings are done. T type has to be Cell or any class with bool getOccupancy() method.
         *
         * @param grid 2D nDGridmap
         * @param path 2D path to plot.
         * @param flipY true: flips the Y dimension. 0 does not flip.
         */
        template<class T, size_t ndims = 2>
        static void plotMapPath
        (nDGridMap<T, ndims> & grid, const Path2D & path, const bool flipY = true) {
            std::array<int,2> d = grid.getDimSizes();
            CImg<double> img(d[0],d[1],1,3,0);

            if (flipY)  {
                // Filling the image flipping Y dim. We want now top left to be the (0,0).
                cimg_forXYZC(img,x,y,z,c) { img(x,y,z,c) = grid[img.width()*(img.height()-y-1)+x].getOccupancy()*255; }

                for (int i = 0; i< path.size(); ++i)
                {
                    img(static_cast<int>(path[i][0]), (img.height()-static_cast<int>(path[i][1])-1), 0, 1) = 0;
                    img(static_cast<int>(path[i][0]), (img.height()-static_cast<int>(path[i][1])-1), 0, 2) = 0;
                }
            }
            else {
                cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getOccupancy()*255; }
                for (int i = 0; i< path.size(); ++i)
                    img(static_cast<int>(path[i][0]), static_cast<int>(path[i][1])) = 255;
                }

            img.display("Grid values", false);

        }

        /**
         * Plots the initial binary map included in a given grid and the given paths vector.
         * It is based on the nDGridMap::getOccupancy() which has to be bool valued. This function
         * has to be overloaded in another occupancy type is being used.
         *
         * Should be used only in 2D grids.
         *
         *  The Y dimension flipping is because nDGridMap works in X-Y coordinates, not in image indices as CImg.
         *
         * IMPORTANT NOTE: no type-checkings are done. T type has to be Cell or any class with bool getOccupancy() method.
         *
         * @param grid 2D nDGridmap
         * @param paths vector of path 2D path to plot.
         * @param flipY true: flips the Y dimension. 0 does not flip.
         */
        template<class T, size_t ndims = 2>
        static void plotMapPath
        (nDGridMap<T, ndims> & grid, const Paths2D & paths, const bool flipY = true) {
            std::array<int,2> d = grid.getDimSizes();
            CImg<double> img(d[0],d[1],1,3,0);

            if (flipY)  {
                // Filling the image flipping Y dim. We want now top left to be the (0,0).
                cimg_forXYZC(img,x,y,z,c) { img(x,y,z,c) = grid[img.width()*(img.height()-y-1)+x].getOccupancy()*255; }

                const unsigned char dark_colors[6][3] = {
                    {51, 0, 0},   // Dark Red
                    {0, 51, 0},   // Dark Green
                    {0, 0, 51},   // Dark Blue
                    {51, 51, 0},  // Dark Yellow
                    {0, 51, 51},  // Dark Cyan
                    {51, 0, 51}   // Dark Magenta
                };

                // Draw the paths using different dark colors
                for (int j = 0; j < paths.size(); ++j) {
                    Path2D path = *paths[j];
                    int color_idx = j % 6; // Cycle through colors if more than 6 paths
                    unsigned char r = dark_colors[color_idx][0];
                    unsigned char g = dark_colors[color_idx][1];
                    unsigned char b = dark_colors[color_idx][2];

                    for (int i = 0; i < path.size(); ++i) {
                        int x = static_cast<int>(path[i][0]);
                        int y = img.height() - static_cast<int>(path[i][1]) - 1;

                        // Set pixel color
                        img(x, y, 0) = r; // Red channel
                        img(x, y, 1) = g; // Green channel
                        img(x, y, 2) = b; // Blue channel
                    }
                }
            }
            else {
                cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getOccupancy()*255; }

                // Draw the path using different colours
                for (int j = 0; j < paths.size(); ++j)
                {
                    Path2D path = *paths[j];

                    for (int i = 0; i< path.size(); ++i)
                        img(static_cast<int>(path[i][0]), static_cast<int>(path[i][1])) = 255/paths.size() * (j+1) ;
                }
            }

            img.display("Grid values", false);

        }

       /**
       * Plots the value map included in a given grid and the given path. It is based on the
       * nDGridMap::getValue() which has to be float valued. This function has to be
       * overloaded in another value type is being used.
       *
       * Should be used only in 2D grids.
       *
       * The Y dimension flipping is because nDGridMap works in X-Y coordinates, not in image indices as CImg.
       *
       * IMPORTANT NOTE: no type-checkings are done. T type has to be Cell or any class with bool getValue() method.
       *
       * @param grid 2D nDGridmap
       * @param path 2D path to plot.
       * @param flipY true: flips the Y dimension. 0 does not flip.
       */
      template<class T, size_t ndims = 2>
	      static void plotArrivalTimesPath
	      (nDGridMap<T, ndims> & grid, const Path2D & path, const bool flipY = true) {
	          std::array<int,2> d = grid.getDimSizes();
	          double max_val = grid.getMaxValue();
	          if (!std::isfinite(max_val) || max_val <= 1e-9) {
	              max_val = 1.0;
	          }
	          CImg<double> img(d[0],d[1],1,1,0);

	          if (flipY)  {
	              // Filling the image flipping Y dim. We want now top left to be the (0,0).
	              cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getValue()/max_val*255; }

	              for (int i = 0; i< path.size(); ++i) {
	                  if (!std::isfinite(path[i][0]) || !std::isfinite(path[i][1])) {
	                      continue;
	                  }
	                  const int px = static_cast<int>(std::lround(path[i][0]));
	                  const int py = img.height() - static_cast<int>(std::lround(path[i][1])) - 1;
	                  if (px < 0 || px >= img.width() || py < 0 || py >= img.height()) {
	                      continue;
	                  }
	                  img(px, py) = 255;
	              }
	          }
	          else {
	              cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getValue()/max_val*255; }
	              for (int i = 0; i< path.size(); ++i) {
	                  if (!std::isfinite(path[i][0]) || !std::isfinite(path[i][1])) {
	                      continue;
	                  }
	                  const int px = static_cast<int>(std::lround(path[i][0]));
	                  const int py = static_cast<int>(std::lround(path[i][1]));
	                  if (px < 0 || px >= img.width() || py < 0 || py >= img.height()) {
	                      continue;
	                  }
	                  img(px, py) = 255;
	              }
	              }


          img.map( CImg<double>::jet_LUT256() );


          img.display("Grid values", false);


      }

    protected:

};



#endif /* GRIDPLOTTER_H_ */
