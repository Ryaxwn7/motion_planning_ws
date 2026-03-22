#ifndef GRIDPLOTTER_H_
#define GRIDPLOTTER_H_

#include <string>
#include <array>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "../ndgridmap/ndgridmap.hpp"

typedef typename std::array<int, 2> Coord2D;
typedef typename std::array<double, 2> Point2D;
typedef typename std::vector<Point2D> Path2D;
typedef typename std::vector<std::shared_ptr<Path2D>> Paths2D;

class GridPlotterCV {
public:
    GridPlotterCV() {};
    virtual ~GridPlotterCV() {};

    template<class T, size_t ndims>
    static void plotMap(nDGridMap<T, ndims>& grid, bool flipY = true) {
        std::array<int, 2> d = grid.getDimSizes();
        cv::Mat img(d[1], d[0], CV_8UC1); // OpenCV Mat is row-major (height, width)

        // Fill the image
        for(int y = 0; y < d[1]; ++y) {
            uchar* p = img.ptr<uchar>(flipY ? (d[1] - y - 1) : y);
            for(int x = 0; x < d[0]; ++x) {
                p[x] = grid[y * d[0] + x].getOccupancy() ? 255 : 0;
            }
        }

        cv::imshow("Grid Map", img);
        cv::imwrite("/home/bsrl-ubuntu/exp_graphs/grid_map.png", img);
        saveDataForMatlab(grid, "/home/bsrl-ubuntu/exp_graphs/grid_data.csv");
    }

    template<class T, size_t ndims = 2>
    static double getMaxVelocity(nDGridMap<T, ndims> & grid) {
        double max = 0;
        double temp;
        for(int i = 0; i < grid.size(); i++) {
            temp = grid.getCell(i).getVelocity();
            if (temp > max)
                max = temp;    
        }
        return max;
    }


    template<class T, size_t ndims = 2>
    static void plotVelocityMap(nDGridMap<T, ndims>& grid, bool flipY = true) {
        std::array<int, 2> d = grid.getDimSizes();
        cv::Mat velocityMap(d[1], d[0], CV_32FC1); // OpenCV使用(height, width)顺序

        // 获取最大速度值
        float maxVelocity = GridPlotterCV::getMaxVelocity(grid);
        
        // 填充速度数据
        for(int y = 0; y < d[1]; ++y) {
            float* row = velocityMap.ptr<float>(flipY ? (d[1] - y - 1) : y);
            for(int x = 0; x < d[0]; ++x) {
                row[x] = grid[y * d[0] + x].getVelocity();
            }
        }

        // 归一化和应用颜色映射
        cv::Mat normalized, colorMap;
        cv::normalize(velocityMap, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::applyColorMap(normalized, colorMap, cv::COLORMAP_JET);

        // 显示和保存结果
        cv::imshow("Velocity Field", colorMap);
        cv::imwrite("/home/bsrl-ubuntu/exp_graphs/velocity_field.png", colorMap);
        saveDataForMatlab(grid, "/home/bsrl-ubuntu/exp_graphs/velocity_data.csv");
    }

    template<class T, size_t ndims = 2>
    static void plotArrivalTimes(nDGridMap<T, ndims>& grid, bool flipY = true) {
        std::array<int, 2> d = grid.getDimSizes();
        cv::Mat img(d[1], d[0], CV_32FC1);
        
        // Find max value and fill matrix
        float maxVal = grid.getMaxValue();
        for(int y = 0; y < d[1]; ++y) {
            float* p = img.ptr<float>(flipY ? (d[1] - y - 1) : y);
            for(int x = 0; x < d[0]; ++x) {
                p[x] = grid[y * d[0] + x].getValue();
            }
        }

        // Normalize and apply color map
        cv::Mat normalized, colorMap;
        cv::normalize(img, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::applyColorMap(normalized, colorMap, cv::COLORMAP_JET);

        cv::imshow("Arrival Times", colorMap);
        cv::imwrite("/home/bsrl-ubuntu/exp_graphs/arrival_times.png", colorMap);
        saveDataForMatlab(grid, "/home/bsrl-ubuntu/exp_graphs/arrival_data.csv");
    }

    

    template<class T, size_t ndims = 2>
    static void plotMapPath(nDGridMap<T, ndims>& grid, const Path2D& path, bool flipY = true) {
        std::array<int, 2> d = grid.getDimSizes();
        cv::Mat img(d[1], d[0], CV_8UC3, cv::Scalar(0, 0, 0));

        // Draw map
        for(int y = 0; y < d[1]; ++y) {
            cv::Vec3b* p = img.ptr<cv::Vec3b>(flipY ? (d[1] - y - 1) : y);
            for(int x = 0; x < d[0]; ++x) {
                uchar val = grid[y * d[0] + x].getOccupancy() ? 255 : 0;
                p[x] = cv::Vec3b(val, val, val);
            }
        }

        // Draw path
        for(const auto& p : path) {
            int imgY = flipY ? (d[1] - 1 - static_cast<int>(p[1])) : static_cast<int>(p[1]);
            cv::circle(img, cv::Point(p[0], imgY), 1, cv::Scalar(0, 0, 255), -1);
        }

        cv::imshow("Map with Path", img);
        cv::imwrite("/home/bsrl-ubuntu/exp_graphs/map_path.png", img);
    }

    template<class T, size_t ndims = 2>
    static void plotMapPath(nDGridMap<T, ndims>& grid, const Paths2D& paths, bool flipY = true) {
        std::array<int, 2> d = grid.getDimSizes();
        cv::Mat img(d[1], d[0], CV_8UC3, cv::Scalar(0, 0, 0)); // 使用BGR格式，尺寸(height, width)

        // 定义6种暗色 (BGR格式)
        const cv::Scalar dark_colors[6] = {
            cv::Scalar(0, 0, 51),    // Dark Red (BGR)
            cv::Scalar(0, 51, 0),    // Dark Green
            cv::Scalar(51, 0, 0),    // Dark Blue
            cv::Scalar(0, 51, 51),   // Dark Yellow
            cv::Scalar(51, 51, 0),   // Dark Cyan
            cv::Scalar(51, 0, 51)    // Dark Magenta
        };

        // 绘制占据地图
        for(int y = 0; y < d[1]; ++y) {
            for(int x = 0; x < d[0]; ++x) {
                bool occupied = grid[y * d[0] + x].getOccupancy();
                int imgY = flipY ? (d[1] - 1 - y) : y;
                img.at<cv::Vec3b>(imgY, x) = occupied ? cv::Vec3b(255, 255, 255) : cv::Vec3b(0, 0, 0);
            }
        }

        // 绘制路径
        for(size_t j = 0; j < paths.size(); ++j) {
            const Path2D& path = *paths[j];
            const cv::Scalar& color = dark_colors[j % 6];
            
            std::vector<cv::Point> cvPoints;
            for(const auto& p : path) {
                int imgX = static_cast<int>(p[0]);
                int imgY = static_cast<int>(p[1]);
                if(flipY) imgY = d[1] - 1 - imgY;
                cvPoints.emplace_back(imgX, imgY);
            }
            
            // 绘制抗锯齿路径
            if(!cvPoints.empty()) {
                const cv::Point* pts = (const cv::Point*)cv::Mat(cvPoints).data;
                int npts = cv::Mat(cvPoints).rows;
                cv::polylines(img, &pts, &npts, 1, false, color, 1, cv::LINE_AA);
            }
        }

        // 显示和保存结果
        cv::imshow("Map with Paths", img);
        cv::imwrite("/home/bsrl-ubuntu/exp_graphs/map_paths.png", img);
    }

    template<class T, size_t ndims = 2>
    static void saveDataForMatlab(nDGridMap<T, ndims>& grid, const std::string& filename) {
        std::ofstream file(filename);
        std::array<int, 2> d = grid.getDimSizes();
        
        for(int y = 0; y < d[1]; ++y) {
            for(int x = 0; x < d[0]; ++x) {
                file << grid[y * d[0] + x].getValue();
                if(x != d[0]-1) file << ",";
            }
            file << "\n";
        }
        file.close();
    }

protected:
    // Helper function for path drawing
    static void drawPath(cv::Mat& img, const Path2D& path, const cv::Scalar& color, bool flipY) {
        std::vector<cv::Point> cvPoints;
        for(const auto& p : path) {
            int imgY = flipY ? (img.rows - 1 - static_cast<int>(p[1])) : static_cast<int>(p[1]);
            cvPoints.emplace_back(static_cast<int>(p[0]), imgY);
        }
        cv::polylines(img, cvPoints, false, color, 1);
    }
};

#endif /* GRIDPLOTTER_H_ */