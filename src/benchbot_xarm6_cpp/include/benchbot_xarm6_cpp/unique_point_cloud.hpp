#ifndef UNIQUEPOINTCLOUD_HPP
#define UNIQUEPOINTCLOUD_HPP
#pragma once
#include <opencv2/opencv.hpp>
#include "open3d/Open3D.h"
namespace benchbot_xarm6
{
    class UniquePointCloud
    {
    public:
        UniquePointCloud();
        UniquePointCloud(std::tuple<uint8_t, uint8_t, uint8_t> segment_color, open3d::camera::PinholeCameraIntrinsic intrinsics);
        ~UniquePointCloud();
        std::shared_ptr<open3d::geometry::PointCloud> o3d_pc;
        std::tuple<uint8_t, uint8_t, uint8_t> segment_color;

        open3d::camera::PinholeCameraIntrinsic intrinsics;

        bool buildPointCloud(cv::Mat &depth_img, cv::Mat &segment_img, cv::Mat &color_img, std::tuple<uint8_t, uint8_t, uint8_t> color, Eigen::Matrix4d transform);
        void savePointCloud( std::string& filepath);

    private:
        bool appendPointCloud(std::shared_ptr<open3d::geometry::PointCloud> pc, std::tuple<uint8_t, uint8_t, uint8_t> color);

    };
}

#endif