#ifndef UNIQUEPOINTCLOUD_HPP
#define UNIQUEPOINTCLOUD_HPP
#pragma once
#include <opencv2/opencv.hpp>
#include "open3d/Open3D.h"
namespace tomato_xarm6
{
    class UniquePointCloud
    {
    public:
        UniquePointCloud();
        UniquePointCloud(std::tuple<uint8_t, uint8_t, uint8_t> segment_color);//, open3d::camera::PinholeCameraIntrinsic intrinsics);
        ~UniquePointCloud();
        std::shared_ptr<open3d::geometry::PointCloud> o3d_pc;
        std::tuple<uint8_t, uint8_t, uint8_t> segment_color;
        Eigen::Matrix4d base_transform;
        //open3d::camera::PinholeCameraIntrinsic intrinsics_;

        bool buildPointCloud(
            cv::Mat &depth_img, cv::Mat &segment_img, cv::Mat &color_img, 
            std::tuple<uint8_t, uint8_t, uint8_t> color, 
            const open3d::camera::PinholeCameraIntrinsic &intrinsics_,
            std::string& save_intermediate, 
            const Eigen::Matrix4d &apply_transform = Eigen::Matrix4d::Identity());
        static std::shared_ptr<open3d::geometry::PointCloud> GeneratePointCloud(
            cv::Mat &depth_img, cv::Mat &segment_img, cv::Mat &color_img, 
            uint8_t semantic, 
            const open3d::camera::PinholeCameraIntrinsic &intrinsics_,
            const Eigen::Matrix4d &apply_transform = Eigen::Matrix4d::Identity());
        void savePointCloud(
            std::string& filepath, 
            bool apply_base_transform = false
        );

    private:
        bool appendPointCloud(std::shared_ptr<open3d::geometry::PointCloud> pc, std::tuple<uint8_t, uint8_t, uint8_t> color);
        std::string filename_;
        uint32_t append_count_ = 0;
        bool pc_updated;
    };
}

#endif