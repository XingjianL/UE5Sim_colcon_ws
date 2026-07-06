
#ifndef HITSCAN_HPP
#define HITSCAN_HPP
#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <rclcpp/rclcpp.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "open3d/Open3D.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "std_msgs/msg/float32_multi_array.hpp"

#include "tomato_xarm6/unique_point_cloud.hpp"
#include <atomic>
namespace tomato_xarm6
{
    class HitScan
    {
    public:
        HitScan(const std::string &node_name, double camera_FOV, int width, int height, const std::string &topic_name, bool reduce_file_size, rclcpp::Time creation_time);
        ~HitScan();

        void start();
        void stop();

        void update_intrinsics(double fov, int width, int height);
        int capture_count_ = 0;
        std::shared_ptr<open3d::geometry::PointCloud> CurrentFrameSemanticPCD(
            uint8_t semantic_id,
            const Eigen::Matrix4d &apply_transform = Eigen::Matrix4d::Identity()
        );
        void GenerateTracedInstance(
            const std::vector<uint8_t>& semantic_id,
            const std::string &save_prefix = "",
            const Eigen::Matrix4d &apply_transform = Eigen::Matrix4d::Identity(),
            const size_t trace_limit = 2000
        );
        void clear_images();

        std::queue<cv::Mat> image_queue_;
        std::mutex image_queue_mutex_;

        cv::Mat cv_img_;
        cv::Mat cv_img_segment_;
        cv::Mat cv_img_depth_;

        cv::Mat cv_generated_instance;
        rclcpp::Time creation_time_;
        void waiting_for_sync();
        void reset();
        bool under_recon_;
        std::atomic<bool> waiting_rayinfo = true;
        std::mutex waiting_msg_mutex;

        bool reduce_file_size_ = true;
        
    private:
        std::string topic_name_;
        std::set<std::tuple<uchar, uchar, uchar>> uniqueColors_;
        open3d::camera::PinholeCameraIntrinsic intrinsics_;

        rclcpp::Node::SharedPtr node_;
        void RayInfoCallback(const std_msgs::msg::Float32MultiArray& msg);

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rayinfo_sub;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr raydest_pub;
        std_msgs::msg::Float32MultiArray pub_msg;
        std_msgs::msg::Float32MultiArray sub_msg;
        std::thread executor_thread_;
        rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
        void build_publish_message(const std::vector<Eigen::Vector3d> &destination_coords);
        void process_rayinfo();
        std::vector<uint8_t> tomato_instance_ray;
        std::vector<uint8_t> tomato_instance_size_ray;
    };
}
#endif