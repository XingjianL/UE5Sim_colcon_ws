
#ifndef ENVIRONMENT_INFO_HPP
#define ENVIRONMENT_INFO_HPP
#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "tomato_xarm6/unique_point_cloud.hpp"
#include "tomato_xarm6/image_subscribe.hpp"

namespace tomato_xarm6
{
    void ParseUE5TransformString(std::string& transform, double (&result)[9]);
    extern const Eigen::Matrix4d UnrealToBlender;
    struct SemanticInfo
    {
        SemanticInfo();
        uint32_t semantic_id;
        uint8_t r;
        uint8_t g;
        uint8_t b;
        std::string semantic_name;
        std::string id_str;
        void ParseData(const std::string& data);
        void UpdateLog();
        bool operator==(const SemanticInfo& rhs) const;
        std::string csv_header;
        std::string csv_data;
    };
    struct PlantInfo
    {
        PlantInfo();

        rclcpp::Time creation_time;

        std::string transforms;
        uint32_t id;
        std::string plant_name;
        std::string plant_variant;
        uint8_t instance_segmentation_id_g;
        uint8_t instance_segmentation_id_b;
        int seedP;
        int seedL;
        std::string leaves;

        bool operator==(const PlantInfo& rhs) const;
        bool isInstance(uint8_t id_g, uint8_t id_b);
        std::vector<UniquePointCloud> unique_point_clouds;
        int GetPointCloudOfSemantic(int semantic_label);
        std::shared_ptr<open3d::geometry::PointCloud> combined_point_cloud;

        void ParseData(const std::string& data);

        std::string csv_header;
        std::string csv_data;
        void UpdateLog();
        void CombinePointClouds();
    };
    struct RobotInfo
    {
        RobotInfo();

        rclcpp::Time creation_time;

        std::string name;
        std::string topic_name;
        double base_transforms[9];
        double camera_transforms[9];
        double camera_quaternions[4];
        
        double camera_FOV;
        int camera_width;
        int camera_height;

        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr robot_transforms_subscriber_;
        void ConfigTFSubscriber(rclcpp::Node::SharedPtr node);
        void TFCallback(const tf2_msgs::msg::TFMessage::ConstSharedPtr& msg);
        bool should_update = false;
        void UpdateRobotStatus(const std::string &data);
        void UpdateLog();
        void WriteVideo();
        std::shared_ptr<tomato_xarm6::ImageSubscriber> image_subscriber;
        std::vector<std::shared_ptr<open3d::geometry::PointCloud>> GetCurrentPointCloud();
        Eigen::Matrix4d ComputeUE5CameraTransform();
        bool operator==(const RobotInfo& rhs) const;
        
        void ConfigCamera(const std::string &rgbd_node_name, const std::string &stereo_node_name, bool capture_both, bool reduce_file_size);
        void ParseData(const std::string& data);

        std::string csv_header;
        std::string csv_data;
        
    };

    class EnvironmentInfo
    {
    public:
        EnvironmentInfo(rclcpp::Node::SharedPtr node);
        ~EnvironmentInfo();

        void waiting_for_sync();
        bool waiting_msg = true;

        std::vector<SemanticInfo> semantic_info_;

        std::vector<RobotInfo> robot_info_;

        std::vector<PlantInfo> plant_info_;
        std::ofstream plant_log_file_;

        void clear();
        void BuildPointClouds(bool save_intermediate, const std::string& robot_name, const std::string& frame_tf_lookup);
        void SavePointClouds();

        void UpdateLog();
        void SaveLog();

        void StartRobotCamera(const std::string& robot_name, const std::string &rgbd_node_name, const std::string &stereo_node_name, bool capture_both, bool reduce_file_size);
        void StopRobotCamera(const std::string& robot_name);

        void SaveRobotImages(std::vector<std::string> robot_names, bool wait_for_sync_);        
        void EnvPublishCommand(const std::string& command);\

        int GetRobotID(const std::string& robot_name);
        //open3d::visualization::Visualizer visualizer;
        std::string save_prefix = "";
        rclcpp::Time creation_time_;
    private:
        const std::string PLANTMARKER = "Tomato";
        const std::string ROBOTMARKER = "Robot";
        const std::string SEMANTICMARKER = "semantic_map_";
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Node::SharedPtr node_;
        void EnvStringCallback(const std_msgs::msg::String::ConstSharedPtr& msg);

        void ParseData(const std::string& data, std::vector<RobotInfo>& robot_info, std::vector<PlantInfo>& plant_info);
        std::vector<std::string> SplitByDelimiter(const std::string& data, const std::string& delimiter);
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr environment_info_;

        int pc_build_count_;  

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr env_publisher;
        
        
        
    };
}
#endif