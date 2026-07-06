#include "tomato_xarm6/hitscan.hpp"
#include <filesystem>
//#include "image_subscribe.hpp"

namespace tomato_xarm6 {
    HitScan::HitScan(const std::string &node_name, double camera_FOV, int width, int height, const std::string &topic_name, bool reduce_file_size, rclcpp::Time creation_time) 
    {
        creation_time_ = creation_time;
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
            rclcpp::ExecutorOptions()
        );

        topic_name_ = topic_name;

        node_ = std::make_shared<rclcpp::Node>(
            node_name,
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );
        intrinsics_ = open3d::camera::PinholeCameraIntrinsic(
            640, 480, 272.868229679, 272.868229679, 320, 240
        );
        reduce_file_size_ = reduce_file_size;
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();
        rayinfo_sub = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            "ue5/"+topic_name_+"/RayInfo",
            10,
            std::bind(&HitScan::RayInfoCallback, this, std::placeholders::_1)
        );
        raydest_pub =  node_->create_publisher<std_msgs::msg::Float32MultiArray>(
            "ue5/"+topic_name_+"/RayDest",
            10
        );

        under_recon_ = false;
        update_intrinsics(camera_FOV, width, height);
    }

    HitScan::~HitScan() 
    {
        stop();
    }

    void HitScan::start()
    {
        executor_->add_node(node_);

        // Run the executor in its own internal thread pool
        executor_thread_ = std::thread([this]() {
            executor_->spin();
        });
    }

    void HitScan::stop()
    {
        executor_->cancel();
        if (executor_thread_.joinable()) {
           executor_thread_.join();
        }
        //video_writer_.release();
        //cv::destroyAllWindows();
    }

    void HitScan::update_intrinsics(double fov, int width, int height)
    {
        //video_writer_.release();
        //video_writer_ = cv::VideoWriter("output.mp4", cv::VideoWriter::fourcc('X', '2', '6', '4'), 30, cv::Size(width, height));

        double fx = (width / tan((fov*M_PI/180.0)/2.0)) / 2;
        double fy = fx;
        RCLCPP_DEBUG(node_->get_logger(), "fx: %f", fx);
        intrinsics_ = open3d::camera::PinholeCameraIntrinsic(width, height, fx, fy, width / 2, height / 2);
    }
    void HitScan::reset()
    {
        //std::unique_lock<std::mutex> lock(waiting_msg_mutex);

        waiting_rayinfo = true;
        rclcpp::sleep_for(std::chrono::milliseconds(300));
        
        //lock.lock();
    }
    void HitScan::waiting_for_sync()
    {
        //clear_images();
        
        //std::unique_lock<std::mutex> lock(waiting_msg_mutex);

        while(waiting_rayinfo.load()){
            //RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "waiting for sync - RayInfo");
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "waiting for sync - RayInfo");
            //lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            //executor_->spin_some();
            //lock.lock();
        }
    }
    void HitScan::clear_images()
    {
        cv_img_.release();
        cv_img_segment_.release();
        cv_img_depth_.release();
    }

    std::shared_ptr<open3d::geometry::PointCloud> HitScan::CurrentFrameSemanticPCD(
            uint8_t semantic_id,
            const Eigen::Matrix4d &apply_transform
        ) 
    {
        auto rgb_image = cv_img_.clone();
        auto seg_image = cv_img_segment_.clone();
        auto depth_cmeters = cv_img_depth_.clone();
        if (reduce_file_size_) {
            depth_cmeters.convertTo(depth_cmeters, CV_32FC1, 0.1);
        }
        auto pcd = UniquePointCloud::GeneratePointCloud(
            depth_cmeters, seg_image, rgb_image, 
            semantic_id, intrinsics_, apply_transform);
        RCLCPP_INFO(node_->get_logger(), "PCD points: %ld", pcd->points_.size());

        return pcd;
    }

    void HitScan::GenerateTracedInstance(
            const std::vector<uint8_t>& semantic_id,
            const std::string &save_prefix,
            const Eigen::Matrix4d &apply_transform,
            const size_t trace_limit
        ) 
    {
        // clone in case of changes
        auto rgb_image = cv_img_.clone();
        auto seg_image = cv_img_segment_.clone();
        auto depth_cmeters = cv_img_depth_.clone();
        if (reduce_file_size_) {
            depth_cmeters.convertTo(depth_cmeters, CV_32FC1, 0.1);
        }
        depth_cmeters += 1.0;
        // obtain the destination points
        auto temp_pcd = std::make_shared<open3d::geometry::PointCloud>();        
        for (uint8_t id : semantic_id){
            auto pcd = UniquePointCloud::GeneratePointCloud(
                depth_cmeters, seg_image, rgb_image, 
                id, intrinsics_, apply_transform);
            *temp_pcd += *pcd;
        }
        RCLCPP_INFO(node_->get_logger(), "Got PCDs");
        //tomato_instance_ray.clear();

        // downsample temp_pcd to reduce number of raycasts
        if (temp_pcd->points_.size() > trace_limit){
            RCLCPP_WARN(node_->get_logger(), "Downsampling the pcd to reduce number of traces from %ld to %ld",temp_pcd->points_.size(), trace_limit);
            temp_pcd = temp_pcd->FarthestPointDownSample(trace_limit);
            // auto size_before = temp_pcd->points_.size();
            // temp_pcd = temp_pcd->VoxelDownSample(0.005);
            //RCLCPP_WARN(node_->get_logger(), "Downsampling the pcd to reduce number of traces from %ld to %ld",size_before, temp_pcd->points_.size());
        }
        // publish destination
        build_publish_message(temp_pcd->points_);
        
        raydest_pub->publish(pub_msg); // ideally this should be action, but ROSIntegration seems to be finicky about it so using topic and polling for response instead
        RCLCPP_INFO(node_->get_logger(), "Published dest ray traces");
        //rclcpp::sleep_for(std::chrono::milliseconds(1000));
        waiting_rayinfo = true;

        while (tomato_instance_ray.size() != temp_pcd->points_.size()){
            // error incomplete rayinfo 
            //tomato_instance_ray.clear();
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Unequal amount of raytrace destination and received: %ld, %ld", tomato_instance_ray.size(),temp_pcd->points_.size());
            waiting_for_sync(); // feedback received
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Got traced feedback");
            process_rayinfo();
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Processed traced feedback");
        }
        temp_pcd->Transform(apply_transform.inverse());
        cv_generated_instance = cv::Mat::zeros(rgb_image.rows, rgb_image.cols, CV_8UC3);
        for (size_t i = 0; i<tomato_instance_ray.size(); i++){
            const Eigen::Vector3d& P = temp_pcd->points_[i]; // (X, Y, Z) in Camera Frame
            double X = P(0), Y = P(1), Z = P(2);
            uint8_t instance_id = tomato_instance_ray[i];

            // 3D to 2D Projection Check:
            if (Z > 0.0) { // Must be in front of the camera
                // Apply the pinhole projection formula
                int u = (int)std::round((X * intrinsics_.GetFocalLength().first) / Z + intrinsics_.GetPrincipalPoint().first); // column index
                int v = (int)std::round((Y * intrinsics_.GetFocalLength().second) / Z + intrinsics_.GetPrincipalPoint().second); // row index

                // Check if the pixel is within image bounds
                if (u >= 0 && u < rgb_image.cols && v >= 0 && v < rgb_image.rows) {
                    // Assign the instance ID to the pixel
                    cv_generated_instance.at<cv::Vec3b>(v, u) = cv::Vec3b(instance_id, tomato_instance_size_ray[i], 255);
                } else {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "out of image bounds");
                }
            } else {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Z negative");
            }
        }
        capture_count_ += 1;
        cv::imwrite("output/robot/"+save_prefix+"images_"+topic_name_+"_"+std::to_string(creation_time_.seconds())+"/seg_instance_" + std::to_string(capture_count_) + ".png", cv_generated_instance);
        
    }

    void HitScan::build_publish_message(const std::vector<Eigen::Vector3d> &destination_coords) 
    {
        size_t size_dim1 = destination_coords.size();
        size_t size_total = size_dim1 * 3;
        
        pub_msg.layout.dim.clear();
        pub_msg.data.clear();

        std_msgs::msg::MultiArrayDimension dim1;
        dim1.label = "num_destinations";
        dim1.size = size_dim1;
        dim1.stride = size_total;
        pub_msg.layout.dim.push_back(dim1);

        std_msgs::msg::MultiArrayDimension dim2;
        dim2.label = "destinaton"; // A descriptive name
        dim2.size = 3;
        dim2.stride = 3;    // The step size to move to the next vector's component (3)
        pub_msg.layout.dim.push_back(dim2);

        pub_msg.layout.data_offset = 0;

        pub_msg.data.reserve(size_total);

        // Iterate through the input vector of 3D points
        for (const auto& vec : destination_coords) {
            // Flatten the vector and cast from double (Eigen default) to float
            pub_msg.data.push_back(static_cast<float>(vec(0))); // X
            pub_msg.data.push_back(static_cast<float>(vec(1))); // Y
            pub_msg.data.push_back(static_cast<float>(vec(2))); // Z
        }
    }

    void HitScan::RayInfoCallback(const std_msgs::msg::Float32MultiArray& msg)
    {
        {
            std::lock_guard<std::mutex> lock(waiting_msg_mutex);
            sub_msg = msg;
        }
        sub_msg = msg;
        waiting_rayinfo = false;
        //RCLCPP_INFO(node_->get_logger(), "RayInfoCallback");
    }
    void HitScan::process_rayinfo()
    {
        std_msgs::msg::Float32MultiArray msg_copy;
        {
            std::lock_guard<std::mutex> lock(waiting_msg_mutex);
            msg_copy=sub_msg;
        }
        tomato_instance_ray.clear();
        tomato_instance_ray.resize(msg_copy.layout.dim[0].stride/msg_copy.layout.dim[0].size, 255);
        tomato_instance_size_ray.clear();
        tomato_instance_size_ray.resize(msg_copy.layout.dim[0].stride/msg_copy.layout.dim[0].size, 255);
        for (size_t i = 0; i < msg_copy.data.size(); i+= msg_copy.layout.dim[0].size) {
            size_t pcd_idx = static_cast<size_t>(std::round(msg_copy.data[i]));
            if (tomato_instance_ray[pcd_idx] != 255) {
                RCLCPP_WARN(node_->get_logger(), "hitscan idx duplicated (UE5 issue)");
            }
            tomato_instance_ray[pcd_idx] = msg_copy.data[i+1];

            // tomato_instance_ray.push_back(sub_msg.data[i+1]);
            if (i+2 < msg_copy.data.size()) {
                if (msg_copy.data[i+2]*30 > 255) {
                    RCLCPP_WARN(node_->get_logger(), "Received tomato radius larger than 255 mm");
                }
                tomato_instance_size_ray[pcd_idx]=((uint8_t)(msg_copy.data[i+2]*30));
            }
        }
    }
}