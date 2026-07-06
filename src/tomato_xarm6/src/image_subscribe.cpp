#include "tomato_xarm6/image_subscribe.hpp"
#include <filesystem>
//#include "image_subscribe.hpp"

namespace tomato_xarm6 {
    ImageSubscriber::ImageSubscriber(
        const std::string &node_name_rgbd,
        const std::string &node_name_stereo,
        double camera_FOV, int width, int height,
        bool capture_both, std::string &topic_name, bool reduce_file_size
    ) : capture_both_(capture_both), reduce_file_size_(reduce_file_size)
    {
        topic_name_ = topic_name;

        // ===== RGBD Node =====
        node_rgbd_ = std::make_shared<rclcpp::Node>(
            node_name_rgbd,
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );

        rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
        qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

        sync_rgbd_color_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();
        sync_rgbd_depth_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();
        sync_rgbd_segment_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();

        sync_rgbd_color_->subscribe(node_rgbd_, "/ue5/"+topic_name+"/Color", qos);
        sync_rgbd_depth_->subscribe(node_rgbd_, "/ue5/"+topic_name+"/Depth", qos);
        sync_rgbd_segment_->subscribe(node_rgbd_, "/ue5/"+topic_name+"/Segmentation", qos);

        rgbd_sync_ = std::make_shared<message_filters::Synchronizer<ApproxTimeSyncPolicyRGBD>>(
            ApproxTimeSyncPolicyRGBD(10),
            *sync_rgbd_color_, *sync_rgbd_segment_, *sync_rgbd_depth_
        );
        rgbd_sync_->registerCallback(
            std::bind(&ImageSubscriber::RGBDImageCallback, this, 
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
        );

        // Executor for RGBD node
        executor_rgbd_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();


        // ===== Stereo Node =====
        node_stereo_ = std::make_shared<rclcpp::Node>(
            node_name_stereo,
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );

        sync_sub_color_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();
        sync_sub_color1_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();
        sync_sub_depth_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();
        sync_sub_depth1_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>();

        sync_sub_color_->subscribe(node_stereo_, "/ue5/"+topic_name+"/Color", qos);
        sync_sub_color1_->subscribe(node_stereo_, "/ue5/"+topic_name+"/ColorOne", qos);
        sync_sub_depth_->subscribe(node_stereo_, "/ue5/"+topic_name+"/Depth", qos);
        sync_sub_depth1_->subscribe(node_stereo_, "/ue5/"+topic_name+"/DepthOne", qos);

        stereo_sync_ = std::make_shared<message_filters::Synchronizer<ApproxTimeSyncPolicyStereo>>(
            ApproxTimeSyncPolicyStereo(10),
            *sync_sub_color_, *sync_sub_color1_, *sync_sub_depth_, *sync_sub_depth1_
        );
        stereo_sync_->registerCallback(
            std::bind(&ImageSubscriber::StereoImageCallback, this, 
                    std::placeholders::_1, std::placeholders::_2, 
                    std::placeholders::_3, std::placeholders::_4)
        );

        // Executor for Stereo node
        executor_stereo_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        

        // Common setup
        update_intrinsics(camera_FOV, width, height);
        waiting_msg_rgbd = true;
        waiting_msg_stereo = true;
        under_recon_ = false;
    }


    ImageSubscriber::~ImageSubscriber() 
    {
        stop();
    }

    std::string ImageSubscriber::RGBImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
    {
        try 
        {
            cv_img_id_ = msg->header.frame_id;
            cv_img_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
            
            cv::cvtColor(cv_img_, rgb_image_, cv::COLOR_BGR2RGB);
        }
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(node_rgbd_->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
        return msg->header.frame_id;
    }
    std::string ImageSubscriber::DepthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
    {
        //std::cout << "imageCallback-Depth" << std::endl;
        try 
        {
            cv_img_depth_ = cv_bridge::toCvShare(msg, "32FC1")->image.clone();
            if (reduce_file_size_){
                cv_img_depth_.convertTo(cv_img_depth_, CV_16UC1,10);
            }
        }
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(node_rgbd_->get_logger(), "Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
        }
        return msg->header.frame_id;
    }

    std::string ImageSubscriber::SegmentImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
    {
        //std::cout << "imageCallback-Segmentation" << std::endl;
        RCLCPP_INFO(node_rgbd_->get_logger(), "SegFormat '%s'", msg->encoding.c_str());
        try 
        {
            cv_img_segment_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
            //cv_img_segment_ = cv_bridge::toCvShare(msg, "bgr16")->image.clone();
            // cv::Mat raw_fixed(
            //     msg->height, 
            //     msg->width, 
            //     CV_16FC4, 
            //     const_cast<uint8_t*>(&msg->data[0]), 
            //     msg->step
            // );
            // raw_fixed.convertTo(cv_img_segment_, CV_32FC4);
            uniqueColors_.clear();
            for (int y = 0; y < cv_img_segment_.rows; ++y) {
                for (int x = 0; x < cv_img_segment_.cols; ++x) {
                    cv::Vec3b color = cv_img_segment_.at<cv::Vec3b>(y, x);
                    uniqueColors_.insert(std::make_tuple(color[2], color[1], color[0])); // bgr -> rgb
                }
            }
        }
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(node_rgbd_->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());            
        }
        return msg->header.frame_id;
    }

    void ImageSubscriber::RGBDImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg_color, const sensor_msgs::msg::Image::ConstSharedPtr& msg_segment, const sensor_msgs::msg::Image::ConstSharedPtr& msg_depth)
    {
        //std::unique_lock<std::mutex> lock(waiting_msg_mutex);
        if (under_recon_) {
            waiting_msg_rgbd = false;
            //RCLCPP_INFO(node_->get_logger(), "Recon is under progress, not processing the received image.");
            return;
        }
        // if(capture_both_) {
        //     std::lock_guard<std::mutex> lock(callback_mutex);
        // }
        RCLCPP_INFO_THROTTLE(node_rgbd_->get_logger(), *node_rgbd_->get_clock(), 5000, "RGBDImageCallback - %s", topic_name_.c_str());
        {
            std::lock_guard<std::mutex> lock(callback_mutex);
            auto hashrgb = RGBImageCallback(msg_color);
            auto hashsem = SegmentImageCallback(msg_segment);
            auto hashdep = DepthImageCallback(msg_depth);
            if ((hashrgb != hashsem) || (hashrgb != hashdep)) {
                RCLCPP_WARN(node_rgbd_->get_logger(), "RGBDImageCallbackHASH - %s - %s - %s", hashrgb.c_str(), hashsem.c_str(), hashdep.c_str());
            } else {
                RCLCPP_INFO(node_rgbd_->get_logger(), "RGBDImageCallbackHASH - %s - %s - %s", hashrgb.c_str(), hashsem.c_str(), hashdep.c_str());
            }
        }
        
        waiting_msg_rgbd = false;
        
    }

    void ImageSubscriber::StereoImageCallback(
            const sensor_msgs::msg::Image::ConstSharedPtr& msg_color, 
            const sensor_msgs::msg::Image::ConstSharedPtr& msg_color1, 
            const sensor_msgs::msg::Image::ConstSharedPtr& msg_depth,
            const sensor_msgs::msg::Image::ConstSharedPtr& msg_depth1)
    {
        // if(capture_both_) {
        //     std::lock_guard<std::mutex> lock(callback_mutex);
        // }
        RCLCPP_INFO_THROTTLE(node_stereo_->get_logger(), *node_stereo_->get_clock(), 5000, "StereoImageCallback");
        {
            std::lock_guard<std::mutex> lock(callback_mutex);
            cv_img_ = cv_bridge::toCvShare(msg_color, "bgr8")->image.clone();
            cv_img1_ = cv_bridge::toCvShare(msg_color1, "bgr8")->image.clone();
            cv_img_depth_ = cv_bridge::toCvShare(msg_depth, "32FC1")->image.clone();
            cv_img1_depth_ = cv_bridge::toCvShare(msg_depth1, "32FC1")->image.clone();
            if (reduce_file_size_) {
                cv_img_depth_.convertTo(cv_img_depth_, CV_16UC1,10);
                cv_img1_depth_.convertTo(cv_img1_depth_, CV_16UC1,10);
            }
        }
        waiting_msg_stereo = false;
    }

    void ImageSubscriber::start()
    {
        executor_rgbd_->add_node(node_rgbd_);
        rgbd_thread_ = std::thread([this]() { executor_rgbd_->spin(); });
        executor_stereo_->add_node(node_stereo_);
        stereo_thread_ = std::thread([this]() { executor_stereo_->spin(); });
        // Give time for subscriptions and DDS to match
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        waiting_for_sync();
    }

    void ImageSubscriber::stop()
    {
        executor_rgbd_->cancel();
        if (rgbd_thread_.joinable()) rgbd_thread_.join();

        executor_stereo_->cancel();
        if (stereo_thread_.joinable()) stereo_thread_.join();
        //video_writer_.release();
        //cv::destroyAllWindows();
    }

    void ImageSubscriber::update_intrinsics(double fov, int width, int height)
    {
        //video_writer_.release();
        //video_writer_ = cv::VideoWriter("output.mp4", cv::VideoWriter::fourcc('X', '2', '6', '4'), 30, cv::Size(width, height));

        double fx = (width / tan((fov*M_PI/180.0)/2.0)) / 2;
        double fy = fx;
        RCLCPP_DEBUG(node_rgbd_->get_logger(), "fx: %f", fx);
        intrinsics_ = open3d::camera::PinholeCameraIntrinsic(width, height, fx, fy, width / 2, height / 2);
    }
    void ImageSubscriber::reset()
    {
        RCLCPP_DEBUG(node_rgbd_->get_logger(), "clearing image topics");
        rgbd_sync_.reset();
        
        stereo_sync_.reset();
        RCLCPP_DEBUG(node_stereo_->get_logger(), "clearing stereo topics");
        stereo_sync_ = std::make_shared<message_filters::Synchronizer<ApproxTimeSyncPolicyStereo>>(
            ApproxTimeSyncPolicyStereo(10),
            *sync_sub_color_, *sync_sub_color1_, *sync_sub_depth_, *sync_sub_depth1_
        );
        stereo_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.3));
        stereo_sync_->registerCallback(
            std::bind(&ImageSubscriber::StereoImageCallback, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,std::placeholders::_4));

        RCLCPP_DEBUG(node_rgbd_->get_logger(), "clearing rgbd topics");
        rgbd_sync_ = std::make_shared<message_filters::Synchronizer<ApproxTimeSyncPolicyRGBD>>(
            ApproxTimeSyncPolicyRGBD(10),
            *sync_rgbd_color_, *sync_rgbd_segment_, *sync_rgbd_depth_
        );
        rgbd_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.3));
        rgbd_sync_->registerCallback(
            std::bind(&ImageSubscriber::RGBDImageCallback, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        waiting_msg_rgbd = true;
        waiting_msg_stereo = true;
        rclcpp::sleep_for(std::chrono::milliseconds(300));
    }
    void ImageSubscriber::waiting_for_sync()
    {
        //clear_images();
        
        //std::unique_lock<std::mutex> lock(waiting_msg_mutex);

        while((capture_both_ && (waiting_msg_rgbd.load() || waiting_msg_stereo.load())) ||
            (!capture_both_ && (waiting_msg_rgbd.load() && waiting_msg_stereo.load()))){
            RCLCPP_INFO_THROTTLE(node_rgbd_->get_logger(), *node_rgbd_->get_clock(), 2000, "waiting for sync - Image: %d", capture_both_);
            //lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            //executor_->spin_some();
            //lock.lock();
        }
    }
    void ImageSubscriber::clear_images()
    {
        rgb_image_.release();
        cv_img1_.release();
        cv_img_.release();
        cv_img1_depth_.release();
        cv_img_segment_.release();
        cv_img_depth_.release();
    }

    bool ImageSubscriber::process_to_pc(
        std::vector<UniquePointCloud>& unique_pcs, 
        uint8_t instance_id_g, uint8_t instance_id_b,
        std::string& save_intermediate,
        const Eigen::Matrix4d &apply_transform
        //open3d::visualization::Visualizer& visualizer
        )
    {
        if (rgb_image_.empty() || cv_img_segment_.empty() || cv_img_depth_.empty()) {
            RCLCPP_WARN(node_rgbd_->get_logger(), "Images empty while reconstructing for instance: %d, %d", instance_id_g, instance_id_b);
            return false;
        }

        auto rgb_image = rgb_image_.clone();
        auto seg_image = cv_img_segment_.clone();
        auto depth_cmeters = cv_img_depth_.clone();
        if (reduce_file_size_) {
            depth_cmeters.convertTo(depth_cmeters, CV_32FC1, 0.1);
        }
        // build point cloud for each unique color
        for (auto uniqueColor : uniqueColors_)
        {
            bool added = false;
            if (std::get<2>(uniqueColor) != instance_id_b || std::get<1>(uniqueColor) != instance_id_g) {
                //RCLCPP_INFO(node_->get_logger(), "color %d %d %d not for instance %d", std::get<0>(uniqueColor), std::get<1>(uniqueColor), std::get<2>(uniqueColor), instance_id);
                continue;
            }
            for (auto& unique_pc : unique_pcs) {
                
                added = unique_pc.buildPointCloud(
                    depth_cmeters, seg_image, rgb_image, 
                    uniqueColor, 
                    intrinsics_,
                    save_intermediate,
                    apply_transform
                    );
                if (added) 
                {
                    // RCLCPP_INFO(node_->get_logger(), 
                    //     "color %d %d %d added for instance %d %d", 
                    //     std::get<0>(uniqueColor), std::get<1>(uniqueColor), std::get<2>(uniqueColor), 
                    //     instance_id_g, instance_id_b);
                    break;
                }
            }
            // new point cloud if not added (point cloud with same color doesn't exist)
            if (!added)
            {
                RCLCPP_INFO(node_rgbd_->get_logger(), "adding new point cloud for color: %d %d %d", std::get<0>(uniqueColor), std::get<1>(uniqueColor), std::get<2>(uniqueColor));
                unique_pcs.push_back(tomato_xarm6::UniquePointCloud(uniqueColor));
                added = unique_pcs.back().buildPointCloud(
                    depth_cmeters, seg_image, rgb_image, 
                    uniqueColor, 
                    intrinsics_,
                    save_intermediate,
                    apply_transform
                    );
            }
            //RCLCPP_INFO(node_->get_logger(), "num of point clouds: %ld", o3d_pc_vector_.size());
            //convert_to_ros_pointcloud(*o3d_pc, ros_pc, callback_time);
        }
        return true;
    }
    std::shared_ptr<open3d::geometry::PointCloud> ImageSubscriber::CurrentFrameSemanticPCD(
            uint8_t semantic_id,
            const Eigen::Matrix4d &apply_transform
        ) 
    {
        auto rgb_image = rgb_image_.clone();
        auto seg_image = cv_img_segment_.clone();
        auto depth_cmeters = cv_img_depth_.clone();
        if (reduce_file_size_) {
            depth_cmeters.convertTo(depth_cmeters, CV_32FC1, 0.1);
        }
        auto pcd = UniquePointCloud::GeneratePointCloud(
            depth_cmeters, seg_image, rgb_image, 
            semantic_id, intrinsics_, apply_transform);
        RCLCPP_INFO(node_rgbd_->get_logger(), "PCD points: %ld", pcd->points_.size());

        return pcd;
    }


    void ImageSubscriber::save_images(std::string path)
    {
        cv::Mat rgb_copy, rgb1_copy, seg_copy, depth_copy, depth1_copy;
        {
            std::lock_guard<std::mutex> lock(callback_mutex);
            if (!cv_img_.empty()) rgb_copy = cv_img_.clone();
            if (!cv_img1_.empty()) rgb1_copy = cv_img1_.clone();
            if (!cv_img_segment_.empty()) seg_copy = cv_img_segment_.clone();
            if (!cv_img_depth_.empty()) depth_copy = cv_img_depth_.clone();
            if (!cv_img1_depth_.empty()) depth1_copy = cv_img1_depth_.clone();
        }
        std::filesystem::create_directories(path);
        std::stringstream ss;
        ss << std::hex << std::setw(8) << std::setfill('0') << cv_img_id_; // hex, 8 chars

        std::string cv_img_id_str = ss.str();
        if (!rgb_copy.empty()){
            if (reduce_file_size_) {
                std::vector<int> compression_params;
                compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
                compression_params.push_back(95);
                cv::imwrite(path + "/rgb_" + std::to_string(capture_count_) +"_"+ cv_img_id_str + ".jpg", rgb_copy, compression_params);
            } else {
                cv::imwrite(path + "/rgb_" + std::to_string(capture_count_) +"_"+ cv_img_id_str + ".png", rgb_copy);
            }
        }
        if (!rgb1_copy.empty()){
            if (reduce_file_size_) {
                std::vector<int> compression_params;
                compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
                compression_params.push_back(95);
                cv::imwrite(path + "/rgb1_" + std::to_string(capture_count_) + ".jpg", rgb1_copy, compression_params);
            } else {
                cv::imwrite(path + "/rgb1_" + std::to_string(capture_count_) + ".png", rgb1_copy);
            }        }
        if (!seg_copy.empty()){
            cv::imwrite(path + "/seg_" + std::to_string(capture_count_) + ".png", seg_copy);
        }
        if (!depth_copy.empty()){
            if (reduce_file_size_) {
                cv::imwrite(path + "/depth_" + std::to_string(capture_count_) + ".png", depth_copy);
            }
            else {
                cv::imwrite(path + "/depth_" + std::to_string(capture_count_) + ".exr", depth_copy);
            }
        }
        if (!depth1_copy.empty()){
            if (reduce_file_size_) {
                cv::imwrite(path + "/depth1_" + std::to_string(capture_count_) + ".png", depth1_copy);
            }
            else {
                cv::imwrite(path + "/depth1_" + std::to_string(capture_count_) + ".exr", depth1_copy);
            }
        }
        std::cout << "Saved saved: " << cv_img_id_str << std::endl;
    }

    void ImageSubscriber::convert_to_ros_pointcloud(const open3d::geometry::PointCloud &pc, sensor_msgs::msg::PointCloud2 &msg, rclcpp::Time callback_time)
    {
        msg.header.frame_id = "link_eef";
        msg.header.stamp = callback_time;
        msg.height = 1;
        msg.width = pc.points_.size();
        msg.is_dense = false;
        msg.is_bigendian = false;
        
        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(pc.points_.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");

        for (size_t i = 0; i < pc.points_.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
        {
            const Eigen::Vector3d &point = pc.points_[i];
            const Eigen::Vector3d &color = pc.colors_[i];

            *iter_x = point.x();
            *iter_y = point.y();
            *iter_z = point.z();
            *iter_r = static_cast<uint8_t>(color.x() * 255.0);
            *iter_g = static_cast<uint8_t>(color.y() * 255.0);
            *iter_b = static_cast<uint8_t>(color.z() * 255.0);
        }
    }
}