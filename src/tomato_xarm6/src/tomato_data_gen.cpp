#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "tomato_xarm6/xarm6_moveit.hpp"
#include "tomato_xarm6/image_subscribe.hpp"
#include "tomato_xarm6/environment_info.hpp"
#include "tomato_xarm6/planar_robot.hpp"
#include <thread>
#include <random>
#include <yaml-cpp/yaml.h>

void spin_node_in_thread(rclcpp::Node::SharedPtr node)
{
    // Spin the node in a separate thread
    rclcpp::spin(node);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto const logger = rclcpp::get_logger("tomato_xarm6");

  printf("hello world tomato_xarm6 package\n");

  //bool reconstruct_point_clouds = false;
  int sample_gap = 4;

  // default arguments
  bool reset_time = false;
  std::string temperature = "3500,40,-4"; // temperature, intensity, exposure
  bool capture_both = false;
  unsigned int seed = 0;
  std::string filtered_disease = "";
  std::string split_height_leaf = "";
  std::string leaf_preprocess = "";
  std::string percent_healthy = "0.5";
  YAML::Node robot_positions;
  bool skip_init = false;
  std::string PCGSeedIncr = "0";
  std::string save_prefix = "";

  double rand_x_intensity = 0;
  double rand_y_intensity = 0;
  double rand_z_intensity = 0;
  double rand_roll_intensity = 0;
  double rand_pitch_intensity = 0;
  double rand_yaw_intensity = 0;

  double const_x_offset = 0;
  double const_y_offset = 0;
  double const_z_offset = 0;
  double const_roll_offset = 0;
  double const_pitch_offset = 0;
  double const_yaw_offset = 0;
  
  bool reduce_file_size = false;

  // add arguments here (see automation.sh for example usages)
  // MARK: Arg List
  
  for (int i = 1; i < argc; i++)
  {
    std::string arg = argv[i];
    RCLCPP_INFO(logger, arg.c_str());
    if (arg == "--reduce-file-size") {
      reduce_file_size = true;
    }
    if (arg == "--offset" && i + 6 < argc) {
      const_x_offset = std::stod(argv[i + 1]);
      const_y_offset = std::stod(argv[i + 2]);
      const_z_offset = std::stod(argv[i + 3]);
      const_roll_offset = std::stod(argv[i + 4]);
      const_pitch_offset = std::stod(argv[i + 5]);
      const_yaw_offset = std::stod(argv[i + 6]);
      i += 6;
    }
    if (arg == "--rand" && i + 6 < argc) {
      rand_x_intensity = std::stod(argv[i + 1]);
      rand_y_intensity = std::stod(argv[i + 2]);
      rand_z_intensity = std::stod(argv[i + 3]);
      rand_roll_intensity = std::stod(argv[i + 4]);
      rand_pitch_intensity = std::stod(argv[i + 5]);
      rand_yaw_intensity = std::stod(argv[i + 6]);
      
      i += 6;
    }
    if (arg == "--reset-time") {
      reset_time = true;
    }
    if (arg == "--light-temp" && i + 1 < argc) {
      temperature = argv[i+1];
      ++i;
    }
    if (arg == "--both") {
      capture_both = true;
    }
    if (arg == "--seed" && i + 1 < argc) {
      seed = std::stoi(argv[i+1]);
      ++i;
    }
    if (arg == "--disease-filter" && i + 1 < argc){
      filtered_disease = argv[i+1];
      ++i;
    }
    if (arg == "--split-height-leaf" && i+1<argc){
      split_height_leaf = argv[i+1];
      ++i;
    }
    if (arg=="--preprocess" && i+1<argc){
      leaf_preprocess = argv[i+1];
      ++i;
    }
    if (arg=="--percent-healthy" && i+1 < argc) {
      percent_healthy = argv[i+1];
      ++i;
    }
    if (arg=="--move-robot" && i+1 < argc) {
      RCLCPP_INFO(logger, "--move-robot: %s", argv[i+1]);
      robot_positions = YAML::Load(argv[i+1]);
      ++i;
    }
    if (arg == "--skip-init") {
      skip_init = true;
    }
    if (arg == "--pcg-seed-incr" && i + 1 < argc) {
      PCGSeedIncr = argv[i+1];
      ++i;
    }
    if (arg == "--save-prefix" && i + 1 < argc) {
      save_prefix = argv[i+1];
      ++i;
    }
  }
  // MARK: Initializations
  std::mt19937 randgen(seed);
  std::uniform_real_distribution<double> rand_x_offset(-0.5*rand_x_intensity, 0.5*rand_x_intensity);
  std::uniform_real_distribution<double> rand_y_offset(-0.5*rand_y_intensity, 0.5*rand_y_intensity);
  std::uniform_real_distribution<double> rand_z_offset(-0.5*rand_z_intensity, 0.5*rand_z_intensity);
  std::uniform_real_distribution<double> rand_roll_offset(-0.05*M_PI*rand_roll_intensity, 0.05*M_PI*rand_roll_intensity);
  std::uniform_real_distribution<double> rand_pitch_offset(-0.05*M_PI*rand_pitch_intensity, 0.05*M_PI*rand_pitch_intensity);
  std::uniform_real_distribution<double> rand_yaw_offset(-0.05*M_PI*rand_yaw_intensity, 0.05*M_PI*rand_yaw_intensity);

  RCLCPP_INFO(logger, "Received arguments %d",argc);
  for (const auto& item: robot_positions) {
    RCLCPP_INFO(logger, "received position for %s: %f, %f", item["name"].as<std::string>().c_str(), item["x"].as<float>(),item["y"].as<float>());
  }
  auto const node = std::make_shared<rclcpp::Node>(
    "tomato_xarm6",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  tomato_xarm6::EnvironmentInfo env(node);            // UE5 environment parser
  env.save_prefix=save_prefix;
  std::string arg_file_name = "output/robot/tomato_data_gen_" + std::to_string(env.creation_time_.seconds()) + ".txt";
  std::ofstream outfile(arg_file_name);
    if (!outfile) {
        std::cerr << "Error opening file for writing.\n";
        return 1;
    }

    for (int i = 0; i < argc; ++i) {
        outfile << argv[i] << "\n";
    }
  outfile.close();
  //tomato_xarm6::XARM6MoveIt robot1("xarm6", node);    // MoveIt control
  tomato_xarm6::PlanarRobot benchbot_platform(node, "BenchBot");
  tomato_xarm6::PlanarRobot spider_platform(node, "Spider");
  tomato_xarm6::PlanarRobot husky_platform(node, "Husky");
  std::thread spin_thread(spin_node_in_thread, node);

  std::string cam_node_name1 = "rgbd_camera";
  std::string cam_node_name2 = "stereo_camera";
  // MARK: UE5 Init
  if (!skip_init){
    rclcpp::sleep_for(std::chrono::milliseconds(2000));
    if (reset_time) {
      env.EnvPublishCommand("TimeIncr:-1:LightSet:"+temperature+":DiseaseFilter:"+filtered_disease+":SplitHeightLeaf:"+split_height_leaf+":LeafPreprocess:"+leaf_preprocess+":PercentHealthy:"+percent_healthy+":PCGSeedIncr:"+PCGSeedIncr);
    } else {
      env.EnvPublishCommand("TimeIncr:2:LightSet:"+temperature+":DiseaseFilter:"+filtered_disease+":SplitHeightLeaf:"+split_height_leaf+":LeafPreprocess:"+leaf_preprocess+":PercentHealthy:"+percent_healthy+":PCGSeedIncr:"+PCGSeedIncr);
    }
    rclcpp::sleep_for(std::chrono::milliseconds(10000));
  }
  env.waiting_for_sync();
  RCLCPP_INFO(logger, "Finished Environment Init");

  env.StartRobotCamera("Husky", cam_node_name1, cam_node_name2, capture_both, reduce_file_size);
  RCLCPP_INFO(logger, "Finished Husky Camera Init");

  env.StartRobotCamera("BenchBot", cam_node_name1, cam_node_name2, capture_both, reduce_file_size);
  RCLCPP_INFO(logger, "Finished Benchbot Camera Init");

  for (int i = 0; i < 1; i+=sample_gap){

    // move robot based on argument
    for (const auto& item: robot_positions) {
      RCLCPP_INFO(logger, "moving argument position for %s: %f, %f", item["name"].as<std::string>().c_str(), item["x"].as<float>(),item["y"].as<float>());
      std::string robot_name = item["name"].as<std::string>();
      double x = item["x"].as<double>();
      double y = item["y"].as<double>();
      tomato_xarm6::PlanarRobot platform(node, robot_name);
      if (robot_name == "BenchBot") {
        platform.set_planar_targets(
          x, 525, 25, 
          0, 0, 0
        );
        platform.set_joints_targets(
          {"benchbot_plate", "benchbot_camera"}, 
          {y, 100}
        );
      } else if (robot_name == "Spider") {
        platform.set_planar_targets(
          x, y, 0, 
          0, 0, 0
        );
      } else {
        platform.set_planar_targets(
          x, y, 0, 
          0, 0, 0
        );
      }
      rclcpp::sleep_for(std::chrono::milliseconds(3000));
    }

    // break; // testing move-robot

    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // wait for the robot in UE5 to settle
    // move spider and husky
    //rclcpp::sleep_for(std::chrono::milliseconds(5000));
    for (int plant_id_x = 0; plant_id_x < 18; plant_id_x++){ // 19
      double platform_pos_x = plant_id_x; //1.0 -> 19.0
      benchbot_platform.set_planar_targets(
        platform_pos_x * 50 + 65 + rand_x_offset(randgen) + const_x_offset, 
        450, 25, 
        rand_roll_offset(randgen) + const_roll_offset, 
        rand_pitch_offset(randgen) + const_pitch_offset,
        rand_yaw_offset(randgen) + const_yaw_offset 
      );
      for (int plant_id_y = 1; plant_id_y < 6; plant_id_y++){ // 7
        // husky set position
        husky_platform.set_planar_targets(
          platform_pos_x * 40 + rand_x_offset(randgen) + const_x_offset, 
          plant_id_y * 75 + rand_y_offset(randgen) + const_y_offset,
          0 + rand_z_offset(randgen)+ const_z_offset,
          rand_roll_offset(randgen) + const_roll_offset, 
          rand_pitch_offset(randgen) + const_pitch_offset,
          rand_yaw_offset(randgen) + const_yaw_offset
        );

        double platform_pos_y = plant_id_y;
        if (plant_id_x % 2) {
          benchbot_platform.set_joints_targets(
            {"benchbot_plate", "benchbot_camera"}, 
            {platform_pos_y * 150-200 + rand_y_offset(randgen) + const_y_offset, 
            100 + rand_z_offset(randgen)+ const_z_offset}
          );
        } else {
          benchbot_platform.set_joints_targets(
            {"benchbot_plate", "benchbot_camera"}, 
            {-platform_pos_y * 150+ 6 * 150 - 200 + rand_y_offset(randgen) + const_y_offset, 
            100 + rand_z_offset(randgen)+ const_z_offset}
          );
        }
        RCLCPP_INFO(logger, "set husky and benchbot pos");
        rclcpp::sleep_for(std::chrono::milliseconds(2000));
        env.waiting_for_sync();
        RCLCPP_INFO(logger, "Saving images");
        if (plant_id_x >= 1 && plant_id_x < 15 && plant_id_y >= 1 && plant_id_y < 5) {
          env.SaveRobotImages({"Husky","BenchBot"}, true);
        }
        else {
          if (plant_id_x >= 1 && plant_id_x < 18 && plant_id_y >= 1 && plant_id_y < 5) {
            env.SaveRobotImages({"Husky"}, true);
          }
          if (plant_id_x >= 0 && plant_id_x < 10 && plant_id_y >= 1 && plant_id_y < 6) {
            env.SaveRobotImages({"BenchBot"}, true);
          }
        }
        env.UpdateLog();
        // logging the environment
        // RCLCPP_INFO(logger, "Update Log");
        
        // env.UpdateLog();
        // RCLCPP_INFO(logger, "LogUpdated");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
    }
    // move benchbot
    // for (int plant_id_x = 0; plant_id_x < 10; plant_id_x++){ // 19
    //   double platform_pos_x = plant_id_x; //1.0 -> 19.0
    //   // move benchbot-amiga forward in small increments
    //   benchbot_platform.set_planar_targets(
    //     platform_pos_x * 100 + 65 + rand_x_offset(randgen) + const_x_offset, 
    //     450, 25, 
    //     rand_roll_offset(randgen) + const_roll_offset, 
    //     rand_pitch_offset(randgen) + const_pitch_offset,
    //     rand_yaw_offset(randgen) + const_yaw_offset 
    //   );

    //   // move benchbot-gantry-camera to scan
    //   for (int plant_id_y = 1; plant_id_y < 6; plant_id_y++){ // 7
    //     double platform_pos_y = plant_id_y; // 2.0 -2.0-> 12.0 (14.0 -2.0-> 4.0)
        
    //     RCLCPP_INFO(logger, "set benchbot pos");
    //     rclcpp::sleep_for(std::chrono::milliseconds(1000));
    //     env.waiting_for_sync();
        
    //   }
    // }
    //break;

    
  }
  //env.robot_info_[0].WriteVideo();
  RCLCPP_INFO(logger, "Saving Env File");
  env.SaveLog();
  RCLCPP_INFO(logger, "Stop Camera");
  // for (size_t i = 0; i < env.robot_info_.size(); i++){
  //   RCLCPP_INFO(logger, "robot names %s, %d", env.robot_info_[i].topic_name.c_str(), env.robot_info_[i].topic_name == "BenchBot");
  //   if (env.robot_info_[i].topic_name == "Husky"){
  //     env.robot_info_[i].ConfigCamera(cam_node_name, capture_both);
  //     env.robot_info_[i].image_subscriber->stop();
  //   }
    
  // }
  env.StopRobotCamera("Husky");
  env.StopRobotCamera("BenchBot");
  //env.EnvPublishCommand("PCGSeedIncr:1");
  rclcpp::sleep_for(std::chrono::milliseconds(1000));
  //
  rclcpp::shutdown();
  spin_thread.join();
  printf("goodbye world tomato_xarm6 package\n");

  return 0;
}
