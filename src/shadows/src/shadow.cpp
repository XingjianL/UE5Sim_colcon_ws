#include "rclcpp/rclcpp.hpp"
#include "tomato_xarm6/image_subscribe.hpp"
#include "tomato_xarm6/environment_info.hpp"
#include "tomato_xarm6/planar_robot.hpp"

void spin_node_in_thread(rclcpp::Node::SharedPtr node)
{
    // Spin the node in a separate thread
    rclcpp::spin(node);
}

// Step 1: start rosbridge server (ros2 launch tomato_xarm6 upsidedown....)
// Step 2: start UE5 environment (ShadowDataCollect)
// Step 3: start Helios program (Procedural...)
// Step 4: start this script (ros2 run shadow ...)
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto const logger = rclcpp::get_logger("shadow_data_collect");

  std::string temperature = "10000,0,1"; // temperature, intensity, exposure
  bool capture_both = false;
  std::string seed = "0";
  std::string save_prefix = "";

  bool reduce_file_size = false;
  for (int i = 1; i < argc; i++)
  {
    std::string arg = argv[i];

    RCLCPP_INFO(logger, arg.c_str());
    if (arg == "--reduce-file-size") {
      reduce_file_size = true;
    }
    if (arg == "--seed" && i + 1 < argc) {
      seed = argv[i+1];
      ++i;
    }
    if (arg == "--save-prefix" && i + 1 < argc) {
      save_prefix = argv[i+1];
      ++i;
    }
    if (arg == "--light-temp" && i + 1 < argc) {
      temperature = argv[i+1];
      ++i;
    }
  }
	auto const node = std::make_shared<rclcpp::Node>(
		"shadow_data_collect",
		rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
	);

	tomato_xarm6::EnvironmentInfo env(node);            // UE5 environment parser
	env.save_prefix=save_prefix;

	std::string arg_file_name = "output/robot/shadow_data_collect_" + std::to_string(env.creation_time_.seconds()) + ".txt";
	std::ofstream outfile(arg_file_name);
	if (!outfile) {
		std::cerr << "Error opening file for writing.\n";
		return 1;
	}

	for (int i = 0; i < argc; ++i) {
			outfile << argv[i] << "\n";
	}
  outfile.close();

  // robot initialization
	tomato_xarm6::PlanarRobot camera_platform(node, "Camera");
  tomato_xarm6::PlanarRobot shadow_platform(node, "Shadow");
  tomato_xarm6::PlanarRobot light_platform(node, "Light");
  std::thread spin_thread(spin_node_in_thread, node);
  // MARK: UE5 Init
  env.EnvPublishCommand("LightSet:" + temperature + ":PCGSeedIncr:" + seed);
  RCLCPP_INFO(logger, "Waiting for Load");
  rclcpp::sleep_for(std::chrono::milliseconds(5000));
  env.waiting_for_sync();
  env.EnvPublishCommand("LightSet:" + temperature + ":PCGSeedIncr:" + seed);
  RCLCPP_INFO(logger, "Finished Environment Init");
	std::string cam_node_name1 = "shadow_camera_rgbd";
  std::string cam_node_name2 = "shadow_camera_stereo";
  env.EnvPublishCommand("CamPub:1");
	env.StartRobotCamera("Camera", cam_node_name1, cam_node_name2, capture_both, reduce_file_size);
  env.StartRobotCamera("Shadow", cam_node_name1, cam_node_name2, capture_both, reduce_file_size);

  RCLCPP_INFO(logger, "Finished Husky Camera Init");
  for (int i = 0; i < 5; i++) {
    
    double angle = 2.0 * 3.14159265358 * i / 36; // evenly spaced angles
    double x = 125 * cos(angle);
    double y = 125 * sin(angle);
    RCLCPP_INFO(logger, "set light pos %f, %f", x, y);
    light_platform.set_planar_targets(
      x, y, 280, 
      0, 0, angle + 3.14159265358
    );

    //rclcpp::sleep_for(std::chrono::milliseconds(1000));
    env.waiting_for_sync();
    //env.EnvPublishCommand("CamPub:1");
    //#rclcpp::sleep_for(std::chrono::milliseconds(1000));
    env.SaveRobotImages({"Camera", "Shadow"}, true);
    RCLCPP_INFO(logger, "Saved image %d", i);
    //env.EnvPublishCommand("CamPub:0");
    //rclcpp::sleep_for(std::chrono::milliseconds(100));
    env.UpdateLog();
  }
  env.EnvPublishCommand("NextAge:NextAge");
  rclcpp::sleep_for(std::chrono::milliseconds(5000));
  env.SaveLog();
  RCLCPP_INFO(logger, "Stop Camera");
  env.StopRobotCamera("Camera");
  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}