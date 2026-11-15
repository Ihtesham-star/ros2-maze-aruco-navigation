#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <chrono>
#include <filesystem>

using namespace std::chrono_literals;

class MapSaverNode : public rclcpp::Node
{
public:
  MapSaverNode() : Node("map_saver_node")
  {
    
    try {
      pkg_share_dir_ = ament_index_cpp::get_package_share_directory("final_project");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Could not find package: %s", e.what());
      pkg_share_dir_ = "/tmp";
    }
    map_dir_ = std::string(getenv("HOME")) + "/final-project-Ihtesham-star-main/maps";
    std::filesystem::create_directories(map_dir_);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(10).transient_local(),
      std::bind(&MapSaverNode::mapCallback, this, std::placeholders::_1));

    status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/fp_navigation_status", 10,
      std::bind(&MapSaverNode::statusCallback, this, std::placeholders::_1));

    // Check coverage every 5 seconds
    timer_ = this->create_wall_timer(5s, std::bind(&MapSaverNode::checkAndSave, this));

    RCLCPP_INFO(this->get_logger(), "Map Saver Node started. Will save to: %s", map_dir_.c_str());
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    int total = msg->info.width * msg->info.height;
    if (total == 0) return;

    int free_cells     = 0;
    int obstacle_cells = 0;

    for (auto & cell : msg->data) {
      if (cell < 0)  continue;         // unknown — skip
      if (cell < 50) free_cells++;     // free space
      else           obstacle_cells++; // wall/obstacle
    }

    int total_accessible = total - obstacle_cells;
    if (total_accessible <= 0) return;

    coverage_ = 100.0 * free_cells / total_accessible;
    has_map_  = true;
  }

  void statusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data.find("exploration_complete") != std::string::npos) {
      RCLCPP_INFO(this->get_logger(), "Received exploration_complete signal! Saving map...");
      saveMap();
    }
  }

  void checkAndSave()
  {
    if (!has_map_) return;

    RCLCPP_INFO(this->get_logger(), "Current map coverage: %.1f%%", coverage_);


    if (!map_saved_ && coverage_ >= 92.0) {
      RCLCPP_INFO(this->get_logger(),
        "Coverage %.1f%% >= 92%% threshold. Saving map...", coverage_);
      saveMap();
    }
  }

  void saveMap()
  {
    if (map_saved_) return;
    map_saved_ = true;

    
    std::string map_path = map_dir_ + "/explored_maze";

    std::string cmd =
      "ros2 run nav2_map_server map_saver_cli -f " + map_path +
      " --ros-args -p save_map_timeout:=10.0 2>&1";

    RCLCPP_INFO(this->get_logger(), "Running: %s", cmd.c_str());

    int ret = std::system(cmd.c_str());

    if (ret == 0) {
      RCLCPP_INFO(this->get_logger(),
        "=== Map saved successfully to %s ===", map_path.c_str());
      RCLCPP_INFO(this->get_logger(),
        "Files: explored_maze.pgm and explored_maze.yaml");

      
      std::string src_maps = pkg_share_dir_ + "/maps";
      std::string copy_cmd = "cp " + map_path + ".pgm " + src_maps +
                             "/explored_maze.pgm 2>/dev/null; " +
                             "cp " + map_path + ".yaml " + src_maps +
                             "/explored_maze.yaml 2>/dev/null";
      std::system(copy_cmd.c_str());

    } else {
      RCLCPP_ERROR(this->get_logger(),
        "Map save FAILED (exit code %d). Will retry on next trigger.", ret);
      map_saved_ = false;  // Allow retry
    }
  }

  std::string pkg_share_dir_;
  std::string map_dir_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double coverage_  = 0.0;
  bool   has_map_   = false;
  bool   map_saved_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapSaverNode>());
  rclcpp::shutdown();
  return 0;
}
