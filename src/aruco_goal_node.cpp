#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <string>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;


static constexpr double MAZE_MAX_X    = 7.8;
static constexpr double MAZE_MAX_Y    = 7.8;
static constexpr double MAZE_MIN_X    = 0.3;
static constexpr double MAZE_MIN_Y    = 0.3;

static constexpr double WALL_MARGIN   = 0.0;

class ArucoGoalNode : public rclcpp::Node
{
public:
  ArucoGoalNode() : Node("aruco_goal_node"), navigating_(false), attempt_(0)
  {
    this->declare_parameter<int>("marker_id", -1);
    marker_id_ = this->get_parameter("marker_id").as_int();

    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/fp_completion_status", 10);

    init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "/navigate_to_pose");

    startup_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&ArucoGoalNode::tryNavigate, this));

    RCLCPP_INFO(this->get_logger(),
      "ArUco Goal Node started. Target marker ID: %d", marker_id_);
  }

private:
  void tryNavigate()
  {
    if (navigating_) return;

    if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
      return;
    }

    setInitialPose();

    if (markers_.empty()) {
      if (!loadMarkers()) return;
    }

    if (markers_.find(marker_id_) == markers_.end()) {
      std::string msg = "Error: Marker ID " + std::to_string(marker_id_) +
                        " not found in aruco_markers.yaml";
      RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
      publishStatus(msg);
      startup_timer_->cancel();
      return;
    }

    attempt_++;
    double raw_x = markers_[marker_id_].first;
    double raw_y = markers_[marker_id_].second;

    RCLCPP_INFO(this->get_logger(),
      "Attempt %d: Navigating to marker ID %d at raw (%.2f, %.2f)",
      attempt_, marker_id_, raw_x, raw_y);

    navigating_ = true;
    sendGoal(raw_x, raw_y);
  }

  void setInitialPose()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.pose.position.x = 0.25;
    pose.pose.pose.position.y = 0.5;
    pose.pose.pose.orientation.w = 1.0;
    pose.pose.covariance[0]  = 0.25;
    pose.pose.covariance[7]  = 0.25;
    pose.pose.covariance[35] = 0.068;
    init_pose_pub_->publish(pose);
    RCLCPP_INFO(this->get_logger(), "Initial pose set at (0.25, 0.5)");
  }

  bool loadMarkers()
  {
    std::string yaml_path =
      ament_index_cpp::get_package_share_directory("final_project") +
      "/maps/aruco_markers.yaml";
    try {
      YAML::Node config = YAML::LoadFile(yaml_path);
      for (const auto & m : config["markers"]) {
        int id   = m["id"].as<int>();
        double x = m["x"].as<double>();
        double y = m["y"].as<double>();
        markers_[id] = {x, y};
        RCLCPP_INFO(this->get_logger(),
          "Loaded marker ID %d at (%.2f, %.2f)", id, x, y);
      }
      return true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load markers: %s", e.what());
      return false;
    }
  }

  // Push the goal away from whichever wall it is closest to,
  // moving it toward the maze center by WALL_MARGIN.
  // This handles corners correctly by offsetting both axes independently.
  void adjustGoalAwayFromWalls(double & x, double & y)
  {
    double orig_x = x, orig_y = y;

    // Hard clamp to maze interior first
    x = std::max(MAZE_MIN_X, std::min(x, MAZE_MAX_X));
    y = std::max(MAZE_MIN_Y, std::min(y, MAZE_MAX_Y));

    // Distance to each wall
    double dist_left   = x - MAZE_MIN_X;   // dist to left wall
    double dist_right  = MAZE_MAX_X - x;   // dist to right wall
    double dist_bottom = y - MAZE_MIN_Y;   // dist to bottom wall
    double dist_top    = MAZE_MAX_Y - y;   // dist to top wall

    // If too close to left/right wall, push toward center x
    if (dist_left < WALL_MARGIN) {
      x = MAZE_MIN_X + WALL_MARGIN;
    } else if (dist_right < WALL_MARGIN) {
      x = MAZE_MAX_X - WALL_MARGIN;
    }

    // If too close to top/bottom wall, push toward center y
    if (dist_bottom < WALL_MARGIN) {
      y = MAZE_MIN_Y + WALL_MARGIN;
    } else if (dist_top < WALL_MARGIN) {
      y = MAZE_MAX_Y - WALL_MARGIN;
    }

    if (std::abs(x - orig_x) > 0.01 || std::abs(y - orig_y) > 0.01) {
      RCLCPP_INFO(this->get_logger(),
        "Goal adjusted from (%.2f, %.2f) to (%.2f, %.2f) to stay in free space",
        orig_x, orig_y, x, y);
    }
  }

  void sendGoal(double x, double y)
  {
    adjustGoalAwayFromWalls(x, y);

    RCLCPP_INFO(this->get_logger(),
      "Sending final goal to Nav2: (%.2f, %.2f)", x, y);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp    = this->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.w = 1.0;

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    opts.goal_response_callback =
      [this, x, y](const GoalHandleNav::SharedPtr & gh) {
        if (!gh) {
          RCLCPP_WARN(this->get_logger(),
            "Goal rejected - Nav2 not fully ready yet, retrying in 2s...");
          navigating_ = false;
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal ACCEPTED! Robot is navigating...");
          startup_timer_->cancel();
          publishStatus("Navigating to marker ID " + std::to_string(marker_id_) +
                        " at (" + std::to_string(x) + ", " + std::to_string(y) + ")");
        }
      };

    opts.result_callback =
      [this](const GoalHandleNav::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          std::string msg = "Successfully reached marker ID " +
                            std::to_string(marker_id_);
          RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
          publishStatus(msg);
        } else {
          RCLCPP_WARN(this->get_logger(),
            "Navigation failed, retrying in 5s...");
          navigating_ = false;
          startup_timer_->reset();
        }
      };

    nav_client_->async_send_goal(goal_msg, opts);
  }

  void publishStatus(const std::string & msg)
  {
    auto s = std_msgs::msg::String();
    s.data = msg;
    status_pub_->publish(s);
    RCLCPP_INFO(this->get_logger(), "Status: %s", msg.c_str());
  }

  int marker_id_;
  int attempt_;
  bool navigating_;
  std::map<int, std::pair<double,double>> markers_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoGoalNode>());
  rclcpp::shutdown();
  return 0;
}
