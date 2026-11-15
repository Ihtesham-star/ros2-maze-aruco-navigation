#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>

struct MarkerInfo { int id; double x, y; };

class MazeExplorer : public rclcpp::Node {
public:
  MazeExplorer() : Node("maze_explorer_node")
  {
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_    = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/fp_navigation_status", 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr m){ scanCB(m); });

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(10).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m){ mapCB(m); });

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr m){ detectAruco(m); });

    aruco_dict_   = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    aruco_params_ = cv::aruco::DetectorParameters::create();
    aruco_params_->adaptiveThreshWinSizeMin  = 3;
    aruco_params_->adaptiveThreshWinSizeMax  = 23;
    aruco_params_->adaptiveThreshWinSizeStep = 10;
    aruco_params_->minMarkerPerimeterRate    = 0.010;
    aruco_params_->maxMarkerPerimeterRate    = 4.0;
    aruco_params_->cornerRefinementMethod    = cv::aruco::CORNER_REFINE_SUBPIX;

    timer_ = create_wall_timer(std::chrono::milliseconds(80),
      [this](){ controlLoop(); });

    RCLCPP_INFO(get_logger(), "=== Maze Explorer: Autonomous Maze Exploration ===");
  }

private:
  static constexpr double FWD        = 0.20;   // wall-follow forward
  static constexpr double TURN       = 1.1;    
  static constexpr double FWD_FRONT  = 0.18;   
  static constexpr double TURN_FRONT = 1.0;    

  
  static constexpr double STOP_DIST  = 0.28;
  static constexpr double WALL_DIST  = 0.35;   // target left-wall distance
  static constexpr double FAR_DIST   = 0.60;
  static constexpr double GOAL_REACH = 0.50;

 

  static constexpr double PURE_WALL_TIME = 550.0;
  static constexpr double GOAL_TMOUT     = 30.0;
  static constexpr double STUCK_DIST     = 0.12;
  static constexpr double STUCK_TIME     = 15.0;

  
  void scanCB(const sensor_msgs::msg::LaserScan::SharedPtr& msg) {
    int n = msg->ranges.size();
    auto safeR = [&](int deg) -> double {
      int i = ((deg % n) + n) % n;
      double r = msg->ranges[i];
      return (std::isinf(r)||std::isnan(r)||r<msg->range_min||r>msg->range_max) ? 3.5 : r;
    };
    front_ = 3.5;
    for (int d : {0,5,355,10,350,15,345,20,340})
      front_ = std::min(front_, safeR(d));
    fleft_  = std::min({safeR(30), safeR(45), safeR(60)});
    fright_ = std::min({safeR(300), safeR(315), safeR(330)});
    left_   = std::min({safeR(80), safeR(90), safeR(100)});
    right_  = std::min({safeR(260), safeR(270), safeR(280)});
  }


  void saveMapNow(const std::string& suffix) {
    std::string path = "" + std::string(getenv("HOME")) + "/final-project-Ihtesham-star-main/maps/" + suffix;
    std::string cmd  = "ros2 run nav2_map_server map_saver_cli -f " + path +
                       " --ros-args -p save_map_timeout:=3.0 &";
    int r = system(cmd.c_str()); (void)r;
    RCLCPP_INFO(get_logger(), ">>> AUTO-SAVE -> maps/%s <<<", suffix.c_str());
  }

  
  void mapCB(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg) {
    map_ = msg;
    map_received_ = true;
    int free = 0, obs = 0, unk = 0;
    for (auto v : msg->data) {
      if (v < 0) unk++;
      else if (v < 50) free++;
      else obs++;
    }
    int total = free + obs + unk;
    double new_cov = total > 0 ? 100.0*(free+obs)/total : 0.0;
    if (current_cov_ == 0.0) current_cov_ = new_cov;
    else current_cov_ = 0.7 * current_cov_ + 0.3 * new_cov;

    if (current_cov_ > peak_cov_) {
      peak_cov_ = current_cov_;
      for (int m : {75, 80, 85, 90, 95}) {
        if (peak_cov_ >= (double)m && !milestone_saved_[m]) {
          milestone_saved_[m] = true;
          saveMapNow("explored_maze_" + std::to_string(m) + "pct");
          saveMapNow("explored_maze");
          RCLCPP_INFO(get_logger(), ">>> MILESTONE %d%% - map saved! <<<", m);
        }
      }
    }
  }

  bool isFrontierReachable(double wx, double wy) {
    // Check permanent dead zones
    for (auto& [dx,dy] : dead_zones_)
      if (std::hypot(wx-dx, wy-dy) < 0.7) return false;

    if (!map_) return true;
    int W = map_->info.width, H = map_->info.height;
    float res = map_->info.resolution;
    float ox  = map_->info.origin.position.x;
    float oy  = map_->info.origin.position.y;
    int cx = (int)((wx-ox)/res), cy = (int)((wy-oy)/res);
    int obs_count = 0, total_check = 0;
    for (int dy=-3; dy<=3; dy++) for (int dx=-3; dx<=3; dx++) {
      int nx=cx+dx, ny=cy+dy;
      if (nx<0||nx>=W||ny<0||ny>=H) continue;
      total_check++;
      if (map_->data[ny*W+nx] >= 50) obs_count++;
    }
    if (total_check > 0 && (double)obs_count/total_check > 0.75) return false;
    return true;
  }

  std::vector<std::pair<double,double>> findFrontiers(double rx, double ry,
                                                       double max_dist) {
    if (!map_) return {};
    int W = map_->info.width, H = map_->info.height;
    float res = map_->info.resolution;
    float ox  = map_->info.origin.position.x;
    float oy  = map_->info.origin.position.y;

    std::vector<std::pair<int,int>> fc;
    for (int y=1; y<H-1; y++)
      for (int x=1; x<W-1; x++) {
        if (map_->data[y*W+x]<0 || map_->data[y*W+x]>=50) continue;
        for (auto [dx,dy]:std::vector<std::pair<int,int>>{{0,1},{0,-1},{1,0},{-1,0}})
          if (map_->data[(y+dy)*W+(x+dx)]<0) { fc.push_back({x,y}); break; }
      }
    if (fc.empty()) return {};

    int cr = (int)(0.6/res);
    std::vector<bool> used(fc.size(), false);
    std::vector<std::pair<double,double>> out;
    for (size_t i=0; i<fc.size(); i++) {
      if (used[i]) continue;
      double cx=fc[i].first, cy=fc[i].second; int cnt=1; used[i]=true;
      for (size_t j=i+1; j<fc.size(); j++) {
        if (used[j]) continue;
        if (std::abs(fc[j].first-fc[i].first)<=cr &&
            std::abs(fc[j].second-fc[i].second)<=cr) {
          cx+=fc[j].first; cy+=fc[j].second; cnt++; used[j]=true;
        }
      }
      if (cnt >= 6) {
        double wx=ox+(cx/cnt+0.5)*res, wy=oy+(cy/cnt+0.5)*res;
        if (wx<0.2||wx>7.8||wy<0.2||wy>7.8) continue;
        double dist = std::hypot(wx-rx, wy-ry);
        if (dist>0.6 && dist<=max_dist && isFrontierReachable(wx,wy))
          out.push_back({wx,wy});
      }
    }
    std::sort(out.begin(), out.end(),
      [&](const auto& a, const auto& b){
        return std::hypot(a.first-rx,a.second-ry) < std::hypot(b.first-rx,b.second-ry);
      });
    return out;
  }

  
  bool getPose(double& rx, double& ry, double& ryaw) {
    try {
      auto t = tf_buffer_->lookupTransform("map","base_footprint",
        tf2::TimePointZero, tf2::durationFromSec(0.3));
      rx = t.transform.translation.x;
      ry = t.transform.translation.y;
      auto& q = t.transform.rotation;
      ryaw = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
      return true;
    } catch (...) { return false; }
  }

  
  void controlLoop() {
    if (done_) return;
    double t = now().seconds();

    
    if (!slam_ready_) {
      double rx, ry, ryaw;
      if (map_received_ && getPose(rx, ry, ryaw) &&
          rx >= 0.0 && rx <= 8.0 && ry >= 0.0 && ry <= 8.0) {
        if (++slam_ready_count_ >= 10) {
          slam_ready_ = true;
          stuck_ref_x_ = rx; stuck_ref_y_ = ry; stuck_check_t_ = t;
          RCLCPP_INFO(get_logger(), "[init] SLAM ready! pose=(%.2f,%.2f)", rx, ry);
        }
      } else { slam_ready_count_ = 0; }
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }

    if (start_time_ == 0.0) {
      start_time_ = t; stuck_check_t_ = t;
    }
    double elapsed = t - start_time_;

    
    if (elapsed > 90.0) {
      if ((peak_cov_ - plateau_ref_) > 1.5) { plateau_ref_ = peak_cov_; plateau_t_ = t; }
      bool stalled = (t - plateau_t_) > 70.0 && peak_cov_ > 88.0;
      bool all_mkr = (int)detected_markers_.size() >= 5;
      if ((peak_cov_ >= 95.0 && all_mkr) ||
          (stalled && all_mkr) ||
          (stalled && peak_cov_ > 92.0) ||
          elapsed > 900.0) {
        finishExploration(); return;
      }
    }

    if (t < pause_until_) {
      cmd_pub_->publish(geometry_msgs::msg::Twist()); return;
    }

    double rx, ry, ryaw;
    if (!getPose(rx, ry, ryaw)) return;

       
    if (t - stuck_check_t_ > STUCK_TIME) {
      double moved = std::hypot(rx-stuck_ref_x_, ry-stuck_ref_y_);
      if (moved < STUCK_DIST) {
        RCLCPP_WARN(get_logger(), "STUCK (moved=%.3fm)", moved);
        // Record failure and do a longer spin-escape
        if (in_frontier_mode_ && has_goal_) {
          dead_zones_.push_back({goal_x_, goal_y_});
          failed_goals_.push_back({goal_x_, goal_y_});
          has_goal_ = false;
        }
        escape_until_  = t + 10.0;
        escape_phase_  = 0;
        escape_dir_    = -escape_dir_;
        in_frontier_mode_ = false;
      }
      stuck_check_t_ = t; stuck_ref_x_ = rx; stuck_ref_y_ = ry;
    }

    if (t < escape_until_) {
      geometry_msgs::msg::Twist tw;
      double phase_t = t - (escape_until_ - 10.0);
      if (phase_t < 2.5) {
        // Back up
        tw.linear.x = -0.15;
      } else if (phase_t < 7.0) {
        // Turn away — full speed turn, no forward motion
        tw.angular.z = escape_dir_ * TURN;
      } else {
        // Creep forward to clear
        if (front_ > STOP_DIST * 1.8) tw.linear.x = 0.15;
        else tw.angular.z = escape_dir_ * TURN;
      }
      cmd_pub_->publish(tw); return;
    }

       
    if (t < recovery_until_) {
      geometry_msgs::msg::Twist tw;
      tw.linear.x = -0.08; tw.angular.z = recovery_dir_ * 0.4;
      cmd_pub_->publish(tw); return;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 8000,
      "%.0fs cov=%.1f%%(pk=%.1f%%) %s mk=%zu/5 F=%.2f L=%.2f R=%.2f",
      elapsed, current_cov_, peak_cov_,
      in_frontier_mode_ ? "FRONT" : "WALL",
      detected_markers_.size(), front_, left_, right_);

    
    if (front_ < STOP_DIST) {
      geometry_msgs::msg::Twist tw;
      tw.linear.x  = -0.10;
      tw.angular.z = (fleft_ >= fright_) ? TURN : -TURN;
      if (front_ < STOP_DIST * 0.65) {
        recovery_until_ = t + 2.0;
        recovery_dir_   = (left_ > right_) ? 1.0 : -1.0;
      }
      cmd_pub_->publish(tw); return;
    }

    
    leftWallFollow();
  }

  
  
  void leftWallFollow() {
    geometry_msgs::msg::Twist tw;

    
    scan_tick_++;
    if (scan_tick_ >= 600) {
      scan_tick_ = 0;
      pause_until_ = now().seconds() + 6.0;
      return;
    }

    if (front_ < STOP_DIST) {
      tw.linear.x  = 0.0;
      tw.angular.z = -TURN;
    } else if (front_ < STOP_DIST * 2.0) {
      // Getting close to front wall — slow and start turning right
      tw.linear.x  = FWD * 0.4;
      tw.angular.z = -TURN * 0.8;
    } else if (left_ > 0.80) {
      // Lost the left wall — turn left to find it
      tw.linear.x  = FWD * 0.6;
      tw.angular.z = TURN * 0.7;
    } else if (left_ < 0.20) {
      // Too close to left wall — nudge right
      tw.linear.x  = FWD * 0.7;
      tw.angular.z = -TURN * 0.5;
    } else {
      // In corridor — proportional control to maintain WALL_DIST from left
      double err = left_ - WALL_DIST;
      tw.linear.x  = FWD;
      tw.angular.z = err * 2.0;        // P controller
      tw.angular.z = std::max(-TURN, std::min(TURN, tw.angular.z));
      // Extra right nudge if right wall also close
      if (right_ < 0.25) tw.angular.z -= TURN * 0.3;
    }

    cmd_pub_->publish(tw);
  }

  
  void frontierStep(double t, double rx, double ry, double ryaw) {
    // Goal timeout
    if (has_goal_ && (t - goal_set_t_) > GOAL_TMOUT) {
      RCLCPP_WARN(get_logger(), "Goal timeout (%.2f,%.2f)", goal_x_, goal_y_);
      dead_zones_.push_back({goal_x_, goal_y_});
      failed_goals_.push_back({goal_x_, goal_y_});
      if (failed_goals_.size() > 25) failed_goals_.erase(failed_goals_.begin());
      has_goal_ = false;
      consecutive_failures_++;
      return;
    }

    // Goal reached
    if (has_goal_ && std::hypot(rx-goal_x_, ry-goal_y_) < GOAL_REACH) {
      RCLCPP_INFO(get_logger(), "Reached (%.2f,%.2f)", goal_x_, goal_y_);
      has_goal_ = false;
      consecutive_failures_ = 0;
      pause_until_ = t + 2.0; // brief spin at each reached frontier
      return;
    }
    if (t < pause_until_) {
      geometry_msgs::msg::Twist tw;
      tw.angular.z = 0.5;
      cmd_pub_->publish(tw); return;
    }

    // Pick frontier
    if (!has_goal_) {
      auto fronts = findFrontiers(rx, ry, 3.5);
      if (fronts.empty()) fronts = findFrontiers(rx, ry, 5.5);

      if (fronts.empty()) {
        RCLCPP_INFO(get_logger(), "No frontiers → wall-follow");
        in_frontier_mode_ = false;
        frontier_block_until_ = t + 30.0;
        return;
      }

      // Filter blacklisted
      std::vector<std::pair<double,double>> valid;
      for (auto& f : fronts) {
        bool bad = false;
        for (auto& [fx,fy] : failed_goals_)
          if (std::hypot(f.first-fx, f.second-fy) < 0.4) { bad=true; break; }
        if (!bad) valid.push_back(f);
      }
      if (valid.empty()) {
        failed_goals_.erase(failed_goals_.begin(),
          failed_goals_.begin() + (int)failed_goals_.size()/2);
        in_frontier_mode_ = false;
        frontier_block_until_ = t + 20.0;
        return;
      }

      goal_x_ = valid[0].first; goal_y_ = valid[0].second;
      has_goal_ = true; goal_set_t_ = t;
      RCLCPP_INFO(get_logger(), "Frontier (%.2f,%.2f) d=%.2f",
        goal_x_, goal_y_, std::hypot(rx-goal_x_, ry-goal_y_));
    }

    // Drive toward frontier using wall-avoidance
    geometry_msgs::msg::Twist tw;
    double adiff = std::atan2(goal_y_-ry, goal_x_-rx) - ryaw;
    while (adiff >  M_PI) adiff -= 2*M_PI;
    while (adiff < -M_PI) adiff += 2*M_PI;

    // If we're in a corridor just go straight
    if (left_ < 0.45 && right_ < 0.45) {
      tw.linear.x  = FWD_FRONT * (front_ > STOP_DIST*2 ? 1.0 : 0.5);
      tw.angular.z = (right_ < left_) ? 0.3 : -0.3;
    } else if (std::abs(adiff) > 1.0) {
      tw.angular.z = (adiff > 0) ? TURN_FRONT : -TURN_FRONT;
    } else {
      double avoid = 0.0;
      if (front_ < FAR_DIST) { avoid += (fleft_>fright_) ? 1.0 : -1.0; }
      if (fleft_  < STOP_DIST*1.5) avoid -= 0.7;
      if (fright_ < STOP_DIST*1.5) avoid += 0.7;
      double spd = FWD_FRONT * std::max(0.2, front_/FAR_DIST);
      spd = std::max(0.04, spd*(1.0-0.3*std::abs(adiff)));
      double ww = std::max(0.0, 1.0-std::min(left_,right_)/0.5);
      tw.linear.x  = spd;
      tw.angular.z = (1.0-ww)*(TURN_FRONT*adiff/M_PI*2.5) + ww*(avoid*0.4);
      tw.angular.z = std::max(-TURN_FRONT*1.2, std::min(TURN_FRONT*1.2, tw.angular.z));
    }
    cmd_pub_->publish(tw);
  }

  
  void detectAruco(const sensor_msgs::msg::Image::SharedPtr& msg) {
    if (done_) return;
    double t = now().seconds();
    if (start_time_ == 0.0 || (t - start_time_) < 8.0) return;
    if (t - last_img_t_ < 0.10) return;
    last_img_t_ = t;

    cv::Mat frame;
    try {
      if (msg->encoding=="rgb8"||msg->encoding=="R8G8B8") {
        cv::Mat tmp(msg->height,msg->width,CV_8UC3,
          const_cast<uint8_t*>(msg->data.data()),msg->step);
        cv::cvtColor(tmp,frame,cv::COLOR_RGB2BGR);
      } else if (msg->encoding=="bgr8") {
        frame=cv::Mat(msg->height,msg->width,CV_8UC3,
          const_cast<uint8_t*>(msg->data.data()),msg->step).clone();
      } else if (msg->encoding=="mono8") {
        cv::Mat g(msg->height,msg->width,CV_8UC1,
          const_cast<uint8_t*>(msg->data.data()),msg->step);
        cv::cvtColor(g,frame,cv::COLOR_GRAY2BGR);
      } else return;
    } catch(...){ return; }

    std::vector<int> ids; std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(frame,aruco_dict_,corners,ids,aruco_params_);
    if (ids.empty()) {
      cv::Mat big; cv::resize(frame,big,cv::Size(),2,2,cv::INTER_LINEAR);
      cv::aruco::detectMarkers(big,aruco_dict_,corners,ids,aruco_params_);
    }
    if (ids.empty()) {
      cv::Mat gray,enh; cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);
      auto cl=cv::createCLAHE(3.0,cv::Size(8,8)); cl->apply(gray,gray);
      cv::cvtColor(gray,enh,cv::COLOR_GRAY2BGR);
      cv::aruco::detectMarkers(enh,aruco_dict_,corners,ids,aruco_params_);
    }
    if (ids.empty()) {
      cv::Mat sharp;
      cv::Mat kernel = (cv::Mat_<float>(3,3) <<
        -0.5,-0.5,-0.5, -0.5,5.0,-0.5, -0.5,-0.5,-0.5);
      cv::filter2D(frame,sharp,-1,kernel);
      cv::aruco::detectMarkers(sharp,aruco_dict_,corners,ids,aruco_params_);
    }
    if (ids.empty()) return;

    {
      std::vector<int> fids; std::vector<std::vector<cv::Point2f>> fcorners;
      for (size_t i=0; i<ids.size(); i++) {
        auto& c=corners[i];
        float minx=c[0].x,maxx=c[0].x,miny=c[0].y,maxy=c[0].y;
        for (auto& p:c){
          minx=std::min(minx,p.x); maxx=std::max(maxx,p.x);
          miny=std::min(miny,p.y); maxy=std::max(maxy,p.y);
        }
        float w=maxx-minx,h=maxy-miny,area=w*h,asp=(h>0.5f)?w/h:0.0f;
        if (area>=60.0f&&asp>=0.3f&&asp<=3.0f) {
          fids.push_back(ids[i]); fcorners.push_back(corners[i]);
        }
      }
      ids=fids; corners=fcorners;
    }
    if (ids.empty()) return;

    double rx,ry,ryaw;
    if (!getPose(rx,ry,ryaw)) return;
    bool newm=false;
    for (int mid:ids) {
      if (mid<0||mid>49||detected_markers_.count(mid)) continue;
      detected_markers_[mid]={mid,rx,ry};
      RCLCPP_INFO(get_logger(),">>>>>> ArUco ID=%d at (%.2f,%.2f) [%zu/5] <<<<<<",
        mid,rx,ry,detected_markers_.size());
      newm=true; pause_until_=now().seconds()+0.4;
    }
    if (newm) saveMarkers();
  }

  
  void saveMarkers() {
    for (auto& p : {
      std::string("" + std::string(getenv("HOME")) + "/final-project-Ihtesham-star-main/maps/aruco_markers.yaml"),
      std::string(getenv("HOME")) + "/ros2_ws/aruco_markers.yaml",
      std::string("aruco_markers.yaml")}) {
      std::ofstream ofs(p); if(!ofs.is_open()) continue;
      ofs<<"markers:\n";
      for (auto& [id,info]:detected_markers_)
        ofs<<"  - id: "<<info.id<<"\n    x: "
           <<std::fixed<<std::setprecision(2)<<info.x<<"\n    y: "<<info.y<<"\n";
    }
    RCLCPP_INFO(get_logger(),"[ArUco] Saved %zu/5 markers",detected_markers_.size());
  }

  
  void finishExploration() {
    if (done_) return; done_=true;
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    saveMarkers();
    std_msgs::msg::String s; s.data="exploration_complete"; status_pub_->publish(s);
    RCLCPP_INFO(get_logger(),"=== DONE t=%.0fs cov=%.1f%% markers=%zu/5 ===",
      now().seconds()-start_time_, peak_cov_, detected_markers_.size());
    for (auto& [id,info]:detected_markers_)
      RCLCPP_INFO(get_logger(),"  ID %d at (%.2f,%.2f)",id,info.x,info.y);
  }

  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  
  bool   done_{false}, map_received_{false}, slam_ready_{false};
  int    slam_ready_count_{0};
  double start_time_{0}, peak_cov_{0}, current_cov_{0};
  double plateau_ref_{0}, plateau_t_{0};

  // Wall-follow
  bool   in_frontier_mode_{false};
  int    scan_tick_{0};
  int    consecutive_failures_{0};
  double frontier_block_until_{0};

  // Frontier goal
  double goal_x_{0}, goal_y_{0};
  bool   has_goal_{false};
  double goal_set_t_{0};
  std::vector<std::pair<double,double>> failed_goals_;
  std::vector<std::pair<double,double>> dead_zones_;

  // Sensor readings
  double front_{3.5}, fleft_{3.5}, fright_{3.5};
  double left_{3.5},  right_{3.5};

  // Timers
  double pause_until_{0};
  double escape_until_{0};
  double escape_dir_{1.0};
  int    escape_phase_{0};
  double recovery_until_{0}, recovery_dir_{1.0};
  double stuck_check_t_{0}, stuck_ref_x_{0}, stuck_ref_y_{0};
  double last_img_t_{0};

  // Data
  std::map<int,MarkerInfo>              detected_markers_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::map<int,bool>                    milestone_saved_;
  cv::Ptr<cv::aruco::Dictionary>        aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MazeExplorer>());
  rclcpp::shutdown();
}
