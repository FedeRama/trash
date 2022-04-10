#ifndef PATH_CHOOSER_NODE_H
#define PATH_CHOOSER_NODE_H

#define SIM

#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <thread>

#include "spline.h"

// Constants
const double RPM2MS     = 0.00012;
const double MAX_STEER  = 0.45;
const double MIN_STEER  = -MAX_STEER;
const double L          = 0.32;
const double pp_k       = 0.10;
const int PP_FREQUENCY  = 40;

typedef struct car_state_t
{
  double x, y, yaw;            // Car position and rotation in the map
  double speed, steer;         // Current car speed and steering angle
  bool pose_received, ready;   // Current car readiness
} car_state_t;

typedef struct obstacle_t
{
  double x, y, radius;
} obstacle_t;

typedef enum launch_mode_t
{
  MODE_TIMEATTACK     = 0,
  MODE_OBSTACLETEST   = 1,
  MODE_HEAD2HEAD      = 2,
  MODE_FRENET	      = 3
} launch_mode_t;

typedef enum path_type_t
{
  PATH_NONE	 = -1,
  PATH_OPTIMAL = 0,
  PATH_LEFT    = 1,
  PATH_CENTER  = 2,
  PATH_RIGHT   = 3
} path_type_t;

typedef struct vector3d{
  std::vector<double> xs, ys, speed;
} vector3d;

static inline vector3d load_flag_path(std::string file_path)
{

  std::string line;
  size_t pos = 0;
  std::string temp;
  int i;
  vector3d xyspeed;

  std::cout << file_path << "\n";

  std::ifstream is(file_path);
  if (!is)
  {
    std::cout << file_path << " not found!\n";
    exit(0);
  }

  //file format:
  // - Name: Flag 0
  // X: 0.442536
  // Y: 0.320954
  // Z: 4.49081

  for(i=0; std::getline(is,line); i++)//Read useless line
  {
      //read x
      std::getline(is,line);
      pos = line.find(":") + 1;
      xyspeed.xs.push_back(atof(line.substr(pos, std::string::npos).c_str()));

      //read y
      std::getline(is,line);
      pos = line.find(":") + 1;
      xyspeed.ys.push_back(atof(line.substr(pos, std::string::npos).c_str()));

      //read speed
      std::getline(is,line);
      pos = line.find(":") + 1;
      xyspeed.speed.push_back(atof(line.substr(pos, std::string::npos).c_str()));
  }

  return xyspeed;

}

static inline point_t point_transform(point_t target, point_t pose, double yaw)
{
  point_t local_straight;
  local_straight.x = target.x - pose.x;
  local_straight.y = target.y - pose.y;

  point_t ret;
  ret.x = local_straight.x*cos(yaw) + local_straight.y*sin(yaw);
  ret.y = -local_straight.x*sin(yaw) + local_straight.y*cos(yaw);

  return ret;
}

class PurePursuitNode
{

private:

  double pp_step(point_t pose, double speed);
  double la_step(double curvature);
  void build_path_msgs();

  // Node
  rclcpp::Node::SharedPtr pub_node;
  rclcpp::Node::SharedPtr sub_node;
  rclcpp::executors::MultiThreadedExecutor executor;


  // Subscribers
  rclcpp::CallbackGroup::SharedPtr callback_group_estop_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_pose_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_scan_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_engine_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_path_to_follow_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_local_path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr engine_sub;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr path_to_follow_sub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub;

  // Publishers
#ifndef GIANNI
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;
#else
  rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr drive_pub;
#endif
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub[4];
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr spline_pos_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
  rclcpp::TimerBase::SharedPtr timer_;

  // path messages (ready to publish once initialized)
  nav_msgs::msg::Path path_msgs[4];

  // dynamic look-ahead parameters
  double look_ahead_max, look_ahead_min;
  double curvature_max, curvature_min;

  // pure pursuit launch mode
  launch_mode_t launch_mode;

  // optimal paths to follow
  vector3d opt_paths[4];
  Spline2D* opt_path_splines[4];
  Spline2D* local_path_spline;
  int num_paths;

  // current car state
  car_state_t state;

  // current path
  path_type_t cur_path;

  // current values
  double speed_limit, min_speed;
  double target_steer;
  double look_ahead;
  double speed_scale_factor;
  double accel_max;

  bool path_switched;

public:
  PurePursuitNode();
  // ROS Callbacks
  void estop_callback(const std_msgs::msg::Bool::SharedPtr data);
  void engine_callback(const geometry_msgs::msg::Point32::SharedPtr data);
  void path_to_follow_callback(const std_msgs::msg::Int8::SharedPtr data);
  void local_path_callback(const nav_msgs::msg::Path::SharedPtr path);

#ifdef SIM
  void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose);
#else
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
#endif
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    
  void publish_paths();
    
  // main loop
  void main_loop();
};

#endif
