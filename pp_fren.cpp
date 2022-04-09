#include "purepursuit_node.h"

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::placeholders::_1;

global local_path;

PurePursuitNode::PurePursuitNode()
{
  sub_node = std::make_shared<rclcpp::Node>("purepursuit_sub_node");
  pub_node = std::make_shared<rclcpp::Node>("purepursuit_pub_node");

  // Declare parameters
  pub_node->declare_parameter("params_file");
  std::string config_file = pub_node->get_parameter("params_file").as_string();
  
  // Callback groups are what the executor looks for when trying to run multiple threads
  callback_group_estop_sub_ = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_pose_sub_ = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_path_to_follow_sub_ = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_local_path_sub_ = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_plan_sub_ = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Everything assigned to a callback group gets bundled into the same thread
  auto estop_sub_opt = rclcpp::SubscriptionOptions();
  estop_sub_opt.callback_group = callback_group_estop_sub_;
  auto pose_sub_opt = rclcpp::SubscriptionOptions();
  pose_sub_opt.callback_group = callback_group_pose_sub_;
  auto path_to_follow_sub_opt = rclcpp::SubscriptionOptions();
  path_to_follow_sub_opt.callback_group = callback_group_path_to_follow_sub_;
  auto local_path_sub_opt = rclcpp::SubscriptionOptions();
  local_path_sub_opt.callback_group = callback_group_local_path_sub_;
  auto plan_sub_opt = rclcpp::SubscriptionOptions();
  plan_sub_opt.callback_group = callback_group_plan_sub_;

  // Init subscribers
  estop_sub = sub_node->create_subscription<std_msgs::msg::Bool>(
    "/commands/stop",
    1,
    std::bind(&PurePursuitNode::estop_callback, this, _1),
    estop_sub_opt);
  pose_sub = sub_node->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    1,
    std::bind(&PurePursuitNode::pose_callback, this, _1),
    pose_sub_opt);
  path_to_follow_sub = sub_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/path_to_follow",
    1,
    std::bind(&PurePursuitNode::path_to_follow_callback, this, _1),
    path_to_follow_sub_opt);
  local_path_sub = sub_node->create_subscription<nav_msgs::msg::Path>(
    "/local_path",
    1,
    std::bind(&PurePursuitNode::local_path_callback, this, _1),
    local_path_sub_opt);  
  plan_sub = sub_node->create_subscription<adx_msgs::msg::Plan>(
    "/plan",
    1,
    std::bind(&PurePursuitNode::plan_callback, this, _1),
    plan_sub_opt);

  // Init publishers
  drive_pub      = pub_node->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive_parameters", 1);
  spline_pos_pub = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>("/pp/spline_pos", 10);
  goal_pub       = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>("/pp/goal", 10);
  timer_ = pub_node->create_wall_timer(
      25ms, // 40-80 Hz
      std::bind(&PurePursuitNode::control_loop, this));

  // Init parameters from yaml configuration
  // TODO: MOVE THIS CONFIGURATIONS TO A SEPARATE YAML FILE
  YAML::Node params = YAML::LoadFile(config_file);
  
  speed_limit        = params["speed_limit"].as<double>();
  min_speed          = params["speed_min"].as<double>();
  speed_scale_factor = params["speed_scale_factor"].as<double>();

  look_ahead_max = params["look_ahead_max"].as<double>();
  look_ahead_min = params["look_ahead_min"].as<double>();
  look_ahead = look_ahead_min;

  // calc this dynamically
  curvature_max = params["curvature_max"].as<double>();
  curvature_min = params["curvature_min"].as<double>();

  accel_max = params["accel_max"].as<double>();

  std::cout << "Launch parameters:" << std::endl
       << "  speed_min     = " << min_speed      << std::endl
       << "  speed_limit   = " << speed_limit    << std::endl
       << "  lookahead = "     << look_ahead     << std::endl;

  opt_path_spline = nullptr;
  local_path = nullptr;
  cur_path = PATH_OPTIMAL;

  executor.add_node(pub_node);
  executor.add_node(sub_node);

  std::cout << "Init completed." << std::endl;
  executor.spin();
}

void PurePursuitNode::estop_callback(const std_msgs::msg::Bool::SharedPtr data)
{
  pp_status.ready = !(data->data);
}

/**
 * @brief Path to follow callback
 * We maybe need to adjust this logic with the advent of the frenet planner
 */
void PurePursuitNode::path_to_follow_callback(const std_msgs::msg::Int8::SharedPtr data)
{
  cur_path = static_cast<path_type_t>(data->data);
}

/**
 * @brief Odometry callback
 */
void PurePursuitNode::pose_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  state.x = odom->pose.pose.position.x;
  state.y = odom->pose.pose.position.y;
  state.yaw = tf2::getYaw<geometry_msgs::msg::Quaternion>(odom->pose.pose.orientation);
  state.speed = sqrt(std::pow(odom->twist.twist.linear.x, 2) + std::pow(odom->twist.twist.linear.y, 2));
  pp_status.pose_received = true;
}

void PurePursuitNode::local_path_callback(const nav_msgs::msg::Path path)
{
    global local_path_spline;
    Vec_d a=[];
    Vec_d b=[];
    double lx = 0;
    double ly = 0;
    for(geometry_msgs::msg::PoseStamped pose : path->poses)
    {
      double ax = pose.position.x;
      double ay = pose.position.y;

      if(local_path_spline != nullptr && ax == local_path_spline.calc_position(0)[0] && ay == local_path_spline.calc_position(0)[1])
        return;

      double distance = calc_distance((ax,ay),(lx,ly))
      if(i == 0 || distance > 0){
        a.push_back(ax);
        b.push_back(ay);
        lx = ax;
        ly = ay;
      }
    }

    local_path_spline = Spline2D(a,b);

}


void PurePursuitNode::plan_callback(const adx_msgs::msg::Plan::SharedPtr plan)
{
  std::cout << "Received new plan." << std::endl;
  pp_status.ready = false;
  std::cout << "Parsing..." << std::endl;
  // TODO: Check for same fixed frame in message header/internal
  // working frame and send warning if they differ

  for(adx_msgs::msg::PlanPoint point : plan->points)
  {
    plan_point_t plan_point;
    plan_point.position.x = point.position.x;
    plan_point.position.y = point.position.y;
    plan_point.position.z = point.position.z;
    plan_point.speed.x = point.speed.x;
    plan_point.speed.y = point.speed.y;
    plan_point.speed.z = point.speed.z;
    opt_plan.push_back(plan_point);
  }
  Vec_d plan_x, plan_y;
  for(plan_point_t plan_point : opt_plan)
  {
    plan_x.push_back(plan_point.position.x);
    plan_y.push_back(plan_point.position.y);
  }
  // calculate spline
  // TODO: fix memory leak and setup online plan change
  opt_path_spline = new Spline2D(plan_x, plan_y);
}

void PurePursuitNode::publish_debug_markers(point_t glob_s_pose, point_t target_pose)
{
  // Publish current goal on path
  geometry_msgs::msg::PoseStamped goal_msg = geometry_msgs::msg::PoseStamped();
  goal_msg.header.frame_id = "map";
  goal_msg.pose.position.x = target_pose.x;
  goal_msg.pose.position.y = target_pose.y;
  goal_pub->publish(goal_msg);

  // Publish current position on the path spline
  geometry_msgs::msg::PoseStamped pos_msg = geometry_msgs::msg::PoseStamped();
  pos_msg.header.frame_id = "map";
  pos_msg.pose.position.x = glob_s_pose.x;
  pos_msg.pose.position.y = glob_s_pose.y;
  spline_pos_pub->publish(pos_msg);
}

/**
 * @brief Calculates the steering angle needed
 * to reach the desired path goal
 * @param target The target point in local coordinates
 * @param speed The current speed of the vehicle
*/
double PurePursuitNode::pp_step(point_t target, double speed)
{
  double delta = atan2(target.y, target.x - L); // (y, x-L)
  double Lf = pp_k * speed + look_ahead;
  double alpha = atan2(2.0 * L * sin(delta) / Lf, 1.0);
  alpha = std::clamp(alpha, MIN_STEER, MAX_STEER);
  return alpha;
}

/***
 * @brief Calculates the optimal lookahead based
 * on the current curvature of the path.
 * @param curvature Path curvature
 */
double PurePursuitNode::la_step(double curvature)
{
  return (
         ((look_ahead_max - look_ahead_min) * (curvature - curvature_min))
         / (curvature_max - curvature_min))
         + look_ahead_min;
}

// Main control loop
void PurePursuitNode::control_loop()
{
  double target_speed = 0;

  if(!pp_status.pose_received)
  {
    std::cout << "No position received yet" << std::endl;
    return;
  }

  car_state_t cur_state = state;
  
  // TODO: implement logic to choose trajectory
  // Choose which trajectory to follow
  Spline2D* cur_path_spline; //path_splines[cur_path];
  plan_t* cur_opt_plan = &opt_plan; //&opt_paths[cur_path];

  if(cur_path == PATH_NONE)
  {
    // Decelerate if there's no feasible path
    target_speed -= accel_max*(1.0/PP_FREQUENCY);
    target_speed = max(target_speed, 0.0);
    target_steer = 0.0;
  }
  else
  {
    if (cur_path == PATH_FRENET)
    {
      cur_path_spline = local_path_spline;
    }
    else
    {
      cur_path_spline = opt_path_spline;
    }

    // Get current position on the spline
    cur_path_spline->update_current_s(cur_state.x, cur_state.y);
    point_t glob_s_pose = cur_path_spline->calc_position(cur_path_spline->cur_s);

    // Calculate optimal target speed based on current location
    // python used also k and state[speed] and  Look ahead
    double p_pos = cur_path_spline->cur_s + look_ahead;
    // python didn't use the % and the ceil function
    int speed_idx = ((int)ceil( (p_pos/cur_path_spline->s.back())*cur_opt_plan->size() )) % cur_opt_plan->size();
    // pyhon used % that was missing before
    target_speed = (*cur_opt_plan)[speed_idx].speed.x;
    //target_speed *= speed_scale_factor;
    
    // Limit speed based on min_speed and speed_limit
    // python also used another method, but idk what is the difference
    target_speed = max(target_speed, min_speed);
    target_speed = min(target_speed, speed_limit);

    // Calculate steer value from target pose
    double target_waypoint = cur_path_spline->cur_s + look_ahead;
    point_t target_pose = cur_path_spline->calc_position(target_waypoint);
    point_t pose;
    pose.x = cur_state.x;
    pose.y = cur_state.y;
    point_t local_target = point_transform(target_pose, pose, cur_state.yaw);
    //all the step above are similat to the python implementation, but is also crete the trj
    target_steer = pp_step(local_target, target_speed);

    // Update optimal look-ahead
    double curvature = std::clamp(cur_path_spline->calc_curvature(target_waypoint), curvature_min, curvature_max);
    look_ahead = la_step(curvature);

    // in python there are also some speed and k_accel/k_decel implementation

    publish_debug_markers(glob_s_pose, target_pose);
    
  }

  // Publish driving message
  ackermann_msgs::msg::AckermannDriveStamped msg = ackermann_msgs::msg::AckermannDriveStamped();
  msg.drive.speed = target_speed;
  msg.drive.steering_angle = target_steer;
  state.steer = target_steer;
  if(pp_status.ready)
    drive_pub->publish(msg);
  else
    std::cout << "Waiting for the start signal." << std::endl;

  //auto loop_end = high_resolution_clock::now();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  PurePursuitNode pp;
  rclcpp::shutdown();
  return 0;
}
