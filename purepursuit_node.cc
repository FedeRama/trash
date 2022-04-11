#define SIM
#include "purepursuit_node.h"

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::placeholders::_1;

using namespace std;

PurePursuitNode::PurePursuitNode()
{
  sub_node = std::make_shared<rclcpp::Node>("purepursuit_sub_node");
  pub_node = std::make_shared<rclcpp::Node>("purepursuit_pub_node");
  // Declare parameters
  //pub_node->declare_parameter("launch_mode");
  pub_node->declare_parameter("speed_limit");
  pub_node->declare_parameter("speed_min");
  pub_node->declare_parameter("speed_scale_factor");
  pub_node->declare_parameter("look_ahead_max");
  pub_node->declare_parameter("look_ahead_min");
  pub_node->declare_parameter("curvature_max");
  pub_node->declare_parameter("curvature_min");
  pub_node->declare_parameter("accel_max");
  pub_node->declare_parameter("trj_path");
  pub_node->declare_parameter("opt_trj");
  //pub_node->declare_parameter("center_trj");
  //pub_node->declare_parameter("left_trj");
  //pub_node->declare_parameter("right_trj");

  // Load launch mode
  int lm = 3;
  //lm = pub_node->get_parameter("launch_mode").as_int();
  cout << "launching in mode #" << (int)lm << endl;
  launch_mode = (launch_mode_t) lm;
  //if(launch_mode == MODE_TIMEATTACK)
    num_paths = 1;
  //else
  //  num_paths = 4;
  
  // Callback groups are what the executor looks for when trying to run multiple threads
  callback_group_estop_sub_ = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_pose_sub_ = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_scan_sub_ = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_engine_sub_ = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_path_to_follow_sub_ = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_local_path_sub_ = sub_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Everything assigned to a callback group gets bundled into the same thread
  auto estop_sub_opt = rclcpp::SubscriptionOptions();
  estop_sub_opt.callback_group = callback_group_estop_sub_;
  auto pose_sub_opt = rclcpp::SubscriptionOptions();
  pose_sub_opt.callback_group = callback_group_pose_sub_;
  auto scan_sub_opt = rclcpp::SubscriptionOptions();
  scan_sub_opt.callback_group = callback_group_scan_sub_;
  auto engine_sub_opt = rclcpp::SubscriptionOptions();
  engine_sub_opt.callback_group = callback_group_engine_sub_;
  auto path_to_follow_sub_opt = rclcpp::SubscriptionOptions();
  path_to_follow_sub_opt.callback_group = callback_group_path_to_follow_sub_;
  auto local_path_sub_opt = rclcpp::SubscriptionOptions();
  local_path_sub_opt.callback_group = callback_group_local_path_sub_;

  // Init subscribers
  estop_sub = sub_node->create_subscription<std_msgs::msg::Bool>(
    "/commands/stop",
    1,
    std::bind(&PurePursuitNode::estop_callback, this, _1),
    estop_sub_opt);
  pose_sub = sub_node->create_subscription<nav_msgs::msg::Odometry>(
    "/pf/pose/odom",
    1,
    std::bind(&PurePursuitNode::pose_callback, this, _1),
    pose_sub_opt);
  scan_sub = sub_node->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan",
    1,
    std::bind(&PurePursuitNode::scan_callback, this, _1),
    scan_sub_opt);
  engine_sub = sub_node->create_subscription<geometry_msgs::msg::Point32>(
    "/drive_act",
    1,
    std::bind(&PurePursuitNode::engine_callback, this, _1),
    engine_sub_opt);
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

  // Init publishers
#ifndef GIANNI
  drive_pub = pub_node->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);
#else
  drive_pub = pub_node->create_publisher<geometry_msgs::msg::Point32>("/drive_parameters", 1);
#endif
  for(int i = 0; i < num_paths; i++)
    path_pub[i]        = pub_node->create_publisher<nav_msgs::msg::Path>("/pp/path" + to_string(i), 10);
  spline_pos_pub  = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>("/pp/spline_pos", 10);
  goal_pub        = pub_node->create_publisher<geometry_msgs::msg::PoseStamped>("/pp/goal", 10);
  timer_ = pub_node->create_wall_timer(
      25ms, // 40-80 Hz
      std::bind(&PurePursuitNode::main_loop, this));

  // Init parameters
  speed_limit = pub_node->get_parameter("speed_limit").as_double();
  min_speed = pub_node->get_parameter("speed_min").as_double();
  speed_scale_factor = pub_node->get_parameter("speed_scale_factor").as_double();

  look_ahead_max = pub_node->get_parameter("look_ahead_max").as_double();
  look_ahead_min = pub_node->get_parameter("look_ahead_min").as_double();
  look_ahead = look_ahead_min;

  // calc this dynamically
  curvature_max = pub_node->get_parameter("curvature_max").as_double();
  curvature_min = pub_node->get_parameter("curvature_min").as_double();

  accel_max = pub_node->get_parameter("accel_max").as_double();

  cout << "Loaded parameters:" << endl
       << "  min_speed     = " << min_speed      << endl
       << "  speed_limit   = " << speed_limit    << endl
       << "  lookahead_min = " << look_ahead_max << endl
       << "  lookahead_max = " << look_ahead_max << endl;

  // Load optimal trajectory
  cout << "Loading optimal trajectories" << endl;
  //cout << opt_path_splines[PATH_OPTIMAL]->s.size()<< endl;
  // TODO: Serialize Spline2D object to file
  // in a different software and load the built spline
  // here. It takes too much to build the spline on the
  // go if you have to load more than 1 (~2min on ultra96
  // for 4 splines).
  //
  // OR
  //
  // Load from a published path message a precise
  // enough path (number of waypoints for it to be
  // precise enough depends on the track length)
  if(launch_mode == MODE_FRENET)
  {
    cout << "Building splines" << endl;
    string trj_path;
    trj_path = pub_node->get_parameter("opt_trj").as_string();
    opt_paths[PATH_OPTIMAL] = load_flag_path(trj_path);
    opt_path_splines[PATH_OPTIMAL] = local_path_spline;
    cout << "Frenet spline built." << endl;
    
  }

  if(launch_mode == MODE_HEAD2HEAD)
  {
    cout << "Building splines" << endl;
    string trj_path;

    // Optimal path
    //private_node_handler.param<std::string>("opt_trj", trj_path, "");
    trj_path = pub_node->get_parameter("opt_trj").as_string();
    opt_paths[PATH_OPTIMAL] = load_flag_path(trj_path);
    opt_path_splines[PATH_OPTIMAL] = new Spline2D(opt_paths[PATH_OPTIMAL].xs, opt_paths[PATH_OPTIMAL].ys);
    cout << "Optimal spline built." << endl;
    
/*
    // Center path
    private_node_handler.param<std::string>("center_trj", trj_path, "");
    opt_paths[PATH_CENTER] = load_flag_path(trj_path);
    opt_path_splines[PATH_CENTER] = new Spline2D(opt_paths[PATH_CENTER].xs, opt_paths[PATH_CENTER].ys);
    cout << "Center spline built." << endl;

    // Left path
    private_node_handler.param<std::string>("left_trj", trj_path, "");
    opt_paths[PATH_LEFT] = load_flag_path(trj_path);
    opt_path_splines[PATH_LEFT] = new Spline2D(opt_paths[PATH_LEFT].xs, opt_paths[PATH_LEFT].ys);
    cout << "Left spline built." << endl;

    // Right path
    private_node_handler.param<std::string>("right_trj", trj_path, "");
    opt_paths[PATH_RIGHT] = load_flag_path(trj_path);
    opt_path_splines[PATH_RIGHT] = new Spline2D(opt_paths[PATH_RIGHT].xs, opt_paths[PATH_RIGHT].ys);
    cout << "Right spline built." << endl;
*/}
  cur_path = PATH_OPTIMAL;
  path_switched = false;

  cur_path = PATH_OPTIMAL;
  path_switched = false;
  build_path_msgs();
  publish_paths();
  
  executor.add_node(pub_node);
  executor.add_node(sub_node);

  cout << "Init done!" << endl;
  executor.spin();
}

void PurePursuitNode::local_path_callback(const nav_msgs::msg::Path::SharedPtr path)
{
	cout<<"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n";
	Vec_d a;
	Vec_d b;
	double lx = 0;
	double ly = 0;
	for(geometry_msgs::msg::PoseStamped data : path->poses)
	{
		double ax = data.pose.position.x;
		double ay = data.pose.position.y;

		double distance = (((ax - lx)*(ax - lx)) + ((ay - ly)*(ay - ly)));
		a.push_back(ax);
		b.push_back(ay);
		lx=ax;
		ly=ay;
	}
	local_path_spline = new Spline2D(a,b);
	cout<<"ccccccc  "<<a.size()<<"  cccccc  "<<b.size()<<"  cccccccc  "<<local_path_spline->cur_s<<"  cccccccccccccccccccccccccc\n";
}

void PurePursuitNode::build_path_msgs()
{
  for(int p = 0; p < 1; p++)
  {
    const int msg_size = 200;
    double step = opt_path_splines[p]->s.back()/msg_size;

    std::vector<double> xs, ys;
    point_t point;
    
    for(double i = step; i < opt_path_splines[p]->s.back()-step; i+=step)
    {
      
      point = opt_path_splines[p]->calc_position(i);
      xs.push_back(point.x);
      ys.push_back(point.y);
    }

    path_msgs[p].header.frame_id = "map";

    for(uint i = 0; i < xs.size(); i++)
    {
      geometry_msgs::msg::PoseStamped pose = geometry_msgs::msg::PoseStamped();
      pose.pose.position.x = xs[i];
      pose.pose.position.y = ys[i];
      path_msgs[p].poses.push_back(pose);
    }
  }
}

void PurePursuitNode::estop_callback(const std_msgs::msg::Bool::SharedPtr data)
{
  state.ready = !(data->data);
}

void PurePursuitNode::path_to_follow_callback(const std_msgs::msg::Int8::SharedPtr data)
{
  if(cur_path != static_cast<path_type_t>(data->data))
    path_switched = true;
  cur_path = static_cast<path_type_t>(data->data);
}

/**
 * @brief Engine speed callback
 */
void PurePursuitNode::engine_callback(const geometry_msgs::msg::Point32::SharedPtr data)
{
  state.speed = data->x*RPM2MS;
}

#ifdef SIM
/**
 * @brief Simulator pose callback
 */
void PurePursuitNode::pose_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  state.x = odom->pose.pose.position.x;
  state.y = odom->pose.pose.position.y;
  state.yaw = tf2::getYaw<geometry_msgs::msg::Quaternion>(odom->pose.pose.orientation);
  state.speed = odom->twist.twist.linear.x;
  state.pose_received = true;
}
#else
/**
 * @brief Particle filtered pose callback
 */
void PurePursuitNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  state.x     = pose->pose.position.x;
  state.y     = pose->pose.position.y;
  state.yaw   = tf2::getYaw<geometry_msgs::msg::Quaternion>(pose->pose.orientation);
  state.pose_received = true;
}
#endif

// TODO: implement crash detection
// Is crash detection doable without the scan callback?
void PurePursuitNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  ;
}

/**
 * @brief Publish loaded paths
*/
void PurePursuitNode::publish_paths()
{
  // The left/center/right paths are published by path_chooser node.
  for(int i = 0; i < num_paths; i++)
    path_pub[i]->publish(path_msgs[i]);
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
  alpha = min(MAX_STEER, alpha);
  alpha = max(MIN_STEER, alpha);
  return alpha;
}

/***
 * @brief Calculates the optimal lookahead based
 * on the current curvature of the path.
 * @param curvature Path curvature
 */
double PurePursuitNode::la_step(double curvature)
{
  //cout << "lookahead = [" << look_ahead_min << ", " << look_ahead_max << "]" << endl;
  //cout << "curvature = [" << curvature_min << ", " << curvature_max << "]" << endl;
  //cout << "act_curvature = " << curvature << endl;
  return (
         ((look_ahead_max - look_ahead_min) * (curvature - curvature_min))
         / (curvature_max - curvature_min))
         + look_ahead_min;
}

// Main control loop
void PurePursuitNode::main_loop()
{
  //cout << "Starting main loop." << endl;

  // Start multithreaded callback spinner
  //ros::AsyncSpinner spinner(5);
  //spinner.start();

  double target_speed = 0;

  //ros::Rate rate(PP_FREQUENCY); // 40-80
  //while(rclcpp::ok())
  //{
    auto loop_start = high_resolution_clock::now();

    if(!state.pose_received)
    {
      //cout << "No position received yet" << endl;
      //rate.sleep();
      //continue;
      return;
    }

    car_state_t cur_state = state;
    cout << cur_state.x << "," << cur_state.y << "," << cur_state.yaw << ",";
    cout << cur_state.speed << "," << cur_state.steer << ",";

    // Choose trajectory with path chooser
    //Spline2D* cur_path_spline = opt_path_splines[cur_path];
    Spline2D* cur_path_spline = local_path_spline;
    vector3d* cur_opt_path = &opt_paths[cur_path];

    if(cur_path == PATH_NONE)
    {
      target_speed -= accel_max*(1.0/PP_FREQUENCY);
      target_speed = max(target_speed, 0.0);
      target_steer = 0.0;
    }
    else
    {

      // TODO: IMPLEMENT REVERSE
      //if(reverse)
      //    do_things;
      //    continue;

      // Keep pose on spline updated for each path
      for (int i = 0; i < num_paths; i++)
	local_path_spline->update_current_s(cur_state.x, cur_state.y);
        //opt_path_splines[i]->update_current_s(cur_state.x, cur_state.y);
                

      // Get spline coordinates
      point_t glob_s_pose = cur_path_spline->calc_position(cur_path_spline->cur_s);

      cout << glob_s_pose.x << "," << glob_s_pose.y << ",";

      // Calc optimal target speed based on current location
      double p_pos = cur_path_spline->cur_s + L;
      int speed_idx = ((int)ceil( (p_pos/cur_path_spline->s.back())*cur_opt_path->speed.size() )) % cur_opt_path->speed.size();
      target_speed = cur_opt_path->speed[speed_idx];
      //target_speed *= speed_scale_factor;
            
      // Apply speed reduce
      /*if(path_switched)
      {
        double max_acc_speed = target_speed + accel_max*(1.0/PP_FREQUENCY);
        double max_dec_speed = target_speed - accel_max*(1.0/PP_FREQUENCY);
        target_speed = min(target_speed, max_acc_speed);
        target_speed = max(target_speed, max_dec_speed);
        path_switched = false;
      }*/

      // Limit speed based on min_speed and speed_limit
      target_speed = max(target_speed, min_speed);
      target_speed = min(target_speed, speed_limit);

      // Calc steer value from target pose
      double target_waypoint = cur_path_spline->cur_s + look_ahead;
      point_t target_pose = cur_path_spline->calc_position(target_waypoint);
      point_t pose;
      pose.x = cur_state.x;
      pose.y = cur_state.y;
      point_t local_target = point_transform(target_pose, pose, cur_state.yaw);
      target_steer = pp_step(local_target, target_speed);

      /*
      k_accel = 0.55 if speed > 12 else 0.3#0.7
      k_decel = 0.05 #0.02#0.05

      diff_speed = state["speed"] - speed
      if diff_speed > 1:
        speed = state["speed"] - k_decel*abs(diff_speed)

      if diff_speed < 3 and state["speed"] > 1:
        speed = state["speed"] + k_accel*abs(diff_speed)
      #print("-------------- REDUCING: ", speed)

      kd_steer = 0.5 if speed < 19 else 0.05
      */

      // Publish path goal
      geometry_msgs::msg::PoseStamped goal_msg = geometry_msgs::msg::PoseStamped();
      goal_msg.header.frame_id = "map";
      goal_msg.pose.position.x = target_pose.x;
      goal_msg.pose.position.y = target_pose.y;
      goal_pub->publish(goal_msg);

      // Publish closest actual position on the path spline
      geometry_msgs::msg::PoseStamped pos_msg = geometry_msgs::msg::PoseStamped();
      pos_msg.header.frame_id = "map";
      pos_msg.pose.position.x = glob_s_pose.x;
      pos_msg.pose.position.y = glob_s_pose.y;
      spline_pos_pub->publish(pos_msg);

      // Update optimal look-ahead
      /*
      double curvature = cur_path_spline->calc_curvature(target_waypoint);
      curvature = max(curvature, curvature_min);
      curvature = min(curvature, curvature_max);
      look_ahead = la_step(curvature);
      */
      //cout << "Next lookahead: " << look_ahead << endl;
    }

    // Publish drive data
#ifndef GIANNI
    ackermann_msgs::msg::AckermannDriveStamped msg = ackermann_msgs::msg::AckermannDriveStamped();
    msg.drive.speed = target_speed;
    msg.drive.steering_angle = target_steer;
    state.steer = target_steer;
#else
    geometry_msgs::msg::Point32 msg = geometry_msgs::msg::Point32();
    //cout << "final target speed: " << target_speed << endl;
    msg.x = target_speed/RPM2MS; // convert to engine RPM
    msg.y = target_steer;
    state.steer = target_steer;
#endif
    cout << target_speed << "," << target_steer << endl;
    if(state.ready)
    {
      drive_pub->publish(msg);
      /*cout << "Driving data:" << endl
             << "\tspeed = " << target_speed << endl
             << "\tsteer = " << target_steer << endl
             << "\tlook_ahead = " << look_ahead << endl;*/
    }
    else
      cout << "Waiting for start signal" << endl;

    // Republish path(s).
    // TODO: move this to a separate, low frequency (1Hz) pthread
    publish_paths();

    auto loop_end = high_resolution_clock::now();
//  cout << "Pure Pursuit main loop completed." << endl
//       << "\tLatency: " << duration_cast<microseconds>(loop_end - loop_start).count() << "us" << endl
//       << "\tPotential max freq: " << 1000000.0/duration_cast<microseconds>(loop_end - loop_start).count() << "Hz" << endl;
    //rate.sleep();
  //}
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  PurePursuitNode pp;
  //pp.main_loop();
  rclcpp::shutdown();
  return 0;
}
