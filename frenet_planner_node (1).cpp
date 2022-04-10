#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp/executor.hpp>


//#include <tf/transform_datatypes.h>
//#include <tf/transform_listener.h>

#include "frenet_optimal_trajectory.hpp"

// Debug import
#include <sstream>
#include <type_traits>
#include "rclcpp/logger.hpp"
#include "rcutils/logging_macros.h"
#include "rclcpp/utilities.hpp"


geometry_msgs::msg::Pose pose;
geometry_msgs::msg::Twist twist;

geometry_msgs::msg::Pose opp_pose;
geometry_msgs::msg::Twist opp_twist;

obstacle opp_obs;

std::vector<obstacle> _obstacles;

int overtake_strategy = 3;

void odom_callback(nav_msgs::msg::Odometry::SharedPtr data)
{
    pose = data->pose.pose;
    twist = data->twist.twist;
}

//opp_obs odom data
void opp_odom_callback(nav_msgs::msg::Odometry::SharedPtr opp_data)
{
    opp_pose = opp_data->pose.pose;
    opp_twist = opp_data->twist.twist;

    opp_obs.x = opp_pose.position.x;
    opp_obs.y = opp_pose.position.y;
    opp_obs.radius = 0.7;
}

/**
void raw_obstacles_callback(const obstacle_detector::Obstacles::ConstPtr obstacles)
{ 
    std::vector<obstacle> temp_obs;
    
    obstacle temp_ob;
    for(const obstacle_detector::CircleObstacle obs : obstacles->circles)
    {
        temp_ob.x = obs.center.x;
        temp_ob.y = obs.center.y;
        temp_ob.radius = obs.true_radius;

        temp_obs.push_back(temp_ob);
    }

    _obstacles = temp_obs;
}
**/

void upload_static_obstacles(vecD obs_x, vecD obs_y)
{
    std::vector<obstacle> temp_obs;

    obstacle temp_ob;
    for(int i=0; i<obs_x.size(); i++)
    {
        temp_ob.x = obs_x.at(i);
        temp_ob.y = obs_y.at(i);
        temp_ob.radius = 0.1;

        temp_obs.push_back(temp_ob);
    }

    _obstacles = temp_obs;
}

void load_flag_path(vecD *vec_x, vecD *vec_y, vecD *vec_z, std::string trj_file, bool z=true)
{

    std::string line;
    size_t pos = 0;
    std::string temp;
    int i;


    std::ifstream is(trj_file);
    if (!is)
    {
	std::cout << trj_file << " not found!\n";
	return;
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
        vec_x->push_back(atof(line.substr(pos, std::string::npos).c_str()));

        //read y
        std::getline(is,line);
        pos = line.find(":") + 1;
        vec_y->push_back(atof(line.substr(pos, std::string::npos).c_str()));

        if(z)
        {
          //read speed
          std::getline(is,line);
          pos = line.find(":") + 1;
          vec_z->push_back(atof(line.substr(pos, std::string::npos).c_str()));
        }
    }

}

double calc_dis(double x1, double y1, double x2, double y2)
{
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}


void find_nearest_in_global_path(vecD global_x, vecD global_y, double &min_x, double &min_y, double &min_dis, int &min_id)
{
    double bot_x = pose.position.x;
    double bot_y = pose.position.y;

    min_dis = FLT_MAX;
    for(int i = 0; i < global_x.size(); i++)
    {
	double dis = calc_dis(global_x[i], global_y[i], bot_x, bot_y);
	if(dis < min_dis)
	{
	    min_dis = dis;
	    min_x = global_x[i];
	    min_y = global_y[i];
	    min_id = i;
	}
    }
}


double calc_s(double ptx, double pty, vecD global_x, vecD global_y)
{
    double s = 0;
    if(global_x[0] == ptx && global_y[0] == pty)
	return s;

    for(int i = 1; i < global_x.size(); i++)
    {
	double dis = calc_dis(global_x[i], global_y[i], global_x[i - 1], global_y[i - 1]);
	s = s + dis;

	if(global_x[i] == ptx && global_y[i] == pty)
	    break;
    }
    //cout<<"HO TROVATO S: "<<s<<endl;
    return s;
}


double get_bot_yaw()
{
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    return yaw;
}

vecD global_path_yaw(Spline2D csp, vecD gx, vecD gy)
{
    vecD yaw;
    vecD t = csp.calc_s(gx, gy);
    for(int i = 0; i < t.size(); i++)
	yaw.push_back(csp.calc_yaw(t[i]));
    return yaw;
}

double find_s(vecD global_x, vecD global_y)
{
    double min_x, min_y;
    int min_id;
    double c_d;

    // getting d
    find_nearest_in_global_path(global_x, global_y, min_x, min_y, c_d, min_id);
    return calc_s(min_x, min_y, global_x, global_y);
}

void initial_conditions(Spline2D csp, vecD global_x, vecD global_y, vecD ryaw, double &s0, double &c_speed, double &c_d, double &c_d_d, double &c_d_dd)
{
    double vx = twist.linear.x;
    double vy = twist.linear.y;
    double v = sqrt(vx*vx + vy*vy);


    double min_x, min_y;
    int min_id;

    // getting d
    find_nearest_in_global_path(global_x, global_y, min_x, min_y, c_d, min_id);

    // deciding the sign for d
    pair<double, double> vec1, vec2;
    vec1.first = pose.position.x - global_x[min_id];
    vec1.second = pose.position.y - global_y[min_id];

    vec2.first = global_x[min_id] - global_x[min_id + 1];
    vec2.second = global_y[min_id] - global_y[min_id + 1];
    double curl2D = vec1.first*vec2.second - vec2.first*vec1.second;
    if(curl2D < 0)
	c_d *= -1;
    
    s0 = calc_s(min_x, min_y, global_x, global_y);
    //cout<<"CALCOLATO S0: "<<s0<<endl;

    double bot_yaw = get_bot_yaw();

    vecD theta = global_path_yaw(csp, global_x, global_y);
    double g_path_yaw = theta[min_id];
    
    //cout<<"BOT_YAW: "<<bot_yaw<<"G_PATH_YAW: "<<g_path_yaw<<endl;
    
    double delta_theta = bot_yaw - g_path_yaw;
    
    //c_d_d = v*sin(delta_theta);
    
    double k_r = csp.calc_curvature(s0);

    c_speed = v*cos(delta_theta) / (1 - k_r*c_d);

    c_d_dd = 0;
}

//NOT SURE ABOUT THE CORRECTNESS
void create_path_msg(nav_msgs::msg::Path path_msg, FrenetPath path, vecD rk, vecD ryaw, double &c_speed, double &c_d, double &c_d_d)
{
    //cout<<"X SIZE: "<<path.x.size()<<endl;
    for(int i = 0; i < path.x.size(); i++)
    {
	geometry_msgs::msg::PoseStamped loc;
	loc.pose.position.x = path.x[i];
	loc.pose.position.y = path.y[i];

	double delta_theta = atan(c_d_d / ((1 - rk[i]*c_d)*c_speed));
	double yaw = delta_theta + ryaw[i];

	tf2::Quaternion q;
	q.setRPY(0, 0, yaw); // roll , pitch = 0
	q.normalize();
	loc.pose.orientation = tf2::toMsg(q);
	//loc.pose.orientation = tf2::Quaternion::toMsg(q);
    //quaternionTFToMsg(q, loc.pose.orientation);

	path_msg.poses.push_back(loc);
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr n = std::make_shared<rclcpp::Node>("frenet_planner");
    RCLCPP_INFO(n->get_logger(), "Starting");
    //ros::NodeHandle privateNodeHandler("~");
    //initializing variables
    std::string trj_file;
    std::string local_path_topic;
    std::string first_path_topic;
    std::string last_path_topic;
    std::string global_path_topic;
    std::string frenet_target_topic;
    std::string frenet_coo_topic;
    std::string obstacles_topic;
    std::string odom_topic;
    std::string path_available_topic;
    std::string ready_topic;

    //track boundaries
    std::string inner_trj;
    std::string outer_trj;


    //initializing parameters
    std::string opp_odom_topic;
    RCLCPP_INFO(n->get_logger(), "Declaring parameters");
    n->declare_parameter<std::string>("trj", "");
    n->declare_parameter<std::string>("local_path_topic", "/local_path");
    n->declare_parameter<std::string>("first_path_topic", "/first_path");
    n->declare_parameter<std::string>("last_path_topic", "/last_path");
    n->declare_parameter<std::string>("global_path_topic", "/global_path");
    n->declare_parameter<std::string>("path_available_topic", "/av_path");
    n->declare_parameter<std::string>("frenet_target_topic", "/frenet_target");
    n->declare_parameter<std::string>("frenet_coo_topic", "/frenet_coo");
    n->declare_parameter<std::string>("obstacles_topic", "");
    n->declare_parameter<std::string>("odom_topic", "/odom");
    n->declare_parameter<std::string>("ready_topic", "/ready");
    n->declare_parameter<std::string>("opp_odom_topic", "/opp_odom");
    n->declare_parameter<std::string>("inner_trj", "");
    n->declare_parameter<std::string>("outer_trj", "");
    
    //get parameters
    RCLCPP_INFO(n->get_logger(), "Getting parameters from yaml");
    n->get_parameter("trj", trj_file);
    n->get_parameter("local_path_topic", local_path_topic);
    n->get_parameter("first_path_topic", first_path_topic);
    n->get_parameter("last_path_topic", last_path_topic);
    n->get_parameter("global_path_topic", global_path_topic);
    n->get_parameter("path_available_topic", path_available_topic);
    n->get_parameter("frenet_target_topic", frenet_target_topic);
    n->get_parameter("frenet_coo_topic", frenet_coo_topic);
    n->get_parameter("obstacles_topic", obstacles_topic);
    n->get_parameter("odom_topic", odom_topic);
    n->get_parameter("ready_topic", ready_topic);
    n->get_parameter("opp_odom_topic", opp_odom_topic);
    n->get_parameter("inner_trj", inner_trj);
    n->get_parameter("outer_trj", outer_trj);
    RCLCPP_INFO(n->get_logger(), trj_file);

    //initializing publisher and subscriber
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frenet_path = n->create_publisher<nav_msgs::msg::Path>(local_path_topic, 1);		//Publish frenet path
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_first_path = n->create_publisher<nav_msgs::msg::Path>(first_path_topic, 1);		//Publish fist frenet path
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_last_path = n->create_publisher<nav_msgs::msg::Path>(last_path_topic, 1);		//Publish last frenet path
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path = n->create_publisher<nav_msgs::msg::Path>(global_path_topic, 1);		//Publish global path
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_available = n->create_publisher<std_msgs::msg::Bool>(path_available_topic, 1);		//Publish path available

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_target_pose = n->create_publisher<geometry_msgs::msg::PoseStamped>(frenet_target_topic, 10);
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_frenet_coo  = n->create_publisher<geometry_msgs::msg::PoseArray>(frenet_coo_topic, 10);			//Publish frenet path coordinates

    //ros::Subscriber raw_obstacles_sub =n->create_subscription("/raw_obstacles", 40, raw_obstacles_callback);  //statici
    //ros::Subscriber raw_obstacles_sub =n->create_subscription(obstacles_topic, 40, raw_obstacles_callback);  //dinamici
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub = n->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 40, odom_callback);		//Subscribe the initial conditions

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr ready_pub = n->create_publisher<std_msgs::msg::Int8>(ready_topic, 1);		//Publish global path

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opp_odom = n->create_subscription<nav_msgs::msg::Odometry>(opp_odom_topic, 40, opp_odom_callback); //opp_odom callback

    FrenetPath first_path, last_path;

    RCLCPP_INFO(n->get_logger(), "0");
    rclcpp::Rate rate(40);
    //ROS_INFO("Getting params");
    // get params
    /*n.param("/frenet_planner/path/max_speed", MAX_SPEED, 100.0);
    n.param("/frenet_planner/path/max_accel", MAX_ACCEL, 100.0);
    n.param("/frenet_planner/path/max_curvature", MAX_CURVATURE, 100.0);
    n.param("/frenet_planner/path/max_road_width", MAX_ROAD_WIDTH, 1.2);
    n.param("/frenet_planner/path/d_road_w", D_ROAD_W, 0.1);
    n.param("/frenet_planner/path/dt", DT, 0.3);
    n.param("/frenet_planner/path/maxt", MAXT, 1.5);
    n.param("/frenet_planner/path/mint", MINT, 1.4);    //1.2
    n.param("/frenet_planner/path/target_speed", TARGET_SPEED, 3.0);
    n.param("/frenet_planner/path/d_t_s", D_T_S, 1.0);
    n.param("/frenet_planner/path/n_s_sample", N_S_SAMPLE, 0.5);
    n.param("/frenet_planner/path/robot_radius", ROBOT_RADIUS, 0.3);
    n.param("/frenet_planner/cost/kj", KJ, 0.03);   //0.03  // piu' e' alto, piu' mantiene la stessa traiettoria
    n.param("/frenet_planner/cost/kt", KT, 0.0);           // preferisci traiettorie che richiedono piu' tempo, quindi quelle piu' lunghe
    n.param("/frenet_planner/cost/kd", KD, 2.0);    //2.0
    n.param("/frenet_planner/cost/klon", KLON, 1.0);
    n.param("/frenet_planner/cost/klat", KLAT, 1.0);*/
    // cout << "param " << MAX_SPEED << endl;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    //track boundaries
    vecD obs_x, obs_y, obs_vel;
    vecD int_x, int_y;
    vecD out_x, out_y;
    /*load_flag_path(&obs_x, &obs_y, &obs_vel, inner_trj);
    load_flag_path(&obs_x, &obs_y, &obs_vel, outer_trj);
    upload_static_obstacles(obs_x, obs_y);*/

    //loading trajectory and track boundary
    RCLCPP_INFO(n->get_logger(), "load inner trasj path");
    load_flag_path(&int_x, &int_y, &obs_vel, inner_trj, false);
    RCLCPP_INFO(n->get_logger(), "load outer trasj path");
    load_flag_path(&out_x, &out_y, &obs_vel, outer_trj, false);

    vecD wx, wy, speeds;
    RCLCPP_INFO(n->get_logger(), "load trasj path");
    load_flag_path(&wx, &wy, &speeds, trj_file);
    RCLCPP_INFO(n->get_logger(), "size %g", wx.size());
    // print wx for testing
    // for(int i =0; i < wx.size(); i++){
    //     RCLCPP_INFO(n->get_logger(), "elemento %g", wx[i]);
    // }
    
    vecD rx, ry, ryaw, rk;

    double ds = 0.1;	//ds represents the step size for cubic_spline

    //Global path is made using the waypoints
    // calculate the spline between the waypoints
    Spline2D csp = calc_spline_course(wx, wy, rx, ry, ryaw, rk, ds);
    RCLCPP_INFO(n->get_logger(), "Spline is made");
    nav_msgs::msg::Path path_msg;
    nav_msgs::msg::Path first_msg;
    nav_msgs::msg::Path last_msg;
    nav_msgs::msg::Path global_path_msg;

    // paths are published in map frame
    path_msg.header.frame_id = "map";
    first_msg.header.frame_id = "map";
    last_msg.header.frame_id = "map";
    global_path_msg.header.frame_id = "map";
    //Global path pushed into a message
    for(int i = 0; i < rx.size(); i++)
    {
        geometry_msgs::msg::PoseStamped loc;
        loc.pose.position.x = rx[i];
        loc.pose.position.y = ry[i];
        global_path_msg.poses.push_back(loc);
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "tempo spline = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    double s0, c_d, c_d_d, c_d_dd, c_speed ;

    geometry_msgs::msg::PoseStamped target_pose;
    FrenetPath path;

    //execution loop
    while(rclcpp::ok())
    {
        std_msgs::msg::Int8 ready;
        ready.data = 1;
        ready_pub->publish(ready);
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        //set initial condition
        initial_conditions(csp, rx, ry, ryaw, s0, c_speed, c_d, c_d_d, c_d_dd);
        //global_path->publish(global_path_msg);

        if(overtake_strategy < 0) 
        {
            rclcpp::spin_some(n);
            //std::cout << "SKIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIP \n";
            rate.sleep();
            continue;
        }
        RCLCPP_INFO(n->get_logger(), "33333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333");
        double actual_speed = std::min(10.0, sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y));
        //c_speed = std::min(0.3, sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y));
        c_speed = std::min(3.0, actual_speed);

        //std::cout<<"c_speed: "<<c_speed<<endl;
        for (auto const& ob : _obstacles)
        {
            c_speed = 0.5; //1.0
            break;
        }

        double x_int_min, y_int_min, c_in;
        double x_out_min, y_out_min, c_out;
        int id_in, id_out;
        //(vecD global_x, vecD global_y, double &min_x, double &min_y, double &min_dis, int &min_id)
        find_nearest_in_global_path(int_x, int_y, x_int_min, y_int_min, c_in, id_in);
        find_nearest_in_global_path(out_x, out_y, x_out_min, y_out_min, c_out, id_out);

        id_in += 15;
        id_out += 15;

        if(id_out >= out_x.size())
        {
            id_out = id_out - out_x.size();
        }

        if(id_in >= int_x.size())
        {
            id_in = id_in - int_x.size();
        }

        vector<obstacle> borders;

        for(int count=0; count < 30; count++)
        {
            obstacle o;
            o.x = int_x[id_in];
            o.y = int_y[id_in];
            o.radius = 0.5;

            id_in += 5;
            if(id_in >= int_x.size())
            {
                id_in = 0;
            }
            borders.push_back(o);

            o.x = out_x[id_out];
            o.y = out_y[id_out];
            o.radius = 0.5;

            id_out += 5;
            if(id_out >= out_x.size())
            {
                id_out = 0;
            }
            borders.push_back(o);
        }

        //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        path = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, borders, first_path, last_path, opp_obs, overtake_strategy);
        //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        //std::cout << "tempo iterazione = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

        std_msgs::msg::Bool available;
        available.data = true;
        //std::cout << "path empty: " << path.empty << "\n";
        if (path.empty)
        {
            cout<<"NO PATH"<<endl;
            //pub
            available.data = false;
            path_available->publish(available);
            rclcpp::spin_some(n);
            rate.sleep();
            continue;
        }

        //ROS_INFO("Frenet path created");

        nav_msgs::msg::Path path_msg;
        nav_msgs::msg::Path first_msg;
        nav_msgs::msg::Path last_msg;

        // paths are published in map frame
        path_msg.header.frame_id = "map";
        first_msg.header.frame_id = "map";
        last_msg.header.frame_id = "map";

        create_path_msg(path_msg, path, rk, ryaw, c_d, c_speed, c_d_d);
        frenet_path->publish(path_msg);
        path_available->publish(available);

        create_path_msg(first_msg, first_path, rk, ryaw, c_d, c_speed, c_d_d);
        create_path_msg(last_msg, last_path, rk, ryaw, c_d, c_speed, c_d_d);

        vector<geometry_msgs::msg::Pose> vecP;
        for (auto const& ob : borders)
        {
            geometry_msgs::msg::Pose px;
            px.position.x = ob.x;
            px.position.y = ob.y;
            vecP.push_back(px);
        }

        geometry_msgs::msg::PoseArray pArray;
        pArray.header.frame_id = "map";
        pArray.poses = vecP;

        pub_first_path->publish(first_msg);
        pub_last_path->publish(last_msg);

        pub_frenet_coo->publish(pArray);

        global_path->publish(global_path_msg);

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "tempo iterazione = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
        //std::cout<< std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;
        //return 0;
        //ROS_INFO("Path published");
        rclcpp::spin_some(n);
        rate.sleep();
    }
}
