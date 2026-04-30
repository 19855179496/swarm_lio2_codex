#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <cascadePID.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>

using namespace std;
using namespace Eigen;

ros::Publisher control_RPM_pub;

Vector3d pos_des,vel_des,acc_des;
double yaw_des = 0.0;
cascadePID quad_PID(20);
double max_speed = 1; // m/s, can be overridden by rosparam
double slow_radius = 0.5; // start slowing when within this radius (m)
double arrival_threshold = 0.05; // within this distance consider arrived (m)

ros::Time t_init,t2;
int drone_id;
double init_x=0, init_y=0, init_z=0;
bool wait_for_start_signal = false;
bool started = true;
std::string start_topic = "/start_figure8";

nav_msgs::Odometry current_odom, last_odom;
int first_odom_flag = 0;
void Odom_callback(const nav_msgs::Odometry& odom)
{
    if(first_odom_flag == 1)
    {
        last_odom = current_odom;
    }else{
        first_odom_flag = 1;
    }
    current_odom = odom;
    ROS_INFO("current odom received");
}

geometry_msgs::PoseStamped pose_cmd;
int control_flag = 0;
void start_signal_callback(const std_msgs::Bool::ConstPtr& msg)
{
    if(!started && msg->data)
    {
        started = true;
        ROS_WARN("[cascadePID] start signal received on %s, controller enabled.", start_topic.c_str());
    }
}

void cmd_callback(const geometry_msgs::PoseStamped& msg)
{
    if(!started)
    {
        return;
    }
    pose_cmd = msg;
    // apply per-drone XY bias: drone_id * 1.5 meters
    pose_cmd.pose.position.x += init_x;
    pose_cmd.pose.position.y += init_y;
    control_flag = 1;
}

void fuel_position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd) {
    if(!started)
    {
        return;
    }
  pos_des = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  vel_des = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  acc_des = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);

  yaw_des = cmd->yaw;
}

void run_control(const ros::TimerEvent& event)
{
        if(first_odom_flag == 1) 
        {

        // ROS_INFO("start running controller");

        // compute actual loop dt from timer event
        double dt = (event.current_real - event.last_real).toSec();
        if(dt <= 0.0) dt = 0.005; // fallback

        //run controller
        Vector3d current_euler;
        Eigen::Quaterniond quaternion_temp(current_odom.pose.pose.orientation.w, current_odom.pose.pose.orientation.x\
                                        , current_odom.pose.pose.orientation.y, current_odom.pose.pose.orientation.z);
        Vector3d current_pos, current_vel;
        current_pos << current_odom.pose.pose.position.x, current_odom.pose.pose.position.y, current_odom.pose.pose.position.z;
        current_vel << current_odom.twist.twist.linear.x, current_odom.twist.twist.linear.y, current_odom.twist.twist.linear.z; 
        quad_PID.FeedbackInput(current_pos, current_vel, quaternion_temp);
        // ROS_INFO("current_pos = %f,%f,%f",current_pos(0),current_pos(1),current_pos(2));
        Eigen::Vector3d eulerAngle_des;
        eulerAngle_des << 0.1,0.1,0.0;
        // quad_PID.setAngledes(eulerAngle_des);

        // ROS_INFO("set des angle");

        if(control_flag == 1)
        {
            // Limit the commanded motion and implement a slow-down and arrival deadband to avoid oscillation
            Eigen::Vector3d goal_pos;
            goal_pos << pose_cmd.pose.position.x, pose_cmd.pose.position.y, pose_cmd.pose.position.z;
            Eigen::Vector3d delta = goal_pos - current_pos;
            double dist = delta.norm();
            // if very close to goal, treat as arrived
            if(dist <= arrival_threshold) {
                pos_des = goal_pos;
                vel_des.setZero();
                // stop updating target until a new cmd arrives
                control_flag = 0;
            } else {
                // compute a scaled speed: reduce speed proportionally when within slow_radius
                double speed = max_speed;
                if(dist < slow_radius && slow_radius > 1e-6) {
                    speed = max_speed * (dist / slow_radius);
                    // enforce a minimum small speed to avoid stalling before arrival
                    speed = std::max(speed, 0.05);
                }
                double step = speed * dt;
                if(dist <= step) {
                    // within one step, go to goal but set zero velocity to prevent oscillation
                    pos_des = goal_pos;
                    vel_des.setZero();
                    control_flag = 0;
                } else {
                    Eigen::Vector3d dir = delta / dist;
                    pos_des = current_pos + dir * step;
                    vel_des = dir * speed;
                }
            }

            Eigen::Quaterniond q_des(pose_cmd.pose.orientation.w, pose_cmd.pose.orientation.x\
                                        , pose_cmd.pose.orientation.y, pose_cmd.pose.orientation.z);
            Eigen::Matrix3d R_des;
            R_des = q_des.matrix();
            Eigen::Vector3d x_body;
            x_body << 1,0,0;
            x_body = R_des * x_body;

            yaw_des = atan2(x_body(1),x_body(0));//pose_cmd.pose.orientation.z
            t2 = ros::Time::now();
            // yaw_des = 3.1415926;//20.8 * (t2-t_init).toSec()
        }
        
        quad_PID.setOdomdes(pos_des,vel_des,acc_des, yaw_des);

        // Ensure vel_des does not exceed max_speed in case position_cmd message directly set velocities
        if(vel_des.norm() > max_speed && vel_des.norm() > 1e-6) {
            vel_des = vel_des.normalized() * max_speed;
        }

        // ROS_INFO("set des odom");

        quad_PID.RunController();
        Eigen::Vector3d Torque_des;
        Torque_des = quad_PID.getTorquedes();
        ROS_INFO("Torque_des = %f,%f,%f",Torque_des(0),Torque_des(1),Torque_des(2));
        Vector4d RPM_output;
        RPM_output = quad_PID.getRPMoutputs();
        // ROS_INFO("RPM_output = %f,%f,%f,%f",RPM_output(0),RPM_output(1),RPM_output(2),RPM_output(3));

        std_msgs::Float32MultiArray rpm_array;
        float rpm_data[4];
        rpm_data[0] = RPM_output(0);
        rpm_data[1] = RPM_output(1);
        rpm_data[2] = RPM_output(2);
        rpm_data[3] = RPM_output(3);
        // rpm_array.data = rpm_data;
        rpm_array.data.clear();
        rpm_array.data.push_back(RPM_output(0));
        rpm_array.data.push_back(RPM_output(1));
        rpm_array.data.push_back(RPM_output(2));
        rpm_array.data.push_back(RPM_output(3));
        control_RPM_pub.publish(rpm_array);

        }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cascade_PID");

    ros::NodeHandle n("~");



    double init_yaw;
    double controller_rate,angle_stable_time,damping_ratio;

    std::string quad_name;
    n.param("init_state_x", init_x, 0.0);
    n.param("init_state_y", init_y, 0.0);
    n.param("init_state_z", init_z, 1.0);
    n.param("init_state_yaw", init_yaw, 1.0);
    init_yaw = init_yaw / 180.0 * M_PI;
    yaw_des = init_yaw;
    n.param("controller_rate", controller_rate, 200.0);
    n.param("quadrotor_name", quad_name, std::string("quadrotor"));
    n.param("angle_stable_time", angle_stable_time, 0.1);
    n.param("damping_ratio", damping_ratio, 0.7);
    n.param("drone_id", drone_id, 0);
    n.param("wait_for_start_signal", wait_for_start_signal, false);
    n.param("start_topic", start_topic, std::string("/start_figure8"));
    started = !wait_for_start_signal;
    // read max_speed param (m/s)
    n.param("max_speed", max_speed, 0.5);
    // read slow radius and arrival threshold
    n.param("slow_radius", slow_radius, 0.5);
    n.param("arrival_threshold", arrival_threshold, 0.05);

    control_RPM_pub = n.advertise<std_msgs::Float32MultiArray>("cmd_RPM", 100);
    ros::Subscriber odom_sub = n.subscribe("odom", 100, Odom_callback,
                                           ros::TransportHints().tcpNoDelay());
    ros::Subscriber cmd_sub = n.subscribe("cmd_pose", 100, cmd_callback,
                                          ros::TransportHints().tcpNoDelay());
    ros::Subscriber position_cmd_sub_ = n.subscribe("position_cmd", 10, fuel_position_cmd_callback,
                                                    ros::TransportHints().tcpNoDelay());
    ros::Subscriber start_sub;
    if(wait_for_start_signal)
    {
        start_sub = n.subscribe(start_topic, 1, start_signal_callback,
                                ros::TransportHints().tcpNoDelay());
        ROS_WARN("[cascadePID] waiting start signal on %s", start_topic.c_str());
    }

    quad_PID.setdroneid(drone_id);

    ros::Timer controller_timer = n.createTimer(ros::Duration(1.0/controller_rate), run_control);

    Matrix3d Internal_mat;
    Internal_mat << 2.64e-3,0,0,
                    0,2.64e-3,0,
                    0,0,4.96e-3;
    double arm_length = 0.22;
    double k_F = 8.98132e-9;
    k_F = 3.0*k_F;
    double mass = 1.9;

    // cascadePID quad_PID(controller_rate);
    quad_PID.setrate(controller_rate);
    quad_PID.setInternal(mass, Internal_mat, arm_length, k_F);
    quad_PID.setParam(angle_stable_time,damping_ratio);

    ros::Rate rate(controller_rate);
    rate.sleep();

        pos_des << init_x,init_y,init_z;
        vel_des << 0,0,0;
        acc_des << 0,0,0;

    t_init = ros::Time::now();

    ros::spin();

    return 0;
  
}