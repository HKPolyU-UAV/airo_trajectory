#ifndef AIRO_TRAJECTORY_UTILS_H
#define AIRO_TRAJECTORY_UTILS_H

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <fstream>
#include <ctime>
#include <cstdlib>
#include <iomanip>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <airo_message/FSMInfo.h>
#include <airo_message/TakeoffLandTrigger.h>
#include <airo_message/ReferenceStamped.h>
#include <airo_message/ReferencePreview.h>

class AIRO_TRAJECTORY_SERVER{
    private:

    // Parameters
    std::string POSE_TOPIC, TWIST_TOPIC, CONTROLLER_TYPE;
    bool RESULT_SAVE = false,RESULT_PLOT = false, PUB_DEBUG = false;
    int FSM_FREQUENCY;

    ros::Subscriber local_pose_sub, local_twist_sub, fsm_info_sub, attitude_target_sub, battery_sub, mpc_debug_sub, backstepping_debug_sub, slidingmode_debug_sub;
    ros::Publisher command_pub, command_preview_pub, takeoff_land_pub;
    airo_message::FSMInfo fsm_info;
    geometry_msgs::PoseStamped local_pose;
    geometry_msgs::TwistStamped local_twist;
    mavros_msgs::AttitudeTarget attitude_target;
    sensor_msgs::BatteryState battery_state;
    double current_twist_norm;

    // For data logging
    std::string log_path;
    std::vector<double> line_to_push; // t,ref_x,x,ref_y,y,ref_z,z,ref_u,u,ref_v,v,ref_w,w,ref_phi,phi,ref_theta,theta,ref_psi,psi,thrust
    std::vector<double> debug_msg;
    bool traj_started = false;
    ros::Time traj_start_time;

    // For MPC
    bool mpc_enable_preview = false, use_preview = false;
    const int preview_size = 21;
    int row_interval;

    public:

    AIRO_TRAJECTORY_SERVER(ros::NodeHandle&);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr&);
    void twist_cb(const geometry_msgs::TwistStamped::ConstPtr&);
    void fsm_info_cb(const airo_message::FSMInfo::ConstPtr&);
    void attitude_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr&);
    void battery_cb(const sensor_msgs::BatteryState::ConstPtr&);
    void mpc_debug_cb(const std_msgs::Float64MultiArray::ConstPtr&);
    void backstepping_debug_cb(const std_msgs::Float64MultiArray::ConstPtr&);
    void slidingmode_debug_cb(const std_msgs::Float64MultiArray::ConstPtr&);
    void pose_cmd(const geometry_msgs::Pose&);
    void pose_cmd(const geometry_msgs::Pose&, const geometry_msgs::Twist&);
    void pose_cmd(const geometry_msgs::Pose&, const geometry_msgs::Twist&, const geometry_msgs::Accel&);
    void pose_cmd(const geometry_msgs::Point&, const double& yaw_angle = 0.0);
    void pose_cmd(const geometry_msgs::Point&, const geometry_msgs::Twist&, const double& yaw_angle = 0.0);
    void pose_cmd(const geometry_msgs::Point&, const geometry_msgs::Twist&, const geometry_msgs::Accel&, const double& yaw_angle = 0.0);
    int is_pose_initialized(const geometry_msgs::Pose&);
    bool target_reached(const geometry_msgs::Pose&);
    bool target_reached(const geometry_msgs::Point&);
    bool target_reached(const geometry_msgs::Point&, const double&);
    geometry_msgs::Quaternion yaw2q(double);
    double q2yaw(const geometry_msgs::Quaternion&);
    Eigen::Vector3d q2rpy(const geometry_msgs::Quaternion&);
    geometry_msgs::Quaternion rpy2q(const Eigen::Vector3d&);
    void file_traj_init(const std::string&, std::vector<std::vector<double>>&);
    geometry_msgs::Pose get_start_pose(const std::vector<std::vector<double>>&);
    geometry_msgs::Pose get_end_pose(const std::vector<std::vector<double>>&);
    bool file_cmd(const std::vector<std::vector<double>>& traj, int&);
    void assign_position(const std::vector<std::vector<double>>&, airo_message::ReferencePreview&);
    void assign_position(const std::vector<double>&, airo_message::ReferenceStamped&);
    void assign_twist(const std::vector<std::vector<double>>&, airo_message::ReferencePreview&);
    void assign_twist(const std::vector<double>&, airo_message::ReferenceStamped&);
    void assign_accel(const std::vector<std::vector<double>>&, airo_message::ReferencePreview&);
    void assign_accel(const std::vector<double>&, airo_message::ReferenceStamped&);
    void assign_yaw(const std::vector<std::vector<double>>&, airo_message::ReferencePreview&);
    void assign_yaw(const std::vector<double>&, airo_message::ReferenceStamped&);
    bool takeoff();
    bool land();
    void update_log(const airo_message::ReferencePreview&);
    void update_log(const airo_message::ReferenceStamped&);
    void init_log();
};

#endif