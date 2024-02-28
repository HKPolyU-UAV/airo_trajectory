#include "airo_trajectory/airo_trajectory_server.h"

AIRO_TRAJECTORY_SERVER::AIRO_TRAJECTORY_SERVER(ros::NodeHandle& nh){
    nh.getParam("airo_control_node/fsm/pose_topic",POSE_TOPIC);
    nh.getParam("airo_control_node/fsm/twist_topic",TWIST_TOPIC);
    nh.getParam("airo_control_node/fsm/controller_type",CONTROLLER_TYPE);
    nh.getParam("/file_trajectory_node/result_save",RESULT_SAVE);
    nh.getParam("airo_control_node/fsm/fsm_frequency",FSM_FREQUENCY);
    local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(POSE_TOPIC,5,&AIRO_TRAJECTORY_SERVER::pose_cb,this);
    local_twist_sub = nh.subscribe<geometry_msgs::TwistStamped>(TWIST_TOPIC,5,&AIRO_TRAJECTORY_SERVER::twist_cb,this);
    fsm_info_sub = nh.subscribe<airo_message::FSMInfo>("/airo_control/fsm_info",1,&AIRO_TRAJECTORY_SERVER::fsm_info_cb,this);
    attitude_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",5,&AIRO_TRAJECTORY_SERVER::attitude_target_cb,this);
    battery_sub = nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",1,&AIRO_TRAJECTORY_SERVER::battery_cb,this);
    command_pub = nh.advertise<airo_message::Reference>("/airo_control/setpoint",1);
    command_preview_pub = nh.advertise<airo_message::ReferencePreview>("/airo_control/setpoint_preview",1);
    takeoff_land_pub = nh.advertise<airo_message::TakeoffLandTrigger>("/airo_control/takeoff_land_trigger",1);
    if (CONTROLLER_TYPE == "mpc"){
        nh.getParam("airo_control_node/mpc/enable_preview",mpc_enable_preview);
        if (mpc_enable_preview){
            use_preview = true;
        }
        nh.getParam("airo_control_node/mpc/pub_debug",PUB_DEBUG);
        if (PUB_DEBUG){
            mpc_debug_sub = nh.subscribe<std_msgs::Float64MultiArray>("/airo_control/mpc/debug",1,&AIRO_TRAJECTORY_SERVER::mpc_debug_cb,this);
            debug_msg.resize(preview_size+3);
        }
    }
    else if (CONTROLLER_TYPE == "backstepping"){
        nh.getParam("airo_control_node/backstepping/pub_debug",PUB_DEBUG);
        if (PUB_DEBUG){
            backstepping_debug_sub = nh.subscribe<std_msgs::Float64MultiArray>("/airo_control/backstepping/debug",1,&AIRO_TRAJECTORY_SERVER::backstepping_debug_cb,this);
            debug_msg.resize(8);
        }
    }
    else if (CONTROLLER_TYPE == "slidingmode"){
        nh.getParam("airo_control_node/slidingmode/pub_debug",PUB_DEBUG);
        if (PUB_DEBUG){
            slidingmode_debug_sub = nh.subscribe<std_msgs::Float64MultiArray>("/airo_control/slidingmode/debug",1,&AIRO_TRAJECTORY_SERVER::slidingmode_debug_cb,this);
            debug_msg.resize(11);
        }
    }
    else{
        ROS_ERROR("[AIRo Trajectory] Controller type not supported!");
    }
}

void AIRO_TRAJECTORY_SERVER::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
}

void AIRO_TRAJECTORY_SERVER::twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_twist.header = msg->header;
    local_twist.twist = msg->twist;
    current_twist_norm = sqrt(pow(msg->twist.linear.x,2) + pow(msg->twist.linear.y, 2) + pow(msg->twist.linear.z,2));
}

void AIRO_TRAJECTORY_SERVER::fsm_info_cb(const airo_message::FSMInfo::ConstPtr& msg){
    fsm_info.header = msg->header;
    fsm_info.is_landed = msg->is_landed;
    fsm_info.is_waiting_for_command = msg->is_waiting_for_command;
}

void AIRO_TRAJECTORY_SERVER::attitude_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg){
    attitude_target.header = msg->header;
    attitude_target.orientation = msg->orientation;
    attitude_target.thrust = msg->thrust;
}

void AIRO_TRAJECTORY_SERVER::battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg){
    battery_state.header = msg->header;
    battery_state.voltage = msg->voltage;
    battery_state.current = msg->current;
}

void AIRO_TRAJECTORY_SERVER::mpc_debug_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    if (CONTROLLER_TYPE == "mpc"){
        for (size_t i = 0; i < debug_msg.size();i++){
            debug_msg[i] = msg->data[i];
        }
    }
}

void AIRO_TRAJECTORY_SERVER::backstepping_debug_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    if (CONTROLLER_TYPE == "backstepping"){
        for (size_t i = 0; i < debug_msg.size();i++){
            debug_msg[i] = msg->data[i];
        }
    }
}

void AIRO_TRAJECTORY_SERVER::slidingmode_debug_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    if (CONTROLLER_TYPE == "slidingmode"){
        for (size_t i = 0; i < debug_msg.size();i++){
            debug_msg[i] = msg->data[i];
        }
    }
}

void AIRO_TRAJECTORY_SERVER::pose_cmd(const geometry_msgs::Point& point, const double& yaw_angle){
    if(fsm_info.is_waiting_for_command){
        geometry_msgs::Twist twist;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        pose_cmd(point,twist,yaw_angle);
    }
}

void AIRO_TRAJECTORY_SERVER::pose_cmd(const geometry_msgs::Pose& pose){
    double yaw_angle = 0.0;
    if (is_pose_initialized(pose)){
        yaw_angle = q2yaw(pose.orientation);
    }
    pose_cmd(pose.position,yaw_angle);
}
void AIRO_TRAJECTORY_SERVER::pose_cmd(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist){
    double yaw_angle = 0.0;
    if (is_pose_initialized(pose)){
        yaw_angle = q2yaw(pose.orientation);
    }
    pose_cmd(pose.position,twist,yaw_angle);
}
void AIRO_TRAJECTORY_SERVER::pose_cmd(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist, const geometry_msgs::Accel& accel){
        double yaw_angle = 0.0;
    if (is_pose_initialized(pose)){
        yaw_angle = q2yaw(pose.orientation);
    }
    pose_cmd(pose.position,twist,accel,yaw_angle);
}

void AIRO_TRAJECTORY_SERVER::pose_cmd(const geometry_msgs::Point& point, const geometry_msgs::Twist& twist, const double& yaw_angle){
    if(fsm_info.is_waiting_for_command){
        geometry_msgs::Accel accel;
        accel.linear.x = 0.0;
        accel.linear.y = 0.0;
        accel.linear.z = 0.0;
        pose_cmd(point,twist,accel,yaw_angle);
    }
}

void AIRO_TRAJECTORY_SERVER::pose_cmd(const geometry_msgs::Point& point, const geometry_msgs::Twist& twist, const geometry_msgs::Accel& accel, const double& yaw_angle){
    if(fsm_info.is_waiting_for_command){
        airo_message::Reference reference;
        reference.header.stamp = ros::Time::now();
        reference.ref_pose.position = point;
        reference.ref_pose.orientation = AIRO_TRAJECTORY_SERVER::yaw2q(yaw_angle);
        reference.ref_twist = twist;
        reference.ref_accel = accel;
        
        ROS_INFO_STREAM_THROTTLE(2.0, "[AIRo Trajectory] Publishing pose command.");
        command_pub.publish(reference);
    }
}

int AIRO_TRAJECTORY_SERVER::is_pose_initialized(const geometry_msgs::Pose& pose){
    if (pose.orientation.w == 0 && pose.orientation.x == 0 && pose.orientation.y == 0 &&pose.orientation.z == 0){
        return 0; 
    }
    else{
        return 1;
    }
}

bool AIRO_TRAJECTORY_SERVER::target_reached(const geometry_msgs::Pose& pose){
    bool position_reached = sqrt(pow(pose.position.x - local_pose.pose.position.x,2)+pow(pose.position.y - local_pose.pose.position.y,2)
    +pow(pose.position.z - local_pose.pose.position.z,2)) < 0.5;
    bool twist_reached = current_twist_norm < 0.5;
    bool yaw_reached = abs(q2yaw(pose.orientation) - q2yaw(local_pose.pose.orientation)) < 5 * M_PI / 180;
    return position_reached && twist_reached && yaw_reached;
}

bool AIRO_TRAJECTORY_SERVER::target_reached(const geometry_msgs::Point& point){
    bool position_reached = sqrt(pow(point.x - local_pose.pose.position.x,2)+pow(point.y - local_pose.pose.position.y,2)
    +pow(point.z - local_pose.pose.position.z,2)) < 0.5;
    bool twist_reached = current_twist_norm < 0.5;
    return position_reached && twist_reached;
}

bool AIRO_TRAJECTORY_SERVER::target_reached(const geometry_msgs::Point& point, const double& yaw_angle){
    bool position_reached = sqrt(pow(point.x - local_pose.pose.position.x,2)+pow(point.y - local_pose.pose.position.y,2)
    +pow(point.z - local_pose.pose.position.z,2)) < 0.5;
    bool twist_reached = current_twist_norm < 0.5;
    bool yaw_reached = abs(yaw_angle - q2yaw(local_pose.pose.orientation)) < 5 * M_PI / 180;
    return position_reached && twist_reached && yaw_reached;
}

geometry_msgs::Quaternion AIRO_TRAJECTORY_SERVER::yaw2q(double yaw){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
    return quaternion;
}

double AIRO_TRAJECTORY_SERVER::q2yaw(const geometry_msgs::Quaternion& quaternion){
    double phi,theta,psi;
    tf::Quaternion tf_quaternion;
    tf::quaternionMsgToTF(quaternion,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(phi, theta, psi);
    return psi;
}

Eigen::Vector3d AIRO_TRAJECTORY_SERVER::q2rpy(const geometry_msgs::Quaternion& quaternion){
    tf::Quaternion tf_quaternion;
    Eigen::Vector3d euler;
    tf::quaternionMsgToTF(quaternion,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(euler.x(), euler.y(), euler.z());
    return euler;
}

geometry_msgs::Quaternion AIRO_TRAJECTORY_SERVER::rpy2q(const Eigen::Vector3d& euler){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(euler.x(), euler.y(), euler.z());
    return quaternion;
}

void AIRO_TRAJECTORY_SERVER::file_traj_init(const std::string& file_name, std::vector<std::vector<double>>& traj){
    std::ifstream file(file_name);
    std::string line;
    int number_of_lines = 0;
    traj.clear();
    if (RESULT_SAVE){
        log_data.clear();
        log_counter = log_interval;
    }

    if(file.is_open()){
        while(getline(file, line)){
            number_of_lines++;
            std::istringstream linestream( line );
            std::vector<double> linedata;
            double number;
            while( linestream >> number ){
                linedata.push_back( number );
            }
            traj.push_back( linedata );
        }
        file.close();
    }
    else{
        ROS_ERROR("[AIRo Trajectory] Cannot open trajectory file!");
    }
}

geometry_msgs::Pose AIRO_TRAJECTORY_SERVER::get_start_pose(const std::vector<std::vector<double>>& traj){
    geometry_msgs::Pose start_point;
    double start_yaw;
    if (traj[0].size() != 3){
        start_yaw = traj[0][traj[0].size()-1];
    }
    else{
        start_yaw = 0.0;
    }
    start_point.position.x = traj[0][0];
    start_point.position.y = traj[0][1];
    start_point.position.z = traj[0][2];
    start_point.orientation = yaw2q(start_yaw);
    return start_point;
}

geometry_msgs::Pose AIRO_TRAJECTORY_SERVER::get_end_pose(const std::vector<std::vector<double>>& traj){
    geometry_msgs::Pose end_point;
    double end_yaw;
    if (traj[0].size() != 3){
        end_yaw = traj[traj.size()-1][traj[traj.size()-1].size()-1];
    }
    else{
        end_yaw = 0.0;
    }
    end_point.position.x = traj[traj.size()-1][0];
    end_point.position.y = traj[traj.size()-1][1];
    end_point.position.z = traj[traj.size()-1][2];
    end_point.orientation = yaw2q(end_yaw);
    return end_point;
}

bool AIRO_TRAJECTORY_SERVER::file_cmd(const std::vector<std::vector<double>>& traj, int& start_row){
    const int total_rows = traj.size();
    int path_ended = false;

    if (use_preview){
        std::vector<std::vector<double>> traj_matrix;
        if (start_row >= total_rows - row_interval) {
            // Construct all rows using the last row of traj
            std::vector<double> last_row = traj.back();
            traj_matrix.assign(preview_size, last_row);
            path_ended = true;
        }
        else{
            // Calculate the end row index
            int end_row = start_row + (preview_size - 1)*row_interval;
            end_row = std::min(end_row, total_rows - 1);  // Make sure end_row doesn't exceed the maximum row index

            // Copy the desired rows into the trajectory matrix
            for (int i = start_row; i <= end_row; i += row_interval){
                traj_matrix.push_back(traj[i]);
            }

            // If there are fewer than num_rows available, fill the remaining rows with the last row
            while (traj_matrix.size() < preview_size){
                traj_matrix.push_back(traj.back());
            }

            // Update the start_row for the next call
            start_row++;
        }

        airo_message::ReferencePreview reference_preview;
        reference_preview.header.stamp = ros::Time::now();
        reference_preview.ref_pose.resize(preview_size);
        reference_preview.ref_twist.resize(preview_size);
        reference_preview.ref_accel.resize(preview_size);
        int column = traj_matrix[0].size();

        if (!path_ended){
            if (column == 3){
                assign_position(traj_matrix,reference_preview);
            }
            else if (column == 4){
                assign_position(traj_matrix,reference_preview);
                assign_yaw(traj_matrix,reference_preview);
            }
            else if (column == 6){
                assign_position(traj_matrix,reference_preview);
                assign_twist(traj_matrix,reference_preview);
            }
            else if (column == 7){
                assign_position(traj_matrix,reference_preview);
                assign_twist(traj_matrix,reference_preview);
                assign_yaw(traj_matrix,reference_preview);
            }
            else if (column == 9){
                assign_position(traj_matrix,reference_preview);
                assign_twist(traj_matrix,reference_preview);
                assign_accel(traj_matrix,reference_preview);
            }
            else if (column == 10){
                assign_position(traj_matrix,reference_preview);
                assign_twist(traj_matrix,reference_preview);
                assign_accel(traj_matrix,reference_preview);
                assign_yaw(traj_matrix,reference_preview);
            }
            else{
                ROS_ERROR("[AIRo Trajectory] Trajectory file dimension wrong!");
                return false;
            }
        }
        else{
            assign_position(traj_matrix,reference_preview);
            if (column == 4 || column == 6 || column == 7 || column == 9 || column == 10){
                assign_yaw(traj_matrix,reference_preview);
            }
        }

        if (RESULT_SAVE){
            update_log(reference_preview);
        }
        command_preview_pub.publish(reference_preview);
    }
    else{
        std::vector<double> traj_row;
        if (start_row >= total_rows - 1) {
            traj_row = traj.back();
            path_ended = true;
        }
        else{
            traj_row = traj[start_row];
            // Update the start_row for the next call
            start_row++;
        }
        airo_message::Reference reference;
        reference.header.stamp = ros::Time::now();
        int column = traj_row.size();

        if (!path_ended){
            if (column == 3){
                assign_position(traj_row,reference);
            }
            else if (column == 4){
                assign_position(traj_row,reference);
                assign_yaw(traj_row,reference);
            }
            else if (column == 6){
                assign_position(traj_row,reference);
                assign_twist(traj_row,reference);
            }
            else if (column == 7){
                assign_position(traj_row,reference);
                assign_twist(traj_row,reference);
                assign_yaw(traj_row,reference);
            }
            else if (column == 9){
                assign_position(traj_row,reference);
                assign_twist(traj_row,reference);
                assign_accel(traj_row,reference);
            }
            else if (column == 10){
                assign_position(traj_row,reference);
                assign_twist(traj_row,reference);
                assign_accel(traj_row,reference);
                assign_yaw(traj_row,reference);
            }
            else{
                ROS_ERROR("[AIRo Trajectory] Trajectory file dimension wrong!");
                return false;
            }
        }
        else{
            assign_position(traj_row,reference);
            if (column == 4 || column == 6 || column == 7 || column == 9 || column == 10){
                assign_yaw(traj_row,reference);
            }
        }

        if (RESULT_SAVE){
            update_log(reference);
        }
        command_pub.publish(reference);
    }

    if (!path_ended){
        ROS_INFO_STREAM_THROTTLE(2.0,"[AIRo Trajectory] Publishing file trajectory.");
    }
    else{
        ROS_INFO_STREAM_THROTTLE(2.0,"[AIRo Trajectory] File trajectory ended!");
    }
    
    return path_ended;
}

void AIRO_TRAJECTORY_SERVER::assign_position(const std::vector<std::vector<double>>& traj_matrix, airo_message::ReferencePreview& reference_preview){
    for (int i = 0; i < preview_size; i++){
        reference_preview.ref_pose[i].position.x = traj_matrix[i][0];
        reference_preview.ref_pose[i].position.y = traj_matrix[i][1];
        reference_preview.ref_pose[i].position.z = traj_matrix[i][2];
        reference_preview.ref_pose[i].orientation = AIRO_TRAJECTORY_SERVER::yaw2q(0.0);
        reference_preview.ref_twist[i].linear.x = 0.0;
        reference_preview.ref_twist[i].linear.y = 0.0;
        reference_preview.ref_twist[i].linear.z = 0.0;
        reference_preview.ref_accel[i].linear.x = 0.0;
        reference_preview.ref_accel[i].linear.y = 0.0;
        reference_preview.ref_accel[i].linear.z = 0.0;
    }
}

void AIRO_TRAJECTORY_SERVER::assign_position(const std::vector<double>& traj_row, airo_message::Reference& reference){
    reference.ref_pose.position.x = traj_row[0];
    reference.ref_pose.position.y = traj_row[1];
    reference.ref_pose.position.z = traj_row[2];
    reference.ref_pose.orientation = AIRO_TRAJECTORY_SERVER::yaw2q(0.0);
    reference.ref_twist.linear.x = 0.0;
    reference.ref_twist.linear.y = 0.0;
    reference.ref_twist.linear.z = 0.0;
    reference.ref_accel.linear.x = 0.0;
    reference.ref_accel.linear.y = 0.0;
    reference.ref_accel.linear.z = 0.0;
}

void AIRO_TRAJECTORY_SERVER::assign_twist(const std::vector<std::vector<double>>& traj_matrix, airo_message::ReferencePreview& reference_preview){
    for (int i = 0; i < preview_size; i++){
        reference_preview.ref_twist[i].linear.x = traj_matrix[i][3];
        reference_preview.ref_twist[i].linear.y = traj_matrix[i][4];
        reference_preview.ref_twist[i].linear.z = traj_matrix[i][5];
    }
}

void AIRO_TRAJECTORY_SERVER::assign_twist(const std::vector<double>& traj_row, airo_message::Reference& reference){
    reference.ref_twist.linear.x = traj_row[3];
    reference.ref_twist.linear.y = traj_row[4];
    reference.ref_twist.linear.z = traj_row[5];
}

void AIRO_TRAJECTORY_SERVER::assign_accel(const std::vector<std::vector<double>>& traj_matrix, airo_message::ReferencePreview& reference_preview){
    for (int i = 0; i < preview_size; i++){
        reference_preview.ref_accel[i].linear.x = traj_matrix[i][6];
        reference_preview.ref_accel[i].linear.y = traj_matrix[i][7];
        reference_preview.ref_accel[i].linear.z = traj_matrix[i][8];
    }
}

void AIRO_TRAJECTORY_SERVER::assign_accel(const std::vector<double>& traj_row, airo_message::Reference& reference){
    reference.ref_accel.linear.x = traj_row[6];
    reference.ref_accel.linear.y = traj_row[7];
    reference.ref_accel.linear.z = traj_row[8];
}

void AIRO_TRAJECTORY_SERVER::assign_yaw(const std::vector<std::vector<double>>& traj_matrix, airo_message::ReferencePreview& reference_preview){
    for (int i = 0; i < preview_size; i++){
        reference_preview.ref_pose[i].orientation = AIRO_TRAJECTORY_SERVER::yaw2q(traj_matrix[i].back());
    }
}

void AIRO_TRAJECTORY_SERVER::assign_yaw(const std::vector<double>& traj_row, airo_message::Reference& reference){
    reference.ref_pose.orientation = AIRO_TRAJECTORY_SERVER::yaw2q(traj_row.back());
}

bool AIRO_TRAJECTORY_SERVER::takeoff(){
    if(fsm_info.is_landed == true){
        airo_message::TakeoffLandTrigger takeoff_trigger;
        ROS_INFO_THROTTLE(2.0, "[AIRo Trajectory] Sending takeoff trigger.");
        takeoff_trigger.takeoff_land_trigger = true; // Takeoff
        takeoff_trigger.header.stamp = ros::Time::now();
        takeoff_land_pub.publish(takeoff_trigger);
        return false;
    }
    else if(fsm_info.is_landed == false && fsm_info.is_waiting_for_command == true){
        ROS_INFO("[AIRo Trajectory] Vehicle already takeoff!");
        return true;
    }
    else{
        return false;
    }
}

bool AIRO_TRAJECTORY_SERVER::land(){
    if(fsm_info.is_landed == false && fsm_info.is_waiting_for_command == true){
        airo_message::TakeoffLandTrigger land_trigger;
        ROS_INFO_THROTTLE(2.0, "[AIRo Trajectory] Sending land trigger.");
        land_trigger.takeoff_land_trigger = false; // Land
        land_trigger.header.stamp = ros::Time::now();
        takeoff_land_pub.publish(land_trigger);
        return false;
    }
    else if(fsm_info.is_landed == true){
        ROS_INFO("[AIRo Trajectory] Vehicle has landed!");
        if (RESULT_SAVE){
            save_result();
        }
        return true;
    }
    else{
        return false;
    }
}

void AIRO_TRAJECTORY_SERVER::update_log(const airo_message::ReferencePreview& ref_preview){
    airo_message::Reference ref;
    ref.header = ref_preview.header;
    ref.ref_pose = ref_preview.ref_pose[0];
    ref.ref_twist = ref_preview.ref_twist[0];
    ref.ref_accel = ref_preview.ref_accel[0];
    update_log(ref);
}

void AIRO_TRAJECTORY_SERVER::update_log(const airo_message::Reference& ref){
    if (log_counter == log_interval){
        std::vector<double> line_to_push;
        Eigen::Vector3d local_euler,target_euler;
        local_euler = q2rpy(local_pose.pose.orientation);
        target_euler = q2rpy(attitude_target.orientation);

        line_to_push.push_back(static_cast<double>(log_data.size())*log_interval/FSM_FREQUENCY); // time
        line_to_push.push_back(ref.ref_pose.position.x); // x position ref
        line_to_push.push_back(local_pose.pose.position.x); // x position
        line_to_push.push_back(ref.ref_pose.position.y); // y position ref
        line_to_push.push_back(local_pose.pose.position.y); // y position
        line_to_push.push_back(ref.ref_pose.position.z); // z position ref
        line_to_push.push_back(local_pose.pose.position.z); // z position
        line_to_push.push_back(ref.ref_twist.linear.x); // x velocity ref
        line_to_push.push_back(local_twist.twist.linear.x); // x velocity
        line_to_push.push_back(ref.ref_twist.linear.y); // y velocity ref
        line_to_push.push_back(local_twist.twist.linear.y); // y velocity
        line_to_push.push_back(ref.ref_twist.linear.z); // z velocity ref
        line_to_push.push_back(local_twist.twist.linear.z); // z velocity
        line_to_push.push_back(target_euler.x()); // phi ref
        line_to_push.push_back(local_euler.x()); // phi
        line_to_push.push_back(target_euler.y()); // theta ref
        line_to_push.push_back(local_euler.y()); // theta
        line_to_push.push_back(target_euler.z()); // psi ref
        line_to_push.push_back(local_euler.z()); // psi
        line_to_push.push_back(attitude_target.thrust); // thrust
        line_to_push.push_back(battery_state.voltage); // voltage
        if (PUB_DEBUG){
            for (size_t i = 0; i < debug_msg.size();i++){
                line_to_push.push_back(debug_msg[i]);
            }
        }
        log_data.push_back(line_to_push);
        log_counter = 1;
    }
    else{
        log_counter++;
    }
}

int AIRO_TRAJECTORY_SERVER::save_result(){
    // Define column headers
    std::vector<std::string> column_headers = {"time","ref_x","x","ref_y","y","ref_z","z","ref_u","u","ref_v","v","ref_w","w","ref_phi","phi","ref_theta","theta","ref_psi","psi","thrust","voltage"};
    if (PUB_DEBUG){
        std::vector<std::string> debug_header;
        if (CONTROLLER_TYPE == "mpc"){
            debug_header = {"acados_status","kkt_res","cpu_time"};
            for (int i = 1; i <= preview_size; i++){
                std::string dummy_string = "yaw_predictin_" + std::to_string(i);
                debug_header.push_back(dummy_string);
            }
        }
        else if (CONTROLLER_TYPE == "backstepping"){
            debug_header = {"e_z1","e_z2","e_x1","e_x2","u_x","e_y1","e_y2","u_y"};
        }
        else if (CONTROLLER_TYPE == "slidingmode"){
            debug_header = {"e_z","e_dz","s_z","e_x","e_dx","s_x","u_x","e_y","e_dy","s_y","u_y"};
        }
        
        for (const std::string& element : debug_header){
            column_headers.push_back(element);
        }
    }

    // Get the current time
    std::time_t now = std::time(nullptr);
    std::tm tm = *std::localtime(&now);

    // Format the time as "yyyy-mm-dd-hh-mm"
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y_%m_%d_%H_%M");
    std::string time_stamp = oss.str();

    // Get the path to the "airo_trajectory" package
    std::string package_path = ros::package::getPath("airo_trajectory");

    if (!package_path.empty()) {
        // Construct the full file path
        std::string log_path = package_path + "/results/" + time_stamp + ".csv";

        // Open the file for writing
        std::ofstream out_file(log_path);

        // Check if the file was successfully opened
        if (out_file.is_open()) {
            // Write column headers as the first row
            for (size_t i = 0; i < column_headers.size(); ++i) {
                out_file << column_headers[i];
                if (i < column_headers.size() - 1) {
                    out_file << ",";
                }
            }
            out_file << "\n";

            // Write the data to the CSV file
            for (const std::vector<double>& row : log_data) {
                for (size_t i = 0; i < row.size(); ++i) {
                    out_file << row[i];
                    if (i < row.size() - 1) {
                        out_file << ",";
                    }
                }
                out_file << "\n";
            }

            // Close the file
            out_file.close();

            ROS_INFO_STREAM("[AIRo Trajectory] Data saved to " << log_path);
        } else {
            ROS_ERROR_STREAM("[AIRo Trajectory] Failed to open the file for writing: " << log_path);
        }
    }

    return 0;
}