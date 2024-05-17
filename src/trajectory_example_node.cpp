#include <airo_trajectory/airo_trajectory_server.h>

int i = 0;
 
enum State{
    TAKEOFF,
    POSE_YAW,
    POSE_TWIST,
    LAND
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_example_node");
    ros::NodeHandle nh;
    int FSM_FREQUENCY;
    nh.getParam("/airo_control_node/fsm/fsm_frequency",FSM_FREQUENCY);
    ros::Rate rate(FSM_FREQUENCY);
    AIRO_TRAJECTORY_SERVER airo_trajectory_server(nh);

    State state = TAKEOFF;
    std::vector<geometry_msgs::Point> target_points;
    geometry_msgs::Twist target_twist;
    geometry_msgs::Accel target_accel;
    std::vector<double> target_yaw;

    target_points.resize(2);
    target_yaw.resize(2);

    target_points[0].x = 0.0;
    target_points[0].y = 0.0;
    target_points[0].z = 1.0;
    target_yaw[0] = M_PI/4;

    target_points[1].x = 1.5;
    target_points[1].y = 1.5;
    target_points[1].z = 1.5;
    target_yaw[1] = -M_PI/4;

    target_twist.linear.x = 0.5;
    target_twist.linear.y = 0.5;
    target_twist.linear.z = 0.5;

    while(ros::ok()){
        ros::spinOnce();

        switch(state){
            case TAKEOFF:{
                if(airo_trajectory_server.takeoff()){
                    state = POSE_YAW;
                }
                break;
            }

            case POSE_YAW:{
                airo_trajectory_server.pose_cmd(target_points[i],target_yaw[i]);
                if(airo_trajectory_server.target_reached(target_points[i],target_yaw[i])){
                    i += 1;
                    if(i == target_yaw.size()){
                        state = POSE_TWIST;
                    }
                }
                break;
            }

            case POSE_TWIST:{
                airo_trajectory_server.pose_cmd(target_points[0],target_twist);
                if(airo_trajectory_server.target_reached(target_points[0],0.0)){
                    state = LAND;
                }
                break;
            }

            case LAND:{
                airo_trajectory_server.land();
                if(airo_trajectory_server.land()){
                    return 0;
                }
                break;
            }
        }

        ros::Duration(rate).sleep();
    }

    return 0;
}