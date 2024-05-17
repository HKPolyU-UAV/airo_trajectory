#include <airo_trajectory/airo_trajectory_server.h>
 
enum State{
    TAKEOFF,
    INIT,
    FILE_TRAJ,
    LAND
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "file_trajectory_node");
    ros::NodeHandle nh;

    int FSM_FREQUENCY,row_to_read;
    std::string TRAJ_NAME,TRAJ_SCRIPT_PATH,TRAJ_FILE_PATH;
    nh.getParam("/file_trajectory_node/traj_name",TRAJ_NAME);
    nh.getParam("/airo_control_node/fsm/fsm_frequency",FSM_FREQUENCY);
    TRAJ_SCRIPT_PATH = ros::package::getPath("airo_trajectory") + "/scripts/" + TRAJ_NAME + ".py";
    TRAJ_FILE_PATH = ros::package::getPath("airo_trajectory") + "/scripts/" + TRAJ_NAME + ".txt";
    ros::Rate rate(FSM_FREQUENCY);
    int result = std::system(TRAJ_SCRIPT_PATH.c_str());
    ros::Duration(2.0).sleep();
    AIRO_TRAJECTORY_SERVER airo_trajectory_server(nh);
    std::vector<std::vector<double>> trajectory_data; // ref_x x ref_y y ref_z z ref_u u ref_v v ref_w w ref_phi phi ref_theta theta ref_psi psi thrust
    airo_trajectory_server.file_traj_init(TRAJ_FILE_PATH,trajectory_data);
    State state = TAKEOFF;

    while(ros::ok()){
        ros::spinOnce();

        switch(state){
            case TAKEOFF:{
                if(airo_trajectory_server.takeoff()){
                    state = INIT;
                }
                break;
            }

            case INIT:{
                airo_trajectory_server.pose_cmd(airo_trajectory_server.get_start_pose(trajectory_data));
                if(airo_trajectory_server.target_reached(airo_trajectory_server.get_start_pose(trajectory_data))){
                    state = FILE_TRAJ;
                    row_to_read = 0;
                }
                break;
            }

            case FILE_TRAJ:{
                if(airo_trajectory_server.file_cmd(trajectory_data, row_to_read) && // Reached end of file
                airo_trajectory_server.target_reached(airo_trajectory_server.get_end_pose(trajectory_data))){
                    state = LAND;
                }
                break;
            }

            case LAND:{
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