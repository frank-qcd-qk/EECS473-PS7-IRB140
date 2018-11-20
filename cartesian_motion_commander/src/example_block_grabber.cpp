// example_block_grabber: 
// wsn, Nov, 2018; 
// illustrates use of a generic action client that communicates with
// an action server called "cartMoveActionServer"
// the actual action server can be customized for a specific robot, whereas
// this client is robot agnostic

//launch with roslaunch irb140_description irb140.launch, which places a block at x=0.5, y=0
// launch action server: rosrun irb140_planner irb140_cart_move_as
// then run this node

//for manual gripper control,  rosrun cwru_sticky_fingers finger_control_dummy_node /sticky_finger/link6 false

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <arm_motion_action/arm_interfaceAction.h>
#include <cartesian_motion_commander/cart_motion_commander.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
#include <std_srvs/SetBool.h>
using namespace std;

//! Following are global setting for target drop off
double xDropoff = 0.0;
double yDropoff = 0.0;
geometry_msgs::PoseStamped block_position;
bool getter_flag;

void getUpdatedPosition(const geometry_msgs::PoseStamped& UpdatedPosition) {
        block_position = UpdatedPosition;
        getter_flag = true;
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "example_arm_cart_move_ac"); // name this node 
    ros::NodeHandle nh; //standard ros node handle     
    CartMotionCommander cart_motion_commander;
    XformUtils xformUtils;
    ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("/sticky_finger/link6");
    ros::Subscriber position_updater = nh.subscribe("block_pose", 1, getUpdatedPosition);
    std_srvs::SetBool srv;
    srv.request.data = true;

    Eigen::VectorXd joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    int njnts;
    int nsteps;
    double arrival_time;
    geometry_msgs::PoseStamped tool_pose, tool_pose_home;

    bool traj_is_valid = false;
    int rtn_code;

    nsteps = 10;
    arrival_time = 2.0;



    Eigen::Vector3d b_des, n_des, t_des, O_des;
    Eigen::Matrix3d R_gripper;
    b_des << 0, 0, -1;
    n_des << -1, 0, 0;
    t_des = b_des.cross(n_des);

    R_gripper.col(0) = n_des;
    R_gripper.col(1) = t_des;
    R_gripper.col(2) = b_des;

    //- Translation: [0.450, -0.000, 0.367]

    O_des << 0.5, 0.3, 0.3;
    Eigen::Affine3d tool_affine;
    tool_affine.linear() = R_gripper;
    tool_affine.translation() = O_des;
    //   geometry_msgs::PoseStamped transformEigenAffine3dToPoseStamped(Eigen::Affine3d e,std::string reference_frame_id);   

    tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "system_ref_frame");
    ROS_INFO("requesting plan to gripper-down pose:");
    xformUtils.printPose(tool_pose);
    rtn_val = cart_motion_commander.plan_jspace_traj_current_to_tool_pose(nsteps, arrival_time, tool_pose);
    if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
        ROS_INFO("successful plan; command execution of trajectory");
        rtn_val = cart_motion_commander.execute_planned_traj();
        ros::Duration(arrival_time + 0.2).sleep();
    } else {
        ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
    }
    while(ros::ok){
        //TODO:  Input Output handle:
        xDropoff = 0.00;
        std::cout << std::endl;
        std::cout << "Please input the X coordinate you want to set.......(Use CTRL+C to Quit)" << std::endl << "Input: ";
        std::cin >> xDropoff;
        while(std::cin.fail()) {
            ROS_ERROR("Input is not valid! Please try again....(Use CTRL+C to Quit)");
            std::cout<< "Input: ";
            std::cin.clear();
            std::cin.ignore(256,'\n');
            std::cin >> xDropoff;
        }
        yDropoff = 0.00;
        std::cout << std::endl;
        std::cout << "Please input the Y coordinate you want to set.......(Use CTRL+C to Quit)" << std::endl << "Input: ";
        std::cin >> yDropoff;
        while(std::cin.fail()) {
            ROS_ERROR("Input is not valid! Please try again....(Use CTRL+C to Quit)");
            std::cout<< "Input: ";
            std::cin.clear();
            std::cin.ignore(256,'\n');
            std::cin >> yDropoff;
        }
        //TODO:  Move out the camera field of view
        ROS_INFO("moving out of camera view");
        tool_pose.pose.position.y=-0.3;         
        tool_pose.pose.position.y=-0.5; 
        tool_pose.pose.position.z = 0.3; //0.01;         
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

        while (!getter_flag) {
                ros::spinOnce();
        }

        getter_flag = false;
        
        //* Construct Matrixes for Transforms:
        Eigen::Matrix4d TF_gripper2object; //Transform matrix of gripper to object
        Eigen::Matrix4d TF_object2robot; //Transform matrix of block to robot
        Eigen::Matrix4d F_TF_gripper2robot; //Final Result Matrix for describing gripper to robot transform

        //* Populating Gripper to Object matrix:
        TF_gripper2object.row(0) <<  1,  0,  0, 0;
        TF_gripper2object.row(1) <<  0, -1,  0, 0;
        TF_gripper2object.row(2) <<  0,  0, -1, 0;
        TF_gripper2object.row(3) <<  0,  0,  0, 1;

        //* Obtain rotation matrix component for object to robot:
        double x = block_position.pose.orientation.x;
        double y = block_position.pose.orientation.y;
        double z = block_position.pose.orientation.z;
        double w = block_position.pose.orientation.w;
        ROS_INFO("[Grabber_Math]Obtained Orientation X: %f, Y: %f, Z: %f, W: %f",x,y,z,w);


        //* Populating Object to Robot matrix
        TF_object2robot.row(0) <<  1-2*y*y-2*z*z    , 2*x*y-2*z*w   , 2*x*z+2*y*w   , block_position.pose.position.x;
        TF_object2robot.row(1) <<  2*x*y+2*z*w      , 1-2*x*x-2*z*z , 2*y*z-2*x*w   , block_position.pose.position.y;
        TF_object2robot.row(2) <<  2*x*z-2*y*w      , 2*y*z+2*x*w   , 1-2*x*x-2*y*y ,              0             ;
        TF_object2robot.row(3) <<       0           ,       0       ,       0       ,              1             ;

        //* Math for calculating the gripper matrix
        F_TF_gripper2robot = TF_object2robot*TF_gripper2object;

        //* Feed back into tool_affine
        tool_affine.linear().row(0) <<  F_TF_gripper2robot(0,0),  F_TF_gripper2robot(0,1),  F_TF_gripper2robot(0,2);
        tool_affine.linear().row(1) <<  F_TF_gripper2robot(1,0),  F_TF_gripper2robot(1,1),  F_TF_gripper2robot(1,2);
        tool_affine.linear().row(2) <<  F_TF_gripper2robot(2,0),  F_TF_gripper2robot(2,1),  F_TF_gripper2robot(2,2);
        tool_affine.translation() << F_TF_gripper2robot(0,3), F_TF_gripper2robot(1,3), F_TF_gripper2robot(2,3); //x,y,z in transformation matrix

        tool_pose = xformUtils.transformEigenAffine3dToPoseStamped(tool_affine, "system_ref_frame");



        //TODO: Move towards the object detected:
        ROS_INFO("[Critical Location:] moving to hover above block");
        //! Get pose here:
        tool_pose.pose.position.x = block_position.pose.position.x;
        tool_pose.pose.position.y = block_position.pose.position.y;
        tool_pose.pose.position.z = 0.5; //* Hover above control          
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

        //TODO: Enable the Vacuum gripper
        ROS_INFO("[Critical Location:] enabling vacuum gripper:");
        //enable the vacuum gripper:
        srv.request.data = true;
        while (!client.call(srv) && ros::ok()) {
            ROS_INFO("Sending command to gripper...");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }

        //TODO: Move down to vacum:
        ROS_INFO("[Critical Location:] moving to pick up item");
        //lower tool to approach part to grasp
        tool_pose.pose.position.z = 0.0343; //block is 0.035 high      
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

        //TODO: Move upward so robot can move with the block:
        ROS_INFO("[Critical Location:] moving to hover above block");
        //! Get pose here:
        tool_pose.pose.position.z = 0.5; //* Hover above control          
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }


        //TODO: Going towards final dropoff site:
        ROS_INFO("[Critical Location:] Moving towards drop off arial:");
        tool_pose.pose.position.x = xDropoff;
        tool_pose.pose.position.y = yDropoff;
        tool_pose.pose.position.z = 0.5; //* Hover above control          
        ROS_INFO("requesting plan to depart with grasped object:");       
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

        //TODO: Move down to drop off:
        ROS_INFO("[Critical Location:] moving to drop off low ");
        //lower tool to approach part to grasp
        tool_pose.pose.position.z = 0.1; //block is 0.035 high      
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }

        //TODO: disable the vacuum gripper:
        ROS_INFO("[Critical Location:] Disabling Gripper:");
        srv.request.data = false;
        while (!client.call(srv) && ros::ok()) {
            ROS_INFO("Sending command to gripper...");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }
        
        //TODO:  Move out the camera field of view
        ROS_INFO("[Critical Location:] moving out of camera view");
        tool_pose.pose.position.y=-0.3;         
        tool_pose.pose.position.y=-0.5; 
        tool_pose.pose.position.z = 0.3; //0.01;          
        ROS_INFO("requesting plan to descend:");
        xformUtils.printPose(tool_pose);
        rtn_val = cart_motion_commander.plan_cartesian_traj_qprev_to_des_tool_pose(nsteps, arrival_time, tool_pose);
        if (rtn_val == arm_motion_action::arm_interfaceResult::SUCCESS) {
            ROS_INFO("successful plan; command execution of trajectory");
            rtn_val = cart_motion_commander.execute_planned_traj();
            ros::Duration(arrival_time + 0.2).sleep();
        } else {
            ROS_WARN("unsuccessful plan; rtn_code = %d", rtn_val);
        }
    }
    return 0;
}