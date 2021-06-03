

// Basic Includes
#include "ros/ros.h"
#include "panda_softhand_control/ReGrasp.h"
#include <moveit/move_group_interface/move_group_interface.h>
geometry_msgs::Point position_arm_1;
std_msgs::Float32 network_state;
std::string sep = "\n-----------------------------------\n";

// Callback for arm 1 position subscriber
void get_position(const geometry_msgs::Point::ConstPtr &msg)
{

    // Saving the message
    position_arm_1.x = msg->x;
    position_arm_1.y = msg->y;
    position_arm_1.z = msg->z;
}

void get_network_state(const std_msgs::Float32::ConstPtr &msg)
{

    // Saving the message
    network_state = *msg;
}
/**********************************************
ROS NODE MAIN GRASP
**********************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_softhand_grasp_2");

    ros::NodeHandle nh_;

    ROS_INFO("Creating the ReGrasp object");
    ros::Subscriber position_sub = nh_.subscribe("/tf_world2EE", 1000, get_position);
    ros::topic::waitForMessage<geometry_msgs::Point>("/tf_world2EE", ros::Duration(2.0));

    // Initializing the network_state_sub subscriber and waiting
    ros::Subscriber network_sub = nh_.subscribe("/output_NN", 1000, get_network_state);
    ros::topic::waitForMessage<std_msgs::Float32>("/output_NN", ros::Duration(2.0));
    ros::Publisher handRef_pub = nh_.advertise<qb_interface::handRef>("/qb_class_2/hand_ref",1);

    ReGrasp re_grasp_obj(nh_);

    ROS_INFO("The main task sequence client is running. Running as fast as possible!");

    ros::Rate rate(10);
    while (ros::ok())
    {
        // Computing the grasp and pregrasp pose and converting to geometry_msgs Pose
        Eigen::Affine3d object_pose_aff;
        tf::poseMsgToEigen(re_grasp_obj.object_pose_T, object_pose_aff);
        Eigen::Affine3d grasp_transform_2_aff;
        tf::poseMsgToEigen(re_grasp_obj.grasp_T, grasp_transform_2_aff);
        Eigen::Affine3d pre_grasp_transform_2_aff;
        tf::poseMsgToEigen(re_grasp_obj.pre_grasp_T, pre_grasp_transform_2_aff);
        Eigen::Affine3d post_grasp_transform_2_aff;
        tf::poseMsgToEigen(re_grasp_obj.post_grasp_T, post_grasp_transform_2_aff);
        Eigen::Affine3d home_grasp_transform_2_aff;
        tf::poseMsgToEigen(re_grasp_obj.home_grasp_T, home_grasp_transform_2_aff);

        geometry_msgs::Pose pre_grasp_pose;
        geometry_msgs::Pose grasp_pose;
        geometry_msgs::Pose post_grasp_pose;
        geometry_msgs::Pose home_grasp_pose;

        tf::poseEigenToMsg(object_pose_aff * pre_grasp_transform_2_aff, pre_grasp_pose);
        tf::poseEigenToMsg(object_pose_aff * post_grasp_transform_2_aff, post_grasp_pose);
        tf::poseEigenToMsg(object_pose_aff * home_grasp_transform_2_aff, home_grasp_pose);
        tf::poseEigenToMsg(object_pose_aff * grasp_transform_2_aff, grasp_pose);

        std::cout << "Object position is \n"
                  << object_pose_aff.translation() << std::endl;


        // Setting zero pose as starting from present
        geometry_msgs::Pose present_pose = geometry_msgs::Pose();
        present_pose.position.x = 0.0;
        present_pose.position.y = 0.0;
        present_pose.position.z = 0.0;
        present_pose.orientation.x = 0.0;
        present_pose.orientation.y = 0.0;
        present_pose.orientation.z = 0.0;
        present_pose.orientation.w = 1.0;
        geometry_msgs::Pose current_pose;
        
        // 1) Going to pregrasp pose
        if (!re_grasp_obj.panda_softhand_client_2.call_pose_service_2(pre_grasp_pose, present_pose, false, re_grasp_obj.tmp_traj, re_grasp_obj.tmp_traj) || !re_grasp_obj.franka_ok)
        {
            ROS_ERROR("Could not plan to the specified pre grasp pose.");
            return false;
        }

        if (!re_grasp_obj.panda_softhand_client_2.call_arm_control_service_2(re_grasp_obj.tmp_traj) || !re_grasp_obj.franka_ok)
        {
            ROS_ERROR("Could not go to pre grasp pose.");
            return false;
        }

        re_grasp_obj.time_0 = ros::Time::now();
        qb_interface::handRef close_msg; 
        qb_interface::handRef open_msg; 
        std::vector<float> v1{-8000};
        std::vector<float> v2{0};
        close_msg.closure = v1;
        open_msg.closure = v2;

        ros::Duration waiting_failure = ros::Duration(60.0);;
        // std::cout << "OUTPUT NN BEFORE WHILE is \n" << network_state.data << std::endl;
        while (ros::Time::now() - re_grasp_obj.time_0 < waiting_failure)
        {
            get_network_state;
            get_position;
            if ((network_state.data == 1 || network_state.data == 2) && re_grasp_obj.regrasp_done == false)
            {

                if (network_state.data == 1)
                {
                    // PER PRESA VERTICALE
                    // re_grasp_obj.arm_1_position = {(position_arm_1.x - 0.10), (position_arm_1.y + 0.23), (position_arm_1.z-0.08), -0.102, 0.360, -1.070};
                    //PER PRESA LATO
                    re_grasp_obj.arm_1_position = {(position_arm_1.x- 0.10), (position_arm_1.y + 0.32), (position_arm_1.z-0.10), -0.102, 0.360, -1.070};
                    std::cout << "Fail because output network is \n"
                              << network_state.data << std::endl;
                }
                if (network_state.data == 2)
                {
                    // PER PRESA VERTICALE
                    // re_grasp_obj.arm_1_position = {(position_arm_1.x - 0.13), (position_arm_1.y + 0.16), (position_arm_1.z-0.09), 0.9, 0.55, -0.1525}; 
                    // PER PRESA LATO
                    re_grasp_obj.arm_1_position = {(position_arm_1.x - 0.16), (position_arm_1.y + 0.29), (position_arm_1.z-0.12), 0.9, 0.022, -0.10412};  
                   
                    std::cout << "Fail because output network is \n"
                              << network_state.data << std::endl;
                }

                re_grasp_obj.arm_1_pose = re_grasp_obj.convert_vector_to_pose(re_grasp_obj.arm_1_position);

                // //valutare se levarli
                Eigen::Affine3d arm_1_aff;
                tf::poseMsgToEigen(re_grasp_obj.arm_1_pose, arm_1_aff);
                geometry_msgs::Pose arm_1_goal;
                tf::poseEigenToMsg(arm_1_aff, arm_1_goal);
                //
                re_grasp_obj.time_start_regrasp = ros::Time::now();
                double timeout = 3.0;
                std::vector<double> pre_grasp_joint;
                std::vector<double> arm_1_goal_joint;
                // bool found_ik_pre_grasp = re_grasp_obj.performIK(pre_grasp_pose, timeout, pre_grasp_joint);
                // re_grasp_obj.time_pre_IK = ros::Time::now();
                // bool found_ik_goal = re_grasp_obj.performIK(arm_1_goal, timeout, arm_1_goal_joint);
                // re_grasp_obj.time_post_IK = ros::Time::now();

                // 2) Going to grasp pose
                if (!re_grasp_obj.panda_softhand_client_2.call_slerp_service_2(arm_1_goal, pre_grasp_pose, false, re_grasp_obj.tmp_traj, re_grasp_obj.tmp_traj) || !re_grasp_obj.franka_ok)
                {
                    ROS_ERROR("Could not plan to the specified grasp pose.");
                    return false;
                }
                re_grasp_obj.time_SLERP = ros::Time::now();

                if (!re_grasp_obj.panda_softhand_client_2.call_arm_wait_service_2(re_grasp_obj.waiting_time) || !re_grasp_obj.franka_ok)
                { // WAITING FOR END EXEC
                    ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre grasp from home joints");
                    return false;
                }

                if (!re_grasp_obj.panda_softhand_client_2.call_arm_control_service_2(re_grasp_obj.tmp_traj) || !re_grasp_obj.franka_ok)
                {
                    ROS_ERROR("Could not go to grasp pose.");
                    return false;
                }

                if (!re_grasp_obj.panda_softhand_client_2.call_arm_wait_service_2(re_grasp_obj.waiting_time) || !re_grasp_obj.franka_ok)
                { // WAITING FOR END EXEC
                    ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to grasp pose");
                    return false;
                }

                handRef_pub.publish(close_msg);
                // Regrasp Done, exit!
                re_grasp_obj.regrasp_done=true;
                std::cout << "Regrasp done \n" << std::endl;
                current_pose = arm_1_goal;
            }

            else if (network_state.data == 0 && re_grasp_obj.regrasp_done == false)
            {
                std::cout << "No fail because output network is \n"
                          << network_state.data << std::endl;
                current_pose = pre_grasp_pose;
            }
            ros::spinOnce();
        }

        // 5) Going to home grasp pose
        if (!re_grasp_obj.panda_softhand_client_2.call_slerp_service_2(home_grasp_pose, current_pose, false, re_grasp_obj.tmp_traj, re_grasp_obj.tmp_traj) || !re_grasp_obj.franka_ok)
        {
            ROS_ERROR("Could not plan to the specified grasp pose.");
            return false;
        }

        if (!re_grasp_obj.panda_softhand_client_2.call_arm_control_service_2(re_grasp_obj.tmp_traj) || !re_grasp_obj.franka_ok)
        {
            ROS_ERROR("Could not go to grasp pose.");
            return false;
        }

        if (!re_grasp_obj.panda_softhand_client_2.call_arm_wait_service_2(re_grasp_obj.waiting_time) || !re_grasp_obj.franka_ok)
        { // WAITING FOR END EXEC
            ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to grasp pose");
            return false;
        }
        handRef_pub.publish(open_msg);
        sleep(2.0);
        //     // re_grasp_obj.call_simple_grasp_task_2(network_state);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}