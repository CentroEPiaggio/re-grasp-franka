/* PANDA SOFTHAND CLIENT - Contains all necessary objects and functions to call the services to 
control Panda + SoftHand
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic includes
#include <ros/service.h>

// ROS msg includes
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>

// Custom msg and srv includes
#include "panda_softhand_control/hand_plan.h"
#include "panda_softhand_control/hand_wait.h"
#include "panda_softhand_control/hand_control.h"

#include "panda_softhand_control/arm_wait.h"
#include "panda_softhand_control/arm_control.h"

#include "panda_softhand_control/joint_plan.h"
#include "panda_softhand_control/pose_plan.h"
#include "panda_softhand_control/slerp_plan.h"
#include <moveit/move_group_interface/move_group_interface.h>

// Defines
#define     DEBUG   1       // Prints out additional stuff
#define     VISUAL          // Publishes visual info on RViz

class PandaSoftHandClient_2

 {

    /// public variables and functions ------------------------------------------------------------
	public:
        PandaSoftHandClient_2
        
        ();

		PandaSoftHandClient_2
        
        (ros::NodeHandle& nh_);

        ~PandaSoftHandClient_2
        
        ();

        // Initializing function
        bool initialize(ros::NodeHandle& nh_);

        // Service call function for hand plan
        bool call_hand_plan_service_2(double goal_syn, double goal_duration, trajectory_msgs::JointTrajectory& computed_trajectory);

        // Service call function for hand control
        bool call_hand_control_service_2(trajectory_msgs::JointTrajectory& computed_trajectory);

        // Service call function for hand wait
        bool call_hand_wait_service_2(ros::Duration wait_time);

        // Service call function for arm control
        bool call_arm_control_service_2(trajectory_msgs::JointTrajectory& computed_trajectory);

        // Service call function for arm wait
        bool call_arm_wait_service_2(ros::Duration wait_time);

        // Service call function for joint plan
        bool call_joint_service_2(std::vector<double> joint_goal, std::vector<double> joint_start, trajectory_msgs::JointTrajectory& computed_trajectory);

        // Service call function for pose plan
        bool call_pose_service_2(geometry_msgs::Pose goal_pose, geometry_msgs::Pose start_pose, bool is_goal_relative, 
                                trajectory_msgs::JointTrajectory& computed_trajectory, trajectory_msgs::JointTrajectory past_trajectory);

        // Service call function for slerp plan
        bool call_slerp_service_2(geometry_msgs::Pose goal_pose, geometry_msgs::Pose start_pose, bool is_goal_relative, 
                                trajectory_msgs::JointTrajectory& computed_trajectory, trajectory_msgs::JointTrajectory past_trajectory);

	/// private variables -------------------------------------------------------------------------
	private:
		ros::NodeHandle nh;

        // Service names
        std::string hand_plan_service_2_name;
        std::string hand_control_service_2_name;
        std::string hand_wait_service_2_name;

        std::string arm_control_service_2_name;
        std::string arm_wait_service_2_name;

        std::string joint_service_2_name;
        std::string pose_service_2_name;
        std::string slerp_service_2_name;

        // Service clients
        ros::ServiceClient hand_plan_client;                // Client for hand plan service
        ros::ServiceClient hand_control_client;             // Client for hand control service
        ros::ServiceClient hand_wait_client;                // Client for hand wait service

        ros::ServiceClient arm_control_client;             // Client for arm control service
        ros::ServiceClient arm_wait_client;                // Client for arm wait service

        ros::ServiceClient joint_client;                    // Client for joint control service
        ros::ServiceClient pose_client;                     // Client for pose control service
        ros::ServiceClient slerp_client;                    // Client for slerp control service
	
};