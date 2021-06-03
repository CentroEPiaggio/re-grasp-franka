#include "ros/ros.h"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>


// ROS Service and Message Includes
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <vector>
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "panda_softhand_control/set_object.h"
#include "panda_softhand_control/complex_grasp.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include <controller_manager_msgs/SwitchController.h>
#include <franka_msgs/FrankaState.h>
#include <franka_msgs/ErrorRecoveryActionGoal.h>

// Parsing includes
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Custom Includes
#include "panda_softhand_control/PandaSoftHandClient_2.h"

#include "qb_class_2.h"

// Defines
#define     DEBUG   1       // Prints out additional stuff
#define     VISUAL          // Publishes visual info on RViz

extern geometry_msgs::Point position_arm_1;
extern std_msgs::Float32 network_state;
class ReGrasp {
    /// public variables and functions ------------------------------------------------------------
    public:
        ReGrasp(ros::NodeHandle& nh_);

        ~ReGrasp();

        // Parameters parsing
        bool parse_task_params();

        geometry_msgs::Pose convert_vector_to_pose(std::vector<double> input_vec);

        void get_object_pose(const geometry_msgs::Pose::ConstPtr &msg);

      //   void get_position(const geometry_msgs::Point::ConstPtr &msg);
        
      //   void get_network_state(const std_msgs::Float32::ConstPtr &msg);

        // Callback for franka state subscriber
        void get_franka_state(const franka_msgs::FrankaState::ConstPtr &msg);

       // Callback for simple grasp task service
       bool call_simple_grasp_task_2(std_msgs::Float32 &network_state);

       bool performIK(geometry_msgs::Pose pose_in, double timeout, std::vector<double>& joints_out);
       bool franka_ok = true;


       // NEW

       // Subscriber to object pose and the pose
       
        geometry_msgs::Pose object_pose_T;


        // Subscriber to franka_states for getting tau_ext on joints and other info and Publisher of its norm
   
        franka_msgs::FrankaState latest_franka_state;
        
   
        
        // Open and Close pub

        // ros::Publisher handRef_pub;
        std::vector<double> arm_1_position;
        std::vector<double> arm_1_rpy;
        std::vector<double> arm_1;
        
        // The Panda SoftHand Client
        PandaSoftHandClient_2 panda_softhand_client_2;
   
   // Service names
        std::string grasp_task_service_name_2;
        std::string franka_state_topic_name_2 = "/franka_state_controller_2/franka_states";
        //ros::ServiceServer grasp_task_server;
       // Parsed task sequence variables
       std::string robot_name_2;                     // Name of the robot (namespace)
       std::string robot_joints_name_2;              // Name of the robot joints (without the number of the joints)
       std::string pos_controller_2;                 // Name of position controller
       std::string imp_controller_2;                 // Name of impedance controller
       std::string object_topic_name_2;              // Name of the topic where the object pose is published
       std::vector<double> home_joints_2;
       std::vector<double> grasp_transform_2;
       geometry_msgs::Pose grasp_T;
       std::vector<double> pre_grasp_transform_2;
       geometry_msgs::Pose pre_grasp_T;
       std::vector<double> post_grasp_transform_2;
       geometry_msgs::Pose post_grasp_T;
       std::vector<double> home_grasp_transform_2;
       geometry_msgs::Pose home_grasp_T;
       geometry_msgs::Pose arm_1_pose;
       // MoveIt stuff and functions for FK and IK
       std::string group_name;
       std::string end_effector_name;
       std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_ptr;
       robot_model::RobotModelPtr kinematic_model;
       robot_state::RobotStatePtr kinematic_state;
    
       //Generic
       trajectory_msgs::JointTrajectory tmp_traj;
       ros::Duration waiting_time;
       ros::Duration waiting_time2;
       ros::Duration waiting_failure;

       ros::Time time_0;
       ros::Time time_1;
       ros::Time time_SLERP;
       ros::Time time_pre_IK;
       ros::Time time_post_IK;
       ros::Time time_start_regrasp;
       ros::Time time_end_regrasp;
       bool regrasp_done = false;
      
       std::vector<double> null_joints;                            // null joints in order to make joint plan from present joints
       ros::ServiceServer grasp_task_server_2;

    /// private variables -------------------------------------------------------------------------
    private:
       ros::NodeHandle nh;

};