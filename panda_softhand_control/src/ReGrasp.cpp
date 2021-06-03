#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "utils/parsing_utilities.h"
#include "panda_softhand_control/ReGrasp.h"
#include "ros/service_client.h"
#include "std_srvs/SetBool.h"


ReGrasp::ReGrasp(ros::NodeHandle& nh_){
    
    // Initializing Node Handle
    this->nh = nh_;

    // Parsing the task params
    if(!this->parse_task_params()){
        ROS_ERROR("The parsing of task parameters went wrong. Be careful, using default values...");
    }
  
    // Initializing the object subscriber and waiting (the object topic name is parsed now)
    ros::Subscriber object_sub = this->nh.subscribe(this->object_topic_name_2, 1, &ReGrasp::get_object_pose, this);
    ros::topic::waitForMessage<geometry_msgs::Pose>(this->object_topic_name_2, ros::Duration(2.0));

    // Initializing the franka_state_sub subscriber and waiting
    ros::Subscriber franka_state_sub = this->nh.subscribe("/" + this->robot_name_2 + this->franka_state_topic_name_2, 1, &ReGrasp::get_franka_state, this);
    ros::topic::waitForMessage<franka_msgs::FrankaState>("/" + this->robot_name_2 + this->franka_state_topic_name_2, ros::Duration(2.0));
     
    // this->handRef_pub = this->nh.advertise<qb_interface::handRef>("/qb_class_2/hand_ref",1);
    
    // Initializing Panda SoftHand Client (TODO: Return error if initialize returns false)
    this->panda_softhand_client_2.initialize(this->nh);

    // Moveit names
    this->group_name = "panda_arm_2";
    this->end_effector_name = "left_hand_ee_link";

    // Initializing other moveit stuff (robot model, kinematic model and state)
    this->robot_model_loader_ptr.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    this->kinematic_model = this->robot_model_loader_ptr->getModel();
    ROS_INFO("Model frame: %s", this->kinematic_model->getModelFrame().c_str());
    this->kinematic_state.reset(new robot_state::RobotState(this->kinematic_model));

    // // Setting the task service names
    // std::cout << "Starting to advertise!!!" << std::endl;
    // this->grasp_task_service_name_2 = "/grasp_task_service_2";
    // this->grasp_task_server_2 = this->nh.advertiseService(this->grasp_task_service_name_2, &ReGrasp::call_simple_grasp_task_2, this);
    // std::cout << "Finished to advertise!!!" << std::endl;

    // Initializing other control values
    this->waiting_time = ros::Duration(50.0);
    this->waiting_time2 = ros::Duration(50.0);
    this->waiting_failure = ros::Duration(40.0);
    this->null_joints.resize(7);
    std::fill(this->null_joints.begin(), this->null_joints.end(), 0.0);
    //Open and Close msg
    
    
    //this->v1[1]=0;

    // Spinning once
    ros::spinOnce();
}

ReGrasp::~ReGrasp(){
    
    // Nothing to do here yet
}

// Parameters parsing
bool ReGrasp::parse_task_params(){
    bool success = true;

    if(!ros::param::get("/grasp_params_2/robot_name_2", this->robot_name_2)){
		ROS_WARN("The param 'robot_name_2' not found in param server! Using default.");
		this->robot_name_2 = "panda_arm_2";
		success = false;
	}

    if(!ros::param::get("/grasp_params_2/robot_joints_name_2", this->robot_joints_name_2)){
		ROS_WARN("The param 'robot_joints_name_2' not found in param server! Using default.");
		this->robot_joints_name_2 = "panda_arm_2_joint1";
		success = false;
	}

    if(!ros::param::get("/grasp_params_2/pos_controller_2", this->pos_controller_2)){
		ROS_WARN("The param 'pos_controller_2' not found in param server! Using default.");
		this->pos_controller_2 = "position_joint_trajectory_controller_2";
		success = false;
	}

    if(!ros::param::get("/grasp_params_2/imp_controller_2", this->imp_controller_2)){
		ROS_WARN("The param 'imp_controller_2' not found in param server! Using default.");
		this->imp_controller_2 = "cartesian_impedance_controller_softbots_stiff_matrix";
		success = false;
	}

    if(!ros::param::get("/grasp_params_2/object_topic_name_2", this->object_topic_name_2)){
		ROS_WARN("The param 'object_topic_name_2' not found in param server! Using default.");
		this->object_topic_name_2 = "/irim_demo/chosen_object_2";
		success = false;
	}

	if(!ros::param::get("/grasp_params_2/home_joints_2", this->home_joints_2)){
		ROS_WARN("The param 'home_joints_2' not found in param server! Using default.");
		this->home_joints_2 = {-0.035, -0.109, -0.048, -1.888, 0.075, 1.797, -0.110};
		success = false;
	}

    if(!ros::param::get("/grasp_params_2/grasp_transform_2", this->grasp_transform_2)){
		ROS_WARN("The param 'grasp_transform_2' not found in param server! Using default.");
		this->grasp_transform_2.resize(6);
        std::fill(this->grasp_transform_2.begin(), this->grasp_transform_2.end(), 0.0);
		success = false;
	}

    // Converting the grasp_transform_2 vector to geometry_msgs Pose
    this->grasp_T = this->convert_vector_to_pose(this->grasp_transform_2);

    if(!ros::param::get("/grasp_params_2/pre_grasp_transform_2", this->pre_grasp_transform_2)){
		ROS_WARN("The param 'pre_grasp_transform_2' not found in param server! Using default.");
		this->pre_grasp_transform_2.resize(6);
        std::fill(this->pre_grasp_transform_2.begin(), this->pre_grasp_transform_2.end(), 0.0);
		success = false;
	}

    // Converting the pre_grasp_transform_2 vector to geometry_msgs Pose
    this->pre_grasp_T = this->convert_vector_to_pose(this->pre_grasp_transform_2);


    if(!ros::param::get("/grasp_params_2/post_grasp_transform_2", this->post_grasp_transform_2)){
		ROS_WARN("The param 'post_grasp_transform_2' not found in param server! Using default.");
		this->post_grasp_transform_2.resize(6);
        std::fill(this->post_grasp_transform_2.begin(), this->post_grasp_transform_2.end(), 0.0);
		success = false;
	}

    // Converting the post_grasp_transform_2 vector to geometry_msgs Pose
    this->post_grasp_T = this->convert_vector_to_pose(this->post_grasp_transform_2);

    if(!ros::param::get("/grasp_params_2/home_grasp_transform_2", this->home_grasp_transform_2)){
		ROS_WARN("The param 'home_grasp_transform_2' not found in param server! Using default.");
		this->home_grasp_transform_2.resize(6);
        std::fill(this->home_grasp_transform_2.begin(), this->home_grasp_transform_2.end(), 0.0);
		success = false;
	}

    // Converting the home_grasp_transform_2 vector to geometry_msgs Pose
    this->home_grasp_T = this->convert_vector_to_pose(this->home_grasp_transform_2);
    std::cout << "OBJECT VECTOR is \n" << this->home_grasp_transform_2[0] << std::endl;
    std::cout << "OBJECT POSE is \n" << this->home_grasp_T.position.x << std::endl;

    return success;
}


// Convert xyzrpy vector to geometry_msgs Pose
geometry_msgs::Pose ReGrasp::convert_vector_to_pose(std::vector<double> input_vec){
    
    // Creating temporary variables
    geometry_msgs::Pose output_pose;
    Eigen::Affine3d output_affine;

    // Getting translation and rotation
    Eigen::Vector3d translation(input_vec[0], input_vec[1], input_vec[2]);
    output_affine.translation() = translation;
    Eigen::Matrix3d rotation = Eigen::Matrix3d(Eigen::AngleAxisd(input_vec[5], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(input_vec[4], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(input_vec[3], Eigen::Vector3d::UnitX()));
    output_affine.linear() = rotation;    
    
    // Converting to geometry_msgs and returning
    tf::poseEigenToMsg(output_affine, output_pose);
    return output_pose;
}


// Callback for object pose subscriber
void ReGrasp::get_object_pose(const geometry_msgs::Pose::ConstPtr &msg){

    // Saving the message
    this->object_pose_T = *msg;
}

// Callback for franka state subscriber
void ReGrasp::get_franka_state(const franka_msgs::FrankaState::ConstPtr &msg){

    // Saving the message
    this->latest_franka_state = *msg;

    // Checking for libfranka errors
    if(msg->robot_mode != 2 && msg->robot_mode != 5){       // The robot state is not "automatic" or "manual guiding"
        this->franka_ok = false;
        if(DEBUG && false) ROS_ERROR("Something happened to the robot!");
    }else if(msg->robot_mode == 2){
        this->franka_ok = true;
        if(DEBUG && false) ROS_WARN("Now Franka is in a good mood!");
    }

    
}


// Callback for simple grasp task service
// bool ReGrasp::call_simple_grasp_task_2(std_msgs::Float32 &network_state){
       
       //geometry_msgs::Pose current_pose;
//     // Computing the grasp and pregrasp pose and converting to geometry_msgs Pose
//     Eigen::Affine3d object_pose_aff; tf::poseMsgToEigen(this->object_pose_T, object_pose_aff);
//     Eigen::Affine3d grasp_transform_2_aff; tf::poseMsgToEigen(this->grasp_T, grasp_transform_2_aff);
//     Eigen::Affine3d pre_grasp_transform_2_aff; tf::poseMsgToEigen(this->pre_grasp_T, pre_grasp_transform_2_aff);
//     Eigen::Affine3d post_grasp_transform_2_aff; tf::poseMsgToEigen(this->post_grasp_T, post_grasp_transform_2_aff);
//     Eigen::Affine3d home_grasp_transform_2_aff; tf::poseMsgToEigen(this->home_grasp_T, home_grasp_transform_2_aff);
    

//     geometry_msgs::Pose pre_grasp_pose; geometry_msgs::Pose grasp_pose; geometry_msgs::Pose post_grasp_pose; geometry_msgs::Pose home_grasp_pose; 
    
//     tf::poseEigenToMsg(object_pose_aff * pre_grasp_transform_2_aff, pre_grasp_pose);
//     tf::poseEigenToMsg(object_pose_aff * post_grasp_transform_2_aff, post_grasp_pose);
//     tf::poseEigenToMsg(object_pose_aff * home_grasp_transform_2_aff, home_grasp_pose);
//     tf::poseEigenToMsg(object_pose_aff * grasp_transform_2_aff, grasp_pose);

    
//     // Couting object pose for debugging
    
     
//     std::cout << "Object position is \n" << object_pose_aff.translation() << std::endl;
    
    
//     qb_interface::handRef close_msg; 
//     qb_interface::handRef open_msg; 
//     close_msg.closure = this->v1;
//     open_msg.closure = this->v2;
     
//     // Setting zero pose as starting from present
//     geometry_msgs::Pose present_pose = geometry_msgs::Pose();
//     present_pose.position.x = 0.0; present_pose.position.y = 0.0; present_pose.position.z = 0.0;
//     present_pose.orientation.x = 0.0; present_pose.orientation.y = 0.0; present_pose.orientation.z = 0.0; present_pose.orientation.w = 1.0;
    
    
//     // 1) Going to pregrasp pose
//     if(!this->panda_softhand_client_2.call_pose_service_2(pre_grasp_pose, present_pose, false, this->tmp_traj, this->tmp_traj) || !this->franka_ok){
//         ROS_ERROR("Could not plan to the specified pre grasp pose.");
//         return false;
//     }
    

//     if(!this->panda_softhand_client_2.call_arm_control_service_2(this->tmp_traj) || !this->franka_ok){
//         ROS_ERROR("Could not go to pre grasp pose.");
//         return false;
//     }
    
//     this->time_0=ros::Time::now();
    
//     // std::cout << "OUTPUT NN BEFORE WHILE is \n" << this->network_state.data << std::endl;
//     while (ros::Time::now() - this->time_0 < this->waiting_failure){
//           std::cout << "output network is \n" << network_state.data << std::endl;
//         // // std::cout << "OUTPUT NN INTO WHILE is \n" << this->network_state.data << std::endl;
//         // if ((this->network_state.data == 1 || this->network_state.data == 2 ) && this->regrasp_done==false){
            
//         //     if(this->network_state.data == 1){
//         //        this->arm_1_position={(position_arm_1.x-0.1), (position_arm_1.y+0.22), (position_arm_1.z-0.15), -0.102, 0.360, -1.270};
//         //     }
//         //     if (this->network_state.data == 2){
//         //        this->arm_1_position={(position_arm_1.x-0.1), (position_arm_1.y+0.24), (position_arm_1.z-0.03), 1.56, 0.360, -0.162};
//         //     }
      
//         //      this->arm_1_pose = this->convert_vector_to_pose(this->arm_1_position);         
            
//         //     // //valutare se levarli
//         //      Eigen::Affine3d arm_1_aff; 
//         //      tf::poseMsgToEigen(this->arm_1_pose, arm_1_aff);
//         //      geometry_msgs::Pose arm_1_goal;
//         //      tf::poseEigenToMsg(arm_1_aff, arm_1_goal);
//         //     //
//         //     this->time_start_regrasp=ros::Time::now();
//         //     // 2) Going to grasp pose
//         //     if(!this->panda_softhand_client_2.call_slerp_service_2(arm_1_goal, pre_grasp_pose, false, this->tmp_traj, this->tmp_traj) || !this->franka_ok){
//         //     ROS_ERROR("Could not plan to the specified grasp pose.");
//         //     res.success = false;
//         //     res.message = "The service call_simple_grasp_task_2 was NOT performed correctly!";
//         //     return false;
//         //     }
//         //     this->time_SLERP=ros::Time::now();

//         //     if(!this->panda_softhand_client_2.call_arm_wait_service_2(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
//         //     ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre grasp from home joints");
//         //     res.success = false;
//         //     res.message = "The service call_simple_grasp_task_2 was NOT performed correctly! Error wait in arm control.";
//         //     return false;
//         //     }

//         //     if(!this->panda_softhand_client_2.call_arm_control_service_2(this->tmp_traj) || !this->franka_ok){
//         //     ROS_ERROR("Could not go to grasp pose.");
//         //     res.success = false;
//         //     res.message = "The service call_simple_grasp_task_2 was NOT performed correctly! Error in arm control.";
//         //     return false;
//         //     }

//         //     if(!this->panda_softhand_client_2.call_arm_wait_service_2(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
//         //     ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to grasp pose");
//         //     res.success = false;
//         //     res.message = "The service call_simple_grasp_task_2 was NOT performed correctly! Error wait in arm control.";
//         //     return false;
//         //     }
//         //     this->time_end_regrasp=ros::Time::now();
//         //     std::cout << "TIME start regrasp \n" << this->time_start_regrasp<< std::endl;
//         //     std::cout << "TIME for slerp planning \n" << this->time_SLERP<< std::endl;
//         //     std::cout << "TIME end regrasp \n" << this->time_end_regrasp<< std::endl;
//         //     // Regrasp Done, exit!
//         //     this->regrasp_done=true;
//         //     std::cout << "Regrasp done \n" << std::endl;
//         //     current_pose = arm_1_goal;
//         // }
        
//         // else if (this->network_state.data == 0 && this->regrasp_done==false) {
//         //     std::cout << "No fail because output network is \n" << this->network_state.data << std::endl;
//               current_pose = pre_grasp_pose;
//         // }

//     } 
    
    // // 5) Going to home grasp pose
    // if(!this->panda_softhand_client_2.call_slerp_service_2(home_grasp_pose, current_pose, false, this->tmp_traj, this->tmp_traj) || !this->franka_ok){
    //           ROS_ERROR("Could not plan to the specified grasp pose.");
    //            return false;
    // }

    // if(!this->panda_softhand_client_2.call_arm_control_service_2(this->tmp_traj) || !this->franka_ok){
    //        ROS_ERROR("Could not go to grasp pose.");
    //            return false;
    // }
   
    // if(!this->panda_softhand_client_2.call_arm_wait_service_2(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
    //         ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to grasp pose");
    //          return false;
    // }

//     return true;
// }



bool ReGrasp::performIK(geometry_msgs::Pose pose_in, double timeout, std::vector<double>& joints_out){
    Eigen::Isometry3d end_effector_state;
    tf::poseMsgToEigen(pose_in, end_effector_state);
    const robot_state::JointModelGroup* joint_model_group = this->kinematic_model->getJointModelGroup(this->group_name);
    bool found_ik = this->kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

    if (!found_ik){
        ROS_ERROR("Could not find IK solution in ReGrasp...");
        return false;
    }

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    kinematic_state->copyJointGroupPositions(joint_model_group, joints_out);
    this->kinematic_state->copyJointGroupPositions(joint_model_group, joints_out);

    if (DEBUG){
        ROS_INFO("Found an IK solution in ReGrasp: ");
        for (std::size_t i = 0; i < joint_names.size(); ++i){
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joints_out[i]);
        }
    }

    return true;
}