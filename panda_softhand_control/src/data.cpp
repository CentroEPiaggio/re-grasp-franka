#include "ros/ros.h"
#include "qb_class_imu.h"
   
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/SVD>
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include <vector>   
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/WrenchStamped.h"
#include <iostream>
#include <eigen_conversions/eigen_msg.h>

#include <sstream>


std_msgs::Float64MultiArray acc_data, gyro_data, F_ext, matrix_F,matrix_A,matrix_G;
std_msgs::Float64 hand_data;
double F_ext_x, F_ext_y, F_ext_z;
std::vector<float> F_ext_vett;
std::string sep = "\n-----------------------------------\n";
Eigen::MatrixXf matrix_acc (8,51), matrix_gyro (8,51), matrix_F_ext(8,3);


    void gyroCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
   {
     gyro_data=*msg;
     ROS_INFO("I heard gyro");
   }
    void accCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
   {
     acc_data=*msg;
     ROS_INFO("I heard acc");
   }

    void handCallback(const std_msgs::Float64::ConstPtr &msg)
   {
     hand_data=*msg;
     ROS_INFO("I heard hand");
   }

    void F_extCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
   {
     F_ext_x =msg->wrench.force.x;
     F_ext_y =msg->wrench.force.y;
     F_ext_z =msg->wrench.force.z;
     ROS_INFO("I heard F_ext");
   }



   int main(int argc, char **argv)
   { 

     ros::init(argc, argv, "talker_old");

     ros::NodeHandle n;


     ros::Subscriber gyro_sub = n.subscribe("qb_class_imu/gyro_vett", 1, gyroCallback);
     ros::Subscriber acc_sub = n.subscribe("qb_class_imu/acc_vett", 1, accCallback);
     ros::Subscriber hand_sub = n.subscribe("/qb_class/hand_measurement/grasp_failure", 1, handCallback);
     ros::Subscriber F_ext_sub = n.subscribe("/panda_arm_1/franka_state_controller_1/F_ext", 1, F_extCallback);

   
     ros::Rate rate(1);

  
     /**
      * A count of how many messages we have sent. This is used to create
      * a unique string for each message.
      */
     int count = 0;
     while (ros::ok())
     {
       /**
       * This is a message object. You stuff it with data, and then publish it.
        */
       //matrix_F_ext.resize(8,3);
      
      
       ros::spinOnce();
  
       rate.sleep();
       ++count;
      }
  
  
      return 0;
    }