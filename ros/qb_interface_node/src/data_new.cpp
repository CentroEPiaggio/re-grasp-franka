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

std_msgs::Float64MultiArray  F_ext, matrix_F, matrix_A, matrix_G;
std_msgs::Float64 hand_data, hand_data_2;
double F_ext_x, F_ext_y, F_ext_z;
float acc_data_0, acc_data_1, acc_data_2,acc_data_3,acc_data_4,acc_data_5,acc_data_6,acc_data_7,acc_data_8,acc_data_9,acc_data_10,acc_data_11,acc_data_12,acc_data_13,acc_data_14,acc_data_15,acc_data_16,acc_data_17,acc_data_18,acc_data_19,acc_data_20,acc_data_21,acc_data_22,acc_data_23,acc_data_24,acc_data_25,acc_data_26,acc_data_27,acc_data_28,acc_data_29,acc_data_30,acc_data_31,acc_data_32,acc_data_33,acc_data_34,acc_data_35,acc_data_36,acc_data_37,acc_data_38,acc_data_39,acc_data_40,acc_data_41,acc_data_42,acc_data_43,acc_data_44;
float gyro_data_0, gyro_data_1, gyro_data_2,gyro_data_3,gyro_data_4,gyro_data_5,gyro_data_6,gyro_data_7,gyro_data_8,gyro_data_9,gyro_data_10,gyro_data_11,gyro_data_12,gyro_data_13,gyro_data_14,gyro_data_15,gyro_data_16,gyro_data_17,gyro_data_18,gyro_data_19,gyro_data_20,gyro_data_21,gyro_data_22,gyro_data_23,gyro_data_24,gyro_data_25,gyro_data_26,gyro_data_27,gyro_data_28,gyro_data_29,gyro_data_30,gyro_data_31,gyro_data_32,gyro_data_33,gyro_data_34,gyro_data_35,gyro_data_36,gyro_data_37,gyro_data_38,gyro_data_39,gyro_data_40,gyro_data_41,gyro_data_42,gyro_data_43,gyro_data_44;
std::vector<float> F_ext_vett, acc_vett, gyro_vett, acc_vett_0_44;
std::string sep = "\n-----------------------------------\n";
Eigen::MatrixXf matrix_acc(13, 45), matrix_gyro(13, 45), matrix_F_ext(13, 3);

void gyroCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  gyro_data_0 = msg->data[0];
  gyro_data_1 = msg->data[1];
  gyro_data_2 = msg->data[2];
  gyro_data_3 = msg->data[3];
  gyro_data_4 = msg->data[4];
  gyro_data_5 = msg->data[5];
  gyro_data_6 = msg->data[6];
  gyro_data_7 = msg->data[7];
  gyro_data_8 = msg->data[8];
  gyro_data_9 = msg->data[9];
  gyro_data_10 = msg->data[10];
  gyro_data_11 = msg->data[11];
  gyro_data_12 = msg->data[12];
  gyro_data_13 = msg->data[13];
  gyro_data_14 = msg->data[14];
  gyro_data_15 = msg->data[15];
  gyro_data_16 = msg->data[16];
  gyro_data_17 = msg->data[17];
  gyro_data_18 = msg->data[18];
  gyro_data_19 = msg->data[19];
  gyro_data_20 = msg->data[20];
  gyro_data_21 = msg->data[21];
  gyro_data_22 = msg->data[22];
  gyro_data_23 = msg->data[23];
  gyro_data_24 = msg->data[24];
  gyro_data_25 = msg->data[25];
  gyro_data_26 = msg->data[26];
  gyro_data_27 = msg->data[27];
  gyro_data_28 = msg->data[28];
  gyro_data_29 = msg->data[29];
  gyro_data_30 = msg->data[30];
  gyro_data_31 = msg->data[31];
  gyro_data_32 = msg->data[32];
  gyro_data_33 = msg->data[33];
  gyro_data_34 = msg->data[34];
  gyro_data_35 = msg->data[35];
  gyro_data_36 = msg->data[36];
  gyro_data_37 = msg->data[37];
  gyro_data_38 = msg->data[38];
  gyro_data_39 = msg->data[39];
  gyro_data_40 = msg->data[40];
  gyro_data_41 = msg->data[41];
  gyro_data_42 = msg->data[42];
  gyro_data_43 = msg->data[43];
  gyro_data_44 = msg->data[44];
  
  ROS_INFO("I heard gyro");
}
void accCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  acc_data_0 = msg->data[0];
  acc_data_1 = msg->data[1];
  acc_data_2 = msg->data[2];
  acc_data_3 = msg->data[3];
  acc_data_4 = msg->data[4];
  acc_data_5 = msg->data[5];
  acc_data_6 = msg->data[6];
  acc_data_7 = msg->data[7];
  acc_data_8 = msg->data[8];
  acc_data_9 = msg->data[9];
  acc_data_10 = msg->data[10];
  acc_data_11 = msg->data[11];
  acc_data_12 = msg->data[12];
  acc_data_13 = msg->data[13];
  acc_data_14 = msg->data[14];
  acc_data_15 = msg->data[15];
  acc_data_16 = msg->data[16];
  acc_data_17 = msg->data[17];
  acc_data_18 = msg->data[18];
  acc_data_19 = msg->data[19];
  acc_data_20 = msg->data[20];
  acc_data_21 = msg->data[21];
  acc_data_22 = msg->data[22];
  acc_data_23 = msg->data[23];
  acc_data_24 = msg->data[24];
  acc_data_25 = msg->data[25];
  acc_data_26 = msg->data[26];
  acc_data_27 = msg->data[27];
  acc_data_28 = msg->data[28];
  acc_data_29 = msg->data[29];
  acc_data_30 = msg->data[30];
  acc_data_31 = msg->data[31];
  acc_data_32 = msg->data[32];
  acc_data_33 = msg->data[33];
  acc_data_34 = msg->data[34];
  acc_data_35 = msg->data[35];
  acc_data_36 = msg->data[36];
  acc_data_37 = msg->data[37];
  acc_data_38 = msg->data[38];
  acc_data_39 = msg->data[39];
  acc_data_40 = msg->data[40];
  acc_data_41 = msg->data[41];
  acc_data_42 = msg->data[42];
  acc_data_43 = msg->data[43];
  acc_data_44 = msg->data[44];

  ROS_INFO("I heard acc");
}


// void gyroCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
// {
// //0              //15
//   gyro_data_0 = msg->data[36];
//   gyro_data_1 = msg->data[37];
//   gyro_data_2 = msg->data[38];
// //1             //4
//   gyro_data_3 = msg->data[3];
//   gyro_data_4 = msg->data[4];
//   gyro_data_5 = msg->data[5];
// //2             //5
//   gyro_data_6 = msg->data[6];
//   gyro_data_7 = msg->data[7];
//   gyro_data_8 = msg->data[8];
// //3
//   gyro_data_9 = msg->data[0];
//   gyro_data_10 = msg->data[1];
//   gyro_data_11 = msg->data[2];
// //4
//   gyro_data_12 = msg->data[3];
//   gyro_data_13 = msg->data[4];
//   gyro_data_14 = msg->data[5];
// //5
//   gyro_data_15 = msg->data[6];
//   gyro_data_16 = msg->data[7];
//   gyro_data_17 = msg->data[8];
// //6
//   gyro_data_18 = msg->data[9];
//   gyro_data_19 = msg->data[10];
//   gyro_data_20 = msg->data[11];
// //7
//   gyro_data_21 = msg->data[12];
//   gyro_data_22 = msg->data[13];
//   gyro_data_23 = msg->data[14];
// //8
//   gyro_data_24 = msg->data[15];
//   gyro_data_25 = msg->data[16];
//   gyro_data_26 = msg->data[17];
// //9
//   gyro_data_27 = msg->data[18];
//   gyro_data_28 = msg->data[19];
//   gyro_data_29 = msg->data[20];
// //10
//   gyro_data_30 = msg->data[21];
//   gyro_data_31 = msg->data[22];
//   gyro_data_32 = msg->data[23];
// //11
//   gyro_data_33 = msg->data[24];
//   gyro_data_34 = msg->data[25];
//   gyro_data_35 = msg->data[26];
// //12
//   gyro_data_36 = msg->data[27];
//   gyro_data_37 = msg->data[28];
//   gyro_data_38 = msg->data[29];
// //13
//   gyro_data_39 = msg->data[30];
//   gyro_data_40 = msg->data[31];
//   gyro_data_41 = msg->data[32];
// //14
//   gyro_data_42 = msg->data[33];
//   gyro_data_43 = msg->data[34];
//   gyro_data_44 = msg->data[35];
  
//   ROS_INFO("I heard gyro");
// }
// void accCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
// {
//   //0              //15
//   acc_data_0 = msg->data[36];
//   acc_data_1 = msg->data[37];
//   acc_data_2 = msg->data[38];
// //1             //4
//   acc_data_3 = msg->data[3];
//   acc_data_4 = msg->data[4];
//   acc_data_5 = msg->data[5];
// //2             //5
//   acc_data_6 = msg->data[6];
//   acc_data_7 = msg->data[7];
//   acc_data_8 = msg->data[8];
// //3
//   acc_data_9 = msg->data[0];
//   acc_data_10 = msg->data[1];
//   acc_data_11 = msg->data[2];
// //4
//   acc_data_12 = msg->data[3];
//   acc_data_13 = msg->data[4];
//   acc_data_14 = msg->data[5];
// //5
//   acc_data_15 = msg->data[6];
//   acc_data_16 = msg->data[7];
//   acc_data_17 = msg->data[8];
// //6
//   acc_data_18 = msg->data[9];
//   acc_data_19 = msg->data[10];
//   acc_data_20 = msg->data[11];
// //7
//   acc_data_21 = msg->data[12];
//   acc_data_22 = msg->data[13];
//   acc_data_23 = msg->data[14];
// //8
//   acc_data_24 = msg->data[15];
//   acc_data_25 = msg->data[16];
//   acc_data_26 = msg->data[17];
// //9
//   acc_data_27 = msg->data[18];
//   acc_data_28 = msg->data[19];
//   acc_data_29 = msg->data[20];
// //10
//   acc_data_30 = msg->data[21];
//   acc_data_31 = msg->data[22];
//   acc_data_32 = msg->data[23];
// //11
//   acc_data_33 = msg->data[24];
//   acc_data_34 = msg->data[25];
//   acc_data_35 = msg->data[26];
// //12
//   acc_data_36 = msg->data[27];
//   acc_data_37 = msg->data[28];
//   acc_data_38 = msg->data[29];
// //13
//   acc_data_39 = msg->data[30];
//   acc_data_40 = msg->data[31];
//   acc_data_41 = msg->data[32];
// //14
//   acc_data_42 = msg->data[33];
//   acc_data_43 = msg->data[34];
//   acc_data_44 = msg->data[35];
//   ROS_INFO("I heard acc");
// }


void handCallback(const std_msgs::Float64::ConstPtr &msg)
{
  hand_data = *msg;
  ROS_INFO("I heard hand");
}

void hand_2Callback(const std_msgs::Float64::ConstPtr &msg)
{
  hand_data_2 = *msg;
  ROS_INFO("I heard hand_2");
}

void F_extCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  F_ext_x = msg->wrench.force.x;
  F_ext_y = msg->wrench.force.y;
  F_ext_z = msg->wrench.force.z;
  ROS_INFO("I heard F_ext");
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Subscriber gyro_sub = n.subscribe("qb_class_imu/gyro_vett", 1, gyroCallback);
  ros::Subscriber acc_sub = n.subscribe("qb_class_imu/acc_vett", 1, accCallback);
  ros::Subscriber hand_sub = n.subscribe("/qb_class/hand_measurement/grasp_failure", 1, handCallback);
  ros::Subscriber hand_2_sub = n.subscribe("/qb_class_2/hand_measurement/grasp_failure", 1, hand_2Callback);
  ros::Subscriber F_ext_sub = n.subscribe("/panda_arm_1/franka_state_controller_1/F_ext", 1, F_extCallback);

  ros::Publisher F_ext_pub = n.advertise<std_msgs::Float64MultiArray>("F_ext", 1);
  ros::Publisher hand_pub = n.advertise<std_msgs::Float64>("hand_opening", 1);
  ros::Publisher hand_pub_2 = n.advertise<std_msgs::Float64>("hand_2", 1);
  ros::Publisher gyro_pub = n.advertise<std_msgs::Float64MultiArray>("gyro", 1);
  ros::Publisher acc_pub = n.advertise<std_msgs::Float64MultiArray>("acc", 1);

  ros::Rate rate(70);

  int count = 0;
  while (ros::ok())
  {
  
    if (count == 10)
    {
      count = 0;
    }
         
        //F EXT
    float F_x = (float)F_ext_x;
    float F_y = (float)F_ext_y;
    float F_z = (float)F_ext_z;
    F_ext_vett = {F_x, F_y, F_z};

    for (int k = 0; k <= 11; k++)
    {
      for (int i = 0; i <= 2; i++)
      {
        matrix_F_ext(k, i) = matrix_F_ext(k + 1, i);
      }
    }

    for (int i = 0; i <= 2; i++)
    {
      matrix_F_ext(12, i) = F_ext_vett[i];
    }

    tf::matrixEigenToMsg(matrix_F_ext, matrix_F);

       // ACC GYRO

    acc_vett = {acc_data_0, acc_data_1, acc_data_2,acc_data_3,acc_data_4,acc_data_5,acc_data_6,acc_data_7,acc_data_8,acc_data_9,acc_data_10,acc_data_11,acc_data_12,acc_data_13,acc_data_14,acc_data_15,acc_data_16,acc_data_17,acc_data_18,acc_data_19,acc_data_20,acc_data_21,acc_data_22,acc_data_23,acc_data_24,acc_data_25,acc_data_26,acc_data_27,acc_data_28,acc_data_29,acc_data_30,acc_data_31,acc_data_32,acc_data_33,acc_data_34,acc_data_35,acc_data_36,acc_data_37,acc_data_38,acc_data_39,acc_data_40,acc_data_41,acc_data_42,acc_data_43,acc_data_44};
    gyro_vett = {gyro_data_0, gyro_data_1, gyro_data_2,gyro_data_3,gyro_data_4,gyro_data_5,gyro_data_6,gyro_data_7,gyro_data_8,gyro_data_9,gyro_data_10,gyro_data_11,gyro_data_12,gyro_data_13,gyro_data_14,gyro_data_15,gyro_data_16,gyro_data_17,gyro_data_18,gyro_data_19,gyro_data_20,gyro_data_21,gyro_data_22,gyro_data_23,gyro_data_24,gyro_data_25,gyro_data_26,gyro_data_27,gyro_data_28,gyro_data_29,gyro_data_30,gyro_data_31,gyro_data_32,gyro_data_33,gyro_data_34,gyro_data_35,gyro_data_36,gyro_data_37,gyro_data_38,gyro_data_39,gyro_data_40,gyro_data_41,gyro_data_42,gyro_data_43,gyro_data_44};

    for (int k = 0; k <= 11; k++)
    {
      for (int i = 0; i <= 44; i++)
      {
        matrix_acc(k, i) = matrix_acc(k + 1, i);
        matrix_gyro(k, i) = matrix_gyro(k + 1, i);
      }
    }
    //   QUI DA ERRORE !!!!!!!!!!!!!! --->
    for (int i = 0; i <= 44; i++)
    {
      matrix_acc(12, i) = acc_vett[i];
      matrix_gyro(12, i) = gyro_vett[i];
    }
    //   QUI DA ERRORE !!!!!!!!!!!!!! <----

    tf::matrixEigenToMsg(matrix_acc, matrix_A);
    tf::matrixEigenToMsg(matrix_gyro, matrix_G);

    //std::cout << "matrix \n" << matrix_A << sep;
    
    count = count + 1;

    //Publish
     F_ext_pub.publish(matrix_F);
     hand_pub.publish(hand_data);
     hand_pub_2.publish(hand_data_2);
     acc_pub.publish(matrix_A); 
     gyro_pub.publish(matrix_G); 

    ros::spinOnce();

    rate.sleep();
   
  }

  return 0;
}