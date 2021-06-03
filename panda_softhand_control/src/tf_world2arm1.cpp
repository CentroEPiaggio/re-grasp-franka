

// Basic Includes
#include "ros/ros.h"
#include "panda_softhand_control/GraspFailure.h"
#include <tf/transform_listener.h>

/**********************************************
ROS NODE MAIN GRASP
**********************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_softhand_tf");

    ros::NodeHandle nh_;

    ros::Rate rate(50);
    //Publish right hand 
    ros::Publisher position_pub = nh_.advertise<geometry_msgs::Point>("/tf_world2EE",1);
    //Listener for tf from world to right hand
    tf::TransformListener listener;

    while(nh_.ok()){
        tf::StampedTransform transform;
        try{
            listener.waitForTransform("/world","/right_hand_ee_link", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/world", "/right_hand_ee_link", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        //Creating right hand 
        geometry_msgs::Point right_hand;
        right_hand.x = transform.getOrigin().x();
        right_hand.y = transform.getOrigin().y();
        right_hand.z = transform.getOrigin().z();
        position_pub.publish(right_hand);
    
    }
    
    while(ros::ok()){
        // Nothing to do here
        }

      rate.sleep();
    return 0;
}