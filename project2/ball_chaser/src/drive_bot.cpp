#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "ball_chaser/DriveToTarget.h"
// TODO: check if the DriveToTarget header is included correctly or not 

/* A global ros::Publisher to publish motor commands */
ros::Publisher motor_command_publisher;

// TODO:
/* Callback function */
// Objectives:
// 1) To publish requested linear_x and angular_z
// 2) To feedback the message after publishing the requested velocities
void drive_request_callback()
{
  

}

int main(int argc, char** argv)
{
  // Initialize a ROS node
  ros::init(argc, argv, "drive_bot");

  // Create a ROS NodeHandle object
  ros::NodeHandle nh;

  // Inform ROS master that we will be publishing a message,
  // which is in a type of geometry_msgs::Twist,
  // on the robot actuation topic, with a publish queue size 10
  motor_cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // TODO: Define a drive /ball_chaser/command_robot service with the callback function

  // TODO: Delete the loop, move the code to the inside of the callback function 
  //       and make changes to publish the requested velocities instead of constant values 
  while(ros::ok()) 
  {
    // here it publishs constant velocities 
  }

  // TODO: Handle ROS communication events
  // ros::spin();

  return 0;

}


