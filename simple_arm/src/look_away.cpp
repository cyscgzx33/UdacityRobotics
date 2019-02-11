#include <ros/ros.h>
#include "simple_arm/GoToPosition.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>


/* 
   Define global variables 
   vector of joints last position, 
   moving state of the arm,
   and the client that can request services
*/

std::vector<double> joint_last_position{0, 0};
bool moving_state = false;
ros::ServiceClient srvClient;

/* This fucntion calls the safe_move service to safely move the arm to the center position */
void move_arm_center() {
  ROS_INFO_STREAM("Moving the arm to the center");

  // Request centering joint angles {1.57, 1.57}
  simple_arm::GoToPosition srv;
  srv.request.joint_1 = 1.57;
  srv.request.joint_2 = 1.57;

  // Call the safe_move service and pass the requested joint angles
  if (!srvClient.call(srv)) ROS_ERROR("Failed to call service safe_move!");
}

/* This callback function continuously executes and reads the arm joint angle positions */
void joint_states_callback(const sensor_msgs::JointState js)
{
  // Get joint current positions
  std::vector<double> joint_current_position = js.position;

  // Define a tolerance threshold of joint positions
  double tolerance = 0.0005;

  // Check if the arm is moving within the tolerance threshold
  if (fabs(joint_current_position[0] - joint_last_position[0]) < tolerance 
      && fabs(joint_current_position[1] - joint_last_position[1]) < tolerance)
  {
    moving_state = false;
  } else 
  {
    moving_state = true;
    joint_last_position = joint_current_position;
  }
}

/* This callback function continuously executes and reads image data */
void look_away_callback(const sensor_msgs::Image img)
{
  bool uniform_image = true;

  // Look through each pixel in the image and check if it's equal to the first one
  for (int i = 0; i < img.height * img.step; i++) {
    if (img.data[i] - img.data[0] != 0) {
      uniform_image = false;
      break;
    }
  }

  // If the image is uniform and the arm is not moving, move the arm to the center
  if (uniform_image && !moving_state) move_arm_center();
}

int main(int argc, char** argv)
{
  // Initialization
  ros::init(argc, argv, "look_away");
  ros::NodeHandle nh_;

  srvClient = nh_.serviceClient<simple_arm::GoToPosition>("/arm_mover/safe_move");

  // Subscribe to "/simple_arm/joint_states" topic to read the arm joints position through joint_states_callback function
  ros::Subscriber sub_arm_pos = nh_.subscribe("/simple_arm/joint_states", 10, joint_states_callback);

  // Subscribe to "/rgb_camera/image_raw" topic to read the image data through look_away_callback function
  ros::Subscriber sub_camera_img = nh_.subscribe("/rgb_camera/image_raw", 10, look_away_callback);

  // Handle ROS communicaion events
  ros::spin();

  return 0;
}

