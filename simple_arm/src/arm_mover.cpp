#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "simple_arm/GoToPosition.h"

/* Global joint publishers */
ros::Publisher joint_1_pub;
ros::Publisher joint_2_pub;

/* This function checks and clamps the joint angles to a safe zone */
std::vector<float> clamp_at_boundaries(float requested_joint_1, float requested_joint_2)
{
  // Define clamped joint angles and assign them to the requested ones
  float clamped_joint_1 = requested_joint_1;
  float clamped_joint_2 = requested_joint_2;

  // Get min and max joint parameters, and assign them to their respective variables
  float min_joint_1, max_joint_1, min_joint_2, max_joint_2;

  // Assign a node and get node name
  ros::NodeHandle nh_;
  std::string node_name = ros::this_node::getName();

  // Get joints' min and max parameters
  nh_.getParam(node_name + "/min_joint_1_angle", min_joint_1);
  nh_.getParam(node_name + "/max_joint_1_angle", max_joint_1);
  nh_.getParam(node_name + "/min_joint_2_angle", min_joint_2);
  nh_.getParam(node_name + "/max_joint_2_angle", max_joint_2);

  // Check if joint_1 stays in the safe zone, otherwise clamp it
  if (requested_joint_1 < min_joint_1 || requested_joint_1 > max_joint_1)
  {
    clamped_joint_1 = std::min(std::max(requested_joint_1, min_joint_1), max_joint_1);
    ROS_WARN("Joint 1 is out of bounds, valid range is: [%1.2f, %1.2f], clamped to: %1.2f", min_joint_1, max_joint_1, clamped_joint_1);
  }

  // Check if joint_2 stays in the safe zone, otherwise clamp it
  if (requested_joint_2 < min_joint_2 || requested_joint_2 > max_joint_2)
  {
    clamped_joint_2 = std::min(std::max(requested_joint_2, min_joint_2), max_joint_2);
    ROS_WARN("Joint 2 is out of bounds, valid range is [%1.2f, %1.2f], clamped to: %1.2f", min_joint_2, max_joint_2, clamped_joint_2);
  }

  // Create and return clamped joint angles
  std::vector<float> clamped_data = {clamped_joint_1, clamped_joint_2};

  return clamped_data;
}


/* This callback function executes whenever a safe_move service is requested */
bool handle_safe_move_request(simple_arm::GoToPosition::Request& req,
                              simple_arm::GoToPosition::Response& res)
{

  ROS_INFO("GoToPositionRequest received - j1: %1.2f, j2: %1.2f", (float)req.joint_1, (float)req.joint_2);
  
  // Check if requested joint angles
  std::vector<float> joint_angles = clamp_at_boundaries(req.joint_1, req.joint_2);

  // Publish clamped joint angles to the arm
  std_msgs::Float64 joint_1_angle;
  std_msgs::Float64 joint_2_angle;

  joint_1_angle.data = joint_angles[0];
  joint_2_angle.data = joint_angles[1];

  joint_1_pub.publish(joint_1_angle);
  joint_2_pub.publish(joint_2_angle);

  // Wait 3 seconds for arm to settle
  ros::Duration(3).sleep();

  // Return a response message
  res.message_feedback = "Joint angles set - j1: " + std::to_string(joint_angles[0]) + ", j2: " + std::to_string(joint_angles[1]);
  ROS_INFO_STREAM(res.message_feedback);

  return true;
}

int main(int argc, char** argv)
{
  // Initialize the arm_mover node and create a NodeHandle to it
  ros::init(argc, argv, "arm_mover");
  ros::NodeHandle n;

  // Define the two global publishers
  joint_1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
  joint_2_pub = n.advertise<std_msgs::Float64>("simple_arm/joint_2_position_controller/command", 10);

  // Define a safe_move service with a callback function
  ros::ServiceServer service = n.advertiseService("/arm_mover/safe_move", handle_safe_move_request);
  ROS_INFO("Ready to send joint commands");

  // Handle ROS communication events
  ros::spin();

  return 0;
}
