/* Must set properly in CMakelist.txt that "include_directories" should contains "include" and "${catkin_INCLUDE_DIRS}" otherwise it does not find ros/ros.h */
#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "simple_arm_mover");
  
  ros::NodeHandle nh_;

  ros::Publisher joint_pub_1 = nh_.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);

  ros::Publisher joint_pub_2 = nh_.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

  // set a frequency to 10 hz
  // too high frequency will cost a lot for cpu
  // too low frequency will result in latency
  ros::Rate rt_(10);

  int start_time, elapsed_time;

  while(!start_time) {
    start_time = ros::Time::now().toSec();
  }

  while(ros::ok()){
    elapsed_time = ros::Time::now().toSec() - start_time;

    std_msgs::Float64 joint_1_angle, joint_2_angle;

    joint_1_angle.data = sin(2 * M_PI * 0.1 * elapsed_time) * (M_PI / 2);

    joint_2_angle.data = sin(2 * M_PI * 0.1 * elapsed_time) * (M_PI / 2);

    joint_pub_1.publish(joint_1_angle);

    joint_pub_2.publish(joint_2_angle);

  }
  
  // due to the call to rt_.sleep(), the loop is traversed at approximately 10 hz
  // when the node receives the signal to shut down, either from ROS Master or via
  // a signal from a console, the loop will exit
  rt_.sleep();

}
