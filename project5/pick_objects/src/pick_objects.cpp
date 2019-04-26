#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client to send goal requests to the "move_base" server,
// through the "SimpleActionClient"
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_objects");

    MoveBaseClient ac("move_base", true);

    // Wait for 5 secs for move_base action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Initialize the goal object
    move_base_msgs::MoveBaseGoal goal;

    // Set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = -2.0;
    goal.target_pose.pose.position.y = 6.0;
    goal.target_pose.pose.orientation.w = 1.5;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goals for the robot to reach");
    ac.sendGoal(goal);

    // Wait in infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("The base moved 1 meter forward");
    } else {
        ROS_INFO("ERROR: The base failed to move forward 1 meter");
    }

    // Wait for 5 secs after reaching the pick up zone

    // Trial method one: not working
    // -------------------------------------------------------------------------
    // while(!ac.waitForServer(ros::Duration(5.0))){
    //     ROS_INFO("Waiting for 5 secs after reaching the pick up zone");
    // }
    // -------------------------------------------------------------------------
    
    // Trial method two
    ros::Duration(5.0).sleep();
    ROS_INFO("Waited for 5 seconds, and end of the whole process.");


    // Add second goal for the robot to reach
    goal.target_pose.pose.position.x = 4.5;
    goal.target_pose.pose.position.y = -0.5;
    goal.target_pose.pose.orientation.w = 1.5;
   
    ROS_INFO("Sending the second goal for the robot to reach");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("The base moved 3 meter forward");
    } else {
        ROS_INFO("ERROR: The base failed to move forward 3 meter");
    }

    

    return 0;

}