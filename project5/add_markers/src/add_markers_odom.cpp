#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "add_markers/addMarkers.h"

/* This callback function continuously executes and reads the goals data */
void odom_callback(const geometry_msgs::PoseWithCovarianceStamped amcl_pose_msg, geometry_msgs::Pose& odom_pose)
{
    ROS_INFO("Subscribing to the /amcl_pose topic ...");
    odom_pose = amcl_pose_msg.pose.pose;
}

/* This function update the status of the odom based on the distance between odom pos and the target pos */
void update_odom_status(bool& isPickup, bool& isDropoff, double cur_pos_x, double cur_orit_w)
{
    if (isPickup && isDropoff) return;

    if (!isPickup && abs(cur_pos_x - pick_up_pos_x) < pick_up_pos_x_thres && abs(cur_orit_w - pick_up_orit_w) < pick_up_orit_w_thres) 
    {
        isPickup = true;
        return;
    }

    if (isPickup && !isDropoff && abs(cur_pos_x - drop_off_pos_x) < drop_off_thres && abs(cur_orit_w - drop_off_orit_w) < drop_off_orit_w_thres)
    {
        isDropoff = true;
    }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers"); // modified from "basic_shape"
    ros::NodeHandle n;
    ros::Rate r(20); // try to be reasonable

    // Define a variable to store the goal pose during the callback process
    geometry_msgs::Pose odom_pose;

    // Subscribe the goals sent from the pick_objects node
    ros::Subscriber odom_sub = n.subscribe("/amcl_pose", 10, odom_callback); // /amcl_pose is more accurate than /odom

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Set the initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    // adding a flag to tell if the pickup zone is reached
    bool isPickup = false;

    // adding a flag to tell if the dropoff zone is reached
    bool isDropoff = false;

    // adding a flag to tell if pause has been conducted
    bool isPaused = false;

    while (ros::ok()) 
    {   
        // update the current status based on the odom info
        update_odom_status(isPickup, isDropoff, odom_pose.position.x, odom_pose.orientation.w);

        visualization_msgs::Marker marker;

        // Set the frame ID and time stamp
        marker.header.frame_id = "map"; // modified from "my_frame"
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for the marker
        // Note: any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "add_markers"; // modified from "basic_shape"
        marker.id = 0;

        // Set the marker type to shape
        // Note: if necessary, one can change the marker.type
        marker.type = shape;

        // Set the marker action: ADD, DELETE or DELETEALL
        marker.action = visualization_msgs::Marker::ADD;


        /*
            PHASE I: Publish the marker at the pickup zone
        */
        if (!isPickup) // the robot is on the way to the pick up zone
        {
            // Set the pose of the marker
            // Note: This is a full 6 DOF pose relative to the frame/time specified in the header
            marker.pose.position.x = isPickup? drop_off_pos_x : pick_up_pos_x; // based on the value in pick_objects
            marker.pose.position.y = 0.0; 
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = isPickup? drop_off_orit_w : pick_up_orit_w; // based on the value in pick_objects

            // Set the scale of the marker
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            // Set the color of the marker
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0;

            marker.lifetime = ros::Duration(); // ros::Duration() means never delete the marker

            // Publish the marker
            while (marker_pub.getNumSubscribers() < 1)
            {
                if (!ros::ok()) return 0;

                ROS_WARN_ONCE("Please create a subscriber to the marker!");
                sleep(1);
            }
            marker_pub.publish(marker);
            ROS_INFO("Published the marker at the pickup position !");
        }
        else // the robot has reached the pickup zone
        {
            if (!isPaused) // the robot haven't paused for 5 secs yet
            {
                ros::Duration(5.0).sleep();
                isPaused = true;
            } 
            else // the robot has paused for 5 secs
            {
                if (!isDropoff) // the robot is reaching to the drop off zone
                {
                    // Set the pose of the marker
                    // Note: This is a full 6 DOF pose relative to the frame/time specified in the header
                    marker.pose.position.x = isPickup? drop_off_pos_x : pick_up_pos_x; // based on the value in pick_objects
                    marker.pose.position.y = 0.0; 
                    marker.pose.position.z = 0.0;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = isPickup? drop_off_orit_w : pick_up_orit_w; // based on the value in pick_objects

                    // Set the scale of the marker
                    marker.scale.x = 1.0;
                    marker.scale.y = 1.0;
                    marker.scale.z = 1.0;

                    // Set the color of the marker
                    marker.color.r = 0.0f;
                    marker.color.g = 0.0f;
                    marker.color.b = 1.0f;

                    marker.lifetime = ros::Duration(); // ros::Duration() means never delete the marker

                    // Publish the marker
                    while (marker_pub.getNumSubscribers() < 1)
                    {
                        if (!ros::ok()) return 0;

                        ROS_WARN_ONCE("Please create a subscriber to the marker!");
                        sleep(1);
                    }
                    marker_pub.publish(marker);
                    ROS_INFO("Published the marker at the drop off position!");
                }
                else // the robot has reached the drop off zone
                {
                    ros::Duration(5.0).sleep();
                    break; // finished the whole process, break the while loop to exit the program
                }
            } 
        }
        
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}