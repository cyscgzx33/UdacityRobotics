#include <ros/ros.h>
#include "add_markers/addMarkers.h"


addMarkers::addMarkers(ros::NodeHandle& nh) : nh_(nh)
{
    // Set the frame ID and time stamp
    marker_.header.frame_id = "map"; // modified from "my_frame"
    marker_.header.stamp = ros::Time::now();

    // Set the namespace and id for the marker
    // Note: any marker sent with the same namespace and id will overwrite the old one
    marker_.ns = "add_markers"; // modified from "basic_shape"
    marker_.id = 0;

    // Set the marker type to shape
    // Note: if necessary, one can change the marker.type
    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker_.type = shape;

    // Set the marker action: ADD, DELETE or DELETEALL
    marker_.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker
    // Note: This is a full 6 DOF pose relative to the frame/time specified in the header
    marker_.pose.position.x = 0.0; // based on the value in pick_objects
    marker_.pose.position.y = 0.0; 
    marker_.pose.position.z = 0.0;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 0.0; // based on the value in pick_objects

    // Set the scale of the marker
    marker_.scale.x = 1.0;
    marker_.scale.y = 1.0;
    marker_.scale.z = 1.0;

    // Set the color of the marker
    marker_.color.r = 0.0f;
    marker_.color.g = 0.0f;
    marker_.color.b = 1.0f;
    marker_.color.a = 1.0;

    marker_.lifetime = ros::Duration(); // ros::Duration() means never delete the marker

    while(ros::ok())
    {   
        if (isDropoff_) break;
        check_odom_status();
    }

}

addMarkers::~addMarkers() {};


void addMarkers::odom_callback(const geometry_msgs::PoseWithCovarianceStamped& amcl_pose_msg)
{
    ROS_INFO("Subscribing to the /amcl_pose topic ...");
    odom_pose_ = amcl_pose_msg.pose.pose;
}

void addMarkers::update_odom_status(double cur_pos_x, double cur_pos_y, double cur_orit_w)
{
    if (isPickup_ && isDropoff_) return;

    if (!isPickup_ && abs(cur_pos_x - pick_up_pos_x) < pick_up_pos_thres && abs(cur_pos_y - pick_up_pos_y) < pick_up_pos_thres && abs(cur_orit_w - pick_up_orit_w) < pick_up_orit_w_thres) 
    {
        isPickup_ = true;
        return;
    }

    if (isPickup_ && !isDropoff_ && abs(cur_pos_y - drop_off_pos_y) < drop_off_pos_thres && abs(cur_pos_x - drop_off_pos_x) < drop_off_pos_thres && abs(cur_orit_w - drop_off_orit_w) < drop_off_orit_w_thres)
    {
        isDropoff_ = true;
    }
}

void addMarkers::check_odom_status()
{
    update_odom_status(odom_pose_.position.x, odom_pose_.position.y, odom_pose_.orientation.w);

    if (!isPickup_)
    {
        marker_.pose.position.x = pick_up_pos_x; // based on the value in pick_objects
        marker_.pose.position.y = pick_up_pos_y; // based on the value in pick_objects
        marker_.pose.orientation.w =pick_up_orit_w; // based on the value in pick_objects
        
        check_publish_markers_status();
        marker_pub_.publish(marker_);
        ROS_INFO("Published the marker at the pickup position.");
    }
    else // the robot has reached the pickup zone
    {
        if (!isPaused_) // the robot haven't paused for 5 secs yet
        {
            marker_.color.a = 0.0; // hide the marker
            check_publish_markers_status();
            marker_pub_.publish(marker_);
            ROS_INFO("Hided the marker at the pickup position to simulate pickup.");
            ros::Duration(5.0).sleep();
            isPaused_ = true;
        } 
        else // the robot has paused for 5 secs
        {
            if (!isDropoff_) // the robot is reaching to the drop off zone
            {   
                marker_.color.a = 1.0; // unhide the marker
                marker_.pose.position.x = drop_off_pos_x; // based on the value in pick_objects
                marker_.pose.position.y = drop_off_pos_y; // based on the value in pick_objects
                marker_.pose.orientation.w = drop_off_orit_w; // based on the value in pick_objects
                
                check_publish_markers_status();
                marker_pub_.publish(marker_);
                ROS_INFO("Published the marker at the dropoff position.");
            }
            else // the robot has reached the drop off zone
            {
                ros::Duration(5.0).sleep();
            }
        } 
    }

    ros::spinOnce();
}


void addMarkers::check_publish_markers_status()
{
    // Publish the marker
    while (marker_pub_.getNumSubscribers() < 1)
    {
        if (!ros::ok()) return;

        ROS_WARN_ONCE("Please create a subscriber to the marker!");
        sleep(1);
    }
}