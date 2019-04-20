#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers"); // modified from "basic_shape"
    ros::NodeHandle n;
    ros::Rate r(1);

    // TODO: make sure what should topic be subscribed
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // Set the initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    // while (ros::ok()) // why do I need ros::ok() at all ??? I'll try delete it
    // {
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

    // Set the pose of the marker
    // Note: This is a full 6 DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 1.0; // based on the value in pick_objects
    marker.pose.position.y = 0.0; 
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color of the marker
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    /*
        PHASE II: Pause for 5 seconds
        PHASE III: Hide the marker
    */
    marker.lifetime = ros::Duration(5); // delete this marker after 5 seconds; in other situations, ros::Duration() means never delete

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok()) return 0;

        ROS_WARN_ONCE("Please create a subscriber to the marker!");
        sleep(1);
    }
    marker_pub.publish(marker);
    ROS_INFO("Published the marker at (1,0, 0.0) !");


    // r.sleep();

    /*
        PHASE IV: Pause for 5 seconds
    */
    ros::Duration(10.0).sleep(); // 5s duration + 5s pause

    ROS_INFO("Finished the time gap of sleeping for 10 seconds !");

    visualization_msgs::Marker marker_drop;

    marker_drop.header.frame_id = "map"; // modified from "my_frame"
    marker_drop.header.stamp = ros::Time::now();

    marker_drop.ns = "add_markers"; // modified from "basic_shape"
    marker_drop.id = 0;

    marker_drop.type = shape;
    marker_drop.action = visualization_msgs::Marker::ADD;


    /*
        PHASE V: Publish the marker at the drop off zone
    */

    marker_drop.pose.position.x = 3.0; // based on the value in pick_objects
    marker_drop.pose.position.y = 0.0; 
    marker_drop.pose.position.z = 0.0;
    marker_drop.pose.orientation.x = 0.0;
    marker_drop.pose.orientation.y = 0.0;
    marker_drop.pose.orientation.z = 0.0;
    marker_drop.pose.orientation.w = 1.0;

    // Set the scale of the marker
    marker_drop.scale.x = 1.0;
    marker_drop.scale.y = 1.0;
    marker_drop.scale.z = 1.0;

    // Set the color of the marker
    marker_drop.color.r = 0.0f;
    marker_drop.color.g = 0.0f;
    marker_drop.color.b = 1.0f;
    marker_drop.color.a = 1.0;

    marker_drop.lifetime = ros::Duration(5); // delete the marker_drop after 5 seconds; in other situations, ros::Duration() means never delete

    // Publish the marker_drop
    while (marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok()) return 0;

        ROS_WARN_ONCE("Please create a subscriber to the marker_drop!");
        sleep(1);
    }
    marker_pub.publish(marker_drop);
    ROS_INFO("Published the marker at (3.0, 0.0) !");

    ros::Duration().sleep(); // "Nice try": sleep till the program is killed by the user
                             // However, the program just finishes after the marker_drop dies
    return 0;
    // }
}