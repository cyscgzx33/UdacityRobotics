#include <res/ros.h>
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

    while (ros::ok())
    {
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

        marker.lifetime = ros::Duration();

        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok()) return 0;

            ROS_WARN_ONCE("Please create a subscriber to the marker!");
            sleep(1);
        }
        marker_pub.publish(marker);

        r.sleep();

    }
}