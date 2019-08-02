#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "yusen_frame_adding_tf_braodcaster");
    ros::NodeHandle nh;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(10.0);

    while ( nh.ok() ) {
        // create a new transform, from parent tutle1 to the new child carrot1
        // the carrot1 frame is 2 meters offset to the left from the turtle1 frame
    
        // a fixed frame that doesn't change over time in relation to the parent frame
        tf::Transform transform_fixed_frame;
        transform_fixed_frame.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );

        // a moving frame that changes overtime in relation to the parent frame
        // Yusen: in the result, carrot1 will moving around turtle1 in a traj of a circle
        tf::Transform transform_chaging_frame;
        transform_chaging_frame.setOrigin( tf::Vector3( 2.0*sin(ros::Time::now().toSec() ), 2.0*cos(ros::Time::now().toSec() ), 0.0 ) );

        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
        br.sendTransform( tf::StampedTransform(transform_chaging_frame, ros::Time::now(), "turtle1", "carrot1") );

        rate.sleep();
    }

    return 0;
}