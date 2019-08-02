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
        transform.setOrigin( tf::Vector3(0.0, 2.0, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
        br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "turtle1", "carrot1") );

        rate.sleep();
    }

    return 0;
}