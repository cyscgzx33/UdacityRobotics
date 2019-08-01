#include <ros/ros.h>
#include <tf/transform_broadcaster.h> // implements TransformBroadcaster to make the task of publishing transforms easier
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg) {
    // create a TransformBroadcaster obj that we'll use later to send the transformations over the wire
    static tf::TransformBroadcaster br;

    // create a Transform obj and copy the info from the 2D turtle pose into the 3D tansform
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);

    // set the rotation
    transform.setRotation(q);

    // this is where the real work is done;
    // sending a transform with a TransformBroadcaster requires 4 arguments:
    // 1. pass in the transform itself;
    // 2. give the transform being published a timestamp, which is the current time;
    // 3. pass the name of the parent frame of the link we're creating, in this case, "world";
    // 4. pass the name of the child frame of the link we're creating, in this case, the turtle itself.
    br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name) );
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "yusen_tf_broadcaster");
    
    if (argc != 2) {
        ROS_ERROR("need turtle name as argument"); 
        return -1;
    }

    turtle_name = argv[1];

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(turtle_name+"/pose", 10, &poseCallback);

    ros::spin();
    return 0;
}