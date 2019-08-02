#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "yusen_tf_listener");

    ros::NodeHandle nh;

    ros::service::waitForService("spawn");
    ros::ServiceClient add_turtle = nh.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn srv;
    add_turtle.call(srv);

    ros::Publisher pub_turtle_vel = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
    
    // create a TransformListener object.
    // once the listener is created, 
    // it starts receiving tf transformations over the wire, 
    // and buffers them for up to 10 seconds.
    tf::TransformListener(listener);

    ros::Rate rate(10.0);

    while ( nh.ok() ) {
        tf::StampedTransform transform;

        try {
            // here, the real work is done, we query the listener for a specific transformation in 4 arguments:
            // 1 & 2. we want the transform from frame /turtle1 to frame /turtle2
            // 3. the time at which we want to transform; here ros::Time(0) gets us the latest available transform.
            // 4. the object in which we store the resulting transform.

            /* **********************************************************************************************************
             * within these steps, it can basically work with one small error:
             * [ERROR] [1564756246.979394372]: "turtle2" passed to lookupTransform argument target_frame does not exist. 
             * - Reason: 
             *           This happens because our listener is trying to compute the transform before messages 
             *           about turtle 2 have been received because it takes a little time to spawn in turtlesim 
             *           and start broadcasting a tf frame.
             * - Counter-measure: 
             *           Add waitForTransform() before lookupTransform
             * ********************************************************************************************************** */

            // 5. (adds-on): waiting for transform for some duration and look up transform
            listener.waitForTransform( "/turtle2", "turtle1", ros::Time(0), ros::Duration(10.0) );
            // listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);

            // change the behavior of the listener
            listener.lookupTransform("/turtle2", "/carrot1", ros::Time(0), transform);
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR( "%s", ex.what() );
            ros::Duration(1.0).sleep();
        }

        // here, the transform is used to calculate new linear and angular vel for turtle2,
        // based on the distance and angle form turtle1.
        // the new vel is published in the topic "turtle2/cmd_vel" and the sim will use it to update turtle2's movement.
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 4.0 * atan2( transform.getOrigin().y(), transform.getOrigin().x() );
        vel_msg.linear.x = 0.5 * sqrt( pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2) );
        pub_turtle_vel.publish(vel_msg);

        rate.sleep();
    }

    return 0;
}