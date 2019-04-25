#include <ros/ros.h>
#include "add_markers/addMarkers.h"


int main( int argc, char** argv )
{   
    
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);
    addMarkers am(nh);

    return 0;

}